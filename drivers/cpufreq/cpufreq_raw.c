
/*
 *  linux/drivers/cpufreq/cpufreq_raw.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2002 - 2004 Dominik Brodowski <linux@brodo.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

//TODO:RAWLINSON - FEITO VARIAS ALTERACOES...

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>

#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>

#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
#define MIN_SAMPLING_RATE_RATIO			(0)
#define MIN_SAMPLING_RATE	MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(1) /* For correct statistics, we need X ticks for each measure */

/**
 * A few values needed by the raw governor
 */
static DEFINE_PER_CPU(unsigned int, cpu_max_freq);
static DEFINE_PER_CPU(unsigned int, cpu_min_freq);
static DEFINE_PER_CPU(unsigned int, cpu_cur_freq); /* current CPU freq */
static DEFINE_PER_CPU(unsigned int, cpu_set_freq); /* CPU freq desired by
							raw */
static DEFINE_PER_CPU(unsigned int, cpu_is_managed);

static DEFINE_MUTEX(raw_mutex);
static int cpus_using_raw_governor;

static struct workqueue_struct	*kraw_wq;

/**
 *
 */
struct task_struct * signaled_task;

#define dprintk(msg...) \
	cpufreq_debug_printk(CPUFREQ_DEBUG_GOVERNOR, "raw", msg)

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_lo;
	unsigned int freq_lo_jiffies;
	unsigned int freq_hi_jiffies;
	unsigned int rate_mult;
	int cpu;
	unsigned int sample_type:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};

static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);

/* Sampling types */
enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

/* keep track of frequency transitions */
static int raw_cpufreq_notifier(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;

	if (!per_cpu(cpu_is_managed, freq->cpu))
		return 0;

	dprintk("saving cpu_cur_freq of cpu %u to be %u kHz\n",
			freq->cpu, freq->new);
	per_cpu(cpu_cur_freq, freq->cpu) = freq->new;

	return 0;
}

static struct notifier_block raw_cpufreq_notifier_block = {
	.notifier_call  = raw_cpufreq_notifier
};

/**
 * Sinaliza para o governor a tarefa que acabou de voltar de preempcao...
 */
static int set_signaled_task(struct cpufreq_policy *policy, struct task_struct *task)
{
	int ret = 1;
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *dbs_info;

	mutex_lock(&raw_mutex);

	dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	printk("DEBUG:RAWLINSON -> set_signaled_task() -> WAKE UP -> PID: %d |%lu|%lu|%d|%d|%d|\n", task->pid, task->tsk_wcec, task->rwcec, task->cpu_frequency, task->cpu_voltage, task->flagReturnPreemption);
	signaled_task = task;

	// Agora vamos sinalizar para a Kraw (Kworker) que uma tarefa voltou de preempcao e vamos cancelar o seu delay...
	// para ela definir uma nova frequencia para o processador continuar executando a tarefa...
	switch (task->flagReturnPreemption) {
		case 2:
			queue_delayed_work_on(dbs_info->cpu, kraw_wq, &dbs_info->work, 0);
			flush_delayed_work(&dbs_info->work);
		break;
		case 3:
			flush_delayed_work_sync(&dbs_info->work);
		break;
		case 4:
			schedule_delayed_work(&dbs_info->work, 0);
			flush_delayed_work(&dbs_info->work);
		break;
		default:
			// nao faz nada se for zero... :-P
		break;
	}

	mutex_unlock(&raw_mutex);
	return ret;
}

/**
 * Sets the CPU frequency to freq.
 */
static int set_frequency(struct cpufreq_policy *policy, struct task_struct *task, unsigned int freq)
{
	int ret = -EINVAL;

	dprintk("cpufreq_raw_set for cpu %u, freq %u kHz\n", policy->cpu, freq);

	printk("DEBUG:RAWLINSON - set_frequency for cpu %u, freq %u kHz\n", policy->cpu, freq);

	mutex_lock(&raw_mutex);
	if (!per_cpu(cpu_is_managed, policy->cpu))
		goto err;

	per_cpu(cpu_set_freq, policy->cpu) = freq;

	if (freq < per_cpu(cpu_min_freq, policy->cpu))
		freq = per_cpu(cpu_min_freq, policy->cpu);
	if (freq > per_cpu(cpu_max_freq, policy->cpu))
		freq = per_cpu(cpu_max_freq, policy->cpu);

	/*
	 * We're safe from concurrent calls to ->target() here
	 * as we hold the raw_mutex lock. If we were calling
	 * cpufreq_driver_target, a deadlock situation might occur:
	 * A: cpufreq_raw_set (lock raw_mutex) ->
	 *      cpufreq_driver_target(lock policy->lock)
	 * B: cpufreq_raw_set_policy(lock policy->lock) ->
	 *      __cpufreq_governor ->
	 *         cpufreq_governor_raw (lock raw_mutex)
	 */
	ret = __cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_H);

 err:
	mutex_unlock(&raw_mutex);
	return ret;
}

/**
 * cpufreq_raw_set - set the CPU frequency
 * @policy: pointer to policy struct where freq is being set
 * @freq: target frequency in kHz
 *
 * Sets the CPU frequency to freq.
 */
static int cpufreq_raw_set(struct cpufreq_policy *policy, unsigned int freq)
{
	int ret = -EINVAL;

	dprintk("cpufreq_raw_set for cpu %u, freq %u kHz\n", policy->cpu, freq);

	printk("DEBUG:RAWLINSON - cpufreq_raw_set for cpu %u, freq %u kHz\n", policy->cpu, freq);

	mutex_lock(&raw_mutex);
	if (!per_cpu(cpu_is_managed, policy->cpu))
		goto err;

	per_cpu(cpu_set_freq, policy->cpu) = freq;

	if (freq < per_cpu(cpu_min_freq, policy->cpu))
		freq = per_cpu(cpu_min_freq, policy->cpu);
	if (freq > per_cpu(cpu_max_freq, policy->cpu))
		freq = per_cpu(cpu_max_freq, policy->cpu);

	/*
	 * We're safe from concurrent calls to ->target() here
	 * as we hold the raw_mutex lock. If we were calling
	 * cpufreq_driver_target, a deadlock situation might occur:
	 * A: cpufreq_raw_set (lock raw_mutex) ->
	 *      cpufreq_driver_target(lock policy->lock)
	 * B: cpufreq_raw_set_policy(lock policy->lock) ->
	 *      __cpufreq_governor ->
	 *         cpufreq_governor_raw (lock raw_mutex)
	 */
	ret = __cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_H);

 err:
	mutex_unlock(&raw_mutex);
	return ret;
}

static ssize_t show_raw_speed(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", per_cpu(cpu_cur_freq, policy->cpu));
}

static void do_dbs_timer(struct work_struct *work)
{
	struct task_struct *g, *task, *task_current;

	struct cpu_dbs_info_s *dbs_info = container_of(work, struct cpu_dbs_info_s, work.work);
	int delay = usecs_to_jiffies(MIN_SAMPLING_RATE);

	mutex_lock(&dbs_info->timer_mutex);

	//TODO:RAWLINSON - APENAS DEBUG... :-P
	// O loop abaixo percorre todas as tarefas de dentro do linux... tarefas pais e filhos...
	task_current = current; // Pega o processo corrente... ou seja o Kworker...

	if(signaled_task)
	{
		printk("DEBUG:RAWLINSON - SIGNALED TASK - do_dbs_timer for cpu(%u) %u MHz-> TASK PID: %d -> Running(%lu) --> (%d)\n", task_cpu(signaled_task), signaled_task->cpu_frequency, signaled_task->pid, signaled_task->state, signaled_task->flagReturnPreemption);
	}

	do_each_thread(g, task)
	{
		//if(task->flagReturnPreemption && task->pid == signaled_task->pid && task->state == TASK_RUNNING)
		if(task->flagReturnPreemption)
		{
			printk("DEBUG:RAWLINSON - RAW GOVERNOR - do_dbs_timer for cpu(%u) %u MHz-> TASK PID: %d -> Running(%lu) --> (%d)\n", task_cpu(task), task->cpu_frequency, task->pid, task->state, task->flagReturnPreemption);

			signaled_task = NULL;
			task->flagReturnPreemption = 0; // O Governor mudou a frequencia do processador visando diminuir o tempo de folga da tarefa...
		}
	} while_each_thread(g, task);

//	printk("DEBUG:RAWLINSON - RAW GOVERNOR - do_dbs_timer for cpu(%u) %u MHz-> TASK PID: %d -> Running(%lu) --> (%d)\n", task_cpu(signaled_task), signaled_task->cpu_frequency, signaled_task->pid, signaled_task->state, signaled_task->flagReturnPreemption);
//	if(signaled_task && signaled_task->flagReturnPreemption && signaled_task->pid > 0 && signaled_task->state == TASK_RUNNING)
//	{
//		printk("DEBUG:RAWLINSON - RAW GOVERNOR - do_dbs_timer for cpu(%u) %u MHz-> TASK PID: %d -> Running(%lu) --> (%d)\n", task_cpu(signaled_task), signaled_task->cpu_frequency, signaled_task->pid, signaled_task->state, signaled_task->flagReturnPreemption);
//
//		signaled_task = NULL;
//		signaled_task->flagReturnPreemption = 0; // O Governor mudou a frequencia do processador visando diminuir o tempo de folga da tarefa...
//	}

	queue_delayed_work_on(dbs_info->cpu, kraw_wq, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(MIN_SAMPLING_RATE);
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;

	// Inicializando a variavel que contera os dados da tarefa que voltou de preempcao no sistema.
	signaled_task = NULL;

	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);

	//schedule_work_on(dbs_info->cpu, &dbs_info->work); // Define a CPU que a kworker ira trabalhar...

	queue_delayed_work_on(dbs_info->cpu, kraw_wq, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->work);
}

static int cpufreq_governor_raw(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	unsigned int cpu_kraw = CPUID_RTAI + 1; //Processador no qual a Kworker ira rodar... sem ser a CPUID do RTAI.

	struct cpu_dbs_info_s *this_dbs_info;
	int rc = 0;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:

		if (!cpu_online(cpu))
			return -EINVAL;

		BUG_ON(!policy->cur);

		mutex_lock(&raw_mutex);

		if (cpus_using_raw_governor == 0) {
			cpufreq_register_notifier(
					&raw_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}
		cpus_using_raw_governor++;

		this_dbs_info->cpu = cpu_kraw;
		this_dbs_info->rate_mult = 1; /* No longer fully busy, reset rate_mult */

		per_cpu(cpu_is_managed, cpu) = 1;
		per_cpu(cpu_min_freq, cpu) = policy->min;
		per_cpu(cpu_max_freq, cpu) = policy->max;
		per_cpu(cpu_cur_freq, cpu) = policy->cur;
		per_cpu(cpu_set_freq, cpu) = policy->cur;
		dprintk("managing cpu %u started "
			"(%u - %u kHz, currently %u kHz)\n",
				cpu,
				per_cpu(cpu_min_freq, cpu),
				per_cpu(cpu_max_freq, cpu),
				per_cpu(cpu_cur_freq, cpu));

		mutex_unlock(&raw_mutex);

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_timer_init(this_dbs_info);
		break;
	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&raw_mutex);

		mutex_destroy(&this_dbs_info->timer_mutex);

		cpus_using_raw_governor--;
		if (cpus_using_raw_governor == 0) {
			cpufreq_unregister_notifier(
					&raw_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}

		per_cpu(cpu_is_managed, cpu) = 0;
		per_cpu(cpu_min_freq, cpu) = 0;
		per_cpu(cpu_max_freq, cpu) = 0;
		per_cpu(cpu_set_freq, cpu) = 0;
		dprintk("managing cpu %u stopped\n", cpu);
		mutex_unlock(&raw_mutex);
		break;
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&raw_mutex);

		dprintk("limit event for cpu %u: %u - %u kHz, "
			"currently %u kHz, last set to %u kHz\n",
			cpu, policy->min, policy->max,
			per_cpu(cpu_cur_freq, cpu),
			per_cpu(cpu_set_freq, cpu));

		if (policy->max < per_cpu(cpu_set_freq, cpu)) {
			__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
		} else if (policy->min > per_cpu(cpu_set_freq, cpu)) {
			__cpufreq_driver_target(policy, policy->min,
						CPUFREQ_RELATION_L);
		} else {
			__cpufreq_driver_target(policy,
						per_cpu(cpu_set_freq, cpu),
						CPUFREQ_RELATION_L);
		}

		per_cpu(cpu_min_freq, cpu) = policy->min;
		per_cpu(cpu_max_freq, cpu) = policy->max;
		per_cpu(cpu_cur_freq, cpu) = policy->cur;
		mutex_unlock(&raw_mutex);
		break;
	}
	return rc;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_RAW
static
#endif
struct cpufreq_governor cpufreq_gov_raw = {
	.name				= "raw",
	.governor			= cpufreq_governor_raw,
	.store_setspeed		= cpufreq_raw_set,
	.set_frequency 		= set_frequency,
	.show_setspeed		= show_raw_speed,
	.set_signaled_task	= set_signaled_task,
	.owner				= THIS_MODULE,
};

static int __init cpufreq_gov_raw_init(void)
{
	int err;

	kraw_wq = create_workqueue("kraw");
	if (!kraw_wq) {
		printk(KERN_ERR "Creation of kraw failed\n");
		return -EFAULT;
	}
	err = cpufreq_register_governor(&cpufreq_gov_raw);
	if (err)
		destroy_workqueue(kraw_wq);

	return err;

}


static void __exit cpufreq_gov_raw_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_raw);
	destroy_workqueue(kraw_wq);
}


MODULE_AUTHOR("Rawlinson <rawlinson.goncalves@gmail.com>");
MODULE_DESCRIPTION("CPUfreq policy governor 'raw'");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_RAW
fs_initcall(cpufreq_gov_raw_init);
#else
module_init(cpufreq_gov_raw_init);
#endif
module_exit(cpufreq_gov_raw_exit);
