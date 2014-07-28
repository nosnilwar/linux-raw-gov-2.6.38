/*
 *  drivers/cpufreq/cpufreq_raw.c
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>

#define POLL_TIME 100000 /* in µs */

struct raw_gov_info_struct {
	unsigned long busy_spus;	/* fixed-point */
	struct cpufreq_policy *policy;
	struct work_struct work;
	unsigned int poll_int;		/* µs */
	int flagDefinicaoPrioWorkqueue; /* verificar se foi atribuida a Prioridade RT para a Workqueue. 1 - Sim / 0 - Nao */

	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};

static DEFINE_PER_CPU(struct raw_gov_info_struct, raw_gov_info);

static DEFINE_MUTEX(raw_mutex);

static struct workqueue_struct	*kraw_wq;

struct task_struct *tarefa_sinalizada;
unsigned long long deadline_tarefa_sinalizada;

struct cpufreq_frequency_table *freq_table;

#define dprintk(msg...) cpufreq_debug_printk(CPUFREQ_DEBUG_GOVERNOR, "raw", msg)

unsigned int get_frequency_table_target(struct cpufreq_policy *policy, unsigned int target_freq)
{
	unsigned int new_freq;
	unsigned int i;

	if (!cpu_online(policy->cpu))
		return -EINVAL;

	//OBS.: as frequencias comecam do MAIOR para o MENOR.
	new_freq = freq_table[0].frequency;
	for (i = 0; (freq_table[i].frequency != CPUFREQ_TABLE_END); i++) {
		unsigned int freq = freq_table[i].frequency;

		if (freq == CPUFREQ_ENTRY_INVALID)
			continue;

		if ((freq < policy->min) || (freq > policy->max))
			continue;

		if (freq < target_freq) {
			break;
		}
		new_freq = freq;
	}

	printk("DEBUG:RAWLINSON - RAW GOVERNOR - get_frequency_table_target(%u) kHz for cpu %u => NOVA FREQ(%u kHz)\n", target_freq, policy->cpu, new_freq);

	return new_freq;
}

/**
 * SINALIZA PARA O RAW MONITOR QUE O TAREFA PREEMPTADA VOLTOU A EXECUCAO.
 */
static int wake_up_kworker(struct cpufreq_policy *policy, struct task_struct *task, unsigned long long deadline_ns)
{
	//unsigned int valid_freq = 0;
	int ret = -EINVAL;
	struct raw_gov_info_struct *info;

	info = &per_cpu(raw_gov_info, policy->cpu);

	mutex_lock(&raw_mutex);

	tarefa_sinalizada = task;
	printk("DEBUG:RAWLINSON - RAW GOVERNOR - wake_up_kworker PID (%d) -> Deadline(%llu)\n", tarefa_sinalizada->pid, deadline_ns);
	queue_work_on(info->policy->cpu, kraw_wq, &info->work);

	mutex_unlock(&raw_mutex);
	return ret;
}

/**
 * Sets the CPU frequency to freq.
 */
static int set_frequency(struct cpufreq_policy *policy, struct task_struct *task, unsigned int freq)
{
	unsigned int valid_freq = 0;
	int ret = -EINVAL;

	mutex_lock(&raw_mutex);

	// Se alguma frequencia foi definida... então o monitor não precisa mais verificar a tarefa que foi sinalizada... \o/
	if(task && task->pid > 0)
	{
		task->flagReturnPreemption = 0;
	}

	/*
	 * We're safe from concurrent calls to ->target() here
	 * as we hold the raw_mutex lock. If we were calling
	 * cpufreq_driver_target, a deadlock situation might occur:
	 * A: cpufreq_set (lock raw_mutex) ->
	 *      cpufreq_driver_target(lock policy->lock)
	 * B: cpufreq_set_policy(lock policy->lock) ->
	 *      __cpufreq_governor ->
	 *         cpufreq_governor_raw (lock raw_mutex)
	 */
	valid_freq = get_frequency_table_target(policy, freq);
	if(valid_freq != policy->cur)
		ret = __cpufreq_driver_target(policy, valid_freq, CPUFREQ_RELATION_H);

	//Atualizando a frequencia da tarefa para uma frequencia valida.
	task->cpu_frequency = policy->cur; // (KHz)

	printk("DEBUG:RAWLINSON - RAW GOVERNOR - set_frequency(%u) for cpu %u - %u KHz - GOV(%s) -> PID (%d)\n", freq, policy->cpu, policy->cur, policy->governor->name, task->pid);

	mutex_unlock(&raw_mutex);
	return ret;
}

/**
 * Sets the CPU frequency to freq.
 */
static int cpufreq_raw_set(struct cpufreq_policy *policy, unsigned int freq)
{
	unsigned int valid_freq = 0;
	int ret = -EINVAL;

	mutex_lock(&raw_mutex);

	valid_freq = get_frequency_table_target(policy, freq);
	if(valid_freq != policy->cur)
		ret = __cpufreq_driver_target(policy, valid_freq, CPUFREQ_RELATION_H);

	printk("DEBUG:RAWLINSON - cpufreq_raw_set(%u) for cpu %u, freq %u kHz\n", freq, policy->cpu, policy->cur);

	mutex_unlock(&raw_mutex);
	return ret;
}

static int calc_freq(struct raw_gov_info_struct *info)
{
	return 800000;
}

static void raw_gov_work(struct work_struct *work)
{
	static struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };
	struct task_struct *task_cur;

	struct raw_gov_info_struct *info;
//	int delay;
	unsigned long target_freq;

	info = container_of(work, struct raw_gov_info_struct, work);

	//*********** BEGIN: Definindo a prioridade do RAW monitor...
	if(!info->flagDefinicaoPrioWorkqueue)
	{
		task_cur = get_current_task(info->policy->cpu);
		sched_setscheduler(task_cur, SCHED_FIFO, &param);
		printk("DEBUG:RAWLINSON - raw_gov_init_work - Passou pela definição de prioridades da Kraw. PID(%d) CPUID(%d)\n", task_cur->pid, info->policy->cpu);

		info->flagDefinicaoPrioWorkqueue = 1;
	}
	//*********** END: Definindo a prioridade do RAW monitor...

	mutex_lock(&info->timer_mutex);

	if(tarefa_sinalizada && tarefa_sinalizada->pid > 0)
	{
		tarefa_sinalizada->flagReturnPreemption = 0;
	}

	target_freq = calc_freq(info);
//	if(target_freq != info->policy->cur)
//		__cpufreq_driver_target(info->policy, target_freq, CPUFREQ_RELATION_H);
	printk("DEBUG:RAWLINSON - raw_gov_work(%lu) for cpu %u, freq %u kHz\n", target_freq, info->policy->cpu, info->policy->cur);
//	delay = usecs_to_jiffies(info->poll_int);
//	queue_delayed_work_on(info->policy->cpu, kraw_wq, &info->work, delay);

	mutex_unlock(&info->timer_mutex);
}

static void raw_gov_init_work(struct raw_gov_info_struct *info)
{
//	int delay = usecs_to_jiffies(info->poll_int);
//	INIT_DELAYED_WORK_DEFERRABLE(&info->work, raw_gov_work);
//	queue_delayed_work_on(info->policy->cpu, kraw_wq, &info->work, delay);

	INIT_WORK(&info->work, raw_gov_work);
	queue_work_on(info->policy->cpu, kraw_wq, &info->work);
}

static void raw_gov_cancel_work(struct raw_gov_info_struct *info)
{
//	cancel_delayed_work_sync(&info->work);
	cancel_work_sync(&info->work);
}

static int cpufreq_governor_raw(struct cpufreq_policy *policy, unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct raw_gov_info_struct *info, *affected_info;
	int i;
	int rc = 0;

	freq_table = cpufreq_frequency_get_table(cpu);
	info = &per_cpu(raw_gov_info, cpu);

	switch (event) {
		case CPUFREQ_GOV_START:
			if (!cpu_online(cpu))
				return -EINVAL;

			/* initialize raw_gov_info for all affected cpus */
			for_each_cpu(i, policy->cpus) {
				affected_info = &per_cpu(raw_gov_info, i);
				affected_info->policy = policy;
			}

			BUG_ON(!policy->cur);

			info->poll_int = POLL_TIME;
			info->flagDefinicaoPrioWorkqueue = 0;

			/* setup timer */
			mutex_init(&info->timer_mutex);
			raw_gov_init_work(info);
		break;

		case CPUFREQ_GOV_STOP:
			/* cancel timer */
			raw_gov_cancel_work(info);
			mutex_destroy(&info->timer_mutex);

			/* clean raw_gov_info for all affected cpus */
			for_each_cpu (i, policy->cpus) {
				info = &per_cpu(raw_gov_info, i);
				info->policy = NULL;
			}
		break;

		case CPUFREQ_GOV_LIMITS:
			mutex_lock(&raw_mutex);
			if (policy->max < info->policy->cur)
				__cpufreq_driver_target(info->policy, policy->max, CPUFREQ_RELATION_H);
			else if (policy->min > info->policy->cur)
				__cpufreq_driver_target(info->policy, policy->min, CPUFREQ_RELATION_L);
			mutex_unlock(&raw_mutex);
		break;
	}
	return rc;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_RAW
static
#endif
struct cpufreq_governor cpufreq_gov_raw = {
       .name						= CPUFREQ_CONST_RAW_GOVERNOR_NAME, // Valor => "raw"
       .governor               		= cpufreq_governor_raw,
	   .store_setspeed		   		= cpufreq_raw_set,
	   .set_frequency 		   		= set_frequency,
	   .wake_up_kworker 		   	= wake_up_kworker,
       .owner                  		= THIS_MODULE,
};

static int __init cpufreq_gov_raw_init(void)
{
	int err;

	tarefa_sinalizada = NULL;
	deadline_tarefa_sinalizada = 0;

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
