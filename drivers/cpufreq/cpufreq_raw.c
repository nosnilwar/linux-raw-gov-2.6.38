/*
 *  drivers/cpufreq/cpufreq_raw.c
 */

#include <linux/kernel.h>
#include <linux/module.h>
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

/**
 * A few values needed by the raw governor
 */
static DEFINE_PER_CPU(unsigned int, cpu_max_freq);
static DEFINE_PER_CPU(unsigned int, cpu_min_freq);
static DEFINE_PER_CPU(unsigned int, cpu_cur_freq); /* current CPU freq */
static DEFINE_PER_CPU(unsigned int, cpu_set_freq); /* CPU freq desired by raw */
static DEFINE_PER_CPU(unsigned int, cpu_is_managed);

static DEFINE_MUTEX(raw_mutex);

static int cpus_using_raw_governor;

struct cpufreq_frequency_table *freq_table;

#define dprintk(msg...) \
	cpufreq_debug_printk(CPUFREQ_DEBUG_GOVERNOR, "raw", msg)

/* keep track of frequency transitions */
static int raw_cpufreq_notifier(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;

	if (!per_cpu(cpu_is_managed, freq->cpu))
		return 0;

	dprintk("DEBUG:RAWLINSON - saving cpu_cur_freq of cpu %u to be %u kHz\n", freq->cpu, freq->new);
	per_cpu(cpu_cur_freq, freq->cpu) = freq->new;

	return 0;
}

static struct notifier_block raw_cpufreq_notifier_block = {
	.notifier_call  = raw_cpufreq_notifier
};

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
 * Sets the CPU frequency to freq.
 */
static int set_frequency(struct cpufreq_policy *policy, struct task_struct *task, unsigned int freq)
{
	unsigned int valid_freq = 0;
	int ret = -EINVAL;

	mutex_lock(&raw_mutex);
	if (!per_cpu(cpu_is_managed, policy->cpu))
		goto err;

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
	ret = __cpufreq_driver_target(policy, valid_freq, CPUFREQ_RELATION_H);

	//Atualizando a frequencia da tarefa para uma frequencia valida.
	task->cpu_frequency = policy->cur; // (KHz)

	printk("DEBUG:RAWLINSON - RAW GOVERNOR - set_frequency(%u) for cpu %u - %u KHz - GOV(%s) -> PID (%d)\n", freq, policy->cpu, policy->cur, policy->governor->name, task->pid);

err:
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
	if (!per_cpu(cpu_is_managed, policy->cpu))
		goto err;

	valid_freq = get_frequency_table_target(policy, freq);
	ret = __cpufreq_driver_target(policy, valid_freq, CPUFREQ_RELATION_H);

	printk("DEBUG:RAWLINSON - cpufreq_raw_set(%u) for cpu %u, freq %u kHz\n", freq, policy->cpu, policy->cur);

err:
	mutex_unlock(&raw_mutex);
	return ret;
}

static int cpufreq_governor_raw(struct cpufreq_policy *policy, unsigned int event)
{
	unsigned int cpu = policy->cpu;
	int rc = 0;

	freq_table = cpufreq_frequency_get_table(cpu);

	switch (event) {
		case CPUFREQ_GOV_START:
			if (!cpu_online(cpu))
				return -EINVAL;

			BUG_ON(!policy->cur);
			mutex_lock(&raw_mutex);

			if (cpus_using_raw_governor == 0) {
				cpufreq_register_notifier(&raw_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
			}
			cpus_using_raw_governor++;

			per_cpu(cpu_is_managed, cpu) = 1;
			per_cpu(cpu_min_freq, cpu) = policy->min;
			per_cpu(cpu_max_freq, cpu) = policy->max;
			per_cpu(cpu_cur_freq, cpu) = policy->cur;
			per_cpu(cpu_set_freq, cpu) = policy->cur;
			dprintk("DEBUG:RAWLINSON - managing cpu %u started "
				"(%u - %u kHz, currently %u kHz)\n",
					cpu,
					per_cpu(cpu_min_freq, cpu),
					per_cpu(cpu_max_freq, cpu),
					per_cpu(cpu_cur_freq, cpu));

			mutex_unlock(&raw_mutex);
		break;

		case CPUFREQ_GOV_STOP:
			mutex_lock(&raw_mutex);
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
			dprintk("DEBUG:RAWLINSON - managing cpu %u stopped\n", cpu);
			mutex_unlock(&raw_mutex);
		break;

		case CPUFREQ_GOV_LIMITS:
			mutex_lock(&raw_mutex);
			dprintk("DEBUG:RAWLINSON - limit event for cpu %u: %u - %u kHz, "
				"currently %u kHz, last set to %u kHz\n",
				cpu, policy->min, policy->max,
				per_cpu(cpu_cur_freq, cpu),
				per_cpu(cpu_set_freq, cpu));

			if (policy->max < per_cpu(cpu_set_freq, cpu)) {
				__cpufreq_driver_target(policy, policy->max, CPUFREQ_RELATION_H);
			} else if (policy->min > per_cpu(cpu_set_freq, cpu)) {
				__cpufreq_driver_target(policy, policy->min, CPUFREQ_RELATION_L);
			} else {
				__cpufreq_driver_target(policy, per_cpu(cpu_set_freq, cpu), CPUFREQ_RELATION_L);
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
       .name						= CPUFREQ_CONST_RAW_GOVERNOR_NAME, // Valor => "raw"
       .governor               		= cpufreq_governor_raw,
	   .store_setspeed		   		= cpufreq_raw_set,
	   .set_frequency 		   		= set_frequency,
       .owner                  		= THIS_MODULE,
};

static int __init cpufreq_gov_raw_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_raw);
}


static void __exit cpufreq_gov_raw_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_raw);
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
