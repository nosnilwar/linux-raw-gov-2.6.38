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

/** Processador no qual a Kworker ira rodar... sem ser a CPUID do RTAI. **/
#define CPU_KRAW	(CPUID_RTAI + 1)

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)

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
#define MIN_SAMPLING_RATE_RATIO			(1)

static unsigned int min_sampling_rate;

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

/* Sampling types */
enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

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

static unsigned int dbs_enable;	/* number of CPUs using this policy */

/*
 * raw_mutex protects data in dbs_tuners_ins from concurrent changes on
 * different CPUs. It protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(raw_mutex);
static DEFINE_MUTEX(raw_mutex_timer);

static struct workqueue_struct	*kraw_wq;

unsigned long cont_kraw;
TYPE_RT_TIME tick_timer_rtai; // possui o timer do RTAI atualizado...

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int down_differential;
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	unsigned int powersave_bias;
	unsigned int io_is_busy;
} dbs_tuners_ins = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.ignore_nice = 0,
	.powersave_bias = 0,
};

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
							cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
			kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.nice);

	idle_time = cputime64_sub(cur_wall_time, busy_time);
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

	return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

static void raw_powersave_bias_init_cpu(int cpu)
{
	struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	dbs_info->freq_table = cpufreq_frequency_get_table(cpu);
	dbs_info->freq_lo = 0;
}

static void raw_powersave_bias_init(void)
{
	int i;
	for_each_online_cpu(i) {
		raw_powersave_bias_init_cpu(i);
	}
}

/************************** sysfs interface ************************/

static ssize_t show_sampling_rate_max(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	printk_once(KERN_INFO "CPUFREQ: raw sampling_rate_max "
	       "sysfs file is deprecated - used by: %s\n", current->comm);
	return sprintf(buf, "%u\n", -1U);
}

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_max);
define_one_global_ro(sampling_rate_min);

/* cpufreq_raw Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)              \
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(io_is_busy, io_is_busy);
show_one(up_threshold, up_threshold);
show_one(sampling_down_factor, sampling_down_factor);
show_one(ignore_nice_load, ignore_nice);
show_one(powersave_bias, powersave_bias);

/*** delete after deprecation time ***/

#define DEPRECATION_MSG(file_name)					\
	printk_once(KERN_INFO "CPUFREQ: Per core raw sysfs "	\
		    "interface is deprecated - " #file_name "\n");

#define show_one_old(file_name)						\
static ssize_t show_##file_name##_old					\
(struct cpufreq_policy *unused, char *buf)				\
{									\
	printk_once(KERN_INFO "CPUFREQ: Per core raw sysfs "	\
		    "interface is deprecated - " #file_name "\n");	\
	return show_##file_name(NULL, NULL, buf);			\
}
show_one_old(sampling_rate);
show_one_old(up_threshold);
show_one_old(ignore_nice_load);
show_one_old(powersave_bias);
show_one_old(sampling_rate_min);
show_one_old(sampling_rate_max);

cpufreq_freq_attr_ro_old(sampling_rate_min);
cpufreq_freq_attr_ro_old(sampling_rate_max);

/*** delete after deprecation time ***/

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&raw_mutex);
	dbs_tuners_ins.sampling_rate = max(input, min_sampling_rate);
	mutex_unlock(&raw_mutex);

	return count;
}

static ssize_t store_io_is_busy(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&raw_mutex);
	dbs_tuners_ins.io_is_busy = !!input;
	mutex_unlock(&raw_mutex);

	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}

	mutex_lock(&raw_mutex);
	dbs_tuners_ins.up_threshold = input;
	mutex_unlock(&raw_mutex);

	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	mutex_lock(&raw_mutex);
	dbs_tuners_ins.sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->rate_mult = 1;
	}
	mutex_unlock(&raw_mutex);

	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	mutex_lock(&raw_mutex);
	if (input == dbs_tuners_ins.ignore_nice) { /* nothing to do */
		mutex_unlock(&raw_mutex);
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;

	}
	mutex_unlock(&raw_mutex);

	return count;
}

static ssize_t store_powersave_bias(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 1000)
		input = 1000;

	mutex_lock(&raw_mutex);
	dbs_tuners_ins.powersave_bias = input;
	raw_powersave_bias_init();
	mutex_unlock(&raw_mutex);

	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(io_is_busy);
define_one_global_rw(up_threshold);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(powersave_bias);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_max.attr,
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = CPUFREQ_CONST_RAW_GOVERNOR_NAME,
};

/*** delete after deprecation time ***/

#define write_one_old(file_name)					\
static ssize_t store_##file_name##_old					\
(struct cpufreq_policy *unused, const char *buf, size_t count)		\
{									\
       printk_once(KERN_INFO "CPUFREQ: Per core raw sysfs "	\
		   "interface is deprecated - " #file_name "\n");	\
       return store_##file_name(NULL, NULL, buf, count);		\
}
write_one_old(sampling_rate);
write_one_old(up_threshold);
write_one_old(ignore_nice_load);
write_one_old(powersave_bias);

cpufreq_freq_attr_rw_old(sampling_rate);
cpufreq_freq_attr_rw_old(up_threshold);
cpufreq_freq_attr_rw_old(ignore_nice_load);
cpufreq_freq_attr_rw_old(powersave_bias);

static struct attribute *dbs_attributes_old[] = {
       &sampling_rate_max_old.attr,
       &sampling_rate_min_old.attr,
       &sampling_rate_old.attr,
       &up_threshold_old.attr,
       &ignore_nice_load_old.attr,
       &powersave_bias_old.attr,
       NULL
};

static struct attribute_group dbs_attr_group_old = {
       .attrs = dbs_attributes_old,
       .name = "raw",
};

/*** delete after deprecation time ***/

/************************** sysfs end ************************/

/**
 * Atualiza os timers de tempo real dos processadores para o melhor gerenciamento do RAW GOVERNOR.
 */
static int update_rt_smp_time_h(TYPE_RT_TIME tick_time)
{
	// Atribui a tarefa o timer do processador no instante que ocorreu a preempcao.
	mutex_lock(&raw_mutex_timer);
	tick_timer_rtai = tick_time;
	mutex_unlock(&raw_mutex_timer);
	return 1;
}

/**
 * Sets the CPU frequency to freq.
 */
static int set_frequency(struct cpufreq_policy *policy, struct task_struct *task, unsigned int freq)
{
	int ret = -EINVAL;

	mutex_lock(&raw_mutex);

	// Se alguma frequencia foi definida... então o monitor não precisa mais verificar a tarefa que foi sinalizada... \o/
	if(task && task->pid > 0)
	{
		task->flagReturnPreemption = 0;
		task->flagGovChangeFrequency = 0;
	}

	__cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_H);

	mutex_unlock(&raw_mutex);

	printk("DEBUG:RAWLINSON - RAW GOVERNOR - set_frequency(%u) for cpu %u - %u - %s -> flagReturnPreemption(%d) -> PID (%d)\n", freq, policy->cpu, policy->cur, policy->governor->name, task->flagReturnPreemption, task->pid);
	return ret;
}

/**
 * Sets the CPU frequency to freq.
 */
static int cpufreq_raw_set(struct cpufreq_policy *policy, unsigned int freq)
{
	int ret = -EINVAL;

	mutex_lock(&raw_mutex);
	__cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_H);
	mutex_unlock(&raw_mutex);

	printk("DEBUG:RAWLINSON - cpufreq_raw_set(%u) for cpu %u, freq %u kHz\n", freq, policy->cpu, policy->cur);

	return ret;
}

// FUNCAO COPIADA DO RTAI_HAL.H
static inline long raw_imuldiv (long i, long mult, long div)
{
    /* Returns (int)i = (int)i*(int)(mult)/(int)div. */
	i = (i * mult) / div;
    return i;
}

static inline long long llimd(long long ll, long mult, long div)
{
	return raw_imuldiv(ll, mult, div);
}

// FUNCAO COPIADA DO RTAI_HAL.H
static TYPE_RT_TIME count2nano(TYPE_RT_TIME counts, unsigned long timer_freq)
{
	int sign;

	if (counts >= 0) {
		sign = 1;
	} else {
		sign = 0;
		counts = - counts;
	}

	//TODO: O TIMER_FREQ estah prevendo apenas o modo rt_set_periodic_mode e nao o rt_set_oneshot_mode, infelizmente. :'(
	counts = llimd(counts, 1000000000, timer_freq);
	return sign ? counts : - counts;
}

static void do_dbs_timer(struct work_struct *work)
{
	unsigned long long ciclosClockRestantes = 0; // Conterah a quantidade de ciclos remanescente da tarefa que retornou de preempcao.
	TYPE_RT_TIME tempoRestanteProcessamento = 0; // Conterah o tempo restante que a tarefa tem para concluir sua execucao.
	unsigned int cpu_frequency_target = 0; // Conterah a frequencia que o processador terah que assumir para que a tarefa conclua seu processamento dentro do seu deadline.
	unsigned int cpu_frequency_table = 0; // Conterah a frequencia mais proxima de "cpu_frequency_target"... suportada pelo processador.
	unsigned int index_freq_table = 0;
	TYPE_RT_TIME tick_timer_atual; // possui o timer do RTAI atualizado...

	struct task_struct *task_cur;

	struct cpu_dbs_info_s *dbs_info = container_of(work, struct cpu_dbs_info_s, work.work);
	struct cpufreq_policy *policy = dbs_info->cur_policy;

	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	mutex_lock(&dbs_info->timer_mutex);

	cont_kraw = cont_kraw + 1;

	//printk("[RAW MONITOR] (%lu) - DELAY(%d)\n", cont_kraw, delay);

	// Verifica se a tarefa em execucao retornou de uma preempcao...
	task_cur = get_current_task(CPUID_RTAI);

	/*
	 * Verificando se a tarefa que estah em executacao agora... estah retornando de uma preempcao...
	 * caso positivo... o RAW GOVERNOR avalia a necessecidade de se aumentar ou diminuir a frequencia do processador.
	 */
	if(task_cur->flagReturnPreemption && task_cur->state == TASK_RUNNING && task_cur->period > 0 && task_cur->timer_freq > 0
	   && task_cur->state_task_period == TASK_PERIOD_RUNNING && task_cur->rwcec > 0) // Significa que uma tarefa de tempo real do RTAI estah retornando de preempcao.
	{
		mutex_lock(&raw_mutex_timer);
		tick_timer_atual = tick_timer_rtai;
		mutex_unlock(&raw_mutex_timer);

		printk("[RAW MONITOR] (%lu) - CPU(%u) %u MHz -> RWCEC(%lu) -> PID(%d) -> STATE(%lu) -> FRP(%d) -> STP(%d)\n", cont_kraw, task_cpu(task_cur), policy->cur, task_cur->rwcec, task_cur->pid, task_cur->state, task_cur->flagReturnPreemption, task_cur->state_task_period);
		printk("[RAW MONITOR] (%lu) - TICK(%llu) PERIOD(%llu) RT(%llu) PRT(%llu)\n", cont_kraw, tick_timer_atual, task_cur->period, task_cur->resume_time, task_cur->periodic_resume_time);
		//printk("[RAW MONITOR] (%lu) - Y(%llu) Q(%d) R(%d)\n", cont_kraw, task_cur->yield_time, task_cur->rr_quantum, task_cur->rr_remaining);

		mutex_lock(&raw_mutex);

		task_cur->flagReturnPreemption = 0; // O Governor desmarca a flag de preempcao, pois ela ja foi verificada.

		/*****************************************************************
		* Fazendo o calculo da frequencia que devera ser aplicada no processador
		* para que a tarefa conclua seu processamento dentro do seu deadline.
		****************************************************************/
		ciclosClockRestantes = task_cur->rwcec; // já foi verificado se eh maior que zero.
		tempoRestanteProcessamento = ((task_cur->periodic_resume_time + task_cur->period) - (tick_timer_atual + CPUFREQ_UPDATE_RATE_TIMER)); // Foi acrescentado o "CPUFREQ_UPDATE_RATE_TIMER", pois o tick pode estar atrasado (ou seja, uma margem de erro).
		tempoRestanteProcessamento = count2nano(tempoRestanteProcessamento, task_cur->timer_freq) / 10^9; // Transformando de count -> nano (/10^9) para -> segundos.
		if(tempoRestanteProcessamento <= 0)
		{
			tempoRestanteProcessamento = 1;
		}

		printk("[RAW MONITOR SET_FREQ] (%lu) - cpu_frequency_target = ciclosClockRestantes(%lld) / tempoRestanteProcessamento(%lld) -> TIMER_FREQ(%ld)\n", cont_kraw, ciclosClockRestantes, tempoRestanteProcessamento, task_cur->timer_freq);
		cpu_frequency_target = ciclosClockRestantes / tempoRestanteProcessamento; // Unidade: Ciclos/segundo (a conversao para segundos foi feita acima 10^9)
		cpu_frequency_target = cpu_frequency_target / 1000; // Unidade: Khz (convertendo para de Hz para KHz)

		// Buscando na tabela de frequencias do processador... qual o valor se aproxima da frequencia calculada.
		index_freq_table = 0;
		cpufreq_frequency_table_target(policy, dbs_info->freq_table, cpu_frequency_target, CPUFREQ_RELATION_H, &index_freq_table);
		cpu_frequency_table = dbs_info->freq_table[index_freq_table].frequency;

		printk("[RAW MONITOR SET_FREQ] (%lu) - Frequencia Calculada(%u) - Frequencia da Tabela(%u)\n", cont_kraw, cpu_frequency_target, cpu_frequency_table);
		//******************************************************************/

		if(cpu_frequency_table != policy->cur)
		{
			task_cur->cpu_frequency = cpu_frequency_table; // (KHz) Nova frequencia para a tarefa... visando diminuir o tempo de folga da tarefa...
			task_cur->flagGovChangeFrequency = 1; // O Governor estah alterando a frequencia original da tarefa.

			cpufreq_driver_target(policy, cpu_frequency_table, CPUFREQ_RELATION_H);
			printk("[RAW MONITOR SET_FREQ] (%lu) - do_dbs_timer(%u) for cpu %u, freq %u kHz\n", cont_kraw, cpu_frequency_table, policy->cpu, policy->cur);
		}
		mutex_unlock(&raw_mutex);
	}

	mutex_unlock(&dbs_info->timer_mutex);
	queue_delayed_work_on(CPU_KRAW, kraw_wq, &dbs_info->work, delay);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

//	if (num_online_cpus() > 1)
//		delay -= jiffies % delay;

	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);

	//queue_delayed_work_on(dbs_info->cpu, kraw_wq, &dbs_info->work, delay); // ORIGINAL...
	queue_delayed_work_on(CPU_KRAW, kraw_wq, &dbs_info->work, delay); // JOGA A EXECUCAO PARA OUTRO PROCESSADOR QUE NAO SEJA O RTAI (cpu_kraw).
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->work);
}

/*
 * Not all CPUs want IO time to be accounted as busy; this dependson how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (androidlcom) calis this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
	/*
	 * For Intel, Core 2 (model 15) andl later have an efficient idle.
	 */
	if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
	    boot_cpu_data.x86 == 6 &&
	    boot_cpu_data.x86_model >= 15)
		return 1;
#endif
	return 0;
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&raw_mutex);

		rc = sysfs_create_group(&policy->kobj, &dbs_attr_group_old);
		if (rc) {
			mutex_unlock(&raw_mutex);
			return rc;
		}

		dbs_enable++;
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
						kstat_cpu(j).cpustat.nice;
			}
		}
		this_dbs_info->cpu = cpu;
		this_dbs_info->rate_mult = 1;
		raw_powersave_bias_init_cpu(cpu);
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			unsigned int latency;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&raw_mutex);
				return rc;
			}

			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;
			/* Bring kernel and HW constraints together */
//			min_sampling_rate = max(min_sampling_rate,
//					MIN_LATENCY_MULTIPLIER * latency);
//			dbs_tuners_ins.sampling_rate =
//				max(min_sampling_rate,
//				    latency * LATENCY_MULTIPLIER);
			dbs_tuners_ins.sampling_rate = min_sampling_rate;
			dbs_tuners_ins.io_is_busy = should_io_be_busy();
		}
		mutex_unlock(&raw_mutex);

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_timer_init(this_dbs_info);
		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&raw_mutex);
		sysfs_remove_group(&policy->kobj, &dbs_attr_group_old);
		mutex_destroy(&this_dbs_info->timer_mutex);
		dbs_enable--;
		mutex_unlock(&raw_mutex);
		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
				policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_dbs_info->timer_mutex);
		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_RAW
static
#endif
struct cpufreq_governor cpufreq_gov_raw = {
       .name						= CPUFREQ_CONST_RAW_GOVERNOR_NAME, // Valor => "raw"
       .governor               		= cpufreq_governor_dbs,
	   .store_setspeed		   		= cpufreq_raw_set,
	   .set_frequency 		   		= set_frequency,
	   .update_rt_smp_time_h   		= update_rt_smp_time_h,
       .max_transition_latency 		= TRANSITION_LATENCY_LIMIT,
       .owner                  		= THIS_MODULE,
};

static int __init cpufreq_gov_raw_init(void)
{
	int err;
	cputime64_t wall;
	u64 idle_time;
	int cpu = get_cpu();

	idle_time = get_cpu_idle_time_us(cpu, &wall);
	put_cpu();
//	if (idle_time != -1ULL) {
//		/* Idle micro accounting is supported. Use finer thresholds */
//		dbs_tuners_ins.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
//		dbs_tuners_ins.down_differential =
//					MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
//		/*
//		 * In no_hz/micro accounting case we set the minimum frequency
//		 * not depending on HZ, but fixed (very low). The deferred
//		 * timer might skip some samples if idle/sleeping as needed.
//		*/
//		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
//	} else {
//		/* For correct statistics, we need 10 ticks for each measure */
//		min_sampling_rate =
//			MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
//	}
	min_sampling_rate = MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(2);

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
