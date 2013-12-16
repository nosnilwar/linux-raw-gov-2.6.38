/*   -*- linux-c -*-
 *   arch/x86/include/asm/ipipe.h
 *
 *   Copyright (C) 2007 Philippe Gerum.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __X86_IPIPE_H
#define __X86_IPIPE_H

#ifdef CONFIG_IPIPE

#ifndef IPIPE_ARCH_STRING
#define IPIPE_ARCH_STRING	"2.11-02"
#define IPIPE_MAJOR_NUMBER	2
#define IPIPE_MINOR_NUMBER	11
#define IPIPE_PATCH_NUMBER	2
#endif

DECLARE_PER_CPU(struct pt_regs, __ipipe_tick_regs);

DECLARE_PER_CPU(unsigned long, __ipipe_cr2);

static inline unsigned __ipipe_get_irq_vector(int irq)
{
#ifdef CONFIG_X86_IO_APIC
	unsigned __ipipe_get_ioapic_irq_vector(int irq);
	return __ipipe_get_ioapic_irq_vector(irq);
#elif defined(CONFIG_X86_LOCAL_APIC)
	return irq >= IPIPE_FIRST_APIC_IRQ && irq < IPIPE_NR_XIRQS ?
		ipipe_apic_irq_vector(irq) : irq + IRQ0_VECTOR;
#else
	return irq + IRQ0_VECTOR;
#endif
}

#ifdef CONFIG_X86_32
# include "ipipe_32.h"
#else
# include "ipipe_64.h"
#endif

/*
 * The logical processor id and the current Linux task are read from the PDA,
 * so this is always safe, regardless of the underlying stack.
 */
#define ipipe_processor_id()	raw_smp_processor_id()
#define ipipe_safe_current()	current

#define prepare_arch_switch(next)		\
do {						\
	ipipe_schedule_notify(current, next);	\
	local_irq_disable_hw();			\
} while(0)

#define task_hijacked(p)						\
	({ int x = __ipipe_root_domain_p;				\
	if (x) local_irq_enable_hw(); !x; })

struct ipipe_domain;

struct ipipe_sysinfo {
	int sys_nr_cpus;	/* Number of CPUs on board */
	int sys_hrtimer_irq;	/* hrtimer device IRQ */
	u64 sys_hrtimer_freq;	/* hrtimer device frequency */
	u64 sys_hrclock_freq;	/* hrclock device frequency */
	u64 sys_cpu_freq;	/* CPU frequency (Hz) */
};

/* Private interface -- Internal use only */

#define __ipipe_check_platform()	do { } while(0)
#define __ipipe_init_platform()		do { } while(0)
#define __ipipe_enable_irq(irq)		irq_to_desc(irq)->chip->enable(irq)
#define __ipipe_disable_irq(irq)	irq_to_desc(irq)->chip->disable(irq)
#define __ipipe_tick_irq		__ipipe_hrtimer_irq /* compat */

#ifdef CONFIG_SMP
void __ipipe_hook_critical_ipi(struct ipipe_domain *ipd);
#else
#define __ipipe_hook_critical_ipi(ipd) do { } while(0)
#endif

#define __ipipe_disable_irqdesc(ipd, irq)	do { } while(0)

void __ipipe_enable_irqdesc(struct ipipe_domain *ipd, unsigned irq);

void __ipipe_enable_pipeline(void);

void __ipipe_do_critical_sync(unsigned irq, void *cookie);

void __ipipe_serial_debug(const char *fmt, ...);

#ifdef CONFIG_X86_LOCAL_APIC
#define ipipe_update_tick_evtdev(evtdev)				\
	do {								\
		if (strcmp((evtdev)->name, "lapic") == 0)		\
			__ipipe_hrtimer_irq =				\
				ipipe_apic_vector_irq(LOCAL_TIMER_VECTOR); \
		else							\
			__ipipe_hrtimer_irq = 0;			\
	} while (0)
#else
#define ipipe_update_tick_evtdev(evtdev)				\
	__ipipe_hrtimer_irq = 0
#endif

int __ipipe_check_lapic(void);

int __ipipe_check_tickdev(const char *devname);

#define __ipipe_syscall_watched_p(p, sc)	\
	(ipipe_notifier_enabled_p(p) || (unsigned long)sc >= NR_syscalls)

#define __ipipe_root_tick_p(regs)	((regs)->flags & X86_EFLAGS_IF)

#else /* !CONFIG_IPIPE */

#define ipipe_update_tick_evtdev(evtdev)	do { } while (0)
#define task_hijacked(p)			0

#endif /* CONFIG_IPIPE */

#if defined(CONFIG_SMP) && defined(CONFIG_IPIPE)
#define __ipipe_move_root_irq(irq)					\
	do {								\
		if (irq < NR_IRQS) {					\
			struct irq_desc *desc = irq_to_desc(irq);	\
			struct irq_chip *chip = desc->irq_data.chip;	\
			if (chip->irq_move)				\
				chip->irq_move(&desc->irq_data);	\
		}							\
	} while (0)
#else /* !(CONFIG_SMP && CONFIG_IPIPE) */
#define __ipipe_move_root_irq(irq)	do { } while (0)
#endif /* !(CONFIG_SMP && CONFIG_IPIPE) */

#if defined(CONFIG_IPIPE) && defined(CONFIG_X86_LOCAL_APIC)
void ipipe_init_vector_irq(int cpu);
#else
static inline void ipipe_init_vector_irq(int cpu) { }
#endif

#endif	/* !__X86_IPIPE_H */
