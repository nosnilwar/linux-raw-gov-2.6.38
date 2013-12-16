/*   -*- linux-c -*-
 *   arch/x86/include/asm/ipipe_32.h
 *
 *   Copyright (C) 2002-2005 Philippe Gerum.
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

#ifndef __X86_IPIPE_32_H
#define __X86_IPIPE_32_H

#include <linux/cpumask.h>
#include <linux/list.h>
#include <linux/threads.h>
#include <linux/ipipe_percpu.h>
#include <asm/ptrace.h>

#define ipipe_read_tsc(t)  __asm__ __volatile__("rdtsc" : "=A" (t))

#define ipipe_tsc2ns(t) \
({ \
	unsigned long long delta = (t)*1000; \
	do_div(delta, cpu_khz/1000+1); \
	(unsigned long)delta; \
})

#define ipipe_tsc2us(t) \
({ \
    unsigned long long delta = (t); \
    do_div(delta, cpu_khz/1000+1); \
    (unsigned long)delta; \
})

/* Private interface -- Internal use only */

extern unsigned int cpu_khz;
extern int __ipipe_hrtimer_irq;
#define __ipipe_cpu_freq	({ unsigned long long __freq = 1000ULL * cpu_khz; __freq; })
#define __ipipe_hrtimer_freq	({ unsigned long long __freq = cpu_has_apic?__ipipe_cpu_freq:PIT_TICK_RATE; __freq; })
#define __ipipe_hrclock_freq	({ unsigned long long __freq = cpu_has_tsc?__ipipe_cpu_freq:PIT_TICK_RATE; __freq; })

int __ipipe_handle_irq(struct pt_regs *regs);

static inline unsigned long __ipipe_ffnz(unsigned long ul)
{
      __asm__("bsrl %1, %0":"=r"(ul)
      :	"r"(ul));
	return ul;
}

struct irq_desc;

void __ipipe_ack_edge_irq(unsigned irq, struct irq_desc *desc);

void __ipipe_end_edge_irq(unsigned irq, struct irq_desc *desc);

static inline void __do_root_xirq(ipipe_irq_handler_t handler,
				  unsigned int irq)
{
	struct pt_regs *regs = &__raw_get_cpu_var(__ipipe_tick_regs);

	regs->orig_ax = ~__ipipe_get_irq_vector(irq);

	__asm__ __volatile__("pushfl\n\t"
			     "orl %[x86if],(%%esp)\n\t"
			     "pushl %%cs\n\t"
			     "pushl $__xirq_end\n\t"
			     "pushl %%eax\n\t"
			     "pushl %%gs\n\t"
			     "pushl %%fs\n\t"
			     "pushl %%es\n\t"
			     "pushl %%ds\n\t"
			     "pushl %%eax\n\t"
			     "pushl %%ebp\n\t"
			     "pushl %%edi\n\t"
			     "pushl %%esi\n\t"
			     "pushl %%edx\n\t"
			     "pushl %%ecx\n\t"
			     "pushl %%ebx\n\t"
			     "movl  %2,%%eax\n\t"
			     "call *%1\n\t"
			     "jmp ret_from_intr\n\t"
			     "__xirq_end: cli\n"
			     : /* no output */
			     : "a" (~irq), "r" (handler), "rm" (regs),
			       [x86if] "i" (X86_EFLAGS_IF));
}

#define __ipipe_do_root_xirq(ipd, irq)			\
	__do_root_xirq((ipd)->irqs[irq].handler, irq)

#endif	/* !__X86_IPIPE_32_H */
