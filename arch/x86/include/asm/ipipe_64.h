/*   -*- linux-c -*-
 *   arch/x86/include/asm/ipipe_64.h
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

#ifndef __X86_IPIPE_64_H
#define __X86_IPIPE_64_H

#include <asm/ptrace.h>
#include <asm/irq.h>
#include <asm/processor.h>
#include <linux/cpumask.h>
#include <linux/list.h>
#include <linux/ipipe_percpu.h>
#ifdef CONFIG_SMP
#include <asm/mpspec.h>
#include <linux/thread_info.h>
#endif

#define ipipe_read_tsc(t)  do {		\
	unsigned int __a,__d;			\
	asm volatile("rdtsc" : "=a" (__a), "=d" (__d)); \
	(t) = ((unsigned long)__a) | (((unsigned long)__d)<<32); \
} while(0)

extern unsigned int cpu_khz;
extern int __ipipe_hrtimer_irq;
#define __ipipe_cpu_freq	({ unsigned long long __freq = (1000ULL * cpu_khz); __freq; })
#define __ipipe_hrtimer_freq	__ipipe_cpu_freq
#define __ipipe_hrclock_freq	__ipipe_cpu_freq

#define ipipe_tsc2ns(t)	(((t) * 1000UL) / (__ipipe_cpu_freq / 1000000UL))
#define ipipe_tsc2us(t)	((t) / (__ipipe_cpu_freq / 1000000UL))

/* Private interface -- Internal use only */

int __ipipe_handle_irq(struct pt_regs *regs);

static inline unsigned long __ipipe_ffnz(unsigned long ul)
{
      __asm__("bsrq %1, %0":"=r"(ul)
	      :	"rm"(ul));
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

	__asm__ __volatile__("movq  %%rsp, %%rax\n\t"
			     "pushq $0\n\t"
			     "pushq %%rax\n\t"
			     "pushfq\n\t"
			     "orq %[x86if],(%%rsp)\n\t"
			     "pushq %[kernel_cs]\n\t"
			     "pushq $__xirq_end\n\t"
			     "pushq %[vector]\n\t"
			     "subq  $9*8,%%rsp\n\t"
			     "movq  %%rdi,8*8(%%rsp)\n\t"
			     "movq  %%rsi,7*8(%%rsp)\n\t"
			     "movq  %%rdx,6*8(%%rsp)\n\t"
			     "movq  %%rcx,5*8(%%rsp)\n\t"
			     "movq  %%rax,4*8(%%rsp)\n\t"
			     "movq  %%r8,3*8(%%rsp)\n\t"
			     "movq  %%r9,2*8(%%rsp)\n\t"
			     "movq  %%r10,1*8(%%rsp)\n\t"
			     "movq  %%r11,(%%rsp)\n\t"
			     "call  *%[handler]\n\t"
			     "cli\n\t"
			     "jmp exit_intr\n\t"
			     "__xirq_end: cli\n"
			     : /* no output */
			     : [kernel_cs] "i" (__KERNEL_CS),
			       [vector] "rm" (regs->orig_ax),
			       [handler] "r" (handler), "D" (regs),
			       [x86if] "i" (X86_EFLAGS_IF)
			     : "rax");
}

#define __ipipe_do_root_xirq(ipd, irq)			\
	__do_root_xirq((ipd)->irqs[irq].handler, irq)

#ifdef CONFIG_PREEMPT
#define __ipipe_check_root_resched()			\
	(preempt_count() == 0 && need_resched() &&	\
	 per_cpu(irq_count, ipipe_processor_id()) < 0)
#else
#define __ipipe_check_root_resched()	0
#endif

#endif	/* !__X86_IPIPE_64_H */
