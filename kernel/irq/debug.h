/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Debugging printout:
 */

#include <linux/kallsyms.h>

#define ___P(f) if (desc->status_use_accessors & f) printk("%14s set\n", #f)
#define ___PS(f) if (desc->istate & f) printk("%14s set\n", #f)
/* FIXME */
#define ___PD(f) do { } while (0)

static inline void print_irq_desc(unsigned int irq, struct irq_desc *desc)
{
	printk("irq hoge, desc: hoge, depth: hoge, count: hoge, unhandled: hoge\n");
	printk("->handle_irq():  hoge, ");
	printk("->irq_data.chip(): hoge, ");
	printk("->action(): hoge\n");
	if (desc->action) {
		printk("->action->handler(): hoge, ");
	}

	//___P(IRQ_LEVEL);
	//___P(IRQ_PER_CPU);
	//___P(IRQ_NOPROBE);
	//___P(IRQ_NOREQUEST);
	//___P(IRQ_NOTHREAD);
	//___P(IRQ_NOAUTOEN);

	//___PS(IRQS_AUTODETECT);
	//___PS(IRQS_REPLAY);
	//___PS(IRQS_WAITING);
	//___PS(IRQS_PENDING);

	//___PD(IRQS_INPROGRESS);
	//___PD(IRQS_DISABLED);
	//___PD(IRQS_MASKED);
}

#undef ___P
#undef ___PS
#undef ___PD
