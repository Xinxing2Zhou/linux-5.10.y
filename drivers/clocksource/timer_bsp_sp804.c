/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#define pr_fmt(fmt) "bsp_sp804: " fmt

#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/cpu.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/securec.h>

#define TIMER_LOAD      0x00             /* ACVR rw */
#define TIMER_VALUE     0x04             /* ACVR ro */
#define TIMER_CTRL      0x08             /* ACVR rw */
#define TIMER_CTRL_ONESHOT      (1 << 0) /*  CVR */
#define TIMER_CTRL_32BIT        (1 << 1) /*  CVR */
#define TIMER_CTRL_DIV1         (0 << 2) /* ACVR */
#define TIMER_CTRL_DIV16        (1 << 2) /* ACVR */
#define TIMER_CTRL_DIV256       (2 << 2) /* ACVR */
#define TIMER_CTRL_IE           (1 << 5) /*   VR */
#define TIMER_CTRL_PERIODIC     (1 << 6) /* ACVR */
#define TIMER_CTRL_ENABLE       (1 << 7) /* ACVR */

#define TIMER_INTCLR    0x0c             /* ACVR wo */
#define TIMER_RIS       0x10             /*  CVR ro */
#define TIMER_MIS       0x14             /*  CVR ro */
#define TIMER_BGLOAD    0x18             /*  CVR rw */

#define CPU_TASKS_FROZEN        0x0010

struct bsp_sp804_clocksource {
	void __iomem *base;
	struct clocksource clksrc;
};

#define to_clksrc(e) \
	container_of(e, struct bsp_sp804_clocksource, clksrc)

static struct bsp_sp804_clocksource bsp_sp804_clksrc;

static void __iomem *bsp_sp804_sched_clock_base;

#define DEV_NAME_LEN  16

struct bsp_sp804_clockevent_device {
	struct clock_event_device clkevt;
	struct irqaction action;
	void __iomem *base;
	unsigned long rate;
	unsigned long reload;
	char name[DEV_NAME_LEN];
};

#define to_clkevt(e) \
	container_of(e, struct bsp_sp804_clockevent_device, clkevt)

static struct bsp_sp804_clockevent_device __percpu *bsp_sp804_clkevt;

static void bsp_sp804_clocksource_enable(void __iomem *base)
{
	writel(0, base + TIMER_CTRL);
	writel(0xffffffff, base + TIMER_LOAD);
	writel(0xffffffff, base + TIMER_VALUE);
	writel(TIMER_CTRL_32BIT | TIMER_CTRL_ENABLE | TIMER_CTRL_PERIODIC,
		base + TIMER_CTRL);
}

static void bsp_sp804_clocksource_resume(struct clocksource *cs)
{
	bsp_sp804_clocksource_enable(to_clksrc(cs)->base);
}

static u64 notrace bsp_sp804_sched_clock_read(void)
{
	return ~readl_relaxed(bsp_sp804_sched_clock_base + TIMER_VALUE);
}

static cycle_t bsp_sp804_clocksource_read(struct clocksource *cs)
{
	return ~(cycle_t)readl_relaxed(to_clksrc(cs)->base + TIMER_VALUE);
}

static void __init bsp_sp804_clocksource_init(void __iomem *base,
					    unsigned long rate)
{
	bsp_sp804_clksrc.base = base;
	bsp_sp804_clksrc.clksrc.name = "bsp_sp804";
	bsp_sp804_clksrc.clksrc.rating = 499;
	bsp_sp804_clksrc.clksrc.read = bsp_sp804_clocksource_read;
	bsp_sp804_clksrc.clksrc.resume = bsp_sp804_clocksource_resume;
	bsp_sp804_clksrc.clksrc.mask = CLOCKSOURCE_MASK(32);
	bsp_sp804_clksrc.clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;

	bsp_sp804_clocksource_enable(base);

	clocksource_register_hz(&bsp_sp804_clksrc.clksrc, rate);

	bsp_sp804_sched_clock_base = base;
	sched_clock_register(bsp_sp804_sched_clock_read, 32, rate);
}

static int bsp_sp804_clockevent_shutdown(struct clock_event_device *clkevt)
{
	struct bsp_sp804_clockevent_device *clkevt = to_clkevt(clkevt);

	writel(0, clkevt->base + TIMER_CTRL);

	return 0;
}

static int bsp_sp804_clockevent_set_next_event(unsigned long next,
					     struct clock_event_device *clkevt)
{
	unsigned long ctrl;
	struct bsp_sp804_clockevent_device *clkevt = to_clkevt(clkevt);

	writel(TIMER_CTRL_32BIT, clkevt->base + TIMER_CTRL);

	writel(next, clkevt->base + TIMER_LOAD);
	writel(next, clkevt->base + TIMER_LOAD);

	ctrl = TIMER_CTRL_32BIT |
	       TIMER_CTRL_IE |
	       TIMER_CTRL_ONESHOT |
	       TIMER_CTRL_ENABLE;
	writel(ctrl, clkevt->base + TIMER_CTRL);

	return 0;
}

static int sp804_clockevent_set_periodic(struct clock_event_device *clkevt)
{
	unsigned long ctrl;
	struct bsp_sp804_clockevent_device *clkevt = to_clkevt(clkevt);

	writel(TIMER_CTRL_32BIT, clkevt->base + TIMER_CTRL);

	writel(clkevt->reload, clkevt->base + TIMER_LOAD);
	writel(clkevt->reload, clkevt->base + TIMER_LOAD);

	ctrl = TIMER_CTRL_32BIT |
	       TIMER_CTRL_IE |
	       TIMER_CTRL_PERIODIC |
	       TIMER_CTRL_ENABLE;
	writel(ctrl, clkevt->base + TIMER_CTRL);

	return 0;
}

static irqreturn_t bsp_sp804_clockevent_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *clkevt = dev_id;
	struct bsp_sp804_clockevent_device *clkevt = to_clkevt(clkevt);

	/* clear the interrupt */
	writel(1, clkevt->base + TIMER_INTCLR);

	clkevt->event_handler(clkevt);

	return IRQ_HANDLED;
}

static int bsp_sp804_clockevent_setup(struct bsp_sp804_clockevent_device *clkevt)
{
	struct clock_event_device *clkevt = &clkevt->clkevt;

	writel(0, clkevt->base + TIMER_CTRL);

	BUG_ON(setup_irq(clkevt->irq, &clkevt->action));

	irq_force_affinity(clkevt->irq, clkevt->cpumask);

	clockevents_config_and_register(clkevt, clkevt->rate, 0xf,
		0x7fffffff);

	return 0;
}

static void bsp_sp804_clockevent_stop(struct bsp_sp804_clockevent_device *clkevt)
{
	struct clock_event_device *clkevt = &clkevt->clkevt;

	pr_info("disable IRQ%d cpu #%d\n", clkevt->irq, smp_processor_id());

	disable_irq(clkevt->irq);

	remove_irq(clkevt->irq, &clkevt->action);

	clkevt->set_state_shutdown(clkevt);
}

static int bsp_sp804_clockevent_cpu_notify(struct notifier_block *self,
		unsigned long action, void *hcpu)
{
	/*
	 * Grab cpu pointer in each case to avoid spurious
	 * preemptible warnings
	 */
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_ONLINE:
		bsp_sp804_clockevent_setup(this_cpu_ptr(bsp_sp804_clkevt));
		break;
	case CPU_DEAD:
		bsp_sp804_clockevent_stop(this_cpu_ptr(bsp_sp804_clkevt));
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block bsp_sp804_clockevent_cpu_nb = {
	.notifier_call = bsp_sp804_clockevent_cpu_notify,
};

static void __init clockevent_init(struct bsp_sp804_clockevent_device *clkevt,
				   void __iomem *base, int irq, int cpu,
				   unsigned long rate, unsigned long reload)
{
	struct irqaction *action = NULL;
	struct clock_event_device *clkevt = NULL;

	clkevt->base = base;
	clkevt->rate = rate;
	clkevt->reload = reload;
	if (snprintf_s(clkevt->name, DEV_NAME_LEN, DEV_NAME_LEN, "clockevent %d", cpu) < 0)
		printk("snprintf_s failed! func:%s, line: %d\n", __func__, __LINE__);

	clkevt = &clkevt->clkevt;

	clkevt->name = clkevt->name;
	clkevt->cpumask = cpumask_of(cpu);
	clkevt->irq = irq;
	clkevt->set_next_event = bsp_sp804_clockevent_set_next_event;
	clkevt->set_state_shutdown = bsp_sp804_clockevent_shutdown;
	clkevt->set_state_periodic = sp804_clockevent_set_periodic;
	clkevt->features = CLOCK_EVT_FEAT_PERIODIC |
			   CLOCK_EVT_FEAT_ONESHOT |
			   CLOCK_EVT_FEAT_DYNIRQ;
	clkevt->rating = 400;

	action = &clkevt->action;

	action->name = clkevt->name;
	action->dev_id = clkevt;
	action->irq = irq;
	action->flags = IRQF_TIMER | IRQF_NOBALANCING;
	action->handler = bsp_sp804_clockevent_timer_interrupt;
}

static int __init bsp_sp804_timer_init(struct device_node *node)
{
	int ret, irq, ix, nr_cpus;
	struct clk *clk1 = NULL, *clk2 = NULL;
	void __iomem *base = NULL;
	unsigned long rate1, rate2, reload1, reload2;

	bsp_sp804_clkevt = alloc_percpu(struct bsp_sp804_clockevent_device);
	if (!bsp_sp804_clkevt) {
		pr_err("can't alloc memory.\n");
		goto out;
	}

	clk1 = of_clk_get(node, 0);
	if (IS_ERR(clk1))
		goto out_free;

	clk_prepare_enable(clk1);

	rate1 = clk_get_rate(clk1);
	reload1 = DIV_ROUND_CLOSEST(rate1, HZ);

	/* Get the 2nd clock if the timer has 3 timer clocks */
	if (of_count_phandle_with_args(node, "clocks", "#clock-cells") == 3) {
		clk2 = of_clk_get(node, 1);
		if (IS_ERR(clk2)) {
			pr_err("bsp_sp804: %s clock not found: %d\n", node->name,
					(int)PTR_ERR(clk2));
			goto out_free;
		}
		clk_prepare_enable(clk2);
		rate2 = clk_get_rate(clk2);
		reload2 = DIV_ROUND_CLOSEST(rate2, HZ);
	} else {
		rate2 = rate1;
		reload2 = rate2;
	}

	nr_cpus = of_irq_count(node);
	if (nr_cpus > num_possible_cpus())
		nr_cpus = num_possible_cpus();

	/* local timer for each CPU */
	for (ix = 0; ix < nr_cpus; ix++) {
		irq = irq_of_parse_and_map(node, ix);
		base = of_iomap(node, ix + 1);
		if (!base) {
			pr_err("can't iomap timer %d\n", ix);
			while (--ix >= 0)
				iounmap(per_cpu_ptr(bsp_sp804_clkevt, ix)->base);
			goto out_free;
		}

		clockevent_init(per_cpu_ptr(bsp_sp804_clkevt, ix), base, irq,
				ix, rate2, reload2);
	}

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("can't iomap timer %d\n", 0);
		goto out_unmap;
	}

	bsp_sp804_clocksource_init(base, rate1);

	ret = register_cpu_notifier(&bsp_sp804_clockevent_cpu_nb);
	if (ret)
		goto out_notifier;

	bsp_sp804_clockevent_setup(this_cpu_ptr(bsp_sp804_clkevt));

	return 0;

out_notifier:
	iounmap(base);
out_unmap:
	for (ix = 0; ix < nr_irqs; ix++)
		iounmap(per_cpu_ptr(bsp_sp804_clkevt, ix)->base);
out_free:
	free_percpu(bsp_sp804_clkevt);
out:
	return -ENODEV;
}
CLOCKSOURCE_OF_DECLARE(bsp_sp804, "vendor,bsp_sp804",bsp_sp804_timer_init);
