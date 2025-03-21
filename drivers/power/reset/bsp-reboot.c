/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>

#include <asm/proc-fns.h>

static void __iomem *base;
static u32 reboot_offset;

static int bsp_restart_handler(struct notifier_block *this,
				unsigned long mode, void *cmd)
{
	writel_relaxed(0xdeadbeef, base + reboot_offset);

	while (1)
		cpu_do_idle();

	return NOTIFY_DONE;
}

static struct notifier_block bsp_restart_nb = {
	.notifier_call = bsp_restart_handler,
	.priority = 128,
};

static int bsp_reboot_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int err;

	base = of_iomap(np, 0);
	if (!base) {
		WARN(1, "failed to map base address");
		return -ENODEV;
	}

	if (of_property_read_u32(np, "reboot-offset", &reboot_offset) < 0) {
		pr_err("failed to find reboot-offset property\n");
		iounmap(base);
		return -EINVAL;
	}

	err = register_restart_handler(&bsp_restart_nb);
	if (err) {
		dev_err(&pdev->dev, "cannot register restart handler (err=%d)\n",
			err);
		iounmap(base);
	}

	return err;
}

static const struct of_device_id bsp_reboot_of_match[] = {
	{ .compatible = "vendor,sysctrl" },
	{}
};

static struct platform_driver bsp_reboot_driver = {
	.probe = bsp_reboot_probe,
	.driver = {
		.name = "bsp-reboot",
		.of_match_table = bsp_reboot_of_match,
	},
};
module_platform_driver(bsp_reboot_driver);
