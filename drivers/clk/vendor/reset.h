/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef	__BSP_RESET_H
#define	__BSP_RESET_H

#include <linux/platform_device.h>

struct device_node;
struct bsp_reset_controller;

#ifdef CONFIG_RESET_CONTROLLER
struct bsp_reset_controller *vendor_reset_init(struct platform_device *pdev);
#ifdef CONFIG_ARCH_BSP
int __init bsp_reset_init(struct device_node *np, int nr_rsts);
#endif
void bsp_reset_exit(struct bsp_reset_controller *rstc);
#else
static inline
struct bsp_reset_controller *vendor_reset_init(struct platform_device *pdev)
{
	return 0;
}
static inline void bsp_reset_exit(struct bsp_reset_controller *rstc)
{}
#endif

#endif	/* __BSP_RESET_H */
