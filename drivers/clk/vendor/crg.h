/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __BSP_CRG_H
#define __BSP_CRG_H

#include <linux/platform_device.h>

struct bsp_clock_data;
struct bsp_reset_controller;

struct bsp_crg_funcs {
	struct bsp_clock_data*	(*register_clks)(struct platform_device *pdev);
	void (*unregister_clks)(const struct platform_device *pdev);
};

struct bsp_crg_dev {
	struct bsp_clock_data *clk_data;
	struct bsp_reset_controller *rstc;
	const struct bsp_crg_funcs *funcs;
};

#endif	/* __BSP_CRG_H */
