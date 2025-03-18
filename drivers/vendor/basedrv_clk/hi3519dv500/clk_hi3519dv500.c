/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#define DRVNAME "basedrv-clk"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include <dt-bindings/clock/basedrv-clock.h>
#include <asm/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "../basedrv_clk.h"
#include "clk_hi3519dv500.h"


#define REG_PERI_CRG3665_USB3_PHY0   0x3944

static struct basedrv_clk_hw g_clks_hw[] = {
	clk(PERI_CRG3664_USB30_CTRL0,  0x0, 0x0, 0x0, 0, &g_clk_ops_usb30_drd),
	clk(PERI_CRG3632_USB2_PHY0,  0x0, 0x0, 0x0, 0, &g_clk_ops_xvpphy0),
	clk_shared(PERI_CRG3665_COMBPHY0_CLK, REG_PERI_CRG3665_USB3_PHY0, 0x0, 0x0,
		0x0, 0, &g_clk_ops_combophy0),
};

static unsigned long hi3519dv500_basedrv_clk_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	return 0;
}

static int hi3519dv500_basedrv_clk_set_rate(struct clk_hw *hw, unsigned long drate,
	unsigned long parent_rate)
{
	return 0;
}

static int hi3519dv500_basedrv_clk_get_phase(struct clk_hw *hw)
{
	return 0;
}

static int hi3519dv500_basedrv_clk_set_phase(struct clk_hw *hw, int degrees)
{
	return 0;
}

static int hi3519dv500_basedrv_clk_enable(struct clk_hw *hw)
{
	basedrv_clk_enable(hw);
	hi3519dv500_basedrv_clk_recalc_rate(hw, 0);
	return 0;
}

static struct clk_ops g_clk_ops = {
	.enable = hi3519dv500_basedrv_clk_enable,
	.recalc_rate = hi3519dv500_basedrv_clk_recalc_rate,
	.set_rate = hi3519dv500_basedrv_clk_set_rate,
	.get_phase = hi3519dv500_basedrv_clk_get_phase,
	.set_phase = hi3519dv500_basedrv_clk_set_phase,
};

static void __init hi3519dv500_basedrv_clocks_init(struct device_node *np)
{
	int ix;

	for (ix = 0; ix < ARRAY_SIZE(g_clks_hw); ix++) {
		struct basedrv_clk_hw *hw = &g_clks_hw[ix];
		struct clk_ops *ops = hw->ops;

		if (ops == NULL) {
			continue;
		}

		if (ops->enable == NULL) {
			ops->enable = hi3519dv500_basedrv_clk_enable;
		}

		if (ops->recalc_rate == NULL) {
			ops->recalc_rate = hi3519dv500_basedrv_clk_recalc_rate;
		}
	}

	basedrv_clocks_init(np, g_clks_hw, ARRAY_SIZE(g_clks_hw),
		CLK_MAX >> IDX_TO_CLK_NUM, &g_clk_ops);
}
CLK_OF_DECLARE(basedrv_clk, "basedrv-ip,clock", hi3519dv500_basedrv_clocks_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("huanglong");
