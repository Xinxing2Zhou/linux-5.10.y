/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef BASEDRV_CLK_H
#define BASEDRV_CLK_H

#include <linux/kernel.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/version.h>

#define  BASEDRV_CLK_DEBUG 0

#define basedrv_clk_dbg(format, arg...) \
	do { \
		if (BASEDRV_CLK_DEBUG) \
			printk(KERN_INFO "[UPS-CLK][%s]"format, __func__, ##arg); \
	} while (0)

#define basedrv_clk_info(format, arg...) \
	printk(KERN_INFO "[UPS-CLK][%s]"format, __func__, ##arg)

#define basedrv_clk_err(format, arg...) \
	printk(KERN_ERR "[UPS-CLK][%s]"format, __func__, ##arg)

struct clk_rate_reg {
	unsigned long rate;
	u32 regval;
};

struct basedrv_clk_hw {
	unsigned int id;
	const char *name;
	u32 offset;
	u32 mask;
	u32 value;
	u32 rstbit;

	unsigned long rate;
	struct clk_ops *ops;
	struct clk_hw hw;
	void *__iomem peri_crgx;
	void *__iomem peri_crg_base;
	void *__iomem peri_ctrl_base;

#define CLKHW_RESET  0x01
#define CLKHW_ENABLE 0x02
	u32 flags;
};

/* clk id is 4bytes alignment, logical shift right 2 bits */
#define IDX_TO_CLK_NUM 2

#define clk(_id, _mask, _value, _rstbit, _rate, _ops)  { \
.id = (_id),         \
.name = #_id,        \
.offset = (_id),     \
.mask = (_mask),     \
.value = (_value),   \
.rstbit = (_rstbit), \
.rate = (_rate),     \
.ops = (_ops),       \
}

#define clk_shared(_id, _off, _mask, _value, _rstbit, _rate, _ops) { \
.id = (_id),         \
.name = #_id,        \
.offset = (_off),    \
.mask = (_mask),     \
.value = (_value),   \
.rstbit = (_rstbit), \
.rate = (_rate),     \
.ops = (_ops),       \
}

#define to_basedrv_clk_hw(_hw) container_of(_hw, struct basedrv_clk_hw, hw)

struct clk *basedrv_clk_register(struct basedrv_clk_hw *hw, struct clk_ops *clkops);

struct clk *basedrv_of_clk_src_get(struct of_phandle_args *clkspec, void *data);

void basedrv_clocks_init(struct device_node *node, struct basedrv_clk_hw *clks_hw,
	int nr_hw, unsigned int clk_num, struct clk_ops *clkops);

int basedrv_clk_enable(struct clk_hw *hw);

void basedrv_clk_disable(struct clk_hw *hw);

unsigned long basedrv_clk_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate);

int basedrv_clk_prepare(struct clk_hw *hw);

void basedrv_clk_unprepare(struct clk_hw *hw);

#if (LINUX_VERSION_CODE >=KERNEL_VERSION(5, 5, 0))
int basedrv_clk_init(struct clk_hw *hw);
#else
void basedrv_clk_init(struct clk_hw *hw);
#endif

#endif /* BASEDRV_CLK_H */
