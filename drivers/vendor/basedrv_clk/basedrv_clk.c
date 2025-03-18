/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#define DRVNAME "basedrv-clk"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include <linux/module.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>
#include <linux/delay.h>

#include "basedrv_clk.h"

static void ins_clk_enable(struct basedrv_clk_hw *clk)
{
	if (clk->value) {
		u32 val = readl(clk->peri_crgx);
		val &= ~clk->mask;
		val |= clk->value;
		writel(val, clk->peri_crgx);

		val = readl(clk->peri_crgx);
		if ((val & clk->mask) != clk->value) {
			pr_warn("enable '%s' clock fail: want:%#x, real:%#x\n",
			    clk->name, clk->value, val);
		}
	}

	clk->flags |= CLKHW_ENABLE;
}

static void ins_clk_disable(struct basedrv_clk_hw *clk)
{
	if (clk->mask) {
		u32 val = readl(clk->peri_crgx);
		val &= ~clk->mask;
		writel(val, clk->peri_crgx);
	}

	clk->flags &= ~CLKHW_ENABLE;
}

static void ins_clk_reset(struct basedrv_clk_hw *clk)
{
	if (clk->rstbit) {
		u32 val = readl(clk->peri_crgx);
		val |= clk->rstbit;
		writel(val, clk->peri_crgx);
	}

	clk->flags |= CLKHW_RESET;
}

static void ins_clk_unreset(struct basedrv_clk_hw *clk)
{
	if (clk->rstbit) {
		u32 val = readl(clk->peri_crgx);
		val &= ~clk->rstbit;
		writel(val, clk->peri_crgx);
	}

	clk->flags &= ~CLKHW_RESET;
}

int basedrv_clk_enable(struct clk_hw *hw)
{
	struct basedrv_clk_hw *clk = NULL;

	if (hw == NULL)
		return -1;

	clk = to_basedrv_clk_hw(hw);

	ins_clk_enable(clk);

	if (clk->flags & CLKHW_RESET) {
		ins_clk_unreset(clk);
	}

	return 0;
}

void basedrv_clk_disable(struct clk_hw *hw)
{
	struct basedrv_clk_hw *clk = NULL;

	if (hw == NULL)
		return;

	clk = to_basedrv_clk_hw(hw);

	ins_clk_disable(clk);
}

unsigned long basedrv_clk_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct basedrv_clk_hw *clk = NULL;

	if (hw == NULL)
		return parent_rate;

	clk = to_basedrv_clk_hw(hw);

	return clk->rate;
}

static long basedrv_clk_round_rate(struct clk_hw *hw, unsigned long rate,
	unsigned long *parent_rate)
{
	return rate;
}

int basedrv_clk_prepare(struct clk_hw *hw)
{
	struct basedrv_clk_hw *clk = NULL;

	if (hw == NULL)
		return -1;

	clk = to_basedrv_clk_hw(hw);

	ins_clk_enable(clk);
	ins_clk_unreset(clk);
	ins_clk_reset(clk);

	return 0;
}

void basedrv_clk_unprepare(struct clk_hw *hw)
{
	struct basedrv_clk_hw *clk = NULL;

	if (hw == NULL)
		return;

	clk = to_basedrv_clk_hw(hw);

	clk->flags |= CLKHW_RESET;
}

#if (LINUX_VERSION_CODE >=KERNEL_VERSION(5, 5, 0))
int basedrv_clk_init(struct clk_hw *hw)
{
	struct basedrv_clk_hw *clk = NULL;

	if (hw == NULL)
		return -1;

	clk = to_basedrv_clk_hw(hw);

	ins_clk_enable(clk);
	ins_clk_reset(clk);
	ins_clk_disable(clk);

	return 0;
}
#else
void basedrv_clk_init(struct clk_hw *hw)
{
	struct basedrv_clk_hw *clk = NULL;

	if (hw == NULL)
		return;

	clk = to_basedrv_clk_hw(hw);

	ins_clk_enable(clk);
	ins_clk_reset(clk);
	ins_clk_disable(clk);
}
#endif

struct clk * __init basedrv_clk_register(struct basedrv_clk_hw *hw,
	struct clk_ops *clkops)
{
	struct clk *clk = NULL;
	struct clk_init_data init;

	if (hw == NULL || clkops == NULL)
		return NULL;

	init.name = hw->name;
	init.flags = CLK_GET_RATE_NOCACHE;
#ifdef CONFIG_ARCH_SHAOLINSWORD
	init.flags |= CLK_IS_ROOT;
#endif
	init.parent_names = NULL;
	init.num_parents = 0;

	if (hw->ops == NULL)
		hw->ops = clkops;

	if (hw->ops->init == NULL)
		hw->ops->init = basedrv_clk_init;

	if (hw->ops->prepare == NULL)
		hw->ops->prepare = basedrv_clk_prepare;

	if (hw->ops->unprepare == NULL)
		hw->ops->unprepare = basedrv_clk_unprepare;

	if (hw->ops->enable == NULL)
		hw->ops->enable = basedrv_clk_enable;

	if (hw->ops->disable == NULL)
		hw->ops->disable = basedrv_clk_disable;

	if (hw->ops->recalc_rate == NULL)
		hw->ops->recalc_rate = basedrv_clk_recalc_rate;

	if (hw->ops->round_rate == NULL)
		hw->ops->round_rate = basedrv_clk_round_rate;

	init.ops = hw->ops;

	hw->hw.init = &init;

	clk = clk_register(NULL, &hw->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: register clock fail.\n", __func__);
		return NULL;
	}

	clk_register_clkdev(clk, hw->name, NULL);

	return clk;
}

static struct clk *basedrv_clk_src_get(struct of_phandle_args *clkspec, void *data)
{
	struct clk_onecell_data *clk_data = NULL;
	unsigned int idx = 0;

	if ((data == NULL) || (clkspec == NULL)) {
		return NULL;
	}

	clk_data = data;
	idx = clkspec->args[0];

	if (idx >= (clk_data->clk_num << IDX_TO_CLK_NUM)) {
		pr_err("%s: invalid clock index %d\n", __func__, idx);
		return ERR_PTR(-EINVAL);
	}

	return clk_data->clks[idx >> IDX_TO_CLK_NUM];
}

#define PERI_CRG_NODE_IDX 0
#define PERI_CTRL_NODE_IDX 1

void __init basedrv_clocks_init(struct device_node *node,
	struct basedrv_clk_hw *clks_hw, int nr_hw, unsigned int clk_num,
	struct clk_ops *clkops)
{
	int ix;
	int ret;
	void __iomem *peri_crg_base = NULL;
	void __iomem *peri_ctrl_base = NULL;
	struct clk_onecell_data *clk_data = NULL;

	if ((nr_hw > clk_num) || (clks_hw == NULL)) {
		basedrv_clk_err("invalid argument\n");
		return;
	}

	peri_crg_base = of_iomap(node, PERI_CRG_NODE_IDX);
	if (peri_crg_base == NULL) {
		basedrv_clk_err("failed to remap peri crg base\n");
		return;
	}

	peri_ctrl_base = of_iomap(node, PERI_CTRL_NODE_IDX);
	if (peri_ctrl_base == NULL) {
		basedrv_clk_err("failed to remap peri ctrl base\n");
		goto exit;
	}

	clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
	if (clk_data == NULL) {
		basedrv_clk_err("failed to allocate clk_data\n");
		goto exit;
	}

	clk_data->clk_num = clk_num;
	clk_data->clks = kzalloc(sizeof(struct clk *) * clk_num, GFP_KERNEL);
	if (clk_data->clks == NULL) {
		basedrv_clk_err("failed to allocate clks\n");
		goto exit;
	}

	for (ix = 0; ix < nr_hw; ix++) {
		struct basedrv_clk_hw *hw = &clks_hw[ix];

		hw->peri_crg_base = peri_crg_base;
		hw->peri_crgx = hw->peri_crg_base + hw->offset;
		hw->peri_ctrl_base = peri_ctrl_base;

		if ((hw->id >> IDX_TO_CLK_NUM) >= clk_num) {
			basedrv_clk_info("clk id is exceed CLK_MAX.\n");
			continue;
		}

		clk_data->clks[hw->id >> IDX_TO_CLK_NUM] = basedrv_clk_register(hw, clkops);
	}

	ret = of_clk_add_provider(node, basedrv_clk_src_get, clk_data);
	if (ret != 0) {
		basedrv_clk_err("add clk provider failed\n");
		goto exit;
	}

	return;

exit:
	if (peri_crg_base != NULL) {
		iounmap(peri_crg_base);
		peri_crg_base = NULL;
	}

	if (peri_ctrl_base != NULL) {
		iounmap(peri_ctrl_base);
		peri_ctrl_base = NULL;
	}

	if (clk_data != NULL && clk_data->clks != NULL) {
		kfree(clk_data->clks);
		clk_data->clks = NULL;
	}

	if (clk_data != NULL) {
		kfree(clk_data);
		clk_data = NULL;
	}
}

