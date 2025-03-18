/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include "clk.h"

static DEFINE_SPINLOCK(bsp_clk_lock);

struct bsp_clock_data *bsp_clk_alloc(struct platform_device *pdev,
						unsigned int nr_clks)
{
	struct bsp_clock_data *clk_data;
	struct resource *res;
	struct clk **clk_table;

	clk_data = devm_kmalloc(&pdev->dev, sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		return NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		goto clk_data_free;
	clk_data->base = devm_ioremap(&pdev->dev,
				res->start, resource_size(res));
	if (!clk_data->base)
		goto clk_data_free;

	clk_table = devm_kmalloc_array(&pdev->dev, nr_clks,
				       sizeof(*clk_table),
				       GFP_KERNEL);
	if (!clk_table)
		goto clk_data_base_unmap;

	clk_data->clk_data.clks = clk_table;
	clk_data->clk_data.clk_num = nr_clks;

	return clk_data;

clk_data_base_unmap:
	if (clk_data->base != NULL)
		devm_iounmap(&pdev->dev, clk_data->base);
clk_data_free:
	if (clk_data != NULL) {
		devm_kfree(&pdev->dev, clk_data);
		clk_data = NULL;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(bsp_clk_alloc);

struct bsp_clock_data *bsp_clk_init(struct device_node *np,
					     unsigned int nr_clks)
{
	struct bsp_clock_data *clk_data;
	struct clk **clk_table;
	void __iomem *base;

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("%s: failed to map clock registers\n", __func__);
		goto err;
	}

	clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		goto err;

	clk_data->base = base;
	clk_table = kcalloc(nr_clks, sizeof(*clk_table), GFP_KERNEL);
	if (!clk_table)
		goto err_data;

	clk_data->clk_data.clks = clk_table;
	clk_data->clk_data.clk_num = nr_clks;
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data->clk_data);
	return clk_data;
err_data:
	if (base) {
		iounmap(base);
		base = NULL;
	}
	kfree(clk_data);
err:
	return NULL;
}
EXPORT_SYMBOL_GPL(bsp_clk_init);

long bsp_clk_register_fixed_rate(const struct bsp_fixed_rate_clock *clks,
		int nums, struct bsp_clock_data *data)
{
	struct clk *clk;
	int i;

	for (i = 0; i < nums; i++) {
		clk = clk_register_fixed_rate(NULL, clks[i].name,
					      clks[i].parent_name,
					      clks[i].flags,
					      clks[i].fixed_rate);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			goto err;
		}
		data->clk_data.clks[clks[i].id] = clk;
	}

	return 0;

err:
	while (i--)
		clk_unregister_fixed_rate(data->clk_data.clks[clks[i].id]);

	return PTR_ERR(clk);
}
EXPORT_SYMBOL_GPL(bsp_clk_register_fixed_rate);

long bsp_clk_register_fixed_factor(const struct bsp_fixed_factor_clock *clks,
					   int nums,
					   struct bsp_clock_data *data)
{
	struct clk *clk;
	int i;

	for (i = 0; i < nums; i++) {
		clk = clk_register_fixed_factor(NULL, clks[i].name,
						clks[i].parent_name,
						clks[i].flags, clks[i].mult,
						clks[i].div);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			goto err;
		}
		data->clk_data.clks[clks[i].id] = clk;
	}

	return 0;

err:
	while (i--)
		clk_unregister_fixed_factor(data->clk_data.clks[clks[i].id]);

	return PTR_ERR(clk);
}
EXPORT_SYMBOL_GPL(bsp_clk_register_fixed_factor);

long bsp_clk_register_mux(const struct bsp_mux_clock *clks,
				  int nums, struct bsp_clock_data *data)
{
	struct clk *clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		u32 mask = BIT(clks[i].width) - 1;

		clk = clk_register_mux_table(NULL, clks[i].name,
					clks[i].parent_names,
					clks[i].num_parents, clks[i].flags,
					base + clks[i].offset, clks[i].shift,
					mask, clks[i].mux_flags,
					clks[i].table, &bsp_clk_lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			goto err;
		}

		if (clks[i].alias)
			clk_register_clkdev(clk, clks[i].alias, NULL);

		data->clk_data.clks[clks[i].id] = clk;
	}

	return 0;

err:
	while (i--)
		clk_unregister_mux(data->clk_data.clks[clks[i].id]);

	return PTR_ERR(clk);
}
EXPORT_SYMBOL_GPL(bsp_clk_register_mux);

long bsp_clk_register_divider(const struct bsp_divider_clock *clks,
				      int nums, struct bsp_clock_data *data)
{
	struct clk *clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		clk = clk_register_divider_table(NULL, clks[i].name,
						 clks[i].parent_name,
						 clks[i].flags,
						 base + clks[i].offset,
						 clks[i].shift, clks[i].width,
						 clks[i].div_flags,
						 clks[i].table,
						 &bsp_clk_lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			goto err;
		}

		if (clks[i].alias)
			clk_register_clkdev(clk, clks[i].alias, NULL);

		data->clk_data.clks[clks[i].id] = clk;
	}

	return 0;

err:
	while (i--)
		clk_unregister_divider(data->clk_data.clks[clks[i].id]);

	return PTR_ERR(clk);
}
EXPORT_SYMBOL_GPL(bsp_clk_register_divider);

long bsp_clk_register_gate(const struct bsp_gate_clock *clks,
				       int nums, struct bsp_clock_data *data)
{
	struct clk *clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		clk = clk_register_gate(NULL, clks[i].name,
						clks[i].parent_name,
						clks[i].flags,
						base + clks[i].offset,
						clks[i].bit_idx,
						clks[i].gate_flags,
						&bsp_clk_lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			goto err;
		}

		if (clks[i].alias)
			clk_register_clkdev(clk, clks[i].alias, NULL);

		data->clk_data.clks[clks[i].id] = clk;
	}

	return 0;

err:
	while (i--)
		clk_unregister_gate(data->clk_data.clks[clks[i].id]);

	return PTR_ERR(clk);
}
EXPORT_SYMBOL_GPL(bsp_clk_register_gate);

void bsp_clk_register_gate_sep(const struct bsp_gate_clock *clks,
				       int nums, struct bsp_clock_data *data)
{
	struct clk *clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		clk = bsp_register_clkgate_sep(NULL, clks[i].name,
						clks[i].parent_name,
						clks[i].flags,
						base + clks[i].offset,
						clks[i].bit_idx,
						clks[i].gate_flags,
						&bsp_clk_lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			continue;
		}

		if (clks[i].alias)
			clk_register_clkdev(clk, clks[i].alias, NULL);

		data->clk_data.clks[clks[i].id] = clk;
	}
}
EXPORT_SYMBOL_GPL(bsp_clk_register_gate_sep);

