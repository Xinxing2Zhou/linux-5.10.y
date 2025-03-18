/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef	__BSP_CLK_H
#define	__BSP_CLK_H

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/spinlock.h>

struct platform_device;

struct bsp_clock_data {
	struct clk_onecell_data	clk_data;
	void __iomem		*base;
};

struct bsp_fixed_rate_clock {
	unsigned int		id;
	char			*name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		fixed_rate;
};

struct bsp_fixed_factor_clock {
	unsigned int		id;
	char			*name;
	const char		*parent_name;
	unsigned long		mult;
	unsigned long		div;
	unsigned long		flags;
};

struct bsp_mux_clock {
	unsigned int		id;
	const char		*name;
	const char		*const *parent_names;
	u8			num_parents;
	unsigned long		flags;
	unsigned long		offset;
	u8			shift;
	u8			width;
	u8			mux_flags;
	u32			*table;
	const char		*alias;
};

struct bsp_phase_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_names;
	unsigned long		flags;
	unsigned long		offset;
	u8			shift;
	u8			width;
	u32			*phase_degrees;
	u32			*phase_regvals;
	u8			phase_num;
};

struct bsp_divider_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		offset;
	u8			shift;
	u8			width;
	u8			div_flags;
	struct clk_div_table	*table;
	const char		*alias;
};

struct bsp_gate_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		offset;
	u8			bit_idx;
	u8			gate_flags;
	const char		*alias;
};

struct clk *bsp_register_clkgate_sep(struct device *dev, const char *name,
				      const char *parent_name,
				      unsigned long flags,
				      void __iomem *reg, u8 bit_idx,
				      u8 clk_gate_flags, spinlock_t *lock);

struct bsp_clock_data *bsp_clk_alloc(struct platform_device *pdev,
		unsigned int nr_clks);
struct bsp_clock_data *bsp_clk_init(struct device_node *np,
		unsigned int nr_clks);
long bsp_clk_register_fixed_rate(const struct bsp_fixed_rate_clock *clks,
		int nums, struct bsp_clock_data *data);
long bsp_clk_register_fixed_factor(const struct bsp_fixed_factor_clock *clks,
		int nums, struct bsp_clock_data *data);
long bsp_clk_register_mux(const struct bsp_mux_clock *clks,
		int nums, struct bsp_clock_data *data);
long bsp_clk_register_divider(const struct bsp_divider_clock *clks,
		int nums, struct bsp_clock_data *data);
long bsp_clk_register_gate(const struct bsp_gate_clock *clks,
		int nums, struct bsp_clock_data *data);
void bsp_clk_register_gate_sep(const struct bsp_gate_clock *clks,
		int nums, struct bsp_clock_data *data);

#define bsp_clk_unregister(type) \
static inline \
void bsp_clk_unregister_##type(const struct bsp_##type##_clock *clks, \
				int nums, struct bsp_clock_data *data) \
{ \
	struct clk **clocks = data->clk_data.clks; \
	int i; \
	for (i = 0; i < nums; i++) { \
		unsigned int id = clks[i].id; \
		if (clocks[id])  \
			clk_unregister_##type(clocks[id]); \
	} \
}

bsp_clk_unregister(fixed_rate)
bsp_clk_unregister(fixed_factor)
bsp_clk_unregister(mux)
bsp_clk_unregister(divider)
bsp_clk_unregister(gate)

#endif	/* __BSP_CLK_H */
