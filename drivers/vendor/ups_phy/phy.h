/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef UPS_PHY_H
#define UPS_PHY_H

#include <linux/phy/phy.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/clk.h>

enum {
	U2PHY_ANA_CFG0 = 0x0,
	U2PHY_ANA_CFG2 = 0x1,
	TRIM_NUM_MAX
};

#define PHY_BASE_NODE_IDX 0
#define PERI_BASE_NODE_IDX 1
#define TRIM_OTP_NODE_IDX 2

#define U2PHY_TRIM_NAME "u2phy-trim"

struct ups_phy_priv {
	void __iomem *phy_base;
	void __iomem *peri_base;
	void __iomem *u2phy_trim_otp;

	struct clk *phy_clk;
	struct phy_ops *phy_ops;

	unsigned int u2phy_trim[TRIM_NUM_MAX];
	bool force_5g;
};

#define UPS_PHY_DEBUG 0

#define ups_phy_dbg(format, arg...) \
	do { \
		if (UPS_PHY_DEBUG) \
			printk(KERN_INFO "[UPS-PHY][%s]"format, __func__, ##arg); \
	} while (0)

#define ups_phy_info(format, arg...)    \
	printk(KERN_INFO "[UPS-PHY][%s]"format, __func__, ##arg)

#define ups_phy_err(format, arg...)    \
	printk(KERN_ERR "[UPS-PHY][%s]"format, __func__, ##arg)

#ifdef CONFIG_WING_UPS_XVP_PHY
extern struct phy_ops g_xvp_phy_ops;
#endif

#ifdef CONFIG_WING_UPS_NANO_PHY
extern struct phy_ops g_nano_phy_common_ops;
extern struct phy_ops g_nano_phy_combophy0_ops;
extern struct phy_ops g_nano_phy_combophy1_ops;
extern struct phy_ops g_nano_phy_combophy22_ops;
extern struct phy_ops g_nano_phy_combophy23_ops;
#endif

#ifdef CONFIG_WING_UPS_MISSILE_PHY
extern struct phy_ops g_missile_phy_common_ops;
#endif

static inline unsigned int phy_readl(const void __iomem *addr)
{
	unsigned int reg = readl(addr);

	ups_phy_dbg("readl(0x%lx) = %#08X\n", (uintptr_t)addr, reg);
	return reg;
}

static inline void phy_writel(unsigned int v, void __iomem *addr)
{
	writel(v, addr);
	ups_phy_dbg("writel(0x%lx) = %#08X\n", (uintptr_t)addr, v);
}

void combphy_write(void __iomem *reg, u32 addr, u32 value);

void combphy2_write(void __iomem *reg, u32 addr, u32 value);

#endif /* UPS_PHY_H */
