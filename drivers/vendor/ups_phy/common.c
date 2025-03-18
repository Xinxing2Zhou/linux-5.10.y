/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#define DRVNAME    "[ups-phy-common]"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include "phy.h"
#include "reg.h"

void combphy_write(void __iomem *reg, u32 addr, u32 value)
{
	u32 val;

	val = phy_readl(reg);
	val |= PERI_COMBPHY0_TEST_RST_N; /* Combophy0,1 just l lane */
	val &= ~PERI_COMBPHY_TEST_ADDR;
	val &= ~PERI_COMBPHY_TEST_I;
	val |= (addr << PERI_COMBPHY_ADDR_OFFSET);
	val |= (value << PERI_COMBPHY_DATA_OFFSET);
	phy_writel(val, reg);

	val = phy_readl(reg);
	val |= PERI_COMBPHY_TEST_WRITE;
	phy_writel(val, reg);
	ups_phy_dbg("ComboPHY write:addr(%#x),value(%#x)\n", addr, value);
	val = phy_readl(reg);
	val &= ~PERI_COMBPHY_TEST_WRITE;
	phy_writel(val, reg);
}

void combphy2_write(void __iomem *reg, u32 addr, u32 value)
{
	u32 val;

	/* Combphy2 is 4 lanes, need configure this */
	val = phy_readl(reg);
	val |= PERI_COMBPHY20_TEST_RST_N;
	val |= PERI_COMBPHY21_TEST_RST_N;
	val |= PERI_COMBPHY22_TEST_RST_N;
	val |= PERI_COMBPHY23_TEST_RST_N;
	phy_writel(val, reg);

	combphy_write(reg, addr, value);
}
