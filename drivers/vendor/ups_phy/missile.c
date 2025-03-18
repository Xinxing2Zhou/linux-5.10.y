/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#define DRVNAME    "[usb-missile-phy]"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include "phy.h"
#include "reg.h"

/* TEST BIT */
#define MISSILE_PHY_TEST_REG_8					(0x20)
#define MISSILE_PHY_CDR_CPF_TRIM_MASK			(0xF)
#define MISSILE_PHY_CDR_CPF_TRIM_VAL				(0xF)

#define MISSILE_PHY_TEST_REG_15					(0x3C)
#define MISSILE_PHY_TX_PLL_TRIM_USB3_SATA_MASK	(0x1F)
#define MISSILE_PHY_TX_PLL_TRIM_USB3_SATA_VAL	(0x3)
#define MISSILE_PHY_TX_PLL_TRIM_PCIE_MASK		(0x1F << 5)
#define MISSILE_PHY_TX_PLL_TRIM_PCIE_VAL			((0x3 << 5) & MISSILE_PHY_TX_PLL_TRIM_PCIE_MASK)

#define MISSILE_PHY_TEST_REG_16					(0x40)
#define MISSILE_PHY_PI_CURRENT_TRIM_MASK			(0x3)
#define MISSILE_PHY_PI_CURRENT_TRIM_VAL			(0x2)

#define MISSILE_PHY_TEST_REG_18					(0x48)
#define MISSILE_PHY_RX_TERM_USB3_MASK			(0xF)
#define MISSILE_PHY_RX_TERM_USB3_VAL				(0xA)
#define MISSILE_PHY_RX_TERM_MASK					(0xF << 4)
#define MISSILE_PHY_RX_TERM_VAL					((0x7 << 4) & MISSILE_PHY_RX_TERM_MASK)

#define MISSILE_PHY_TEST_REG_22					(0x58)
#define MISSILE_PHY_REDUCE_RX_DET_TH_MASK		(0x1 << 1)
#define MISSILE_PHY_REDUCE_RX_DET_TH_VAL			(0x1 << 1)
#define MISSILE_PHY_INTERPOLATOR_JUMP_LATER_MASK			(0x1 << 2)
#define MISSILE_PHY_INTERPOLATOR_JUMP_LATER_VAL         (0x1 << 2)

#define MISSILE_PHY_ANALOG_REG0					(0x80)
#define MISSILE_PHY_CDR_CTRL_MASK			(0x7F << 16)
#define MISSILE_PHY_CDR_CTRL_VAL				((0x48 << 16) & MISSILE_PHY_CDR_CTRL_MASK)

/* Performance optimization */
#define MISSILE_PHY_ANALOG_REG1					(0x84)
#define MISSILE_PHY_CSEL_MASK					(0x3 << 10)
#define MISSILE_PHY_CSEL_VAL						((0x1 << 10) & MISSILE_PHY_CSEL_MASK)

#define MISSILE_PHY_RESEL11_MASK					(0x3 << 12)
#define MISSILE_PHY_RESEL11_VAL					((0x1 << 12) & MISSILE_PHY_RESEL11_MASK)

#define MISSILE_PHY_RESEL12_MASK					(0x3 << 14)
#define MISSILE_PHY_RESEL12_VAL					((0x0 << 14) & MISSILE_PHY_RESEL12_MASK)

#define MISSILE_PHY_AUTO_EQ_REG0					(0xB8)
#define MISSILE_PHY_CFG_AUTO_DESKEW_EN_MASK		(0x1 << 20)
#define MISSILE_PHY_CFG_AUTO_DESKEW_EN_VAL		((0x0 << 20) & MISSILE_PHY_CFG_AUTO_DESKEW_EN_MASK)

#define MISSILE_PHY_CFG_AUTO_EQ_EN_MASK			(0x1 << 21)
#define MISSILE_PHY_CFG_AUTO_EQ_EN_VAL			((0x0 << 21) & MISSILE_PHY_CFG_AUTO_EQ_EN_MASK)

#define MISSILE_PHY_AUTO_EQ_REG5					(0xCC)
#define MISSILE_PHY_CFG_EQ_OW_MASK				(0x1F << 0)
#define MISSILE_PHY_CFG_EQ_OW_VAL				((0xC << 0) & MISSILE_PHY_CFG_EQ_OW_MASK)

#define MISSILE_PHY_CFG_EQ_OW_EN_MASK			(0x1 << 5)
#define MISSILE_PHY_CFG_EQ_OW_EN_VAL				((0x1 << 5) & MISSILE_PHY_CFG_EQ_OW_EN_MASK)

#define MISSILE_PHY_CFG_DESKEW_OW_MASK			(0x1F << 6)
#define MISSILE_PHY_CFG_DESKEW_OW_VAL			((0x10 << 6) & MISSILE_PHY_CFG_DESKEW_OW_MASK)

#define MISSILE_PHY_CFG_DESKEW_OW_EN_MASK		(0x1 << 11)
#define MISSILE_PHY_CFG_DESKEW_OW_EN_VAL			((0x1 << 11) & MISSILE_PHY_CFG_DESKEW_OW_EN_MASK)


static void missile_phy_eye(const struct ups_phy_priv *priv)
{
	u32 reg;

	reg = phy_readl(priv->phy_base + MISSILE_PHY_ANALOG_REG1);
	reg &= ~(MISSILE_PHY_CSEL_MASK);
	reg |= (MISSILE_PHY_CSEL_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_ANALOG_REG1);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_ANALOG_REG1);
	reg &= ~(MISSILE_PHY_RESEL11_MASK);
	reg |= (MISSILE_PHY_RESEL11_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_ANALOG_REG1);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_ANALOG_REG1);
	reg &= ~(MISSILE_PHY_RESEL12_MASK);
	reg |= (MISSILE_PHY_RESEL12_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_ANALOG_REG1);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_AUTO_EQ_REG0);
	reg &= ~(MISSILE_PHY_CFG_AUTO_DESKEW_EN_MASK);
	reg |= (MISSILE_PHY_CFG_AUTO_DESKEW_EN_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_AUTO_EQ_REG0);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_AUTO_EQ_REG0);
	reg &= ~(MISSILE_PHY_CFG_AUTO_EQ_EN_MASK);
	reg |= (MISSILE_PHY_CFG_AUTO_EQ_EN_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_AUTO_EQ_REG0);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_AUTO_EQ_REG5);
	reg &= ~(MISSILE_PHY_CFG_EQ_OW_MASK);
	reg |= (MISSILE_PHY_CFG_EQ_OW_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_AUTO_EQ_REG5);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_AUTO_EQ_REG5);
	reg &= ~(MISSILE_PHY_CFG_EQ_OW_EN_MASK);
	reg |= (MISSILE_PHY_CFG_EQ_OW_EN_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_AUTO_EQ_REG5);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_AUTO_EQ_REG5);
	reg &= ~(MISSILE_PHY_CFG_DESKEW_OW_MASK);
	reg |= (MISSILE_PHY_CFG_DESKEW_OW_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_AUTO_EQ_REG5);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_AUTO_EQ_REG5);
	reg &= ~(MISSILE_PHY_CFG_DESKEW_OW_EN_MASK);
	reg |= (MISSILE_PHY_CFG_DESKEW_OW_EN_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_AUTO_EQ_REG5);

	return;
}

static int missile_phy_common_init(const struct ups_phy_priv *priv)
{
	u32 reg;

	reg = phy_readl(priv->phy_base + MISSILE_PHY_TEST_REG_8);
	reg &= ~(MISSILE_PHY_CDR_CPF_TRIM_MASK);
	reg |= (MISSILE_PHY_CDR_CPF_TRIM_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_TEST_REG_8);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_TEST_REG_15);
	reg &= ~(MISSILE_PHY_TX_PLL_TRIM_USB3_SATA_MASK);
	reg |= (MISSILE_PHY_TX_PLL_TRIM_USB3_SATA_VAL);
	reg &= ~(MISSILE_PHY_TX_PLL_TRIM_PCIE_MASK);
	reg |= (MISSILE_PHY_TX_PLL_TRIM_PCIE_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_TEST_REG_15);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_TEST_REG_16);
	reg &= ~(MISSILE_PHY_PI_CURRENT_TRIM_MASK);
	reg |= (MISSILE_PHY_PI_CURRENT_TRIM_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_TEST_REG_16);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_TEST_REG_18);
	reg &= ~(MISSILE_PHY_RX_TERM_USB3_MASK);
	reg |= (MISSILE_PHY_RX_TERM_USB3_VAL);
	reg &= ~(MISSILE_PHY_RX_TERM_MASK);
	reg |= (MISSILE_PHY_RX_TERM_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_TEST_REG_18);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_TEST_REG_22);
	reg &= ~(MISSILE_PHY_REDUCE_RX_DET_TH_MASK |
			MISSILE_PHY_INTERPOLATOR_JUMP_LATER_MASK);
	reg |= (MISSILE_PHY_REDUCE_RX_DET_TH_VAL |
			MISSILE_PHY_INTERPOLATOR_JUMP_LATER_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_TEST_REG_22);

	reg = phy_readl(priv->phy_base + MISSILE_PHY_ANALOG_REG0);
	reg &= ~(MISSILE_PHY_CDR_CTRL_MASK);
	reg |= (MISSILE_PHY_CDR_CTRL_VAL);
	phy_writel(reg, priv->phy_base + MISSILE_PHY_ANALOG_REG0);

	return 0;
}

static int missile_phy_power_on(struct phy *phy)
{
	int ret;
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	ups_phy_dbg("+++\n");

	ret = clk_prepare(priv->phy_clk);
	if (ret != 0) {
		ups_phy_err("missile phy clk prepare failed\n");
		return -1;
	}

	ret = clk_enable(priv->phy_clk);
	if (ret != 0) {
		ups_phy_err("missile phy clk enable failed\n");
		return -1;
	}

	ret = missile_phy_common_init(priv);
	if (ret != 0) {
		ups_phy_err("missile phy common init failed\n");
		return -1;
	}

	missile_phy_eye(priv);

	ups_phy_dbg("---\n");

	return 0;
}

static int missile_phy_power_off(struct phy *phy)
{
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	clk_disable_unprepare(priv->phy_clk);

	return 0;
}

struct phy_ops g_missile_phy_common_ops = {
	.power_on = missile_phy_power_on,
	.power_off = missile_phy_power_off,
};

