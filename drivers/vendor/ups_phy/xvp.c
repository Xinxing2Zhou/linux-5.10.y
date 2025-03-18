/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#define DRVNAME    "[ups-xvp-phy]"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include "phy.h"

#define REG_OTPC_REGBASE    0x00b00000
#define PHY_PARA_OTP_OFFSET 0x032c
#define PHY_RT_TRIM_MASK    0x001f
#define RG_RT_TRIM_MASK     0x1f00
#define U2_PHY_TRIM_BIT     0x0008
#define TRIM_MAX_VALUE      0x001d
#define TRIM_MIN_VALUE      0x0009

#define U2PHY_ANA_CFG0_OFFSET 0x0000
#define U2PHY_ANA_CFG0_VALUE  0x0A33CC2B
#define U2PHY_ANA_CFG2_OFFSET 0x0008
#define U2PHY_ANA_CFG2_VALUE  0x00260F0F
#define U2PHY_ANA_CFG5_OFFSET 0x0014
#define PLL_EN_MASK  (0x3U << 0)
#define PLL_EN_VALUE (0x3U << 0)

#define TX_DEEMPHASIS_STRENGTH_MASK (0xFU << 8)
#define MBIAS_MASK                  (0xFU << 0)
#define DEEMPHASIS_HALF_BIT_MASK    (0xFFU << 20)
#define DISCONNECT_VREF_MASK        (0x7U << 16)
#define TX_DEEMPHASIS_ENABLE        (0x1 << 5)

static void xvp_phy_config(const struct ups_phy_priv *priv)
{
	u32 reg;
	u32 usb2_phy_rt_trim;

	reg = phy_readl(priv->phy_base + U2PHY_ANA_CFG5_OFFSET);
	reg &= ~(PLL_EN_MASK);
	reg |= PLL_EN_VALUE;
	phy_writel(reg, priv->phy_base + U2PHY_ANA_CFG5_OFFSET);

	/* BE CAREFUL
	 * ANA_CFG2 phy eye diagram config must set before trim config,
	 * because it will write total 32 bits when config ANA_CFG2.
	 * And just set several bits when config trim
	 */
	reg = priv->u2phy_trim[U2PHY_ANA_CFG0];
	phy_writel(reg, priv->phy_base + U2PHY_ANA_CFG0_OFFSET);

	reg = priv->u2phy_trim[U2PHY_ANA_CFG2];
	phy_writel(reg, priv->phy_base + U2PHY_ANA_CFG2_OFFSET);

	if (priv->u2phy_trim_otp == NULL)
		return;

	/* configure trim from otp */
	usb2_phy_rt_trim = phy_readl(priv->u2phy_trim_otp);
	usb2_phy_rt_trim &= PHY_RT_TRIM_MASK;
	if ((usb2_phy_rt_trim >= TRIM_MIN_VALUE) &&
		(usb2_phy_rt_trim <= TRIM_MAX_VALUE)) {
		reg = phy_readl(priv->phy_base + U2PHY_ANA_CFG2_OFFSET);
		reg &=  ~(RG_RT_TRIM_MASK);
		reg |= (usb2_phy_rt_trim << U2_PHY_TRIM_BIT);
		phy_writel(reg, priv->phy_base + U2PHY_ANA_CFG2_OFFSET);
	}
}

static int xvp_phy_power_on(struct phy *phy)
{
	int ret;
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	ups_phy_dbg("+++");

	ret = clk_prepare(priv->phy_clk);
	if (ret != 0) {
		ups_phy_err("xvp phy clk prepare failed\n");
		return ret;
	}

	xvp_phy_config(priv);

	ret = clk_enable(priv->phy_clk);
	if (ret != 0) {
		ups_phy_err("xvp phy clk enable failed\n");
		return ret;
	}

	ups_phy_dbg("---");

	return 0;
}

static int xvp_phy_power_off(struct phy *phy)
{
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	ups_phy_dbg("+++");

	clk_disable_unprepare(priv->phy_clk);

	ups_phy_dbg("---");

	return 0;
}

struct phy_ops g_xvp_phy_ops = {
	.power_on = xvp_phy_power_on,
	.power_off = xvp_phy_power_off,
};

