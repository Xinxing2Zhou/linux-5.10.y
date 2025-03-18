/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#define DRVNAME    "[usb-nano-phy]"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include "phy.h"
#include "reg.h"

#ifndef CONFIG_ARCH_BSP
#include <linux/huanglong/utils/hlkapi.h>
#endif

#ifndef WUDANGSTICK_D_MASK
#define WUDANGSTICK_D_MASK (3272147200ULL)
#endif

#ifndef CONFIG_ARCH_BSP
static bool chip_type_d(void)
{
	return (get_chipid(WUDANGSTICK_D_MASK) == WUDANGSTICK_D_MASK);
}
#else
static inline bool chip_type_d(void)
{
	return true;
}
#endif

static void nano_phy_x1phy_eye(const struct ups_phy_priv *priv, u32 phy_offset)
{
	/* Disable X1 slew assist and SSC offset down 200ppm */
	combphy_write(priv->peri_base + phy_offset,
		X1_SLEW_ASSIST_DIS_AND_SSC_ADDR,
		X1_SLEW_ASSIST_DIS_AND_SSC_VAL);

	/* Set X1 TX swing compensation to be level 10 */
	combphy_write(priv->peri_base + phy_offset,
		X1_TX_SWING_COMP_ADDR, X1_TX_SWING_COMP_VAL);

	/* Set X1 RX CDR direct trim & EQ peaking bit[0] */
	combphy_write(priv->peri_base + phy_offset,
		X1_CDR_DIRECT_TRIM_EQ_PEAKING_ADDR, X1_CDR_DIRECT_TRIM_EQ_PEAKING_VAL);

	/* Disable X1 all DFE for 8G & 10G */
	combphy_write(priv->peri_base + phy_offset,
		X1_DFE_DIS_8G10G_ADDR, X1_DFE_DIS_8G10G_VAL);

	/* Set X1 RX EQ swing & EQ peaking bit[1] */
	combphy_write(priv->peri_base + phy_offset,
		EQ_SWING_INC_PEAK_FREQ_ADDR, EQ_SWING_INC_PEAK_FREQ_VAL);

	/* Set X1 TX PLL charge pump current to be 1.33uA */
	combphy_write(priv->peri_base + phy_offset,
		X1_TXPLL_TRIM_ADDR, X1_TXPLL_TRIM_VAL);

	/* Set X1 reference PLL to be 100MHz */
	combphy_write(priv->peri_base + phy_offset,
		X1_REF_CLK_100N250_ADDR, X1_REF_CLK_100N250_VAL);

	/* Set X1 EQ initial value to be 0x2a */
	combphy_write(priv->peri_base + phy_offset,
		X1_EQ_INIT_MANUAL_SET1_ADDR, X1_EQ_INIT_MANUAL_SET1_VAL);
	udelay(1); /* delay 1 us */

	combphy_write(priv->peri_base + phy_offset,
		X1_EQ_INIT_PWON_CDR_MANUAL_SET1_ADDR,
		X1_EQ_INIT_PWON_CDR_MANUAL_SET1_VAL);
	udelay(1); /* delay 1 us */

	combphy_write(priv->peri_base + phy_offset,
		X1_EQ_INIT_MANUAL_SET0_ADDR, X1_EQ_INIT_MANUAL_SET0_VAL);
	udelay(1); /* delay 1 us */

	combphy_write(priv->peri_base + phy_offset,
		X1_EQ_INIT_PWON_CDR_MANUAL_SET0_ADDR,
		X1_EQ_INIT_PWON_CDR_MANUAL_SET0_VAL);
}

static void nano_phy_x4phy_lanec_eye(const struct ups_phy_priv *priv)
{
	u32 reg;

	reg = phy_readl(priv->peri_base + PERI_COMBOPHY2_CTRL);
	reg &= COMBOPHY2_2_MODE_MASK;
	if (reg != COMBOPHY2_2_MODE_USB) {
		return;
	}

	/* Disable X4 slew assist and SSC offset down 200ppm */
	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_SLEW_ASSIST_DIS_AND_SSC_ADDR,
		X4_LANEC_SLEW_ASSIST_DIS_AND_SSC_VAL);

	/* Set X4 TX swing compensation to be level 10 */
	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_TW_SWING_COMP_ADDR, X4_LANEC_TW_SWING_COMP_VAL);

	/* Set X4 RX CDR direct trim & EQ peaking bit[0] */
	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_CDR_DIRECT_TRIM_ADDR, X4_LANEC_CDR_DIRECT_TRIM_VAL);

	/* Disable X4 all DFE for 8G & 10G */
	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_DFE_DIS_8G10G_ADDR, X4_LANEC_DFE_DIS_8G10G_VAL);

	/* Set X4 RX EQ swing & EQ peaking bit[1] */
	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_EQ_SWING_ADDR, X4_LANEC_EQ_SWING_VAL);

	/* Set X4 TX PLL charge pump current to be 1.33uA */
	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_TXPLL_TRIM_ADDR, X4_LANEC_TXPLL_TRIM_VAL);

	/* Set X4 reference PLL to be 100MHz */
	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_REF_CLK_100N250_ADDR, X4_LANEC_REF_CLK_100N250_VAL);

	/* Set X4 EQ initial value to be 0x2a */
	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_EQ_INIT_MANUAL_SET1_ADDR, X4_LANEC_EQ_INIT_MANUAL_SET1_VAL);
	udelay(1); /* delay 1 us */

	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_EQ_INIT_PWON_CDR_MANUAL_SET1_ADDR,
		X4_LANEC_EQ_INIT_PWON_CDR_MANUAL_SET1_VAL);
	udelay(1); /* delay 1 us */

	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_EQ_INIT_MANUAL_SET0_ADDR, X4_LANEC_EQ_INIT_MANUAL_SET0_VAL);
	udelay(1); /* delay 1 us */

	combphy_write(priv->peri_base + PERI_COMBOPHY2_CTRL1,
		X4_LANEC_EQ_INIT_PWON_CDR_MANUAL_SET0_ADDR,
		X4_LANEC_EQ_INIT_PWON_CDR_MANUAL_SET0_VAL);
}

static void nano_phy_x4phy_laned_eye(const struct ups_phy_priv *priv)
{
	u32 reg;

	reg = phy_readl(priv->peri_base + PERI_COMBOPHY2_CTRL);
	reg &= COMBOPHY2_3_MODE_MASK;
	if (reg != COMBOPHY2_3_MODE_USB) {
		return;
	}

	/* Disable X4 slew assist and SSC offset down 200ppm */
	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_SLEW_ASSIST_DIS_AND_SSC_ADDR,
		X4_LANED_SLEW_ASSIST_DIS_AND_SSC_VAL);

	/* Set X4 TX swing compensation to be level 10 */
	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_TW_SWING_COMP_ADDR, X4_LANED_TW_SWING_COMP_VAL);

	/* Set X4 RX CDR direct trim & EQ peaking bit[0] */
	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_CDR_DIRECT_TRIM_ADDR, X4_LANED_CDR_DIRECT_TRIM_VAL);

	/* Disable X4 all DFE for 8G & 10G */
	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_DFE_DIS_8G10G_ADDR, X4_LANED_DFE_DIS_8G10G_VAL);

	/* Set X4 RX EQ swing & EQ peaking bit[1] */
	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_EQ_SWING_ADDR, X4_LANED_EQ_SWING_VAL);

	/* Set X4 TX PLL charge pump current to be 1.33uA */
	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_TXPLL_TRIM_ADDR, X4_LANED_TXPLL_TRIM_VAL);

	/* Set X4 reference PLL to be 100MHz */
	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_REF_CLK_100N250_ADDR, X4_LANED_REF_CLK_100N250_VAL);

	/* Set inv_rxcdrclk to invert the polarity of CDR clock at input from
	 * analog to PCS, this is for X4 LaneD only */
	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_INV_RXCDRCLK_ADDR, X4_LANED_INV_RXCDRCLK_VAL);

	/* Set X4 EQ initial value to be 0x2a */
	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_EQ_INIT_MANUAL_SET1_ADDR, X4_LANED_EQ_INIT_MANUAL_SET1_VAL);
	udelay(1); /* delay 1 us */

	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_EQ_INIT_PWON_CDR_MANUAL_SET1_ADDR,
		X4_LANED_EQ_INIT_PWON_CDR_MANUAL_SET1_VAL);
	udelay(1); /* delay 1 us */

	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_EQ_INIT_MANUAL_SET0_ADDR,
		X4_LANED_EQ_INIT_MANUAL_SET0_VAL);
	udelay(1); /* delay 1 us */

	combphy2_write(priv->phy_base + PERI_COMBOPHY2_CTRL1,
		X4_LANED_EQ_INIT_PWON_CDR_MANUAL_SET0_ADDR,
		X4_LANED_EQ_INIT_PWON_CDR_MANUAL_SET0_VAL);
}

static int nano_phy_common_init(const struct ups_phy_priv *priv,
	u32 reg, u32 u3_disable_bit)
{
	int ret;
	u32 val;

	val = phy_readl(priv->peri_base + reg);
	val &= ~(u3_disable_bit);
	if (priv->force_5g)
		val |= PERI_USB3_PORT_FORCE_5G;
	phy_writel(val, priv->peri_base + reg);

	ret = clk_prepare_enable(priv->phy_clk);
	if (ret != 0) {
		ups_phy_err("clk prepare and enable failed\n");
		return -1;
	}

	return 0;
}

static int nano_phy_combophy0_power_on(struct phy *phy)
{
	int ret;
	u32 val;
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	ups_phy_dbg("+++\n");

	ret = nano_phy_common_init(priv, PERI_USB31_CTRL_CFG0,
		PERI_USB3_PORT_DISABLE);
	if (ret < 0) {
		ups_phy_err("nano combophy0 init failed.\n");
		return -1;
	}

	val = phy_readl(priv->peri_base + PERI_USB31_CTRL_CFG0);
	/* both cs and cs ec need close ovrcur */
	val &= ~PERI_PORT_OVRCUR_EN;
	if (!chip_type_d()) {
		/* because cs chip X1 combphy bug, disable u3 */
		val |= PERI_USB3_PORT_DISABLE;
	}
	phy_writel(val, priv->peri_base + PERI_USB31_CTRL_CFG0);

	nano_phy_x1phy_eye(priv, PERI_COMBOPHY0_CTRL1);

	ups_phy_dbg("---\n");

	return 0;
}

static int nano_phy_combophy1_power_on(struct phy *phy)
{
	int ret;
	u32 val;
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	ups_phy_dbg("+++\n");

	ret = nano_phy_common_init(priv, PERI_USB31_CTRL_CFG1,
		PERI_USB3_PORT_DISABLE);
	if (ret < 0) {
		ups_phy_err("nano combophy1 init failed.\n");
		return -1;
	}

	val = phy_readl(priv->peri_base + PERI_USB31_CTRL_CFG1);
	/* both cs and cs ec need close ovrcur */
	val &= ~PERI_PORT_OVRCUR_EN;
	if (!chip_type_d()) {
		/* because cs chip X1 combphy bug, disable u3 */
		val |= PERI_USB3_PORT_DISABLE;
	}
	phy_writel(val, priv->peri_base + PERI_USB31_CTRL_CFG1);

	nano_phy_x1phy_eye(priv, PERI_COMBOPHY1_CTRL1);

	ups_phy_dbg("---\n");

	return 0;
}

static int nano_phy_combophy22_power_on(struct phy *phy)
{
	int ret;
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	ups_phy_dbg("+++\n");

	ret = nano_phy_common_init(priv, PERI_USB2_CTRL_CFG1,
		PERI_USB3_PORT_DISABLE_CSU30);
	if (ret < 0) {
		ups_phy_err("nano combophy22 init failed.\n");
		return -1;
	}

	nano_phy_x4phy_lanec_eye(priv);

	ups_phy_dbg("---\n");

	return 0;
}

static int nano_phy_combophy23_power_on(struct phy *phy)
{
	int ret;
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	ups_phy_dbg("+++\n");

	ret = nano_phy_common_init(priv, PERI_USB31_CTRL_CFG2,
		PERI_USB3_PORT_DISABLE);
	if (ret < 0) {
		ups_phy_err("nano combophy23 init failed.\n");
		return -1;
	}

	nano_phy_x4phy_laned_eye(priv);

	ups_phy_dbg("---\n");

	return 0;
}

static int nano_phy_power_on(struct phy *phy)
{
	int ret;
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	ups_phy_dbg("+++\n");

	ret = clk_prepare_enable(priv->phy_clk);
	if (ret != 0) {
		ups_phy_err("nano phy clk prepare and enable failed\n");
		return -1;
	}

	ups_phy_dbg("---\n");

	return 0;
}

static int nano_phy_power_off(struct phy *phy)
{
	struct ups_phy_priv *priv = phy_get_drvdata(phy);

	clk_disable_unprepare(priv->phy_clk);

	return 0;
}

struct phy_ops g_nano_phy_common_ops = {
	.power_on = nano_phy_power_on,
	.power_off = nano_phy_power_off,
};

struct phy_ops g_nano_phy_combophy0_ops = {
	.power_on = nano_phy_combophy0_power_on,
	.power_off = nano_phy_power_off,
};

struct phy_ops g_nano_phy_combophy1_ops = {
	.power_on = nano_phy_combophy1_power_on,
	.power_off = nano_phy_power_off,
};

struct phy_ops g_nano_phy_combophy22_ops = {
	.power_on = nano_phy_combophy22_power_on,
	.power_off = nano_phy_power_off,
};

struct phy_ops g_nano_phy_combophy23_ops = {
	.power_on = nano_phy_combophy23_power_on,
	.power_off = nano_phy_power_off,
};

