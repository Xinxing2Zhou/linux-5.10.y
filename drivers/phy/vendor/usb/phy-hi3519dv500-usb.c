/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/usb/ch9.h>

#include "phy-bsp-usb.h"

#define PINOUT_REG_BASE 0x179F0010
#define PINOUT_USB_VAL 0x1201

#define USB2_PHY_CRG		0x38c0
#define USB3_PHY_CRG		0x3944
#define USB3_CTRL_CRG		0x3940
#define USB3_U2_PHY_ADDR	0x17350000
#define USB3_U3_PHY_ADDR	0x17361000

#define USB3_CTRL_MISC			0x208
#define USB3_DISABLE_SUPER_SPEED	(0x1 << 9)

#define USB3_CTRL_CRG_DEFAULT_VALUE 	0x10003
#define USB2_PHY_CRG_DEFAULT_VALUE		0x107
#define USB3_PHY_CRG_DEFAULT_VALUE		0x3

#define USB3_CRG_PCLK_OCC_SEL			(0x1 << 18)
#define USB3_CRG_FREECLK_SEL			(0x1 << 16)
#define USB3_CRG_PIPE_CKEN				(0x1 << 12)
#define USB3_CRG_UTMI_CKEN				(0x1 << 8)
#define USB3_CRG_SUSPEND_CKEN			(0x1 << 6)
#define USB3_CRG_REF_CKEN				(0x1 << 5)
#define USB3_CRG_BUS_CKEN				(0x1 << 4)
#define USB3_CRG_SRST_REQ				(0x1 << 0)

#define USB2_PHY_CRG_APB_CKEN			(0x1 << 6)
#define USB2_PHY_CRG_XTAL_CKEN			(0x1 << 4)
#define USB2_PHY_CRG_APB_SREQ			(0x1 << 2)
#define USB2_PHY_CRG_TREQ				(0x1 << 1)
#define USB2_PHY_CRG_REQ				(0x1 << 0)

#define USB3_PHY_CRG_REF_CKEN			(0x1 << 4)
#define USB3_PHY_CRG_APB_SREQ			(0x1 << 2)
#define USB3_PHY_CRG_SRST_REQ			(0x1 << 1)
#define USB3_PHY_CRG_REQ				(0x1 << 0)

#define USB2_PHY_PLLCK_ADDR_OFFSET	0x14
#define USB2_PHY_PLLCK_MASK			0x00000003
#define USB2_PHY_PLLCK_VAL			((0x3 << 0) & USB2_PHY_PLLCK_MASK)

/* U2 Performance optimization */
#define U2_PHY_ANA_CFG0						(0x0)
#define U2_PHY_RG_HSTX_MBIAS_MASK			(0xF << 0)
#define U2_PHY_RG_HSTX_MBIAS_VAL			((0xB << 0) & U2_PHY_RG_HSTX_MBIAS_MASK)

#define U2_PHY_RG_HSTX_DEEN_MASK			(0x1 << 5)
#define U2_PHY_RG_HSTX_DEEN_VAL				((0x1 << 5) & U2_PHY_RG_HSTX_DEEN_MASK)

#define U2_PHY_RG_HSTX_DE_MASK				(0xF << 8)
#define U2_PHY_RG_HSTX_DE_VAL				((0x4 << 8) & U2_PHY_RG_HSTX_DE_MASK)

#define U2_PHY_ANA_CFG2						(0x8)
#define U2_PHY_RG_RT_TRIM_MASK				(0x1F << 8)
#define U2_PHY_RG_RT_TRIM_VAL				((0x18 << 8) & U2_PHY_RG_RT_TRIM_MASK)

#define U2_PHY_RG_VDISCREF_SEL_MASK			(0x7 << 16)
#define U2_PHY_RG_VDISCREF_SEL_VAL			((0x5 <<16) & U2_PHY_RG_VDISCREF_SEL_MASK)

#define U2_PHY_RG_TEST_TX_MASK				(0x3 << 20)
#define U2_PHY_RG_TEST_TX_VAL				((0x3 << 20) & U2_PHY_RG_TEST_TX_MASK)

/* OTP USB2 PHY0 TRIM */
#define OTP_REG_BASE						0x101E0000
#define OTP_USB2_PHY0_TRIM					(0x124)
#define USB2_PHY_TRIM_MASK					(0x1F)
#define USB2_PHY_TRIM_MAX					(0x1D)
#define ANA_CFG2_TRIM_SHIFT					(8)
#define USB2_PHY_TRIM_MIN					(0x9)
#define USB_U32_MAX							(0xFFFFFFFF)

/* U3 TEST BIT */
#define U3_PHY_TEST_REG_8					(0x20)
#define U3_PHY_CDR_CPF_TRIM_MASK			(0xF)
#define U3_PHY_CDR_CPF_TRIM_VAL				(0xF)

#define U3_PHY_TEST_REG_15					(0x3C)
#define U3_PHY_TX_PLL_TRIM_USB3_SATA_MASK	(0x1F)
#define U3_PHY_TX_PLL_TRIM_USB3_SATA_VAL	(0x3)
#define U3_PHY_TX_PLL_TRIM_PCIE_MASK		(0x1F << 5)
#define U3_PHY_TX_PLL_TRIM_PCIE_VAL			((0x3 << 5) & U3_PHY_TX_PLL_TRIM_PCIE_MASK)

#define U3_PHY_TEST_REG_16					(0x40)
#define U3_PHY_PI_CURRENT_TRIM_MASK			(0x3)
#define U3_PHY_PI_CURRENT_TRIM_VAL			(0x2)

#define U3_PHY_TEST_REG_18					(0x48)
#define U3_PHY_RX_TERM_USB3_MASK			(0xF)
#define U3_PHY_RX_TERM_USB3_VAL				(0xA)
#define U3_PHY_RX_TERM_MASK					(0xF << 4)
#define U3_PHY_RX_TERM_VAL					((0x7 << 4) & U3_PHY_RX_TERM_MASK)

#define U3_PHY_TEST_REG_22					(0x58)
#define U3_PHY_REDUCE_RX_DET_TH_MASK		(0x1 << 1)
#define U3_PHY_REDUCE_RX_DET_TH_VAL			(0x1 << 1)

#define U3_PHY_ANALOG_REG0					(0x80)
#define U3_PHY_CDR_CTRL_MASK			(0x7F << 16)
#define U3_PHY_CDR_CTRL_VAL				((0x48 << 16) & U3_PHY_CDR_CTRL_MASK)

/* U3 Performance optimization */
#define U3_PHY_TEST_REG22					(0x58)
#define U3_PHY_INTERPOLATOR_JUMP_LATER_MASK			(0x1 << 2)
#define U3_PHY_INTERPOLATOR_JUMP_LATER_VAL			(0x1 << 2)

#define U3_PHY_ANALOG_REG1					(0x84)
#define U3_PHY_CSEL_MASK					(0x3 << 10)
#define U3_PHY_CSEL_VAL						((0x1 << 10) & U3_PHY_CSEL_MASK)

#define U3_PHY_RESEL11_MASK					(0x3 << 12)
#define U3_PHY_RESEL11_VAL					((0x1 << 12) & U3_PHY_RESEL11_MASK)

#define U3_PHY_RESEL12_MASK					(0x3 << 14)
#define U3_PHY_RESEL12_VAL					((0x0 << 14) & U3_PHY_RESEL12_MASK)

#define U3_PHY_AUTO_EQ_REG0					(0xB8)
#define U3_PHY_CFG_AUTO_DESKEW_EN_MASK		(0x1 << 20)
#define U3_PHY_CFG_AUTO_DESKEW_EN_VAL		((0x0 << 20) & U3_PHY_CFG_AUTO_DESKEW_EN_MASK)

#define U3_PHY_CFG_AUTO_EQ_EN_MASK			(0x1 << 21)
#define U3_PHY_CFG_AUTO_EQ_EN_VAL			((0x0 << 21) & U3_PHY_CFG_AUTO_EQ_EN_MASK)

#define U3_PHY_AUTO_EQ_REG5					(0xCC)
#define U3_PHY_CFG_EQ_OW_MASK				(0x1F << 0)
#define U3_PHY_CFG_EQ_OW_VAL				((0xC << 0) & U3_PHY_CFG_EQ_OW_MASK)

#define U3_PHY_CFG_EQ_OW_EN_MASK			(0x1 << 5)
#define U3_PHY_CFG_EQ_OW_EN_VAL				((0x1 << 5) & U3_PHY_CFG_EQ_OW_EN_MASK)

#define U3_PHY_CFG_DESKEW_OW_MASK			(0x1F << 6)
#define U3_PHY_CFG_DESKEW_OW_VAL			((0x10 << 6) & U3_PHY_CFG_DESKEW_OW_MASK)

#define U3_PHY_CFG_DESKEW_OW_EN_MASK		(0x1 << 11)
#define U3_PHY_CFG_DESKEW_OW_EN_VAL			((0x1 << 11) & U3_PHY_CFG_DESKEW_OW_EN_MASK)

/* ctrl setting */
#define GTXTHRCFG	0xc108
#define GRXTHRCFG	0xc10c
#define REG_GCTL	0xc110

#define PORT_CAP_DIR		(0x3 << 12)
#define DEFAULT_HOST_MOD	(0x1 << 12)

#define USB_TXPKT_CNT_SEL		(0x1 << 29)
#define USB_TXPKT_CNT			(0x11 << 24)
#define USB_MAXTX_BURST_SIZE	(0x1 << 20)
#define CLEAN_USB3_GTXTHRCFG	0x0

#define REG_GUSB3PIPECTL0		0xc2c0
#define PCS_SSP_SOFT_RESET		(0x1 << 31)
#define SUSPEND_USB3_SS_PHY		(0x1 << 17)
#define USB3_TX_MARGIN_VAL		0x10c0012

#define USB3_GUSB2PHYCFGN	0xc200
#define USB3_SUSPENDUSB20_PHY		(0x1 << 6)

static int otp_trim_val = USB_U32_MAX;

static void usb_pinout_cfg(void)
{
	void __iomem *pinout_regbase = ioremap(PINOUT_REG_BASE, 0x4);
	if (pinout_regbase == NULL)
		return;

	writel(PINOUT_USB_VAL, pinout_regbase);
	iounmap(pinout_regbase);
}

static void get_trim_from_otp(void)
{
	void __iomem *otp_reg_base = NULL;

	if (otp_trim_val != USB_U32_MAX)
		return;

	otp_reg_base = ioremap(OTP_REG_BASE, __1K__);
	if (otp_reg_base == NULL)
		return;

	otp_trim_val = readl(otp_reg_base + OTP_USB2_PHY0_TRIM);

	iounmap(otp_reg_base);
	otp_reg_base = NULL;
}

static void usb3_u2_optimize_parameters(void)
{
	unsigned int reg, reg_trim;
	void __iomem	*u2_phy_base = ioremap(USB3_U2_PHY_ADDR, __64K__);
	if (u2_phy_base == NULL)
		return;

	reg = readl(u2_phy_base + U2_PHY_ANA_CFG0);
	reg &= ~U2_PHY_RG_HSTX_MBIAS_MASK;
	reg |= U2_PHY_RG_HSTX_MBIAS_VAL;
	writel(reg, u2_phy_base + U2_PHY_ANA_CFG0);

	reg = readl(u2_phy_base + U2_PHY_ANA_CFG0);
	reg &= ~U2_PHY_RG_HSTX_DEEN_MASK;
	reg |= U2_PHY_RG_HSTX_DEEN_VAL;
	writel(reg, u2_phy_base + U2_PHY_ANA_CFG0);

	reg = readl(u2_phy_base + U2_PHY_ANA_CFG0);
	reg &= ~U2_PHY_RG_HSTX_DE_MASK;
	reg |= U2_PHY_RG_HSTX_DE_VAL;
	writel(reg, u2_phy_base + U2_PHY_ANA_CFG0);

	reg = readl(u2_phy_base + U2_PHY_ANA_CFG2);
	reg &= ~U2_PHY_RG_VDISCREF_SEL_MASK;
	reg |= U2_PHY_RG_VDISCREF_SEL_VAL;
	writel(reg, u2_phy_base + U2_PHY_ANA_CFG2);

	reg = readl(u2_phy_base + U2_PHY_ANA_CFG2);
	reg &= ~U2_PHY_RG_TEST_TX_MASK;
	reg |= U2_PHY_RG_TEST_TX_VAL;
	writel(reg, u2_phy_base + U2_PHY_ANA_CFG2);

	reg = readl(u2_phy_base + U2_PHY_ANA_CFG2);
	reg &= ~U2_PHY_RG_RT_TRIM_MASK;
	reg |= U2_PHY_RG_RT_TRIM_VAL;

	/* trim val */
	if (otp_trim_val == USB_U32_MAX)
		goto trim_default;

	reg_trim = otp_trim_val;
	reg_trim = (reg_trim & USB2_PHY_TRIM_MASK);
	if ((reg_trim < USB2_PHY_TRIM_MIN) || (reg_trim > USB2_PHY_TRIM_MAX))
		goto trim_default;

	reg &= ~(USB2_PHY_TRIM_MASK << ANA_CFG2_TRIM_SHIFT);
	reg |= (reg_trim << ANA_CFG2_TRIM_SHIFT);

trim_default:
	writel(reg, u2_phy_base + U2_PHY_ANA_CFG2);
	iounmap(u2_phy_base);
	u2_phy_base = NULL;
}

static void usb3_u3_optimize_parameters(void)
{
	unsigned int reg;
	void __iomem	*u3phy_base = ioremap(USB3_U3_PHY_ADDR, __64K__);
	if (u3phy_base == NULL)
		return;

	reg = readl(u3phy_base + U3_PHY_TEST_REG22);
	reg &= ~(U3_PHY_INTERPOLATOR_JUMP_LATER_MASK);
	reg |= (U3_PHY_INTERPOLATOR_JUMP_LATER_VAL);
	writel(reg, u3phy_base + U3_PHY_TEST_REG22);

	reg = readl(u3phy_base + U3_PHY_ANALOG_REG1);
	reg &= ~(U3_PHY_CSEL_MASK);
	reg |= (U3_PHY_CSEL_VAL);
	writel(reg, u3phy_base + U3_PHY_ANALOG_REG1);

	reg = readl(u3phy_base + U3_PHY_ANALOG_REG1);
	reg &= ~(U3_PHY_RESEL11_MASK);
	reg |= (U3_PHY_RESEL11_VAL);
	writel(reg, u3phy_base + U3_PHY_ANALOG_REG1);

	reg = readl(u3phy_base + U3_PHY_ANALOG_REG1);
	reg &= ~(U3_PHY_RESEL12_MASK);
	reg |= (U3_PHY_RESEL12_VAL);
	writel(reg, u3phy_base + U3_PHY_ANALOG_REG1);

	reg = readl(u3phy_base + U3_PHY_AUTO_EQ_REG0);
	reg &= ~(U3_PHY_CFG_AUTO_DESKEW_EN_MASK);
	reg |= (U3_PHY_CFG_AUTO_DESKEW_EN_VAL);
	writel(reg, u3phy_base + U3_PHY_AUTO_EQ_REG0);

	reg = readl(u3phy_base + U3_PHY_AUTO_EQ_REG0);
	reg &= ~(U3_PHY_CFG_AUTO_EQ_EN_MASK);
	reg |= (U3_PHY_CFG_AUTO_EQ_EN_VAL);
	writel(reg, u3phy_base + U3_PHY_AUTO_EQ_REG0);

	reg = readl(u3phy_base + U3_PHY_AUTO_EQ_REG5);
	reg &= ~(U3_PHY_CFG_EQ_OW_MASK);
	reg |= (U3_PHY_CFG_EQ_OW_VAL);
	writel(reg, u3phy_base + U3_PHY_AUTO_EQ_REG5);

	reg = readl(u3phy_base + U3_PHY_AUTO_EQ_REG5);
	reg &= ~(U3_PHY_CFG_EQ_OW_EN_MASK);
	reg |= (U3_PHY_CFG_EQ_OW_EN_VAL);
	writel(reg, u3phy_base + U3_PHY_AUTO_EQ_REG5);

	reg = readl(u3phy_base + U3_PHY_AUTO_EQ_REG5);
	reg &= ~(U3_PHY_CFG_DESKEW_OW_MASK);
	reg |= (U3_PHY_CFG_DESKEW_OW_VAL);
	writel(reg, u3phy_base + U3_PHY_AUTO_EQ_REG5);

	reg = readl(u3phy_base + U3_PHY_AUTO_EQ_REG5);
	reg &= ~(U3_PHY_CFG_DESKEW_OW_EN_MASK);
	reg |= (U3_PHY_CFG_DESKEW_OW_EN_VAL);
	writel(reg, u3phy_base + U3_PHY_AUTO_EQ_REG5);

	iounmap(u3phy_base);
	u3phy_base = NULL;
}

static void usb3_u3_phy_init(struct phy *phy)
{
	unsigned int reg;
	struct bsp_priv *priv = phy_get_drvdata(phy);
	void __iomem	*u3phy_crg_base = priv->peri_crg + USB3_PHY_CRG;
	void __iomem	*u3phy_base = ioremap(USB3_U3_PHY_ADDR, __64K__);
	if (u3phy_base == NULL)
		return;

	reg = readl(u3phy_crg_base);
	reg &= ~(USB3_PHY_CRG_SRST_REQ);
	writel(reg, u3phy_crg_base);
	udelay(U_LEVEL6);

	reg = readl(u3phy_base + U3_PHY_TEST_REG_8);
	reg &= ~(U3_PHY_CDR_CPF_TRIM_MASK);
	reg |= (U3_PHY_CDR_CPF_TRIM_VAL);
	writel(reg, u3phy_base + U3_PHY_TEST_REG_8);

	reg = readl(u3phy_base + U3_PHY_TEST_REG_15);
	reg &= ~(U3_PHY_TX_PLL_TRIM_USB3_SATA_MASK);
	reg |= (U3_PHY_TX_PLL_TRIM_USB3_SATA_VAL);
	reg &= ~(U3_PHY_TX_PLL_TRIM_PCIE_MASK);
	reg |= (U3_PHY_TX_PLL_TRIM_PCIE_VAL);
	writel(reg, u3phy_base + U3_PHY_TEST_REG_15);

	reg = readl(u3phy_base + U3_PHY_TEST_REG_16);
	reg &= ~(U3_PHY_PI_CURRENT_TRIM_MASK);
	reg |= (U3_PHY_PI_CURRENT_TRIM_VAL);
	writel(reg, u3phy_base + U3_PHY_TEST_REG_16);

	reg = readl(u3phy_base + U3_PHY_TEST_REG_18);
	reg &= ~(U3_PHY_RX_TERM_USB3_MASK);
	reg |= (U3_PHY_RX_TERM_USB3_VAL);
	reg &= ~(U3_PHY_RX_TERM_MASK);
	reg |= (U3_PHY_RX_TERM_VAL);
	writel(reg, u3phy_base + U3_PHY_TEST_REG_18);

	reg = readl(u3phy_base + U3_PHY_TEST_REG_22);
	reg &= ~(U3_PHY_REDUCE_RX_DET_TH_MASK);
	reg |= (U3_PHY_REDUCE_RX_DET_TH_VAL);
	writel(reg, u3phy_base + U3_PHY_TEST_REG_22);

	reg = readl(u3phy_base + U3_PHY_ANALOG_REG0);
	reg &= ~(U3_PHY_CDR_CTRL_MASK);
	reg |= (U3_PHY_CDR_CTRL_VAL);
	writel(reg, u3phy_base + U3_PHY_ANALOG_REG0);

	reg = readl(u3phy_crg_base);
	reg &= ~(USB3_PHY_CRG_REQ);
	writel(reg, u3phy_crg_base);
	udelay(U_LEVEL6);

	iounmap(u3phy_base);
	u3phy_base = NULL;
}

static void usb3_crg_config(struct phy *phy)
{
	unsigned int reg;
	struct bsp_priv *priv = phy_get_drvdata(phy);
	void __iomem	*ctrl_crg_base = priv->peri_crg + USB3_CTRL_CRG;
	void __iomem	*u2phy_crg_base = priv->peri_crg + USB2_PHY_CRG;
	void __iomem	*u3phy_crg_base = priv->peri_crg + USB3_PHY_CRG;
	void __iomem	*ctrl_misc_base = priv->misc_ctrl + USB3_CTRL_MISC;
	void __iomem	*u2_phy_base = ioremap(USB3_U2_PHY_ADDR, __64K__);

	if (u2_phy_base == NULL)
		return;

	/* write default crg value */
	writel(USB3_CTRL_CRG_DEFAULT_VALUE, ctrl_crg_base);
	writel(USB2_PHY_CRG_DEFAULT_VALUE, u2phy_crg_base);
	writel(USB3_PHY_CRG_DEFAULT_VALUE, u3phy_crg_base);
	udelay(U_LEVEL6);

	/* enable port0 ss */
	reg = readl(ctrl_misc_base);
	reg &= ~(USB3_DISABLE_SUPER_SPEED);
	writel(reg, ctrl_misc_base);
	udelay(U_LEVEL6);

	/* open ctrl cken */
	reg = readl(ctrl_crg_base);
		reg |= (USB3_CRG_FREECLK_SEL |
			USB3_CRG_PIPE_CKEN | USB3_CRG_UTMI_CKEN |
			USB3_CRG_SUSPEND_CKEN | USB3_CRG_REF_CKEN |
			USB3_CRG_BUS_CKEN);
	writel(reg, ctrl_crg_base);

	/* U2 phy crg setting */
	reg = readl(u2phy_crg_base);
	reg |= (USB2_PHY_CRG_APB_CKEN | USB2_PHY_CRG_XTAL_CKEN);
	reg &= ~(USB2_PHY_CRG_APB_SREQ);
	writel(reg, u2phy_crg_base);
	udelay(U_LEVEL6);

	/* U3 phy crg setting */
	reg = readl(u3phy_crg_base);
	reg |= (USB3_PHY_CRG_REF_CKEN | USB3_PHY_CRG_APB_SREQ);
	writel(reg, u3phy_crg_base);
	udelay(U_LEVEL6);

	/* U2 phy setting */
	reg = readl(u2phy_crg_base);
	reg &= ~(USB2_PHY_CRG_REQ);
	writel(reg, u2phy_crg_base);

	reg = readl(u2_phy_base + USB2_PHY_PLLCK_ADDR_OFFSET);
	reg &= ~USB2_PHY_PLLCK_MASK;
	reg |= USB2_PHY_PLLCK_VAL;
	writel(reg, u2_phy_base + USB2_PHY_PLLCK_ADDR_OFFSET);

	reg = readl(u2phy_crg_base);
	reg &= ~(USB2_PHY_CRG_TREQ);
	writel(reg, u2phy_crg_base);

	/* U3 phy setting */
	usb3_u3_phy_init(phy);

	/* ctrl crg reset release */
	reg = readl(ctrl_crg_base);
	reg &= ~USB3_CRG_SRST_REQ;
	writel(reg, ctrl_crg_base);
	udelay(U_LEVEL6); // delay 200us

	iounmap(u2_phy_base);
	u2_phy_base = NULL;
}

static void usb3_ctrl_config(struct phy *phy)
{
	unsigned int reg;
	struct bsp_priv *priv = phy_get_drvdata(phy);
	void __iomem	*ctrl_base = priv->ctrl_base;

	reg = readl(ctrl_base + USB3_GUSB2PHYCFGN);
	reg &= ~(USB3_SUSPENDUSB20_PHY);
	writel(reg, ctrl_base + USB3_GUSB2PHYCFGN);
	udelay(U_LEVEL6);

	reg = readl(ctrl_base + REG_GUSB3PIPECTL0);
	reg |= PCS_SSP_SOFT_RESET;
	writel(reg, ctrl_base + REG_GUSB3PIPECTL0);
	udelay(U_LEVEL6);

	reg = readl(ctrl_base + REG_GCTL);
	reg &= ~PORT_CAP_DIR;
	reg |= DEFAULT_HOST_MOD; /* [13:12] 01: Host; 10: Device; 11: OTG */
	writel(reg, ctrl_base + REG_GCTL);
	udelay(U_LEVEL2);

	reg = readl(ctrl_base + REG_GUSB3PIPECTL0);
	reg &= ~PCS_SSP_SOFT_RESET;
	reg &= ~SUSPEND_USB3_SS_PHY;  /* disable suspend */
	writel(reg, ctrl_base + REG_GUSB3PIPECTL0);
	udelay(U_LEVEL2);

	reg &= CLEAN_USB3_GTXTHRCFG;
	reg |= USB_TXPKT_CNT_SEL;
	reg |= USB_TXPKT_CNT;
	reg |= USB_MAXTX_BURST_SIZE;
	writel(reg, ctrl_base + GTXTHRCFG);
	udelay(U_LEVEL2);
	writel(reg, ctrl_base + GRXTHRCFG);
	udelay(U_LEVEL2);

	iounmap(ctrl_base);
}

void bsp_usb3_phy_on(struct phy *phy)
{
	if (otp_trim_val == USB_U32_MAX)
		get_trim_from_otp();

	usb_pinout_cfg();
	usb3_crg_config(phy);
	usb3_ctrl_config(phy);
	usb3_u2_optimize_parameters();
	usb3_u3_optimize_parameters();
}
EXPORT_SYMBOL(bsp_usb3_phy_on);

void bsp_usb3_phy_off(struct phy *phy)
{
	struct bsp_priv *priv = phy_get_drvdata(phy);

	/* write default crg value */
	writel(USB3_CTRL_CRG_DEFAULT_VALUE, priv->peri_crg + USB3_CTRL_CRG);
	writel(USB3_PHY_CRG_DEFAULT_VALUE, priv->peri_crg + USB3_PHY_CRG);
	writel(USB2_PHY_CRG_DEFAULT_VALUE, priv->peri_crg + USB2_PHY_CRG);
	udelay(U_LEVEL6);
}
EXPORT_SYMBOL(bsp_usb3_phy_off);
