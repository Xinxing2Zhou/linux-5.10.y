/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <asm/io.h>
#include <linux/delay.h>

#include "../basedrv_clk.h"

#define REG_PERI_CRG3664_USB30_CTRL  0x3940
#define REG_PERI_CRG3632_USB2_PHY0   0x38C0
#define REG_PERI_CRG3665_USB3_PHY0   0x3944

#define USB3_CTRL_CRG_DEFAULT_VALUE 0x10003
#define USB2_PHY_CRG_DEFAULT_VALUE  0x107
#define USB3_PHY_CRG_DEFAULT_VALUE  0x3

#define USB3_CRG_PCLK_OCC_SEL (0x1 << 18)
#define USB3_CRG_FREECLK_SEL  (0x1 << 16)
#define USB3_CRG_PIPE_CKEN    (0x1 << 12)
#define USB3_CRG_UTMI_CKEN    (0x1 << 8)
#define USB3_CRG_SUSPEND_CKEN (0x1 << 6)
#define USB3_CRG_REF_CKEN     (0x1 << 5)
#define USB3_CRG_BUS_CKEN     (0x1 << 4)
#define USB3_CRG_SRST_REQ     (0x1 << 0)

#define USB2_PHY_CRG_APB_CKEN  (0x1 << 6)
#define USB2_PHY_CRG_XTAL_CKEN (0x1 << 4)
#define USB2_PHY_CRG_APB_SREQ  (0x1 << 2)
#define USB2_PHY_CRG_TREQ      (0x1 << 1)
#define USB2_PHY_CRG_REQ       (0x1 << 0)

#define COMBPHY_REF_CKEN      (0x1<< 24)
#define COMBPHY_SRST_REQ      (0x1<< 16)

#define USB3_PHY_CRG_REF_CKEN (0x1 << 4)
#define USB3_PHY_CRG_APB_SREQ (0x1 << 2)
#define USB3_PHY_CRG_SRST_REQ (0x1 << 1)
#define USB3_PHY_CRG_REQ      (0x1 << 0)

#define REG_USB3_CTRL_MISC          0x208
#define USB3_DISABLE_SUPER_SPEED    (0x1 << 9)

#define PINOUT_REG_BASE           0x179F0010
#define PINOUT_USB_VAL            0x1201

static unsigned int basedrv_clk_readl(const void __iomem *addr)
{
	unsigned int reg = readl(addr);

	basedrv_clk_dbg("readl(0x%lx) = %#08X\n", (uintptr_t)addr, reg);
	return reg;
}

static void basedrv_clk_writel(unsigned int v, void __iomem *addr)
{
	writel(v, addr);
	basedrv_clk_dbg("writel(0x%lx) = %#08X\n", (uintptr_t)addr, v);
}

static void usb_pinout_cfg(void)
{
	void __iomem *addr = NULL;

	addr = ioremap(PINOUT_REG_BASE, 0x4);
	if (addr == NULL) {
		basedrv_clk_info("map pinout registor failed\n");
		return;
	}

	basedrv_clk_writel(PINOUT_USB_VAL, addr);
	udelay(200); /* delay 200 us to wait vbus stable */

	iounmap(addr);
	addr = NULL;
}

static int xvpphy0_clk_prepare(struct clk_hw *hw)
{
	u32 val;
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	basedrv_clk_writel(USB2_PHY_CRG_DEFAULT_VALUE,
		clk->peri_crg_base + REG_PERI_CRG3632_USB2_PHY0);

	udelay(200); // delay 200 us

	val = basedrv_clk_readl(clk->peri_crg_base + REG_PERI_CRG3632_USB2_PHY0);
	val |= (USB2_PHY_CRG_APB_CKEN | USB2_PHY_CRG_XTAL_CKEN);
	val &= ~(USB2_PHY_CRG_APB_SREQ | USB2_PHY_CRG_REQ);
	basedrv_clk_writel(val, clk->peri_crg_base + REG_PERI_CRG3632_USB2_PHY0);

	basedrv_clk_dbg("---");

	return 0;
}

static void xvpphy0_clk_unprepare(struct clk_hw *hw)
{
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	basedrv_clk_writel(USB2_PHY_CRG_DEFAULT_VALUE,
		clk->peri_crg_base + REG_PERI_CRG3632_USB2_PHY0);

	basedrv_clk_dbg("---");
}

static int xvpphy0_clk_enable(struct clk_hw *hw)
{
	u32 val;
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	val = basedrv_clk_readl(clk->peri_crg_base + REG_PERI_CRG3632_USB2_PHY0);
	val &= ~(USB2_PHY_CRG_TREQ);
	basedrv_clk_writel(val, clk->peri_crg_base + REG_PERI_CRG3632_USB2_PHY0);

	basedrv_clk_dbg("---");

	return 0;
}

static void xvpphy0_clk_disable(struct clk_hw *hw)
{
	u32 val;
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	val = basedrv_clk_readl(clk->peri_crg_base + REG_PERI_CRG3632_USB2_PHY0);
	val |= USB2_PHY_CRG_TREQ;
	basedrv_clk_writel(val, clk->peri_crg_base + REG_PERI_CRG3632_USB2_PHY0);

	basedrv_clk_dbg("---");
}

static int combophy0_clk_prepare(struct clk_hw *hw)
{
	u32 val;
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	basedrv_clk_writel(USB3_PHY_CRG_DEFAULT_VALUE,
		clk->peri_crg_base + REG_PERI_CRG3665_USB3_PHY0);

	udelay(200); // delay 200 us

	val = basedrv_clk_readl(clk->peri_crg_base + REG_PERI_CRG3665_USB3_PHY0);
	val |= (USB3_PHY_CRG_REF_CKEN | USB3_PHY_CRG_APB_SREQ);
	basedrv_clk_writel(val, clk->peri_crg_base + REG_PERI_CRG3665_USB3_PHY0);

	basedrv_clk_dbg("---");

	return 0;
}

static void combophy0_clk_unprepare(struct clk_hw *hw)
{
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	basedrv_clk_writel(USB3_PHY_CRG_DEFAULT_VALUE,
		clk->peri_crg_base + REG_PERI_CRG3665_USB3_PHY0);

	basedrv_clk_dbg("---");
}

static int combophy0_clk_enable(struct clk_hw *hw)
{
	u32 val;
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	val = basedrv_clk_readl(clk->peri_crg_base + REG_PERI_CRG3665_USB3_PHY0);
	val &= ~(USB3_PHY_CRG_SRST_REQ | USB3_PHY_CRG_REQ);
	basedrv_clk_writel(val, clk->peri_crg_base + REG_PERI_CRG3665_USB3_PHY0);

	return 0;
}

static void combophy0_clk_disable(struct clk_hw *hw)
{
	u32 val;
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	val = basedrv_clk_readl(clk->peri_crg_base + REG_PERI_CRG3665_USB3_PHY0);
	val |= (USB3_PHY_CRG_SRST_REQ | USB3_PHY_CRG_REQ);
	basedrv_clk_writel(val, clk->peri_crg_base + REG_PERI_CRG3665_USB3_PHY0);
}

static int usb30_drd_clk_prepare(struct clk_hw *hw)
{
	u32 val;
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	usb_pinout_cfg();

	basedrv_clk_writel(USB3_CTRL_CRG_DEFAULT_VALUE,
		clk->peri_crg_base + REG_PERI_CRG3664_USB30_CTRL);

	val = basedrv_clk_readl(clk->peri_ctrl_base + REG_USB3_CTRL_MISC);
	val &= ~(USB3_DISABLE_SUPER_SPEED);
	basedrv_clk_writel(val, clk->peri_ctrl_base + REG_USB3_CTRL_MISC);
	udelay(200); // delay 200 us

	basedrv_clk_dbg("---");

	return 0;
}

static void usb30_drd_clk_unprepare(struct clk_hw *hw)
{
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	basedrv_clk_writel(USB3_CTRL_CRG_DEFAULT_VALUE,
		clk->peri_crg_base + REG_PERI_CRG3664_USB30_CTRL);

	basedrv_clk_dbg("---");
}

static int usb30_drd_clk_enable(struct clk_hw *hw)
{
	u32 val;
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	val = basedrv_clk_readl(clk->peri_crg_base + REG_PERI_CRG3664_USB30_CTRL);
	val |= (USB3_CRG_FREECLK_SEL | USB3_CRG_PIPE_CKEN | USB3_CRG_UTMI_CKEN |
		USB3_CRG_SUSPEND_CKEN | USB3_CRG_REF_CKEN | USB3_CRG_BUS_CKEN);
	val &= ~USB3_CRG_SRST_REQ;
	basedrv_clk_writel(val, clk->peri_crg_base + REG_PERI_CRG3664_USB30_CTRL);

	basedrv_clk_dbg("---");

	return 0;
}

static void usb30_drd_clk_disable(struct clk_hw *hw)
{
	u32 val;
	struct basedrv_clk_hw *clk = to_basedrv_clk_hw(hw);

	basedrv_clk_dbg("+++");

	val = basedrv_clk_readl(clk->peri_crg_base + REG_PERI_CRG3664_USB30_CTRL);
	val |= USB3_CRG_SRST_REQ;
	basedrv_clk_writel(val, clk->peri_crg_base + REG_PERI_CRG3664_USB30_CTRL);

	basedrv_clk_dbg("---");
}

struct clk_ops g_clk_ops_xvpphy0 = {
	.prepare = xvpphy0_clk_prepare,
	.unprepare = xvpphy0_clk_unprepare,
	.enable = xvpphy0_clk_enable,
	.disable = xvpphy0_clk_disable,
};

struct clk_ops g_clk_ops_combophy0 = {
	.prepare = combophy0_clk_prepare,
	.unprepare = combophy0_clk_unprepare,
	.enable = combophy0_clk_enable,
	.disable = combophy0_clk_disable,
};

struct clk_ops g_clk_ops_usb30_drd = {
	.prepare = usb30_drd_clk_prepare,
	.unprepare = usb30_drd_clk_unprepare,
	.enable = usb30_drd_clk_enable,
	.disable = usb30_drd_clk_disable,
};

