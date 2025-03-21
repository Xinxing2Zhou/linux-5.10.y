/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/usb/ch9.h>

#include "phy-bsp-usb.h"

#define XVP_PHY_TRIM_OFFSET 0x0008
#define XVP_PHY_TRIM_MASK   0x1f00
#define xvp_phy_trim_val(a) (((a) << 8) & XVP_PHY_TRIM_MASK)

#define XVP_PHY_SVB_OFFSET 0x0000
#define XVP_PHY_SVB_MASK   0x0f000000
#define xvp_phy_svb_val(a) (((a) << 24) & XVP_PHY_SVB_MASK)

struct bsp_xvp_priv {
	void __iomem *crg_base;
	void __iomem *phy_base;
	void __iomem *pin_base;
	struct phy *phy;
	struct device *dev;
	struct clk **clks;
	int num_clocks;
	u32 phy_pll_offset;
	u32 phy_pll_mask;
	u32 phy_pll_val;
	u32 crg_offset;
	u32 crg_defal_mask;
	u32 crg_defal_val;
	u32 vbus_offset;
	u32 vbus_val;
	int vbus_flag;
	u32 pwren_offset;
	u32 pwren_val;
	int pwren_flag;
	u32 ana_cfg_0_eye_val;
	u32 ana_cfg_0_offset;
	int ana_cfg_0_flag;
	u32 ana_cfg_2_eye_val;
	u32 ana_cfg_2_offset;
	int ana_cfg_2_flag;
	u32 ana_cfg_4_eye_val;
	u32 ana_cfg_4_offset;
	int ana_cfg_4_flag;
	struct reset_control *usb_phy_tpor_rst;
	struct reset_control *usb_phy_por_rst;
	u32 trim_otp_addr;
	u32 trim_otp_mask;
	u32 trim_otp_bit_offset;
	u32 trim_otp_min;
	u32 trim_otp_max;
	int trim_flag;
	u32 svb_otp_addr;
	u32 svb_otp_predev5_min;
	u32 svb_otp_predev5_max;
	u32 svb_phy_predev5_val;
	int svb_predev5_flag;
	u32 svb_otp_predev4_min;
	u32 svb_otp_predev4_max;
	u32 svb_phy_predev4_val;
	int svb_predev4_flag;
	u32 svb_otp_predev3_min;
	u32 svb_otp_predev3_max;
	u32 svb_phy_predev3_val;
	int svb_predev3_flag;
	u32 svb_otp_predev2_min;
	u32 svb_otp_predev2_max;
	u32 svb_phy_predev2_val;
	int svb_predev2_flag;
	int svb_flag;
};

void bsp_usb_xvp_def_all_exist(struct bsp_xvp_priv *priv)
{
	if (priv == NULL)
		return;

	/* All parameters exist by default */
	priv->vbus_flag = 1;

	priv->pwren_flag = 1;

	priv->ana_cfg_0_flag = 1;

	priv->ana_cfg_2_flag = 1;

	priv->ana_cfg_4_flag = 1;

	priv->trim_flag = 1;

	priv->svb_predev5_flag = 1;

	priv->svb_predev4_flag = 1;

	priv->svb_predev3_flag = 1;

	priv->svb_predev2_flag = 1;

	priv->svb_flag = 1;
}

void bsp_usb_xvp_get_eye_para(const struct device *dev,
	struct bsp_xvp_priv *priv)
{
	int ret;

	if ((dev == NULL) || (priv == NULL))
		return;

	if (dev->of_node == NULL)
		return;

	/*
	 * Get phy eye parameters,if you want to change them,please open
	 * dtsi file and modify parameters at phy node.
	 */
	ret = of_property_read_u32(dev->of_node, "ana_cfg_0_eye_val",
				   &(priv->ana_cfg_0_eye_val));
	if (ret)
		priv->ana_cfg_0_flag = 0;

	ret = of_property_read_u32(dev->of_node, "ana_cfg_0_offset",
				   &(priv->ana_cfg_0_offset));
	if (ret)
		priv->ana_cfg_0_flag = 0;

	ret = of_property_read_u32(dev->of_node, "ana_cfg_2_eye_val",
				   &(priv->ana_cfg_2_eye_val));
	if (ret)
		priv->ana_cfg_2_flag = 0;

	ret = of_property_read_u32(dev->of_node, "ana_cfg_2_offset",
				   &(priv->ana_cfg_2_offset));
	if (ret)
		priv->ana_cfg_2_flag = 0;

	ret = of_property_read_u32(dev->of_node, "ana_cfg_4_eye_val",
				   &(priv->ana_cfg_4_eye_val));
	if (ret)
		priv->ana_cfg_4_flag = 0;

	ret = of_property_read_u32(dev->of_node, "ana_cfg_4_offset",
				   &(priv->ana_cfg_4_offset));
	if (ret)
		priv->ana_cfg_4_flag = 0;
}

void bsp_usb_xvp_phy_eye_config(const struct bsp_xvp_priv *priv)
{
	if (priv == NULL)
		return;

	if (priv->ana_cfg_0_flag)
		writel(priv->ana_cfg_0_eye_val, priv->phy_base + priv->ana_cfg_0_offset);

	if (priv->ana_cfg_2_flag)
		writel(priv->ana_cfg_2_eye_val, priv->phy_base + priv->ana_cfg_2_offset);

	if (priv->ana_cfg_4_flag)
		writel(priv->ana_cfg_4_eye_val, priv->phy_base + priv->ana_cfg_4_offset);
}

void bsp_usb_xvp_get_trim_para(const struct device *dev,
	struct bsp_xvp_priv *priv)
{
	int ret;

	if ((dev == NULL) || (priv == NULL))
		return;

	if (dev->of_node == NULL)
		return;

	/* get phy trim parameters */
	ret = of_property_read_u32(dev->of_node, "trim_otp_addr",
				   &(priv->trim_otp_addr));
	if (ret)
		priv->trim_flag = 0;

	ret = of_property_read_u32(dev->of_node, "trim_otp_mask",
				   &(priv->trim_otp_mask));
	if (ret)
		priv->trim_flag = 0;

	ret = of_property_read_u32(dev->of_node, "trim_otp_bit_offset",
				   &(priv->trim_otp_bit_offset));
	if (ret)
		priv->trim_flag = 0;

	ret = of_property_read_u32(dev->of_node, "trim_otp_min", &(priv->trim_otp_min));
	if (ret)
		priv->trim_flag = 0;

	ret = of_property_read_u32(dev->of_node, "trim_otp_max", &(priv->trim_otp_max));
	if (ret)
		priv->trim_flag = 0;
}

void bsp_usb_xvp_phy_trim_config(const struct bsp_xvp_priv *priv)
{
	unsigned int trim_otp_val;
	unsigned int reg;
	void __iomem *phy_trim = NULL;

	if (priv == NULL)
		return;

	if (priv->trim_flag) {
		phy_trim = ioremap(priv->trim_otp_addr, __1K__);
		if (phy_trim == NULL)
			return;

		reg = readl(phy_trim);
		trim_otp_val = (reg & priv->trim_otp_mask);
		if ((trim_otp_val >= priv->trim_otp_min) &&
				(trim_otp_val <= priv->trim_otp_max)) {
			/* set trim value to XVPV100 phy */
			reg = readl(priv->phy_base + XVP_PHY_TRIM_OFFSET);
			reg &= ~XVP_PHY_TRIM_MASK;
			reg |= xvp_phy_trim_val(trim_otp_val >> priv->trim_otp_bit_offset);
			writel(reg, priv->phy_base + XVP_PHY_TRIM_OFFSET);
		}
		iounmap(phy_trim);
	}
}

void bsp_usb_xvp_get_svb_para_1(const struct device *dev,
	struct bsp_xvp_priv *priv)
{
	int ret;

	if ((dev == NULL) || (priv == NULL))
		return;

	if (dev->of_node == NULL)
		return;

	/* get phy svb parmteters */
	ret = of_property_read_u32(dev->of_node, "svb_otp_addr", &
				   (priv->svb_otp_addr));
	if (ret)
		priv->svb_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_otp_predev5_min",
				   &(priv->svb_otp_predev5_min));
	if (ret)
		priv->svb_predev5_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_otp_predev5_max",
				   &(priv->svb_otp_predev5_max));
	if (ret)
		priv->svb_predev5_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_phy_predev5_val",
				   &(priv->svb_phy_predev5_val));
	if (ret)
		priv->svb_predev5_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_otp_predev4_min",
				   &(priv->svb_otp_predev4_min));
	if (ret)
		priv->svb_predev4_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_otp_predev4_max",
				   &(priv->svb_otp_predev4_max));
	if (ret)
		priv->svb_predev4_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_phy_predev4_val",
				   &(priv->svb_phy_predev4_val));
	if (ret)
		priv->svb_predev4_flag = 0;
}

void bsp_usb_xvp_get_svb_para_2(const struct device *dev,
	struct bsp_xvp_priv *priv)
{
	int ret;

	if ((dev == NULL) || (priv == NULL))
		return;

	if (dev->of_node == NULL)
		return;

	ret = of_property_read_u32(dev->of_node, "svb_otp_predev3_min",
				   &(priv->svb_otp_predev3_min));
	if (ret)
		priv->svb_predev3_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_otp_predev3_max",
				   &(priv->svb_otp_predev3_max));
	if (ret)
		priv->svb_predev3_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_phy_predev3_val",
				   &(priv->svb_phy_predev3_val));
	if (ret)
		priv->svb_predev3_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_otp_predev2_min",
				   &(priv->svb_otp_predev2_min));
	if (ret)
		priv->svb_predev2_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_otp_predev2_max",
				   &(priv->svb_otp_predev2_max));
	if (ret)
		priv->svb_predev2_flag = 0;

	ret = of_property_read_u32(dev->of_node, "svb_phy_predev2_val",
				   &(priv->svb_phy_predev2_val));
	if (ret)
		priv->svb_predev2_flag = 0;
}

void bsp_usb_xvp_phy_svb_config(const struct bsp_xvp_priv *priv)
{
	unsigned int reg;
	unsigned int ret;
	void __iomem *phy_svb = NULL;

	if (priv == NULL)
		return;

	if (priv->svb_flag) {
		phy_svb = ioremap(priv->svb_otp_addr, __1K__);
		if (phy_svb == NULL)
			return;

		ret = readl(phy_svb);
		reg = readl(priv->phy_base + XVP_PHY_SVB_OFFSET);
		reg &= ~XVP_PHY_SVB_MASK;
		if ((ret >= priv->svb_otp_predev5_min) &&
				(ret < priv->svb_otp_predev5_max) && (priv->svb_predev5_flag))
			reg |= xvp_phy_svb_val(priv->svb_phy_predev5_val);
		else if ((ret >= priv->svb_otp_predev4_min) &&
				(ret < priv->svb_otp_predev4_max) && (priv->svb_predev4_flag))
			reg |= xvp_phy_svb_val(priv->svb_phy_predev4_val);
		else if ((ret >= priv->svb_otp_predev3_min) &&
				(ret <= priv->svb_otp_predev3_max) && (priv->svb_predev3_flag))
			reg |= xvp_phy_svb_val(priv->svb_phy_predev3_val);
		else if ((ret > priv->svb_otp_predev2_min) &&
				(ret <= priv->svb_otp_predev2_max) && (priv->svb_predev2_flag))
			reg |= xvp_phy_svb_val(priv->svb_phy_predev2_val);
		else
			reg |= xvp_phy_svb_val(priv->svb_phy_predev4_val);

		writel(reg, priv->phy_base + XVP_PHY_SVB_OFFSET);
		iounmap(phy_svb);
	}
}

static void bsp_usb_vbus_and_pwren_config(const struct device *dev,
	struct bsp_xvp_priv *priv)
{
	int ret;

	if ((dev == NULL) || (priv == NULL))
		return;

	if (dev->of_node == NULL)
		return;

	/* Some chips do not have VBUS encapsulation and need to be configured */
	ret = of_property_read_u32(dev->of_node, "vbus_offset", &(priv->vbus_offset));
	if (ret)
		priv->vbus_flag = 0;

	ret = of_property_read_u32(dev->of_node, "vbus_val", &(priv->vbus_val));
	if (ret)
		priv->vbus_flag = 0;

	/* Some chips do not have PWREN encapsulation and need to be configured */
	ret = of_property_read_u32(dev->of_node, "pwren_offset", &(priv->pwren_offset));
	if (ret)
		priv->pwren_flag = 0;

	ret = of_property_read_u32(dev->of_node, "pwren_val", &(priv->pwren_val));
	if (ret)
		priv->pwren_flag = 0;

	if (priv->vbus_flag)
		writel(priv->vbus_val, priv->pin_base + priv->vbus_offset);

	udelay(U_LEVEL2);

	if (priv->pwren_flag)
		writel(priv->pwren_val, priv->pin_base + priv->pwren_offset);

	udelay(U_LEVEL2);
}

static int bsp_usb_xvp_get_pll_clk(const struct device *dev,
	struct bsp_xvp_priv *priv)
{
	int ret;

	if ((dev == NULL) || (priv == NULL))
		return -EINVAL;

	if (dev->of_node == NULL)
		return -EINVAL;

	/* Get phy pll clk config parameters from the phy node of the dtsi file */
	ret = of_property_read_u32(dev->of_node, "phy_pll_offset",
				   &(priv->phy_pll_offset));
	if (ret) {
		dev_err(dev, "get phy_pll_offset failed: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "phy_pll_mask", &(priv->phy_pll_mask));
	if (ret) {
		dev_err(dev, "get phy_pll_mask failed: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "phy_pll_val", &(priv->phy_pll_val));
	if (ret) {
		dev_err(dev, "get phy_pll_val failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int bsp_usb_xvp_set_crg_val(const struct device *dev,
	struct bsp_xvp_priv *priv)
{
	int ret;
	unsigned int reg;

	if ((dev == NULL) || (priv == NULL))
		return -EINVAL;

	if (dev->of_node == NULL)
		return -EINVAL;

	/* Get CRG default value from the phy node of the dtsi file */
	ret = of_property_read_u32(dev->of_node, "crg_offset", &(priv->crg_offset));
	if (ret) {
		dev_err(dev, "get crg_offset failed: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "crg_defal_mask",
				   &(priv->crg_defal_mask));
	if (ret) {
		dev_err(dev, "get crg_defal_mask failed: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "crg_defal_val",
				   &(priv->crg_defal_val));
	if (ret) {
		dev_err(dev, "get crg_defal_val failed: %d\n", ret);
		return ret;
	}

	/* write phy crg default value */
	reg = readl(priv->crg_base + priv->crg_offset);
	reg &= ~priv->crg_defal_mask;
	reg |= priv->crg_defal_val;
	writel(reg, priv->crg_base + priv->crg_offset);

	return 0;
}

static int bsp_usb_xvp_phy_get_para(struct device *dev,
	struct bsp_xvp_priv *priv)
{
	int ret;

	if ((dev == NULL) || (priv == NULL))
		return -EINVAL;

	bsp_usb_xvp_def_all_exist(priv);

	ret = bsp_usb_xvp_get_pll_clk(dev, priv);
	if (ret) {
		dev_err(dev, "get pll clk failed: %d\n", ret);
		return ret;
	}

	bsp_usb_xvp_get_trim_para(dev, priv);
	bsp_usb_xvp_get_eye_para(dev, priv);
	bsp_usb_xvp_get_svb_para_1(dev, priv);
	bsp_usb_xvp_get_svb_para_2(dev, priv);

	return 0;
}

static int bsp_usb_xvp_phy_get_clks(struct bsp_xvp_priv *priv, int count)
{
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	int i;
	if (np == NULL)
		return -EINVAL;

	priv->num_clocks = count;

	if (!count)
		return 0;

	priv->clks =
		devm_kcalloc(dev, priv->num_clocks, sizeof(struct clk *), GFP_KERNEL);
	if (priv->clks == NULL)
		return -ENOMEM;

	for (i = 0; i < priv->num_clocks; i++) {
		struct clk *clk;

		clk = of_clk_get(np, i);
		if (IS_ERR(clk)) {
			while (--i >= 0)
				clk_put(priv->clks[i]);

			devm_kfree(dev, priv->clks);
			priv->clks = NULL;
			return PTR_ERR(clk);
		}

		priv->clks[i] = clk;
	}
	return 0;
}

static int bsp_usb_xvp_clk_rst_config(struct platform_device *pdev,
	struct bsp_xvp_priv *priv)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	ret = bsp_usb_xvp_phy_get_clks(priv, of_clk_get_parent_count(np));
	if (ret) {
		dev_err(dev, "get xvp phy clk failed\n");
		return ret;
	}

	priv->usb_phy_tpor_rst = devm_reset_control_get(dev, "phy_tpor_reset");
	if (IS_ERR_OR_NULL(priv->usb_phy_tpor_rst)) {
		dev_err(dev, "get phy_tpor_reset failed: %d\n", ret);
		return PTR_ERR(priv->usb_phy_tpor_rst);
	}

	priv->usb_phy_por_rst = devm_reset_control_get(dev, "phy_por_reset");
	if (IS_ERR_OR_NULL(priv->usb_phy_por_rst)) {
		dev_err(dev, "get phy_por_reset failed: %d\n", ret);
		return PTR_ERR(priv->usb_phy_por_rst);
	}

	return 0;
}

static int bsp_usb_xvp_iomap(struct device_node *np,
				struct bsp_xvp_priv *priv)
{
	if ((np == NULL) || (priv == NULL))
		return -EINVAL;

	priv->phy_base = of_iomap(np, 0);
	if (IS_ERR(priv->phy_base))
		return -ENOMEM;

	priv->crg_base = of_iomap(np, 1);
	if (IS_ERR(priv->crg_base)) {
		iounmap(priv->phy_base);
		return -ENOMEM;
	}

	priv->pin_base = of_iomap(np, 2); /* 2:index of the pin_base in dtsi */
	if (IS_ERR(priv->pin_base)) {
		iounmap(priv->phy_base);
		iounmap(priv->crg_base);
		return -ENOMEM;
	}

	return 0;
}

static int bsp_usb_xvp_phy_init(struct phy *phy)
{
	struct bsp_xvp_priv *priv = phy_get_drvdata(phy);
	int i, ret;
	unsigned int reg;

	for (i = 0; i < priv->num_clocks; i++) {
		ret = clk_prepare_enable(priv->clks[i]);
		if (ret < 0) {
			while (--i >= 0) {
				clk_disable_unprepare(priv->clks[i]);
				clk_put(priv->clks[i]);
			}
		}
	}

	udelay(U_LEVEL5);

	/* undo por reset */
	ret = reset_control_deassert(priv->usb_phy_por_rst);
	if (ret)
		return ret;

	/* pll out clk */
	reg = readl(priv->phy_base + priv->phy_pll_offset);
	reg &= ~priv->phy_pll_mask;
	reg |= priv->phy_pll_val;
	writel(reg, priv->phy_base + priv->phy_pll_offset);

	mdelay(M_LEVEL1);

	/* undo tpor reset */
	ret = reset_control_deassert(priv->usb_phy_tpor_rst);
	if (ret)
		return ret;

	udelay(U_LEVEL6);

	bsp_usb_xvp_phy_eye_config(priv);

	bsp_usb_xvp_phy_trim_config(priv);

	bsp_usb_xvp_phy_svb_config(priv);
	return 0;
}

static int bsp_usb_xvp_phy_exit(struct phy *phy)
{
	struct bsp_xvp_priv *priv = phy_get_drvdata(phy);
	int i, ret;

	for (i = 0; i < priv->num_clocks; i++)
		clk_disable_unprepare(priv->clks[i]);

	ret = reset_control_assert(priv->usb_phy_por_rst);
	if (ret)
		return ret;

	ret = reset_control_assert(priv->usb_phy_tpor_rst);
	if (ret)
		return ret;

	return 0;
}

static const struct phy_ops bsp_usb_xvp_phy_ops = {
	.init = bsp_usb_xvp_phy_init,
	.exit = bsp_usb_xvp_phy_exit,
	.owner = THIS_MODULE,
};

static int bsp_usb_xvp_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy *phy = NULL;
	struct bsp_xvp_priv *priv = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct phy_provider *phy_provider = NULL;
	int ret;

	phy = devm_phy_create(dev, dev->of_node, &bsp_usb_xvp_phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	ret = bsp_usb_xvp_iomap(np, priv);
	if (ret) {
		devm_kfree(dev, priv);
		priv = NULL;

		return -ENOMEM;
	}

	platform_set_drvdata(pdev, priv);
	priv->dev = dev;

	ret = bsp_usb_xvp_clk_rst_config(pdev, priv);
	if (ret)
		goto xvp_unmap;

	ret = bsp_usb_xvp_phy_get_para(dev, priv);
	if (ret)
		goto xvp_unmap;

	bsp_usb_vbus_and_pwren_config(dev, priv);

	ret = bsp_usb_xvp_set_crg_val(dev, priv);
	if (ret)
		goto xvp_unmap;

	platform_set_drvdata(pdev, priv);
	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		ret = PTR_ERR(phy_provider);
		goto xvp_unmap;
	}

	return 0;
xvp_unmap:
	iounmap(priv->phy_base);
	iounmap(priv->crg_base);
	iounmap(priv->pin_base);

	devm_kfree(dev, priv);
	priv = NULL;

	return ret;
}

static int bsp_usb_xvp_phy_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bsp_xvp_priv *priv = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < priv->num_clocks; i++)
		clk_put(priv->clks[i]);

	iounmap(priv->phy_base);
	iounmap(priv->crg_base);
	iounmap(priv->pin_base);

	devm_kfree(dev, priv);
	priv = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bsp_usb_xvp_phy_suspend(struct device *dev)
{
	struct phy *phy = dev_get_drvdata(dev);

	if (bsp_usb_xvp_phy_exit(phy))
		return -1;

	return 0;
}

static int bsp_usb_xvp_phy_resume(struct device *dev)
{
	struct phy *phy = dev_get_drvdata(dev);

	if (bsp_usb_xvp_phy_init(phy))
		return -1;

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(bsp_usb_pm_ops, bsp_usb_xvp_phy_suspend,
			 bsp_usb_xvp_phy_resume);

static const struct of_device_id bsp_usb_xvp_phy_of_match[] = {
	{ .compatible = "vendor,xvp-usb2-phy" },
	{},
};

static struct platform_driver bsp_usb_xvp_phy_driver = {
	.probe = bsp_usb_xvp_phy_probe,
	.remove = bsp_usb_xvp_phy_remove,
	.driver = {
		.name = "bsp-usb-xvp-phy",
		.pm = &bsp_usb_pm_ops,
		.of_match_table = bsp_usb_xvp_phy_of_match,
	}
};
module_platform_driver(bsp_usb_xvp_phy_driver);
MODULE_DESCRIPTION("Vendor USB XVP PHY driver");
MODULE_ALIAS("platform:bsp-usb-xvp-phy");
MODULE_LICENSE("GPL v2");
