/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#define DRVNAME    "ups-phy"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif
#include <linux/delay.h>
#include <linux/version.h>

#include "phy.h"

static struct phy *ups_phy_xlate(struct device *dev,
	struct of_phandle_args *args)
{
	struct phy *phy = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(phy)) {
		ups_phy_err("phy address is error or null\n");
		return ERR_PTR(-ENODEV);
	}

	return phy;
}

static void set_dev_args(struct ups_phy_priv *priv, struct device *dev)
{
	const struct of_device_id *match = NULL;

	match = of_match_device(dev->driver->of_match_table, dev);
	if (match != NULL) {
		priv->phy_ops = (struct phy_ops*)(match->data);
	}
}

static int ups_phy_set_priv(struct platform_device *pdev,
	struct ups_phy_priv *priv)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	priv->phy_clk = devm_clk_get(dev, "phy-clk");
	if (IS_ERR(priv->phy_clk)) {
		ups_phy_err("get phy_clk failed.\n");
		return PTR_ERR(priv->phy_clk);
	}

	priv->phy_base = of_iomap(np, PHY_BASE_NODE_IDX);
	if (IS_ERR_OR_NULL(priv->phy_base)) {
		ups_phy_err("phy_base ioremap failed.\n");
		goto fail;
	}

	priv->peri_base = of_iomap(np, PERI_BASE_NODE_IDX);
	if (IS_ERR_OR_NULL(priv->peri_base)) {
		ups_phy_err("peri_base ioremap failed.\n");
		goto fail;
	}

	priv->u2phy_trim_otp = of_iomap(np, TRIM_OTP_NODE_IDX);
	if (IS_ERR_OR_NULL(priv->u2phy_trim_otp))
		ups_phy_dbg("don't get u32phy trim otp.\n");

	ret = of_property_read_u32_array(np, U2PHY_TRIM_NAME,
		priv->u2phy_trim, TRIM_NUM_MAX);
	if (ret != 0)
		ups_phy_dbg("don't get u2phy trim\n");

	priv->force_5g = of_property_read_bool(np, "force-5G");

	set_dev_args(priv, dev);

	return 0;

fail:
	devm_clk_put(dev, priv->phy_clk);
	priv->phy_clk = NULL;

	if (priv->phy_base != NULL) {
		iounmap(priv->phy_base);
		priv->phy_base = NULL;
	}

	return -1;
}

static int ups_phy_probe(struct platform_device *pdev)
{
	int ret;
	struct phy *phy;
	struct phy_provider *phy_provider;
	struct ups_phy_priv *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		ups_phy_err("failed dev_kzalloc\n");
		return -ENOMEM;
	}

	ret = ups_phy_set_priv(pdev, priv);
	if (ret != 0)
		return ret;

	phy = devm_phy_create(&pdev->dev, NULL, priv->phy_ops);
	if (IS_ERR(phy)) {
		ups_phy_err("failed to create PHY, %ld\n", PTR_ERR(phy));
		return PTR_ERR(phy);
	}

	phy_set_drvdata(phy, priv);
	dev_set_drvdata(&pdev->dev, phy);

	phy_provider = devm_of_phy_provider_register(&pdev->dev, ups_phy_xlate);
	if (IS_ERR(phy_provider)) {
		ups_phy_err("failed to register phy provider, %ld\n",
			PTR_ERR(phy_provider));
		return PTR_ERR(phy_provider);
	}

	return 0;
}

static int __exit ups_phy_platform_remove(struct platform_device *pdev)
{
	struct ups_phy_priv *priv = platform_get_drvdata(pdev);

	if (priv->phy_base != NULL) {
		iounmap(priv->phy_base);
		priv->phy_base = NULL;
	}

	if (priv->peri_base != NULL) {
		iounmap(priv->peri_base);
		priv->peri_base = NULL;
	}

	if (priv->u2phy_trim_otp != NULL) {
		iounmap(priv->u2phy_trim_otp);
		priv->u2phy_trim_otp = NULL;
	}

	return 0;
}

static struct of_device_id g_ups_phy_of_match[] = {
#ifdef CONFIG_WING_UPS_XVP_PHY
	{
		.compatible = "usb2phy,xvpphy",
		.data = &g_xvp_phy_ops
	},
#endif
#ifdef CONFIG_WING_UPS_NANO_PHY
	{
		.compatible = "combophy,common",
		.data = &g_nano_phy_common_ops
	},
	{
		.compatible = "combophy,combophy0",
		.data = &g_nano_phy_combophy0_ops
	},
	{
		.compatible = "combophy,combophy1",
		.data = &g_nano_phy_combophy1_ops
	},
	{
		.compatible = "combophy,combophy2_2",
		.data = &g_nano_phy_combophy22_ops
	},
	{
		.compatible = "combophy,combophy2_3",
		.data = &g_nano_phy_combophy23_ops
	},
#endif
#ifdef CONFIG_WING_UPS_MISSILE_PHY
	{
		.compatible = "combophy,common",
		.data = &g_missile_phy_common_ops
	},
#endif
	{},
};

static struct platform_driver g_ups_phy_driver = {
	.probe = ups_phy_probe,
	.remove = __exit_p(ups_phy_platform_remove),
	.driver = {
		.name = DRVNAME,
		.of_match_table = g_ups_phy_of_match,
		.owner = THIS_MODULE,
	}
};

static int __init ups_phy_module_init(void)
{
	ups_phy_info("registered new ups phy driver\n");

	return platform_driver_register(&g_ups_phy_driver);
}
subsys_initcall(ups_phy_module_init);

static void __exit ups_phy_module_exit(void)
{
	platform_driver_unregister(&g_ups_phy_driver);
}
module_exit(ups_phy_module_exit);

MODULE_AUTHOR("Wing");
MODULE_DESCRIPTION("Wing UPS PHY driver");
MODULE_LICENSE("GPL v2");
