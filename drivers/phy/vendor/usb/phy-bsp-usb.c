/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include "phy-bsp-usb.h"
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>

static int bsp_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy *phy = NULL;
	struct bsp_priv *priv = NULL;
	struct device_node *np = pdev->dev.of_node;
	if (np == NULL)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (phy == NULL) {
		devm_kfree(dev, priv);
		return -ENOMEM;
	}

	priv->peri_crg = of_iomap(np, CRG_NODE_IDX);
	if (IS_ERR(priv->peri_crg))
		priv->peri_crg = NULL;

	priv->misc_ctrl = of_iomap(np, MISC_NODE_IDX);
	if (IS_ERR(priv->misc_ctrl))
		priv->misc_ctrl = NULL;

	priv->sys_ctrl = of_iomap(np, SYS_NODE_IDX);
	if (IS_ERR(priv->sys_ctrl))
		priv->sys_ctrl = NULL;

	priv->ctrl_base = of_iomap(np, CTRL_NODE_IDX);
	if (IS_ERR(priv->ctrl_base))
		priv->ctrl_base = NULL;
#if defined(CONFIG_ARCH_HI3559AV100) || defined(CONFIG_ARCH_HI3569V100)
	if (of_property_read_u32(np, "phyid", &priv->phyid))
		return -EINVAL;
#endif
	platform_set_drvdata(pdev, phy);
	phy_set_drvdata(phy, priv);

#ifdef CONFIG_PHY_BSP_USB2
	bsp_usb_phy_on(phy);
#endif
#ifdef CONFIG_PHY_BSP_USB3
	bsp_usb3_phy_on(phy);
#endif
	iounmap(priv->peri_crg);
	iounmap(priv->misc_ctrl);
	iounmap(priv->sys_ctrl);
#if defined(CONFIG_ARCH_HI3559AV100) || defined(CONFIG_ARCH_HI3569V100)
	iounmap(priv->ctrl_base);
#endif

	return 0;
}

static int bsp_usb_phy_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy *phy = dev_get_drvdata(&pdev->dev);
	struct bsp_priv *priv = phy_get_drvdata(phy);

#ifdef CONFIG_PHY_BSP_USB2
	bsp_usb_phy_off(phy);
#endif
#ifdef CONFIG_PHY_BSP_USB3
	bsp_usb3_phy_off(phy);
#endif
	devm_kfree(dev, priv);
	devm_kfree(dev, phy);

	return 0;
}

static const struct of_device_id bsp_usb_phy_of_match[] = {
	{ .compatible = "vendor,usb-phy", },
	{ .compatible = "vendor,usb3-phy_0", },
	{ .compatible = "vendor,usb3-phy_1", },
	{ },
};
MODULE_DEVICE_TABLE(of, bsp_usb_phy_of_match);

#ifdef CONFIG_PM_SLEEP
static int bsp_usb_phy_suspend(struct device *dev)
{
	struct phy *phy = dev_get_drvdata(dev);
#ifdef CONFIG_PHY_BSP_USB2
	bsp_usb_phy_off(phy);
#endif
#ifdef CONFIG_PHY_BSP_USB3
	bsp_usb3_phy_off(phy);
#endif
	return 0;
}

static int bsp_usb_phy_resume(struct device *dev)
{
	struct phy *phy = dev_get_drvdata(dev);
#ifdef CONFIG_PHY_BSP_USB2
	bsp_usb_phy_on(phy);
#endif
#ifdef CONFIG_PHY_BSP_USB3
	bsp_usb3_phy_on(phy);
#endif
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(bsp_usb_pm_ops, bsp_usb_phy_suspend,
			 bsp_usb_phy_resume);

static struct platform_driver bsp_usb_phy_driver = {
	.probe	= bsp_usb_phy_probe,
	.remove = bsp_usb_phy_remove,
	.driver = {
		.name	= "bsp-usb-phy",
		.pm	= &bsp_usb_pm_ops,
		.of_match_table	= bsp_usb_phy_of_match,
	}
};
module_platform_driver(bsp_usb_phy_driver);
MODULE_LICENSE("GPL v2");
