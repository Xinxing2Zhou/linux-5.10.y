/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include "gmac_phy_fixup.h"

static int ksz8051mnl_phy_fix(struct phy_device *phy_dev)
{
	u32 v;
	int ret;

	if (phy_dev->interface != PHY_INTERFACE_MODE_RMII)
		return 0;

	ret = phy_read(phy_dev, 0x1F);
	if (ret < 0)
		return ret;
	v = ret;
	v |= (1 << 7); /* set bit 7, phy RMII 50MHz clk; */
	phy_write(phy_dev, 0x1F, v);

	ret = phy_read(phy_dev, 0x16);
	if (ret < 0)
		return ret;
	v = ret;
	v |= (1 << 1); /* set phy RMII override; */
	phy_write(phy_dev, 0x16, v);

	return 0;
}

static int ksz8081rnb_phy_fix(struct phy_device *phy_dev)
{
	u32 v;
	int ret;

	if (phy_dev->interface != PHY_INTERFACE_MODE_RMII)
		return 0;

	ret = phy_read(phy_dev, 0x1F);
	if (ret < 0)
		return ret;
	v = ret;
	v |= (1 << 7); /* set bit 7, phy RMII 50MHz clk; */
	phy_write(phy_dev, 0x1F, v);

	return 0;
}

static int unknown_phy_fix(struct phy_device *phy_dev)
{
	u32 v;
	int ret;

	if (phy_dev->interface != PHY_INTERFACE_MODE_RMII)
		return 0;

	ret = phy_read(phy_dev, 0x1F);
	if (ret < 0)
		return ret;
	v = ret;
	v |= (1 << 7); /* set bit 7, phy RMII 50MHz clk; */
	phy_write(phy_dev, 0x1F, v);

	return 0;
}

static int rtl8211e_phy_fix(struct phy_device *phy_dev)
{
	u32 v;
	int ret;

	/* select Extension page */
	phy_write(phy_dev, 0x1f, 0x7);
	/* switch ExtPage 164 */
	phy_write(phy_dev, 0x1e, 0xa4);

	/* config RGMII rx pin io driver max */
	ret = phy_read(phy_dev, 0x1c);
	if (ret < 0)
		return ret;
	v = ret;
	v = (v & 0xff03) | 0xfc;
	phy_write(phy_dev, 0x1c, v);

	/* select to page 0 */
	phy_write(phy_dev, 0x1f, 0);

	return 0;
}

void gmac_phy_register_fixups(void)
{
	phy_register_fixup_for_uid(PHY_ID_UNKNOWN, DEFAULT_PHY_MASK,
				   unknown_phy_fix);
	phy_register_fixup_for_uid(PHY_ID_KSZ8051MNL, DEFAULT_PHY_MASK,
				   ksz8051mnl_phy_fix);
	phy_register_fixup_for_uid(PHY_ID_KSZ8081RNB, DEFAULT_PHY_MASK,
				   ksz8081rnb_phy_fix);
	phy_register_fixup_for_uid(REALTEK_PHY_ID_8211E, REALTEK_PHY_MASK,
				   rtl8211e_phy_fix);
}

void gmac_phy_unregister_fixups(void)
{
	phy_unregister_fixup_for_uid(PHY_ID_UNKNOWN, DEFAULT_PHY_MASK);
	phy_unregister_fixup_for_uid(PHY_ID_KSZ8051MNL, DEFAULT_PHY_MASK);
	phy_unregister_fixup_for_uid(PHY_ID_KSZ8081RNB, DEFAULT_PHY_MASK);
	phy_unregister_fixup_for_uid(REALTEK_PHY_ID_8211E, REALTEK_PHY_MASK);
}
