/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include "autoeee.h"
#include "../gmac.h"
#include <linux/phy.h>
#include <linux/micrel_phy.h>

static u32 set_link_stat(struct gmac_netdev_local const *ld)
{
	u32 link_stat = 0;

	switch (ld->phy->speed) {
	case SPEED_10:
		link_stat |= GMAC_SPD_10M;
		break;
	case SPEED_100:
		link_stat |= GMAC_SPD_100M;
		break;
	case SPEED_1000:
		link_stat |= GMAC_SPD_1000M;
		break;
	default:
		break;
	}
	return link_stat;
}

static void set_eee_clk(struct gmac_netdev_local const *ld, u32 phy_id)
{
	u32 v;

	if ((phy_id & REALTEK_PHY_MASK) == REALTEK_PHY_ID_8211E) {
		v = readl(ld->gmac_iobase + EEE_CLK);
		v &= ~MASK_EEE_CLK;
		v |= BIT_DISABLE_TX_CLK;
		writel(v, ld->gmac_iobase + EEE_CLK);
	} else if ((phy_id & MICREL_PHY_ID_MASK) == PHY_ID_KSZ9031) {
		v = readl(ld->gmac_iobase + EEE_CLK);
		v &= ~MASK_EEE_CLK;
		v |= (BIT_DISABLE_TX_CLK | BIT_PHY_KSZ9031);
		writel(v, ld->gmac_iobase + EEE_CLK);
	}
}

static void enable_eee(struct gmac_netdev_local const *ld)
{
	u32 v;

	/* EEE_1us: 0x7c for 125M */
	writel(0x7c, ld->gmac_iobase +
	       EEE_TIME_CLK_CNT);
	writel(0x1e0400, ld->gmac_iobase + EEE_TIMER);

	v = readl(ld->gmac_iobase + EEE_LINK_STATUS);
	v |= 0x3 << 1; /* auto EEE and ... */
	v |= BIT_PHY_LINK_STATUS; /* phy linkup */
	writel(v, ld->gmac_iobase + EEE_LINK_STATUS);

	v = readl(ld->gmac_iobase + EEE_ENABLE);
	v |= BIT_EEE_ENABLE; /* enable EEE */
	writel(v, ld->gmac_iobase + EEE_ENABLE);
}

static void set_phy_eee_mode(struct gmac_netdev_local const *ld)
{
	u32 v;
	if (netif_msg_wol(ld))
		pr_info("enter phy-EEE mode\n");

	v = readl(ld->gmac_iobase + EEE_ENABLE);
	v &= ~BIT_EEE_ENABLE; /* disable auto-EEE */
	writel(v, ld->gmac_iobase + EEE_ENABLE);
}

void init_autoeee(struct gmac_netdev_local *ld)
{
	int phy_id;
	int eee_available, lp_eee_capable;
	u32 v, link_stat;
	struct phy_info *phy_info = NULL;
	if (ld == NULL || ld->eee_init == NULL || ld->phy == NULL)
		return;
	phy_id = ld->phy->phy_id;
	if (ld->eee_init != NULL)
		goto eee_init;

	phy_info = phy_search_ids(phy_id);
	if (phy_info == NULL)
		goto not_support;

	eee_available = phy_info->eee_available;
	if (netif_msg_wol(ld) && phy_info->name != NULL)
		pr_info("fit phy_id:0x%x, phy_name:%s, eee:%d\n",
			phy_info->phy_id, phy_info->name, eee_available);

	if (!eee_available)
		goto not_support;

	if (eee_available == PHY_EEE) {
		set_phy_eee_mode(ld);
		return;
	}

	ld->eee_init = phy_info->eee_init;
eee_init:
	link_stat = set_link_stat(ld);

	lp_eee_capable = ld->eee_init(ld->phy);
	if (lp_eee_capable < 0)
		return;

	if (ld->phy->link) {
		if (((u32)lp_eee_capable) & link_stat) {
			set_eee_clk(ld, phy_id);
			enable_eee(ld);

			if (netif_msg_wol(ld))
				pr_info("enter auto-EEE mode\n");
		} else {
			if (netif_msg_wol(ld))
				pr_info("link partner not support EEE\n");
		}
	} else {
		v = readl(ld->gmac_iobase + EEE_LINK_STATUS);
		v &= ~(BIT_PHY_LINK_STATUS); /* phy linkdown */
		writel(v, ld->gmac_iobase + EEE_LINK_STATUS);
	}

	return;

not_support:
	ld->eee_init = NULL;
	if (netif_msg_wol(ld))
		pr_info("non-EEE mode\n");
}
