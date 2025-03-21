/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef	_AUTO_EEE_H
#define	_AUTO_EEE_H

#include "../gmac.h"

#define NO_EEE          0
#define MAC_EEE         1
#define PHY_EEE         2
#define PARTNER_EEE     2

struct phy_info {
	char *name;
	int phy_id;
	char eee_available; /* eee support by this phy */
	int (*eee_init)(struct phy_device *phy_dev);
};

/* GMAC register definition */
#define EEE_CLK			0x800
#define MASK_EEE_CLK		(0x3 << 20)
#define BIT_DISABLE_TX_CLK	BIT(21)
#define BIT_PHY_KSZ9031		BIT(20)
#define EEE_ENABLE		0x808
#define BIT_EEE_ENABLE		BIT(0)
#define EEE_TIMER		0x80C
#define EEE_LINK_STATUS		0x810
#define BIT_PHY_LINK_STATUS	BIT(0)
#define EEE_TIME_CLK_CNT	0x814

/* ----------------------------phy register-------------------------------*/
/* MMD: MDIO Manageable Device */
#define MACR		0x0D
#define MAADR		0x0E
#define EEE_DEV		0x3
#define EEE_CAPABILITY	0x14
#define	EEELPAR_DEV	0x7
#define EEELPAR		0x3D /* EEE link partner ability register */
#define EEE_ADVERTISE	0x3c
#define LP_1000BASE_EEE	BIT(2)
#define LP_100BASE_EEE	BIT(1)

struct phy_info *phy_search_ids(int phy_id);
void init_autoeee(struct gmac_netdev_local *ld);

#endif
