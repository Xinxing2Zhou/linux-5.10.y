/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#ifndef USB2_INCLUDE_PHY_H
#define USB2_INCLUDE_PHY_H

#include <linux/phy/phy.h>
extern void bsp_usb_phy_on(struct phy *phy);
extern void bsp_usb_phy_off(struct phy *phy);
extern void bsp_usb3_phy_on(struct phy *phy);
extern void bsp_usb3_phy_off(struct phy *phy);

struct bsp_priv {
	void __iomem    *peri_crg;
	void __iomem	*misc_ctrl;
	void __iomem	*sys_ctrl;
	void __iomem	*ctrl_base;
	unsigned int	phyid;
};

typedef enum mode {
	PCIE_X2 = 0,
	PCIE_X1,
	USB3
} combphy_mode;

#define U_LEVEL1 10
#define U_LEVEL2 20
#define U_LEVEL3 30
#define U_LEVEL4 50
#define U_LEVEL5 100
#define U_LEVEL6 200
#define U_LEVEL7 300
#define U_LEVEL8 500

#define M_LEVEL1 2
#define M_LEVEL2 5
#define M_LEVEL3 10
#define M_LEVEL4 20
#define M_LEVEL5 50
#define M_LEVEL6 100
#define M_LEVEL7 200

#define __1K__  0x400
#define __2K__  0x800
#define __4K__  0x1000
#define __8K__  0x2000
#define __64K__ 0x10000

#define CRG_NODE_IDX    0
#define MISC_NODE_IDX   1
#define SYS_NODE_IDX    2
#define CTRL_NODE_IDX   3

#endif /* USB2_INCLUDE_PHY_H */
