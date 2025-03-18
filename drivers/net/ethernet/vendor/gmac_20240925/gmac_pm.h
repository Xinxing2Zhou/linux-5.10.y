/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __GMAC_PM_H__
#define __GMAC_PM_H__

#include "gmac.h"

#define N			31
#define FILTERS			4
#define PM_FILTER_OFFSET_MIN	12
struct pm_config {
	unsigned char index; /* bit0--eth0 bit1--eth1 */
	unsigned char uc_pkts_enable;
	unsigned char magic_pkts_enable;
	unsigned char wakeup_pkts_enable;
	struct {
		unsigned int mask_bytes : N;
		unsigned int reserved : 1; /* userspace ignore this bit */
		unsigned char offset; /* >= 12 */
		unsigned char value[N]; /* byte string */
		unsigned char valid; /* valid filter */
	} filter[FILTERS];
};

struct pm_reg_config {
	unsigned int pmt_ctrl;
	unsigned int pmt_mask0;
	unsigned int pmt_mask1;
	unsigned int pmt_mask2;
	unsigned int pmt_mask3;
	unsigned int pmt_cmd;
	unsigned int pmt_offset;
	unsigned int pmt_crc1_0;
	unsigned int pmt_crc3_2;
};

#define PMT_CTRL		0xa00
#define PMT_MASK0		0xa04
#define PMT_MASK1		0xa08
#define PMT_MASK2		0xa0c
#define PMT_MASK3		0xa10
#define PMT_CMD			0xa14
#define PMT_OFFSET		0xa18
#define PMT_CRC1_0		0xa1c
#define PMT_CRC3_2		0xa20
#define MASK_INVALID_BIT	BIT(31)


int pmt_config(struct net_device const *ndev, struct pm_config const *config);
bool pmt_enter(struct gmac_netdev_local *ld);
void pmt_exit(struct gmac_netdev_local *ld);
void pmt_reg_restore(struct gmac_netdev_local *ld);

#endif
