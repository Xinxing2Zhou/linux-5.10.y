/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __VDMA_USER_H__
#define __VDMA_USER_H__

struct vdmac_host {
	struct device *dev;
	struct clk *clk;
	struct reset_control *rstc;
	void __iomem *regbase;

	int irq;
};

#define VDMA_DATA_CMD   0x6

struct dmac_user_para {
	uintptr_t src;
	uintptr_t dst;
	unsigned int size;
};

extern int vdma_m2m_copy(void *dst, const void *src, size_t count);


#endif
