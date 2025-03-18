/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __FMC100_NAND_OS_H__
#define __FMC100_NAND_OS_H__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <asm/errno.h>
#include <asm/setup.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/resource.h>
#include <linux/clk.h>
#include <linux/clkdev.h>

#if (KERNEL_VERSION(3, 4, 5) <= LINUX_VERSION_CODE)
#include "../../mtdcore.h"
#endif

#define DEFAULT_NAND_PAGESIZE   2048
#define DEFAULT_NAND_OOBSIZE    64

#define NAND_BUFFER_LEN     (DEFAULT_NAND_PAGESIZE + DEFAULT_NAND_OOBSIZE)

#ifndef CONFIG_RW_H_WIDTH
#define CONFIG_RW_H_WIDTH   (10)
#warning NOT config CONFIG_RW_H_WIDTH, used default value, maybe invalid.
#endif

#ifndef CONFIG_R_L_WIDTH
#define CONFIG_R_L_WIDTH    (10)
#warning NOT config CONFIG_R_L_WIDTH, used default value, maybe invalid.
#endif

#ifndef CONFIG_W_L_WIDTH
#define CONFIG_W_L_WIDTH    (10)
#warning NOT config CONFIG_W_L_WIDTH, used default value, maybe invalid.
#endif

extern void fmc100_nand_controller_enable(int enable);

#endif /* End of __FMC100_NAND_OS_H__ */
