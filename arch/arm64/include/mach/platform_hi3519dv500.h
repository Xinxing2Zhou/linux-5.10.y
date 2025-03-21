/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __BSP_CHIP_REGS_H__
#define __BSP_CHIP_REGS_H__

#define GET_SYS_BOOT_MODE(_reg)        (((_reg) >> 4) & 0x3)
#define BOOT_FROM_SPI                  0
#define BOOT_FROM_NAND                 1
#endif /* End of __BSP_CHIP_REGS_H__ */
