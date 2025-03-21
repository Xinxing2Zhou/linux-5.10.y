/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __EDMA_HI3519DV500_H__
#define __EDMA_HI3519DV500_H__

#include "edmacv310.h"
#define EDMAC_MAX_PERIPHERALS  32
#define EDMAC_CHANNEL_NUM 8

#define UART4_REG_BASE          0x11044000
#define UART3_REG_BASE          0x11043000
#define UART2_REG_BASE          0x11042000
#define UART1_REG_BASE          0x11041000
#define UART0_REG_BASE          0x11040000

#define UART0_DR    (UART0_REG_BASE + 0x0)
#define UART1_DR    (UART1_REG_BASE + 0x0)
#define UART2_DR    (UART2_REG_BASE + 0x0)
#define UART3_DR    (UART3_REG_BASE + 0x0)
#define UART4_DR    (UART4_REG_BASE + 0x0)

#define I2C5_REG_BASE             0x11065000
#define I2C4_REG_BASE             0x11064000
#define I2C3_REG_BASE             0x11063000
#define I2C2_REG_BASE             0x11062000
#define I2C1_REG_BASE             0x11061000
#define I2C0_REG_BASE             0x11060000

#define I2C0_TX_FIFO   (I2C0_REG_BASE + 0x20)
#define I2C0_RX_FIFO   (I2C0_REG_BASE + 0x24)

#define I2C1_TX_FIFO   (I2C1_REG_BASE + 0x20)
#define I2C1_RX_FIFO   (I2C1_REG_BASE + 0x24)

#define I2C2_TX_FIFO   (I2C2_REG_BASE + 0x20)
#define I2C2_RX_FIFO   (I2C2_REG_BASE + 0x24)

#define I2C3_TX_FIFO   (I2C3_REG_BASE + 0x20)
#define I2C3_RX_FIFO   (I2C3_REG_BASE + 0x24)

#define I2C4_TX_FIFO   (I2C4_REG_BASE + 0x20)
#define I2C4_RX_FIFO   (I2C4_REG_BASE + 0x24)

#define I2C5_TX_FIFO   (I2C5_REG_BASE + 0x20)
#define I2C5_RX_FIFO   (I2C5_REG_BASE + 0x24)

#define I2C6_TX_FIFO   (I2C6_REG_BASE + 0x20)
#define I2C6_RX_FIFO   (I2C6_REG_BASE + 0x24)

#define I2C7_TX_FIFO   (I2C7_REG_BASE + 0x20)
#define I2C7_RX_FIFO   (I2C7_REG_BASE + 0x24)

#define I2C8_TX_FIFO   (I2C8_REG_BASE + 0x20)
#define I2C8_RX_FIFO   (I2C8_REG_BASE + 0x24)

#define I2C9_TX_FIFO   (I2C9_REG_BASE + 0x20)
#define I2C9_RX_FIFO   (I2C9_REG_BASE + 0x24)

#define I2C10_TX_FIFO   (I2C10_REG_BASE + 0x20)
#define I2C10_RX_FIFO   (I2C10_REG_BASE + 0x24)

#define I2C11_TX_FIFO   (I2C11_REG_BASE + 0x20)
#define I2C11_RX_FIFO   (I2C11_REG_BASE + 0x24)


#define EDMAC_TX 1
#define EDMAC_RX 0

edmac_peripheral  g_peripheral[EDMAC_MAX_PERIPHERALS] = {
	{0, I2C0_RX_FIFO, DMAC_HOST1, (0x40000004), PERI_8BIT_MODE, 0},
	{1, I2C0_TX_FIFO, DMAC_HOST1, (0x80000004), PERI_8BIT_MODE, 0},
	{2, I2C1_RX_FIFO, DMAC_HOST1, (0x40000004), PERI_8BIT_MODE, 0},
	{3, I2C1_TX_FIFO, DMAC_HOST1, (0x80000004), PERI_8BIT_MODE, 0},
	{4, I2C2_RX_FIFO, DMAC_HOST1, (0x40000004), PERI_8BIT_MODE, 0},
	{5, I2C2_TX_FIFO, DMAC_HOST1, (0x80000004), PERI_8BIT_MODE, 0},
	{6, I2C3_RX_FIFO, DMAC_HOST1, (0x40000004), PERI_8BIT_MODE, 0},
	{7, I2C3_TX_FIFO, DMAC_HOST1, (0x80000004), PERI_8BIT_MODE, 0},
	{8, I2C4_RX_FIFO, DMAC_HOST1, (0x40000004), PERI_8BIT_MODE, 0},
	{9, I2C4_TX_FIFO, DMAC_HOST1, (0x80000004), PERI_8BIT_MODE, 0},
	{10, I2C5_RX_FIFO, DMAC_HOST1, (0x40000004), PERI_8BIT_MODE, 0},
	{11, I2C5_TX_FIFO, DMAC_HOST1, (0x80000004), PERI_8BIT_MODE, 0},
};
#endif
