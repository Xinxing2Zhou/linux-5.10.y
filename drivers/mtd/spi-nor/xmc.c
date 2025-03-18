// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

static const struct flash_info xmc_parts[] = {
	/* XMC (Wuhan Xinxin Semiconductor Manufacturing Corp.) */


#ifdef CONFIG_ARCH_BSP
	{ "xm25qh64a",    INFO(0x207017, 0, 64 * 1024,   128, SPI_NOR_QUAD_READ)
			PARAMS(xmc), CLK_MHZ_2X(104) },
	{ "xm25qh64b",    INFO(0x206017, 0, 64 * 1024,   128, SPI_NOR_QUAD_READ)
			PARAMS(xmc), CLK_MHZ_2X(104) },
	{ "xm25qh128a",   INFO(0x207018, 0, 64 * 1024,   256, SPI_NOR_QUAD_READ)
			PARAMS(xmc), CLK_MHZ_2X(104) },
	{ "xm25qh128b",   INFO(0x206018, 0, 64 * 1024,   256, SPI_NOR_QUAD_READ)
			PARAMS(xmc), CLK_MHZ_2X(104) },
#else
	{ "XM25QH64A", INFO(0x207017, 0, 64 * 1024, 128,
			    SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "XM25QH128A", INFO(0x207018, 0, 64 * 1024, 256,
			     SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
#endif /* CONFIG_ARCH_BSP */
};

const struct spi_nor_manufacturer spi_nor_xmc = {
	.name = "xmc",
	.parts = xmc_parts,
	.nparts = ARRAY_SIZE(xmc_parts),
};
