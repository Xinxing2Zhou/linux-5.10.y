/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <asm/setup.h>
#include <linux/mtd/nand.h>
#include <linux/mfd/bsp_fmc.h>
#include <linux/uaccess.h>

#include "../raw/nfc_gen.h"
#include "fmc100_nand.h"
#include <linux/securec.h>

#define _768K           (_256K + _512K)


struct nand_flash_special_dev {
	unsigned char id[8];
	int length;             /* length of id. */
	unsigned long long chipsize;
	struct nand_flash_dev *(*probe)(unsigned char *id);
	char *name;

	unsigned long pagesize;
	unsigned long erasesize;
	unsigned long oobsize;
	unsigned long options;
	unsigned int read_retry_type;

#define BBP_LAST_PAGE                    0x01
#define BBP_FIRST_PAGE                   0x02
	unsigned int badblock_pos;
	int flags;
};


/*                    this is nand probe function.                           */


static struct nand_flash_dev *hynix_probe_v02(unsigned char *id)
{
	struct nand_flash_dev *type = &g_nand_dev.flash_dev;

	int pagesizes[]   = {_2K, _4K, _8K, 0};
	int oobsizes[]    = {128, 224, 448, 0, 0, 0, 0, 0};
	int blocksizes[]  = {_128K, _256K, _512K, _768K, _1M, _2M, 0, 0};

	int blocktype = (((id[3] >> 5) & 0x04) | ((id[3] >> 4) & 0x03));
	int oobtype   = (((id[3] >> 2) & 0x03) | ((id[3] >> 4) & 0x04));

	type->options   = 0;
	type->pagesize  = pagesizes[(id[3] & 0x03)]; /* 0x3: check bit[0] and bit[1] */
	type->erasesize = blocksizes[blocktype];
	type->oobsize = oobsizes[oobtype];

	return type;
}


static struct nand_flash_dev *samsung_probe_v02(unsigned char *id)
{
	struct nand_flash_dev *type = &g_nand_dev.flash_dev;

	int pagesizes[]   = {_2K, _4K, _8K, 0};
	int oobsizes[]    = {0, 128, 218, 400, 436, 0, 0, 0};
	int blocksizes[]  = {_128K, _256K, _512K, _1M, 0, 0, 0, 0};

	int blocktype = (((id[3] >> 5) & 0x04) | ((id[3] >> 4) & 0x03));
	int oobtype   = (((id[3] >> 4) & 0x04) | ((id[3] >> 2) & 0x03));

	type->options   = 0;
	type->pagesize  = pagesizes[(id[3] & 0x03)]; /* 0x3: check bit[0] and bit[1] */
	type->erasesize = blocksizes[blocktype];
	type->oobsize = oobsizes[oobtype];

	return type;
}

#define DRV_VERSION     "1.40"

/******************************************************************************
 * We do not guarantee the compatibility of the following device models in the
 * table.Device compatibility is based solely on the list of compatible devices
 * in the release package.
 ******************************************************************************/

static struct nand_flash_special_dev nand_flash_special_table[] = {

	/************************* 1.8V MXIC Macronix **************************/
	{       /* SLC 4bit/512 1.8V */
		.name      = "MX30UF2G18AC",
		.id        = {0xC2, 0xAA, 0x90, 0x15, 0x06},
		/* id length(unit:byte) */
		.length    = 5, /* 5 Bytes */
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		/* oobsize(unit:byte) */
		.oobsize   = 64, /* 64 Bytes */
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},

	/****************************** Spansion *******************************/

	{      /* SLC S34ML02G200TFI000 */
		.name      = "S34ML02G200TFI000",
		.id        = {0x01, 0xDA, 0x90, 0x95, 0x46, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},

	{      /* SLC S34ML04G200TFI000 */
		.name      = "S34ML04G200TFI000",
		.id        = {0x01, 0xDC, 0x90, 0x95, 0x56, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _512M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},

	{      /* SLC S34MS02G200TFI00 1.8V */
		.name      = "S34MS02G200TFI00",
		.id        = {0x01, 0xAA, 0x90, 0x15, 0x46, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},

	{      /* SLC S34MS04G200TFI00 1.8V */
		.name      = "S34MS04G200TFI00",
		.id        = {0x01, 0xAC, 0x90, 0x15, 0x56, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _512M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},

	/****************************** Micron *******************************/

	{        /* MLC 40bit/1k */
		.name      = "MT29F64G08CBABA",
		.id        = {0x2C, 0x64, 0x44, 0x4B, 0xA9, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _8G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _2M,
		.oobsize   = 744,
		.options   = 0,
		.read_retry_type = NAND_RR_MICRON,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = NAND_RANDOMIZER | NAND_CHIP_MICRON,
	},
	{        /* MLC 40bit/1k */
		.name      = "MT29F32G08CBADA",
		.id        = {0x2C, 0x44, 0x44, 0x4B, 0xA9, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _2M,
		.oobsize   = 744,
		.options   = 0,
		.read_retry_type = NAND_RR_MICRON,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{        /* SLC 4bit/512 */
		.name      = "MT29F8G08ABxBA",
		.id        = {0x2C, 0x38, 0x00, 0x26, 0x85, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _1G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _512K,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* MLC 12bit/512 */
		.name      = "MT29F16G08CBABx",
		.id        = {0x2C, 0x48, 0x04, 0x46, 0x85, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _2G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _1M,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* MLC 24bit/1k */
		.name      = "MT29F16G08CBACA",
		.id        = {0x2C, 0x48, 0x04, 0x4A, 0xA5, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _2G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _1M,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* MLC 24bit/1k */
		.name      = "MT29F32G08CBACA",
		.id        = {0x2C, 0x68, 0x04, 0x4A, 0xA9, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _1M,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* MLC 24bit/1k */
		.name      = "MT29F64G08CxxAA",
		.id        = {0x2C, 0x88, 0x04, 0x4B, 0xA9, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _8G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _2M,
		.oobsize   = 448,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{        /* MLC 24bit/1k 2CE */
		.name      = "MT29F256G08CJAAA",
		.id        = {0x2C, 0xA8, 0x05, 0xCB, 0xA9, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _16G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _2M,
		.oobsize   = 448,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{        /* MLC 40bit/1k */
		.name      = "MT29F256G08CMCBB",
		.id        = {0x2C, 0x64, 0x44, 0x4B, 0xA9, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _8G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _2M,
		.oobsize   = 744,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* SLC 8bit/512 */
		.name      = "MT29F8G08ABACA",
		.id        = {0x2C, 0xD3, 0x90, 0xA6, 0x64, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _1G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _256K,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* SLC 8bit/512 */
		.name      = "MT29F4G08ABAEA",
		.id        = {0x2C, 0xDC, 0x90, 0xA6, 0x54, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _512M,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _256K,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* SLC 8bit/512 */
		.name      = "MT29F2G08ABAFA",
		.id        = {0x2C, 0xDA, 0x90, 0x95, 0x04, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{      /* SLC MT29F2G08ABAEA */
		.name      = "MT29F2G08ABAEA",
		.id        = {0x2C, 0xDA, 0x90, 0x95, 0x06, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 64,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* SLC 8bit/512 */
		.name      = "MT29F16G08ABACA",
		.id        = {0x2C, 0x48, 0x00, 0x26, 0xA9, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _2G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _512K,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},

	/****************************** Toshaba *******************************/

	{       /* MLC 24bit/1k 32nm */
		.name      = "TC58NVG4D2FTA00",
		.id        = {0x98, 0xD5, 0x94, 0x32, 0x76, 0x55, 0x00, 0x00},
		.length    = 6,
		.chipsize  = _2G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _1M,
		.oobsize   = 448,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC 24bit/1k 32nm 2CE */
		.name      = "TH58NVG6D2FTA20",
		.id        = {0x98, 0xD7, 0x94, 0x32, 0x76, 0x55, 0x00, 0x00},
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _1M,
		.oobsize   = 448,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC 40bit/1k 24nm */
		.name      = "TC58NVG5D2HTA00 24nm",
		.id        = {0x98, 0xD7, 0x94, 0x32, 0x76, 0x56, 0x08, 0x00},
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _1M,
		.oobsize   = 640,
		.options   = 0,
		.read_retry_type = NAND_RR_TOSHIBA_24nm,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{       /* MLC 40bit/1k */
		.name      = "TC58NVG6D2GTA00",
		.id        = {0x98, 0xDE, 0x94, 0x82, 0x76, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _8G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _2M,
		.oobsize   = 640,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC 19nm */
		.name      = "TC58NVG6DCJTA00 19nm",
		.id        = {0x98, 0xDE, 0x84, 0x93, 0x72, 0x57, 0x08, 0x04},
		.length    = 8,
		.chipsize  = _8G,
		.probe     = NULL,
		.pagesize  = _16K,
		.erasesize = _4M,
		.oobsize   = 1280,
		.options   = 0,
		.read_retry_type = NAND_RR_TOSHIBA_24nm,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{       /* MLC 19nm */
		.name      = "TC58TEG5DCJTA00 19nm",
		.id        = {0x98, 0xD7, 0x84, 0x93, 0x72, 0x57, 0x08, 0x04},
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _16K,
		.erasesize = _4M,
		.oobsize   = 1280,
		.options   = 0,
		.read_retry_type = NAND_RR_TOSHIBA_24nm,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER | NAND_CHIP_TOSHIBA_TOGGLE_10,
	},
	{       /* SLC 8bit/512 */
		.name      = "TC58NVG0S3HTA00",
		.id        = {0x98, 0xF1, 0x80, 0x15, 0x72, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _128M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		/*
		 * Datasheet: read one column of any page in each block. If the
		 * data of the column is 00 (Hex), define the block as a bad
		 * block.
		 */
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* SLC 8bit/512 */
		.name      = "TC58NVG1S3HTA00",
		.id        = {0x98, 0xDA, 0x90, 0x15, 0x76, 0x16, 0x08, 0x00},
		.length    = 7,
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* SLC 4bit/512 */
		.name      = "TC58NVG1S3ETA00",
		.id        = {0x98, 0xDA, 0x90, 0x15, 0x76, 0x14, 0x03, 0x00},
		.length    = 7,
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 64,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* SLC 4bit/512 */
		.name      = "TC58NVG3S0FTA00",
		.id        = {0x98, 0xD3, 0x90, 0x26, 0x76, 0x15, 0x02, 0x08},
		.length    = 8,
		.chipsize  = _1G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _256K,
		.oobsize   = 232,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* SLC 24bit/1k */
		.name      = "TC58NVG3S0HTA00",
		.id        = {0x98, 0xD3, 0x91, 0x26, 0x76, 0x16, 0x08, 0x00},
		.length    = 8,
		.chipsize  = _1G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _256K,
		.oobsize   = 256,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* SLC 24bit/1k */
		.name      = "TC58NVG2S0HTA00",
		.id        = {0x98, 0xDC, 0x90, 0x26, 0x76, 0x16, 0x08, 0x00},
		.length    = 8,
		.chipsize  = _512M,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _256K,
		.oobsize   = 256,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* SLC 4bit/512 */
		.name      = "TC58NVG2S0FTA00",
		.id        = {0x98, 0xDC, 0x90, 0x26, 0x76, 0x15, 0x01, 0x08},
		.length    = 8,
		.chipsize  = _512M,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _256K,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* SLC 4bit/512 */
		.name      = "TH58NVG2S3HTA00",
		.id        = {0x98, 0xDC, 0x91, 0x15, 0x76},
		.length    = 5,
		.chipsize  = _512M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* TLC 60bit/1k 19nm */
		.name      = "TC58NVG5T2JTA00 19nm TLC",
		/* datasheet says 6 ids id data, but really has 8 ids. */
		.id        = {0x98, 0xD7, 0x98, 0x92, 0x72, 0x57, 0x08, 0x10},
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _4M,
		.oobsize   = 1024,
		.options   = 0,
		.read_retry_type = NAND_RR_TOSHIBA_24nm,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{ /* TLC 60bit/1k 19nm */
		.name      = "TC58TEG5DCKTAx0 19nm MLC",
		/* datasheet says 6 ids id data, but really has 8 ids. */
		.id    = {0x98, 0xD7, 0x84, 0x93, 0x72, 0x50, 0x08, 0x04},
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _16K,
		.erasesize = _4M,
		.oobsize   = 1280,
		.options   = 0,
		.read_retry_type = NAND_RR_TOSHIBA_19nm,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{
		.name      = "Tx58TEGxDDKTAx0 19nm MLC",
		.id    = {0x98, 0xDE, 0x94, 0x93, 0x76, 0x50},
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _16K,
		.erasesize = _4M,
		.oobsize   = 1280,
		.options   = 0,
		.read_retry_type = NAND_RR_TOSHIBA_19nm,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	/******************************* Samsung ******************************/
	{       /* MLC 8bit/512B */
		.name     = "K9LB(HC/PD/MD)G08U0(1)D",
		.id       = {0xEC, 0xD7, 0xD5, 0x29, 0x38, 0x41, 0x00, 0x00},
		.length   = 6,
		.chipsize = _4G,
		.probe    = samsung_probe_v02,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC 24bit/1KB */
		.name      = "K9GAG08U0E",
		.id        = {0xEC, 0xD5, 0x84, 0x72, 0x50, 0x42, 0x00, 0x00},
		.length    = 6,
		.chipsize  = _2G,
		.probe     = samsung_probe_v02,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC 24bit/1KB */
		.name     = "K9LBG08U0E",
		.id       = {0xEC, 0xD7, 0xC5, 0x72, 0x54, 0x42, 0x00, 0x00},
		.length   = 6,
		.chipsize = _4G,
		.probe    = samsung_probe_v02,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC 24bit/1KB */
		.name     = "K9G8G08U0C",
		.id       = {0xEC, 0xD3, 0x84, 0x72, 0x50, 0x42, 0x00, 0x00},
		.length   = 6,
		.chipsize = _1G,
		.probe    = samsung_probe_v02,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{        /* MLC 24bit/1k */
		.name      = "K9GAG08U0F",
		.id        = {0xEC, 0xD5, 0x94, 0x76, 0x54, 0x43, 0x00, 0x00},
		.length    = 6,
		.chipsize  = _2G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _1M,
		.oobsize   = 512,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{        /* MLC */
		.name      = "K9LBG08U0M",
		.id        = {0xEC, 0xD7, 0x55, 0xB6, 0x78, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _512K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_LAST_PAGE,
		.flags = 0,
	},
	{        /* MLC 24bit/1k */
		.name      = "K9GBG08U0A 20nm",
		.id        = {0xEC, 0xD7, 0x94, 0x7A, 0x54, 0x43, 0x00, 0x00},
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _1M,
		.oobsize   = 640,
		.options   = 0,
		.read_retry_type = NAND_RR_SAMSUNG,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{        /* MLC 40bit/1k */
		.name      = "K9GBG08U0B",
		.id        = {0xEC, 0xD7, 0x94, 0x7E, 0x64, 0x44, 0x00, 0x00},
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _1M,
		.oobsize   = 1024,
		.options   = 0,
		.read_retry_type = NAND_RR_SAMSUNG,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},

	/*********************************** Hynix ****************************/
	{       /* MLC */
		.name     = "H27UAG8T2A",
		.id       = { 0xAD, 0xD5, 0x94, 0x25, 0x44, 0x41, },
		.length   = 6,
		.chipsize = _2G,
		.probe    = hynix_probe_v02,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC */
		.name     = "H27UAG8T2B",
		.id       = { 0xAD, 0xD5, 0x94, 0x9A, 0x74, 0x42, },
		.length   = 6,
		.chipsize = _2G,
		.probe    = hynix_probe_v02,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC */
		.name     = "H27UBG8T2A",
		.id       = { 0xAD, 0xD7, 0x94, 0x9A, 0x74, 0x42, },
		.length   = 6,
		.chipsize = _4G,
		.probe    = hynix_probe_v02,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{
		.name      = "H27UBG8T2BTR 26nm",
		.id        = { 0xAD, 0xD7, 0x94, 0xDA, 0x74, 0xC3, },
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _2M,
		.oobsize   = 640,
		.options   = 0,
		.read_retry_type = NAND_RR_HYNIX_BG_BDIE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{        /* MLC 40bit/1k */
		.name      = "H27UCG8T2A",
		.id        = { 0xAD, 0xDE, 0x94, 0xDA, 0x74, 0xC4, },
		.length    = 6,
		.chipsize  = _8G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _2M,
		.oobsize   = 640,
		.options   = 0,
		.read_retry_type = NAND_RR_HYNIX_CG_ADIE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{        /* MLC 40bit/1k */
		.name      = "H27UBG8T2C",
		.id        = { 0xAD, 0xD7, 0x94, 0x91, 0x60, 0x44, },
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = _8K,
		.erasesize = _2M,
		.oobsize   = 640,
		.options   = 0,
		.read_retry_type = NAND_RR_HYNIX_BG_CDIE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},

	/********************** MISC ******************************************/
	{        /* MLC 8bit/512 */
		.name      = "P1UAGA30AT-GCA",
		.id        = { 0xC8, 0xD5, 0x14, 0x29, 0x34, 0x01, },
		.length    = 6,
		.chipsize  = _2G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _512K,
		.oobsize   = 218,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC 4bit/512 */
		/*
		 * PowerFlash ASU8GA30IT-G30CA ID and MIRA PSU8GA30AT-GIA ID are
		 * the same ID
		 */
		.name      = "PSU8GA30AT-GIA/ASU8GA30IT-G30CA",
		.id        = { 0xC8, 0xD3, 0x90, 0x19, 0x34, 0x01, },
		.length    = 6,
		.chipsize  = _1G,
		.probe     = NULL,
		.pagesize  = _4K,
		.erasesize = _256K,
		.oobsize   = 218,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{        /* SLC 1bit/512 */
		.name      = "PSU2GA30AT",
		.id        = { 0x7F, 0x7F, 0x7F, 0x7F, 0xC8, 0xDA, 0x00, 0x15, },
		.length    = 8,
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = _2K,
		.erasesize = _128K,
		.oobsize   = 64,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{{0}, 0, 0, 0, 0, 0, 0, 0, 0},
};

struct nand_dev_t g_nand_dev;

struct nand_flash_dev *fmc_get_spl_flash_type(struct mtd_info *mtd, unsigned char *id)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_flash_special_dev *spl_dev = nand_flash_special_table;
	struct nand_flash_dev *type = &g_nand_dev.flash_dev;
	struct nand_dev_t *nand_dev = &g_nand_dev;
	int ret;

	fmc_pr(BT_DBG, "\t *-Start find special nand flash\n");

	pr_info("Nand ID: %#X %#X %#X %#X %#X %#X %#X %#X\n", id[0], id[1],
		id[2], id[3], id[4], id[5], id[6], id[7]);

	for (; spl_dev->length; spl_dev++) {
		if (memcmp(id, spl_dev->id, spl_dev->length))
			continue;

		fmc_pr(BT_DBG, "\t |-Found special Nand flash: %s\n",
		       spl_dev->name);

		if (spl_dev->probe) {
			type = spl_dev->probe(id);
		} else {
			type->options   = spl_dev->options;
			type->pagesize  = spl_dev->pagesize;
			type->erasesize = spl_dev->erasesize;
			type->oobsize = spl_dev->oobsize;
		}

		type->name = spl_dev->name;
		type->id_len = spl_dev->length;
		ret = memcpy_s(type->id, NAND_MAX_ID_LEN, id, type->id_len);
		if (ret) {
			printk("%s,memcpy_s failed\n", __func__);
			return NULL;
		}
		type->chipsize = (unsigned int)(spl_dev->chipsize >> 20); /* 1M unit need shift right 20 bit */
		fmc_pr(BT_DBG, "\t |-Save struct nand_flash_dev info\n");

		ret = memcpy_s(nand_dev->ids, NAND_MAX_ID_LEN, id, MAX_NAND_ID_LEN);
		if (ret) {
			printk("%s,mcmcpy_s failed\n", __func__);
			return NULL;
		}

		nand_dev->oobsize = type->oobsize;
		nand_dev->flags = spl_dev->flags;
		nand_dev->read_retry_type = spl_dev->read_retry_type;
		fmc_pr(BT_DBG, "\t |-Save struct nand_dev_t information\n");

		mtd->oobsize = spl_dev->oobsize;
		mtd->erasesize = spl_dev->erasesize;
		mtd->writesize = spl_dev->pagesize;
		mtd->size = spl_dev->chipsize;

		chip->base.memorg.pagesize = spl_dev->pagesize;
		chip->base.memorg.pages_per_eraseblock = spl_dev->erasesize / spl_dev->pagesize;
		chip->base.memorg.eraseblocks_per_lun = spl_dev->chipsize / spl_dev->erasesize;
		chip->base.memorg.oobsize = spl_dev->oobsize;

		return type;
	}
	nand_dev->read_retry_type = NAND_RR_NONE;

	chip->legacy.cmdfunc(chip, NAND_CMD_READID, 0x00, -1);
	chip->legacy.read_byte(chip);

	fmc_pr(BT_DBG, "\t *-Not found special nand flash\n");

	return NULL;
}


void fmc_spl_ids_register(void)
{
	pr_info("Special NAND id table Version %s\n", DRV_VERSION);
	get_spi_nand_flash_type_hook = fmc_get_spl_flash_type;
}
