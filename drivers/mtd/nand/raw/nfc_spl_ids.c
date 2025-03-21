/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/mtd/mtd.h>
#include <linux/mfd/bsp_fmc.h>
#include <linux/securec.h>
#include "nfc_gen.h"

struct nand_flash_special_dev {
	unsigned char id[8];
	int length;             /* length of id. */
	unsigned long long chipsize;
	struct nand_flash_dev *(*probe)(struct nand_dev_t *nand_dev);
	char *name;

	unsigned long pagesize;
	unsigned long erasesize;
	unsigned long oobsize;
	unsigned long options;
	unsigned int read_retry_type;

#define BBP_LAST_PAGE                    0x01
#define BBP_FIRST_PAGE                   0x02
	unsigned int badblock_pos;
	unsigned int flags;
};


/*                    this is nand probe function.                           */


static struct nand_flash_dev *hynix_probe_v02(
	struct nand_dev_t *nand_dev)
{
	unsigned char *id = nand_dev->ids;
	struct nand_flash_dev *type = &nand_dev->flash_dev;

	int pagesizes[]   = {SZ_2K, SZ_4K, SZ_8K, 0};
	int oobsizes[]    = {128, 224, 448, 0, 0, 0, 0, 0};
	int blocksizes[]  = {SZ_128K, SZ_256K, SZ_512K,
			     (SZ_256K + SZ_512K), SZ_1M, SZ_2M, 0, 0
			    };

	int blocktype = (((id[3] >> 5) & 0x04) | ((id[3] >> 4) & 0x03));
	int oobtype   = (((id[3] >> 2) & 0x03) | ((id[3] >> 4) & 0x04));

	type->options   = 0;
	type->pagesize  = pagesizes[(id[3] & 0x03)];
	type->erasesize = blocksizes[blocktype];
	nand_dev->oobsize = oobsizes[oobtype];

	return type;
}


static struct nand_flash_dev *samsung_probe_v02(
	struct nand_dev_t *nand_dev)
{
	unsigned char *id = nand_dev->ids;
	struct nand_flash_dev *type = &nand_dev->flash_dev;

	int pagesizes[]   = {SZ_2K, SZ_4K, SZ_8K, 0};
	int oobsizes[]    = {0, 128, 218, 400, 436, 0, 0, 0};
	int blocksizes[]  = {SZ_128K, SZ_256K, SZ_512K, SZ_1M, 0, 0, 0, 0};

	int blocktype = (((id[3] >> 5) & 0x04) | ((id[3] >> 4) & 0x03));
	int oobtype   = (((id[3] >> 4) & 0x04) | ((id[3] >> 2) & 0x03));

	type->options   = 0;
	type->pagesize  = pagesizes[(id[3] & 0x03)];
	type->erasesize = blocksizes[blocktype];
	nand_dev->oobsize = oobsizes[oobtype];

	return type;
}


#define DRV_VERSION     "1.38"


/*
 * samsung:  27nm need randomizer, 21nm need read retry;
 * micron:   25nm need read retry, datasheet will explain read retry.
 * toshaba   32nm need randomizer, 24nm need read retry.
 * hynix:    2xnm need read retry.
 *
 *		The special nand flash ID table version 1.37
 *
 * manufactory  |  type  |       name 	     |   ecc_type  | version_tag
 * Micron		|  MLC	 |  MT29F64G08CBABA  |   40bit/1k  |  1.36
 * Micron		|  MLC	 |  MT29F32G08CBADA  |   40bit/1k  |
 * Micron		|  SLC	 |  MT29F8G08ABxBA   |   4bit/512  |
 * Micron		|  MLC	 |  MT29F16G08CBABx  |   12bit/512 |
 * Micron		|  MLC	 |  MT29F16G08CBACA  |   24bit/1k  |
 * Micron		|  MLC	 |  MT29F32G08CBACA  |   24bit/1k  |
 * Micron		|  MLC	 |  MT29F64G08CxxAA  |   24bit/1k  |
 * Micron		|  MLC	 |  MT29F256G08CJAAA |   24bit/1k  |   2CE
 * Micron		|  MLC	 |  MT29F256G08CMCBB |   24bit/1k  |
 * Micron		|  SLC	 |  MT29F8G08ABACA   |   8bit/512  |
 * Micron		|  SLC	 |  MT29F4G08ABAEA   |   8bit/512  |
 * Micron		|  SLC	 |  MT29F2G08ABAFA   |   8bit/512  |
 * Micron		|  SLC	 |  MT29F16G08ABACA  |   8bit/512  |
 * Toshiba		|  MLC   |  TC58NVG4D2FTA00  |   24bit/1k  |
 * Toshiba		|  MLC   |  TH58NVG6D2FTA20  |   24bit/1k  |   2CE
 * Toshiba		|  MLC   |  TC58NVG5D2HTA00  |   40bit/1k  |
 * Toshiba		|  MLC   |  TC58NVG6D2GTA00  |   40bit/1k  |
 * Toshiba		|  MLC   |  TC58NVG6DCJTA00  |			   |
 * Toshiba		|  MLC   |  TC58TEG5DCJTA00  |			   |
 * Toshiba		|  SLC   |  TC58NVG0S3HTA00  |   8bit/512  |
 * Toshiba		|  SLC   |  TC58NVG1S3HTA00  |   8bit/512  |
 * Toshiba		|  SLC   |  TC58NVG1S3ETA00  |   4bit/512  |
 * Toshiba		|  SLC   |  TC58NVG3S0FTA00  |   4bit/512  |
 * Toshiba		|  SLC   |  TC58NVG2S0FTA00  |   4bit/512  |
 * Toshiba		|  SLC   |  TH58NVG2S3HTA00  |   4bit/512  |
 * Toshiba		|  TLC   |  TC58NVG5T2JTA00  |   60bit/1k  |
 * Toshiba		|  TLC   |  TC58TEG5DCKTAx0  |   60bit/1k  |
 * Toshiba		|  MLC   |  Tx58TEGxDDKTAx0  |			   |
 * Samsung		|  MLC   |  K9LB(HC/PD/MD)G08U0(1)D  |   8bit/512B  |
 * Samsung		|  MLC   |  K9GAG08U0E	     |   24bit/1KB |
 * Samsung		|  MLC   |  K9LBG08U0E	     |   24bit/1KB |
 * Samsung		|  MLC   |  K9G8G08U0C	     |   24bit/1KB |
 * Samsung		|  MLC   |  K9GAG08U0F	     |   24bit/1KB |
 * Samsung		|  MLC   |  K9LBG08U0M	     |             |
 * Samsung		|  MLC   |  K9GBG08U0A	     |   24bit/1KB |
 * Samsung		|  MLC   |  K9GBG08U0B	     |   40bit/1KB |
 * Hynix		|  MLC   |  H27UAG8T2A	     |			   |
 * Hynix		|  MLC   |  H27UAG8T2B	     |			   |
 * Hynix		|  MLC   |  H27UBG8T2A	     |			   |
 * Hynix		|  MLC   |  H27UBG8T2BTR	 |	 24bit/1KB |
 * Hynix		|  MLC   |  H27UCG8T2A		 |	 40bit/1KB |
 * Hynix		|  MLC   |  H27UBG8T2C		 |	 40bit/1KB |
 * MISC			|  MLC   |  P1UAGA30AT-GCA	 |	 8bit/512  |
 * MISC			|  MLC   |  PSU8GA30AT-GIA/ASU8GA30IT-G30CA	 |	 4bit/512  |
 * MISC			|  SLC   |  PSU2GA30AT   	 |	 1bit/512  |   1.36
 * Toshiba		|  SLC   |  TC58NVG2S0HTA00  |	 24bit/1K  |   1.37
 * Toshiba		|  SLC   |  TC58NVG3S0HTA00  |   24bit/1K  |   1.37
 * Micron		|  SLC	 |  MT29F2G08ABAEA   |   4bit/512 |
 * Spansion		|  SLC	 | S34ML02G200TFI000	 | 24bit/1K |
 * Spansion		|  SLC	 | S34ML04G200TFI000	 | 24bit/1K |  1.38
 *
 */

static struct nand_flash_special_dev nand_flash_special_dev[] = {

/****************************** Spansion *******************************/

	{ /* SLC S34ML02G200TFI000 */
		.name      = "S34ML02G200TFI000",
		.id        = {0x01, 0xDA, 0x90, 0x95, 0x46, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = SZ_2K,
		.erasesize = SZ_128K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},

	{ /* SLC S34ML04G200TFI000 */
		.name      = "S34ML04G200TFI000",
		.id        = {0x01, 0xDC, 0x90, 0x95, 0x56, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _512M,
		.probe     = NULL,
		.pagesize  = SZ_2K,
		.erasesize = SZ_128K,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_2M,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_2M,
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
		.chipsize  = SZ_1G,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_512K,
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
		.chipsize  = SZ_2G,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_1M,
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
		.chipsize  = SZ_2G,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_1M,
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
		.pagesize  = SZ_4K,
		.erasesize = SZ_1M,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_2M,
		.oobsize   = 448,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* MLC 24bit/1k 2CE */
		.name      = "MT29F256G08CJAAA",
		.id        = {0x2C, 0xA8, 0x05, 0xCB, 0xA9, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _16G,
		.probe     = NULL,
		.pagesize  = SZ_8K,
		.erasesize = SZ_2M,
		.oobsize   = 448,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{        /* MLC 40bit/1k */
		.name      = "MT29F256G08CMCBB",
		.id        = {0x2C, 0x64, 0x44, 0x4B, 0xA9, 0x00, 0x00, 0x00},
		.length    = 8,
		.chipsize  = _8G,
		.probe     = NULL,
		.pagesize  = SZ_8K,
		.erasesize = SZ_2M,
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
		.chipsize  = SZ_1G,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_256K,
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
		.chipsize  = SZ_512M,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_256K,
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
		.chipsize  = SZ_256M,
		.probe     = NULL,
		.pagesize  = SZ_2K,
		.erasesize = SZ_128K,
		.oobsize   = 224,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{ /* SLC MT29F2G08ABAEA */
		.name      = "MT29F2G08ABAEA",
		.id        = {0x2C, 0xDA, 0x90, 0x95, 0x06, 0x00, 0x00, 0x00},
		.length    = 5,
		.chipsize  = _256M,
		.probe     = NULL,
		.pagesize  = SZ_2K,
		.erasesize = SZ_128K,
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
		.chipsize  = SZ_2G,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_512K,
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
		.chipsize  = SZ_2G,
		.probe     = NULL,
		.pagesize  = SZ_8K,
		.erasesize = SZ_1M,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_1M,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_1M,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_2M,
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
		.pagesize  = SZ_16K,
		.erasesize = SZ_4M,
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
		.pagesize  = SZ_16K,
		.erasesize = SZ_4M,
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
		.chipsize  = SZ_128M,
		.probe     = NULL,
		.pagesize  = SZ_2K,
		.erasesize = SZ_128K,
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
		.chipsize  = SZ_256M,
		.probe     = NULL,
		.pagesize  = SZ_2K,
		.erasesize = SZ_128K,
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
		.chipsize  = SZ_256M,
		.probe     = NULL,
		.pagesize  = SZ_2K,
		.erasesize = SZ_128K,
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
		.chipsize  = SZ_1G,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_256K,
		.oobsize   = 232,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* SLC 4bit/512 */
		.name      = "TC58NVG3S0HTA00",
		.id        = {0x98, 0xD3, 0x91, 0x26, 0x76, 0x16, 0x08, 0x00},
		.length    = 8,
		.chipsize  = SZ_1G,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_256K,
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
		.chipsize  = SZ_512M,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_256K,
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
		.chipsize  = SZ_512M,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_256K,
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
		.chipsize  = SZ_512M,
		.probe     = NULL,
		.pagesize  = SZ_2K,
		.erasesize = SZ_128K,
		.oobsize   = 128,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE,
		.flags = 0,
	},
	{       /* TLC 60bit/1k 19nm */
		.name      = "TC58NVG5T2JTA00 19nm TLC",
		.id        = {0x98, 0xD7, 0x98, 0x92, 0x72, 0x57, 0x08, 0x10},
		.length    = 6,
		.chipsize  = _4G,
		.probe     = NULL,
		.pagesize  = SZ_8K,
		.erasesize = SZ_4M,
		.oobsize   = 1024,
		.options   = 0,
		.read_retry_type = NAND_RR_TOSHIBA_24nm,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{ /* TLC 60bit/1k 19nm */
		.name	   = "TC58TEG5DCKTAx0 19nm MLC",
		/* datasheet says 6 ids id data, but really has 8 ids. */
		.id	   = {0x98, 0xD7, 0x84, 0x93, 0x72, 0x50, 0x08, 0x04},
		.length    = 6,
		.chipsize  = _4G,
		.probe	   = NULL,
		.pagesize  = SZ_16K,
		.erasesize = SZ_4M,
		.oobsize   = 1280,
		.options   = 0,
		.read_retry_type = NAND_RR_TOSHIBA_19nm,
		.badblock_pos	 = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = NAND_RANDOMIZER,
	},
	{
		.name	   = "Tx58TEGxDDKTAx0 19nm MLC",
		.id	   = {0x98, 0xDE, 0x94, 0x93, 0x76, 0x50},
		.length    = 6,
		.chipsize  = _4G,
		.probe	   = NULL,
		.pagesize  = SZ_16K,
		.erasesize = SZ_4M,
		.oobsize   = 1280,
		.options   = 0,
		.read_retry_type = NAND_RR_TOSHIBA_19nm,
		.badblock_pos	 = BBP_FIRST_PAGE | BBP_LAST_PAGE,
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
		.chipsize  = SZ_2G,
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
		.chipsize = SZ_1G,
		.probe    = samsung_probe_v02,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{        /* MLC 24bit/1k */
		.name      = "K9GAG08U0F",
		.id        = {0xEC, 0xD5, 0x94, 0x76, 0x54, 0x43, 0x00, 0x00},
		.length    = 6,
		.chipsize  = SZ_2G,
		.probe     = NULL,
		.pagesize  = SZ_8K,
		.erasesize = SZ_1M,
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
		.pagesize  = SZ_4K,
		.erasesize = SZ_512K,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_1M,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_1M,
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
		.chipsize = SZ_2G,
		.probe    = hynix_probe_v02,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{       /* MLC */
		.name     = "H27UAG8T2B",
		.id       = { 0xAD, 0xD5, 0x94, 0x9A, 0x74, 0x42, },
		.length   = 6,
		.chipsize = SZ_2G,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_2M,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_2M,
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
		.pagesize  = SZ_8K,
		.erasesize = SZ_2M,
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
		.chipsize  = SZ_2G,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_512K,
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
		.chipsize  = SZ_1G,
		.probe     = NULL,
		.pagesize  = SZ_4K,
		.erasesize = SZ_256K,
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
		.chipsize  = SZ_256M,
		.probe     = NULL,
		.pagesize  = SZ_2K,
		.erasesize = SZ_128K,
		.oobsize   = 64,
		.options   = 0,
		.read_retry_type = NAND_RR_NONE,
		.badblock_pos    = BBP_FIRST_PAGE | BBP_LAST_PAGE,
		.flags = 0,
	},
	{{0}, 0, 0, 0, 0, 0, 0, 0, 0},
};

#define NUM_OF_SPECIAL_DEVICE  \
	(sizeof(nand_flash_special_dev) / sizeof(struct nand_flash_special_dev))

int (*nfc_param_adjust)(struct mtd_info *, struct nand_chip *,
			  struct nand_dev_t *) = NULL;
EXPORT_SYMBOL_GPL(nfc_param_adjust);

static struct nand_dev_t __nand_dev;


static struct nand_flash_dev *nfc_nand_probe(struct mtd_info *mtd,
					       struct nand_chip *chip,
					       struct nand_dev_t *nand_dev)
{
	struct nand_flash_special_dev *spl_dev = NULL;
	unsigned char *byte = nand_dev->ids;
	struct nand_flash_dev *type = &nand_dev->flash_dev;

	nfc_pr_msg("Nand ID: 0x%02X 0x%02X 0x%02X 0x%02X",
		     byte[0], byte[1], byte[2], byte[3]);
	nfc_pr_msg(" 0x%02X 0x%02X 0x%02X 0x%02X\n",
		     byte[4], byte[5], byte[6], byte[7]);

	for (spl_dev = nand_flash_special_dev; spl_dev->length; spl_dev++) {
		if (memcmp(byte, spl_dev->id, spl_dev->length))
			continue;

		nfc_pr_msg("The Special NAND id table Version: %s\n", DRV_VERSION);

		if (spl_dev->probe) {
			type = spl_dev->probe(nand_dev);
		} else {
			type->options   = spl_dev->options;
			type->pagesize  = spl_dev->pagesize;
			type->erasesize = spl_dev->erasesize;
			nand_dev->oobsize = spl_dev->oobsize;
		}

		nand_dev->read_retry_type = spl_dev->read_retry_type;
		nand_dev->flags = spl_dev->flags;

		type->id[1] = byte[1];
		type->chipsize = (unsigned long)(spl_dev->chipsize >> 20);
		type->name = spl_dev->name;
		return type;
	}
	nand_dev->read_retry_type = NAND_RR_NONE;

	return NULL;
}


struct nand_flash_dev *nfc_get_flash_type(struct mtd_info *mtd,
					    struct nand_chip *chip,
					    u8 *id_data, int *busw)
{
	struct nand_flash_dev *type = NULL;
	struct nand_dev_t *nand_dev = &__nand_dev;
	int ret;

	(void)memset_s(nand_dev, sizeof(struct nand_dev_t), 0,
		sizeof(struct nand_dev_t));
	ret = memcpy_s(nand_dev->ids, 8, id_data, 8);
	if (ret) {
		printk("%s:memcpy_s failed!\n", __func__);
		return NULL;
	}

	if (!nfc_nand_probe(mtd, chip, nand_dev))
		return NULL;

	type = &nand_dev->flash_dev;

	if (!mtd->name)
		mtd->name = type->name;

	mtd->erasesize = type->erasesize;
	mtd->writesize = type->pagesize;
	mtd->oobsize   = nand_dev->oobsize;
	*busw = (type->options & NAND_BUSWIDTH_16);

	return type;
}


void nfc_nand_param_adjust(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct nand_dev_t *nand_dev = &__nand_dev;

	if (!nand_dev->oobsize)
		nand_dev->oobsize = mtd->oobsize;

	if (nfc_param_adjust)
		nfc_param_adjust(mtd, chip, nand_dev);
}


void nfc_show_info(struct mtd_info *mtd, const char *vendor, char *chipname)
{
	struct nand_dev_t *nand_dev = &__nand_dev;

	if (IS_NAND_RANDOM(nand_dev))
		nfc_pr_msg("Randomizer \n");

	if (nand_dev->read_retry_type != NAND_RR_NONE)
		nfc_pr_msg("Read-Retry \n");

	if (nand_dev->start_type)
		nfc_pr_msg("Nand(%s): ", nand_dev->start_type);
	else
		nfc_pr_msg("Nand: ");

	nfc_pr_msg("OOB:%dB ", nand_dev->oobsize);
	nfc_pr_msg("ECC:%s ", nand_ecc_name(nand_dev->ecctype));
}


void nfc_show_chipsize(struct nand_chip *chip)
{
}
