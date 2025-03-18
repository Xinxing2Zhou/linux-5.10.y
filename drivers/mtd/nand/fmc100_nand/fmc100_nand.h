/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __FMC100_NAND_H__
#define __FMC100_NAND_H__

#include <linux/mfd/bsp_fmc.h>

/******************************************************************************/
/* These macroes are for debug only, reg option is slower then dma option */
#undef FMC100_NAND_SUPPORT_REG_READ
/* Open it as you need #define FMC100_NAND_SUPPORT_REG_READ */

#undef FMC100_NAND_SUPPORT_REG_WRITE
/* Open it as you need #define FMC100_NAND_SUPPORT_REG_WRITE */

/*****************************************************************************/
#define PAGE_PER_BLK_64			64
#define PAGE_PER_BLK_128		128
#define PAGE_PER_BLK_256		256
#define PAGE_PER_BLK_512		512

/*****************************************************************************/
#define OOB_LENGTH_DEFAULT		32
#define OOB_OFFSET_DEFAULT		32
#define OOB_LENGTH_DEFAULT_FREE		30
#define OOB_OFFSET_DEFAULT_FREE		2

#define OOB_LENGTH_4K16BIT		14
#define OOB_OFFSET_4K16BIT		14
#define OOB_LENGTH_4K16BIT_FREE		14
#define OOB_OFFSET_4K16BIT_FREE		2

#define OOB_LENGTH_2K16BIT		6
#define OOB_OFFSET_2K16BIT		6
#define OOB_LENGTH_2K16BIT_FREE		6
#define OOB_OFFSET_2K16BIT_FREE		2

/*****************************************************************************/
#define FMC100_ECC_ERR_NUM0_BUF0      0xc0
#define FMC100_ECC_ERR_NUM1_BUF0      0xc4
#define FMC100_ECC_ERR_NUM0_BUF1      0xc8
#define FMC100_ECC_ERR_NUM1_BUF1      0xcc

#define get_ecc_err_num(_i, _reg)       (((_reg) >> ((_i) * 8)) & 0xff)

#define ECC_STEP_SHIFT			10
#define ECC_STEP_MAX_4K_PAGE		4
#define ECC_STEP_MAX_8K_PAGE		8
#define ECC_STEP_MAX_16K_PAGE		8

/*****************************************************************************/
#define NAND_MAX_PAGESIZE           32768
#define NAND_MAX_OOBSIZE            4800

#define CONFIG_SUPPORT_YAFFS
#define FMC100_NAND_OOBSIZE_FOR_YAFFS     32

/*****************************************************************************/
#define REG_CNT_HIGH_BLOCK_NUM_SHIFT        10

#define REG_CNT_BLOCK_NUM_MASK          0x3ff
#define REG_CNT_BLOCK_NUM_SHIFT         22

#define REG_CNT_PAGE_NUM_MASK           0x3f
#define REG_CNT_PAGE_NUM_SHIFT          16

/*****************************************************************************/
#define WORD_READ_OFFSET_ADD_LENGTH	2
#define WORD_READ_START_OFFSET		2
/*****************************************************************************/
#define FMC100_ADDR_CYCLE_MASK        0x4
#define FMC100_ADDR_CYCLE_SHIFT       0x3
#define NAND_EDO_MODE_SHIFT            9
#define NAND_EDO_MODE_MASK             (1<<NAND_EDO_MODE_SHIFT)
#define set_nand_edo_mode_en(reg)   ((reg) | NAND_EDO_MODE_MASK)
/*****************************************************************************/
struct fmc_host {
	struct nand_chip *chip;
	struct mtd_info  *mtd;

	struct fmc_cmd_op cmd_op;
	void __iomem *regbase;
	void __iomem *iobase;

	/* Controller config option nand flash */
	unsigned int nand_cfg;
	unsigned int nand_cfg_ecc0;

	unsigned int offset;

	struct device *dev;

	/* This is maybe an un-aligment address, only for malloc or free */
	char *buforg;
	char *buffer;

#ifdef CONFIG_64BIT
	unsigned long long dma_buffer;
	unsigned long long dma_oob;
#else
	unsigned int dma_buffer;
	unsigned int dma_oob;
#endif
	unsigned int dma_len;

	unsigned int addr_cycle;
	unsigned int addr_value[2];
	unsigned int cache_addr_value[2];

	unsigned int column;
	unsigned int block_page_mask;

	unsigned int ecctype;
	unsigned int pagesize;
	unsigned int oobsize;

	int  need_rr_data;
#define FMC100_READ_RETRY_DATA_LEN         128
	char rr_data[FMC100_READ_RETRY_DATA_LEN];
	int  version;
	int   add_partition;

	/* BOOTROM read two bytes to detect the bad block flag */
#define FMC100_BAD_BLOCK_POS              0
	unsigned char *bbm;  /* nand bad block mark */
#define FMC_OOB_LEN_30_EB_OFFSET	(30 - 2)
#define FMC_OOB_LEN_6_EB_OFFSET		(6 - 2)
#define FMC_OOB_LEN_14_EB_OFFSET	(14 - 2)
	unsigned short *epm;  /* nand empty page mark */
	unsigned int flags;

#define FMC100_PS_UC_ECC        0x01 /* page has ecc error */
#define FMC100_PS_BAD_BLOCK     0x02 /* bad block */
#define FMC100_PS_EMPTY_PAGE    0x04 /* page is empty */
#define FMC100_PS_EPM_ERROR     0x0100 /* empty page mark word has error. */
#define FMC100_PS_BBM_ERROR     0x0200 /* bad block mark word has error. */
	unsigned int page_status;

	struct clk *clk;

	void (*send_cmd_pageprog)(struct fmc_host *host);
	void (*send_cmd_status)(struct fmc_host *host);
	void (*send_cmd_readstart)(struct fmc_host *host);
	void (*send_cmd_erase)(struct fmc_host *host);
	void (*send_cmd_readid)(struct fmc_host *host);
	void (*send_cmd_reset)(struct fmc_host *host);
	void (*enable)(int enable);

	void (*enable_ecc_randomizer)(struct fmc_host *host,
				      int ecc_en, int randomizer_en);

	void (*detect_ecc)(struct fmc_host *host);

	struct read_retry_t *read_retry;
};

extern struct nand_dev_t g_nand_dev;

int fmc100_nand_init(struct nand_chip *chip);

extern void fmc_spl_ids_register(void);

#ifdef CONFIG_PM
void fmc100_nand_config(const struct fmc_host *host);
#endif

#endif /* End of __FMC100_NAND_H__ */
