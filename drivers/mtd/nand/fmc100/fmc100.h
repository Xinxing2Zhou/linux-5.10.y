/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __FMC100_H__
#define __FMC100_H__

#include <linux/platform_device.h>
#include <linux/mfd/bsp_fmc.h>
#include <linux/mtd/rawnand.h>
#include "../raw/nfc_gen.h"

#define INFINITE	    (0xFFFFFFFF)

#define SPI_IF_READ_STD	 (0x01)
#define SPI_IF_READ_FAST	(0x02)
#define SPI_IF_READ_DUAL	(0x04)
#define SPI_IF_READ_DUAL_ADDR       (0x08)
#define SPI_IF_READ_QUAD	(0x10)
#define SPI_IF_READ_QUAD_ADDR       (0x20)

#define SPI_IF_WRITE_STD	(0x01)
#define SPI_IF_WRITE_DUAL       (0x02)
#define SPI_IF_WRITE_DUAL_ADDR      (0x04)
#define SPI_IF_WRITE_QUAD       (0x08)
#define SPI_IF_WRITE_QUAD_ADDR      (0x10)

#define SPI_IF_ERASE_SECTOR_4K      (0x01)
#define SPI_IF_ERASE_SECTOR_32K     (0x02)
#define SPI_IF_ERASE_SECTOR_64K     (0x04)
#define SPI_IF_ERASE_SECTOR_128K    (0x08)
#define SPI_IF_ERASE_SECTOR_256K    (0x10)

#define FMC_SPI_NAND_SUPPORT_READ (SPI_IF_READ_STD | \
	SPI_IF_READ_FAST | \
	SPI_IF_READ_DUAL | \
	SPI_IF_READ_DUAL_ADDR | \
	SPI_IF_READ_QUAD | \
	SPI_IF_READ_QUAD_ADDR)

#define FMC_SPI_NAND_SUPPORT_WRITE    (SPI_IF_WRITE_STD | SPI_IF_WRITE_QUAD)

#define FMC_SPI_NAND_SUPPORT_MAX_DUMMY    8

#define SPI_CMD_READ_STD	0x03    /* Standard read cache */
#define SPI_CMD_READ_FAST       0x0B    /* Higher speed read cache */
#define SPI_CMD_READ_DUAL       0x3B    /* 2 IO read cache only date */
#define SPI_CMD_READ_DUAL_ADDR      0xBB    /* 2 IO read cache date&addr */
#define SPI_CMD_READ_QUAD       0x6B    /* 4 IO read cache only date */
#define SPI_CMD_READ_QUAD_ADDR      0xEB    /* 4 IO read cache date&addr */

#define SPI_CMD_WRITE_STD       0x02    /* Standard page program */
#define SPI_CMD_WRITE_DUAL      0xA2    /* 2 IO program only date */
#define SPI_CMD_WRITE_DUAL_ADDR     0xD2    /* 2 IO program date&addr */
#define SPI_CMD_WRITE_QUAD      0x32    /* 4 IO program only date */
#define SPI_CMD_WRITE_QUAD_ADDR     0x12    /* 4 IO program date&addr */

#define SPI_CMD_SE_4K	   0x20    /* 4KB sector Erase */
#define SPI_CMD_SE_32K	  0x52    /* 32KB sector Erase */
#define SPI_CMD_SE_64K	  0xD8    /* 64KB sector Erase */
#define SPI_CMD_SE_128K	 0xD8    /* 128KB sector Erase */
#define SPI_CMD_SE_256K	 0xD8    /* 256KB sector Erase */

#define set_read_std(_dummy_, _size_, _clk_) \
	static struct spi_op read_std_##_dummy_##_size_##_clk_ = { \
	SPI_IF_READ_STD, SPI_CMD_READ_STD, _dummy_, _size_, _clk_ }

#define set_read_fast(_dummy_, _size_, _clk_) \
	static struct spi_op read_fast_##_dummy_##_size_##_clk_ = { \
	SPI_IF_READ_FAST, SPI_CMD_READ_FAST, _dummy_, _size_, _clk_ }

#define set_read_dual(_dummy_, _size_, _clk_) \
	static struct spi_op read_dual_##_dummy_##_size_##_clk_ = { \
	SPI_IF_READ_DUAL, SPI_CMD_READ_DUAL, _dummy_, _size_, _clk_ }

#define set_read_dual_addr(_dummy_, _size_, _clk_) \
	static struct spi_op read_dual_addr_##_dummy_##_size_##_clk_ = { \
	SPI_IF_READ_DUAL_ADDR, SPI_CMD_READ_DUAL_ADDR, _dummy_, _size_, _clk_ }

#define set_read_quad(_dummy_, _size_, _clk_) \
	static struct spi_op read_quad_##_dummy_##_size_##_clk_ = { \
	SPI_IF_READ_QUAD, SPI_CMD_READ_QUAD, _dummy_, _size_, _clk_ }

#define set_read_quad_addr(_dummy_, _size_, _clk_) \
	static struct spi_op read_quad_addr_##_dummy_##_size_##_clk_ = { \
	SPI_IF_READ_QUAD_ADDR, SPI_CMD_READ_QUAD_ADDR, _dummy_, _size_, _clk_ }


#define set_write_std(_dummy_, _size_, _clk_) \
	static struct spi_op write_std_##_dummy_##_size_##_clk_ = { \
	SPI_IF_WRITE_STD, SPI_CMD_WRITE_STD, _dummy_, _size_, _clk_ }

#define set_write_dual(_dummy_, _size_, _clk_) \
	static struct spi_op write_dual_##_dummy_##_size_##_clk_ = { \
	SPI_IF_WRITE_DUAL, SPI_CMD_WRITE_DUAL, _dummy_, _size_, _clk_ }

#define set_write_dual_addr(_dummy_, _size_, _clk_) \
	static struct spi_op write_dual_addr_##_dummy_##_size_##_clk_ = { \
	SPI_IF_WRITE_DUAL_ADDR, SPI_CMD_WRITE_DUAL_ADDR, _dummy_, _size_, _clk_ }

#define set_write_quad(_dummy_, _size_, _clk_) \
	static struct spi_op write_quad_##_dummy_##_size_##_clk_ = { \
	SPI_IF_WRITE_QUAD, SPI_CMD_WRITE_QUAD, _dummy_, _size_, _clk_ }

#define set_write_quad_addr(_dummy_, _size_, _clk_) \
	static struct spi_op write_quad_addr_##_dummy_##_size_##_clk_ = { \
	SPI_IF_WRITE_QUAD_ADDR, SPI_CMD_WRITE_QUAD_ADDR, _dummy_, _size_, _clk_ }

#define set_erase_sector_4k(_dummy_, _size_, _clk_) \
	static struct spi_op erase_sector_4k_##_dummy_##_size_##_clk_ = { \
	SPI_IF_ERASE_SECTOR_4K, SPI_CMD_SE_4K, _dummy_, _size_, _clk_ }

#define set_erase_sector_32k(_dummy_, _size_, _clk_) \
	static struct spi_op erase_sector_32k_##_dummy_##_size_##_clk_ = { \
	SPI_IF_ERASE_SECTOR_32K, SPI_CMD_SE_32K, _dummy_, _size_, _clk_ }

#define set_erase_sector_64k(_dummy_, _size_, _clk_) \
	static struct spi_op erase_sector_64k_##_dummy_##_size_##_clk_ = { \
	SPI_IF_ERASE_SECTOR_64K, SPI_CMD_SE_64K, _dummy_, _size_, _clk_ }

#define set_erase_sector_128k(_dummy_, _size_, _clk_) \
	static struct spi_op erase_sector_128k_##_dummy_##_size_##_clk_ = { \
	SPI_IF_ERASE_SECTOR_128K, SPI_CMD_SE_128K, _dummy_, _size_, _clk_ }

#define set_erase_sector_256k(_dummy_, _size_, _clk_) \
	static struct spi_op erase_sector_256k_##_dummy_##_size_##_clk_ = { \
	SPI_IF_ERASE_SECTOR_256K, SPI_CMD_SE_256K, _dummy_, _size_, _clk_ }

#define read_std(_dummy_, _size_, _clk_) read_std_##_dummy_##_size_##_clk_
#define read_fast(_dummy_, _size_, _clk_) read_fast_##_dummy_##_size_##_clk_
#define read_dual(_dummy_, _size_, _clk_) read_dual_##_dummy_##_size_##_clk_
#define read_dual_addr(_dummy_, _size_, _clk_) \
	read_dual_addr_##_dummy_##_size_##_clk_
#define read_quad(_dummy_, _size_, _clk_) read_quad_##_dummy_##_size_##_clk_
#define read_quad_addr(_dummy_, _size_, _clk_) \
	read_quad_addr_##_dummy_##_size_##_clk_

#define write_std(_dummy_, _size_, _clk_) write_std_##_dummy_##_size_##_clk_
#define write_dual(_dummy_, _size_, _clk_) write_dual_##_dummy_##_size_##_clk_
#define write_dual_addr(_dummy_, _size_, _clk_) \
	write_dual_addr_##_dummy_##_size_##_clk_
#define write_quad(_dummy_, _size_, _clk_) write_quad_##_dummy_##_size_##_clk_
#define write_quad_addr(_dummy_, _size_, _clk_) \
	write_quad_addr_##_dummy_##_size_##_clk_

#define erase_sector_4k(_dummy_, _size_, _clk_) \
	erase_sector_4k_##_dummy_##_size_##_clk_
#define erase_sector_32k(_dummy_, _size_, _clk_) \
	erase_sector_32k_##_dummy_##_size_##_clk_
#define erase_sector_64k(_dummy_, _size_, _clk_) \
	erase_sector_64k_##_dummy_##_size_##_clk_
#define erase_sector_128k(_dummy_, _size_, _clk_) \
	erase_sector_128k_##_dummy_##_size_##_clk_
#define erase_sector_256k(_dummy_, _size_, _clk_) \
	erase_sector_256k_##_dummy_##_size_##_clk_

#define SPI_CMD_WREN	    0x06    /* Write Enable */
#define SPI_CMD_WRDI	    0x04    /* Write Disable */

#define SPI_CMD_RDID	    0x9F    /* Read Identification */

#define SPI_CMD_GET_FEATURES	0x0F    /* Get Features */
#define SPI_CMD_SET_FEATURE     0x1F    /* Set Feature */

#define SPI_CMD_PAGE_READ       0x13    /* Page Read to Cache */

#define SPI_CMD_RESET	   0xff    /* Reset the device */

/* These macroes are for debug only, reg option is slower then dma option */
#undef FMC100_SPI_NAND_SUPPORT_REG_READ
/* Open it as you need #define FMC100_SPI_NAND_SUPPORT_REG_READ */

#undef FMC100_SPI_NAND_SUPPORT_REG_WRITE
/* Open it as you need #define FMC100_SPI_NAND_SUPPORT_REG_WRITE */

#define WORD_READ_OFFSET_ADD_LENGTH	2
#define WORD_READ_START_OFFSET		2

#define PAGE_PER_BLK_64			64
#define PAGE_PER_BLK_128		128
#define PAGE_PER_BLK_256		256
#define PAGE_PER_BLK_512		512

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

#ifdef CONFIG_BSP_NAND_ECC_STATUS_REPORT

#define FMC100_ECC_ERR_NUM0_BUF0      0xc0

#define get_ecc_err_num(_i, _reg)       (((_reg) >> ((_i) * 8)) & 0xff)
#define ECC_STEP_SHIFT			10
#endif

#define REG_CNT_HIGH_BLOCK_NUM_SHIFT	10

#define REG_CNT_BLOCK_NUM_MASK	  0x3ff
#define REG_CNT_BLOCK_NUM_SHIFT	 22

#define REG_CNT_PAGE_NUM_MASK	   0x3f
#define REG_CNT_PAGE_NUM_SHIFT	  16

#define ERR_STR_DRIVER "Driver does not support this configure "
#define ERR_STR_CHECK "Please make sure the hardware configuration is correct"

#define FMC100_ADDR_CYCLE_MASK	0x2
#define FMC100_ADDR_CYCLE_SHIFT       0x3

#define OP_STYPE_NONE	   0x0
#define OP_STYPE_READ	   0x01
#define OP_STYPE_WRITE	  0x02
#define OP_STYPE_ERASE	  0x04
#define clk_fmc_to_crg_mhz(_clk)    ((_clk) * 2000000)

#define MAX_SPI_OP	  8

/* SPI general operation parameter */
struct spi_op {
	unsigned char iftype;
	unsigned char cmd;
	unsigned char dummy;
	unsigned int size;
	unsigned int clock;
};

struct spi_drv;

/* SPI interface all operation */
struct fmc_spi {
	char *name;
	int chipselect;
	unsigned long long chipsize;
	unsigned int erasesize;
#define SPI_NOR_3BYTE_ADDR_LEN  3   /* address len 3Bytes */
#define SPI_NOR_4BYTE_ADDR_LEN  4   /* address len 4Bytes for 32MB */
	unsigned int addrcycle;

	struct spi_op read[1];
	struct spi_op write[1];
	struct spi_op erase[MAX_SPI_OP];

	void *host;

	struct spi_drv *driver;
};

/* SPI interface special operation function hook */
struct spi_drv {
	int (*wait_ready)(struct fmc_spi *spi);
	int (*write_enable)(struct fmc_spi *spi);
	int (*qe_enable)(struct fmc_spi *spi);
	int (*bus_prepare)(struct fmc_spi *spi, int op);
	int (*entry_4addr)(struct fmc_spi *spi, int en);
};

struct spi_nand_info {
	char *name;
	unsigned char id[MAX_SPI_NAND_ID_LEN];
	unsigned char id_len;
	unsigned long long chipsize;
	unsigned int erasesize;
	unsigned int pagesize;
	unsigned int oobsize;
#define BBP_LAST_PAGE       0x01
#define BBP_FIRST_PAGE      0x02
	unsigned int badblock_pos;
	struct spi_op *read[MAX_SPI_OP];
	struct spi_op *write[MAX_SPI_OP];
	struct spi_op *erase[MAX_SPI_OP];
	struct spi_drv *driver;
};

extern char spi_nand_feature_op(struct fmc_spi *spi, u_char op, u_char addr,
				u_char *val);

struct fmc_host {
	struct mtd_info *mtd;
	struct nand_chip *chip;
	struct fmc_spi spi[CONFIG_SPI_NAND_MAX_CHIP_NUM];
	struct fmc_cmd_op cmd_op;
	void __iomem *iobase;
	void __iomem *regbase;
	struct clk *clk;
	u32 clkrate;
	unsigned int fmc_cfg;
	unsigned int fmc_cfg_ecc0;
	unsigned int offset;
	struct device *dev;
	struct mutex *lock;

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
	unsigned int addr_cycle;
	unsigned int addr_value[2];
	unsigned int cache_addr_value[2];

	unsigned int column;
	unsigned int block_page_mask;
	unsigned int ecctype;
	unsigned int pagesize;
	unsigned int oobsize;
	int add_partition;
	int  need_rr_data;
#define FMC100_READ_RETRY_DATA_LEN	 128
	char rr_data[FMC100_READ_RETRY_DATA_LEN];
	struct read_retry_t *read_retry;

	int version;

	/* BOOTROM read two bytes to detect the bad block flag */
#define FMC_BAD_BLOCK_POS     0
	unsigned char *bbm; /* nand bad block mark */
#define FMC_OOB_LEN_30_EB_OFFSET	(30 - 2)
#define FMC_OOB_LEN_6_EB_OFFSET		(6 - 2)
#define FMC_OOB_LEN_14_EB_OFFSET	(14 - 2)
	unsigned short *epm;    /* nand empty page mark */

	unsigned int uc_er;

	void (*send_cmd_write)(struct fmc_host *host);
	void (*send_cmd_status)(struct fmc_host *host);
	void (*send_cmd_read)(struct fmc_host *host);
	void (*send_cmd_erase)(struct fmc_host *host);
	void (*send_cmd_readid)(struct fmc_host *host);
	void (*send_cmd_reset)(struct fmc_host *host);
#ifdef CONFIG_PM
	int (*suspend)(struct platform_device *pltdev, pm_message_t state);
	int (*resume)(struct platform_device *pltdev);
#endif
};

void fmc100_ecc0_switch(struct fmc_host *host, unsigned char op);

void fmc100_spi_nand_init(struct nand_chip *chip);

extern void fmc_spi_nand_ids_register(void);

extern void fmc_set_nand_system_clock(struct spi_op *op, int clk_en);

int spi_general_wait_ready(struct fmc_spi *spi);
int spi_general_write_enable(struct fmc_spi *spi);
int spi_general_qe_enable(struct fmc_spi *spi);

#ifdef CONFIG_PM
int fmc100_suspend(struct platform_device *pltdev, pm_message_t state);
int fmc100_resume(struct platform_device *pltdev);
void fmc100_spi_nand_config(struct fmc_host *host);
#endif

#endif /* End of __FMC100_H__ */
