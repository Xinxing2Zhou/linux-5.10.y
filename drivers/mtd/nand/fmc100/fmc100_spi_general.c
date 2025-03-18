/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <asm/setup.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/mfd/bsp_fmc.h>
#include <linux/uaccess.h>

#include <linux/securec.h>
#include "fmc100.h"

static int spi_nand_feature_op_config(struct fmc_host *host,
				      u_char op,
				      const u_char *val,
				      u_char addr)
{
	unsigned int reg;

	reg = fmc_cmd_cmd1((op != 0) ? SPI_CMD_SET_FEATURE : SPI_CMD_GET_FEATURES);
	fmc_writel(host, FMC_CMD, reg);
	fmc_pr(FT_DBG, "\t||||-Set CMD[%#x]%#x\n", FMC_CMD, reg);

	fmc_writel(host, FMC_ADDRL, addr);
	fmc_pr(FT_DBG, "\t||||-Set ADDRL[%#x]%#x\n", FMC_ADDRL, addr);

	reg = op_cfg_fm_cs(host->cmd_op.cs) |
	      op_cfg_addr_num(FEATURES_OP_ADDR_NUM) |
	      OP_CFG_OEN_EN;
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(FT_DBG, "\t||||-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	reg = fmc_data_num_cnt(FEATURES_DATA_LEN);
	fmc_writel(host, FMC_DATA_NUM, reg);
	fmc_pr(FT_DBG, "\t||||-Set DATA_NUM[%#x]%#x\n", FMC_DATA_NUM, reg);

	reg = FMC_OP_CMD1_EN |
	      FMC_OP_ADDR_EN |
	      FMC_OP_REG_OP_START;

	if (op == SET_OP) {
		if (!val || !host->iobase) {
			db_msg("Error: host->iobase is NULL !\n");
			return -1;
		}
		reg |= FMC_OP_WRITE_DATA_EN;
		fmc_writeb(*val, host->iobase);
		fmc_pr(FT_DBG, "\t||||-Write IO[%#lx]%#x\n", (long)host->iobase,
		       *(u_char *)host->iobase);
	} else {
		reg |= FMC_OP_READ_DATA_EN;
	}

	fmc_writel(host, FMC_OP, reg);
	fmc_pr(FT_DBG, "\t||||-Set OP[%#x]%#x\n", FMC_OP, reg);

	fmc_cmd_wait_cpu_finish(host);

	return 0;
}

static int spi_nand_get_op(struct fmc_host *host, u_char *val)
{
	unsigned int reg;

	if (!val) {
		db_msg("Error: val is NULL !\n");
		return -1;
	}
	if (SR_DBG)
		pr_info("\n");
	fmc_pr(SR_DBG, "\t\t|*-Start Get Status\n");

	reg = op_cfg_fm_cs(host->cmd_op.cs) | OP_CFG_OEN_EN;
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(SR_DBG, "\t\t||-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	reg = FMC_OP_READ_STATUS_EN | FMC_OP_REG_OP_START;
	fmc_writel(host, FMC_OP, reg);
	fmc_pr(SR_DBG, "\t\t||-Set OP[%#x]%#x\n", FMC_OP, reg);

	fmc_cmd_wait_cpu_finish(host);

	*val = fmc_readl(host, FMC_STATUS);
	fmc_pr(SR_DBG, "\t\t|*-End Get Status, result: %#x\n", *val);

	return 0;
}

/*
    Send set/get features command to SPI Nand flash
*/
char spi_nand_feature_op(struct fmc_spi *spi, u_char op, u_char addr,
			 u_char *val)
{
	const char *str[] = {"Get", "Set"};
	struct fmc_host *host = NULL;
	int ret;

	if (!spi) {
		db_msg("Error: spi is NULL !\n");
		return -1;
	}
	host = (struct fmc_host *)spi->host;
	if (!host) {
		db_msg("Error: host is NULL !\n");
		return -1;
	}

	if ((op == GET_OP) && (addr == STATUS_ADDR)) {
		ret = spi_nand_get_op(host, val);
		return ret;
	}

	fmc_pr(FT_DBG, "\t|||*-Start %s feature, addr[%#x]\n", str[op], addr);

	fmc100_ecc0_switch(host, ENABLE);

	ret = spi_nand_feature_op_config(host, op, val, addr);
	if (ret)
		return -1;

	if (op == GET_OP) {
		if (!val || !host->iobase) {
			db_msg("Error: val or host->iobase is NULL !\n");
			return -1;
		}
		*val = fmc_readb(host->iobase);
		fmc_pr(FT_DBG, "\t||||-Read IO[%#lx]%#x\n", (long)host->iobase,
		       *(u_char *)host->iobase);
	}

	fmc100_ecc0_switch(host, DISABLE);

	fmc_pr(FT_DBG, "\t|||*-End %s Feature[%#x]:%#x\n", str[op], addr, *val);

	return 0;
}

/*
    Read status[C0H]:[0]bit OIP, judge whether the device is busy or not
*/
int spi_general_wait_ready(struct fmc_spi *spi)
{
	unsigned char status;
	int ret;
	unsigned long deadline = jiffies + FMC_MAX_READY_WAIT_JIFFIES;
	struct fmc_host *host = NULL;

	if (spi == NULL || spi->host == NULL) {
		db_msg("Error: host or host->spi is NULL!\n");
		return -1;
	}
	host = (struct fmc_host *)spi->host;

	do {
		ret = spi_nand_feature_op(spi, GET_OP, STATUS_ADDR, &status);
		if (ret)
			return -1;
		if ((status & STATUS_OIP_MASK) == 0) {
			if ((host->cmd_op.l_cmd == NAND_CMD_ERASE2) &&
			    ((((unsigned int)status) & STATUS_E_FAIL_MASK) != 0))
				return status;

			if ((host->cmd_op.l_cmd == NAND_CMD_PAGEPROG) &&
			    ((((unsigned int)status) & STATUS_P_FAIL_MASK) != 0))
				return status;

			return 0;
		}

		cond_resched();
	} while (time_after_eq(jiffies, deadline) == 0);

	db_msg("Error: SPI Nand wait ready timeout, status: %#x\n", status);

	return 1;
}

static void spi_general_write_enable_op(struct fmc_host *host)
{
	unsigned int regl;

	regl = fmc_readl(host, FMC_GLOBAL_CFG);
	fmc_pr(WE_DBG, "\t||-Get GLOBAL_CFG[%#x]%#x\n", FMC_GLOBAL_CFG, regl);
	if (regl & FMC_GLOBAL_CFG_WP_ENABLE) {
		regl &= ~FMC_GLOBAL_CFG_WP_ENABLE;
		fmc_writel(host, FMC_GLOBAL_CFG, regl);
		fmc_pr(WE_DBG, "\t||-Set GLOBAL_CFG[%#x]%#x\n",
		       FMC_GLOBAL_CFG, regl);
	}

	regl = fmc_cmd_cmd1(SPI_CMD_WREN);
	fmc_writel(host, FMC_CMD, regl);
	fmc_pr(WE_DBG, "\t||-Set CMD[%#x]%#x\n", FMC_CMD, regl);

	regl = op_cfg_fm_cs(host->cmd_op.cs) | OP_CFG_OEN_EN;
	fmc_writel(host, FMC_OP_CFG, regl);
	fmc_pr(WE_DBG, "\t||-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, regl);

	regl = FMC_OP_CMD1_EN | FMC_OP_REG_OP_START;
	fmc_writel(host, FMC_OP, regl);
	fmc_pr(WE_DBG, "\t||-Set OP[%#x]%#x\n", FMC_OP, regl);

	fmc_cmd_wait_cpu_finish(host);
}

/*
    Send write enable cmd to SPI Nand, status[C0H]:[2]bit WEL must be set 1
*/
int spi_general_write_enable(struct fmc_spi *spi)
{
	u_char reg;
	int ret;
	struct fmc_host *host = NULL;
	if (spi == NULL || spi->host == NULL) {
		db_msg("Error: host or host->spi is NULL!\n");
		return -1;
	}
	host = spi->host;
	if (WE_DBG)
		pr_info("\n");
	fmc_pr(WE_DBG, "\t|*-Start Write Enable\n");

	ret = spi_nand_feature_op(spi, GET_OP, STATUS_ADDR, &reg);
	if (ret)
		return -1;
	if (reg & STATUS_WEL_MASK) {
		fmc_pr(WE_DBG, "\t||-Write Enable was opened! reg: %#x\n",
		       reg);
		return 0;
	}

	spi_general_write_enable_op(host);

#if WE_DBG
	if (!spi->driver) {
		db_msg("Error: spi->driver is NULL!\n");
		return -1;
	}
	spi->driver->wait_ready(spi);

	ret = spi_nand_feature_op(spi, GET_OP, STATUS_ADDR, &reg);
	if (ret)
		return -1;
	if (reg & STATUS_WEL_MASK) {
		fmc_pr(WE_DBG, "\t||-Write Enable success. reg: %#x\n", reg);
	} else {
		db_msg("Error: Write Enable failed! reg: %#x\n", reg);
		return reg;
	}
#endif

	fmc_pr(WE_DBG, "\t|*-End Write Enable\n");
	return 0;
}

/*
    judge whether SPI Nand support QUAD read/write or not
*/
static int spi_is_quad(const struct fmc_spi *spi)
{
	const char *if_str[] = {"STD", "DUAL", "DIO", "QUAD", "QIO"};
	fmc_pr(QE_DBG, "\t\t|||*-SPI read iftype: %s write iftype: %s\n",
	       if_str[spi->read->iftype], if_str[spi->write->iftype]);

	if ((spi->read->iftype == IF_TYPE_QUAD) ||
	    (spi->read->iftype == IF_TYPE_QIO) ||
	    (spi->write->iftype == IF_TYPE_QUAD) ||
	    (spi->write->iftype == IF_TYPE_QIO))
		return 1;

	return 0;
}

/*
    Send set features cmd to SPI Nand, feature[B0H]:[0]bit QE would be set
*/
int spi_general_qe_enable(struct fmc_spi *spi)
{
	int op;
	u_char reg;
	int ret;
	const char *str[] = {"Disable", "Enable"};
	if (!spi || !spi->host || !spi->driver) {
		db_msg("Error: host or spi->host or spi->driver is NULL!\n");
		return -1;
	}
	fmc_pr(QE_DBG, "\t||*-Start SPI Nand flash QE\n");

	op = spi_is_quad(spi);

	fmc_pr(QE_DBG, "\t|||*-End Quad check, SPI Nand %s Quad.\n", str[op]);

	ret = spi_nand_feature_op(spi, GET_OP, FEATURE_ADDR, &reg);
	if (ret)
		return -1;
	fmc_pr(QE_DBG, "\t|||-Get [%#x]feature: %#x\n", FEATURE_ADDR, reg);
	if ((reg & FEATURE_QE_ENABLE) == op) {
		fmc_pr(QE_DBG, "\t||*-SPI Nand quad was %sd!\n", str[op]);
		return op;
	}

	if (op == ENABLE)
		reg |= FEATURE_QE_ENABLE;
	else
		reg &= ~FEATURE_QE_ENABLE;

	ret = spi_nand_feature_op(spi, SET_OP, FEATURE_ADDR, &reg);
	if (ret)
		return -1;
	fmc_pr(QE_DBG, "\t|||-SPI Nand %s Quad\n", str[op]);

	spi->driver->wait_ready(spi);

	ret = spi_nand_feature_op(spi, GET_OP, FEATURE_ADDR, &reg);
	if (ret)
		return -1;
	if ((reg & FEATURE_QE_ENABLE) == op)
		fmc_pr(QE_DBG, "\t|||-SPI Nand %s Quad succeed!\n", str[op]);
	else
		db_msg("Error: %s Quad failed! reg: %#x\n", str[op], reg);

	fmc_pr(QE_DBG, "\t||*-End SPI Nand %s Quad.\n", str[op]);

	return op;
}
