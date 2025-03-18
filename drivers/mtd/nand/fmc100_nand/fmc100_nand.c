/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/io.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/clk.h>
#include <linux/mfd/bsp_fmc.h>

#include "../raw/nfc_gen.h"
#include "fmc100_nand_os.h"
#include "fmc100_nand.h"

#include <mach/platform.h>
#include <linux/securec.h>

static void fmc100_dma_addr_config(struct fmc_host *host, char *op)
{
	unsigned int reg = (unsigned int)host->dma_buffer;

	fmc_pr(DMA_DB, "\t\t *-Start %s page dma transfer\n", op);

	fmc_writel(host, FMC_DMA_SADDR_D0, reg);
	fmc_pr(DMA_DB, "\t\t |-Set ADDR0[%#x]%#x\n", FMC_DMA_SADDR_D0, reg);

#ifdef CONFIG_64BIT
	reg = (unsigned int)((host->dma_buffer & FMC_DMA_SADDRH_MASK) >>
		FMC_DMA_BIT_SHIFT_LENTH);
	fmc_writel(host, FMC_DMA_SADDRH_D0, reg);
	fmc_pr(DMA_DB, "\t\t |-Set ADDRH0[%#x]%#x\n", FMC_DMA_SADDRH_D0, reg);
#endif

	reg += FMC_DMA_ADDR_OFFSET;
	fmc_writel(host, FMC_DMA_SADDR_D1, reg);
	fmc_pr(DMA_DB, "\t\t |-Set ADDR1[%#x]%#x\n", FMC_DMA_SADDR_D1, reg);

	reg += FMC_DMA_ADDR_OFFSET;
	fmc_writel(host, FMC_DMA_SADDR_D2, reg);
	fmc_pr(DMA_DB, "\t\t |-Set ADDR2[%#x]%#x\n", FMC_DMA_SADDR_D2, reg);

	reg += FMC_DMA_ADDR_OFFSET;
	fmc_writel(host, FMC_DMA_SADDR_D3, reg);
	fmc_pr(DMA_DB, "\t\t |-Set ADDR3[%#x]%#x\n", FMC_DMA_SADDR_D3, reg);

	reg = host->dma_oob;
	fmc_writel(host, FMC_DMA_SADDR_OOB, reg);
	fmc_pr(DMA_DB, "\t\t |-Set OOB[%#x]%#x\n", FMC_DMA_SADDR_OOB, reg);

#ifdef CONFIG_64BIT
	reg = (unsigned int)((host->dma_oob & FMC_DMA_SADDRH_MASK) >>
		FMC_DMA_BIT_SHIFT_LENTH);
	fmc_writel(host, FMC_DMA_SADDRH_OOB, reg);
	fmc_pr(DMA_DB, "\t\t |-Set ADDRH0[%#x]%#x\n", FMC_DMA_SADDRH_OOB, reg);
#endif
}

static void fmc100_dma_transfer(struct fmc_host *host, unsigned int todev)
{
	unsigned int reg;
	char *op = todev ? "write" : "read";

	fmc100_dma_addr_config(host, op);

	if (host->ecctype == NAND_ECC_0BIT) {
		fmc_writel(host, FMC_DMA_LEN, fmc_dma_len_set(host->oobsize));
		fmc_pr(DMA_DB, "\t\t |-Set LEN[%#x]%#x\n", FMC_DMA_LEN, reg);
	}
	reg = FMC_OP_READ_DATA_EN | FMC_OP_WRITE_DATA_EN;
	fmc_writel(host, FMC_OP, reg);
	fmc_pr(DMA_DB, "\t\t |-Set OP[%#x]%#x\n", FMC_OP, reg);

	reg = FMC_DMA_AHB_CTRL_DMA_PP_EN |
	      FMC_DMA_AHB_CTRL_BURST16_EN |
	      FMC_DMA_AHB_CTRL_BURST8_EN |
	      FMC_DMA_AHB_CTRL_BURST4_EN;
	fmc_writel(host, FMC_DMA_AHB_CTRL, reg);
	fmc_pr(DMA_DB, "\t\t |-Set AHBCTRL[%#x]%#x\n", FMC_DMA_AHB_CTRL, reg);

	reg = op_cfg_fm_cs(host->cmd_op.cs) |
	      op_cfg_addr_num(host->addr_cycle);
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(DMA_DB, "\t\t |-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	reg = OP_CTRL_DMA_OP_READY;
	if (todev)
		reg |= op_ctrl_rw_op(todev);

	fmc_writel(host, FMC_OP_CTRL, reg);
	fmc_pr(DMA_DB, "\t\t |-Set OP_CTRL[%#x]%#x\n", FMC_OP_CTRL, reg);

	fmc_dma_wait_cpu_finish(host);

	fmc_pr(DMA_DB, "\t\t *-End %s page dma transfer\n", op);

	return;
}

static void fmc100_send_cmd_write(struct fmc_host *host)
{
	unsigned int reg;

	fmc_pr(WR_DBG, "\t|*-Start send page programme cmd\n");

	if (*host->bbm != 0xFF && *host->bbm != 0x00)
		pr_info("WARNING: attempt to write an invalid bbm. "
			"page: 0x%08x, mark: 0x%02x,\n",
			get_page_index(host), *host->bbm);

	host->enable_ecc_randomizer(host, ENABLE, ENABLE);

	reg = host->addr_value[1];
	fmc_writel(host, FMC_ADDRH, reg);
	fmc_pr(WR_DBG, "\t||-Set ADDRH[%#x]%#x\n", FMC_ADDRH, reg);

	reg = host->addr_value[0] & 0xffff0000;
	fmc_writel(host, FMC_ADDRL, reg);
	fmc_pr(WR_DBG, "\t||-Set ADDRL[%#x]%#x\n", FMC_ADDRL, reg);

	reg = fmc_cmd_cmd2(NAND_CMD_PAGEPROG) | fmc_cmd_cmd1(NAND_CMD_SEQIN);
	fmc_writel(host, FMC_CMD, reg);
	fmc_pr(WR_DBG, "\t||-Set CMD[%#x]%#x\n", FMC_CMD, reg);

	*host->epm = 0x0000;

	fmc100_dma_transfer(host, RW_OP_WRITE);

	fmc_pr(WR_DBG, "\t|*-End send page read cmd\n");
}

static void fmc100_send_cmd_read(struct fmc_host *host)
{
	unsigned int reg;

	fmc_pr(RD_DBG, "\t*-Start send page read cmd\n");

	if ((host->addr_value[0] == host->cache_addr_value[0]) &&
	    (host->addr_value[1] == host->cache_addr_value[1])) {
		fmc_pr(RD_DBG, "\t*-Cache hit! addr1[%#x], addr0[%#x]\n",
		       host->addr_value[1], host->addr_value[0]);
		return;
	}

	host->page_status = 0;

	host->enable_ecc_randomizer(host, ENABLE, ENABLE);

	reg = FMC_INT_CLR_ALL;
	fmc_writel(host, FMC_INT_CLR, reg);
	fmc_pr(RD_DBG, "\t|-Set INT_CLR[%#x]%#x\n", FMC_INT_CLR, reg);

	reg = host->nand_cfg;
	fmc_writel(host, FMC_CFG, reg);
	fmc_pr(RD_DBG, "\t|-Set CFG[%#x]%#x\n", FMC_CFG, reg);

	reg = host->addr_value[1];
	fmc_writel(host, FMC_ADDRH, reg);
	fmc_pr(RD_DBG, "\t|-Set ADDRH[%#x]%#x\n", FMC_ADDRH, reg);

	reg = host->addr_value[0] & 0xffff0000;
	fmc_writel(host, FMC_ADDRL, reg);
	fmc_pr(RD_DBG, "\t|-Set ADDRL[%#x]%#x\n", FMC_ADDRL, reg);

	reg = fmc_cmd_cmd2(NAND_CMD_READSTART) | fmc_cmd_cmd1(NAND_CMD_READ0);
	fmc_writel(host, FMC_CMD, reg);
	fmc_pr(RD_DBG, "\t|-Set CMD[%#x]%#x\n", FMC_CMD, reg);

	fmc100_dma_transfer(host, RW_OP_READ);

	if (fmc_readl(host, FMC_INT) & FMC_INT_ERR_INVALID)
		host->page_status |= FMC100_PS_UC_ECC;

	host->cache_addr_value[0] = host->addr_value[0];
	host->cache_addr_value[1] = host->addr_value[1];

	fmc_pr(RD_DBG, "\t*-End send page read cmd\n");
}

static void fmc100_send_cmd_erase(struct fmc_host *host)
{
	unsigned int reg;

	fmc_pr(ER_DBG, "\t *-Start send cmd erase\n");

	/* Don't case the read retry config */
	host->enable_ecc_randomizer(host, DISABLE, DISABLE);

	reg = host->addr_value[0];
	fmc_writel(host, FMC_ADDRL, reg);
	fmc_pr(ER_DBG, "\t |-Set ADDRL[%#x]%#x\n", FMC_ADDRL, reg);

	reg = fmc_cmd_cmd2(NAND_CMD_ERASE2) | fmc_cmd_cmd1(NAND_CMD_ERASE1);
	fmc_writel(host, FMC_CMD, reg);
	fmc_pr(ER_DBG, "\t |-Set CMD[%#x]%#x\n", FMC_CMD, reg);

	reg = op_cfg_fm_cs(host->cmd_op.cs) |
	      op_cfg_addr_num(host->addr_cycle);
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(ER_DBG, "\t |-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	/* need to config WAIT_READY_EN */
	reg = FMC_OP_WAIT_READY_EN |
	      FMC_OP_CMD1_EN |
	      FMC_OP_CMD2_EN |
	      FMC_OP_ADDR_EN |
	      FMC_OP_REG_OP_START;
	fmc_writel(host, FMC_OP, reg);
	fmc_pr(ER_DBG, "\t |-Set OP[%#x]%#x\n", FMC_OP, reg);

	fmc_cmd_wait_cpu_finish(host);

	fmc_pr(ER_DBG, "\t |*-End send cmd erase\n");
}

static void fmc100_ecc_randomizer(struct fmc_host *host, int ecc_en,
				    int randomizer_en)
{
	unsigned int old_reg, reg;
	unsigned int  change = 0;
	char *ecc_op = ecc_en ? "Quit" : "Enter";
	char *rand_op = randomizer_en ? "Enable" : "Disable";

	if (IS_NAND_RANDOM(host)) {
		reg = old_reg = fmc_readl(host, FMC_GLOBAL_CFG);
		if (randomizer_en)
			reg |= FMC_GLOBAL_CFG_RANDOMIZER_EN;
		else
			reg &= ~FMC_GLOBAL_CFG_RANDOMIZER_EN;

		if (old_reg != reg) {
			fmc_pr(EC_DBG, "\t |*-Start %s randomizer\n", rand_op);
			fmc_pr(EC_DBG, "\t ||-Get global CFG[%#x]%#x\n",
			       FMC_GLOBAL_CFG, old_reg);
			fmc_writel(host, FMC_GLOBAL_CFG, reg);
			fmc_pr(EC_DBG, "\t ||-Set global CFG[%#x]%#x\n",
			       FMC_GLOBAL_CFG, reg);
			change++;
		}
	}

	old_reg = fmc_readl(host, FMC_CFG);
	reg = (ecc_en ? host->nand_cfg : host->nand_cfg_ecc0);

	if (old_reg != reg) {
		fmc_pr(EC_DBG, "\t |%s-Start %s ECC0 mode\n", change ? "|" : "*",
		       ecc_op);
		fmc_pr(EC_DBG, "\t ||-Get CFG[%#x]%#x\n", FMC_CFG, old_reg);
		fmc_writel(host, FMC_CFG, reg);
		fmc_pr(EC_DBG, "\t ||-Set CFG[%#x]%#x\n", FMC_CFG, reg);
		change++;
	}

	if (EC_DBG && change)
		fmc_pr(EC_DBG, "\t |*-End randomizer and ECC0 mode config\n");
}

static void fmc100_send_cmd_status(struct fmc_host *host)
{
	unsigned int regval;

	host->enable_ecc_randomizer(host, DISABLE, DISABLE);

	regval = op_cfg_fm_cs(host->cmd_op.cs);
	fmc_writel(host, FMC_OP_CFG, regval);

	regval = FMC_OP_READ_STATUS_EN | FMC_OP_REG_OP_START;
	fmc_writel(host, FMC_OP, regval);

	fmc_cmd_wait_cpu_finish(host);
}

static void fmc100_send_cmd_readid(struct fmc_host *host)
{
	unsigned int reg;

	fmc_pr(BT_DBG, "\t *-Start read nand flash ID\n");

	host->enable_ecc_randomizer(host, DISABLE, DISABLE);

	reg = fmc_data_num_cnt(host->cmd_op.data_no);
	fmc_writel(host, FMC_DATA_NUM, reg);
	fmc_pr(BT_DBG, "\t |-Set DATA_NUM[%#x]%#x\n", FMC_DATA_NUM, reg);

	reg = fmc_cmd_cmd1(NAND_CMD_READID);
	fmc_writel(host, FMC_CMD, reg);
	fmc_pr(BT_DBG, "\t |-Set CMD[%#x]%#x\n", FMC_CMD, reg);

	reg = 0;
	fmc_writel(host, FMC_ADDRL, reg);
	fmc_pr(BT_DBG, "\t |-Set ADDRL[%#x]%#x\n", FMC_ADDRL, reg);

	reg = op_cfg_fm_cs(host->cmd_op.cs) |
	      op_cfg_addr_num(READ_ID_ADDR_NUM);
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(BT_DBG, "\t |-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	reg = FMC_OP_CMD1_EN |
	      FMC_OP_ADDR_EN |
	      FMC_OP_READ_DATA_EN |
	      FMC_OP_REG_OP_START;
	fmc_writel(host, FMC_OP, reg);
	fmc_pr(BT_DBG, "\t |-Set OP[%#x]%#x\n", FMC_OP, reg);

	host->addr_cycle = 0x0;

	fmc_cmd_wait_cpu_finish(host);

	fmc_pr(BT_DBG, "\t *-End read nand flash ID\n");
}

static void fmc100_send_cmd_reset(struct fmc_host *host)
{
	unsigned int reg;

	fmc_pr(BT_DBG, "\t *-Start reset nand flash\n");

	reg = fmc_cmd_cmd1(NAND_CMD_RESET);
	fmc_writel(host, FMC_CMD, reg);
	fmc_pr(BT_DBG, "\t |-Set CMD[%#x]%#x\n", FMC_CMD, reg);

	reg = op_cfg_fm_cs(host->cmd_op.cs);
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(BT_DBG, "\t |-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	reg = FMC_OP_CMD1_EN |
	      FMC_OP_WAIT_READY_EN |
	      FMC_OP_REG_OP_START;
	fmc_writel(host, FMC_OP, reg);
	fmc_pr(BT_DBG, "\t |-Set OP[%#x]%#x\n", FMC_OP, reg);

	fmc_cmd_wait_cpu_finish(host);

	fmc_pr(BT_DBG, "\t *-End reset nand flash\n");
}

static unsigned char fmc100_read_byte(struct nand_chip *chip)
{
	unsigned char value = 0;
	struct fmc_host *host = chip->priv;

	if (host->cmd_op.l_cmd == NAND_CMD_READID) {
		value = fmc_readb((void __iomem *)(chip->legacy.IO_ADDR_R + host->offset));
		host->offset++;
		if (host->cmd_op.data_no == host->offset)
			host->cmd_op.l_cmd = 0;

		return value;
	}

	if (host->cmd_op.cmd == NAND_CMD_STATUS) {
		value = fmc_readl(host, FMC_STATUS);
		if (host->cmd_op.l_cmd == NAND_CMD_ERASE1)
			fmc_pr(ER_DBG, "\t*-Erase WP status: %#x\n", value);

		if (host->cmd_op.l_cmd == NAND_CMD_PAGEPROG)
			fmc_pr(WR_DBG, "\t*-Write WP status: %#x\n", value);

		return value;
	}

	if (host->cmd_op.l_cmd == NAND_CMD_READOOB) {
		value = fmc_readb((void __iomem *)(host->buffer +
						     host->pagesize + host->offset));
		host->offset++;
		return value;
	}

	host->offset++;

	return fmc_readb((void __iomem *)(host->buffer + host->column +
					    host->offset - 1));
}

static void fmc100_write_buf(struct nand_chip *chip,
			       const u_char *buf, int len)
{
	struct fmc_host *host = chip->priv;
	int ret;

#ifdef FMC100_NAND_SUPPORT_REG_WRITE
	if (buf == chip->oob_poi)
		ret = memcpy_s((char *)host->iobase + host->pagesize,
				FMC_MEM_LEN, buf, len);
	else
		ret = memcpy_s((char *)host->iobase, FMC_MEM_LEN, buf, len);

#else
	if (buf == chip->oob_poi)
		ret = memcpy_s((char *)host->buffer + host->pagesize,
				FMC_MAX_DMA_LEN, buf, len);
	else
		ret = memcpy_s((char *)host->buffer, FMC_MAX_DMA_LEN, buf, len);

#endif
	if (ret)
		printk("%s: memcpy_s failed\n", __func__);

	return;
}

#ifdef CONFIG_BSP_NAND_ECC_STATUS_REPORT

static void fmc100_ecc_err_num_count(struct mtd_info *mtd,
				       u_int ecc_st, u_int reg)
{
	u_char err_num;

	if (ecc_st > ECC_STEP_MAX_4K_PAGE)
		ecc_st = ECC_STEP_MAX_4K_PAGE;

	while (ecc_st) {
		err_num = get_ecc_err_num(--ecc_st, reg);
		if (err_num == 0xff)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += err_num;
	}
}
#endif

static void fmc100_read_buf(struct nand_chip *chip, u_char *buf, int len)
{
#ifdef CONFIG_BSP_NAND_ECC_STATUS_REPORT
	struct mtd_info *mtd = nand_to_mtd(chip);
#endif
	struct fmc_host *host = chip->priv;
	int ret;

#ifdef FMC100_NAND_SUPPORT_REG_READ
	if (buf == chip->oob_poi)
		ret = memcpy_s(buf, MAX_OOB_LEN, (char *)host->iobase +
				host->pagesize, len);
	else
		ret = memcpy_s(buf, MAX_PAGE_SIZE, (char *)host->iobase, len);

#else
	if (buf == chip->oob_poi)
		ret = memcpy_s(buf, MAX_OOB_LEN, (char *)host->buffer +
				host->pagesize, len);
	else
		ret = memcpy_s(buf, MAX_PAGE_SIZE, (char *)host->buffer, len);
#endif
	if (ret)
		printk("%s: memcpy_s failed\n", __func__);

#ifdef CONFIG_BSP_NAND_ECC_STATUS_REPORT
	if (buf != chip->oob_poi) {
		u_int reg;
		u_int ecc_step = host->pagesize >> ECC_STEP_SHIFT;

		/* 2K or 4K or 8K(1) or 16K(1-1) pagesize */
		reg = fmc_readl(host, FMC100_ECC_ERR_NUM0_BUF0);
		fmc100_ecc_err_num_count(mtd, ecc_step, reg);

		if (ecc_step > ECC_STEP_MAX_4K_PAGE) {
			/* 8K(2) or 16K(1-2) pagesize */
			reg = fmc_readl(host, FMC100_ECC_ERR_NUM1_BUF0);
			fmc100_ecc_err_num_count(mtd, ecc_step, reg);
			if (ecc_step > ECC_STEP_MAX_8K_PAGE) {
				/* 16K(2-1) pagesize */
				reg = fmc_readl(host,
						  FMC100_ECC_ERR_NUM0_BUF1);
				fmc100_ecc_err_num_count(mtd, ecc_step, reg);
				/* 16K(2-2) pagesize */
				reg = fmc_readl(host,
						  FMC100_ECC_ERR_NUM1_BUF1);
				fmc100_ecc_err_num_count(mtd, ecc_step, reg);
			}
		}
	}
#endif

	return;
}

static void fmc100_select_chip(struct nand_chip *chip, int chipselect)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct fmc_host *host = chip->priv;

	if (chipselect < 0) {
		mutex_unlock(&fmc_switch_mutex);
		return;
	}

	mutex_lock(&fmc_switch_mutex);

	if (chipselect > CONFIG_FMC100_MAX_NAND_CHIP)
		db_bug("Error: Invalid chip select: %d\n", chipselect);

	host->cmd_op.cs = chipselect;
	if (host->mtd != mtd)
		host->mtd = mtd;
}

static void fmc100_nand_ale_init(struct fmc_host *host, unsigned ctrl, int dat)
{
	unsigned int addr_value = 0;
	unsigned int addr_offset;

	if (ctrl & NAND_CTRL_CHANGE) {
		host->addr_cycle = 0x0;
		host->addr_value[0] = 0x0;
		host->addr_value[1] = 0x0;
	}
	addr_offset = host->addr_cycle << FMC100_ADDR_CYCLE_SHIFT;

	if (host->addr_cycle >= FMC100_ADDR_CYCLE_MASK) {
		addr_offset = (host->addr_cycle -
			       FMC100_ADDR_CYCLE_MASK) <<
			       FMC100_ADDR_CYCLE_SHIFT;
		addr_value = 1;
	}

	host->addr_value[addr_value] |=
		(((unsigned int)dat & 0xff) << addr_offset);

	host->addr_cycle++;
}

static void fmc100_nand_cle_init(struct fmc_host *host,
				   struct nand_chip *chip,
				   int dat,
				   int *is_cache_invalid)
{
	unsigned char cmd;

	cmd = (unsigned int)dat & 0xff;
	host->cmd_op.cmd = cmd;
	switch (cmd) {
	case NAND_CMD_PAGEPROG:
		host->offset = 0;
		host->send_cmd_pageprog(host);
		break;

	case NAND_CMD_READSTART:
		*is_cache_invalid = 0;
		if (host->addr_value[0] == host->pagesize)
			host->cmd_op.l_cmd = NAND_CMD_READOOB;

		host->send_cmd_readstart(host);
		break;

	case NAND_CMD_ERASE2:
		host->cmd_op.l_cmd = cmd;
		host->send_cmd_erase(host);
		break;

	case NAND_CMD_READID:
		/* dest fmcbuf just need init ID lenth */
		if (memset_s((u_char *)(chip->legacy.IO_ADDR_R), FMC_MEM_LEN,
				0, MAX_NAND_ID_LEN)) {
			printk("%s %d:memset_s failed\n", __func__, __LINE__);
			break;
		}
		host->cmd_op.l_cmd = cmd;
		host->cmd_op.data_no = MAX_NAND_ID_LEN;
		host->send_cmd_readid(host);
		break;

	case NAND_CMD_STATUS:
		host->send_cmd_status(host);
		break;

	case NAND_CMD_READ0:
		host->cmd_op.l_cmd = cmd;
		break;

	case NAND_CMD_RESET:
		host->send_cmd_reset(host);
		break;

	case NAND_CMD_SEQIN:
	case NAND_CMD_ERASE1:
	default:
		break;
	}
}

static void fmc100_cmd_ctrl(struct nand_chip *chip, int dat, unsigned ctrl)
{
	int is_cache_invalid = 1;
	struct fmc_host *host = chip->priv;

	if (ctrl & NAND_ALE)
		fmc100_nand_ale_init(host, ctrl, dat);

	if ((ctrl & NAND_CLE) && (ctrl & NAND_CTRL_CHANGE))
		fmc100_nand_cle_init(host, chip, dat, &is_cache_invalid);

	/* pass pagesize and ecctype to kernel when startup. */
	host->enable_ecc_randomizer(host, ENABLE, ENABLE);

	if ((dat == NAND_CMD_NONE) && host->addr_cycle) {
		if (host->cmd_op.cmd == NAND_CMD_SEQIN ||
		    host->cmd_op.cmd == NAND_CMD_READ0 ||
		    host->cmd_op.cmd == NAND_CMD_READID) {
			host->offset = 0x0;
			host->column = (host->addr_value[0] & 0xffff);
		}
	}

	if (is_cache_invalid) {
		host->cache_addr_value[0] = ~0;
		host->cache_addr_value[1] = ~0;
	}
}

static int fmc100_dev_ready(struct nand_chip *chip)
{
	return 0x1;
}

/*
 * 'host->epm' only use the first oobfree[0] field, it looks very simple, But...
 */
static int fmc_ooblayout_ecc_default(struct mtd_info *mtd, int section,
				       struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->length = OOB_LENGTH_DEFAULT;
	oobregion->offset = OOB_OFFSET_DEFAULT;

	return 0;
}

static int fmc_ooblayout_free_default(struct mtd_info *mtd, int section,
					struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->length = OOB_LENGTH_DEFAULT_FREE;
	oobregion->offset = OOB_OFFSET_DEFAULT_FREE;

	return 0;
}

static struct mtd_ooblayout_ops fmc_ooblayout_default_ops = {
	.ecc = fmc_ooblayout_ecc_default,
	.free = fmc_ooblayout_free_default,
};

#ifdef CONFIG_BSP_NAND_FS_MAY_NO_YAFFS2
static int fmc_ooblayout_ecc_4k16bit(struct mtd_info *mtd, int section,
				       struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->length = OOB_LENGTH_4K16BIT;
	oobregion->offset = OOB_OFFSET_4K16BIT;

	return 0;
}

static int fmc_ooblayout_free_4k16bit(struct mtd_info *mtd, int section,
					struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->length = OOB_LENGTH_4K16BIT_FREE;
	oobregion->offset = OOB_OFFSET_4K16BIT_FREE;

	return 0;
}
tatic struct mtd_ooblayout_ops fmc_ooblayout_4k16bit_ops = {
	.ecc = fmc_ooblayout_ecc_4k16bit,
	.free = fmc_ooblayout_free_4k16bit,
};

static int fmc_ooblayout_ecc_2k16bit(struct mtd_info *mtd, int section,
				       struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->length = OOB_LENGTH_2K16BIT;
	oobregion->offset = OOB_OFFSET_2K16BIT;

	return 0;
}

static int fmc_ooblayout_free_2k16bit(struct mtd_info *mtd, int section,
					struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->length = OOB_LENGTH_2K16BIT_FREE;
	oobregion->offset = OOB_OFFSET_2K16BIT_FREE;

	return 0;
}

static struct mtd_ooblayout_ops fmc_ooblayout_2k16bit_ops = {
	.ecc = fmc_ooblayout_ecc_2k16bit,
	.free = fmc_ooblayout_free_2k16bit,
};
#endif

/* ecc/pagesize get from NAND controller */
static struct nand_config_info fmc100_nand_hw_auto_config_table[] = {
	{NAND_PAGE_16K, NAND_ECC_64BIT, 64, 1824, &fmc_ooblayout_default_ops}, /* 1824 */
	{NAND_PAGE_16K, NAND_ECC_40BIT, 40, 1200, &fmc_ooblayout_default_ops}, /* 1152 */
	{NAND_PAGE_16K, NAND_ECC_0BIT,  0, 32,          &fmc_ooblayout_default_ops},
	{NAND_PAGE_8K, NAND_ECC_64BIT, 64, 928, &fmc_ooblayout_default_ops}, /* 928 */
	{NAND_PAGE_8K, NAND_ECC_40BIT, 40, 600, &fmc_ooblayout_default_ops}, /* 592 */
	{NAND_PAGE_8K, NAND_ECC_24BIT, 24, 368, &fmc_ooblayout_default_ops}, /* 368 */
	{NAND_PAGE_8K, NAND_ECC_0BIT,  0, 32, &fmc_ooblayout_default_ops},
	{NAND_PAGE_4K, NAND_ECC_24BIT, 24, 200, &fmc_ooblayout_default_ops}, /* 200 */
#ifdef CONFIG_BSP_NAND_FS_MAY_NO_YAFFS2
	{NAND_PAGE_4K, NAND_ECC_16BIT, 16, 128, &fmc_ooblayout_4k16bit_ops}, /* 128 */
#endif
	{NAND_PAGE_4K, NAND_ECC_8BIT, 8, 128, &fmc_ooblayout_default_ops}, /* 88 */
	{NAND_PAGE_4K, NAND_ECC_0BIT, 0, 32, &fmc_ooblayout_default_ops},
	{NAND_PAGE_2K, NAND_ECC_24BIT, 24, 128, &fmc_ooblayout_default_ops}, /* 116 */
#ifdef CONFIG_BSP_NAND_FS_MAY_NO_YAFFS2
	{NAND_PAGE_2K, NAND_ECC_16BIT, 16, 64, &fmc_ooblayout_2k16bit_ops}, /* 64 */
#endif
	{NAND_PAGE_2K, NAND_ECC_8BIT,  8, 64, &fmc_ooblayout_default_ops}, /* 60 */
	{NAND_PAGE_2K, NAND_ECC_0BIT,  0, 32, &fmc_ooblayout_default_ops},
	{0,     0,      0,      0,      NULL},
};

/*
 *  0 - This NAND NOT support randomizer
 *  1 - This NAND support randomizer.
 */
static int fmc100_nand_support_randomizer(u_int pageisze, u_int ecctype)
{
	switch (pageisze) {
	case _8K:
		return (ecctype >= NAND_ECC_24BIT && ecctype <= NAND_ECC_80BIT);
	case _16K:
		return (ecctype >= NAND_ECC_40BIT && ecctype <= NAND_ECC_80BIT);
	case _32K:
		return (ecctype >= NAND_ECC_40BIT && ecctype <= NAND_ECC_80BIT);
	default:
		return 0;
	}
}

/* used the best correct arithmetic. */
static struct nand_config_info *fmc100_get_config_type_info(
	struct mtd_info *mtd, struct nand_dev_t *nand_dev)
{
	struct nand_config_info *best = NULL;
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct fmc_host *host = chip->priv;
	struct nand_config_info *info = fmc100_nand_hw_auto_config_table;

	nand_dev->start_type = "Auto";
	nand_dev->flags |= (IS_NANDC_HW_AUTO(host) | IS_NANDC_CONFIG_DONE(host));

	for (; info->ooblayout_ops; info++) {
		if (match_page_type_to_size(info->pagetype) != mtd->writesize)
			continue;

		if (mtd->oobsize < info->oobsize)
			continue;

		if (!best || (best->ecctype < info->ecctype))
			best = info;
	}

	return best;
}

static unsigned int fmc100_get_ecc_reg(struct fmc_host *host,
		const struct nand_config_info *info, struct nand_dev_t *nand_dev)
{
	host->ecctype = info->ecctype;
	fmc_pr(BT_DBG, "\t |-Save best EccType %d(%s)\n", host->ecctype,
	       match_ecc_type_to_str(info->ecctype));

	nand_dev->ecctype = host->ecctype;

	return fmc_cfg_ecc_type(match_ecc_type_to_reg(info->ecctype));
}

static unsigned int fmc100_get_page_reg(struct fmc_host *host,
		const struct nand_config_info *info)
{
	host->pagesize = match_page_type_to_size(info->pagetype);
	fmc_pr(BT_DBG, "\t |-Save best PageSize %d(%s)\n", host->pagesize,
	       match_page_type_to_str(info->pagetype));

	return fmc_cfg_page_size(match_page_type_to_reg(info->pagetype));
}

static unsigned int fmc100_get_block_reg(struct fmc_host *host,
		const struct nand_config_info *info)
{
	unsigned int block_reg = 0;
	unsigned int page_per_block;
	struct mtd_info *mtd = host->mtd;

	host->block_page_mask = ((mtd->erasesize / mtd->writesize) - 1);
	page_per_block = mtd->erasesize / match_page_type_to_size(info->pagetype);
	switch (page_per_block) {
	case PAGE_PER_BLK_64:
		block_reg = BLOCK_SIZE_64_PAGE;
		break;
	case PAGE_PER_BLK_128:
		block_reg = BLOCK_SIZE_128_PAGE;
		break;
	case PAGE_PER_BLK_256:
		block_reg = BLOCK_SIZE_256_PAGE;
		break;
	case PAGE_PER_BLK_512:
		block_reg = BLOCK_SIZE_512_PAGE;
		break;
	default:
		db_msg("Can't support block %#x and page %#x size\n",
		       mtd->erasesize, mtd->writesize);
	}

	return fmc_cfg_block_size(block_reg);
}

static void fmc100_set_fmc_cfg_reg(struct fmc_host *host,
				     const struct nand_config_info *type_info, struct nand_dev_t *nand_dev)
{
	unsigned int page_reg, ecc_reg, block_reg, reg_fmc_cfg;

	ecc_reg = fmc100_get_ecc_reg(host, type_info, nand_dev);
	page_reg = fmc100_get_page_reg(host, type_info);
	block_reg = fmc100_get_block_reg(host, type_info);

	if (fmc100_nand_support_randomizer(host->pagesize, host->ecctype))
		host->flags |= IS_NAND_RANDOM(nand_dev);

	/*
	 * Check if hardware enable randomizer PIN, But NAND does not need
	 * randomizer. We will notice user.
	 */
	if (IS_NAND_RANDOM(host) &&
	    !fmc100_nand_support_randomizer(host->pagesize,
					      host->ecctype)) {
		db_bug(ERSTR_HARDWARE
		       "This NAND flash does not support `randomizer`, "
		       "Please don't configure hardware randomizer PIN.");
	}
	/* Save value of FMC_CFG and FMC_CFG_ECC0 to turn on/off ECC */
	reg_fmc_cfg = fmc_readl(host, FMC_CFG);
	reg_fmc_cfg &= ~(PAGE_SIZE_MASK | ECC_TYPE_MASK | BLOCK_SIZE_MASK);
	reg_fmc_cfg |= ecc_reg | page_reg | block_reg;
	host->nand_cfg = reg_fmc_cfg;
	host->nand_cfg_ecc0 = (host->nand_cfg & ~ECC_TYPE_MASK) | ECC_TYPE_0BIT;
	fmc_pr(BT_DBG, "\t|-Save FMC_CFG[%#x]: %#x and FMC_CFG_ECC0: %#x\n",
	       FMC_CFG, host->nand_cfg, host->nand_cfg_ecc0);

	/* pass pagesize and ecctype to kernel when spiflash startup. */
	host->enable_ecc_randomizer(host, ENABLE, ENABLE);

	/*
	 * If it want to support the 'read retry' feature, the 'randomizer'
	 * feature must be support first.
	 */
	host->read_retry = NULL;

	if (host->read_retry && !IS_NAND_RANDOM(host)) {
		db_bug(ERSTR_HARDWARE
		       "This Nand flash need to enable 'randomizer' feature. "
		       "Please configure hardware randomizer PIN.");
	}
}

static void fmc100_set_oob_info(struct mtd_info *mtd,
				  struct nand_config_info *info, struct nand_dev_t *nand_dev)
{
	int buffer_len;
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct fmc_host *host = chip->priv;
	struct mtd_oob_region fmc_oobregion = {0, 0};
	if (info->ecctype != NAND_ECC_0BIT)
		mtd->oobsize = info->oobsize;

	mtd->oobavail = FMC100_NAND_OOBSIZE_FOR_YAFFS;

	host->oobsize = mtd->oobsize;

	buffer_len = host->pagesize + host->oobsize;
	/* dest buffer just need init buffer_len */
	if (memset_s(host->buffer, FMC_MAX_DMA_LEN, 0xff, buffer_len)) {
		printk("%s %d:memset_s failed\n", __func__, __LINE__);
		return;
	}
	host->dma_oob = host->dma_buffer + host->pagesize;

	host->bbm = (unsigned char *)(host->buffer + host->pagesize +
				      FMC100_BAD_BLOCK_POS);

	info->ooblayout_ops->free(mtd, 0, &fmc_oobregion);

	mtd_set_ooblayout(mtd, info->ooblayout_ops);

	/* EB bits locate in the bottom two of CTRL(30) */
	host->epm = (u_short *)(host->buffer + host->pagesize +
				fmc_oobregion.offset +
				FMC_OOB_LEN_30_EB_OFFSET);

#ifdef CONFIG_BSP_NAND_FS_MAY_NO_YAFFS2
	if (info->ecctype == NAND_ECC_16BIT) {
		if (host->pagesize == _2K) {
			/* EB bits locate in the bottom two of CTRL(6) */
			host->epm = (u_short *)(host->buffer + host->pagesize +
						fmc_oobregion.offset +
						FMC_OOB_LEN_6_EB_OFFSET);
		} else if (host->pagesize == _4K) {
			/* EB bit locate in the bottom two of CTRL(14) */
			host->epm = (u_short *)(host->buffer + host->pagesize +
						fmc_oobregion.offset +
						FMC_OOB_LEN_14_EB_OFFSET);
		}
	}
#endif
}

static int fmc100_set_config_info(struct mtd_info *mtd,
				    struct nand_chip *chip, struct nand_dev_t *dev)
{
	struct fmc_host *host = chip->priv;
	struct nand_dev_t *nand_dev = dev;
	struct nand_config_info *type_info = NULL;

	fmc_pr(BT_DBG, "\t*-Start config Block Page OOB and Ecc\n");

	type_info = fmc100_get_config_type_info(mtd, nand_dev);
	WARN_ON(!type_info);

	fmc_pr(BT_DBG, "\t|-%s Config, PageSize %s EccType %s OobSize %d\n",
	       nand_dev->start_type, nand_page_name(type_info->pagetype),
	       nand_ecc_name(type_info->ecctype), type_info->oobsize);

	/* Set the page_size, ecc_type, block_size of FMC_CFG[0x0] register */
	fmc100_set_fmc_cfg_reg(host, type_info, nand_dev);

	fmc100_set_oob_info(mtd, type_info, nand_dev);

	if (mtd->writesize > NAND_MAX_PAGESIZE ||
	    mtd->oobsize > NAND_MAX_OOBSIZE) {
		db_bug(ERSTR_DRIVER
		       "Driver does not support this Nand Flash. Please " \
		       "increase NAND_MAX_PAGESIZE and NAND_MAX_OOBSIZE.\n");
	}

	/* Some Nand Flash devices have subpage structure */
	if (mtd->writesize != host->pagesize) {
		unsigned int shift = 0;
		unsigned int writesize = mtd->writesize;

		while (writesize > host->pagesize) {
			writesize >>= 1;
			shift++;
		}
		mtd->erasesize = mtd->erasesize >> shift;
		mtd->writesize = host->pagesize;
		pr_info("Nand divide into 1/%u\n", (1 << shift));
	}

	fmc_pr(BT_DBG, "\t*-End config Block Page Oob and Ecc\n");

	return 0;
}

static void fmc100_chip_init(struct nand_chip *chip)
{
	struct fmc_host *host = chip->priv;
	/* dest fmcbuf just need init dma_len lenth */
	if (memset_s((char *)chip->legacy.IO_ADDR_R, FMC_MEM_LEN, 0xff, host->dma_len)) {
		printk("%s %d:memset_s failed\n", __func__, __LINE__);
		return;
	}

	chip->legacy.read_byte = fmc100_read_byte;

	chip->legacy.write_buf = fmc100_write_buf;
	chip->legacy.read_buf = fmc100_read_buf;

	chip->legacy.select_chip = fmc100_select_chip;

	chip->legacy.cmd_ctrl = fmc100_cmd_ctrl;
	chip->legacy.dev_ready = fmc100_dev_ready;

	chip->legacy.chip_delay = FMC_CHIP_DELAY;

	chip->options = NAND_NEED_READRDY | NAND_BROKEN_XD |
			NAND_SKIP_BBTSCAN;
	chip->ecc.engine_type = NAND_ECC_ENGINE_TYPE_NONE;
}

static void fmc100_host_fun_init(struct fmc_host *host)
{
	host->send_cmd_pageprog = fmc100_send_cmd_write;
	host->send_cmd_status = fmc100_send_cmd_status;
	host->send_cmd_readstart = fmc100_send_cmd_read;
	host->send_cmd_erase = fmc100_send_cmd_erase;
	host->send_cmd_readid = fmc100_send_cmd_readid;
	host->send_cmd_reset = fmc100_send_cmd_reset;
}

static inline unsigned int get_sys_boot_mode(unsigned int reg_val)
{
	return (reg_val >> 4) & 0x3; /* 4: boot mode */
}

static int fmc100_host_init(struct fmc_host *host)
{
	unsigned int reg, flash_type;

	fmc_pr(BT_DBG, "\t *-Start nand host init\n");

	reg = fmc_readl(host, FMC_CFG);
	fmc_pr(BT_DBG, "\t |-Read FMC CFG[%#x]%#x\n", FMC_CFG, reg);
	flash_type = get_spi_flash_type(reg);
	if (flash_type != FLASH_TYPE_NAND) {
		db_msg("Error: Flash type isn't Nand flash. reg[%#x]\n", reg);
		reg |= fmc_cfg_flash_sel(FLASH_TYPE_NAND);
		fmc_pr(BT_DBG, "\t |-Change flash type to Nand flash\n");
	}

	if ((reg & FMC_CFG_OP_MODE_MASK) == FMC_CFG_OP_MODE_BOOT) {
		reg |= fmc_cfg_op_mode(FMC_CFG_OP_MODE_NORMAL);
		fmc_pr(BT_DBG, "\t |-Controller enter normal mode\n");
	}
	fmc_writel(host, FMC_CFG, reg);
	fmc_pr(BT_DBG, "\t |-Set CFG[%#x]%#x\n", FMC_CFG, reg);

	host->nand_cfg = reg;
	host->nand_cfg_ecc0 = (reg & ~ECC_TYPE_MASK) | ECC_TYPE_0BIT;

	reg = fmc_readl(host, FMC_GLOBAL_CFG);
	fmc_pr(BT_DBG, "\t |-Read global CFG[%#x]%#x\n", FMC_GLOBAL_CFG, reg);
	if (reg & FMC_GLOBAL_CFG_RANDOMIZER_EN) {
		host->flags &= ~NAND_RANDOMIZER;
		fmc_pr(BT_DBG, "\t |-Default disable randomizer\n");
		reg &= ~FMC_GLOBAL_CFG_RANDOMIZER_EN;
		fmc_writel(host, FMC_GLOBAL_CFG, reg);
		fmc_pr(BT_DBG, "\t |-Set global CFG[%#x]%#x\n", FMC_GLOBAL_CFG, reg);
	}

#ifdef CONFIG_FMC100_NAND_EDO_MODE
	/* enable EDO node */
	reg = fmc_readl(host, FMC_GLOBAL_CFG);
	fmc_writel(host, FMC_GLOBAL_CFG, set_nand_edo_mode_en(reg));
#endif

	host->addr_cycle = 0;
	host->addr_value[0] = 0;
	host->addr_value[1] = 0;
	host->cache_addr_value[0] = ~0;
	host->cache_addr_value[1] = ~0;

	fmc100_host_fun_init(host);

	/*
	 * check if start from nand.
	 * This register REG_SYSSTAT is set in start.S
	 * When start in NAND (Auto), the ECC/PAGESIZE driver don't detect.
	 */
	host->flags |= NANDC_HW_AUTO;

	if (get_sys_boot_mode(reg) == BOOT_FROM_NAND) {
		host->flags |= NANDC_CONFIG_DONE;
		fmc_pr(BT_DBG, "\t |-Auto config pagesize and ecctype\n");
	}

	host->enable_ecc_randomizer = fmc100_ecc_randomizer;

	fmc_pr(BT_DBG, "\t *-End nand host init\n");

	return 0;
}

int fmc100_nand_init(struct nand_chip *chip)
{
	struct fmc_host *host = chip->priv;

	/* enable and set system clock */
	clk_prepare_enable(host->clk);

	/* fmc ip version check */
	host->version = fmc_readl(host, FMC_VERSION);
	if (host->version != FMC_VER_100)
		return -EFAULT;

	pr_info("Found Flash Memory Controller v100 Nand Driver\n");

	/* fmc host init */
	if (fmc100_host_init(host)) {
		db_msg("Error: Nand host init failed!\n");
		return -EFAULT;
	}
	host->chip = chip;

	fmc_writel(host,
		     FMC_PND_PWIDTH_CFG,
		     pwidth_cfg_rw_hcnt(CONFIG_RW_H_WIDTH) |
		     pwidth_cfg_r_lcnt(CONFIG_R_L_WIDTH) |
		     pwidth_cfg_w_lcnt(CONFIG_W_L_WIDTH));

	/* fmc nand_chip struct init */
	fmc100_chip_init(chip);

	fmc_spl_ids_register();
	nfc_param_adjust = fmc100_set_config_info;

	return 0;
}

#ifdef CONFIG_PM

void fmc100_nand_config(const struct fmc_host *host)
{
	/* enable system clock */
	clk_prepare_enable(host->clk);
	fmc_pr(PM_DBG, "\t |-enable system clock\n");
}
#endif  /* CONFIG_PM */
