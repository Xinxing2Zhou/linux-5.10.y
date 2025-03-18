/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <asm/setup.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <linux/mfd/bsp_fmc.h>
#include <linux/securec.h>
#include "fmc100.h"

static void fmc100_switch_to_spi_nand(struct fmc_host *host)
{
	u32 reg;

	reg = fmc_readl(host, FMC_CFG);
	reg &= ~FLASH_TYPE_SEL_MASK;
	reg |= fmc_cfg_flash_sel(FLASH_TYPE_SPI_NAND);
	fmc_writel(host, FMC_CFG, reg);
}

static void fmc100_set_str_mode(const struct fmc_host *host)
{
	u32 reg;

	reg = fmc_readl(host, FMC_GLOBAL_CFG);
	reg &= (~FMC_GLOBAL_CFG_DTR_MODE);
	fmc_writel(host, FMC_GLOBAL_CFG, reg);
}

static void hifmc100_voltage_state_check(unsigned long *clkrate)
{
	int nfc_voltage_state = NFC_3_3_STATA;
	unsigned int value = 0;
	void __iomem *sys_ctrl_state_reg = NULL;
	if (clkrate == NULL) {
		return;
	}
	if ((*clkrate) <= (u_long)clk_fmc_to_crg_mhz(NFC_CLK_75MHZ)) {
		return;
	}
	sys_ctrl_state_reg = (void *)ioremap(SYS_CTRL_REG_BASE + SYS_CTRL_STATE_OFF, SYS_CTRL_STATE_SIZE);
	if (sys_ctrl_state_reg == NULL) {
		fmc_pr(PM_DBG, "|-ERR:sys state ioremap fail\n");
		return;
	}
	value = readl(sys_ctrl_state_reg);
	if (((value >> NFC_STATE_BIT_OFF) & NFC_STATA_MASK) == NFC_1_8_STATA) {
		nfc_voltage_state = NFC_1_8_STATA;
		fmc_pr(PM_DBG, "|-nfc voltage is 1.8v\n");
	}
	if ((nfc_voltage_state == NFC_1_8_STATA) && \
			(*clkrate > (u_long)clk_fmc_to_crg_mhz(NFC_CLK_75MHZ))) {
		*clkrate = (u_long)clk_fmc_to_crg_mhz(NFC_CLK_75MHZ);
		fmc_pr(PM_DBG, "|-nfc clk is set to 75MHZ\n");
	}
	iounmap(sys_ctrl_state_reg);
}

static void fmc100_operation_config(struct fmc_host *host, int op)
{
	int ret;
	unsigned long clkrate = 0;
	struct fmc_spi *spi = host->spi;

	fmc100_switch_to_spi_nand(host);
	clk_prepare_enable(host->clk);
	switch (op) {
	case OP_STYPE_WRITE:
		clkrate = min((u_long)host->clkrate,
			      (u_long)clk_fmc_to_crg_mhz(spi->write->clock));
		break;
	case OP_STYPE_READ:
		clkrate = min((u_long)host->clkrate,
			      (u_long)clk_fmc_to_crg_mhz(spi->read->clock));
		break;
	case OP_STYPE_ERASE:
		clkrate = min((u_long)host->clkrate,
			      (u_long)clk_fmc_to_crg_mhz(spi->erase->clock));
		break;
	default:
		break;
	}

	hifmc100_voltage_state_check(&clkrate);

	ret = clk_set_rate(host->clk, clkrate);
	if (WARN_ON((ret != 0)))
		pr_err("clk_set_rate failed: %d\n", ret);
}

static void fmc100_dma_wr_addr_config(struct fmc_host *host)
{
	unsigned int reg;

#ifndef FMC100_SPI_NAND_SUPPORT_REG_WRITE
	reg = host->dma_buffer;
	fmc_writel(host, FMC_DMA_SADDR_D0, reg);
	fmc_pr(WR_DBG, "|-Set DMA_SADDR_D[0x40]%#x\n", reg);

#ifdef CONFIG_64BIT
	reg = (host->dma_buffer & FMC_DMA_SADDRH_MASK) >>
		FMC_DMA_BIT_SHIFT_LENTH;
	fmc_writel(host, FMC_DMA_SADDRH_D0, reg);
	fmc_pr(WR_DBG, "\t|-Set DMA_SADDRH_D0[%#x]%#x\n", FMC_DMA_SADDRH_D0, reg);
#endif

	reg = host->dma_oob;
	fmc_writel(host, FMC_DMA_SADDR_OOB, reg);
	fmc_pr(WR_DBG, "|-Set DMA_SADDR_OOB[%#x]%#x\n", FMC_DMA_SADDR_OOB, reg);
#ifdef CONFIG_64BIT
	reg = (host->dma_oob & FMC_DMA_SADDRH_MASK) >>
		FMC_DMA_BIT_SHIFT_LENTH;
	fmc_writel(host, FMC_DMA_SADDRH_OOB, reg);
	fmc_pr(WR_DBG, "\t|-Set DMA_SADDRH_OOB[%#x]%#x\n", FMC_DMA_SADDRH_OOB,
	       reg);
#endif
#endif
}

static void fmc100_dma_wr_op_config(struct fmc_host *host, const struct fmc_spi *spi)
{
	unsigned int reg;
	unsigned int block_num;
	unsigned int block_num_h;
	unsigned int page_num;
	unsigned char pages_per_block_shift;
	struct nand_chip *chip = host->chip;

	reg = FMC_INT_CLR_ALL;
	fmc_writel(host, FMC_INT_CLR, reg);
	fmc_pr(WR_DBG, "|-Set INT_CLR[%#x]%#x\n", FMC_INT_CLR, reg);

	reg = op_cfg_fm_cs(host->cmd_op.cs) |
	      op_cfg_mem_if_type(spi->write->iftype) |
	      OP_CFG_OEN_EN;
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(WR_DBG, "|-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	pages_per_block_shift = chip->phys_erase_shift - chip->page_shift;
	block_num = host->addr_value[1] >> pages_per_block_shift;
	block_num_h = block_num >> REG_CNT_HIGH_BLOCK_NUM_SHIFT;
	reg = fmc_addrh_set(block_num_h);
	fmc_writel(host, FMC_ADDRH, reg);
	fmc_pr(WR_DBG, "|-Set ADDRH[%#x]%#x\n", FMC_ADDRH, reg);

	page_num = host->addr_value[1] - (block_num << pages_per_block_shift);
	reg = ((block_num & REG_CNT_BLOCK_NUM_MASK) << REG_CNT_BLOCK_NUM_SHIFT) |
	      ((page_num & REG_CNT_PAGE_NUM_MASK) << REG_CNT_PAGE_NUM_SHIFT);
	fmc_writel(host, FMC_ADDRL, reg);
	fmc_pr(WR_DBG, "|-Set ADDRL[%#x]%#x\n", FMC_ADDRL, reg);

	*host->epm = 0x0000;

	fmc100_dma_wr_addr_config(host);

	reg = op_ctrl_wr_opcode(spi->write->cmd) |
#ifdef FMC100_SPI_NAND_SUPPORT_REG_WRITE
	      op_ctrl_dma_op(OP_TYPE_REG) |
#else
	      op_ctrl_dma_op(OP_TYPE_DMA) |
#endif
	      op_ctrl_rw_op(RW_OP_WRITE) |
	      OP_CTRL_DMA_OP_READY;
	fmc_writel(host, FMC_OP_CTRL, reg);
	fmc_pr(WR_DBG, "|-Set OP_CTRL[%#x]%#x\n", FMC_OP_CTRL, reg);

	fmc_dma_wait_int_finish(host);
}

static void fmc100_send_cmd_write(struct fmc_host *host)
{
	int ret;
	struct fmc_spi *spi = host->spi;
#ifdef FMC100_SPI_NAND_SUPPORT_REG_WRITE
	const char *op = "Reg";
#else
	const char *op = "Dma";
#endif

	if (WR_DBG)
		pr_info("\n");
	fmc_pr(WR_DBG, "*-Start send %s page write command\n", op);

	mutex_lock(host->lock);
	fmc100_operation_config(host, OP_STYPE_WRITE);

	ret = spi->driver->wait_ready(spi);
	if (ret) {
		db_msg("Error: %s program wait ready failed! status: %#x\n",
		       op, ret);
		goto end;
	}

	ret = spi->driver->write_enable(spi);
	if (ret) {
		db_msg("Error: %s program write enable failed! ret: %#x\n",
		       op, ret);
		goto end;
	}

	fmc100_dma_wr_op_config(host, spi);

end:
	mutex_unlock(host->lock);
	fmc_pr(WR_DBG, "*-End %s page program!\n", op);
}

static void fmc100_send_cmd_status(struct fmc_host *host)
{
	u_char status;
	int ret;
	unsigned char addr = STATUS_ADDR;
	struct fmc_spi *spi = NULL;

	if (host == NULL || host->spi == NULL) {
		db_msg("Error: host or host->spi is NULL!\n");
		return;
	}
	spi = host->spi;
	if (host->cmd_op.l_cmd == NAND_CMD_GET_FEATURES)
		addr = PROTECT_ADDR;

	ret = spi_nand_feature_op(spi, GET_OP, addr, &status);
	if (ret)
		return;
	fmc_pr((ER_DBG || WR_DBG), "\t*-Get status[%#x]: %#x\n", addr, status);
}

static void fmc100_dma_rd_addr_config(struct fmc_host *host)
{
	unsigned int reg;

#ifndef FMC100_SPI_NAND_SUPPORT_REG_READ
	reg = host->dma_buffer;
	fmc_writel(host, FMC_DMA_SADDR_D0, reg);
	fmc_pr(RD_DBG, "\t|-Set DMA_SADDR_D0[%#x]%#x\n", FMC_DMA_SADDR_D0, reg);

#ifdef CONFIG_64BIT
	reg = (host->dma_buffer & FMC_DMA_SADDRH_MASK) >>
		FMC_DMA_BIT_SHIFT_LENTH;
	fmc_writel(host, FMC_DMA_SADDRH_D0, reg);
	fmc_pr(RD_DBG, "\t|-Set DMA_SADDRH_D0[%#x]%#x\n", FMC_DMA_SADDRH_D0, reg);
#endif

	reg = host->dma_oob;
	fmc_writel(host, FMC_DMA_SADDR_OOB, reg);
	fmc_pr(RD_DBG, "\t|-Set DMA_SADDR_OOB[%#x]%#x\n", FMC_DMA_SADDR_OOB,
	       reg);

#ifdef CONFIG_64BIT
	reg = (host->dma_oob & FMC_DMA_SADDRH_MASK) >>
		FMC_DMA_BIT_SHIFT_LENTH;
	fmc_writel(host, FMC_DMA_SADDRH_OOB, reg);
	fmc_pr(RD_DBG, "\t|-Set DMA_SADDRH_OOB[%#x]%#x\n", FMC_DMA_SADDRH_OOB,
	       reg);
#endif
#endif
}

static void fmc100_dma_rd_op_config(struct fmc_host *host, const struct fmc_spi *spi)
{
	unsigned int reg;
	unsigned int block_num;
	unsigned int block_num_h;
	unsigned int page_num;
	unsigned char pages_per_block_shift;
	struct nand_chip *chip = host->chip;

	reg = FMC_INT_CLR_ALL;
	fmc_writel(host, FMC_INT_CLR, reg);
	fmc_pr(RD_DBG, "\t|-Set INT_CLR[%#x]%#x\n", FMC_INT_CLR, reg);

	if (host->cmd_op.l_cmd == NAND_CMD_READOOB)
		host->cmd_op.op_cfg = op_ctrl_rd_op_sel(RD_OP_READ_OOB);
	else
		host->cmd_op.op_cfg = op_ctrl_rd_op_sel(RD_OP_READ_ALL_PAGE);

	reg = op_cfg_fm_cs(host->cmd_op.cs) |
	      op_cfg_mem_if_type(spi->read->iftype) |
	      op_cfg_dummy_num(spi->read->dummy) |
	      OP_CFG_OEN_EN;
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(RD_DBG, "\t|-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	pages_per_block_shift = chip->phys_erase_shift - chip->page_shift;
	block_num = host->addr_value[1] >> pages_per_block_shift;
	block_num_h = block_num >> REG_CNT_HIGH_BLOCK_NUM_SHIFT;

	reg = fmc_addrh_set(block_num_h);
	fmc_writel(host, FMC_ADDRH, reg);
	fmc_pr(RD_DBG, "\t|-Set ADDRH[%#x]%#x\n", FMC_ADDRH, reg);

	page_num = host->addr_value[1] - (block_num << pages_per_block_shift);

	reg = ((block_num & REG_CNT_BLOCK_NUM_MASK) << REG_CNT_BLOCK_NUM_SHIFT) |
	      ((page_num & REG_CNT_PAGE_NUM_MASK) << REG_CNT_PAGE_NUM_SHIFT);
	fmc_writel(host, FMC_ADDRL, reg);
	fmc_pr(RD_DBG, "\t|-Set ADDRL[%#x]%#x\n", FMC_ADDRL, reg);

	fmc100_dma_rd_addr_config(host);

	reg = op_ctrl_rd_opcode(spi->read->cmd) | host->cmd_op.op_cfg |
#ifdef FMC100_SPI_NAND_SUPPORT_REG_READ
	      op_ctrl_dma_op(OP_TYPE_REG) |
#else
	      op_ctrl_dma_op(OP_TYPE_DMA) |
#endif
	      op_ctrl_rw_op(RW_OP_READ) | OP_CTRL_DMA_OP_READY;
	fmc_writel(host, FMC_OP_CTRL, reg);
	fmc_pr(RD_DBG, "\t|-Set OP_CTRL[%#x]%#x\n", FMC_OP_CTRL, reg);

	fmc_dma_wait_int_finish(host);
}

static void fmc100_send_cmd_read(struct fmc_host *host)
{
	struct fmc_spi *spi = host->spi;
	int ret;

#ifdef FMC100_SPI_NAND_SUPPORT_REG_READ
	char *op = "Reg";
#else
	char *op = "Dma";
#endif

	if (RD_DBG)
		pr_info("\n");

	fmc_pr(RD_DBG, "\t*-Start %s page read\n", op);

	if ((host->addr_value[0] == host->cache_addr_value[0])
	    && (host->addr_value[1] == host->cache_addr_value[1])) {
		fmc_pr(RD_DBG, "\t*-%s read cache hit, addr[%#x %#x]\n",
		       op, host->addr_value[1], host->addr_value[0]);
		return;
	}

	mutex_lock(host->lock);
	fmc100_operation_config(host, OP_STYPE_READ);

	fmc_pr(RD_DBG, "\t|-Wait ready before %s page read\n", op);
	ret = spi->driver->wait_ready(spi);
	if (ret) {
		db_msg("Error: %s read wait ready fail! ret: %#x\n", op, ret);
		goto end;
	}

	fmc100_dma_rd_op_config(host, spi);

	host->cache_addr_value[0] = host->addr_value[0];
	host->cache_addr_value[1] = host->addr_value[1];

end:
	mutex_unlock(host->lock);
	fmc_pr(RD_DBG, "\t*-End %s page read\n", op);
}

static void fmc100_send_cmd_erase(struct fmc_host *host)
{
	unsigned int reg;
	struct fmc_spi *spi = host->spi;
	int ret;

	if (ER_DBG)
		pr_info("\n");

	fmc_pr(ER_DBG, "\t*-Start send cmd erase!\n");

	mutex_lock(host->lock);
	fmc100_operation_config(host, OP_STYPE_ERASE);

	ret = spi->driver->wait_ready(spi);
	fmc_pr(ER_DBG, "\t|-Erase wait ready, ret: %#x\n", ret);
	if (ret) {
		db_msg("Error: Erase wait ready fail! status: %#x\n", ret);
		goto end;
	}

	ret = spi->driver->write_enable(spi);
	if (ret) {
		db_msg("Error: Erase write enable failed! ret: %#x\n", ret);
		goto end;
	}

	reg = FMC_INT_CLR_ALL;
	fmc_writel(host, FMC_INT_CLR, reg);
	fmc_pr(ER_DBG, "\t|-Set INT_CLR[%#x]%#x\n", FMC_INT_CLR, reg);

	reg = spi->erase->cmd;
	fmc_writel(host, FMC_CMD, fmc_cmd_cmd1(reg));
	fmc_pr(ER_DBG, "\t|-Set CMD[%#x]%#x\n", FMC_CMD, reg);

	reg = fmc_addrl_block_h_mask(host->addr_value[1]) |
	      fmc_addrl_block_l_mask(host->addr_value[0]);
	fmc_writel(host, FMC_ADDRL, reg);
	fmc_pr(ER_DBG, "\t|-Set ADDRL[%#x]%#x\n", FMC_ADDRL, reg);

	reg = op_cfg_fm_cs(host->cmd_op.cs) |
	      op_cfg_mem_if_type(spi->erase->iftype) |
	      op_cfg_addr_num(STD_OP_ADDR_NUM) |
	      op_cfg_dummy_num(spi->erase->dummy) |
	      OP_CFG_OEN_EN;
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(ER_DBG, "\t|-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	reg = FMC_OP_CMD1_EN |
	      FMC_OP_ADDR_EN |
	      FMC_OP_REG_OP_START;
	fmc_writel(host, FMC_OP, reg);
	fmc_pr(ER_DBG, "\t|-Set OP[%#x]%#x\n", FMC_OP, reg);

	fmc_cmd_wait_cpu_finish(host);

end:
	mutex_unlock(host->lock);
	fmc_pr(ER_DBG, "\t*-End send cmd erase!\n");
}

void fmc100_ecc0_switch(struct fmc_host *host, unsigned char op)
{
	unsigned int config;
#if EC_DBG
	unsigned int cmp_cfg;

	if (host == NULL) {
		db_msg("Error: host is NULL!\n");
		return;
	}
	config = fmc_readl(host, FMC_CFG);
	fmc_pr(EC_DBG, "\t *-Get CFG[%#x]%#x\n", FMC_CFG, config);

	if (op)
		cmp_cfg = host->fmc_cfg;
	else
		cmp_cfg = host->fmc_cfg_ecc0;

	if (cmp_cfg != config)
		db_msg("Warning: FMC config[%#x] is different.\n",
		       cmp_cfg);
#endif
	if (host == NULL) {
		db_msg("Error: host is NULL!\n");
		return;
	}
	if (op == ENABLE) {
		config = host->fmc_cfg_ecc0;
	} else if (op == DISABLE) {
		config = host->fmc_cfg;
	} else {
		db_msg("Error: Invalid opcode: %d\n", op);
		return;
	}

	fmc_writel(host, FMC_CFG, config);
	fmc_pr(EC_DBG, "\t *-Set CFG[%#x]%#x\n", FMC_CFG, config);
}

static void fmc100_send_cmd_readid(struct fmc_host *host)
{
	unsigned int reg;

	fmc_pr(BT_DBG, "\t|*-Start send cmd read ID\n");

	fmc100_ecc0_switch(host, ENABLE);

	reg = fmc_cmd_cmd1(SPI_CMD_RDID);
	fmc_writel(host, FMC_CMD, reg);
	fmc_pr(BT_DBG, "\t||-Set CMD[%#x]%#x\n", FMC_CMD, reg);

	reg = READ_ID_ADDR;
	fmc_writel(host, FMC_ADDRL, reg);
	fmc_pr(BT_DBG, "\t||-Set ADDRL[%#x]%#x\n", FMC_ADDRL, reg);

	reg = op_cfg_fm_cs(host->cmd_op.cs) |
	      op_cfg_addr_num(READ_ID_ADDR_NUM) |
	      OP_CFG_OEN_EN;
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(BT_DBG, "\t||-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	reg = fmc_data_num_cnt(MAX_SPI_NAND_ID_LEN);
	fmc_writel(host, FMC_DATA_NUM, reg);
	fmc_pr(BT_DBG, "\t||-Set DATA_NUM[%#x]%#x\n", FMC_DATA_NUM, reg);

	reg = FMC_OP_CMD1_EN |
	      FMC_OP_ADDR_EN |
	      FMC_OP_READ_DATA_EN |
	      FMC_OP_REG_OP_START;
	fmc_writel(host, FMC_OP, reg);
	fmc_pr(BT_DBG, "\t||-Set OP[%#x]%#x\n", FMC_OP, reg);

	host->addr_cycle = 0x0;

	fmc_cmd_wait_cpu_finish(host);

	fmc100_ecc0_switch(host, DISABLE);

	fmc_pr(BT_DBG, "\t|*-End read flash ID\n");
}

static void fmc100_send_cmd_reset(struct fmc_host *host)
{
	unsigned int reg;

	fmc_pr(BT_DBG, "\t|*-Start send cmd reset\n");

	reg = fmc_cmd_cmd1(SPI_CMD_RESET);
	fmc_writel(host, FMC_CMD, reg);
	fmc_pr(BT_DBG, "\t||-Set CMD[%#x]%#x\n", FMC_CMD, reg);

	reg = op_cfg_fm_cs(host->cmd_op.cs) | OP_CFG_OEN_EN;
	fmc_writel(host, FMC_OP_CFG, reg);
	fmc_pr(BT_DBG, "\t||-Set OP_CFG[%#x]%#x\n", FMC_OP_CFG, reg);

	reg = FMC_OP_CMD1_EN | FMC_OP_REG_OP_START;
	fmc_writel(host, FMC_OP, reg);
	fmc_pr(BT_DBG, "\t||-Set OP[%#x]%#x\n", FMC_OP, reg);

	fmc_cmd_wait_cpu_finish(host);

	fmc_pr(BT_DBG, "\t|*-End send cmd reset\n");
}

static void fmc100_host_init(struct fmc_host *host)
{
	unsigned int reg;

	fmc_pr(BT_DBG, "\t||*-Start SPI Nand host init\n");

	reg = fmc_readl(host, FMC_CFG);
	if ((reg & FMC_CFG_OP_MODE_MASK) == FMC_CFG_OP_MODE_BOOT) {
		reg |= fmc_cfg_op_mode(FMC_CFG_OP_MODE_NORMAL);
		fmc_writel(host, FMC_CFG, reg);
		fmc_pr(BT_DBG, "\t|||-Set CFG[%#x]%#x\n", FMC_CFG, reg);
	}

	host->fmc_cfg = reg;
	host->fmc_cfg_ecc0 = (reg & ~ECC_TYPE_MASK) | ECC_TYPE_0BIT;

	reg = fmc_readl(host, FMC_GLOBAL_CFG);
	if (reg & FMC_GLOBAL_CFG_WP_ENABLE) {
		reg &= ~FMC_GLOBAL_CFG_WP_ENABLE;
		fmc_writel(host, FMC_GLOBAL_CFG, reg);
	}

	host->addr_cycle = 0;
	host->addr_value[0] = 0;
	host->addr_value[1] = 0;
	host->cache_addr_value[0] = ~0;
	host->cache_addr_value[1] = ~0;

	host->send_cmd_write = fmc100_send_cmd_write;
	host->send_cmd_status = fmc100_send_cmd_status;
	host->send_cmd_read = fmc100_send_cmd_read;
	host->send_cmd_erase = fmc100_send_cmd_erase;
	host->send_cmd_readid = fmc100_send_cmd_readid;
	host->send_cmd_reset = fmc100_send_cmd_reset;
#ifdef CONFIG_PM
	host->suspend = fmc100_suspend;
	host->resume  = fmc100_resume;
#endif

	reg = timing_cfg_tcsh(CS_HOLD_TIME) |
	      timing_cfg_tcss(CS_SETUP_TIME) |
	      timing_cfg_tshsl(CS_DESELECT_TIME);
	fmc_writel(host, FMC_SPI_TIMING_CFG, reg);

	reg = ALL_BURST_ENABLE;
	fmc_writel(host, FMC_DMA_AHB_CTRL, reg);

	fmc_pr(BT_DBG, "\t||*-End SPI Nand host init\n");
}

static unsigned char fmc100_read_byte(struct nand_chip *chip)
{
	struct fmc_host *host = chip->priv;
	unsigned char value;
	unsigned char ret_val = 0;

	if (host->cmd_op.l_cmd == NAND_CMD_READID) {
		value = fmc_readb(host->iobase + host->offset);
		host->offset++;
		if (host->cmd_op.data_no == host->offset)
			host->cmd_op.l_cmd = 0;

		return value;
	}

	if (host->cmd_op.cmd == NAND_CMD_STATUS) {
		value = fmc_readl(host, FMC_STATUS);
		if (host->cmd_op.l_cmd == NAND_CMD_GET_FEATURES) {
			fmc_pr((ER_DBG || WR_DBG), "\t\tRead BP status:%#x\n",
			       value);
			if (any_bp_enable(value))
				ret_val |= NAND_STATUS_WP;

			host->cmd_op.l_cmd = NAND_CMD_STATUS;
		}

		if ((value & STATUS_OIP_MASK) == 0)
			ret_val |= NAND_STATUS_READY;

		if (value & STATUS_E_FAIL_MASK) {
			fmc_pr(ER_DBG, "\t\tGet erase status: %#x\n", value);
			ret_val |= NAND_STATUS_FAIL;
		}

		if (value & STATUS_P_FAIL_MASK) {
			fmc_pr(WR_DBG, "\t\tGet write status: %#x\n", value);
			ret_val |= NAND_STATUS_FAIL;
		}

		return ret_val;
	}

	if (host->cmd_op.l_cmd == NAND_CMD_READOOB) {
		value  = fmc_readb(host->buffer + host->pagesize + host->offset);
		host->offset++;
		return value;
	}

	host->offset++;

	return fmc_readb(host->buffer + host->column + host->offset - 1);
}

static void fmc100_write_buf(struct nand_chip *chip,
			       const u_char *buf, int len)
{
	struct fmc_host *host = chip->priv;
	int ret;

#ifdef FMC100_SPI_NAND_SUPPORT_REG_WRITE
	if (buf == chip->oob_poi)
		ret = memcpy_s((char *)host->iobase + host->pagesize,
			FMC_MEM_LEN, buf, len);
	else
		ret = memcpy_s((char *)host->iobase, FMC_MEM_LEN, buf, len);

#else
	if (buf == chip->oob_poi)
		ret = memcpy_s((char *)(host->buffer + host->pagesize),
			FMC_MAX_DMA_LEN, buf, len);
	else
		ret = memcpy_s((char *)host->buffer, FMC_MAX_DMA_LEN, buf, len);

#endif
	if (ret)
		printk("%s:memcpy_s failed\n", __func__);

	return;
}

static void fmc100_read_buf(struct nand_chip *chip, u_char *buf, int len)
{
	struct fmc_host *host = chip->priv;
	int ret;

#ifdef FMC100_SPI_NAND_SUPPORT_REG_READ
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
	if (ret) {
		printk("%s %d:memcpy_s failed\n", __func__, __LINE__);
		return;
	}

#ifdef CONFIG_BSP_NAND_ECC_STATUS_REPORT
	if (buf != chip->oob_poi) {
		u_int reg;
		u_int ecc_step = host->pagesize >> ECC_STEP_SHIFT;

		reg = fmc_readl(host, FMC100_ECC_ERR_NUM0_BUF0);
		while (ecc_step) {
			u_char err_num;

			err_num = get_ecc_err_num(--ecc_step, reg);
			if (err_num == 0xff)
				mtd->ecc_stats.failed++;
			else
				mtd->ecc_stats.corrected += err_num;
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

	if (chipselect > CONFIG_SPI_NAND_MAX_CHIP_NUM)
		db_bug("Error: Invalid chipselect: %d\n", chipselect);

	if (host->mtd != mtd) {
		host->mtd = mtd;
		host->cmd_op.cs = chipselect;
	}
}

static void fmc100_ale_init(struct fmc_host *host, unsigned ctrl, unsigned int udat)
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
		((udat & 0xff) << addr_offset);

	host->addr_cycle++;
}

static void fmc100_cle_init(struct fmc_host *host,
				unsigned ctrl,
				unsigned int udat,
				int *is_cache_invalid)
{
	unsigned char cmd;

	cmd = udat & 0xff;
	host->cmd_op.cmd = cmd;
	switch (cmd) {
	case NAND_CMD_PAGEPROG:
		host->offset = 0;
		host->send_cmd_write(host);
		break;

	case NAND_CMD_READSTART:
		*is_cache_invalid = 0;
		if (host->addr_value[0] == host->pagesize)
			host->cmd_op.l_cmd = NAND_CMD_READOOB;

		host->send_cmd_read(host);
		break;

	case NAND_CMD_ERASE2:
		host->send_cmd_erase(host);
		break;

	case NAND_CMD_READID:
		/* dest fmcbuf just need init ID lenth */
		if (memset_s((u_char *)(host->iobase), FMC_MEM_LEN,
				0, MAX_SPI_NAND_ID_LEN)) {
			printk("%s %d:memset_s failed\n", __func__, __LINE__);
			break;
		}
		host->cmd_op.l_cmd = cmd;
		host->cmd_op.data_no = MAX_SPI_NAND_ID_LEN;
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
	unsigned int udat = (unsigned int)dat;

	if (ctrl & NAND_ALE)
		fmc100_ale_init(host, ctrl, udat);

	if ((ctrl & NAND_CLE) != 0 && (ctrl & NAND_CTRL_CHANGE) != 0)
		fmc100_cle_init(host, ctrl, udat, &is_cache_invalid);

	if ((dat == NAND_CMD_NONE) && (host->addr_cycle != 0)) {
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
	unsigned int reg;
	unsigned long deadline = jiffies + FMC_MAX_READY_WAIT_JIFFIES;
	struct fmc_host *host = chip->priv;

	do {
		reg = op_cfg_fm_cs(host->cmd_op.cs) | OP_CFG_OEN_EN;
		fmc_writel(host, FMC_OP_CFG, reg);

		reg = FMC_OP_READ_STATUS_EN | FMC_OP_REG_OP_START;
		fmc_writel(host, FMC_OP, reg);

		fmc_cmd_wait_cpu_finish(host);

		reg = fmc_readl(host, FMC_STATUS);
		if ((reg & STATUS_OIP_MASK) == 0)
			return NAND_STATUS_READY;

		cond_resched();
	} while ((time_after_eq(jiffies, deadline)) == 0);

	if ((chip->options & NAND_SCAN_SILENT_NODEV) == 0)
		pr_warn("Wait SPI nand ready timeout, status: %#x\n", reg);

	return 0;
}

/*
 * 'host->epm' only use the first oobfree[0] field, it looks very simple, But...
 */
/* Default OOB area layout */
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

static struct mtd_ooblayout_ops fmc_ooblayout_4k16bit_ops = {
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

static struct nand_config_info fmc_spi_nand_config_table[] = {
	{NAND_PAGE_4K,  NAND_ECC_24BIT, 24, 200,    &fmc_ooblayout_default_ops},
#ifdef CONFIG_BSP_NAND_FS_MAY_NO_YAFFS2
	{NAND_PAGE_4K,  NAND_ECC_16BIT, 16, 128,    &fmc_ooblayout_4k16bit_ops},
#endif
	{NAND_PAGE_4K,  NAND_ECC_8BIT,  8, 128,     &fmc_ooblayout_default_ops},
	{NAND_PAGE_4K,  NAND_ECC_0BIT,  0, 32,      &fmc_ooblayout_default_ops},
	{NAND_PAGE_2K,  NAND_ECC_24BIT, 24, 128,    &fmc_ooblayout_default_ops},
#ifdef CONFIG_BSP_NAND_FS_MAY_NO_YAFFS2
	{NAND_PAGE_2K,  NAND_ECC_16BIT, 16, 64,     &fmc_ooblayout_2k16bit_ops},
#endif
	{NAND_PAGE_2K,  NAND_ECC_8BIT,  8, 64,      &fmc_ooblayout_default_ops},
	{NAND_PAGE_2K,  NAND_ECC_0BIT,  0, 32,      &fmc_ooblayout_default_ops},
	{0, 0, 0, 0, NULL},
};

/*
 * Auto-sensed the page size and ecc type value. driver will try each of page
 * size and ecc type one by one till flash can be read and wrote accurately.
 * so the page size and ecc type is match adaptively without switch on the board
 */
static struct nand_config_info *fmc100_get_config_type_info(
	struct mtd_info *mtd, struct nand_dev_t *nand_dev)
{
	struct nand_config_info *best = NULL;
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_config_info *info = fmc_spi_nand_config_table;

	nand_dev->start_type = "Auto";

	for (; info->ooblayout_ops; info++) {
		if (match_page_type_to_size(info->pagetype) != mtd->writesize)
			continue;

		if (mtd->oobsize < info->oobsize)
			continue;

		if (!best || (best->ecctype < info->ecctype))
			best = info;
	}

	/* All SPI NAND are small-page, SLC */
	chip->base.memorg.bits_per_cell = 1;

	return best;
}

static void fmc100_chip_init(struct nand_chip *chip)
{
	chip->legacy.read_byte = fmc100_read_byte;
	chip->legacy.write_buf = fmc100_write_buf;
	chip->legacy.read_buf = fmc100_read_buf;

	chip->legacy.select_chip = fmc100_select_chip;

	chip->legacy.cmd_ctrl = fmc100_cmd_ctrl;
	chip->legacy.dev_ready = fmc100_dev_ready;

	chip->legacy.chip_delay = FMC_CHIP_DELAY;

	chip->options = NAND_SKIP_BBTSCAN | NAND_BROKEN_XD
			| NAND_SCAN_SILENT_NODEV;

	chip->ecc.engine_type = NAND_ECC_ENGINE_TYPE_NONE;
}

static void fmc100_set_oob_info(struct mtd_info *mtd,
				  struct nand_config_info *info, struct nand_dev_t *nand_dev)
{
	struct nand_chip *chip = NULL;
	struct fmc_host *host = NULL;
	struct mtd_oob_region fmc_oobregion = {0, 0};

	if (info == NULL || mtd == NULL || nand_dev == NULL) {
		db_msg("set oob info err!!!\n");
		return;
	}

	chip = mtd_to_nand(mtd);
	host = chip->priv;

	if (info->ecctype != NAND_ECC_0BIT)
		mtd->oobsize = info->oobsize;

	host->oobsize = mtd->oobsize;
	nand_dev->oobsize = host->oobsize;

	host->dma_oob = host->dma_buffer + host->pagesize;
	host->bbm = (u_char *)(host->buffer + host->pagesize
			       + FMC_BAD_BLOCK_POS);
	if (info->ooblayout_ops == NULL) {
		db_msg("Error: info->ooblayout_ops or is NULL!\n");
		return;
	}
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
				fmc_oobregion.offset + FMC_OOB_LEN_6_EB_OFFSET);
		} else if (host->pagesize == _4K) {
			/* EB bit locate in the bottom two of CTRL(14) */
			host->epm = (u_short *)(host->buffer + host->pagesize +
				fmc_oobregion.offset + FMC_OOB_LEN_14_EB_OFFSET);
		}
	}
#endif
}

static unsigned int fmc100_get_ecc_reg(struct fmc_host *host,
		const struct nand_config_info *info, struct nand_dev_t *nand_dev)
{
	if (info == NULL || host == NULL || nand_dev == NULL) {
		db_msg("get ecc reg err!!!\n");
		return 0;
	}
	host->ecctype = info->ecctype;
	nand_dev->ecctype = host->ecctype;

	return fmc_cfg_ecc_type(match_ecc_type_to_reg(info->ecctype));
}

static unsigned int fmc100_get_page_reg(struct fmc_host *host,
		const struct nand_config_info *info)
{
	if (info == NULL || host == NULL) {
		db_msg("get page reg err!!!\n");
		return 0;
	}
	host->pagesize = match_page_type_to_size(info->pagetype);

	return fmc_cfg_page_size(match_page_type_to_reg(info->pagetype));
}

static unsigned int fmc100_get_block_reg(struct fmc_host *host,
		const struct nand_config_info *info)
{
	unsigned int block_reg = 0;
	unsigned int page_per_block;
	struct mtd_info *mtd = NULL;

	if (info == NULL || host == NULL) {
		db_msg("get block reg err!!!\n");
		return 0;
	}

	mtd = host->mtd;
	if (mtd == NULL) {
		db_msg("err:mtd is NULL!!!\n");
		return 0;
	}
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

	reg_fmc_cfg = fmc_readl(host, FMC_CFG);
	reg_fmc_cfg &= ~(PAGE_SIZE_MASK | ECC_TYPE_MASK | BLOCK_SIZE_MASK);
	reg_fmc_cfg |= ecc_reg | page_reg | block_reg;
	fmc_writel(host, FMC_CFG, reg_fmc_cfg);

	/* Save value of FMC_CFG and FMC_CFG_ECC0 to turn on/off ECC */
	host->fmc_cfg = reg_fmc_cfg;
	host->fmc_cfg_ecc0 = (host->fmc_cfg & ~ECC_TYPE_MASK) | ECC_TYPE_0BIT;
	fmc_pr(BT_DBG, "\t|-Save FMC_CFG[%#x]: %#x and FMC_CFG_ECC0: %#x\n",
	       FMC_CFG, host->fmc_cfg, host->fmc_cfg_ecc0);
}

static int fmc100_set_config_info(struct mtd_info *mtd,
				    struct nand_chip *chip, struct nand_dev_t *nand_dev)
{
	struct fmc_host *host = chip->priv;
	struct nand_config_info *type_info = NULL;

	fmc_pr(BT_DBG, "\t*-Start config Block Page OOB and Ecc\n");

	type_info = fmc100_get_config_type_info(mtd, nand_dev);
	WARN_ON(type_info == NULL);
	if (type_info == NULL) {
		db_msg("set config info err!!!\n");
		return 0;
	}

	fmc_pr(BT_DBG, "\t|-%s Config, PageSize %s EccType %s OOBSize %d\n",
	       nand_dev->start_type, nand_page_name(type_info->pagetype),
	       nand_ecc_name(type_info->ecctype), type_info->oobsize);

	/* Set the page_size, ecc_type, block_size of FMC_CFG[0x0] register */
	fmc100_set_fmc_cfg_reg(host, type_info, nand_dev);

	fmc100_set_oob_info(mtd, type_info, nand_dev);

	fmc_pr(BT_DBG, "\t*-End config Block Page Oob and Ecc\n");

	return 0;
}

void fmc100_spi_nand_init(struct nand_chip *chip)
{
	struct fmc_host *host = NULL;

	if ((chip == NULL) || (chip->priv == NULL)) {
		db_msg("Error: chip or chip->priv is NULL!\n");
		return;
	}
	host = chip->priv;
	fmc_pr(BT_DBG, "\t|*-Start fmc100 SPI Nand init\n");

	/* Switch SPI type to SPI nand */
	fmc100_switch_to_spi_nand(host);

	/* hold on STR mode */
	fmc100_set_str_mode(host);

	/* fmc host init */
	fmc100_host_init(host);
	host->chip = chip;

	/* fmc nand_chip struct init */
	fmc100_chip_init(chip);

	fmc_spi_nand_ids_register();
	nfc_param_adjust = fmc100_set_config_info;

	fmc_pr(BT_DBG, "\t|*-End fmc100 SPI Nand init\n");
}
#ifdef CONFIG_PM
int fmc100_suspend(struct platform_device *pltdev, pm_message_t state)
{
	int ret;
	struct fmc_host *host = platform_get_drvdata(pltdev);
	struct fmc_spi *spi = host->spi;

	mutex_lock(host->lock);
	fmc100_switch_to_spi_nand(host);

	ret = spi->driver->wait_ready(spi);
	if (ret) {
		db_msg("Error: wait ready failed!");
		clk_disable_unprepare(host->clk);
		mutex_unlock(host->lock);
		return 0;
	}

	clk_disable_unprepare(host->clk);
	mutex_unlock(host->lock);

	return 0;
}

int fmc100_resume(struct platform_device *pltdev)
{
	int cs;
	struct fmc_host *host = platform_get_drvdata(pltdev);
	struct nand_chip *chip = host->chip;
	struct nand_memory_organization *memorg;

	memorg = nanddev_get_memorg(&chip->base);

	mutex_lock(host->lock);
	fmc100_switch_to_spi_nand(host);
	clk_prepare_enable(host->clk);

	for (cs = 0; cs < memorg->ntargets; cs++)
		host->send_cmd_reset(host);

	fmc100_spi_nand_config(host);

	mutex_unlock(host->lock);
	return 0;
}
#endif
