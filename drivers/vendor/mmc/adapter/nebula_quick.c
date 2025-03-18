/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#ifdef CONFIG_MMC_QUICKBOOT
#include <linux/mmc/host.h>

#include "sdhci.h"
#include "sdhci_nebula.h"
#include "nebula_quick.h"

static emmc_qboot_u mmc_get_qboot_mode(struct sdhci_host *host)
{
	u32 param1_offset;
	emmc_param_gen1_u param1;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		BUG_ON(nebula->qboot_virt_addr == NULL);
		param1_offset = nebula->info->qboot_param1_ofs;
		param1.u32 = readl(nebula->qboot_virt_addr + param1_offset);
		return (emmc_qboot_u)param1.bits.emmc_qboot_mode;
	}

	return QUICK_BOOT_DIS;
}

bool mmc_is_fast_boot(struct sdhci_host *host)
{
	return (mmc_get_qboot_mode(host) != QUICK_BOOT_DIS);
}
EXPORT_SYMBOL(mmc_is_fast_boot);

int mmc_fast_boot_init(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		if (nebula->info->qboot_phy_addr == INVALID_DATA) {
			pr_err("%s: invalid quick phy addr.\n", mmc_hostname(host->mmc));
			return -EINVAL;
		}

		nebula->qboot_virt_addr = ioremap(nebula->info->qboot_phy_addr, PAGE_SIZE);
		if (nebula->qboot_virt_addr == NULL) {
			pr_err("%s: quick boot remap failed.\n", mmc_hostname(host->mmc));
			return -ENOMEM;
		}

		if (mmc_is_fast_boot(host)) {
			nebula->priv_cap |= NEBULA_CAP_QUICK_BOOT;
		}
	}

	return ERET_SUCCESS;
}

static emmc_mode_u mmc_get_cur_mode(struct sdhci_host *host)
{
	u32 param1_offset;
	emmc_param_gen1_u param1;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->devid == MMC_DEV_TYPE_MMC_0 &&
			(mmc_get_qboot_mode(host) == QUICK_BOOT_WARM)) {
		param1_offset = nebula->info->qboot_param1_ofs;
		param1.u32 = readl(nebula->qboot_virt_addr + param1_offset);
		return (emmc_mode_u)param1.bits.emmc_cur_mode;
	}

	return INIT_MODE;
}

void mmc_set_cur_mode(struct sdhci_host *host, emmc_mode_u mode)
{
	u32 param1_offset;
	emmc_param_gen1_u param1;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->devid == MMC_DEV_TYPE_MMC_0 &&
			(mmc_get_qboot_mode(host) == QUICK_BOOT_WARM)) {
		BUG_ON(nebula->qboot_virt_addr == NULL);
		param1_offset = nebula->info->qboot_param1_ofs;
		param1.u32 = readl(nebula->qboot_virt_addr + param1_offset);
		param1.bits.emmc_cur_mode = mode;
		writel(param1.u32, nebula->qboot_virt_addr + param1_offset);
	}
}

void mmc_reset_init_mode(struct sdhci_host *host)
{
	return mmc_set_cur_mode(host, INIT_MODE);
}
EXPORT_SYMBOL(mmc_reset_init_mode);

void mmc_parameter_init(struct mmc_host *mmc)
{
	u8 reg8;
	u32 param1_offset;
	emmc_param_gen1_u param1;
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_nebula *nebula = nebula_priv(host);
	const u8 bus_width[] = {MMC_BUS_WIDTH_1, MMC_BUS_WIDTH_4, \
		MMC_BUS_WIDTH_8, MMC_BUS_WIDTH_1};
	const u8 uhs_mode[] = {MMC_TIMING_LEGACY, MMC_TIMING_MMC_HS, 0, \
		MMC_TIMING_MMC_HS200, MMC_TIMING_MMC_DDR52, 0, 0, MMC_TIMING_MMC_HS400};

	if (nebula->devid != MMC_DEV_TYPE_MMC_0) {
		return;
	}

	BUG_ON(nebula->qboot_virt_addr == NULL);
	param1_offset = nebula->info->qboot_param1_ofs;
	param1.u32 = readl(nebula->qboot_virt_addr + param1_offset);

	if (mmc_get_qboot_mode(host) == QUICK_BOOT_COLD) {
		/* Cold boot choice from host controller */
		param1.bits.emmc_uhs_mode_sel = \
				sdhci_readb(host, SDHCI_HOST_CONTROL2) & SDHCI_CTRL_UHS_MASK;
		param1.bits.emmc_enh_strobe = \
				((sdhci_readw(host, SDHCI_EMMC_CTRL) & SDHCI_ENH_STROBE_EN) != 0);
		reg8 = sdhci_readb(host, SDHCI_HOST_CONTROL);
		param1.bits.emmc_bus_width = ((reg8 & SDHCI_CTRL_8BITBUS) != 0) ? BUS_8BIT_IDX : \
				(((reg8 & SDHCI_CTRL_4BITBUS) != 0) ? BUS_4BIT_IDX : BUS_1BIT_IDX);
	}

	mmc->ios.bus_width = bus_width[param1.bits.emmc_bus_width];
	mmc->ios.timing = uhs_mode[param1.bits.emmc_uhs_mode_sel];
	host->timing = mmc->ios.timing;
	nebula->tuning_phase = sdhci_readl(host, SDHCI_AT_STAT) & SDHCI_PHASE_SEL_MASK;
	mmc->ios.enhanced_strobe = param1.bits.emmc_enh_strobe;

	mmc->ios.vdd = (unsigned short)fls(mmc->ocr_avail) - 1;
	mmc->ios.power_mode = MMC_POWER_ON;
	/* OPENDRAIN: Identify process, PUSHPULL: Transfer */
	mmc->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	mmc->ios.drv_type = 0;
	mmc->ios.clock = mmc->f_max;
	mmc->actual_clock = mmc->f_max;
}

u32 mmc_get_rocr(struct sdhci_host *host)
{
	u32 rocr = 0;
	u32 param1_offset;
	emmc_param_gen1_u param1;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		BUG_ON(nebula->qboot_virt_addr == NULL);
		param1_offset = nebula->info->qboot_param1_ofs;
		param1.u32 = readl(nebula->qboot_virt_addr + param1_offset);
		rocr = MMC_VDD_165_195;
		rocr |= MMC_VDD_32_33 | MMC_VDD_33_34;
		/* for >= 2GiB Device, hcs mode assign from ext csd */
		rocr |= (param1.bits.emmc_hcs_mode << HCS_BIT);
	}

	return rocr;
}

static u32 mmc_get_io_info(struct sdhci_host *host)
{
	u32 timing;
	emmc_param_gen0_u param0;
	nebula_timing *timing_data = NULL;
	struct sdhci_nebula *nebula = nebula_priv(host);
	const nebula_info *info = nebula->info;

	param0.u32 = 0;

	/* choice host timing data */
	timing_data = info->timing + host->timing;
	if (!timing_data->data_valid) {
		/* choice legacy mode timing */
		timing_data = info->timing;
	}

	timing = timing_data->timing[IO_TYPE_CMD];
	param0.bits.emmc_cmd_drv = \
		(timing & info->io_drv_str_mask) >> info->io_drv_str_bit_ofs;
	param0.bits.emmc_cmd_sl = \
		(timing & info->io_drv_str_mask) >> info->io_drv_str_bit_ofs;

	timing = timing_data->timing[IO_TYPE_CLK];
	param0.bits.emmc_clk_drv = \
		(timing & info->io_drv_str_mask) >> info->io_drv_str_bit_ofs;
	param0.bits.emmc_clk_sl = \
		(timing & info->io_drv_str_mask) >> info->io_drv_str_bit_ofs;

	timing = timing_data->timing[IO_TYPE_DATA];
	param0.bits.emmc_data_drv = \
		(timing & info->io_drv_str_mask) >> info->io_drv_str_bit_ofs;
	param0.bits.emmc_data_sl = \
		(timing & info->io_drv_str_mask) >> info->io_drv_str_bit_ofs;

	return param0.u32;
}

int mmc_save_parameters(struct mmc_host *mmc)
{
	unsigned int reg;
	emmc_param_gen0_u param0;
	emmc_param_gen1_u param1;
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_nebula *nebula = nebula_priv(host);
	const nebula_info *info = nebula->info;

	/* eMMC work parameters store by uboot, only eMMC support, cold boot no need */
	if ((nebula->devid != MMC_DEV_TYPE_MMC_0) || \
		mmc_get_qboot_mode(host) == QUICK_BOOT_COLD || \
		(mmc_get_cur_mode(host) == TRAN_MODE)) {
		return ERET_SUCCESS;
	}

	BUG_ON(nebula->qboot_virt_addr == NULL);
	param0.u32 = readl(nebula->qboot_virt_addr);

	regmap_read(nebula->crg_regmap, info->crg_ofs[CRG_CLK_RST], &reg);
	param0.bits.emmc_clk_sel = \
		((reg & nebula->mask->crg_clk_sel_mask) >> nebula->mask->crg_clk_sel_ofs);
	param0.bits.emmc_clk_ph_sel = nebula->drv_phase;
	param0.bits.emmc_sw_clk_ph = sdhci_readb(host, SDHCI_AT_STAT);

	/* eMMC IO info */
	param0.u32 |= mmc_get_io_info(host);

	writel(param0.u32, nebula->qboot_virt_addr);

	param1.u32 = readl(nebula->qboot_virt_addr + info->qboot_param1_ofs);
	param1.bits.emmc_uhs_mode_sel = \
		sdhci_readb(host, SDHCI_HOST_CONTROL2) & SDHCI_CTRL_UHS_MASK;
	param1.bits.emmc_enh_strobe = mmc->ios.enhanced_strobe;
	param1.bits.emmc_bus_width = (mmc->ios.bus_width  == MMC_BUS_WIDTH_8) ? BUS_8BIT_IDX : \
						((mmc->ios.bus_width == MMC_BUS_WIDTH_4) ? BUS_4BIT_IDX : BUS_1BIT_IDX);
	param1.bits.emmc_hcs_mode = mmc_card_is_blockaddr(mmc->card);
	writel(param1.u32, nebula->qboot_virt_addr + info->qboot_param1_ofs);

	mmc_set_cur_mode(host, TRAN_MODE);

	return ERET_SUCCESS;
}
EXPORT_SYMBOL(mmc_save_parameters);
#endif
