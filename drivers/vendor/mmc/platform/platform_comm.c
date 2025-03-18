/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include "sdhci_nebula.h"
#include "platform_priv.h"

#define sdhci_nebula_dump(f, x...) \
	pr_err("%s: sdhci: " f, mmc_hostname(host->mmc), ## x)

static void comm_set_regmap(struct regmap *regmap,
		u32 offset, u32 mask, u32 data)
{
	u32 reg;

	regmap_read(regmap, offset, &reg);
	reg &= ~mask;
	data &= mask;
	reg |= data;
	regmap_write(regmap, offset, reg);
}

static int comm_wait_dll_timeout(struct sdhci_host *host,
		u32 offset, u32 mask, u32 timeout)
{
	u32 reg, save_timeout;
	struct sdhci_nebula *nebula = nebula_priv(host);

	save_timeout = timeout;

	do {
		reg = 0;
		regmap_read(nebula->crg_regmap, offset, &reg);
		if (reg & mask)
			return ERET_SUCCESS;

		mdelay(1);
		timeout--;
	} while (timeout > 0);

	pr_err("%s: wait ofs 0x%x mask 0x%x timeout after %d ms\n",
		mmc_hostname(host->mmc), offset, mask, save_timeout);
	return -ETIMEDOUT;
}

int plat_wait_sample_dll_ready(struct sdhci_host *host)
{
	unsigned int offset, mask;
	struct sdhci_nebula *nebula = nebula_priv(host);

	offset = nebula->info->crg_ofs[CRG_DLL_STA];
	mask = nebula->mask->samp_ready_mask;
	return comm_wait_dll_timeout(host, offset, mask, WAIT_MAX_TIMEOUT);
}

int plat_wait_p4_dll_lock(struct sdhci_host *host)
{
	unsigned int offset, mask;
	struct sdhci_nebula *nebula = nebula_priv(host);

	offset = nebula->info->crg_ofs[CRG_DLL_STA];
	mask = nebula->mask->p4_lock_mask;
	return comm_wait_dll_timeout(host, offset, mask, WAIT_MAX_TIMEOUT);
}

int plat_wait_ds_dll_ready(struct sdhci_host *host)
{
	unsigned int offset, mask;
	struct sdhci_nebula *nebula = nebula_priv(host);

	offset = nebula->info->crg_ofs[CRG_DLL_STA];
	mask = nebula->mask->dll_ready_mask;
	return comm_wait_dll_timeout(host, offset, mask, WAIT_MAX_TIMEOUT);
}

void plat_hs400_enhanced_strobe(struct mmc_host *mmc, struct mmc_ios *ios)
{
	u16 ctrl;
	struct sdhci_host *host = mmc_priv(mmc);

	ctrl = sdhci_readw(host, SDHCI_EMMC_CTRL);
	if (ios->enhanced_strobe)
		ctrl |= SDHCI_ENH_STROBE_EN;
	else
		ctrl &= ~SDHCI_ENH_STROBE_EN;

	sdhci_writew(host, ctrl, SDHCI_EMMC_CTRL);
}

void plat_get_drv_samp_phase(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);
	const nebula_info *info = nebula->info;
	nebula_timing *timing_data = NULL;

	if ((info->timing_size == 0) || host->timing > info->timing_size) {
		pr_err("%s: warning: check fixed timing %d\n",
			mmc_hostname(host->mmc), host->timing);
		return;
	}

	/* choice host timing data */
	timing_data = info->timing + host->timing;
	if (timing_data->data_valid == false) {
		/* choice legacy mode timing */
		timing_data = info->timing;
	}

	if (timing_data->data_valid == true) {
		nebula->drv_phase = (timing_data->phase[DRV_PHASE] & TIMING_MASK);
		if (is_timing_valid(timing_data->phase[SAMP_PHASE])) {
			nebula->sample_phase = (timing_data->phase[SAMP_PHASE] & TIMING_MASK);
		} else {
			nebula->sample_phase = nebula->tuning_phase;
		}
	} else {
		pr_err("%s: warning: check default timing valid?\n",
			mmc_hostname(host->mmc));
	}
}

void plat_set_drv_phase(struct sdhci_host *host, u32 phase)
{
	unsigned int reg, offset;
	struct sdhci_nebula *nebula = nebula_priv(host);

	offset = nebula->info->crg_ofs[CRG_DRV_DLL];
	regmap_read(nebula->crg_regmap, offset, &reg);
	reg &= ~(nebula->mask->drv_phase_mask);
	reg |= phase;
	regmap_write(nebula->crg_regmap, offset, reg);
}

void plat_set_drv_cap(struct sdhci_host *host)
{
	int idx, bus_width;
	struct sdhci_nebula *nebula = nebula_priv(host);
	const nebula_info *info = nebula->info;
	nebula_timing *timing_data = NULL;
	struct mmc_ios *ios = &host->mmc->ios;

	if (nebula->priv_quirk & NEBULA_QUIRK_FPGA) {
		return;
	}

	if ((info->timing_size == 0) || host->timing > info->timing_size) {
		pr_err("%s: warning: check fixed timing %d\n",
			mmc_hostname(host->mmc), host->timing);
		return;
	}

	/* choice host timing data */
	timing_data = info->timing + host->timing;
	if (timing_data->data_valid == false) {
		/* choice legacy mode timing */
		timing_data = info->timing;
	}

	/* clk cmd rst timing setting */
	for (idx = 0; idx < IO_TYPE_MAX; idx++) {
		if (is_timing_valid(timing_data->timing[idx])) {
			comm_set_regmap(nebula->iocfg_regmap, info->io_offset[idx], \
				info->io_drv_mask, timing_data->timing[idx] & TIMING_MASK);
		}
	}

	/* data line drv valid? fixed bus width */
	if (is_timing_valid(timing_data->timing[IO_TYPE_DATA])) {
		/* data0 line set already */
		bus_width = (1 << ios->bus_width) - 1;
		for (idx = IO_TYPE_D1; (idx < IO_TYPE_DMAX) && (bus_width != 0); idx++) {
			bus_width--;
			comm_set_regmap(nebula->iocfg_regmap, info->io_offset[idx], \
				info->io_drv_mask, timing_data->timing[IO_TYPE_DATA] & TIMING_MASK);
		}
	}
}

void plat_dll_reset_assert(struct sdhci_host *host)
{
	unsigned int reg, offset;
	struct sdhci_nebula *nebula = nebula_priv(host);

	offset = nebula->info->crg_ofs[CRG_DLL_RST];
	regmap_read(nebula->crg_regmap, offset, &reg);
	reg |= nebula->mask->dll_srst_mask;
	regmap_write(nebula->crg_regmap, offset, reg);
}

void plat_dll_reset_deassert(struct sdhci_host *host)
{
	unsigned int reg, offset;
	struct sdhci_nebula *nebula = nebula_priv(host);

	offset = nebula->info->crg_ofs[CRG_DLL_RST];
	regmap_read(nebula->crg_regmap, offset, &reg);
	reg &= ~nebula->mask->dll_srst_mask;
	regmap_write(nebula->crg_regmap, offset, reg);
}

static void comm_crg_enable_clock(struct sdhci_host *host)
{
	unsigned int reg, offset;
	struct sdhci_nebula *nebula = nebula_priv(host);

	offset = nebula->info->crg_ofs[CRG_CLK_RST];
	regmap_read(nebula->crg_regmap, offset, &reg);
	reg |= nebula->mask->crg_cken_mask;
	regmap_write(nebula->crg_regmap, offset, reg);
}

static void comm_crg_reset_assert(struct sdhci_host *host)
{
	unsigned int reg, offset;
	struct sdhci_nebula *nebula = nebula_priv(host);

	offset = nebula->info->crg_ofs[CRG_CLK_RST];
	regmap_read(nebula->crg_regmap, offset, &reg);
	reg |= nebula->mask->crg_srst_mask;
	regmap_write(nebula->crg_regmap, offset, reg);
}

static void comm_crg_reset_deassert(struct sdhci_host *host)
{
	unsigned int reg, offset;
	struct sdhci_nebula *nebula = nebula_priv(host);

	offset = nebula->info->crg_ofs[CRG_CLK_RST];
	regmap_read(nebula->crg_regmap, offset, &reg);
	reg &= ~nebula->mask->crg_srst_mask;
	regmap_write(nebula->crg_regmap, offset, reg);
}

int plat_crg_init(struct sdhci_host *host)
{
	int ret;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_nebula *nebula = nebula_priv(host);

	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret) {
		pr_err("%s: enable mmc clk failed\n", mmc_hostname(host->mmc));
		return ret;
	}

	if ((nebula->priv_cap & NEBULA_CAP_QUICK_BOOT) != 0) {
		return ERET_SUCCESS;
	}

	if (nebula->priv_cap & NEBULA_CAP_RST_IN_DRV) {
		comm_crg_enable_clock(host);
		comm_crg_reset_assert(host);
		plat_dll_reset_assert(host);

		udelay(25); /* delay 25 us */
		comm_crg_reset_deassert(host);
		udelay(10); /* delay 10 us */
	} else {
		reset_control_assert(nebula->crg_rst);
		reset_control_assert(nebula->crg_tx);
		reset_control_assert(nebula->crg_rx);

		reset_control_assert(nebula->dll_rst);

		udelay(25); /* delay 25 us */
		reset_control_deassert(nebula->crg_rst);
		reset_control_deassert(nebula->crg_tx);
		reset_control_deassert(nebula->crg_rx);

		udelay(10); /* delay 10 us */
	}

	return ERET_SUCCESS;
}

/* Do ZQ resistance calibration for eMMC PHY IO */
static int comm_resistance_calibration(struct sdhci_host *host)
{
	int i;
	u32 reg_val;
	void __iomem *viraddr = NULL;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->info->zq_phy_addr == 0) {
		pr_err("%s: zq_phy_addr is invalid.\n", mmc_hostname(host->mmc));
		return -EINVAL;
	}

	viraddr = ioremap(nebula->info->zq_phy_addr, sizeof(u32));
	if (viraddr == NULL) {
		pr_err("%s: io calibration ioremap error.\n", mmc_hostname(host->mmc));
		return -ENOMEM;
	}

	reg_val = readl(viraddr);
	reg_val |= EMMC_ZQ_INIT_EN | EMMC_ZQ_ZCAL_EN;
	writel(reg_val, viraddr);

	for (i = 0; i < EMMC_ZQ_CHECK_TIMES; i++) {
		reg_val = readl(viraddr);
		if ((reg_val & (EMMC_ZQ_INIT_EN | EMMC_ZQ_ZCAL_EN)) == 0) {
			iounmap(viraddr);
			return ERET_SUCCESS;
		}
		udelay(10); /* delay 10 us */
	}

	iounmap(viraddr);
	return -ETIMEDOUT;
}

int plat_resistance_calibration(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if ((nebula->priv_cap & NEBULA_CAP_QUICK_BOOT) != 0) {
		return ERET_SUCCESS;
	}

	if (nebula->priv_cap & NEBULA_CAP_ZQ_CALB) {
		return comm_resistance_calibration(host);
	}

	return ERET_SUCCESS;
}

static int comm_voltage_switch(struct sdhci_host *host, const struct mmc_ios *ios)
{
	u32 ctrl;
	void __iomem *viraddr;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (ios == NULL || nebula->info->volt_sw_phy_addr == 0) {
		pr_err("%s: ios or volt_sw_phy_addr is invalid.\n",
			mmc_hostname(host->mmc));
		return -EINVAL;
	}

	viraddr = ioremap(nebula->info->volt_sw_phy_addr, sizeof(u32));
	if (viraddr == NULL) {
		pr_err("%s: volt switch ioremap error.\n", mmc_hostname(host->mmc));
		return -ENOMEM;
	}

	ctrl = readl(viraddr);
	ctrl |= nebula->mask->volt_sw_en_mask;
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180)
		ctrl |= nebula->mask->volt_sw_1v8_mask;
	else if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		ctrl &= ~nebula->mask->volt_sw_1v8_mask;

	writel(ctrl, viraddr);

	usleep_range(1000, 2000); /* Sleep between 1000 and 2000us */

	iounmap(viraddr);

	return ERET_SUCCESS;
}

int plat_voltage_switch(struct sdhci_host *host, struct mmc_ios *ios)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->priv_cap & NEBULA_CAP_VOLT_SW) {
		return comm_voltage_switch(host, ios);
	}

	return ERET_SUCCESS;
}

void __weak plat_caps_quirks_init(struct sdhci_host *host)
{
}

void __weak plat_extra_init(struct sdhci_host *host)
{
}

void __weak plat_dump_io_info(struct sdhci_host *host)
{
	int idx, bus_width;
	u32 reg0, reg1;
	struct sdhci_nebula *nebula = nebula_priv(host);
	const nebula_info *info = nebula->info;

	/* Cmd Clk */
	regmap_read(nebula->iocfg_regmap, info->io_offset[IO_TYPE_CLK], &reg0);
	regmap_read(nebula->iocfg_regmap, info->io_offset[IO_TYPE_CMD], &reg1);
	sdhci_nebula_dump("Cmd io:    0x%08x | Clk io:   0x%08x\n", reg0, reg1);

	if (info->io_offset[IO_TYPE_RST] != INVALID_DATA && \
		info->io_offset[IO_TYPE_DQS] != INVALID_DATA) {
		/* Rst/Detect Dqs/Power_en */
		regmap_read(nebula->iocfg_regmap, info->io_offset[IO_TYPE_RST], &reg0);
		regmap_read(nebula->iocfg_regmap, info->io_offset[IO_TYPE_DQS], &reg1);
		sdhci_nebula_dump("Rst/Det:   0x%08x | Dqs/Pwen: 0x%08x\n", reg0, reg1);
	}

	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_1) {
		regmap_read(nebula->iocfg_regmap, info->io_offset[IO_TYPE_D0], &reg0);
		sdhci_nebula_dump("Data0 io:  0x%08x\n", reg0);
		return;
	}

	bus_width = (1 << host->mmc->ios.bus_width);
	for (idx = 0; idx < bus_width; idx += DUMP_DATA_IO_STEP) {
		regmap_read(nebula->iocfg_regmap, info->io_offset[idx + IO_TYPE_D0], &reg0);
		regmap_read(nebula->iocfg_regmap, info->io_offset[idx + IO_TYPE_D0 + 1], &reg1);
		sdhci_nebula_dump("Data%d io:  0x%08x | Data%d io: 0x%08x\n",
				idx, reg0, idx + 1, reg1);
	}
}

static u32 comm_get_mmc_bus_width(struct sdhci_host *host)
{
	void __iomem *sys_stat_reg;
	unsigned int sys_stat;
	unsigned int bus_width;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->info->bus_width_phy_addr == 0) {
		pr_err("%s: bus_width_phy_addr is invalid.\n", mmc_hostname(host->mmc));
		return -EINVAL;
	}

	sys_stat_reg = ioremap(nebula->info->bus_width_phy_addr, sizeof(u32));
	if (sys_stat_reg == NULL) {
		pr_err("%s: bus width ioremap error.\n", mmc_hostname(host->mmc));
		return -ENOMEM;
	}

	sys_stat = readl(sys_stat_reg);
	iounmap(sys_stat_reg);

	if ((sys_stat & BOOT_FLAG_MASK) == BOOT_MEDIA_EMMC) {
		bus_width = ((sys_stat & EMMC_BOOT_8BIT) != 0) ?
			MMC_BUS_WIDTH_8 : MMC_BUS_WIDTH_4;
	} else {
		/* up to 4 bit mode support when spi nand start up */
		bus_width = MMC_BUS_WIDTH_4;
	}

	return bus_width;
}

void __maybe_unused plat_set_mmc_bus_width(struct sdhci_host *host)
{
	u32 bus_width;
	struct sdhci_nebula *nebula = nebula_priv(host);

	/* for eMMC devices only */
	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		bus_width = comm_get_mmc_bus_width(host);
		if (bus_width == MMC_BUS_WIDTH_8) {
			host->mmc->caps |= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA;
		} else {
			host->mmc->caps |= MMC_CAP_4_BIT_DATA;
			host->mmc->caps &= ~MMC_CAP_8_BIT_DATA;
		}
	}
}

void __maybe_unused plat_comm_caps_quirks_init(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	/*
	 * only eMMC has a hw reset, eMMC and NM card is fixed 1.8V voltage
	 */
	if ((host->mmc->caps & MMC_CAP_HW_RESET)
		|| (nebula->priv_cap & NEBULA_CAP_NM_CARD)) {
		host->flags &= ~SDHCI_SIGNALING_330;
		host->flags |= SDHCI_SIGNALING_180;
	}

	/*
	 * we parse the support timings from dts, so we read the
	 * host capabilities early and clear the timing capabilities,
	 * SDHCI_QUIRK_MISSING_CAPS is set so that sdhci driver would
	 * not read it again
	 */
	host->caps = sdhci_readl(host, SDHCI_CAPABILITIES);
	host->caps &= ~(SDHCI_CAN_DO_HISPD | SDHCI_CAN_VDD_300);
	host->caps1 = sdhci_readl(host, SDHCI_CAPABILITIES_1);
	host->caps1 &= ~(SDHCI_SUPPORT_SDR50 | SDHCI_SUPPORT_SDR104 |
			 SDHCI_SUPPORT_DDR50 | SDHCI_CAN_DO_ADMA3);
	host->quirks |= SDHCI_QUIRK_MISSING_CAPS |
			SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC |
			SDHCI_QUIRK_SINGLE_POWER_WRITE;
	host->quirks2 &= ~SDHCI_QUIRK2_ACMD23_BROKEN;
}

void plat_set_emmc_type(struct sdhci_host *host)
{
	u32 ctrl;

	ctrl = sdhci_readl(host, SDHCI_EMMC_CTRL);
	ctrl |= SDHCI_CARD_IS_EMMC;
	sdhci_writel(host, ctrl, SDHCI_EMMC_CTRL);
}
