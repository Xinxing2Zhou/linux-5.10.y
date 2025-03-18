/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/version.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/mmc/mmc.h>

#include "cqhci.h"
#include "nebula_quirk_ids.h"
#include "nebula_quick.h"
#include "sdhci_nebula.h"
#include "core.h"
#include "card.h"

static u32 __read_mostly g_mmc_flag = 0;

#define sdhci_nebula_dump(f, x...) \
	pr_err("%s: sdhci: " f, mmc_hostname(host->mmc), ## x)

void __maybe_unused sdhci_nebula_dump_vendor_regs(struct sdhci_host *host)
{
	u32 reg0, reg1;
	struct sdhci_nebula *nebula = nebula_priv(host);

	sdhci_nebula_dump("========= SDHCI NEBULA DEBUG DUMP ==========\n");
	sdhci_nebula_dump("Mshc ctl:  0x%08x | Ahb ctl:  0x%08x\n",
		   sdhci_readl(host, SDHCI_MSHC_CTRL_R),
		   sdhci_readl(host, SDHCI_AXI_MBIU_CTRL));
	sdhci_nebula_dump("Debug1:    0x%08x | Debug2:   0x%08x\n",
		   sdhci_readl(host, SDHCI_DEBUG1_PORT),
		   sdhci_readl(host, SDHCI_DEBUG2_PORT));
	sdhci_nebula_dump("eMMC ctl:  0x%08x | eMMC rst: 0x%08x\n",
		   sdhci_readl(host, SDHCI_EMMC_CTRL),
		   sdhci_readl(host, SDHCI_EMMC_HW_RESET));
	sdhci_nebula_dump("AT ctl:    0x%08x | AT stat:  0x%08x\n",
		   sdhci_readl(host, SDHCI_AT_CTRL),
		   sdhci_readl(host, SDHCI_AT_STAT));
	sdhci_nebula_dump("eMMC reg:  0x%08x | Mutl cyl: 0x%08x\n",
		   sdhci_readl(host, SDHCI_EMAX_R),
		   sdhci_readl(host, SDHCI_MUTLI_CYCLE_EN));

	/* Crg */
	regmap_read(nebula->crg_regmap, nebula->info->crg_ofs[CRG_CLK_RST], &reg0);
	regmap_read(nebula->crg_regmap, nebula->info->crg_ofs[CRG_DLL_RST], &reg1);
	sdhci_nebula_dump("CRG ctl:   0x%08x | Dll rst:  0x%08x\n", reg0, reg1);
	plat_dump_io_info(host);
}

static int __maybe_unused __init nebula_mmc_setup(char *str)
{
	/* cqe off */
	if (strcasecmp(str, "cqeoff") == 0) {
		g_mmc_flag |= MMC_CMDQ_FORCE_OFF;
	}

	/* no whitelist */
	if (strcasecmp(str, "nowhitelist") == 0) {
		g_mmc_flag |= MMC_CMDQ_DIS_WHITELIST;
	}

	return 1;
}
__setup("mmc=", nebula_mmc_setup);

static void nebula_set_emmc_card(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		unsigned int reg;

		reg = sdhci_readl(host, SDHCI_EMMC_CTRL);
		reg |= SDHCI_CARD_IS_EMMC;
		sdhci_writel(host, reg, SDHCI_EMMC_CTRL);
	}
}

static void nebula_enable_sample(struct sdhci_host *host)
{
	unsigned int reg;

	reg = sdhci_readl(host, SDHCI_AT_CTRL);
	reg |= SDHCI_SAMPLE_EN;
	sdhci_writel(host, reg, SDHCI_AT_CTRL);
}

static void nebula_set_sample_phase(struct sdhci_host *host, unsigned int phase)
{
	unsigned int reg;

	reg = sdhci_readl(host, SDHCI_AT_STAT);
	reg &= ~SDHCI_PHASE_SEL_MASK;
	reg |= phase;
	sdhci_writel(host, reg, SDHCI_AT_STAT);
}

static void nebula_disable_card_clk(struct sdhci_host *host)
{
	u16 clk;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

static void nebula_enable_card_clk(struct sdhci_host *host)
{
	u16 clk;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

static void nebula_disable_internal_clk(struct sdhci_host *host)
{
	u16 clk;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

static void nebula_enable_internal_clk(struct sdhci_host *host)
{
	u16 clk, timeout;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk |= SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_PLL_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	while ((clk & SDHCI_CLOCK_INT_STABLE) == 0) {
		if (timeout == 0) {
			pr_err("%s: Internal clock never stabilised.\n", mmc_hostname(host->mmc));
			return;
		}
		timeout--;
		udelay(1000); /* delay 1000 us */
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	}
}

void sdhci_nebula_set_clock(struct sdhci_host *host, unsigned int clk)
{
	struct sdhci_nebula *nebula = nebula_priv(host);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	nebula_disable_card_clk(host);
	udelay(25); /* delay 25 us */
	nebula_disable_internal_clk(host);

	host->mmc->actual_clock = clk;
	if (clk == 0)
		return;

	clk_set_rate(pltfm_host->clk, clk);

	host->mmc->actual_clock = (unsigned int)clk_get_rate(pltfm_host->clk);

	plat_get_drv_samp_phase(host);
	plat_set_drv_phase(host, nebula->drv_phase);
	nebula_enable_sample(host);
	nebula_set_sample_phase(host, nebula->sample_phase);
	udelay(5); /* delay 5 us */
	nebula_enable_internal_clk(host);

	if ((host->timing == MMC_TIMING_MMC_HS400) ||
	    (host->timing == MMC_TIMING_MMC_HS200) ||
	    (host->timing == MMC_TIMING_UHS_SDR104) ||
	    (host->timing == MMC_TIMING_UHS_SDR50)) {
		if (nebula->priv_cap & NEBULA_CAP_RST_IN_DRV) {
			plat_dll_reset_assert(host);
			plat_dll_reset_deassert(host);
		} else {
			reset_control_assert(nebula->dll_rst);
			reset_control_deassert(nebula->dll_rst);
		}
		plat_wait_p4_dll_lock(host);
		plat_wait_sample_dll_ready(host);
	}

	if (host->timing == MMC_TIMING_MMC_HS400)
		plat_wait_ds_dll_ready(host);

	nebula_enable_card_clk(host);
	udelay(75); /* delay 75 us */
}

static void nebula_select_sample_phase(struct sdhci_host *host, unsigned int phase)
{
	nebula_disable_card_clk(host);
	nebula_set_sample_phase(host, phase);
	plat_wait_sample_dll_ready(host);
	nebula_enable_card_clk(host);
	udelay(1);
}

static int nebula_send_tuning(struct sdhci_host *host, u32 opcode)
{
	int count, err;

	count = 0;
	do {
		err = mmc_send_tuning(host->mmc, opcode, NULL);
		if (err) {
			if (host->tuning_delay > 0)
				mdelay(host->tuning_delay);
			mmc_abort_tuning(host->mmc, MMC_SEND_TUNING_BLOCK_HS200);
			break;
		}

		count++;
	} while (count < MAX_TUNING_NUM);

	return err;
}

static void nebula_pre_tuning(struct sdhci_host *host)
{
	sdhci_writel(host, host->ier | SDHCI_INT_DATA_AVAIL, SDHCI_INT_ENABLE);
	sdhci_writel(host, host->ier | SDHCI_INT_DATA_AVAIL, SDHCI_SIGNAL_ENABLE);

	nebula_enable_sample(host);
	host->is_tuning = 1;
}

static void nebula_post_tuning(struct sdhci_host *host)
{
	unsigned short ctrl;

	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	ctrl |= SDHCI_CTRL_TUNED_CLK;
	sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);

	sdhci_writel(host, host->ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, host->ier, SDHCI_SIGNAL_ENABLE);
	host->is_tuning = 0;
}

static int nebula_get_best_sample(u32 candidates)
{
	int rise = NOT_FOUND;
	int fall, i, win;
	int win_max_r = NOT_FOUND;
	int win_max_f = NOT_FOUND;
	int end_fall = NOT_FOUND;
	int found = NOT_FOUND;
	int win_max = 0;

	for (i = 0; i < PHASE_SCALE; i++) {
		if ((candidates & WIN_MASK) == WIN_RISE)
			rise = (i + 1) % PHASE_SCALE;

		if ((candidates & WIN_MASK) == WIN_FALL) {
			fall = i;
			win = fall - rise + 1;
			if (rise == NOT_FOUND) {
				end_fall = fall;
			} else if ((rise != NOT_FOUND) && (win > win_max)) {
				win_max = win;
				found = (fall + rise) / WIN_DIV;
				win_max_r = rise;
				win_max_f = fall;
				rise = NOT_FOUND;
				fall = NOT_FOUND;
			}
		}
		candidates = ror32(candidates, 1);
	}

	if (end_fall != NOT_FOUND && rise != NOT_FOUND) {
		fall = end_fall;
		if (end_fall < rise)
			end_fall += PHASE_SCALE;

		win = end_fall - rise + 1;

		if (win > win_max) {
			found = (rise + (win / WIN_DIV)) % PHASE_SCALE;
			win_max_r = rise;
			win_max_f = fall;
		}
	}

	if (found != NOT_FOUND)
		pr_err("valid phase shift [%d, %d] Final Phase:%d\n",
				win_max_r, win_max_f, found);

	return found;
}

static int nebula_exec_sample_tuning(struct sdhci_host *host, u32 opcode)
{
	struct sdhci_nebula *nebula = nebula_priv(host);
	unsigned int sample;
	int err, phase;
	unsigned int candidates = 0;

	nebula_pre_tuning(host);

	for (sample = 0; sample < PHASE_SCALE; sample++) {
		nebula_select_sample_phase(host, sample);

		err = nebula_send_tuning(host, opcode);
		if (err)
			pr_debug("%s: send tuning CMD%u fail! phase:%d err:%d\n",
					mmc_hostname(host->mmc), opcode, sample, err);
		else
			candidates |= (0x1 << sample);
	}

	pr_info("%s: tuning done! candidates 0x%X: ",
			mmc_hostname(host->mmc), candidates);

	phase = nebula_get_best_sample(candidates);
	if (phase == NOT_FOUND) {
		phase = nebula->sample_phase;
		pr_err("%s: no valid phase shift! use default %d\n", mmc_hostname(host->mmc), phase);
	}

	nebula->tuning_phase = (unsigned int)phase;
	nebula_select_sample_phase(host, nebula->tuning_phase);
	nebula_post_tuning(host);

	return ERET_SUCCESS;
}

static void nebula_enable_edge_tuning(struct sdhci_host *host)
{
	unsigned int reg;

	reg = sdhci_readl(host, SDHCI_MULTI_CYCLE);
	reg |= SDHCI_EDGE_DETECT_EN;
	sdhci_writel(host, reg, SDHCI_MULTI_CYCLE);
}

static void nebula_disable_edge_tuning(struct sdhci_host *host)
{
	unsigned int reg;

	reg = sdhci_readl(host, SDHCI_MULTI_CYCLE);
	reg &= ~SDHCI_EDGE_DETECT_EN;
	sdhci_writel(host, reg, SDHCI_MULTI_CYCLE);
}

static int nebula_get_best_edges(struct sdhci_host *host, u32 opcode,
				unsigned int *edge_p2f_ptr, unsigned int *edge_f2p_ptr)
{
	int err;
	unsigned int index, val;
	bool found = false;
	bool prev_found = false;
	unsigned int edge_p2f, edge_f2p, start, end;

	start = 0;
	end = PHASE_SCALE / EDGE_TUNING_PHASE_STEP;

	edge_p2f = start;
	edge_f2p = end;
	for (index = 0; index <= end; index++) {
		nebula_select_sample_phase(host, index * EDGE_TUNING_PHASE_STEP);

		err = nebula_send_tuning(host, opcode);
		if (err == 0) {
			val = sdhci_readl(host, SDHCI_MULTI_CYCLE);
			found = ((val & SDHCI_FOUND_EDGE) != 0);
		} else {
			found = true;
		}

		if (prev_found && !found) {
			edge_f2p = index;
		} else if (!prev_found && found) {
			edge_p2f = index;
		}

		if ((edge_p2f != start) && (edge_f2p != end))
			break;

		prev_found = found;
		found = false;
	}

	if ((edge_p2f == start) && (edge_f2p == end))
		return NOT_FOUND;

	*edge_p2f_ptr = edge_p2f;
	*edge_f2p_ptr = edge_f2p;

	return ERET_SUCCESS;
}

static unsigned int nebula_get_best_phase(struct sdhci_host *host, u32 opcode,
				unsigned int *edge_p2f_ptr, unsigned int *edge_f2p_ptr)
{
	unsigned int index, start, end;
	unsigned int phase, fall, rise;
	bool fall_updat_flag = false;
	int err;
	int prev_err = 0;

	start = *edge_p2f_ptr * EDGE_TUNING_PHASE_STEP;
	end = *edge_f2p_ptr * EDGE_TUNING_PHASE_STEP;
	if (end <= start)
		end += PHASE_SCALE;

	fall = start;
	rise = end;
	for (index = start; index <= end; index++) {
		nebula_select_sample_phase(host, index % PHASE_SCALE);

		err = nebula_send_tuning(host, opcode);
		if (err) {
			pr_debug("%s: send tuning CMD%u fail! phase:%d err:%d\n",
				mmc_hostname(host->mmc), opcode, index, err);
		}

		if ((err != 0) && (index == start)) {
			if (!fall_updat_flag) {
				fall_updat_flag = true;
				fall = start;
			}
		} else if ((prev_err == 0) && (err != 0)) {
			if (!fall_updat_flag) {
				fall_updat_flag = true;
				fall = index;
			}
		}

		if ((prev_err != 0) && (err == 0))
			rise = index;

		if ((err != 0) && (index == end)) {
			rise = end;
		}

		prev_err = err;
	}

	/* Calculate the center value by devide 2 */
	phase = ((fall + rise) / 2 + PHASE_SCALE / 2) % PHASE_SCALE;

	pr_info("%s: tuning done! valid phase shift [%d, %d] Final Phase:%d\n",
			mmc_hostname(host->mmc), rise % PHASE_SCALE,
			fall % PHASE_SCALE, phase);

	return phase;
}

static int nebula_exec_edge_tuning(struct sdhci_host *host, u32 opcode)
{
	int ret;
	unsigned int phase, edge_p2f, edge_f2p;
	struct sdhci_nebula *nebula = nebula_priv(host);

	nebula_pre_tuning(host);
	nebula_enable_edge_tuning(host);

	ret = nebula_get_best_edges(host, opcode, &edge_p2f, &edge_f2p);
	if (ret == NOT_FOUND) {
		pr_err("%s: tuning failed! can not found edge!\n", mmc_hostname(host->mmc));
		return ret;
	}

	nebula_disable_edge_tuning(host);

	phase = nebula_get_best_phase(host, opcode, &edge_p2f, &edge_f2p);

	nebula->tuning_phase = phase;
	nebula_select_sample_phase(host, phase);
	nebula_post_tuning(host);

	return ERET_SUCCESS;
}

int sdhci_nebula_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->priv_quirk & NEBULA_QUIRK_FPGA)
		return ERET_SUCCESS;

	if (nebula->priv_quirk & NEBULA_QUIRK_SAMPLE_TURNING) {
		return nebula_exec_sample_tuning(host, opcode);
	} else {
		return nebula_exec_edge_tuning(host, opcode);
	}
}

static void nebula_parse_priv_cap(struct sdhci_host *host,
	struct device_node *np)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (of_get_property(np, "reset_out_drv", NULL) == NULL)
		nebula->priv_cap |= NEBULA_CAP_RST_IN_DRV;

	if (of_get_property(np, "pm_runtime_enable", NULL))
		nebula->priv_cap |= NEBULA_CAP_PM_RUNTIME;

	if (of_get_property(np, "nm_card", NULL))
		nebula->priv_cap |= NEBULA_CAP_NM_CARD;
}

static void nebula_parse_priv_quirk(struct sdhci_host *host,
	const struct device_node *np)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (of_get_property(np, "sample_turning", NULL))
		nebula->priv_quirk |= NEBULA_QUIRK_SAMPLE_TURNING;

	if (of_get_property(np, "fpga", NULL))
		nebula->priv_quirk |= NEBULA_QUIRK_FPGA;

	if (of_get_property(np, "cd_inverted", NULL))
		nebula->priv_quirk |= NEBULA_QUIRK_CD_INVERTED;
}

static void nebula_of_parse(struct platform_device *pdev,
		struct sdhci_host *host)
{
	u32 bus_width;
	struct device *dev = &pdev->dev;

	if (device_property_present(dev, "mmc-broken-cmd23"))
		host->quirks2 |= SDHCI_QUIRK2_HOST_NO_CMD23;

	if (device_property_present(dev, "sdhci,1-bit-only") ||
		(device_property_read_u32(dev, "bus-width", &bus_width) == 0 &&
		bus_width == 1))
		host->quirks |= SDHCI_QUIRK_FORCE_1_BIT_DATA;
}

static int nebula_parse_comm_dt(struct platform_device *pdev,
		struct sdhci_host *host)
{
	int ret;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_nebula *nebula = nebula_priv(host);
	struct device_node *np = pdev->dev.of_node;

	ret = mmc_of_parse(host->mmc);
	if (ret) {
		dev_err(mmc_dev(host->mmc), "parse comm dt failed.\n");
		return ret;
	}

	if ((of_property_read_u32(np, "devid", &nebula->devid) != 0) ||
		(nebula->devid >= MMC_DEV_TYPE_MAX)) {
		return -EINVAL;
	}

	nebula_of_parse(pdev, host);

	nebula_parse_priv_cap(host, np);

	nebula_parse_priv_quirk(host, np);

#ifdef CONFIG_MMC_CQHCI
	if (of_get_property(np, "mmc-cmdq", NULL))
		host->mmc->caps2 |= MMC_CAP2_CQE | MMC_CAP2_CQE_DCMD;
#endif

	if (of_property_read_u32(np, "max-frequency", &host->mmc->f_max))
		host->mmc->f_max = MAX_FREQ;

	pltfm_host->clk = devm_clk_get(mmc_dev(host->mmc), "mmc_clk");
	if (IS_ERR_OR_NULL(pltfm_host->clk)) {
		dev_err(mmc_dev(host->mmc), "get clk failed\n");
		return -EINVAL;
	}

	nebula->hclk = devm_clk_get(mmc_dev(host->mmc), "mmc_hclk");
	if (IS_ERR_OR_NULL(nebula->hclk)) {
		dev_warn(mmc_dev(host->mmc), "dts no hclk\n");
	} else {
		ret = clk_prepare_enable(nebula->hclk);
		if (ret) {
			dev_err(mmc_dev(host->mmc), "enable hclk failed.\n");
			return -EINVAL;
		}
	}

	return ERET_SUCCESS;
}

static int nebula_parse_reset_dt(struct platform_device *pdev,
		struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if ((nebula->priv_cap & NEBULA_CAP_RST_IN_DRV) == 0) {
		nebula->crg_rst = devm_reset_control_get(&pdev->dev, "crg_reset");
		if (IS_ERR_OR_NULL(nebula->crg_rst)) {
			dev_err(&pdev->dev, "get crg_rst failed. %ld\n", \
						PTR_ERR(nebula->crg_rst));
			return (int)PTR_ERR(nebula->crg_rst);
		}

		nebula->dll_rst = devm_reset_control_get(&pdev->dev, "dll_reset");
		if (IS_ERR_OR_NULL(nebula->dll_rst)) {
			dev_err(&pdev->dev, "get dll_reset failed. %ld\n", \
						PTR_ERR(nebula->dll_rst));
			return (int)PTR_ERR(nebula->dll_rst);
		}

		/* below crg rst not must */
		nebula->crg_tx = devm_reset_control_get(&pdev->dev, "crg_tx");
		if (IS_ERR_OR_NULL(nebula->crg_tx)) {
			dev_warn(&pdev->dev, "crg tx rst not found with dts\n");
			nebula->crg_tx = nebula->crg_rst;
		}

		nebula->crg_rx = devm_reset_control_get(&pdev->dev, "crg_rx");
		if (IS_ERR_OR_NULL(nebula->crg_rx)) {
			dev_warn(&pdev->dev, "crg rx rst not found with dts\n");
			nebula->crg_rx = nebula->crg_rst;
		}

		nebula->samp_rst = devm_reset_control_get(&pdev->dev, "samp_rst");
		if (IS_ERR_OR_NULL(nebula->samp_rst)) {
			dev_warn(&pdev->dev, "crg samp rst not found with dts\n");
			nebula->samp_rst = NULL;
		}
	}

	return ERET_SUCCESS;
}

static int nebula_parse_regmap_dt(struct platform_device *pdev,
		struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);
	struct device_node *np = pdev->dev.of_node;

	nebula->crg_regmap = syscon_regmap_lookup_by_phandle(np,
								"crg_regmap");
	if (IS_ERR(nebula->crg_regmap)) {
		dev_err(&pdev->dev, "get crg regmap failed. %ld\n", \
					PTR_ERR(nebula->crg_regmap));
		return (int)PTR_ERR(nebula->crg_regmap);
	}

	nebula->iocfg_regmap = syscon_regmap_lookup_by_phandle(np,
								"iocfg_regmap");
	if (IS_ERR(nebula->iocfg_regmap)) {
		dev_err(&pdev->dev, "get iocfg regmap failed. %ld\n", \
					PTR_ERR(nebula->iocfg_regmap));
		return (int)PTR_ERR(nebula->iocfg_regmap);
	}

	return ERET_SUCCESS;
}

int sdhci_nebula_pltfm_init(struct platform_device *pdev,
		struct sdhci_host *host)
{
	int ret;
	struct sdhci_nebula *nebula = nebula_priv(host);

	ret = nebula_parse_comm_dt(pdev, host);
	if (ret) {
		dev_err(&pdev->dev, "parse comm dt failed\n");
		return ret;
	}

	ret = plat_host_pre_init(pdev, host);
	if (ret) {
		dev_err(&pdev->dev, "pltfm pre init failed\n");
		return ret;
	}

	/* check nebula *info and *mask valid? */
	if (nebula->info == NULL || nebula->mask == NULL) {
		dev_err(&pdev->dev, "info or mask data invalid\n");
		return -EINVAL;
	}

	ret = nebula_parse_reset_dt(pdev, host);
	if (ret)
		return ret;

	ret = nebula_parse_regmap_dt(pdev, host);
	if (ret)
		return ret;

#ifdef CONFIG_MMC_QUICKBOOT
	ret = mmc_fast_boot_init(host);
	if (ret)
		return ret;
#endif

	/* Do ZQ calibration */
	ret = plat_resistance_calibration(host);
	if (ret)
		return ret;

	ret = plat_crg_init(host);
	if (ret)
		return ret;

	host->tuning_delay = 1;
	plat_caps_quirks_init(host);
	host->mmc_host_ops.hs400_enhanced_strobe = plat_hs400_enhanced_strobe;

	return ERET_SUCCESS;
}

void sdhci_nebula_set_uhs_signaling(struct sdhci_host *host,
					 unsigned int timing)
{
	nebula_set_emmc_card(host);
	sdhci_set_uhs_signaling(host, timing);
	host->timing = timing;
	plat_set_drv_cap(host);
}

void sdhci_nebula_hw_reset(struct sdhci_host *host)
{
	sdhci_writel(host, 0x0, SDHCI_EMMC_HW_RESET);
	udelay(10); /* delay 10 us */
	sdhci_writel(host, 0x1, SDHCI_EMMC_HW_RESET);
	udelay(200); /* delay 200 us */
}

int sdhci_nebula_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);

	nebula_disable_card_clk(host);

	return ERET_SUCCESS;
}

int sdhci_nebula_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);

	nebula_enable_card_clk(host);

	return ERET_SUCCESS;
}

int sdhci_nebula_voltage_switch(struct sdhci_host *host, struct mmc_ios *ios)
{
	return plat_voltage_switch(host, ios);
}

void sdhci_nebula_extra_init(struct sdhci_host *host)
{
	host->error_count = 0;
	return plat_extra_init(host);
}

#ifdef CONFIG_MMC_CQHCI
static void nebula_init_card(struct mmc_host *host, struct mmc_card *card)
{
	u32 idx;
	/* eMMC spec: cid product name offset: 0, 7, 6, 5, 4, 11 */
	const u8 cid_pnm_offset[] = {0, 7, 6, 5, 4, 11};

	if (host == NULL || card == NULL) {
		pr_err("%s: null card or host\n", mmc_hostname(host));
		return;
	}

	if ((card->type == MMC_TYPE_MMC) && (host->caps2 & MMC_CAP2_CQE)) {
		u8 *raw_cid = (u8 *)card->raw_cid;

		/* Skip whitelist */
		if (g_mmc_flag & MMC_CMDQ_DIS_WHITELIST) {
			return;
		}

		/* Clear MMC CQE capblility */
		host->caps2 &= ~(MMC_CAP2_CQE | MMC_CAP2_CQE_DCMD);

		/* Manufacturer ID: b[127:120](eMMC v2.0 and upper), 0/24: idx/bit offset */
		card->cid.manfid = ((card->raw_cid[0] >> 24) & 0xFF);

		/* Decode CID with eMMC v2.0 and upper */
		for (idx = 0; idx < sizeof(cid_pnm_offset); idx++) {
			card->cid.prod_name[idx] = raw_cid[cid_pnm_offset[idx]];
		}
		card->cid.prod_name[++idx] = 0;
		mmc_fixup_device(card, mmc_cmdq_whitelist);
	}
}

static void nebula_controller_v4_enable(struct sdhci_host *host, bool enable)
{
	u16 ctrl;

	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	if (enable)
		ctrl |= SDHCI_CTRL_V4_ENABLE;
	else
		ctrl &= ~SDHCI_CTRL_V4_ENABLE;

	if (host->flags & SDHCI_USE_64_BIT_DMA)
		ctrl |= SDHCI_CTRL_64BIT_ADDR;

	sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
}

static void nebula_cqe_enable(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	u32 timeout = CQE_MAX_TIMEOUT;
	u16 reg, clk;
	u8 ctrl;

	/* SW_RST_DAT */
	sdhci_reset(host, SDHCI_RESET_DATA);

	nebula_controller_v4_enable(host, true);

	/* Set the DMA boundary value and block size */
	sdhci_writew(host, SDHCI_MAKE_BLKSZ(host->sdma_boundary,
					    MMC_BLOCK_SIZE), SDHCI_BLOCK_SIZE);

	/* need to set multitransfer for cmdq */
	reg = sdhci_readw(host, SDHCI_TRANSFER_MODE);
	reg |= SDHCI_TRNS_MULTI;
	reg |= SDHCI_TRNS_BLK_CNT_EN;
	sdhci_writew(host, reg, SDHCI_TRANSFER_MODE);

	/* ADMA2 only */
	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	ctrl &= ~SDHCI_CTRL_DMA_MASK;
	ctrl |= SDHCI_CTRL_ADMA32;
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk |= SDHCI_CLOCK_PLL_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	while (mmc->ops->card_busy(mmc)) {
		timeout--;
		if (!timeout) {
			pr_err("%s: cqe enable wait busy timeout\n", mmc_hostname(mmc));
			break;
		}
		udelay(1);
	}

	sdhci_cqe_enable(mmc);
}

static void nebula_cqe_disable(struct mmc_host *mmc, bool recovery)
{
	u32 timeout = CQE_MAX_TIMEOUT;

	while (mmc->ops->card_busy(mmc)) {
		timeout--;
		if (!timeout) {
			pr_err("%s: cqe disable wait busy timeout\n", mmc_hostname(mmc));
			break;
		}
		udelay(1);
	}

	nebula_controller_v4_enable(mmc_priv(mmc), 0);

	sdhci_cqe_disable(mmc, recovery);
}

static void nebula_dumpregs(struct mmc_host *mmc)
{
	sdhci_dumpregs(mmc_priv(mmc));
}

static const struct cqhci_host_ops sdhci_nebula_cqe_ops = {
	.enable = nebula_cqe_enable,
	.disable = nebula_cqe_disable,
	.dumpregs = nebula_dumpregs,
};

static int nebula_cqe_add_host(struct sdhci_host *host)
{
	int ret;
	struct cqhci_host *cq_host = NULL;
	bool dma64 = false;

	if (g_mmc_flag & MMC_CMDQ_FORCE_OFF) {
		/* force cmdq off by bootarges */
		host->mmc->caps2 &= ~(MMC_CAP2_CQE | MMC_CAP2_CQE_DCMD);
		return sdhci_add_host(host);
	}

	ret = sdhci_setup_host(host);
	if (ret)
		return ret;

	cq_host = devm_kzalloc(host->mmc->parent, sizeof(struct cqhci_host), GFP_KERNEL);
	if (cq_host == NULL) {
		pr_err("%s: allocate cqe host failed.\n", mmc_hostname(host->mmc));
		ret = -ENOMEM;
		goto cleanup;
	}

	cq_host->mmio = host->ioaddr + NEBULA_CQE_OFS;
	cq_host->ops = &sdhci_nebula_cqe_ops;

	/*
	 * synopsys controller has dma 128M algin limit,
	 * may split the trans descriptors
	 */
	cq_host->quirks |= CQHCI_QUIRK_TXFR_DESC_SZ_SPLIT;
	host->mmc->max_segs *= CQHCI_MAX_SEGS_MUL;

	dma64 = host->flags & SDHCI_USE_64_BIT_DMA;
	if (dma64)
		cq_host->caps |= CQHCI_TASK_DESC_SZ_128;

	ret = cqhci_init(cq_host, host->mmc, dma64);
	if (ret) {
		pr_err("%s: cqe init fail\n", mmc_hostname(host->mmc));
		return ret;
	}

	ret = __sdhci_add_host(host);
	if (ret)
		return ret;

	host->mmc_host_ops.init_card = nebula_init_card;

	return ERET_SUCCESS;

cleanup:
	sdhci_cleanup_host(host);
	return ret;
}

static u32 nebula_cqe_irq(struct sdhci_host *host, u32 intmask)
{
	int cmd_error = 0;
	int data_error = 0;

	if (!sdhci_cqe_irq(host, intmask, &cmd_error, &data_error))
		return intmask;

	cqhci_irq(host->mmc, intmask, cmd_error, data_error);

	return ERET_SUCCESS;
}
#else
static int nebula_cqe_add_host(struct sdhci_host *host)
{
	/* Rollback to no cqe mode */
	host->mmc->caps2 &= ~(MMC_CAP2_CQE | MMC_CAP2_CQE_DCMD);

	return sdhci_add_host(host);
}

static u32 nebula_cqe_irq(struct sdhci_host *host, u32 intmask)
{
	return intmask;
}
#endif

u32 sdhci_nebula_irq(struct sdhci_host *host, u32 intmask)
{
#ifdef CONFIG_SDHCI_NEBULA_DFX
	sdhci_nebula_dfx_irq(host, intmask);
#endif

	if (host->mmc->caps2 & MMC_CAP2_CQE)
		return nebula_cqe_irq(host, intmask);

	return intmask;
}

int sdhci_nebula_add_host(struct sdhci_host *host)
{
	int ret;

#ifdef CONFIG_MMC_QUICKBOOT
	if (mmc_is_fast_boot(host)) {
		host->mmc->rescan_entered = 1;
		/* Do not do repowerup before scan */
		host->mmc->caps2 |= MMC_CAP2_NO_PRESCAN_POWERUP;
		/* Skip reset device for fast boot */
		host->quirks |= SDHCI_QUIRK_NO_CARD_NO_RESET;
		sdhci_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
	}
#endif

	if (host->mmc->caps2 & MMC_CAP2_CQE) {
		ret = nebula_cqe_add_host(host);
	} else {
		ret = sdhci_add_host(host);
	}
	if (ret != ERET_SUCCESS)
		return ret;

#ifdef CONFIG_MMC_QUICKBOOT
	if (mmc_is_fast_boot(host)) {
		/* Clear SDHCI_QUIRK_NO_CARD_NO_RESET for normal reset */
		host->quirks &= ~SDHCI_QUIRK_NO_CARD_NO_RESET;

		mmc_parameter_init(host->mmc);
		ret = mmc_quick_init_card(host->mmc, mmc_get_rocr(host), NULL);
		if (ret) {
			host->mmc->rescan_entered = 0;
			mmc_detect_change(host->mmc, 0);
		}
		host->mmc->card_status = MMC_CARD_INIT;
		if (host->mmc->ops->card_info_save)
			host->mmc->ops->card_info_save(host->mmc);
	}
#endif
	return ret;
}

void sdhci_nebula_set_bus_width(struct sdhci_host *host, int width)
{
	u8 ctrl;

	if (width <= MMC_BUS_WIDTH_4) {
		ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
		ctrl &= ~SDHCI_CTRL_8BITBUS;
		sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	}

	return sdhci_set_bus_width(host, width);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
void sdhci_nebula_adma_write_desc(struct sdhci_host *host, void **desc,
		dma_addr_t addr, int len, unsigned int cmd)
{
	int split_len;

	/* work around for buffer across 128M boundary, split the buffer */
	if (((addr & (SZ_128M - 1)) + len) > SZ_128M) {
		split_len = SZ_128M - (int)(addr & (SZ_128M - 1));
		sdhci_adma_write_desc(host, desc, addr, split_len, ADMA2_TRAN_VALID);
		addr += split_len;
		len -= split_len;
	}

	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}
#endif

void sdhci_nebula_reset(struct sdhci_host *host, u8 mask)
{
	u8 ctrl;
	struct sdhci_nebula *nebula = nebula_priv(host);

#ifdef CONFIG_MMC_QUICKBOOT
	/* eMMC quick boot up no need reset */
	if ((nebula->devid == MMC_DEV_TYPE_MMC_0) && \
		((host->quirks & SDHCI_QUIRK_NO_CARD_NO_RESET) != 0)) {
		return;
	}
#endif
	sdhci_reset(host, mask);

	/* eMMC quick boot up no need reset */
	if (nebula->priv_quirk & NEBULA_QUIRK_CD_INVERTED) {
		ctrl = sdhci_readb(host, SDHCI_WAKE_UP_CONTROL);
		ctrl |= SDHCI_DETECT_POLARITY;
		sdhci_writeb(host, ctrl, SDHCI_WAKE_UP_CONTROL);
	}
}

#ifdef CONFIG_PM
int sdhci_nebula_pltfm_suspend(struct device *dev)
{
	int ret;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_nebula *nebula = nebula_priv(host);
#ifdef CONFIG_MMC_QUICKBOOT
	struct mmc_card *card = host->mmc->card;

	if (mmc_is_fast_boot(host)) {
		if ((card != NULL) && (mmc_card_suspended(card) != 0))
			mmc_set_cur_mode(host, SLEEP_MODE);
	}
#endif
	ret = sdhci_pltfm_suspend(dev);
	if (ret != 0) {
		pr_err("%s: pltfm suspend fail\n", mmc_hostname(host->mmc));
		return ret;
	}

	if (!IS_ERR_OR_NULL(nebula->hclk))
		clk_disable_unprepare(nebula->hclk);

	return ERET_SUCCESS;
}

int sdhci_nebula_pltfm_resume(struct device *dev)
{
	int ret;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_nebula *nebula = nebula_priv(host);

#ifdef CONFIG_MMC_QUICKBOOT
	struct mmc_card *card = host->mmc->card;

	if (mmc_is_fast_boot(host)) {
		if ((card != NULL) && (mmc_card_suspended(card) == 0))
			mmc_set_cur_mode(host, INIT_MODE);
	}
#endif
	if (!IS_ERR_OR_NULL(nebula->hclk)) {
		ret = clk_prepare_enable(nebula->hclk);
		if (ret != 0) {
			pr_err("%s: resume hclk enable fail\n", mmc_hostname(host->mmc));
			return ret;
		}
	}

	return sdhci_pltfm_resume(dev);
}
#endif
