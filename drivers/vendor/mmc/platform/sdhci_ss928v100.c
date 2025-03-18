/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#define EMMC_BUS_WIDTH_PHY_ADDR     0x11020018

#define IO_CFG_SDIO_MUX             0x1
#define IO_CFG_EMMC_MUX             0x2
#define IO_CFG_MUX_MASK             0xF

/* Host controller CRG */
#define EMMC_CRG_CLK_OFS            0x34C0
#define SDIO0_CRG_CLK_OFS           0x35C0
#define SDIO1_CRG_CLK_OFS           0x36C0
#define CRG_SRST_REQ                BIT(16)
#define CRG_CLK_EN_MASK             BIT(0) | BIT(1)

/* Host dll reset register */
#define EMMC_DLL_RST_OFS            0x34C4
#define SDIO0_DLL_RST_OFS           0x35C4
#define SDIO1_DLL_RST_OFS           0x36C4
#define CRG_DLL_SRST_REQ            BIT(1)

/* Host dll phase register */
#define EMMC_DRV_DLL_OFS            0x34C8
#define SDIO0_DRV_DLL_OFS           0x35C8
#define SDIO1_DRV_DLL_OFS           0x36C8
#define CRG_DRV_PHASE_SHIFT         15
#define CRG_DRV_PHASE_MASK          (0x1F << CRG_DRV_PHASE_SHIFT)

/* Host dll state register */
#define EMMC_DLL_STA_OFS            0x34D8
#define SDIO0_DLL_STA_OFS           0x35D8
#define SDIO1_DLL_STA_OFS           0x36D8
#define CRG_P4_DLL_LOCKED           BIT(9)
#define CRG_DS_DLL_READY            BIT(10)
#define CRG_SAM_DLL_READY           BIT(12)

/* Host drv cap config */
#define DRV_STR_SHIFT               4
#define DRV_STR_MASK_GPIO           (0xF << DRV_STR_SHIFT)
#define SR_STR_SHIFT                8
#define SR_STR_MASK_GPIO            (0x3 << SR_STR_SHIFT)
#define NEBULA_VOLT_SW_BVT          1

/* EMMC IO register offset */
#define EMMC_CLK_GPIO_OFS           0x00
#define EMMC_CMD_GPIO_OFS           0x04
#define EMMC_RSTN_GPIO_OFS          0x2C
#define EMMC_DQS_GPIO_OFS           0x28
#define EMMC_D0_GPIO_OFS            0x08
#define EMMC_D1_GPIO_OFS            0x0C
#define EMMC_D2_GPIO_OFS            0x10
#define EMMC_D3_GPIO_OFS            0x14
#define EMMC_D4_GPIO_OFS            0x18
#define EMMC_D5_GPIO_OFS            0x1C
#define EMMC_D6_GPIO_OFS            0x20
#define EMMC_D7_GPIO_OFS            0x24

/* ZQ calibration physical address */
#define EMMC_ZQ_CTRL_PHY_ADDR       0x10010004

/* SDIO0 IO register offset */
#define SDIO0_DETECT_OFS            0x80
#define SDIO0_PWEN_OFS              0x84
#define SDIO0_CLK_OFS               0x9C
#define SDIO0_CMD_OFS               0x88
#define SDIO0_D0_OFS                0x8C
#define SDIO0_D1_OFS                0x90
#define SDIO0_D2_OFS                0x94
#define SDIO0_D3_OFS                0x98

/* Voltage switch physical address */
#define SDIO0_VOLT_SW_PHY_ADDR      0x102E0010

/* SDIO1 IO register offset */
#define SDIO1_CLK_OFS               0x50
#define SDIO1_CMD_OFS               0x54
#define SDIO1_D0_OFS                0x40
#define SDIO1_D1_OFS                0x44
#define SDIO1_D2_OFS                0x48
#define SDIO1_D3_OFS                0x4C

#include "sdhci_nebula.h"
#include "platform_priv.h"
#include "platform_timing.h"

static const nebula_crg_mask g_crg_mask = NEBULA_CRG_MASK_DESC;

/* EMMC fixed timing parameter */
static nebula_timing g_timing_gpio_emmc[] = {
	[MMC_TIMING_LEGACY] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_1),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_2, SR_LVL_3),
		.timing[IO_TYPE_RST] = fixed_gpio_drv(TM_LVL_0, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_2, SR_LVL_3),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_MMC_HS] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_1),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_2, SR_LVL_3),
		.timing[IO_TYPE_RST] = fixed_gpio_drv(TM_LVL_0, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_2, SR_LVL_3),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_MMC_HS200] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_1),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_5, SR_LVL_3),
		.timing[IO_TYPE_RST] = fixed_gpio_drv(TM_LVL_0, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_5, SR_LVL_3),
		fixed_drv_phase_only(PHASE_LVL_18),
	},
	[MMC_TIMING_MMC_HS400] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_1),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_5, SR_LVL_3),
		.timing[IO_TYPE_RST] = fixed_gpio_drv(TM_LVL_0, SR_LVL_1),
		.timing[IO_TYPE_DQS] = fixed_gpio_drv(TM_LVL_4, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_5, SR_LVL_3),
		fixed_drv_phase_only(PHASE_LVL_7),
	},
};

/* EMMC info struct */
static const nebula_info g_gpio_emmc_info = \
	nebula_emmc_info_desc(g_timing_gpio_emmc);

/* SDIO0 fixed timing parameter */
static nebula_timing g_timing_gpio_sdio0[] = {
	[MMC_TIMING_LEGACY] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_9, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_3, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_3, SR_LVL_1),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_SD_HS] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_10, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_4, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_4, SR_LVL_1),
		fixed_drv_samp_phase(PHASE_LVL_18, PHASE_LVL_4),
	},
	[MMC_TIMING_UHS_SDR12] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_5, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_0, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_0, SR_LVL_1),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_UHS_SDR25] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_7, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_1, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_1, SR_LVL_1),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_UHS_SDR50] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_7, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_3, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_3, SR_LVL_1),
		fixed_drv_phase_only(PHASE_LVL_20),
	},
	[MMC_TIMING_UHS_SDR104] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_13, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_7, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_7, SR_LVL_1),
		fixed_drv_phase_only(PHASE_LVL_20),
	},
};

/* SDIO0 info struct */
static const nebula_info g_gpio_sdio0_info = \
	nebula_sdio0_info_desc(g_timing_gpio_sdio0);

/* SDIO1 fixed timing parameter */
static nebula_timing g_timing_gpio_sdio1[] = {
	[MMC_TIMING_LEGACY] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_8, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_2, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_2, SR_LVL_1),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_MMC_HS] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_9, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_3, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_3, SR_LVL_1),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_SD_HS] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_3, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_3, SR_LVL_1),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_UHS_SDR12] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_1, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_1, SR_LVL_1),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_UHS_SDR25] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_6, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_2, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_2, SR_LVL_1),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_UHS_SDR50] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_6, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_2, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_2, SR_LVL_1),
		fixed_drv_phase_only(PHASE_LVL_20),
	},
	[MMC_TIMING_UHS_SDR104] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_11, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_6, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_6, SR_LVL_1),
		fixed_drv_phase_only(PHASE_LVL_20),
	},
	[MMC_TIMING_MMC_HS200] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_11, SR_LVL_2),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_6, SR_LVL_1),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_6, SR_LVL_1),
		fixed_drv_phase_only(PHASE_LVL_20),
	},
};

/* SDIO1 info struct */
static const nebula_info g_gpio_sdio1_info = \
	nebula_sdio1_info_desc(g_timing_gpio_sdio1);

static inline void priv_set_io_mux(struct sdhci_host *host,
		unsigned int offset, unsigned int pin_mux)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	regmap_write_bits(nebula->iocfg_regmap, offset,
			  IO_CFG_MUX_MASK, pin_mux);
}

static void priv_io_mux_config(struct sdhci_host *host)
{
	u32 idx, bus_width, pin_mux;
	struct sdhci_nebula *nebula = nebula_priv(host);
	const nebula_info *info = nebula->info;

	if (nebula->priv_quirk & NEBULA_QUIRK_FPGA) {
		return;
	}

	pin_mux = (nebula->devid == MMC_DEV_TYPE_MMC_0) ? \
			IO_CFG_EMMC_MUX : IO_CFG_SDIO_MUX;

	priv_set_io_mux(host, info->io_offset[IO_TYPE_CLK], pin_mux);
	priv_set_io_mux(host, info->io_offset[IO_TYPE_CMD], pin_mux);

	if (host->mmc->caps & MMC_CAP_8_BIT_DATA) {
		bus_width = (1 << MMC_BUS_WIDTH_8);
	} else if (host->mmc->caps & MMC_CAP_4_BIT_DATA) {
		bus_width = (1 << MMC_BUS_WIDTH_4);
	} else {
		bus_width = (1 << MMC_BUS_WIDTH_1);
	}
	for (idx = IO_TYPE_D0; (idx < IO_TYPE_DMAX) && (bus_width != 0); idx++) {
		bus_width--;
		priv_set_io_mux(host, info->io_offset[idx], pin_mux);
	}

	if (nebula->devid == MMC_DEV_TYPE_MMC_0) { /* eMMC device */
		priv_set_io_mux(host, info->io_offset[IO_TYPE_RST], pin_mux);
		if (bus_width == (1 << MMC_BUS_WIDTH_8))
			priv_set_io_mux(host, info->io_offset[IO_TYPE_DQS], pin_mux);
	} else if (nebula->devid == MMC_DEV_TYPE_SDIO_0) {
		priv_set_io_mux(host, SDIO0_DETECT_OFS, pin_mux);
		priv_set_io_mux(host, SDIO0_PWEN_OFS, pin_mux);
	}
}

int plat_host_pre_init(struct platform_device *pdev, struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	nebula->mask = &g_crg_mask;
	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		nebula->info = &g_gpio_emmc_info;
		nebula->priv_cap |= NEBULA_CAP_ZQ_CALB;
	} else if (nebula->devid == MMC_DEV_TYPE_SDIO_0) {
		nebula->info = &g_gpio_sdio0_info;
		nebula->priv_cap |= NEBULA_CAP_VOLT_SW;
	} else if (nebula->devid == MMC_DEV_TYPE_SDIO_1) {
		nebula->info = &g_gpio_sdio1_info;
	} else {
		pr_err("error: invalid device\n");
		return -EINVAL;
	}

	return ERET_SUCCESS;
}

void plat_caps_quirks_init(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);
	struct mmc_host *mmc = host->mmc;

	plat_comm_caps_quirks_init(host);

	plat_set_mmc_bus_width(host);

	/* Initialization pin multiplexing first */
	priv_io_mux_config(host);

	if (!(mmc->caps2 & MMC_CAP2_NO_SDIO))
		nebula->priv_cap |= NEBULA_CAP_PM_RUNTIME;
}

void plat_extra_init(struct sdhci_host *host)
{
	u32 ctrl;
	struct sdhci_nebula *nebula = nebula_priv(host);

	ctrl = sdhci_readl(host, SDHCI_AXI_MBIU_CTRL);
	ctrl &= ~SDHCI_UNDEFL_INCR_EN;
	sdhci_writel(host, ctrl, SDHCI_AXI_MBIU_CTRL);

	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		ctrl = sdhci_readl(host, SDHCI_EMMC_CTRL);
		ctrl |= SDHCI_CARD_IS_EMMC;
		sdhci_writel(host, ctrl, SDHCI_EMMC_CTRL);
	}
}
