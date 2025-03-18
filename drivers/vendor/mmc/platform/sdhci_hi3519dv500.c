/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/regmap.h>

#include "sdhci.h"

/* Host controller CRG */
#define EMMC_CRG_CLK_OFS            0x34c0
#define SDIO0_CRG_CLK_OFS           0x35c0
#define SDIO1_CRG_CLK_OFS           0x36c0
#define CRG_SRST_REQ                (BIT(16) | BIT(17) | BIT(18))
#define CRG_CLK_EN_MASK             (BIT(0) | BIT(1))
#define CRG_CLK_BIT_OFS             24
#define CRG_CLK_SEL_MASK            (0x7 << CRG_CLK_BIT_OFS)

/* Host dll reset register */
#define EMMC_DLL_RST_OFS            0x34c4
#define SDIO0_DLL_RST_OFS           0x35c4
#define SDIO1_DLL_RST_OFS           0x36c4
#define CRG_DLL_SRST_REQ            BIT(1)

/* Host dll phase register */
#define EMMC_DRV_DLL_OFS            0x34c8
#define SDIO0_DRV_DLL_OFS           0x35c8
#define SDIO1_DRV_DLL_OFS           0x36c8
#define CRG_DRV_PHASE_SHIFT         15
#define CRG_DRV_PHASE_MASK          (0x1F << CRG_DRV_PHASE_SHIFT)

/* Host dll state register */
#define EMMC_DLL_STA_OFS            0x34d8
#define SDIO0_DLL_STA_OFS           0x35d8
#define SDIO1_DLL_STA_OFS           0x36d8
#define CRG_P4_DLL_LOCKED           BIT(9)
#define CRG_DS_DLL_READY            BIT(10)
#define CRG_SAM_DLL_READY           BIT(12)

/* Host drv cap config */
#define DRV_STR_SHIFT               4
#define DRV_STR_MASK_GPIO           (0xF << DRV_STR_SHIFT)
#define SR_STR_SHIFT                10
#define SR_STR_MASK_GPIO            (0x1 << SR_STR_SHIFT)
#define NEBULA_VOLT_SW_BVT          1

/* EMMC IO register offset */
#define EMMC_CLK_GPIO_OFS           0x34
#define EMMC_CMD_GPIO_OFS           0x38
#define EMMC_RSTN_GPIO_OFS          0x4c
#define EMMC_DQS_GPIO_OFS           0x50
#define EMMC_D0_GPIO_OFS            0x3c
#define EMMC_D1_GPIO_OFS            0x40
#define EMMC_D2_GPIO_OFS            0x44
#define EMMC_D3_GPIO_OFS            0x48
#define EMMC_D4_GPIO_OFS            0x24
#define EMMC_D5_GPIO_OFS            0x28
#define EMMC_D6_GPIO_OFS            0x2c
#define EMMC_D7_GPIO_OFS            0x30

#define EMMC_BUS_WIDTH_PHY_ADDR     0x11020018
#define EMMC_QUICK_BOOT_PHY_ADDR    0x11120020
#define EMMC_QUICK_BOOT_PARAM1_OFS  0x4

/* SDIO0 IO register offset */
#define SDIO0_DETECT_OFS            0xd0
#define SDIO0_PWEN_OFS              0xd4
#define SDIO0_CLK_OFS               0xcc
#define SDIO0_CMD_OFS               0xb8
#define SDIO0_D0_OFS                0xbc
#define SDIO0_D1_OFS                0xc0
#define SDIO0_D2_OFS                0xc4
#define SDIO0_D3_OFS                0xc8

/* Voltage switch physical address */
#define SDIO0_VOLT_SW_PHY_ADDR      0x11024700

/* SDIO1 IO register offset */
#define SDIO1_CLK_OFS               0xb0
#define SDIO1_CMD_OFS               0xb4
#define SDIO1_D0_OFS                0xa0
#define SDIO1_D1_OFS                0xa4
#define SDIO1_D2_OFS                0xa8
#define SDIO1_D3_OFS                0xac

#include "sdhci_nebula.h"
#include "platform_priv.h"
#include "platform_timing.h"

static const nebula_crg_mask g_crg_mask = NEBULA_CRG_MASK_DESC;

/* EMMC fixed timing parameter */
static nebula_timing g_timing_gpio_emmc[] = {
	[MMC_TIMING_LEGACY] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_2, SR_LVL_0),
		.timing[IO_TYPE_RST] = fixed_gpio_drv(TM_LVL_0, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_2, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_MMC_HS] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		.timing[IO_TYPE_RST] = fixed_gpio_drv(TM_LVL_0, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_MMC_HS200] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_15, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_12, SR_LVL_0),
		.timing[IO_TYPE_RST] = fixed_gpio_drv(TM_LVL_0, SR_LVL_0),
		.timing[IO_TYPE_DQS] = fixed_gpio_drv(TM_LVL_0, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_12, SR_LVL_0),
		fixed_drv_phase_only(PHASE_LVL_18),
	},
	[MMC_TIMING_MMC_HS400] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_15, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_15, SR_LVL_0),
		.timing[IO_TYPE_RST] = fixed_gpio_drv(TM_LVL_0, SR_LVL_0),
		.timing[IO_TYPE_DQS] = fixed_gpio_drv(TM_LVL_0, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_15, SR_LVL_0),
		fixed_drv_phase_only(PHASE_LVL_9),
	},
};

/* EMMC info struct */
static const nebula_info g_gpio_emmc_info = \
	nebula_emmc_info_desc(g_timing_gpio_emmc);

/* SDIO0 fixed timing parameter */
static nebula_timing g_timing_gpio_sdio0[] = {
	[MMC_TIMING_LEGACY] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_2, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_2, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_MMC_HS] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_19, PHASE_LVL_4),
	},
	[MMC_TIMING_MMC_HS200] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_8, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_6, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_6, SR_LVL_0),
		fixed_drv_phase_only(PHASE_LVL_20),
	},
	[MMC_TIMING_SD_HS] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_UHS_SDR12] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_9, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_7, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_7, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_UHS_SDR25] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_9, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_7, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_7, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_UHS_SDR50] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_12, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_10, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_10, SR_LVL_0),
		fixed_drv_phase_only(PHASE_LVL_20),
	},
	[MMC_TIMING_UHS_SDR104] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_8, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_6, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_6, SR_LVL_0),
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
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_2, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_2, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_MMC_HS] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_MMC_HS200] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_15, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_11, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_11, SR_LVL_0),
		fixed_drv_phase_only(PHASE_LVL_20),
	},
	[MMC_TIMING_SD_HS] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_4, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_3, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_UHS_SDR12] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_9, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_7, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_7, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_0),
	},
	[MMC_TIMING_UHS_SDR25] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_9, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_7, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_7, SR_LVL_0),
		fixed_drv_samp_phase(PHASE_LVL_16, PHASE_LVL_4),
	},
	[MMC_TIMING_UHS_SDR50] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_12, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_10, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_10, SR_LVL_0),
		fixed_drv_phase_only(PHASE_LVL_20),
	},
	[MMC_TIMING_UHS_SDR104] = {
		.data_valid = true,
		.timing[IO_TYPE_CLK] = fixed_gpio_drv(TM_LVL_15, SR_LVL_0),
		.timing[IO_TYPE_CMD] = fixed_gpio_drv(TM_LVL_15, SR_LVL_0),
		.timing[IO_TYPE_DATA] = fixed_gpio_drv(TM_LVL_15, SR_LVL_0),
		fixed_drv_phase_only(PHASE_LVL_19),
	},
};

/* SDIO1 info struct */
static const nebula_info g_gpio_sdio1_info = \
	nebula_sdio1_info_desc(g_timing_gpio_sdio1);

int plat_host_pre_init(struct platform_device *pdev, struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	nebula->mask = &g_crg_mask;
	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		nebula->info = &g_gpio_emmc_info;
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

void plat_extra_init(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		plat_set_emmc_type(host);
	}
}

static void priv_plat_mux_init(struct sdhci_host *host)
{
	u32 i;
	u32 bus_width = 1;
	struct sdhci_nebula *nebula = nebula_priv(host);
	const nebula_info *info = nebula->info;

	if (host->mmc->caps & MMC_CAP_8_BIT_DATA) {
		bus_width = (1 << MMC_BUS_WIDTH_8);
	} else if (host->mmc->caps & MMC_CAP_4_BIT_DATA) {
		bus_width = (1 << MMC_BUS_WIDTH_4);
	}

	if (nebula->devid == MMC_DEV_TYPE_MMC_0) {
		regmap_write(nebula->iocfg_regmap, info->io_offset[IO_TYPE_CLK], \
			0x11e1); /* 0x11e1: pinmux value */

		regmap_write(nebula->iocfg_regmap, info->io_offset[IO_TYPE_CMD], \
			0x1391); /* 0x1391: pinmux value */

		for (i = IO_TYPE_D0; i < (IO_TYPE_D0 + bus_width); i++)
			regmap_write(nebula->iocfg_regmap, info->io_offset[i], \
				(i < IO_TYPE_D4) ? 0x1391 : 0x1393); /* 0x1391, 0x1393: pinmux value */

		regmap_write(nebula->iocfg_regmap, info->io_offset[IO_TYPE_RST], \
			0x1301); /* 0x1301: pinmux value */

		if (host->mmc->caps & MMC_CAP_8_BIT_DATA)
			regmap_write(nebula->iocfg_regmap, info->io_offset[IO_TYPE_DQS], \
				0x1101); /* 0x1101: pinmux value */
	} else if (nebula->devid == MMC_DEV_TYPE_SDIO_0 || \
		nebula->devid == MMC_DEV_TYPE_SDIO_1) {
		if (nebula->devid == MMC_DEV_TYPE_SDIO_0) {
			regmap_write(nebula->iocfg_regmap, info->io_offset[IO_TYPE_DET], \
				0x1901); /* 0x1901: pinmux value */
			regmap_write(nebula->iocfg_regmap, info->io_offset[IO_TYPE_PWE], \
				0x1501); /* 0x1501: pinmux value */
		}

		regmap_write(nebula->iocfg_regmap, info->io_offset[IO_TYPE_CLK], \
			0x1101); /* 0x1101: pinmux value */
		regmap_write(nebula->iocfg_regmap, info->io_offset[IO_TYPE_CMD], \
			0x1301); /* 0x1301: pinmux value */

		for (i = IO_TYPE_D0; i < (IO_TYPE_D0 + bus_width); i++)
			regmap_write(nebula->iocfg_regmap, info->io_offset[i], \
				0x1301); /* 0x1301: pinmux value */
	}
}

void plat_caps_quirks_init(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	plat_comm_caps_quirks_init(host);

	plat_set_mmc_bus_width(host);
	if ((nebula->priv_cap & NEBULA_CAP_QUICK_BOOT) == 0)
		priv_plat_mux_init(host);

	if ((host->mmc->caps2 & MMC_CAP2_NO_SDIO) == 0)
		nebula->priv_cap |= NEBULA_CAP_PM_RUNTIME;
}
