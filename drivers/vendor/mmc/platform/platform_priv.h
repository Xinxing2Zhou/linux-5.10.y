/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef _DRIVERS_MMC_NEBULA_PLATFOMR_H
#define _DRIVERS_MMC_NEBULA_PLATFOMR_H

#include "platform_priv.h"

#define DUMP_DATA_IO_STEP       2

#define WAIT_MAX_TIMEOUT        20

/* ZQ resistance calibration confiuration */
#define EMMC_ZQ_INIT_EN                0x1
#define EMMC_ZQ_ZCAL_EN                (0x1 << 3)
#define EMMC_ZQ_CHECK_TIMES            100

/* Voltage switch configure */
#ifdef NEBULA_VOLT_SW_BVT
#define PWR_CTRL_BY_MISC          BIT(0)
#define PWR_CTRL_BY_MISC_EN       BIT(4)
#define PWR_CRTL_EN               (PWR_CTRL_BY_MISC | PWR_CTRL_BY_MISC_EN)
#define IO_MODE_SEL_1V8           BIT(1)
#define PWRSW_SEL_1V8             BIT(5)
#define PWR_CRTL_1V8              (IO_MODE_SEL_1V8 | PWRSW_SEL_1V8)
#else
#define PWR_CRTL_EN               BIT(0)
#define PWR_CRTL_1V8              BIT(2)
#endif

#define BOOT_MEDIA_EMMC           0xC
#define EMMC_BOOT_8BIT            BIT(11)
#define BOOT_FLAG_MASK            (0x3 << 2)

#ifndef CRG_CLK_BIT_OFS
#define CRG_CLK_BIT_OFS           INVALID_DATA
#define CRG_CLK_SEL_MASK          INVALID_DATA
#endif

/* Help macro for create nebula crg mask info struct */
#if defined(CRG_SRST_REQ) && defined(CRG_DLL_SRST_REQ)
#define NEBULA_CRG_MASK_DESC \
{ \
	.crg_srst_mask = CRG_SRST_REQ, \
	.crg_clk_sel_ofs = CRG_CLK_BIT_OFS, \
	.crg_clk_sel_mask = CRG_CLK_SEL_MASK, \
	.crg_cken_mask = CRG_CLK_EN_MASK, \
	.dll_srst_mask = CRG_DLL_SRST_REQ, \
	.p4_lock_mask = CRG_P4_DLL_LOCKED, \
	.dll_ready_mask = CRG_DS_DLL_READY, \
	.samp_ready_mask = CRG_SAM_DLL_READY, \
	.drv_phase_mask = CRG_DRV_PHASE_MASK, \
	.volt_sw_en_mask = PWR_CRTL_EN, \
	.volt_sw_1v8_mask = PWR_CRTL_1V8, \
}
#endif

#ifndef EMMC_ZQ_CTRL_PHY_ADDR
#define EMMC_ZQ_CTRL_PHY_ADDR       0x0
#endif
#ifndef EMMC_VOLT_SW_PHY_ADDR
#define EMMC_VOLT_SW_PHY_ADDR       0x0
#endif
#ifndef EMMC_BUS_WIDTH_PHY_ADDR
#define EMMC_BUS_WIDTH_PHY_ADDR     0x0
#endif

#ifndef EMMC_QUICK_BOOT_PHY_ADDR
#define EMMC_QUICK_BOOT_PHY_ADDR    INVALID_DATA
#define EMMC_QUICK_BOOT_PARAM1_OFS  INVALID_DATA
#endif

/* Help macro for create nebula emmc info description struct */
#if defined(EMMC_CRG_CLK_OFS) && defined(EMMC_D7_GPIO_OFS)
#define nebula_emmc_info_desc(_timing) \
{ \
	.io_offset = {EMMC_CLK_GPIO_OFS, EMMC_CMD_GPIO_OFS, \
		EMMC_RSTN_GPIO_OFS, EMMC_DQS_GPIO_OFS, \
		EMMC_D0_GPIO_OFS, EMMC_D1_GPIO_OFS, \
		EMMC_D2_GPIO_OFS, EMMC_D3_GPIO_OFS, \
		EMMC_D4_GPIO_OFS, EMMC_D5_GPIO_OFS, \
		EMMC_D6_GPIO_OFS, EMMC_D7_GPIO_OFS}, \
	.io_drv_mask = DRV_STR_MASK_GPIO | SR_STR_MASK_GPIO, \
	.io_drv_str_bit_ofs = DRV_STR_SHIFT, \
	.io_drv_str_mask = DRV_STR_MASK_GPIO, \
	.io_drv_sr_bit_ofs = SR_STR_SHIFT, \
	.io_drv_sr_mask = SR_STR_MASK_GPIO, \
	.crg_ofs = {EMMC_CRG_CLK_OFS, EMMC_DLL_RST_OFS, EMMC_DRV_DLL_OFS, EMMC_DLL_STA_OFS}, \
	.zq_phy_addr = EMMC_ZQ_CTRL_PHY_ADDR, \
	.volt_sw_phy_addr = EMMC_VOLT_SW_PHY_ADDR, \
	.bus_width_phy_addr = EMMC_BUS_WIDTH_PHY_ADDR, \
	.qboot_phy_addr = EMMC_QUICK_BOOT_PHY_ADDR, \
	.qboot_param1_ofs = EMMC_QUICK_BOOT_PARAM1_OFS, \
	.timing_size = ARRAY_SIZE(_timing), \
	.timing = (_timing), \
}
#endif

#ifndef SDIO0_ZQ_CTRL_PHY_ADDR
#define SDIO0_ZQ_CTRL_PHY_ADDR      0x0
#endif

#ifndef SDIO0_VOLT_SW_PHY_ADDR
#define SDIO0_VOLT_SW_PHY_ADDR      0x0
#endif

#ifndef SDIO0_DETECT_OFS
#define SDIO0_DETECT_OFS            INVALID_DATA
#endif

#ifndef SDIO0_PWEN_OFS
#define SDIO0_PWEN_OFS              INVALID_DATA
#endif

/* Help macro for create nebula sdio0 info description struct */
#if defined(SDIO0_CLK_OFS) && defined(SDIO0_CRG_CLK_OFS)
#define nebula_sdio0_info_desc(_timing) \
{ \
	.io_offset = {SDIO0_CLK_OFS, SDIO0_CMD_OFS, SDIO0_DETECT_OFS, \
				SDIO0_PWEN_OFS, SDIO0_D0_OFS, \
				SDIO0_D1_OFS, SDIO0_D2_OFS, SDIO0_D3_OFS}, \
	.io_drv_mask = DRV_STR_MASK_GPIO | SR_STR_MASK_GPIO, \
	.crg_ofs = {SDIO0_CRG_CLK_OFS, SDIO0_DLL_RST_OFS, SDIO0_DRV_DLL_OFS, SDIO0_DLL_STA_OFS}, \
	.zq_phy_addr = SDIO0_ZQ_CTRL_PHY_ADDR, \
	.volt_sw_phy_addr = SDIO0_VOLT_SW_PHY_ADDR, \
	.timing_size = ARRAY_SIZE(_timing), \
	.timing = (_timing), \
}
#endif

#ifndef SDIO1_ZQ_CTRL_PHY_ADDR
#define SDIO1_ZQ_CTRL_PHY_ADDR       0x0
#endif

#ifndef SDIO1_VOLT_SW_PHY_ADDR
#define SDIO1_VOLT_SW_PHY_ADDR       0x0
#endif

#ifndef SDIO1_DETECT_OFS
#define SDIO1_DETECT_OFS            INVALID_DATA
#endif

#ifndef SDIO1_PWEN_OFS
#define SDIO1_PWEN_OFS              INVALID_DATA
#endif

/* Help macro for create nebula sdio1 info description struct */
#if defined(SDIO0_CLK_OFS) && defined(SDIO0_CRG_CLK_OFS)
#define nebula_sdio1_info_desc(_timing) \
{ \
	.io_offset = {SDIO1_CLK_OFS, SDIO1_CMD_OFS, SDIO1_DETECT_OFS, \
				SDIO1_PWEN_OFS, SDIO1_D0_OFS, \
				SDIO1_D1_OFS, SDIO1_D2_OFS, SDIO1_D3_OFS}, \
	.io_drv_mask = DRV_STR_MASK_GPIO | SR_STR_MASK_GPIO, \
	.crg_ofs = {SDIO1_CRG_CLK_OFS, SDIO1_DLL_RST_OFS, SDIO1_DRV_DLL_OFS, SDIO1_DLL_STA_OFS}, \
	.zq_phy_addr = SDIO1_ZQ_CTRL_PHY_ADDR, \
	.volt_sw_phy_addr = SDIO1_VOLT_SW_PHY_ADDR, \
	.timing_size = ARRAY_SIZE(_timing), \
	.timing = (_timing), \
}
#endif

/* Help macro for create nebula emmc high speed info description struct */
#if defined(EMMC_D0_IO_OFS) && defined(EMMC_CRG_CLK_OFS)
#define nebula_emmc_hsio_info_desc(_timing) \
{ \
	.io_offset = {EMMC_CLK_IO_OFS, EMMC_CMD_IO_OFS, \
		EMMC_RSTN_IO_OFS, EMMC_DQS_IO_OFS, \
		EMMC_D0_IO_OFS, EMMC_D1_IO_OFS, \
		EMMC_D2_IO_OFS, EMMC_D3_IO_OFS, \
		EMMC_D4_IO_OFS, EMMC_D5_IO_OFS, \
		EMMC_D6_IO_OFS, EMMC_D7_IO_OFS}, \
	.io_drv_mask = DRV_STR_MASK_IO, \
	.io_drv_str_bit_ofs = DRV_STR_SHIFT, \
	.io_drv_str_mask = DRV_STR_MASK_IO, \
	.io_drv_sr_bit_ofs = SR_STR_SHIFT, \
	.io_drv_sr_mask = SR_STR_MASK_GPIO, \
	.crg_ofs = {EMMC_CRG_CLK_OFS, EMMC_DLL_RST_OFS, EMMC_DRV_DLL_OFS, EMMC_DLL_STA_OFS}, \
	.zq_phy_addr = EMMC_ZQ_CTRL_PHY_ADDR, \
	.volt_sw_phy_addr = EMMC_VOLT_SW_PHY_ADDR, \
	.qboot_phy_addr = EMMC_QUICK_BOOT_PHY_ADDR, \
	.qboot_param1_ofs = EMMC_QUICK_BOOT_PARAM1_OFS, \
	.timing_size = ARRAY_SIZE(_timing), \
	.timing = (_timing), \
}
#endif

#define TIMING_MASK            0x7FFFFFFF
#define TIMING_VALID           0x80000000
#define is_timing_valid(x)     (((x) & TIMING_VALID) == TIMING_VALID)

/* Generial GPIO CFG */
#if defined(DRV_STR_SHIFT) && defined(SR_STR_SHIFT)
#define gpio_drv_sel(str)  ((str) << DRV_STR_SHIFT)
#define gpio_sr_sel(sr)  ((sr) << SR_STR_SHIFT)
#define fixed_gpio_drv(drv_sel, sr_sel) \
	(TIMING_VALID | gpio_drv_sel(drv_sel) | gpio_sr_sel(sr_sel))
#endif

/* High speed IO CFG */
#if defined(DRV_STR_SHIFT)
#define hsio_drv_sel(str)  ((str) << DRV_STR_SHIFT)
#define fixed_hsio_drv(drv_sel) (TIMING_VALID | hsio_drv_sel(drv_sel))
#endif

#ifdef CRG_DRV_PHASE_SHIFT
/* Set drv phase only */
#define fixed_drv_phase_only(drv_phase) \
	.phase[DRV_PHASE] = (TIMING_VALID | ((drv_phase) << CRG_DRV_PHASE_SHIFT))

/* Set drv and sample phase */
#define fixed_drv_samp_phase(drv_phase, samp_phase) \
	.phase[DRV_PHASE] = (TIMING_VALID | ((drv_phase) << CRG_DRV_PHASE_SHIFT)), \
	.phase[SAMP_PHASE] = (TIMING_VALID | (samp_phase))
#endif

struct sdhci_host;
void plat_comm_caps_quirks_init(struct sdhci_host *host);

#endif /* _DRIVERS_MMC_NEBULA_ADAPTER_H */
