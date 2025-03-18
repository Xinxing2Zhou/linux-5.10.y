/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef _DRIVERS_MMC_SDHCI_NEBULA_H
#define _DRIVERS_MMC_SDHCI_NEBULA_H

#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/mmc/host.h>

#include "sdhci-pltfm.h"
#include "sdhci_nebula.h"

/* The operation completed successfully. */
#define ERET_SUCCESS 0

#define SDHCI_HL_EDGE_TUNING /* enable edge tuning */

#define PHASE_SCALE             32
#define EDGE_TUNING_PHASE_STEP  4
#define NOT_FOUND               (-1)
#define MAX_TUNING_NUM          1
#define WIN_DIV                 2
#define WIN_MASK                0x3
#define WIN_RISE                0x2
#define WIN_FALL                0x1

#define MAX_FREQ    200000000
#define MMC_BLOCK_SIZE	512

#ifndef CQHCI_QUIRK_TXFR_DESC_SZ_SPLIT
#define CQHCI_QUIRK_TXFR_DESC_SZ_SPLIT	0x2
#endif
#define CQHCI_MAX_SEGS_MUL          2
#define SDHCI_CTRL_64BIT_ADDR       0x2000
#define SDHCI_CTRL_V4_ENABLE        0x1000

#define CQE_MAX_TIMEOUT             10000
/* Software auto suspend delay */
#define MMC_AUTOSUSPEND_DELAY_MS    50

/*
 * Nebula extended host controller registers.
 */
#define SDHCI_EMMC_CTRL         0x52C
#define SDHCI_CARD_IS_EMMC      0x0001
#define SDHCI_ENH_STROBE_EN     0x0100

#define SDHCI_EMMC_HW_RESET     0x534

#define SDHCI_AT_CTRL           0x540
#define SDHCI_SAMPLE_EN         0x00000010

#define SDHCI_AXI_MBIU_CTRL     0x510
#define SDHCI_UNDEFL_INCR_EN    0x1

#define SDHCI_AT_STAT           0x544
#define SDHCI_PHASE_SEL_MASK    0x000000FF

#define SDHCI_MULTI_CYCLE       0x54C
#define SDHCI_FOUND_EDGE        (0x1 << 11)
#define SDHCI_EDGE_DETECT_EN    (0x1 << 8)
#define SDHCI_DOUT_EN_F_EDGE    (0x1 << 6)
#define SDHCI_DATA_DLY_EN       (0x1 << 3)
#define SDHCI_CMD_DLY_EN        (0x1 << 2)

#define SDHCI_MSHC_CTRL_R       0x508
#define SDHCI_DEBUG1_PORT       0x520
#define SDHCI_DEBUG2_PORT       0x524
#define SDHCI_GP_OUT_R          0x534
#define SDHCI_EMAX_R            0x548
#define SDHCI_MUTLI_CYCLE_EN    0x54C

#define NEBULA_CQE_OFS          0x180

#define SDHCI_DETECT_POLARITY   BIT(3)

#define CONFIG_SDHCI_NEBULA_DFX
#define NEBULA_DFX_BT_MAX_NUM   8

#define INVALID_DATA            0xFFFFFFFF

#define MCI_SLOT_NUM            4

enum mmc_crg_type {
	CRG_CLK_RST = 0,
	CRG_DLL_RST,
	CRG_DRV_DLL,
	CRG_DLL_STA,
	CRG_TYPE_MAX,
};

enum phase_type {
	DRV_PHASE = 0,
	SAMP_PHASE,
	PHASE_MAX,
};

enum mmc_io_type {
	MMC_IO_TYPE_IO,
	MMC_IO_TYPE_GPIO,
	MMC_IO_TYPE_MAX,
};

enum mmc_dev_type {
	MMC_DEV_TYPE_MMC_0  = 0,
	MMC_DEV_TYPE_SDIO_0,
	MMC_DEV_TYPE_SDIO_1,
	MMC_DEV_TYPE_MAX,
};

/**
 * mmc spec define 5 io types,
 * data line with 8 bit width
 */
enum io_type {
	IO_TYPE_CLK = 0,
	IO_TYPE_CMD,
	IO_TYPE_RST,
	IO_TYPE_DET = IO_TYPE_RST,
	IO_TYPE_DQS,
	IO_TYPE_PWE = IO_TYPE_DQS,
	IO_TYPE_DATA,
	IO_TYPE_MAX,
	IO_TYPE_D0 = IO_TYPE_DATA,
	IO_TYPE_D1 = IO_TYPE_MAX,
	IO_TYPE_D2,
	IO_TYPE_D3,
	IO_TYPE_D4,
	IO_TYPE_D5,
	IO_TYPE_D6,
	IO_TYPE_D7,
	IO_TYPE_DMAX,
};

/**
 * struct mmc_timing_s - nebula host timing data array
 * @data_valid: this timing valid?
 * @timing: io timing array
 * @phase: phase timing array
 */
typedef struct mmc_timing_s {
	bool data_valid;
	u32 timing[IO_TYPE_MAX];
	u32 phase[PHASE_MAX];
} nebula_timing;

/**
 * struct nebula_info_s - nebula host info data array
 * @io_offset: io pin register offset array
 * @io_drv_mask: io pin driver cap configure mask
 * @io_drv_str_bit_ofs: io pin driver cap bit offset
 * @io_drv_str_mask: io pin driver cap mask
 * @io_drv_sr_bit_ofs: io pin driver cap bit offset
 * @io_drv_sr_mask: io pin driver cap mask
 * @crg_ofs: host crg register offset array
 * @zq_phy_addr: zq resistance calibration physical addr
 * @volt_sw_phy_addr: voltage switch ctrl phisical addr
 * @bus_width_phy_addr: get bus width phisical addr
 * @qboot_phy_addr: emmc quick boot parameters phisical addr
 * @qboot_param1_ofs: emmc quick boot parameter1 offset
 * @timing_size: host fixed timing size
 * @timing: host fixed timing point
 */
typedef struct nebula_info_s {
	u32 io_offset[IO_TYPE_DMAX];
	u32 io_drv_mask;
	u32 io_drv_str_bit_ofs;
	u32 io_drv_str_mask;
	u32 io_drv_sr_bit_ofs;
	u32 io_drv_sr_mask;
	u32 crg_ofs[CRG_TYPE_MAX];
	phys_addr_t zq_phy_addr;
	phys_addr_t volt_sw_phy_addr;
	phys_addr_t bus_width_phy_addr;
	phys_addr_t qboot_phy_addr;
	u32 qboot_param1_ofs;
	u32 timing_size;
	nebula_timing *timing;
}nebula_info;

/**
 * struct nebula_crg_mask_s - nebula host crg mask info
 * @crg_srst_mask: reset/unreset mmc controller mask
 * @crg_cken_mask: mmc controller clock enable mask
 * @crg_clk_sel_ofs: mmc controller clock select bit offset
 * @crg_clk_sel_mask: mmc controller clock select mask
 * @dll_srst_mask: dll reset/unreset mask
 * @p4_lock_mask: wait p4 lock status mask
 * @dll_ready_mask: wait dll ready mask
 * @samp_ready_mask: wait sample ready mask
 * @drv_phase_mask: driver phase setting mask
 */
typedef struct nebula_crg_mask_s {
	u32 crg_srst_mask;
	u32 crg_clk_sel_ofs;
	u32 crg_clk_sel_mask;
	u32 crg_cken_mask;
	u32 dll_srst_mask;
	u32 p4_lock_mask;
	u32 dll_ready_mask;
	u32 samp_ready_mask;
	u32 drv_phase_mask;
	u32 volt_sw_en_mask;
	u32 volt_sw_1v8_mask;
} nebula_crg_mask;

typedef struct nebula_cmd_info {
	u32 opcode[NEBULA_DFX_BT_MAX_NUM];
	u32 sp;
} nebula_cmd_bt;

typedef struct nebula_cap_s {
	u32 help            : 1;  // [0]
	u32 log_level       : 7;  // [7:1]
} nebula_cap;

struct sdhci_nebula {
	enum mmc_io_type io_type;      /* io type: gpio or high speed io */
	struct reset_control *crg_rst; /* reset handle for host controller */
	struct reset_control *crg_tx;
	struct reset_control *crg_rx;
	struct reset_control *dll_rst;
	struct reset_control *samp_rst;
	struct regmap *crg_regmap;     /* regmap for host controller */
	struct regmap *iocfg_regmap;
	const nebula_crg_mask *mask;
	const nebula_info *info;       /* io, timing, crg info */
	struct clk *hclk;              /* AHB clock */

	u32 priv_cap;
#define NEBULA_CAP_PM_RUNTIME   (1 << 0) /* Support PM runtime */
#define NEBULA_CAP_QUICK_BOOT   (1 << 1) /* Support quick boot */
#define NEBULA_CAP_RST_IN_DRV   (1 << 2) /* Support reset in driver */
#define NEBULA_CAP_VOLT_SW      (1 << 3) /* Support voltage switch */
#define NEBULA_CAP_ZQ_CALB      (1 << 4) /* Support ZQ resistance calibration */
#define NEBULA_CAP_NM_CARD      (1 << 5) /* This controller plugged NM card */

	u32 priv_quirk;                       /* Deviations from soc. */
#define NEBULA_QUIRK_FPGA                 (1 << 0) /* FPGA board */
#define NEBULA_QUIRK_SAMPLE_TURNING       (1 << 1) /* for not support edge turning */
#define NEBULA_QUIRK_CD_INVERTED          (1 << 2) /* This card detect inverted */

	unsigned int devid;         /* device id, mapping to enum mmc_dev_type */
	unsigned int drv_phase;
	unsigned int sample_phase;
	unsigned int tuning_phase;
	nebula_cmd_bt cmd_bt;
	nebula_cap dfx_cap;
	struct proc_dir_entry *proc_root;
	struct proc_dir_entry *proc_stat;
	void __iomem *qboot_virt_addr;
};

static inline void *nebula_priv(struct sdhci_host *host)
{
	return sdhci_pltfm_priv(sdhci_priv(host));
}

extern struct mmc_host *g_mci_host[MCI_SLOT_NUM];
extern struct mmc_host *g_sdio_mmc_host;

/* Export api by platform */
void plat_extra_init(struct sdhci_host *host);
void plat_set_drv_cap(struct sdhci_host *host);
void plat_get_drv_samp_phase(struct sdhci_host *host);
void plat_set_drv_phase(struct sdhci_host *host, u32 phase);
void plat_dll_reset_assert(struct sdhci_host *host);
void plat_dll_reset_deassert(struct sdhci_host *host);
void plat_caps_quirks_init(struct sdhci_host *host);
void plat_dump_io_info(struct sdhci_host *host);
void plat_set_mmc_bus_width(struct sdhci_host *host);
void plat_set_emmc_type(struct sdhci_host *host);
void plat_hs400_enhanced_strobe(struct mmc_host *mmc, struct mmc_ios *ios);
int plat_crg_init(struct sdhci_host *host);
int plat_wait_sample_dll_ready(struct sdhci_host *host);
int plat_wait_p4_dll_lock(struct sdhci_host *host);
int plat_wait_ds_dll_ready(struct sdhci_host *host);
int plat_host_init(struct platform_device *pdev, struct sdhci_host *host);
int plat_voltage_switch(struct sdhci_host *host, struct mmc_ios *ios);
int plat_host_pre_init(struct platform_device *pdev, struct sdhci_host *host);
int plat_resistance_calibration(struct sdhci_host *host);

/*
 * Export api for sdhci ops (by adapter)
 */
void sdhci_nebula_extra_init(struct sdhci_host *host);
void sdhci_nebula_set_uhs_signaling(struct sdhci_host *host, unsigned int timing);
void sdhci_nebula_hw_reset(struct sdhci_host *host);
void sdhci_nebula_set_clock(struct sdhci_host *host, unsigned int clk);
int sdhci_nebula_execute_tuning(struct sdhci_host *host, u32 opcode);
int sdhci_nebula_voltage_switch(struct sdhci_host *host, struct mmc_ios *ios);
int sdhci_nebula_pltfm_init(struct platform_device *pdev, struct sdhci_host *host);
int sdhci_nebula_runtime_suspend(struct device *dev);
int sdhci_nebula_runtime_resume(struct device *dev);
int sdhci_nebula_add_host(struct sdhci_host *host);
int sdhci_nebula_pltfm_suspend(struct device *dev);
int sdhci_nebula_pltfm_resume(struct device *dev);
u32 sdhci_nebula_irq(struct sdhci_host *host, u32 intmask);
void sdhci_nebula_dump_vendor_regs(struct sdhci_host *host);
void sdhci_nebula_set_bus_width(struct sdhci_host *host, int width);
void sdhci_nebula_adma_write_desc(struct sdhci_host *host, void **desc,
		dma_addr_t addr, int len, unsigned int cmd);
void sdhci_nebula_reset(struct sdhci_host *host, u8 mask);

/*
 * Export api by dfx
 */
int sdhci_nebula_proc_init(struct sdhci_host *host);
int sdhci_nebula_proc_shutdown(struct sdhci_host *host);
void sdhci_nebula_dfx_irq(struct sdhci_host *host, u32 intmask);

#endif /* _DRIVERS_MMC_SDHCI_NEBULA_H */
