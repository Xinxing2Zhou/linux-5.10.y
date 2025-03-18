/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/mmc/mmc.h>
#include "sdhci_nebula.h"
#include "nebula_intf.h"

/*
 * This api is for wifi driver rescan the sdio device
 */
int bsp_sdio_rescan(int slot)
{
	struct mmc_host *mmc = NULL;

	if ((slot >= MCI_SLOT_NUM) || (slot < 0)) {
		pr_err("invalid mmc slot, please check the argument\n");
		return -EINVAL;
	}

	mmc = g_mci_host[slot];
	if (mmc == NULL) {
		pr_err("invalid mmc, please check the argument\n");
		return -EINVAL;
	}

	mmc->rescan_entered = 0;
	mmc_detect_change(mmc, 0);
	return 0;
}
EXPORT_SYMBOL_GPL(bsp_sdio_rescan);

int hl_drv_sdio_rescan(int index)
{
	struct mmc_host *mmc = g_sdio_mmc_host;

	if (mmc == NULL) {
		pr_err("invalid mmc_host for SDIO\n");
		return -EINVAL;
	}
	if (index != 0) {
		pr_err("invalic mmc_host index for sdio %d\n", index);
		return -EINVAL;
	}

	mmc->rescan_entered = 0;
	mmc_detect_change(mmc, 0);
	return 0;
}
EXPORT_SYMBOL_GPL(hl_drv_sdio_rescan);
