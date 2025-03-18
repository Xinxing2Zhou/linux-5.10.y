/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include "core.h"
#include "mmc_ops.h"
#include "drv_dft_event.h"
#include "nebula_fmea.h"

#ifndef CONFIG_LINUX_PRODUCT

static int g_check_immediately = 0;
module_param(g_check_immediately, int, 0644);

static int nebula_fmea_update_extcsd(struct mmc_card *card)
{
	int ret;
	u8 *ext_csd;

	mmc_claim_host(card->host);
	ret = mmc_get_ext_csd(card, &ext_csd);
	mmc_release_host(card->host);
	if (ret)
		return ret;

	card->ext_csd.device_life_time_est_typ_a = ext_csd[EXT_CSD_DEVICE_LIFE_TIME_EST_TYP_A];
	card->ext_csd.device_life_time_est_typ_b = ext_csd[EXT_CSD_DEVICE_LIFE_TIME_EST_TYP_B];
	kfree(ext_csd);

	return 0;
}

static noinline bool nebula_fmea_lifettime_exceed(struct mmc_card *card)
{
	u8 type_a, type_b;

	type_a = card->ext_csd.device_life_time_est_typ_a;
	type_b = card->ext_csd.device_life_time_est_typ_b;

	if ((type_a >= DEVICE_LIFE_TIME_EST_VAL) || (type_b >= DEVICE_LIFE_TIME_EST_VAL)) {
		return true;
	}

	return false;
}

static void nebula_fmea_check_lifetime(struct work_struct *work)
{
	int ret;
	td_handle handle;
	nebula_fmea *fmea =
		container_of(work, struct sdhci_nebula_fmea, mmc_lifecheck_work.work);
	struct mmc_card *card = fmea->host->mmc->card;

	if ((card == NULL) || (card->type != MMC_TYPE_MMC)) {
		goto out;
	}

	if ((fmea->life_check_interval > 0) && (g_check_immediately == 0)) {
		fmea->life_check_interval--;
		goto out;
	}

	fmea->life_check_interval = DEVICE_LIFE_CHECK_INTERVAL;
	ret = nebula_fmea_update_extcsd(card);
	if (ret != 0) {
		goto out;
	}

	if (nebula_fmea_lifettime_exceed(card)) {
		ret = dft_drv_event_create(SDHCI_FMEA_LIFE_TIME_EXCEED, &handle);
	    if (ret == TD_SUCCESS) {
			dft_drv_event_put_string(handle, "PNAME", current->comm);
			dft_drv_event_put_string(handle, "F1NAME", __func__);
			dft_drv_event_put_string(handle, "FLASH_TYPE", "EMMC");
			dft_drv_event_put_integral(handle, "DEVICE_LIFE_TIME_EST_TYP_A", \
										card->ext_csd.device_life_time_est_typ_a);
			dft_drv_event_put_integral(handle, "DEVICE_LIFE_TIME_EST_TYP_B", \
										card->ext_csd.device_life_time_est_typ_b);
			dft_drv_event_report(handle);
			dft_drv_event_destroy(handle);
	    }
	}

out:
	queue_delayed_work(system_freezable_wq, (struct delayed_work *)work, HZ);
}

int sdhci_nebula_fmea_init(struct sdhci_host *host, nebula_fmea *fmea)
{
	struct delayed_work *work = &fmea->mmc_lifecheck_work;

	fmea->host = host;
	fmea->life_check_interval = DEVICE_LIFE_CHECK_INTERVAL;
	INIT_DELAYED_WORK(work, nebula_fmea_check_lifetime);
	queue_delayed_work(system_freezable_wq, work, HZ);

	return 0;
}

int sdhci_nebula_fmea_deinit(nebula_fmea *fmea)
{
	cancel_delayed_work(&fmea->mmc_lifecheck_work);

	return 0;
}
#else
int sdhci_nebula_fmea_init(struct sdhci_host *host, nebula_fmea *fmea)
{
	return 0;
}

int sdhci_nebula_fmea_deinit(nebula_fmea *fmea)
{
	return 0;
}
#endif

