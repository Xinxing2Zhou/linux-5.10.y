/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __SDHCI_FMEA_H__
#define __SDHCI_FMEA_H__

#include "sdhci.h"

#include "nebula_fmea.h"

#define SDHCI_FMEA_LIFE_TIME_EXCEED 955465100

#define DEVICE_LIFE_CHECK_INTERVAL (3600)
#define DEVICE_LIFE_TIME_EST_VAL (0x5)

typedef struct sdhci_nebula_fmea {
	struct sdhci_host *host;
	struct delayed_work mmc_lifecheck_work;
	int life_check_interval;
} nebula_fmea;

int sdhci_nebula_fmea_init(struct sdhci_host *host, nebula_fmea *fmea);
int sdhci_nebula_fmea_deinit(nebula_fmea *fmea);
#endif
