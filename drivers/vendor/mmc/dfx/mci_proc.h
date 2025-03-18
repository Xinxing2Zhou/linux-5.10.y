/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __MCI_PROC_H__
#define __MCI_PROC_H__

#include "mci_proc.h"

#define MAX_CARD_TYPE	4
#define MAX_SPEED_MODE	5

int mci_proc_init(void);
int mci_proc_shutdown(void);

#endif /*  __MCI_PROC_H__ */
