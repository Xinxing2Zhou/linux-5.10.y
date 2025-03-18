/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef GMAC_PROC_H
#define GMAC_PROC_H

#include <linux/sockios.h>

#define SIOCSETPM	(SIOCDEVPRIVATE + 4) /* set pmt wake up config */
#define SIOCSETSUSPEND	(SIOCDEVPRIVATE + 5) /* call dev->suspend, debug */
#define SIOCSETRESUME	(SIOCDEVPRIVATE + 6) /* call dev->resume, debug */

void gmac_proc_create(void);
void gmac_proc_destroy(void);

/* netdev ops related func */
int set_suspend(int eth_n);
int set_resume(int eth_n);

#endif
