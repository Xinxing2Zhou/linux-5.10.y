/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef _DRIVERS_MMC_NEBULA_PLATFOMR_TIMING_H
#define _DRIVERS_MMC_NEBULA_PLATFOMR_TIMING_H

enum drv_timing_lvl {
	TM_LVL_0 = 0,
	TM_LVL_1,
	TM_LVL_2,
	TM_LVL_3,
	TM_LVL_4,
	TM_LVL_5,
	TM_LVL_6,
	TM_LVL_7,
	TM_LVL_8,
	TM_LVL_9,
	TM_LVL_10,
	TM_LVL_11,
	TM_LVL_12,
	TM_LVL_13,
	TM_LVL_14,
	TM_LVL_15,
	TM_LVL_MAX
};

enum sr_lvl {
	SR_LVL_0 = 0,
	SR_LVL_1,
	SR_LVL_2,
	SR_LVL_3,
	SR_LVL_MAX
};

enum phase_lvl {
	PHASE_LVL_0, /* 0 degree */
	PHASE_LVL_1, /* 11.25 degree */
	PHASE_LVL_2, /* 22.5 degree */
	PHASE_LVL_3, /* 33.75 degree */
	PHASE_LVL_4, /* 45 degree */
	PHASE_LVL_5, /* 56.25 degree */
	PHASE_LVL_6, /* 67.5 degree */
	PHASE_LVL_7, /* 78.75 degree */
	PHASE_LVL_8, /* 90 degree */
	PHASE_LVL_9, /* 101.25 degree */
	PHASE_LVL_10, /* 112.5 degree */
	PHASE_LVL_11, /* 123.75 degree */
	PHASE_LVL_12, /* 135 degree */
	PHASE_LVL_13, /* 146.25 degree */
	PHASE_LVL_14, /* 157.5 degree */
	PHASE_LVL_15, /* 168.75 degree */
	PHASE_LVL_16, /* 180 degree */
	PHASE_LVL_17, /* 191.25 degree */
	PHASE_LVL_18, /* 202.5 degree */
	PHASE_LVL_19, /* 213.75 degree */
	PHASE_LVL_20, /* 225 degree */
	PHASE_LVL_21, /* 236.25 degree */
	PHASE_LVL_22, /* 247.5 degree */
	PHASE_LVL_23, /* 258.75 degree */
	PHASE_LVL_24, /* 270 degree */
	PHASE_LVL_25, /* 281.25 degree */
	PHASE_LVL_26, /* 292.5 degree */
	PHASE_LVL_27, /* 303.75 degree */
	PHASE_LVL_28, /* 315 degree */
	PHASE_LVL_29, /* 326.25 degree */
	PHASE_LVL_30, /* 337.5 degree */
	PHASE_LVL_31, /* 348.75 degree */
};

#endif
