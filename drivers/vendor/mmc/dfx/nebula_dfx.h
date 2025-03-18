/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __NEBULA_DFX_H__
#define __NEBULA_DFX_H__

#include "nebula_dfx.h"

typedef enum {
	DEBUG_LVL_INFO = 0,
	DEBUG_LVL_NOTICE,
	DEBUG_LVL_VERBOSE,
	DEBUG_LVL_MAX,
} nebula_dfx_log_lvl;

#define MAX_STR_LEN             20
#define MAX_NAMELEN             10
#define CLOCK_KHZ               1000

enum sd_speed_class {
	SD_SPEED_CLASS0 = 0,
	SD_SPEED_CLASS1,
	SD_SPEED_CLASS2,
	SD_SPEED_CLASS3,
	SD_SPEED_CLASS4,
	SD_SPEED_CLASS_MAX,
};

enum sd_uhs_speed_grade {
	SD_SPEED_GRADE0 = 0,
	SD_SPEED_GRADE1,
	SD_SPEED_GRADE_MAX,
};
	
#endif
