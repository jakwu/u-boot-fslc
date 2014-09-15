/*
 * Copyright (C) 2014 Eukréa Electromatique
 * Author: Eric Bénard <eric@eukrea.com>
 *
 * Configuration settings for the Embest RIoTboard
 *
 * based on mx6*sabre*.h which are :
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __RIOTBOARD_CONFIG_H
#define __RIOTBOARD_CONFIG_H


#define CONFIG_DEFAULT_FDT_FILE     "imx6dl-riotboard.dtb"
#define CONFIG_SYS_FSL_USDHC_NUM    3
#define CONFIG_SYS_MMC_ENV_DEV      2  /* SDHC4 */

#ifndef CONFIG_MFG
	#define CONFIG_ENV_IS_IN_MMC
#endif


#include "mx6embest_common.h"

#endif
