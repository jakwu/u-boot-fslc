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

#ifndef __MARSBOARD_CONFIG_H
#define __MARSBOARD_CONFIG_H

#define CONFIG_DEFAULT_FDT_FILE     "imx6q-marsboard.dtb"
#define CONFIG_SYS_FSL_USDHC_NUM    2
#define CONFIG_SYS_MMC_ENV_DEV      1  /* SDHC3 */

#define CONFIG_CMD_SF

#define CONFIG_ENV_IS_IN_SPI_FLASH

#include "mx6embest_common.h"

#endif
