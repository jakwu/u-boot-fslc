/*
 * Copyright (C) 2014 MSC Vertriebs GmbH, Design Center Aachen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef __MX6DL_NANORISC_CONFIG_H
#define __MX6DL_NANORISC_CONFIG_H

#define CONFIG_MACH_TYPE			4500
#define CONFIG_MMCROOT				"/dev/mmcblk0p2"
#define CONFIG_DEFAULT_FDT_FILE		"imx6dl-nanoRISC.dtb"
#define CONFIG_BOOT_ZIMAGE
#define CONFIG_SYS_USE_SPINOR
#define CONFIG_SYS_USE_NAND

#include "mx6x_nanoRISC_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_MX6DL_NANORISC

#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_SYS_MMC_ENV_PART		0

/*
 * SPLASH SCREEN Configs
 */
#if defined(CONFIG_SPLASH_SCREEN) && defined(CONFIG_MXC_EPDC)
	/*
	 * Framebuffer and LCD
	 */
	#define CONFIG_CMD_BMP
	#define CONFIG_LCD
	#define CONFIG_FB_BASE				(CONFIG_SYS_TEXT_BASE + 0x300000)
	#define CONFIG_SYS_CONSOLE_IS_IN_ENV
	#undef LCD_TEST_PATTERN
	/* #define CONFIG_SPLASH_IS_IN_MMC			1 */
	#define LCD_BPP					LCD_MONOCHROME
	/* #define CONFIG_SPLASH_SCREEN_ALIGN		1 */

	#define CONFIG_WORKING_BUF_ADDR			(CONFIG_SYS_TEXT_BASE + 0x100000)
	#define CONFIG_WAVEFORM_BUF_ADDR		(CONFIG_SYS_TEXT_BASE + 0x200000)
	#define CONFIG_WAVEFORM_FILE_OFFSET		0x600000
	#define CONFIG_WAVEFORM_FILE_SIZE		0xF0A00
	#define CONFIG_WAVEFORM_FILE_IN_MMC

#ifdef CONFIG_SPLASH_IS_IN_MMC
	#define CONFIG_SPLASH_IMG_OFFSET		0x4c000
	#define CONFIG_SPLASH_IMG_SIZE			0x19000
#endif
#endif /* CONFIG_SPLASH_SCREEN && CONFIG_MXC_EPDC */

#endif /* __MX6DL_NANORISC_CONFIG_H */
