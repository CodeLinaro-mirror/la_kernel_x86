/*
 * platform_bq25898.c: bq25898 platform data initilization file
 *
 * (C) Copyright 2016 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_BQ25898_H_
#define _PLATFORM_BQ25898_H_


#include <linux/sfi.h>
#include <asm/intel-mid.h>
#include <linux/power/bq25898_charger.h>
#define SHASTA_CHRGR_DEV_NAME	"bq25898_charger"

#define PMIC_SRAM_INTR_MAP 0xFFFFF616
#define PMIC_EXT_INTR_MASK 0x01

#define bq25898_CHRG_CUR_LOW		100	/* 100mA */
#define bq25898_CHRG_CUR_MEDIUM		500	/* 500mA */
#define bq25898_CHRG_CUR_HIGH		900	/* 900mA */
#define bq25898_CHRG_CUR_NOLIMIT	2500	/* 2500mA */

extern void __initdata *bq25898_platform_data(
			void *info) __attribute__((weak));
#endif
