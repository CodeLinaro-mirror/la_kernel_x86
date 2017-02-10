/*
 * platform_lsm6ds3h.c: lsm6ds3h platform data initilization file
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/platform_data/st_lsm6ds3h_pdata.h>
#include <asm/intel_scu_flis.h>
#include "platform_lsm6ds3h.h"


static int gpio_conf(void) {
	int ret1, ret2;

	ret1 = config_pin_flis(tng_gp_i2c_6_sda, PULL, NONE);
	ret2 = config_pin_flis(tng_gp_i2c_6_scl, PULL, NONE);

	if (ret1 < 0)
		return ret1;
	if (ret2 < 0)
		return ret2;
	return 0;
}

void *lsm6ds3h_platform_data(void *info)
{
	static struct st_lsm6ds3h_platform_data lsm6ds3h_pdata;

	lsm6ds3h_pdata.gpio_int1 = get_gpio_by_name("accel_int1");	/* ACCEL_INT_1 <-> GPIO46 */

	if (INTEL_MID_BOARD(3, PHONE, MRFL, GLC, ENG, 4) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, GLC, PRO, 4)||
			INTEL_MID_BOARD(3, PHONE, MRFL, MVN, ENG, 4)||
			INTEL_MID_BOARD(3, PHONE, MRFL, MVN, PRO, 4)) {
		lsm6ds3h_pdata.gpio_conf = gpio_conf;
	} else
		lsm6ds3h_pdata.gpio_conf = NULL;

	if (INTEL_MID_BOARD(2, PHONE, MRFL, SHA, ENG) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, SHA, PRO))
		lsm6ds3h_pdata.tilt_thres = ST_LSM6DS3H_WRIST_TILT_THRESHOLD_15DEG;
	else
		lsm6ds3h_pdata.tilt_thres = ST_LSM6DS3H_WRIST_TILT_THRESHOLD_20DEG;

	return &lsm6ds3h_pdata;
}
