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
#include <linux/sfi.h>
#include <asm/intel-mid.h>

void __init *lsm6ds3h_platform_data(void *info)
{
	static struct st_lsm6ds3h_platform_data lsm6ds3h_pdata;

	lsm6ds3h_pdata.gpio_int1 = get_gpio_by_name("accel_int1");	/* ACCEL_INT_1 <-> GPIO46 */
	lsm6ds3h_pdata.gpio_conf = NULL;

	return &lsm6ds3h_pdata;
}

static const struct devs_id lsm6ds3h_dev_id __initconst = {
	.name = "lsm6ds3h",
	.type = SFI_DEV_TYPE_I2C,
	.delay = 1,
	.get_platform_data = &lsm6ds3h_platform_data,
	.device_handler = NULL,
};

sfi_device(lsm6ds3h_dev_id);
