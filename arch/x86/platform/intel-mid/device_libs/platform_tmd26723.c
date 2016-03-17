/*
 * platform_ps_tmd26723.c: TMD26723 proximity sensor platform data initilization file
 *
 * (C) Copyright 2016 Inter Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GUN General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/platform_device.h>
#include <linux/input/tmd26723.h>
#include <asm/intel-mid.h>
#include "platform_tmd26723.h"
void *tmd26723_ps_platform_data(void *info)
{
	static struct proximity_sensor_platform_data proximity_sensor_pdata;

	proximity_sensor_pdata.gpio_int = get_gpio_by_name("prox_int_n");
	proximity_sensor_pdata.init = NULL;
	proximity_sensor_pdata.exit = NULL;
	proximity_sensor_pdata.power_on = NULL;
	proximity_sensor_pdata.power_off = NULL;

	return &proximity_sensor_pdata;
}
