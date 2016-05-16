/*
 * platform_tmd26723.c: TMD26723 proximity sensor platform data initilization file
 *
 * (C) Copyright 2016 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GUN General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/intel-mid.h>
#include "platform_lps22hb.h"
#include <linux/platform_data/st_lps22hb_pdata.h>

void *lps22hb_platform_data(void *info)
{
	static struct st_lps22hb_platform_data lps22hb_pdata;

	lps22hb_pdata.gpio_int = get_gpio_by_name("baro_int_n");

	return &lps22hb_pdata;
}
