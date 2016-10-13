/*
 * platform_soc_thermal.c: Platform data for SoC DTS driver
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#define pr_fmt(fmt)  "intel_soc_thermal: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include "platform_soc_thermal.h"
#include "platform_ipc.h"

#include <asm/intel-mid.h>
#include <asm/intel_mid_thermal.h>

#define BYT_SOC_THRM_IRQ	86
#define BYT_SOC_THRM		"soc_thrm"

static struct resource res = {
		.flags = IORESOURCE_IRQ,
};

static struct soc_throttle_data tng_soc_data[] = {
	{
		.power_limit = 0xbb, /* 6W */
		.floor_freq = 0x00,
	},
	{
		.power_limit = 0x41, /* 2.1W */
		.floor_freq = 0x01,
	},
	{
		.power_limit = 0x1C, /* 0.9W */
		.floor_freq = 0x01,
	},
	{
		.power_limit = 0x1C, /* 0.9W */
		.floor_freq = 0x01,
	},
};

void *soc_thrm_device_handler(void *info)
{
	if (INTEL_MID_BOARD(1, PHONE, MRFL) ||
			INTEL_MID_BOARD(1, TABLET, MRFL))
		return &tng_soc_data;
}

static const struct devs_id soc_thrm_dev_id __initconst = {
	.name = "soc_thrm",
	.type = SFI_DEV_TYPE_IPC,
	.delay = 1,
	.get_platform_data = &soc_thrm_device_handler,
	.device_handler = &ipc_device_handler,
};

sfi_device(soc_thrm_dev_id);
