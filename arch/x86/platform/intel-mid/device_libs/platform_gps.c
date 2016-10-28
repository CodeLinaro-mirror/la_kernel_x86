/*
 * platform_gps.c: gps platform data initialization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/intel_mid_gps.h>
#include <linux/sfi.h>
#include <asm/intel-mid.h>
#include "platform_hsu.h"


#define GPS_GPIO_RESET	"GPS-Reset"
#define GPS_GPIO_ENABLE	"GPS-On"
#define GPS_GPIO_MCU_REQ	"GPS-Mcureq"
#define GPS_GPIO_MCU_REQ_RESP	"GPS-Mcureq_Resp"

static struct intel_mid_gps_platform_data gps_data = {
	.gpio_reset  = -EINVAL,
	.gpio_enable = -EINVAL,
	.gpio_mcu_req = -EINVAL,
	.gpio_mcu_req_resp = -EINVAL,
	.reset  = RESET_ON,
	.enable = ENABLE_OFF,
	.hsu_port = -EINVAL,
};

static struct platform_device intel_mid_gps_device = {
	.name			= "intel_mid_gps",
	.id			= -1,
	.dev			= {
		.platform_data	= &gps_data,
	},
};


void  __init *intel_mid_gps_platform_data(void *info)
{
	struct mid_board_info* entry = info;
	int ret;

	gps_data.gpio_reset  = get_gpio_by_name(GPS_GPIO_RESET);
	gps_data.gpio_enable = get_gpio_by_name(GPS_GPIO_ENABLE);
	gps_data.gpio_mcu_req = get_gpio_by_name(GPS_GPIO_MCU_REQ);
	gps_data.gpio_mcu_req_resp  = get_gpio_by_name(GPS_GPIO_MCU_REQ_RESP);
	gps_data.hsu_port = entry->bus_num;

	/* force a different HSU config for cg2000 */
	if (!strncmp(entry->name, "cg2000", SFI_NAME_LEN))
	    intel_mid_hsu_force_cfg(config_alternative);

	ret = platform_device_register(&intel_mid_gps_device);

	if (ret < 0)
		pr_err("platform_device_register failed for intel_mid_gps\n");

	return NULL;
}

static const struct devs_id bcm4774_dev_id __initconst = {
	    .name = "bcm4774",
		.type = SFI_DEV_TYPE_UART,
		.delay = 0,
		.get_platform_data = &intel_mid_gps_platform_data,
};

static const struct devs_id bcm4752_dev_id __initconst = {
	    .name = "bcm4752",
		.type = SFI_DEV_TYPE_UART,
		.delay = 0,
		.get_platform_data = &intel_mid_gps_platform_data,
};

static const struct devs_id bcm47521_dev_id __initconst = {
	    .name = "bcm47521",
		.type = SFI_DEV_TYPE_UART,
		.delay = 0,
		.get_platform_data = &intel_mid_gps_platform_data,
};

static const struct devs_id bcm47531_dev_id __initconst = {
	    .name = "bcm47531",
		.type = SFI_DEV_TYPE_UART,
		.delay = 0,
		.get_platform_data = &intel_mid_gps_platform_data,
};

static const struct devs_id cg2000_dev_id __initconst = {
	    .name = "cg2000",
		.type = SFI_DEV_TYPE_UART,
		.delay = 0,
		.get_platform_data = &intel_mid_gps_platform_data,
};

static const struct devs_id csrg05t_dev_id __initconst = {
	    .name = "csrg05t",
		.type = SFI_DEV_TYPE_UART,
		.delay = 0,
		.get_platform_data = &intel_mid_gps_platform_data,
};

sfi_device(bcm4774_dev_id);
sfi_device(bcm4752_dev_id);
sfi_device(bcm47521_dev_id);
sfi_device(bcm47531_dev_id);
sfi_device(cg2000_dev_id);
sfi_device(csrg05t_dev_id);
