/*
 * platform_pn5xx.c: pn544 platform data initilization file
 *
 * (C) Copyright 2016 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/pn5xx_i2c.h>
#include <asm/intel-mid.h>
#include "platform_pn5xx.h"

void *pn544_platform_data(void *info)
{
	static struct pn544_i2c_platform_data plat;

	plat.irq_gpio = get_gpio_by_name(NFC_HOST_INT_GPIO);
	if (plat.irq_gpio == -1)
		return NULL;

	plat.ven_gpio = get_gpio_by_name(NFC_ENABLE_GPIO);
	if (plat.ven_gpio  == -1)
		return NULL;

	plat.firm_gpio = get_gpio_by_name(NFC_FW_RESET_GPIO);
	if (plat.firm_gpio == -1)
		return NULL;

	return &plat;
}

static const struct devs_id pn544_dev_id __initconst = {
	.name = "pn544",
	.type = SFI_DEV_TYPE_I2C,
	.delay = 0,
	.get_platform_data = &pn544_platform_data,
};

sfi_device(pn544_dev_id);

