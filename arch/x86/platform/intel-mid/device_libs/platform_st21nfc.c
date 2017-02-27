/*
 * platform_st21nfc.c: st21nfc platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <asm/intel-mid.h>
#include <linux/st21nfc.h>
#include "platform_st21nfc.h"



static unsigned int nfc_host_int_gpio, nfc_enable_gpio, nfc_fw_reset_gpio;


static int st21nfc_nfc_request_resources(struct i2c_client *client)
{
	int ret;

	ret = gpio_request(nfc_host_int_gpio, NFC_HOST_INT_GPIO);
	if (ret) {
		dev_err(&client->dev, "Request NFC INT GPIO fails %d\n", ret);
		return -1;
	}

	ret = gpio_direction_input(nfc_host_int_gpio);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_int;
	}

	ret = gpio_request(nfc_fw_reset_gpio, NFC_ENABLE_GPIO);
	if (ret) {
		dev_err(&client->dev,
			"Request for NFC Enable GPIO fails %d\n", ret);
		goto err_int;
	}

	ret = gpio_direction_output(nfc_enable_gpio, 1);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_enable;
	}

	ret = gpio_request(nfc_enable_gpio, NFC_FW_RESET_GPIO);
	if (ret) {
		dev_err(&client->dev,
			"Request for NFC FW Reset GPIO fails %d\n", ret);
		goto err_enable;
	}

	ret = gpio_direction_output(nfc_fw_reset_gpio, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_fw;
	}

	return 0;
err_fw:
	gpio_free(nfc_fw_reset_gpio);
err_enable:
	gpio_free(nfc_enable_gpio);
err_int:
	gpio_free(nfc_host_int_gpio);
	return -1;
}

void *st21nfc_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = (struct i2c_board_info *) info;
	static struct st21nfc_i2c_platform_data st21nfc_nfc_platform_data;

	memset(&st21nfc_nfc_platform_data, 0x00,
		sizeof(struct st21nfc_i2c_platform_data));

	nfc_host_int_gpio = get_gpio_by_name(NFC_HOST_INT_GPIO);
	if (nfc_host_int_gpio == -1)
		return NULL;
	nfc_enable_gpio = get_gpio_by_name(NFC_FW_RESET_GPIO);
	if (nfc_enable_gpio  == -1)
		return NULL;
	nfc_fw_reset_gpio = get_gpio_by_name(NFC_ENABLE_GPIO);
	if (nfc_fw_reset_gpio == -1)
		return NULL;

	st21nfc_nfc_platform_data.irq_gpio = nfc_host_int_gpio;
	st21nfc_nfc_platform_data.ena_gpio = nfc_enable_gpio;
	st21nfc_nfc_platform_data.reset_gpio = nfc_fw_reset_gpio;

	st21nfc_nfc_platform_data.polarity_mode = IRQF_TRIGGER_FALLING;

	i2c_info->irq = nfc_host_int_gpio + INTEL_MID_IRQ_OFFSET;
	i2c_info->addr = 0x08;
	st21nfc_nfc_platform_data.request_resources =
		st21nfc_nfc_request_resources;

	return &st21nfc_nfc_platform_data;
}

static const struct devs_id st21nfc_dev_id __initconst = {
	.name = "st21nfc",
	.type = SFI_DEV_TYPE_I2C,
	.delay = 0,
	.get_platform_data = &st21nfc_platform_data,
};

sfi_device(st21nfc_dev_id);
