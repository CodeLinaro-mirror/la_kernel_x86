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
#include <linux/regulator/consumer.h>
#include "platform_tmd26723.h"

static struct regulator *vprog2_reg;
#define VPROG2_VAL 2850000

/*
 * PMIC regulator vprog2 init
 */
static int tmd26723_platform_init(struct device *dev)
{
	int ret;

	vprog2_reg = regulator_get(dev, "vprog2");
	if (IS_ERR(vprog2_reg)) {
		dev_err(dev, "regulator_get failed\n");
		return PTR_ERR(vprog2_reg);
	}

	ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
	if (ret) {
		dev_err(dev, "regulator voltage set failed\n");
		regulator_put(vprog2_reg);
	}
	return ret;
}

/*
 * PMIC regulator vprog2 de-init
 */
static int tmd26723_platform_exit(struct device *dev)
{
	regulator_put(vprog2_reg);
	return 0;
}

static int tmd26723_power_on(struct device *dev)
{
	int ret;

	ret = regulator_enable(vprog2_reg);
	if (ret < 0) {
		dev_err(dev, "%s:is failed\n", __func__);
		return ret;
	}
	return 0;
}

static int tmd26723_power_off(struct device *dev)
{
	int ret;

	ret = regulator_disable(vprog2_reg);
	if (ret < 0) {
		dev_err(dev, "%s:is failed\n", __func__);
		return ret;
	}
	return 0;
}

void *tmd26723_ps_platform_data(void *info)
{
	static struct proximity_sensor_platform_data proximity_sensor_pdata;

	proximity_sensor_pdata.gpio_int = get_gpio_by_name("prox_int_n");

	if (INTEL_MID_BOARD(2, PHONE, MRFL, RBY, PRO) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, RBY, ENG)) {
		proximity_sensor_pdata.init = NULL;
		proximity_sensor_pdata.exit = NULL;
		proximity_sensor_pdata.power_on = NULL;
		proximity_sensor_pdata.power_off = NULL;
	} else {
		proximity_sensor_pdata.init = tmd26723_platform_init;
		proximity_sensor_pdata.exit = tmd26723_platform_exit;
		proximity_sensor_pdata.power_on = tmd26723_power_on;
		proximity_sensor_pdata.power_off = tmd26723_power_off;
	}

	return &proximity_sensor_pdata;
}
