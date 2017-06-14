/*
 * platform_tsl258x.c: TAOS TSL258x light sensor platform data initilization file
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/tsl258x_als.h>
#include <asm/intel-mid.h>
#include <linux/tsl258x_als.h>
#include <asm/intel_scu_flis.h>
#include "platform_tsl258x.h"

/******************************************************************
 * TSL2584TSV lux equation on Marvin is of the form:
 * Lux1 = 1000 * ((ch0 - (2.16 * ch1)) / (Atime * Again))
 * Lux2 = 1000 * ((0.95 * ch0) - (1.11 * ch1) / (Atime * Again))
 * Lux = MAX(Lux1, Lux2)
 *
 * in other form:
 * lux1 = (1000 * ch0 - 2160 * ch1) / (Atime * Again)
 * lux2 = (950 * ch0 - 1110 * ch1) / (Atime * Again)
 * Lux = MAX(Lux1, Lux2)
 *****************************************************************/
static const struct lux_coefficients mvn_coeffs = {
        .ch0_coeff0 = 1000,
        .ch1_coeff0 = 2160,
        .ch0_coeff1 = 950,
        .ch1_coeff1 = 1110,
        .gain_ratio = 16,
};

static const struct lux_coefficients spl_coeffs = {
        .ch0_coeff0 = 33974,
        .ch1_coeff0 = 43487,
        .ch0_coeff1 = 27655,
        .ch1_coeff1 = 32785,
        .gain_ratio = 1,
};

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

static int convert_lux(struct device *dev, int ch0, int ch1, int gain,
		       struct lux_coefficients *coeffs)
{
	int lux1 = (coeffs->ch0_coeff0 * ch0 - coeffs->ch1_coeff0 * ch1) / gain;
	int lux2 = (coeffs->ch0_coeff1 * ch0 - coeffs->ch1_coeff1 * ch1) / gain;

	dev_dbg(dev, "lux1: %d, lux2: %d\n", lux1, lux2);

	if ((lux1 < 0) && (lux2 < 0))
		return -ERANGE;

	return max(lux1, lux2) / coeffs->gain_ratio;
}

void *tsl258x_als_platform_data(void *info)
{
	static struct tsl258x_platform_data tsl258x_platform_data;

	tsl258x_platform_data.als_def_odr = TSL258X_ALS_DEF_ODR;
	tsl258x_platform_data.als_def_als_time = TSL258X_ALS_DEF_TIME;
	tsl258x_platform_data.als_def_gain = TSL258X_ALS_DEF_GAIN;
	tsl258x_platform_data.als_def_gain_trim = TSL258X_ALS_DEF_GAIN_TRIM;
	tsl258x_platform_data.als_def_cal_target = TSL258X_ALS_DEF_CAL_TARGET;

	if (INTEL_MID_BOARD(2, PHONE, MRFL, MVN, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, MVN, ENG)) {
		tsl258x_platform_data.lux_coefficients = &mvn_coeffs;
		tsl258x_platform_data.convert_lux = convert_lux;
	} else if (INTEL_MID_BOARD(2, PHONE, MRFL, SPL, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, SPL, ENG)) {
		tsl258x_platform_data.lux_coefficients = &spl_coeffs;
		tsl258x_platform_data.convert_lux = convert_lux;
	} else {
		tsl258x_platform_data.lux_coefficients = NULL;
		tsl258x_platform_data.convert_lux = NULL;
	}

	if (INTEL_MID_BOARD(3, PHONE, MRFL, GLC, ENG, 4) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, GLC, PRO, 4) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, MVN, ENG, 4) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, MVN, PRO, 4))
		tsl258x_platform_data.gpio_conf = gpio_conf;
	else
		tsl258x_platform_data.gpio_conf = NULL;

	return &tsl258x_platform_data;
}

static const struct devs_id tsl258x_dev_id __initconst = {
	.name = "tsl2584",
	.type = SFI_DEV_TYPE_I2C,
	.delay = 0,
	.get_platform_data = &tsl258x_als_platform_data,
	.device_handler = NULL,
};

sfi_device(tsl258x_dev_id);
