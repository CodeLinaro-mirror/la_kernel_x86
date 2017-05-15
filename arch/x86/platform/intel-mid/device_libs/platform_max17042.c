/*
 * platform_max17042.c: max17042 platform data initialization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/export.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/max17042_battery_ext.h>
#include <linux/power/intel_mdf_battery.h>
#include <linux/power/battery_id.h>
#include <asm/pmic_pdata.h>
#include <asm/intel-mid.h>
#include <asm/delay.h>
#include <asm/intel_scu_ipc.h>
#include "platform_max17042.h"

#define MRFL_SMIP_SRAM_ADDR		0xFFFCE000
#define MOFD_SMIP_SRAM_ADDR		0xFFFC5C00
#define MRFL_PLATFORM_CONFIG_OFFSET	0x3B3
#define MRFL_SMIP_SHUTDOWN_OFFSET	1
#define MRFL_SMIP_RESV_CAP_OFFSET	3

#define MRFL_VOLT_SHUTDOWN_MASK (1 << 1)
#define MRFL_NFC_RESV_MASK	(1 << 3)

#define BYT_FFRD8_TEMP_MIN_LIM	0	/* 0degC */
#define BYT_FFRD8_TEMP_MAX_LIM	55	/* 55degC */
#define BYT_FFRD8_BATT_MIN_VOLT	3400	/* 3400mV */
#define BYT_FFRD8_BATT_MAX_VOLT	4350	/* 4350mV */

#define BYT_CRV2_TEMP_MIN_LIM	0	/* 0degC */
#define BYT_CRV2_TEMP_MAX_LIM	45	/* 45degC */
#define BYT_CRV2_BATT_MIN_VOLT	3400	/* 3400mV */
#define BYT_CRV2_BATT_MAX_VOLT	4350	/* 4350mV */

#define TEMP_MADE_UP_20C		20

void max17042_i2c_reset_workaround(void)
{
/* toggle clock pin of I2C to recover devices from abnormal status.
 * currently, only max17042 on I2C needs such workaround
 */
#if defined(CONFIG_BATTERY_INTEL_MDF)
#define I2C_GPIO_PIN 27
#elif defined(CONFIG_BOARD_CTP)
#define I2C_GPIO_PIN 29
#elif defined(CONFIG_X86_MRFLD)
#define I2C_GPIO_PIN 21
#else
#define I2C_GPIO_PIN 27
#endif
#define I2C0_GPIO_PIN_BYT_CR_V2 79

	int i2c_gpio_pin = I2C_GPIO_PIN;

	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, CRV2) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, CRV2))
		i2c_gpio_pin = I2C0_GPIO_PIN_BYT_CR_V2;
	lnw_gpio_set_alt(i2c_gpio_pin, LNW_GPIO);
	gpio_direction_output(i2c_gpio_pin, 0);
	gpio_set_value(i2c_gpio_pin, 1);
	udelay(10);
	gpio_set_value(i2c_gpio_pin, 0);
	udelay(10);
	lnw_gpio_set_alt(i2c_gpio_pin, LNW_ALT_1);
}
EXPORT_SYMBOL(max17042_i2c_reset_workaround);

#define UMIP_REF_FG_TBL			0x806	/* 2 bytes */
#define BATT_FG_TBL_BODY		14	/* 144 bytes */

static int made_up_20C_get_temp(int *temp)
{
	*temp = TEMP_MADE_UP_20C;

	return 0;
}

static int mrfl_get_bat_health(void)
{

	int pbat_health = -ENODEV;
	int bqbat_health = -ENODEV;
#ifdef CONFIG_PMIC_CCSM
	pbat_health = pmic_get_health();
#endif

	/*Battery temperature exceptions are reported to PMIC. ALl other
	* exceptions are reported to bq24261 charger. Need to read the
	* battery health reported by both drivers, before reporting
	* the actual battery health
	*/

	/* FIXME: need to have a time stamp based implementation to
	* report battery health
	*/

	if (pbat_health < 0 && bqbat_health < 0)
		return pbat_health;
	if (pbat_health > 0 && pbat_health != POWER_SUPPLY_HEALTH_GOOD)
		return pbat_health;
	else
		return bqbat_health;
}

#define DEFAULT_VMIN	3400000		/* 3400mV */
static int mrfl_get_vsys_min(void)
{
	struct ps_batt_chg_prof batt_profile;
	int ret;

	ret = get_batt_prop(&batt_profile);
	if (!ret)
		return ((struct ps_pse_mod_prof *)batt_profile.batt_prof)
					->low_batt_mV * 1000;
	return DEFAULT_VMIN;
}
#define DEFAULT_VMAX_LIM	4200000		/* 4200mV */
static int mrfl_get_volt_max(void)
{
	struct ps_batt_chg_prof batt_profile;
	int ret;

	ret = get_batt_prop(&batt_profile);
	if (!ret)
		return ((struct ps_pse_mod_prof *)batt_profile.batt_prof)
					->voltage_max * 1000;
	return DEFAULT_VMAX_LIM;
}

static void init_tgain_toff(struct max17042_platform_data *pdata)
{
	if (INTEL_MID_BOARD(2, PHONE, MRFL, RBY, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, RBY, ENG) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, MVN, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, MVN, ENG) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, GLC, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, GLC, ENG) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, MRS, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, MRS, ENG) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, SPL, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, SPL, ENG) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, SHA, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, SHA, ENG) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, ATC, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, ATC, ENG)) {
		pdata->tgain = NTC_10K_MVN_TGAIN;
		pdata->toff = NTC_10K_MVN_TOFF;
	} else if (INTEL_MID_BOARD(1, PHONE, MRFL) ||
		INTEL_MID_BOARD(1, TABLET, MRFL) ||
		INTEL_MID_BOARD(1, PHONE, MOFD) ||
		INTEL_MID_BOARD(1, TABLET, MOFD)) {
		pdata->tgain = NTC_10K_MURATA_TGAIN;
		pdata->toff = NTC_10K_MURATA_TOFF;
	} else {
		pdata->tgain = NTC_47K_TGAIN;
		pdata->toff = NTC_47K_TOFF;
	}
}

static void init_callbacks(struct max17042_platform_data *pdata)
{
	if  (INTEL_MID_BOARD(3, PHONE, MRFL, RBY, PRO, 0) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, RBY, ENG, 0) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, RBY, PRO, 1) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, RBY, ENG, 1) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, RBY, PRO, 20) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, RBY, ENG, 20) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, MVN, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, MVN, ENG) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, GLC, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, GLC, ENG)) {
		pdata->get_vmin_threshold = mrfl_get_vsys_min;
		pdata->get_vmax_threshold = mrfl_get_volt_max;
		pdata->battery_status = NULL;
	} else if (INTEL_MID_BOARD(3, PHONE, MRFL, SHA, PRO, 0) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, SHA, ENG, 0) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, SHA, PRO, 1) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, SHA, ENG, 1)) {
		pdata->get_vmin_threshold = mrfl_get_vsys_min;
		pdata->get_vmax_threshold = mrfl_get_volt_max;
		pdata->battery_status = NULL;
		pdata->battery_pack_temp = made_up_20C_get_temp;
	} else if  (INTEL_MID_BOARD(2, PHONE, MRFL, RBY, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, RBY, ENG) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, ATC, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, ATC, ENG) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, SHA, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, SHA, ENG) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, MRS, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, MRS, ENG) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, SPL, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, SPL, ENG)) {
		pdata->get_vmin_threshold = mrfl_get_vsys_min;
		pdata->get_vmax_threshold = mrfl_get_volt_max;
		pdata->battery_status = NULL;
	} else if (INTEL_MID_BOARD(1, PHONE, MRFL)
			|| INTEL_MID_BOARD(1, TABLET, MRFL)
			|| INTEL_MID_BOARD(1, PHONE, MOFD)
			|| INTEL_MID_BOARD(1, TABLET, MOFD)) {
		/* MRFL Phones and tablets*/
		pdata->battery_health = mrfl_get_bat_health;
		pdata->battery_pack_temp = pmic_get_battery_pack_temp;
		pdata->get_vmin_threshold = mrfl_get_vsys_min;
		pdata->get_vmax_threshold = mrfl_get_volt_max;
	}
	pdata->reset_i2c_lines = max17042_i2c_reset_workaround;
}

static void init_platform_params(struct max17042_platform_data *pdata)
{
	pdata->fg_algo_model = 100;
	pdata->enable_current_sense = true;
	pdata->technology = POWER_SUPPLY_TECHNOLOGY_LION;
	pdata->file_sys_storage_enabled = 1;
	pdata->soc_intr_mode_enabled = true;
	pdata->valid_battery = true;
	pdata->is_init_done = 0;
}

static void init_platform_thresholds(struct max17042_platform_data *pdata)
{
	pdata->resv_cap = 0;
	if (INTEL_MID_BOARD(2, PHONE, MRFL, RBY, PRO) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, RBY, ENG)) {
		pdata->temp_min_lim = 0;
		pdata->temp_max_lim = 45;
		pdata->volt_min_lim = 3200;
		pdata->volt_max_lim = 4350;
	} else if (INTEL_MID_BOARD(2, PHONE, MRFL, MVN, PRO) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, MVN, ENG) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, GLC, PRO) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, GLC, ENG) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, ATC, PRO) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, ATC, ENG) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, SHA, ENG) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, SHA, PRO) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, MRS, PRO) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, MRS, ENG)) {
		pdata->temp_min_lim = -20;
		pdata->temp_max_lim = 60;
		pdata->volt_min_lim = 3200;
		pdata->volt_max_lim = 4350;
	} else if (INTEL_MID_BOARD(2, PHONE, MRFL, SPL, PRO) ||
				INTEL_MID_BOARD(2, PHONE, MRFL, SPL, ENG)) {
		pdata->temp_min_lim = -20;
		pdata->temp_max_lim = 60;
		pdata->volt_min_lim = 3600;
		pdata->volt_max_lim = 4350;
	}
}

void *max17042_get_platform_data(void *info)
{
	static struct max17042_platform_data platform_data;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;
	int intr = get_gpio_by_name("max_fg_alert");

	if (i2c_info)
		i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;

	init_tgain_toff(&platform_data);
	init_callbacks(&platform_data);
	init_platform_params(&platform_data);
	init_platform_thresholds(&platform_data);
	return &platform_data;
}

static const struct devs_id max17042_dev_id __initconst = {
	.name = "max17042",
	.type = SFI_DEV_TYPE_I2C,
	.delay = 1,
	.get_platform_data = &max17042_get_platform_data,
};
static const struct devs_id max17047_dev_id __initconst = {
	.name = "max17047",
	.type = SFI_DEV_TYPE_I2C,
	.delay = 1,
	.get_platform_data = &max17042_get_platform_data,
};
static const struct devs_id max17050_dev_id __initconst = {
	.name = "max17050",
	.type = SFI_DEV_TYPE_I2C,
	.delay = 1,
	.get_platform_data = &max17042_get_platform_data,
};

sfi_device(max17042_dev_id);
sfi_device(max17047_dev_id);
sfi_device(max17050_dev_id);
