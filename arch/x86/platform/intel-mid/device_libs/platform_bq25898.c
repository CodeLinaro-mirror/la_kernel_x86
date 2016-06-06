/*
 * platform_bq25898.c: Platform data for bq25898 charger driver
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/power_supply.h>
#include <asm/pmic_pdata.h>
#include <asm/intel-mid.h>

#include "platform_ipc.h"
#include "platform_bq25898.h"

#define BOOST_CUR_LIM	500

/*
 * Extract of the documentation:
 * Power supply charging driver can take actions for Thermal throttling requests.
 * Power supply core sends event PSY_EVENT_THROTTLE  on power_supply_notifier
 * chain. Power supply charging driver handle this event and takes throttling
 * actions as in the throttling configuration structure.
 */
static struct power_supply_throttle bq25898_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = bq25898_CHRG_CUR_NOLIMIT,

	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = bq25898_CHRG_CUR_MEDIUM,

	},
	{
		.throttle_action = PSY_THROTTLE_DISABLE_CHARGING,
	},
	{
		.throttle_action = PSY_THROTTLE_DISABLE_CHARGER,
	},

};

static char *bq25898_supplied_to[] = {
				"max17047_battery",
};


void __initdata *bq25898_platform_data(void *info)
{
	static struct bq25898_plat_data bq25898_pdata;

	bq25898_pdata.irq_map = PMIC_SRAM_INTR_MAP;
	bq25898_pdata.irq_mask = PMIC_EXT_INTR_MASK;
	bq25898_pdata.supplied_to = bq25898_supplied_to;
	bq25898_pdata.num_supplicants = ARRAY_SIZE(bq25898_supplied_to);
	bq25898_pdata.throttle_states = bq25898_throttle_states;
	bq25898_pdata.num_throttle_states = ARRAY_SIZE(bq25898_throttle_states);
	bq25898_pdata.enable_charger = NULL;
	bq25898_pdata.set_iterm = NULL;
	bq25898_pdata.boost_mode_ma = BOOST_CUR_LIM;
	if (INTEL_MID_BOARD(2, PHONE, MRFL, ATC, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, ATC, ENG) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, SHA, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, SHA, ENG)) {
		bq25898_pdata.gpio_charger_int_n = get_gpio_by_name("charger_stat_n");
		bq25898_pdata.is_pmic_notifier = 0;
	} else {
		bq25898_pdata.is_pmic_notifier = 1;
	}

	return &bq25898_pdata;
}
