/*
 * bq25898_charger.h: platform data structure for bq25898 driver
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __BQ25898_CHARGER_H__
#define __BQ25898_CHARGER_H__

/* this structure holds the restore register configuration for bq25898 */
struct restore_reg_config {
	u8 reg00;
	u8 reg04;
	u8 reg05;
	u8 reg06;
};

struct bq25898_plat_data {
	u32 irq_map;
	u8 irq_mask;
	struct power_supply_config *psy_cfg;
	struct power_supply_throttle *throttle_states;
	size_t num_throttle_states;
	int safety_timer;
	int boost_mode_ma;
	bool is_ts_enabled;
	int max_cc;
	int max_cv;
	bool is_wdt_kick_needed;
	int gpio_charger_int_n;
	int is_pmic_notifier;
	bool enable_postcharge;
	struct restore_reg_config reg_config;

	int (*enable_charging) (bool val);
	int (*enable_charger) (bool val);
	int (*set_inlmt) (int val);
	int (*set_cc) (int val);
	int (*set_cv) (int val);
	int (*set_iterm) (int val);
	int (*enable_vbus) (bool val);
	int (*handle_otgmode) (bool val);
	int (*handle_low_supply) (void);
	void (*dump_master_regs) (void);
};

#endif
