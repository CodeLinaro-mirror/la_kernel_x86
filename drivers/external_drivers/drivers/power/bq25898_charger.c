/*
 * bq25898_charger.c - BQ25898 Charger I2C client driver
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICLAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */
/*
 *  To enable dev_dbg, add this to the cmdline:
 *	dyndbg = "module bq25898 +p"
 *  or at compile time:
 *	#define DEBUG
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/power/bq25898_charger.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>

#include <asm/intel_scu_ipc.h>
#include <asm/pmic_pdata.h>

/* for test purpose only */
#define DEBUG_DUMP_REG			0

#define NR_RETRY_CNT			3

#define BUFSIZ_COPY_FROM_USER		20

#define DEV_NAME			"bq25898_charger"
#define DEV_MANUFACTURER		"TI"
#define MODEL_NAME			"BQ25898"
#define MODEL_NAME_SIZE			8
#define DEV_MANUFACTURER_NAME_SIZE	4

#define BQ25898_POSTCHARGE_DEFAULT_DURATION_MN	30
#define BQ25898_WDT_RESET_DELAY			(5 * HZ)
#define BQ25898_BAT_MONITOR_DELAY		(60 * HZ)

#define BQ25898_I2C_SLAVE_ADDR			0x6B

/*
 * Input source control register (REG00)
 */
#define BQ25898_INPUT_SRC_CTRL_REG		0x00

/* Set Bit7 to enable HIZ , default unset (HIZ disabled) */
#define INPUT_SRC_CNTL_ENABLE_HIZ		(1 << 7)

/*
 * Bits 0 to 5 to set the IINLIM (Input Current Limit)
 * IINLIM
 * Default: b011100 (1500mA), offset of 100mA
 */
#define INPUT_SRC_CUR_IINLIM5		(1 << 5)	/* 1600mA */
#define INPUT_SRC_CUR_IINLIM4		(1 << 4)	/* 800mA */
#define INPUT_SRC_CUR_IINLIM3		(1 << 3)	/* 400mA */
#define INPUT_SRC_CUR_IINLIM2		(1 << 2)	/* 200mA */
#define INPUT_SRC_CUR_IINLIM1		(1 << 1)	/* 100mA */
#define INPUT_SRC_CUR_IINLIM0		(1 << 0)	/* 50mA */

/*
 * Input voltage control register (REG01)
 */
#define BQ25898_INPUT_VOLTAGE_CTRL_REG		0x01
/* EN_12V -- 0: disable 12V for maxcharge, 1: enable 12V for maxcharge */
#define INPUT_SRC_VOLTAGE_EN_12V		(1 << 1)
/* VINDPM_OS --  0: 400mA offset, 1: 600mA offset */
#define INPUT_SRC_VOLTAGE_VINDPM0		(1 << 0)

/*
 * ADC converter control register (REG02)
 */
#define BQ25898_ADC_CTRL_REG			0x02
/* CONV_START -- 0: ADC not active, 1: Start ADC conversion */
#define ADC_CONV_START				(1 << 7)
/* CONV_RATE --  0: One short ADC conversion, 1: Start 1s Continuous Conversion */
#define ADC_CONV_RATE				(1 << 6)
/* BOOST_FREQ -- 0: 1.5Mhz 1: 500kHz*/
#define BOOST_FREQ				(1 << 5)
/* ICD_EN -- 0: ICD Disabled, 1: ICD Enabled */
#define ICD_EN					(1 << 4)
/* HVDCP_EN -- 0: HVDCP_EN Disabled, 1: HVDCP_EN Enabled */
#define HVDCP_EN				(1 << 3)
/* MAXC_EN -- 0: MAXC_EN Disabled, 1: MAXC_EN Enabled */
#define MAXC_EN					(1 << 2)
/* FORCE_DPM -- 0: Not in PSEL detection, 1: Force PSEL detection */
#define ADC_FORCE_DPDM				(1 << 1)
/* AUTO_DPM_EN -- 0: Disable PSEL detection, 1: Enable PSEL detection */
#define ADC_AUTO_DPDM_ENABLE			(1 << 0)

/*
 * Charge/Timer control register ( REG03)
 */
#define BQ25898_CHARGE_CTRL_REG			0x03
/* VOK_OTG_EN -- 0: VOK_OTG_EN disabled, 1: VOK_OTG_EN enabled) */
#define VOK_OTG_EN				(1 << 7)
/* WD_RST -- 0: Normal, 1: reset (back to 0 after timer reset) */
#define WDT_TIMER_RESET				(1 << 6)
/* OTG_CONFIG -- 0: OTG_CONFIG disabled,1: OTG_CONFIG enabled */
#define OTG_CONFIG				(1 << 5)
/* CHG_CONFIG -- 0: Charge disable,1: Charge enable */
#define CHG_CONFIG				(1 << 4)
/* SYS_MIN
 * Default: b101, offset 3.0V
 */
#define SYS_MIN2				(1 << 3)	/* 0.4v */
#define SYS_MIN1				(1 << 2)	/* 0.2v */
#define SYS_MIN0				(1 << 1)	/* 0.1v */

/*
 *  Fast Charge control register ( REG04)
 */
#define BQ25898_FAST_CHARGE_CTRL_REG			0x04
/* EN_PUMPX -- 0: Current pulse control disabled, 1: Current pulse control enabled */
#define EN_PUMPX					(1 << 7)
/* ICHG  Fast Charge current limit
 * Default b000000, 0mv offset
 */
#define FAST_CHARGE_ICHG6			(1 << 6) /* 4096mA */
#define FAST_CHARGE_ICHG5			(1 << 5) /* 2048mA */
#define FAST_CHARGE_ICHG4			(1 << 4) /* 1024mA */
#define FAST_CHARGE_ICHG3			(1 << 3) /* 512mA */
#define FAST_CHARGE_ICHG2			(1 << 2) /* 256mA */
#define FAST_CHARGE_ICHG1			(1 << 1) /* 128mA */
#define FAST_CHARGE_ICHG0			(1 << 0) /* 64mA */

/*
 * Pre charge and Termination current limit (REG05)
 */
#define BQ25898_CUR_LIMIT_CTRL_REG		0x05
/*
 * IPRECHG Pre charge current limit
 * Default: b0001, 64mA offset
 */
#define PRECHARGE_CUR_LIMIT3			(1 << 7)	/* 512mA */
#define PRECHARGE_CUR_LIMIT2			(1 << 6)	/* 256mA */
#define PRECHARGE_CUR_LIMIT1			(1 << 5)	/* 128Ma */
#define PRECHARGE_CUR_LIMIT0			(1 << 4)	/* 64mA  */
/*
 * ITERM  Termination current limit
 * Default: b0011, offset 64mA
 */
#define TERM_CUR_LIMIT3				(1 << 3)	/* 512mA */
#define TERM_CUR_LIMIT2				(1 << 2)	/* 256mA */
#define TERM_CUR_LIMIT1				(1 << 1)	/* 128mA */
#define TERM_CUR_LIMIT0				(1 << 0)	/* 64 mA */

/*
 * Charge voltage limit register
 * Bat precharge/recharge threshold reg (REG06)
 */
#define BQ25898_VLIM_CHRG_CTRL_REG		0x06
/*
 * VREG Charge Voltage Limit
 * Default: b10000, 3.840V offset
 */
#define CHARGE_VOLT_LIMIT5			(1 << 7)	/* 512mV */
#define CHARGE_VOLT_LIMIT4			(1 << 6)	/* 256mV */
#define CHARGE_VOLT_LIMIT3			(1 << 5)	/* 128mV */
#define CHARGE_VOLT_LIMIT2			(1 << 4)	/* 64mV  */
#define CHARGE_VOLT_LIMIT1			(1 << 3)	/* 32mV  */
#define CHARGE_VOLT_LIMIT0			(1 << 2)	/* 16mV  */

/* Battery Precharge to Fast Charge Threshold -- 0: 2.8V, 1: 3.0V, default 3.0V */
#define BAT_PRE2FAST_CHARGE_THRESHOLD		(1 << 1)

/* Battery Recharge Threshold -- 0: 100mV, 1: 200mV, default: 100mV */
#define BAT_RECHARGE_THRESHOLD			(1 << 0)

/*
 * Charge termination, termination indicator
 * I2C Watchdog Timer register
 * Safety Timer  (REG07)
 */
#define BQ25898_TERM_WDT_SFTY_CTRL_REG		0x07
/* */
#define CHARGE_TERM_ENABLE			(1 << 7)	/* 0: disable, 1: enable */
#define TERM_INDICATOR_THRESHOLD		(1 << 6)	/* 0: enable STAT pin, 1: disable STAT pin */
/* WATCHDOG */
#define I2C_WDT_TIMER_SETTINGS1			(1 << 5)	/* b00 Disable Timer, b01:40s, b10:80s*/
#define I2C_WDT_TIMER_SETTINGS0			(1 << 4)	/* b11: 160s, default b01 */
/* EN_TIMER Enable safety timer*/
#define CHARGING_SAFETY_TIMER_ENABLE		(1 << 3)	/* 0: disable, 1: enable */
/* CHG_TIMER Fast charge safety timer*/
#define CHARGING_SAFETY_TIMER_ENABLE1		(1 << 2)	/* b00: 5hrs, b01:8Hrs, b10: 12Hrs */
#define CHARGING_SAFETY_TIMER_ENABLE0		(1 << 1)	/* b11: 20Hrs, default b10 */
/* JEITA_ISET low temp setting */
#define JEITA_ISET				(1 << 0)	/* 0: 50%, 1: 20% */

/*
 * Thermal regulation threshold register (REG08)
 */
#define BQ25898_THERM_REGULATION_CTRL_REG	0x08
/* BAT_COMP */
#define BAT_COMP2				(1 << 7)	/* 80m Ohm */
#define BAT_COMP1				(1 << 6)	/* 40m Ohm */
#define BAT_COMP0				(1 << 5)	/* 20m Ohm */
/* VCLAMP */
#define VCLAMP2					(1 << 4)	/* 128mV */
#define VCLAMP1					(1 << 3)	/* 64mV */
#define VCLAMP0					(1 << 2)	/* 32mV */
/* TREG */
#define THERMAL_REGUL_THRESOLD1			(1 << 1)	/* b00: 60C, b01:80C, b10: 100C */
#define THERMAL_REGUL_THRESOLD0			(1 << 0)	/* b11: 120C, default b11 */

/*
 *  safety timer control register (REG09)
 */
#define BQ25898_SAFETY_TIMER_CTRL_REG		0x09
/* FORCE_ICD -- 0: do not force input detection , 1: force input detection algo*/
#define FORCE_ICD				(1 << 7)
/* TMR2X_EN -- 0: Safety timer not slowed by 2X, 1: slowed by 2X*/
#define SAFETY_TIMER_SLOW_ENABLE		(1 << 6)
/* BATFET_Disable -- 0: Allow Q4 turn on, 1: turn off Q4*/
#define BATFET_DISABLE				(1 << 5)
/* JEITA_VSET -- 0: set charge voltage to VREG-200mv, 1: set charge voltage to VREG*/
#define JEITA_VSET				(1 << 4)
/* BATFET_DLY -- 0: Turn off BATFET immediatly when BATFET_DISABLE is set,
 1: Turn off BATFET after Tbatfet_dly (10s) when BATFET_DISABLE is set */
#define BATFET_DLY				(1 << 3)

/* BATFET_RST_EN -- 0: disable BATFET reset function, 1: enable BATFET reset function */
#define BATFET_RST_EN				(1 << 2)
/* PUMPX_UP */
#define PUMPX_UP				(1 << 1)	/* 0: , 1: */
/* PUMPX_DN */
#define PUMPX_DN				(1 << 0)	/* 0: , 1: */

/*
 * Boost register (REG0A)
 */
#define BQ25898_BOOST_CTRL_REG			0x0A
/* BOOSTV offset: 4.55V , default b0111 4.998V*/
#define BOOSTV3					(1 << 7)	/* 512mV */
#define BOOSTV2					(1 << 6)	/* 256mV */
#define BOOSTV1					(1 << 5)	/* 128mV */
#define BOOSTV0					(1 << 4)	/* 64mV	 */
/* PMF_OTG_DIS */
#define PMF_OTG_DIS				(1 << 3)	/* 0: enable , 1: disable */
/* BOOST_LIM */
#define BOOST_LIM2				(1 << 2)	/* b000: 0.5A b001: 0.8A b010: 1.0A   */
#define BOOST_LIM1				(1 << 1)	/* b011: 1.2A b100: 1.5A b101 1.8A  */
#define BOOST_LIM0				(1 << 0)	/* b110: 2.1A b111: 2.4A  */

/*
 *  Status register (REG0B)
 */
#define BQ25898_STATUS_REG			0x0B
/* VBUS_STAT */
#define VBUS_STATUS2				(1 << 7)	/* b000: No input, b001: USB Host SDP */
#define VBUS_STATUS1				(1 << 6)	/* b010: Adapter (3.25A), b111: NA */
#define VBUS_STATUS0				(1 << 5)
/* CHRG_STAT */
#define CHARGER_STATUS1				(1 << 4)	/* b00: Not charging, b01: Precharge */
#define CHARGER_STATUS0				(1 << 3)	/* b10: Fast charge, b11: Charge termination done */
/* PG_STAT -- 0: Not power good, 1: Power good */
#define PG_STAT					(1 << 2)
/* VSYS_STAT -- 0: Not in VSYSmin, 1: in Vsys min regulation (BAT < VSYSMIN) */
#define VSYS_STAT				(1 << 0)

/*
 *  Fault Register (REG0C)
 */
#define BQ25898_FAULT_REG			0x0C
/* WATCHDOG_FAULT */
#define WATCHDOG_FAULT				(1 << 7)	/* 0: normal, 1: WDT timer expiration */
/* CHRG_FAULT */
#define CHARGER_FAULT1				(1 << 5)	/* b00: normal, b01: input fault */
#define CHARGER_FAULT0				(1 << 4)	/* b10: thermal shutdown, b11: charge safety timer expiration */
/* BAT_FAULT */
#define BAT_FAULT				(1 << 3)	/* 0: normal, 1: BATOVP */
/* NTC_FAULT */
#define NTC_FAULT2				(1 << 2)	/* Cold/Hot : b000: normal b001: TS Cold b010:TS Hot */
#define NTC_FAULT1				(1 << 1)	/* JEITA: b000:normal  b010:warm b011:cool b101: cold*/
#define NTC_FAULT0				(1 << 0)	/* b110: Hot */

/*
 * Voltage IN control register (REG0D),
 * all bits will be reset when adaptor plug in
 * default: b00010010
 */
#define BQ25898_VIN_CTRL_REG			0x0D
/* FORCE_VINDPM -- 0: run VINDPM threshold algo, 1: force VINDMP; bypass algo */
#define FORCE_VINDPM				(1 << 7)
/* VINDPM
 * Absolute VINDPM threshold offset: 2.6V, default b0010010; 4.4V
 */
#define VINDPM6					(1 << 6)	/* 6400mV */
#define VINDPM5					(1 << 5)	/* 3200mV */
#define VINDPM4					(1 << 4)	/* 1600mV */
#define VINDPM3					(1 << 3)	/* 800mV */
#define VINDPM2					(1 << 2)	/* 400mV */
#define VINDPM1					(1 << 1)	/* 200mV */
#define VINDPM0					(1 << 0)	/* 100mV */

/*
 * Battery voltage register	(REG0E), readonly
 */
#define BQ25898_BAT_VOLT_REG			0x0E
/* THERM_STAT -- 0: normal, 1:in thermal regulation */
#define THERM_STAT				(1 << 7)
/* BATV - Adc conversion Bat voltage
 * Offset 2.304V, default b000000*/
#define ADC_CONV_BATV6				(1 << 6)	/* 1280mV */
#define ADC_CONV_BATV5				(1 << 5)	/* 640mV */
#define ADC_CONV_BATV4				(1 << 4)	/* 320mV */
#define ADC_CONV_BATV3				(1 << 3)	/* 160mV */
#define ADC_CONV_BATV2				(1 << 2)	/* 80mV */
#define ADC_CONV_BATV1				(1 << 1)	/* 40mV */
#define ADC_CONV_BATV0				(1 << 0)	/* 20mV */

/*
 * System voltage register	(REG0F), readonly
 */
#define BQ25898_SYS_VOLTAGE_REG			0x0F
/* SYSV
 * ADDC Conversion of system Voltage
 * Default: b0000000, offsert 2.304V
 */
#define ADDC_CONV_SYSV6				(1 << 6)	/* 1280mV */
#define ADDC_CONV_SYSV5				(1 << 5)	/* 640mV */
#define ADDC_CONV_SYSV4				(1 << 4)	/* 320mV */
#define ADDC_CONV_SYSV3				(1 << 3)	/* 160mV */
#define ADDC_CONV_SYSV2				(1 << 2)	/* 80mV */
#define ADDC_CONV_SYSV1				(1 << 1)	/* 40mV */
#define ADDC_CONV_SYSV0				(1 << 0)	/* 20mV */

/*
 * ADC Conversion of TS Voltage (REG10), readonly
 * TSPCPT
 */
#define BQ25898_ADC_TSPCPT_REG			0x10
/* TSPCPT  Adc Conversion of TS voltage
 * offset 21%, default b0000000
 */
#define TSPCPT6					(1 << 6)	/* 29.76% */
#define TSPCPT5					(1 << 5)	/* 14.88% */
#define TSPCPT4					(1 << 4)	/* 7.44% */
#define TSPCPT3					(1 << 3)	/* 3.72% */
#define TSPCPT2					(1 << 2)	/* 1.86% */
#define TSPCPT1					(1 << 1)	/* 0.93% */
#define TSPCPT0					(1 << 0)	/* 0.465% */

/*
 * VBUS ADC conversion register (REG11), readonly
 */
#define BQ25898_VBUS_ADC_REG			0x11
/* VBUS_GD */
#define VBUS_ATTACHED				(1 << 7)	/* 0: No VBUS attached, 1: VBUS attached */
/* VBUSV  Adc Conversion of VBUS voltage
 * Default b0000000, Offset 2.6V
 */
#define VBUSV_ADC_VOLT6				(1 << 6)	/* 6400mV */
#define VBUSV_ADC_VOLT5				(1 << 5)	/* 3200mV */
#define VBUSV_ADC_VOLT4				(1 << 4)	/* 1600mV */
#define VBUSV_ADC_VOLT3				(1 << 3)	/* 800mV */
#define VBUSV_ADC_VOLT2				(1 << 2)	/* 400mV */
#define VBUSV_ADC_VOLT1				(1 << 1)	/* 200mV */
#define VBUSV_ADC_VOLT0				(1 << 0)	/* 100mV */

/*
 *  ADC Charge Current conversion register (REG12), readonly
 */
#define BQ25898_CC_ADC_REG			0x12
/* ICHGR  Adc Conversion of charge current
 * Default b0000000, Offset 0mA
 */
#define ADC_CHARGE_CURRENT6			(1 << 6)	/* 3200mA */
#define ADC_CHARGE_CURRENT5			(1 << 5)	/* 1600mA */
#define ADC_CHARGE_CURRENT4			(1 << 4)	/* 800mA */
#define ADC_CHARGE_CURRENT3			(1 << 3)	/* 400mA */
#define ADC_CHARGE_CURRENT2			(1 << 2)	/* 200mA */
#define ADC_CHARGE_CURRENT1			(1 << 1)	/* 100mA */
#define ADC_CHARGE_CURRENT0			(1 << 0)	/* 50mA */

/*
 *  VINDPM Current Limit register (REG13), readonly
 */
#define BQ25898_VINDPM_LIM_REG			0x13
/* VDPM_STAT */
#define VDPM_STAT				(1 << 7)	/* 0: not in VINDPM, 1: VINDPM */
/* IDPM_STAT */
#define IDPM_STAT				(1 << 6)	/* 0: not in IINDPM, 1: IINDPM */
/* IDMP_LIM  Input current DPM limit
 * Default b0000000, Offset 100mA
 */
#define IDPM_CURRENT_LIM5			(1 << 5)	/* 1600mA */
#define IDPM_CURRENT_LIM4			(1 << 4)	/* 800mA */
#define IDPM_CURRENT_LIM3			(1 << 3)	/* 400mA */
#define IDPM_CURRENT_LIM2			(1 << 2)	/* 200mA */
#define IDPM_CURRENT_LIM1			(1 << 1)	/* 100mA */
#define IDPM_CURRENT_LIM0			(1 << 0)	/* 50mA */

/*
 * Device Register control register (REG14)
 */
#define BQ25898_DEVREG_CTRL_REG			0x14
/* REG_RST Register reset -- 0: keep current register setting, 1: reset to default */
#define REG_RST					(1 << 7)
/* ICD_OPTIMIZED -- 0: detection in progress or disabled, 1: Maximum input current detected */
#define ICD_OPTIMIZED				(1 << 6)
/* PN Device configuration, default:b001 */
#define PN2					(1 << 5)
#define PN1					(1 << 4)
#define PN0					(1 << 3)
/* DEV_REV default:b00 */
#define DEV_REV1				(1 << 1)
#define DEV_REV0				(1 << 0)

#define BQ25898_FIRST_REGISTER			0x00
#define BQ25898_LAST_REGISTER			0x14

#define BQ25898_ALL_REGISTER			0x15

#define BQ25898_INT_COUNTER "bq25898_irq_counter"

static int bq25898_charger_configure(struct i2c_client *client);

enum bq25898_wdt_timing {
	BQ25898_WDT_TIMER_DISABLE,
	BQ25898_WDT_TIMER_40S,
	BQ25898_WDT_TIMER_80S,
	BQ25898_WDT_TIMER_160S
};

struct bq25898_debugfs_file {
	struct bq25898_charger *parent;
	u8 regnr;
};

struct bq25898_charger {
	struct mutex stat_lock;
	struct mutex sysfs_lock;
	struct i2c_client *client;
	struct bq25898_plat_data *pdata;
	struct power_supply psy_usb;
	struct delayed_work sw_term_work;
	struct delayed_work wdt_work;
	struct delayed_work batmon_work;
	struct work_struct sw_config_work;
	struct work_struct charge_status_work;
	struct notifier_block otg_usb_change;
	struct notifier_block pmic_notifier;
	struct usb_phy *transceiver;
	struct dentry *dbgfs_dir;
	struct bq25898_debugfs_file dbgfs_file[BQ25898_ALL_REGISTER+1];
	int revision;
	char model_name[MODEL_NAME_SIZE];
	char manufacturer[DEV_MANUFACTURER_NAME_SIZE];
	int current_now;		/* provided by healthd */
	int temperature;		/* tbd whom provide this */
	int postcharge_duration;	/* duration in mn after charge termination interrupt */
	u32 irq_counter;
};

static enum power_supply_property bq25898_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,		/* charging status */
	POWER_SUPPLY_PROP_HEALTH,		/* battery health */
	POWER_SUPPLY_PROP_PRESENT,		/* battery present */
	POWER_SUPPLY_PROP_TEMP,			/* battery temperature */
	POWER_SUPPLY_PROP_TECHNOLOGY,		/* battery technology */
	POWER_SUPPLY_PROP_CURRENT_NOW		/* battery current */
};

static int bq25898_usb_change_notifier(struct notifier_block *self, unsigned long action,
				void *param)
{
	struct power_supply_cable_props *caps = param;
	struct bq25898_charger *chip = container_of(self, struct bq25898_charger,
						otg_usb_change);

	if (!chip || !caps)
		return NOTIFY_DONE;

	switch (action) {
	case USB_EVENT_CHARGER:
		switch (caps->chrg_evt) {
		case POWER_SUPPLY_CHARGER_EVENT_CONNECT:
			/* schedule battery monitoring */
			schedule_delayed_work(&chip->batmon_work, BQ25898_BAT_MONITOR_DELAY);
			break;
		case POWER_SUPPLY_CHARGER_EVENT_DISCONNECT:
			/* cancel battery monitor */
			cancel_delayed_work(&chip->batmon_work);

			/* ensure charge termination is enabled */
			queue_work(system_nrt_wq, &chip->sw_config_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}


static int bq25898_read_reg(struct i2c_client *client, u8 reg)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			dev_dbg(&client->dev, "pm_runtime_get_sync failed:%d",ret);
			continue;
		}
		ret = i2c_smbus_read_byte_data(client, reg);
		pm_runtime_put_sync(&client->dev);
		if (ret < 0)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "error in reading reg %d:%d\n", reg, ret);

	return ret;
}

static int bq25898_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			dev_dbg(&client->dev, "pm_runtime_get_sync failed:%d\n",ret);
			continue;
		}
		ret = i2c_smbus_write_byte_data(client, reg, data);
		pm_runtime_put_sync(&client->dev);
		if (ret < 0)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "error in writing %d to reg %d:%d\n",
			data, reg, ret);

	return ret;
}

static int bq25898_read_modify_reg(struct i2c_client *client, u8 reg,
					  u8 mask, u8 val)
{
	int ret;

	ret = bq25898_read_reg(client, reg);
	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "bq read mod reg: 0x%02x\n", ret);

	ret = (ret & ~mask)|(mask & val);

	dev_dbg(&client->dev, "bq read mod new reg: 0x%02x\n", ret);

	return bq25898_write_reg(client, reg, ret);
}

#if DEBUG_DUMP_REG
static void bq25898_dump_regs(struct i2c_client *client)
{
	int i;
	int ret;
	struct bq25898_charger *chip;
	char buf[160] = {0};
	int used = 0;

	if (!client)
		return;

	chip = i2c_get_clientdata(client);

	dev_info(&client->dev, "BQ25898 Register dump: ");

	for (i = BQ25898_FIRST_REGISTER; i <= BQ25898_LAST_REGISTER; ++i) {
		ret = bq25898_read_reg(client, i);
		if (ret < 0)
			dev_err(&client->dev,
				"Error in reading REG %X:%d\n", i, ret);
		else
			used += snprintf(buf + used, sizeof(buf) - used,
					"%02X:0x%X ", i, ret);
	}
	dev_info(&client->dev, "%s\n", buf);

}
#endif

#ifdef CONFIG_DEBUG_FS
static void bq25898_reg0_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 0;

	if (val & INPUT_SRC_CNTL_ENABLE_HIZ)
		seq_puts(seq, "HIZ enabled\n");
	else
		seq_puts(seq, "HIZ disabled\n");

	res = 100;
	if (val & INPUT_SRC_CUR_IINLIM5)
		res += 1600;
	if (val & INPUT_SRC_CUR_IINLIM4)
		res += 800;
	if (val & INPUT_SRC_CUR_IINLIM3)
		res += 400;
	if (val & INPUT_SRC_CUR_IINLIM2)
		res += 200;
	if (val & INPUT_SRC_CUR_IINLIM1)
		res += 100;
	if (val & INPUT_SRC_CUR_IINLIM0)
		res += 50;

	seq_printf(seq, "IINLIM %dmA\n", res);
}

static void bq25898_reg1_to_human(struct seq_file *seq, u8 reg, u8 val)
{

	if (val & INPUT_SRC_VOLTAGE_EN_12V)
		seq_puts(seq, "12V enabled for maxcharge");
	else
		seq_puts(seq, "12V disabled for maxcharge\n");

	if (val & INPUT_SRC_VOLTAGE_VINDPM0)
		seq_puts(seq, "VINDPM 600mA offset\n");
	else
		seq_puts(seq, "VINDPM 400mA offset\n");

}

static void bq25898_reg2_to_human(struct seq_file *seq, u8 reg, u8 val)
{

	if (val & ADC_CONV_START)
		seq_puts(seq, "Start ADC conversion\n");
	else
		seq_puts(seq, "ADC not active\n");

	if (val & ADC_CONV_RATE)
		seq_puts(seq, "Start 1s Continuous conversion\n");
	else
		seq_puts(seq, "One short ADC conversion\n");

	if (val & BOOST_FREQ)
		seq_puts(seq, "BOOST_FREQ 500kHz\n");
	else
		seq_puts(seq, "BOOST_FREQ 1.5Mhz\n");

	if (val & ICD_EN)
		seq_puts(seq, "ICD_EN enabled\n");
	else
		seq_puts(seq, "ICD_EN disabled\n");

	if (val & HVDCP_EN)
		seq_puts(seq, "HVDCP_EN enabled\n");
	else
		seq_puts(seq, "HVDCP_EN disabled\n");

	if (val & MAXC_EN)
		seq_puts(seq, "MAXC_EN enabled\n");
	else
		seq_puts(seq, "MAXC_EN disabled\n");

	if (val & ADC_FORCE_DPDM)
		seq_puts(seq, "Force PSEL detection\n");
	else
		seq_puts(seq, "Not in PSEL detection\n");

	if (val & ADC_AUTO_DPDM_ENABLE)
		seq_puts(seq, "Enable PSEL detection\n");
	else
		seq_puts(seq, "Disable PSEL detection\n");
}

static void bq25898_reg3_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 0;

	if (val & VOK_OTG_EN)
		seq_puts(seq, "VOK_OTG_EN enabled\n");
	else
		seq_puts(seq, "VOK_OTG_EN disabled\n");

	if (val & WDT_TIMER_RESET)
		seq_puts(seq, "WDT_RST Reset\n");
	else
		seq_puts(seq, "WDT_RST Normal\n");

	if (val & OTG_CONFIG)
		seq_puts(seq, "OTG_CONFIG enabled\n");
	else
		seq_puts(seq, "OTG_CONFIG disabled\n");

	if (val & CHG_CONFIG)
		seq_puts(seq, "CHG_CONFIG enabled\n");
	else
		seq_puts(seq, "CHG_CONFIG disabled\n");

	res = 3000;
	if (val & SYS_MIN2)
		res += 400;
	if (val & SYS_MIN1)
		res += 200;
	if (val & SYS_MIN0)
		res += 100;

	seq_printf(seq, "SYS_MIN %dmV\n", res);
}

static void bq25898_reg4_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 0;

	if (val & EN_PUMPX)
		seq_puts(seq, "EN_PUMPX current pulse control enabled\n");
	else
		seq_puts(seq, "EN_PUMPX current pulse control disabled\n");

	if (val & FAST_CHARGE_ICHG6)
		res += 4096;
	if (val & FAST_CHARGE_ICHG5)
		res += 2048;
	if (val & FAST_CHARGE_ICHG4)
		res += 1024;
	if (val & FAST_CHARGE_ICHG3)
		res += 512;
	if (val & FAST_CHARGE_ICHG2)
		res += 256;
	if (val & FAST_CHARGE_ICHG1)
		res += 128;
	if (val & FAST_CHARGE_ICHG0)
		res += 64;

	seq_printf(seq, "ICHG Fast charge %dmA\n", res);
}

static void bq25898_reg5_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 64;

	if (val & PRECHARGE_CUR_LIMIT3)
		res += 512;
	if (val & PRECHARGE_CUR_LIMIT2)
		res += 256;
	if (val & PRECHARGE_CUR_LIMIT1)
		res += 128;
	if (val & PRECHARGE_CUR_LIMIT0)
		res += 64;

	seq_printf(seq, "IPRECHG %dmA\n", res);

	res = 64;
	if (val & TERM_CUR_LIMIT3)
		res += 512;
	if (val & TERM_CUR_LIMIT2)
		res += 256;
	if (val & TERM_CUR_LIMIT1)
		res += 128;
	if (val & TERM_CUR_LIMIT0)
		res += 64;

	seq_printf(seq, "ITERM %dmA\n", res);
}

static void bq25898_reg6_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 3840;

	if (val & CHARGE_VOLT_LIMIT5)
		res += 512;
	if (val & CHARGE_VOLT_LIMIT4)
		res += 256;
	if (val & CHARGE_VOLT_LIMIT3)
		res += 128;
	if (val & CHARGE_VOLT_LIMIT2)
		res += 64;
	if (val & CHARGE_VOLT_LIMIT1)
		res += 32;
	if (val & CHARGE_VOLT_LIMIT0)
		res += 16;

	seq_printf(seq, "Charge Voltage Limit %dmV\n", res);

	if (val & BAT_PRE2FAST_CHARGE_THRESHOLD)
		seq_puts(seq, "Precharge2Fast threshold 3.0V\n");
	else
		seq_puts(seq, "Precharge2Fast threshold 2.8V\n");

	if (val & BAT_RECHARGE_THRESHOLD)
		seq_puts(seq, "Recharge threshold 200mV\n");
	else
		seq_puts(seq, "Recharge threshold 100mV\n");
}

static void bq25898_reg7_to_human(struct seq_file *seq, u8 reg, u8 val)
{

	if (val & CHARGE_TERM_ENABLE)
		seq_puts(seq, "Charge Term enabled\n");
	else
		seq_puts(seq, "Charge Term disabled\n");

	if (!(val & I2C_WDT_TIMER_SETTINGS1) && !(val & I2C_WDT_TIMER_SETTINGS0))
		seq_puts(seq, "WDT disabled\n");
	else if (!(val & I2C_WDT_TIMER_SETTINGS1) && (val & I2C_WDT_TIMER_SETTINGS0))
		seq_puts(seq, "WDT timer 40s\n");
	else if ((val & I2C_WDT_TIMER_SETTINGS1) && !(val & I2C_WDT_TIMER_SETTINGS0))
		seq_puts(seq, "WDT timer 80s\n");
	else if ((val & I2C_WDT_TIMER_SETTINGS1) && (val & I2C_WDT_TIMER_SETTINGS0))
		seq_puts(seq, "WDT timer 160s\n");

	if (!(val & CHARGING_SAFETY_TIMER_ENABLE1) && !(val & CHARGING_SAFETY_TIMER_ENABLE0))
		seq_puts(seq, "Charging safety timer 5hrs");
	else if (!(val & CHARGING_SAFETY_TIMER_ENABLE1) && (val & CHARGING_SAFETY_TIMER_ENABLE0))
		seq_puts(seq, "Charging safety timer 8hrs");
	else if ((val & CHARGING_SAFETY_TIMER_ENABLE1) && !(val & CHARGING_SAFETY_TIMER_ENABLE0))
		seq_puts(seq, "Charging safety timer 12hrs");
	else if ((val & CHARGING_SAFETY_TIMER_ENABLE1) && (val & CHARGING_SAFETY_TIMER_ENABLE0))
		seq_puts(seq, "Charging safety timer 20hrs");

	if (val & JEITA_ISET)
		seq_printf(seq, "JEITA_ISET low temperature set to 20%%\n");
	else
		seq_printf(seq, "JEITA_ISET low temperature set to 50%%\n");

}

static void bq25898_reg8_to_human(struct seq_file *seq, u8 reg, u8 val)
{

	int res = 0;

	if (val & BAT_COMP2)
		res += 80;
	if (val & BAT_COMP1)
		res += 40;
	if (val & BAT_COMP0)
		res += 20;

	seq_printf(seq, "BAT_COMP IR compensation %d Ohm\n", res);

	res = 0;

	if (val & VCLAMP2)
		res += 128;
	if (val & VCLAMP1)
		res += 64;
	if (val & VCLAMP0)
		res += 32;

	seq_printf(seq, "VCLAMP IRcompensation voltage clamp %dmV\n", res);

	if (!(val & THERMAL_REGUL_THRESOLD1) && !(val & THERMAL_REGUL_THRESOLD0))
		seq_puts(seq, "Thermal threshold 60C\n");
	else if (!(val & THERMAL_REGUL_THRESOLD1) && (val & THERMAL_REGUL_THRESOLD0))
		seq_puts(seq, "Thermal threshold 80C\n");
	else if ((val & THERMAL_REGUL_THRESOLD1) && !(val & THERMAL_REGUL_THRESOLD0))
		seq_puts(seq, "Thermal threshold 100C\n");
	else if ((val & THERMAL_REGUL_THRESOLD1) && (val & THERMAL_REGUL_THRESOLD0))
		seq_puts(seq, "Thermal threshold 120C\n");
}

static void bq25898_reg9_to_human(struct seq_file *seq, u8 reg, u8 val)
{

	if (val & FORCE_ICD)
		seq_puts(seq, "FORCE_ICD input detection algo forced\n");
	else
		seq_puts(seq, "FORCE_ICD input detection alogo NOT forced\n");

	if (val & SAFETY_TIMER_SLOW_ENABLE)
		seq_puts(seq, "Safety timer slowed x2\n");
	else
		seq_puts(seq, "Safety timer normal\n");

	if (val & BATFET_DISABLE)
		seq_puts(seq, "BATFET_DISABLE Q4 turned off\n");
	else
		seq_puts(seq, "BATFET_DISABLE Q4 turned on\n");

	if (val & JEITA_VSET)
		seq_puts(seq, "JEITA_VSET charge voltage set to VREG\n");
	else
		seq_puts(seq, "JEITA_VSET charge voltage set to VREG minus 200mV\n");

	if (val & BATFET_DLY)
		seq_puts(seq, "BATFET_DLY Turn off BATFET after 10s if BATFET_DISABLE set\n");
	else
		seq_puts(seq, "BATFET_DLY Turn off BATFET immedialety if BATFET_DISABLE set\n");

	if (val & BATFET_RST_EN)
		seq_puts(seq, "BATFET_RST_EN is enabled\n");
	else
		seq_puts(seq, "BATFET_RST_EN_DLY is disabled\n");

	if (val & PUMPX_UP)
		seq_puts(seq, "PUMPX_UP pump express voltage up is enabled\n");
	else
		seq_puts(seq, "PUMPX_UP pump express voltage up is disabled\n");

	if (val & PUMPX_DN)
		seq_puts(seq, "PUMPX_DN pump express voltage down is enabled\n");
	else
		seq_puts(seq, "PUMPX_DN pump express voltage down is disabled\n");
}

static void bq25898_rega_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 4550;

	if (val & BOOSTV3)
		res += 512;
	if (val & BOOSTV2)
		res += 256;
	if (val & BOOSTV1)
		res += 128;
	if (val & BOOSTV0)
		res += 64;

	seq_printf(seq, "Boost Voltage %dmV\n", res);

	if (val & PMF_OTG_DIS)
		seq_puts(seq, "PMF_OTG_DIS disabled\n");
	else
		seq_puts(seq, "PMF_OTG_DIS enabled\n");

	seq_puts(seq, "BOOST_LIM ");
	if (!(val & BOOST_LIM2) && !(val & BOOST_LIM1) && !(val & BOOST_LIM0))
		seq_puts(seq, "0.5A\n");
	else if (!(val & BOOST_LIM2) && !(val & BOOST_LIM1) && (val & BOOST_LIM0))
		seq_puts(seq, "0.8A\n");
	else if (!(val & BOOST_LIM2) && (val & BOOST_LIM1) && !(val & BOOST_LIM0))
		seq_puts(seq, "1.0A\n");
	else if (!(val & BOOST_LIM2) && (val & BOOST_LIM1) && (val & BOOST_LIM0))
		seq_puts(seq, "1.2A\n");
	else if ((val & BOOST_LIM2) && !(val & BOOST_LIM1) && !(val & BOOST_LIM0))
		seq_puts(seq, "1.5A\n");
	else if ((val & BOOST_LIM2) && !(val & BOOST_LIM1) && (val & BOOST_LIM0))
		seq_puts(seq, "1.8A\n");
	else if ((val & BOOST_LIM2) && (val & BOOST_LIM1) && !(val & BOOST_LIM0))
		seq_puts(seq, "2.1A\n");
	else if ((val & BOOST_LIM2) && (val & BOOST_LIM1) && (val & BOOST_LIM0))
		seq_puts(seq, "2.4A\n");

}

static void bq25898_regb_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	seq_puts(seq, "VBUS_STATUS ");
	if (!(val & VBUS_STATUS2) && !(val & VBUS_STATUS1) && !(val & VBUS_STATUS0))
		seq_puts(seq, "VBUS not input\n");
	else if (!(val & VBUS_STATUS2) && !(val & VBUS_STATUS1) && (val & VBUS_STATUS0))
		seq_puts(seq, "VBUS USB host SDP\n");
	else if (!(val & VBUS_STATUS2) && (val & VBUS_STATUS1) && !(val & VBUS_STATUS0))
		seq_puts(seq, "VBUS adapter 3.25A\n");


	seq_puts(seq, "CHARGER_STATUS ");
	if (!(val & CHARGER_STATUS1) && !(val & CHARGER_STATUS0))
		seq_puts(seq, "Not charging\n");
	else if (!(val & CHARGER_STATUS1) && (val & CHARGER_STATUS0))
		seq_puts(seq, "Precharge\n");
	else if ((val & CHARGER_STATUS1) && !(val & CHARGER_STATUS0))
		seq_puts(seq, "Fast charging\n");
	else if ((val & CHARGER_STATUS1) && (val & CHARGER_STATUS0))
		seq_puts(seq, "Charge termination done\n");

	seq_puts(seq, "VSYS_STAT ");
	if (val & VSYS_STAT)
		seq_puts(seq, "In VSYS min regulation (BAT < VSYSMIN)\n");
	else
		seq_puts(seq, "Not in VSYSMIN\n");

}

static void bq25898_regc_to_human(struct seq_file *seq, u8 reg, u8 val)
{

	seq_puts(seq, "WATCHDOG_FAULT ");
	if (val & WATCHDOG_FAULT)
		seq_puts(seq, "WDT Timer expiration\n");
	else
		seq_puts(seq, "WDT Normal\n");

	seq_puts(seq, "CHARGER_FAULT ");
	if (!(val & CHARGER_FAULT1) && !(val & CHARGER_FAULT0))
		seq_puts(seq, "Charger Normal\n");
	else if (!(val & CHARGER_FAULT1) && (val & CHARGER_FAULT0))
		seq_puts(seq, "Charger input fault\n");
	else if ((val & CHARGER_FAULT1) && !(val & CHARGER_FAULT0))
		seq_puts(seq, "Charger thermal shutdown\n");
	else if ((val & CHARGER_FAULT1) && (val & CHARGER_FAULT0))
		seq_puts(seq, "Charger safety timer expiration\n");

	seq_puts(seq, "BAT_FAULT ");
	if (val & BAT_FAULT)
		seq_puts(seq, "BATOVP!!\n");
	else
		seq_puts(seq, "Normal\n");

	seq_puts(seq, "NTC_FAULT");
	if (!(val & NTC_FAULT2) && !(val & NTC_FAULT1) && !(val & NTC_FAULT0))
		seq_puts(seq, "Normal");
	else if (!(val & NTC_FAULT2) && !(val & NTC_FAULT1) && (val & NTC_FAULT0))
		seq_puts(seq, "TS Cold");
	else if (!(val & NTC_FAULT2) && (val & NTC_FAULT1) && !(val & NTC_FAULT0))
		seq_puts(seq, "TS Hot or Warm (buck mode)");
	else if (!(val & NTC_FAULT2) && (val & NTC_FAULT1) && (val & NTC_FAULT0))
		seq_puts(seq, "cool");
	else if ((val & NTC_FAULT2) && !(val & NTC_FAULT1) && (val & NTC_FAULT0))
		seq_puts(seq, "Cold");
	else if ((val & NTC_FAULT2) && (val & NTC_FAULT1) && !(val & NTC_FAULT0))
		seq_puts(seq, "Hot (buck mode)");
	else if ((val & NTC_FAULT2) && !(val & NTC_FAULT1) && (val & NTC_FAULT0))
		seq_puts(seq, "Cold (boost mode)");
	else if ((val & NTC_FAULT2) && (val & NTC_FAULT1) && !(val & NTC_FAULT0))
		seq_puts(seq, "Hot (boost mode)");
}

static void bq25898_regd_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 0;

	seq_puts(seq, "FORCE_VINDPM ");
	if (val & FORCE_VINDPM)
		seq_puts(seq, "Force VINDPM, bypass algo\n");
	else
		seq_puts(seq, "Run VINDPM threshold algo\n");

	res = 2600;
	if (val & VINDPM6)
		res += 6400;
	if (val & VINDPM5)
		res += 3200;
	if (val & VINDPM4)
		res += 1600;
	if (val & VINDPM3)
		res += 800;
	if (val & VINDPM2)
		res += 400;
	if (val & VINDPM1)
		res += 200;
	if (val & VINDPM0)
		res += 100;

	seq_printf(seq, "VINDPM Absolute threshold %dmV\n", res);
}

static void bq25898_rege_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 2304;

	seq_puts(seq, "THERM_STAT ");
	if (val & THERM_STAT)
		seq_puts(seq, "in thermal regulation\n");
	else
		seq_puts(seq, "normal thermal\n");

	if (val & ADC_CONV_BATV6)
		res += 1280;
	if (val & ADC_CONV_BATV5)
		res += 640;
	if (val & ADC_CONV_BATV4)
		res += 320;
	if (val & ADC_CONV_BATV3)
		res += 160;
	if (val & ADC_CONV_BATV2)
		res += 80;
	if (val & ADC_CONV_BATV1)
		res += 40;
	if (val & ADC_CONV_BATV6)
		res += 20;

	seq_printf(seq, "ADC_CONV ADC Conversion BAT voltage %dmV\n", res);

}

static void bq25898_regf_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 2304;

	if (val & ADDC_CONV_SYSV6)
		res += 1280;
	if (val & ADDC_CONV_SYSV5)
		res += 640;
	if (val & ADDC_CONV_SYSV4)
		res += 320;
	if (val & ADDC_CONV_SYSV3)
		res += 160;
	if (val & ADDC_CONV_SYSV2)
		res += 80;
	if (val & ADDC_CONV_SYSV1)
		res += 40;
	if (val & ADDC_CONV_SYSV6)
		res += 20;

	seq_printf(seq, "ADDC_CONV ADDC Conversion SYS voltage %dmV\n", res);
}

static void bq25898_reg10_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 2100;

	if (val & TSPCPT6)
		res += 2976;
	if (val & TSPCPT5)
		res += 1488;
	if (val & TSPCPT4)
		res += 744;
	if (val & TSPCPT3)
		res += 372;
	if (val & TSPCPT2)
		res += 186;
	if (val & TSPCPT1)
		res += 93;
	if (val & TSPCPT0)
		res += 46;

	seq_printf(seq, "TSPCPT ADC Conversion of TS Voltage ~%d/100 %% of REGN\n", res);
}

static void bq25898_reg11_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 0;

	if (val & VBUS_ATTACHED)
		seq_puts(seq, "VBUS attached\n");
	else
		seq_puts(seq, "VBUS NOT attached\n");

	res = 2600;
	if (val & VBUSV_ADC_VOLT6)
		res += 6400;
	if (val & VBUSV_ADC_VOLT5)
		res += 3200;
	if (val & VBUSV_ADC_VOLT4)
		res += 1600;
	if (val & VBUSV_ADC_VOLT3)
		res += 800;
	if (val & VBUSV_ADC_VOLT2)
		res += 400;
	if (val & VBUSV_ADC_VOLT1)
		res += 200;
	if (val & VBUSV_ADC_VOLT0)
		res += 100;

	seq_printf(seq, "VBUSV ADC conversion of Vbus voltage %dmV\n", res);

}

static void bq25898_reg12_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 0;
	if (val & ADC_CHARGE_CURRENT6)
		res += 3200;
	if (val & ADC_CHARGE_CURRENT5)
		res += 1600;
	if (val & ADC_CHARGE_CURRENT4)
		res += 800;
	if (val & ADC_CHARGE_CURRENT3)
		res += 400;
	if (val & ADC_CHARGE_CURRENT2)
		res += 200;
	if (val & ADC_CHARGE_CURRENT1)
		res += 100;
	if (val & ADC_CHARGE_CURRENT0)
		res += 50;

	seq_printf(seq, "ADC_CHARGE_CURRENT ADC conversion of charge current %dmV\n", res);
}

static void bq25898_reg13_to_human(struct seq_file *seq, u8 reg, u8 val)
{
	int res = 100;

	seq_puts(seq, "VDPM_STAT ");
	if (val & VDPM_STAT)
		seq_puts(seq, "in VINDPM\n");
	else
		seq_puts(seq, "not in VINDPM\n");

	seq_puts(seq, "IDPM_STAT ");
	if (val & IDPM_STAT)
		seq_puts(seq, "in IINDPM\n");
	else
		seq_puts(seq, "not in IINDPM\n");


	if (val & IDPM_CURRENT_LIM5)
		res += 1600;
	if (val & IDPM_CURRENT_LIM4)
		res += 800;
	if (val & IDPM_CURRENT_LIM3)
		res += 400;
	if (val & IDPM_CURRENT_LIM2)
		res += 200;
	if (val & IDPM_CURRENT_LIM1)
		res += 100;
	if (val & IDPM_CURRENT_LIM0)
		res += 50;

	seq_printf(seq, "IDPMC_CURRENT_LIM %dmA\n", res);
}

static void bq25898_reg14_to_human(struct seq_file *seq, u8 reg, u8 val)
{

	seq_puts(seq, "REG_RST ");
	if (val & REG_RST)
		seq_puts(seq, "reset to default\n");
	else
		seq_puts(seq, "keep current register setting\n");

	seq_puts(seq, "IDC_OPTIMIZED ");
	if (val & ICD_OPTIMIZED)
		seq_puts(seq, "maximum input current detected\n");
	else
		seq_puts(seq, "detection in progress or disabled\n");

	seq_printf(seq, "Revision: %d", (BQ25898_DEVREG_CTRL_REG & (DEV_REV1 | DEV_REV0)) & 0x3);
}

/* invoked by reading content of register >= BQ25898_ALL_REGISTER
 * basically, any unknown register */
static void bq25898_regall_to_human(struct i2c_client *client, struct seq_file *seq)
{
	u8 reg = 0;
	u8 val = 0;

	if (!client || !seq)
		return;

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg0_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg1_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg2_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg3_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg4_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg5_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg6_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg7_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg8_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg9_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_rega_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_regb_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_regc_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_regd_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_rege_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_regf_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg10_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg11_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg12_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg13_to_human(seq, reg, val);

	seq_printf(seq, "REG %02x\n", reg);
	val = bq25898_read_reg(client, reg++);
	if (val >= 0)
		bq25898_reg14_to_human(seq, reg, val);
}


static int bq25898_reg_show(struct seq_file *seq, void *unused)
{
	int val;
	u8 reg;
	struct bq25898_debugfs_file *dbgfs_file;

	if (!seq)
		return -EINVAL;

	dbgfs_file = (struct bq25898_debugfs_file *) seq->private;

	reg = (u8) (dbgfs_file->regnr);
	val = bq25898_read_reg(dbgfs_file->parent->client, reg);
	seq_printf(seq, "0x%02x\n", val);

	/* if we want to have a human output -dump all register */
	if (reg >= BQ25898_ALL_REGISTER)
		bq25898_regall_to_human(dbgfs_file->parent->client, seq);

	return 0;
}

static ssize_t bq25898_dbgfs_write(struct file *pfile, const char __user *user_data,
				size_t count, loff_t *ppos)
{
	u8 val, reg;
	struct bq25898_debugfs_file *dbgfs_file;
	struct seq_file *seq = pfile->private_data;
	int ret;
	char locbuf[BUFSIZ_COPY_FROM_USER+1] = {0};

	if (!seq)
		return -EINVAL;

	dbgfs_file = (struct bq25898_debugfs_file*) seq->private;

	if (!dbgfs_file)
		return -EINVAL;

	if (count > BUFSIZ_COPY_FROM_USER)
		count = BUFSIZ_COPY_FROM_USER;

	if (copy_from_user(locbuf, user_data, count))
		return -EINVAL;

	ret = kstrtou8(locbuf, 10, &val);
	if (ret < 0) {
		/* try base 16 */
		ret = kstrtou8(locbuf, 16, &val);
		if (ret < 0)
			return ret;
	}

	reg = (u8) dbgfs_file->regnr;

	if (reg < BQ25898_FIRST_REGISTER || reg > BQ25898_LAST_REGISTER)
		return -EINVAL;

	ret = bq25898_write_reg(dbgfs_file->parent->client, reg, val);

	if (ret < 0) {
		dev_err(&dbgfs_file->parent->client->dev, "Error writing val 0x%02X:%d\n", val, ret);
		return ret;
	}

	return count;
}

static int bq25898_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq25898_reg_show, inode->i_private);
}

static const struct file_operations bq25898_dbg_fops = {
	.open = bq25898_dbgfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = bq25898_dbgfs_write,
};

static void bq25898_debugfs_init(struct bq25898_charger *chip)
{
	struct dentry *fentry;
	u32 i;
	char name[6] = {0};

	if (!chip)
		return;

	chip->dbgfs_dir = debugfs_create_dir(DEV_NAME, NULL);
	if (chip->dbgfs_dir == NULL)
		goto debugfs_root_exit;

	for (i = 0; i <= BQ25898_ALL_REGISTER; i++) {
		snprintf(name, 6, "%02x", i);
		chip->dbgfs_file[i].parent = chip;
		chip->dbgfs_file[i].regnr = i;
		fentry = debugfs_create_file(name, S_IRUGO,
						chip->dbgfs_dir,
						&chip->dbgfs_file[i],
						&bq25898_dbg_fops);

		if (fentry == NULL)
			goto debugfs_err_exit;
	}

	fentry = debugfs_create_u32(BQ25898_INT_COUNTER, S_IRUGO,
			chip->dbgfs_dir, &chip->irq_counter);

	if (fentry == NULL)
		goto debugfs_err_exit;

	return;

debugfs_err_exit:
	debugfs_remove_recursive(chip->dbgfs_dir);
debugfs_root_exit:
	dev_err(&chip->client->dev, "Error Creating debugfs!!\n");
	return;
}

static void bq25898_debugfs_exit(struct bq25898_charger *chip)
{

	if (!chip)
		return;

	debugfs_remove_recursive(chip->dbgfs_dir);

	return;
}
#else /* !CONFIG_DEBUG_FS */

static int bq25898_debugfs_init(struct bq25898_charger *chip)
{
	(void) chip;
	return 0;
}

static int bq25898_debugfs_exit(struct bq25898_charger *chip)
{
	(void) chip;
	return;
}
#endif /* !CONFIG_DEBUG_FS */


/* Don't call this function directly from interrupt context! */
static int bq25898_charger_configure(struct i2c_client *client)
{
	int ret = 0;

	if (!client)
		return -EINVAL;

	dev_dbg(&client->dev, "Reconfigure charger, disabling charging\n");

	/* disable charging */
	ret = bq25898_read_modify_reg(client, BQ25898_CHARGE_CTRL_REG,
				CHG_CONFIG, 0);

	if (ret < 0)
		dev_err(&client->dev, "error disabling charging %d\n", ret);

	dev_dbg(&client->dev, "enabling charge termination\n");

	/* enable charge termination */
	ret = bq25898_read_modify_reg(client, BQ25898_TERM_WDT_SFTY_CTRL_REG,
				CHARGE_TERM_ENABLE, 0x80);

	if (ret < 0)
		dev_err(&client->dev, "error enabling charge termination %d\n", ret);

	dev_dbg(&client->dev, "enabling charging\n");

	/* enable charging */
	ret = bq25898_read_modify_reg(client, BQ25898_CHARGE_CTRL_REG,
				CHG_CONFIG, 0x10);

	if (ret < 0)
		dev_err(&client->dev, "error enabling charging %d\n", ret);

	return ret;
}


static int bq25898_wdt_configure(struct i2c_client *client, enum bq25898_wdt_timing val)
{
	u8 reg;

	if (!client)
		return -EINVAL;

	reg = bq25898_read_reg(client, BQ25898_TERM_WDT_SFTY_CTRL_REG);
	if (reg < 0)
		return reg;

	dev_dbg(&client->dev, "current wdt reg: 0x%02x\n", reg);

	switch (val) {
	case BQ25898_WDT_TIMER_DISABLE:
		reg &= ~(I2C_WDT_TIMER_SETTINGS1 | I2C_WDT_TIMER_SETTINGS0);
		dev_info(&client->dev, "wdt disabled, reg: 0x%02x\n", reg);
		break;
	case BQ25898_WDT_TIMER_40S:
		reg &= I2C_WDT_TIMER_SETTINGS1;
		reg |= I2C_WDT_TIMER_SETTINGS0;
		break;
	case BQ25898_WDT_TIMER_80S:
		reg &= I2C_WDT_TIMER_SETTINGS0;
		reg |= I2C_WDT_TIMER_SETTINGS1;
		break;
	case BQ25898_WDT_TIMER_160S:
		reg |= (I2C_WDT_TIMER_SETTINGS1 | I2C_WDT_TIMER_SETTINGS0);
		break;
	}

	dev_dbg(&client->dev, "wdt new value: 0x%02x\n", reg);

	reg = bq25898_write_reg(client, BQ25898_TERM_WDT_SFTY_CTRL_REG, reg);
	dev_dbg(&client->dev, "wdt ret: 0x%02x\n", reg);

	return reg;
}

static void bq25898_handle_charging_worker(struct work_struct *work)
{
	int ret, val;
	struct bq25898_charger *chip = container_of(work, struct bq25898_charger,
						charge_status_work);

	if (!chip)
		return;

	dev_dbg(&chip->client->dev, "Received charger interrupt\n");

	val = bq25898_read_reg(chip->client, BQ25898_STATUS_REG);
	if (val < 0) {
		dev_dbg(&chip->client->dev, "Error reading charger reg %d:%d\n",
			BQ25898_STATUS_REG, val);
		return;
	}

	dev_dbg(&chip->client->dev, "Charger_status (0x%02x):0x%02x\n", BQ25898_STATUS_REG, val);

	chip->irq_counter++;

	/*
	 * kick hack for the POC
	 * if charge termination, then schedule a charging for 30 more mn.
	 */

	if ((val & CHARGER_STATUS1) && (val & CHARGER_STATUS0)) {
		/* Charge termination done */
		dev_dbg(&chip->client->dev, "Charge terminated, disabling charging\n");
		/* disable charging */
		ret = bq25898_read_modify_reg(chip->client, BQ25898_CHARGE_CTRL_REG,
					CHG_CONFIG, 0);

		if (ret < 0)
			dev_err(&chip->client->dev, "error disabling charging %d\n", ret);

		dev_dbg(&chip->client->dev, "disabling charge termination\n");
		/* disable charge termination */
		ret = bq25898_read_modify_reg(chip->client, BQ25898_TERM_WDT_SFTY_CTRL_REG,
					CHARGE_TERM_ENABLE, 0);

		if (ret < 0)
			dev_err(&chip->client->dev, "error disabling charge termination %d\n", ret);

		dev_dbg(&chip->client->dev, "enabling charging\n");
		/* enable charging */
		ret = bq25898_read_modify_reg(chip->client, BQ25898_CHARGE_CTRL_REG,
					CHG_CONFIG, 0x10);

		if (ret < 0)
			dev_err(&chip->client->dev, "error enabling charging %d\n", ret);

		dev_info(&chip->client->dev, "charge restarted for %d mn\n", chip->postcharge_duration);

		/* the timer will stop the charging when fired and will reenable charge termination */
		schedule_delayed_work(&chip->sw_term_work, (chip->postcharge_duration * 60 * HZ));
	} else
		dev_dbg(&chip->client->dev,
			"charge is not complete, discarding received charger interrupt\n");


	val = bq25898_read_reg(chip->client, BQ25898_FAULT_REG);
	if (val < 0) {
		dev_dbg(&chip->client->dev, "Error reading charger reg %d:%d\n",
			BQ25898_FAULT_REG, val);
		return;
	}

	dev_info(&chip->client->dev, "charger_fault reg (0x%02x):0x%02x\n", BQ25898_FAULT_REG, val);

	return;
}

static int bq25898_notify_charge_status_change(struct notifier_block *self,
					unsigned long action, void *param)
{
	struct bq25898_charger *chip = container_of(self, struct bq25898_charger,
						pmic_notifier);

	if (!chip)
		return NOTIFY_DONE;

	dev_dbg(&chip->client->dev, "Received PMIC notification action:%lu\n", action);

	switch (action)	{
	case PMIC_ACTION_CHARGING_STATUS:
		queue_work(system_nrt_wq, &chip->charge_status_work);
		return NOTIFY_OK;
	case PMIC_ACTION_BATTERY_ZONE_CHANGED:
	case PMIC_ACTION_OVERHEAT:
	default:
		return NOTIFY_DONE;
	}
}

static void bq25898_sw_config_worker(struct work_struct *work)
{
	int ret = 0;
	struct bq25898_charger *chip = container_of(work, struct bq25898_charger,
						sw_config_work);

	if (!chip)
		return;

	ret = bq25898_charger_configure(chip->client);
	dev_dbg(&chip->client->dev, "notifier charger configure: 0x%x\n", ret);
}


static void bq25898_sw_charge_term_worker(struct work_struct *work)
{
	int ret = 0;
	struct bq25898_charger *chip = container_of(work, struct bq25898_charger,
						sw_term_work.work);

	if (!chip)
		return;

	dev_dbg(&chip->client->dev, "timer expired, disabling charging\n");
	/* disable charging, we will need to reenable termination when usb
	 * will be unpluggeg/plugged */
	ret = bq25898_read_modify_reg(chip->client, BQ25898_CHARGE_CTRL_REG,
				CHG_CONFIG, 0);
	if (ret < 0)
		dev_err(&chip->client->dev, "error disabling charging\n");

}

static void bq25898_sw_batmon_worker(struct work_struct *work)
{
	struct bq25898_charger *chip = container_of(work, struct bq25898_charger,
						batmon_work.work);

	if (!chip)
		return;

	dev_dbg(&chip->client->dev, "running battery monitor\n");


	/*
	 * ** THIS IS A PLACEHOLDER **
	 * This is here that we will handle battery profile
	 * Temperature handling, SHOULD BE REFINED before MERGE:
	 *  0 to 10 deg : charging 100mA
	 * 10 to 45 deg : charging 330mA
	 * 45 to 50 deg :	V > 4.1v -> Stop charging
	 *			V < 4.1v -> Charging 330 mA
	 * greater than	 50 deg: Stop charging (HW handling)
	 */

	/* reschedule ourself */
	schedule_delayed_work(&chip->batmon_work, BQ25898_BAT_MONITOR_DELAY);
	return;
}

static int bq25898_suspend(struct device *dev)
{
	struct bq25898_charger *chip;

	chip = dev_get_drvdata(dev);

	dev_dbg(&chip->client->dev, "suspend\n");
	return 0;
}

static int bq25898_resume(struct device *dev)
{
	struct bq25898_charger *chip;

	chip = dev_get_drvdata(dev);

	dev_dbg(&chip->client->dev, "%s\n", __func__);
	return 0;
}

static int bq25898_runtime_suspend(struct device *dev)
{
	struct bq25898_charger *chip;

	chip = dev_get_drvdata(dev);

	dev_dbg(&chip->client->dev, "%s\n", __func__);
	return 0;
}

static int bq25898_runtime_resume(struct device *dev)
{
	struct bq25898_charger *chip;

	chip = dev_get_drvdata(dev);

	dev_dbg(&chip->client->dev, "%s\n", __func__);
	return 0;
}

static int bq25898_runtime_idle(struct device *dev)
{
	struct bq25898_charger *chip;

	chip = dev_get_drvdata(dev);

	dev_dbg(&chip->client->dev, "%s\n", __func__);
	return 0;
}

static int register_otg_notification(struct bq25898_charger *chip)
{
	int retval;

	chip->otg_usb_change.notifier_call = bq25898_usb_change_notifier;
	chip->otg_usb_change.priority = 1;

	/*
	 * Get the USB transceiver instance
	 */
	chip->transceiver = usb_get_phy(USB_PHY_TYPE_USB2);
	if (chip->transceiver < 0) {
		dev_err(&chip->client->dev, "failed to get the USB transceiver\n");
		return -ENODEV;
	}

	retval = usb_register_notifier(chip->transceiver, &chip->otg_usb_change);
	if (retval) {
		dev_err(&chip->client->dev, "failed to register otg notifier:%d\n",
			retval);
		return -EINVAL;
	}

	return 0;
}

static int register_pmic_notification(struct bq25898_charger *chip)
{
	int ret;

	chip->pmic_notifier.notifier_call = bq25898_notify_charge_status_change;

	ret = register_pmic_notifier(&chip->pmic_notifier);
	if (ret) {
		dev_err(&chip->client->dev, "failed to register pmic notifier:%d\n",
			ret);
		return -EINVAL;
	}

	return 0;
}

static int bq25898_get_prop_health(struct bq25898_charger *chip)
{
	int val;

	if (!chip)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	val = bq25898_read_reg(chip->client, BQ25898_FAULT_REG);

	if (val & WATCHDOG_FAULT)
		return POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;

	if ((val & CHARGER_FAULT1) && !(val & CHARGER_FAULT0))
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (!(val & CHARGER_FAULT1) && !(val & CHARGER_FAULT0))
		return POWER_SUPPLY_HEALTH_GOOD;

	if ((val & CHARGER_FAULT1) && (val & CHARGER_FAULT0))
		return POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;

	if (val & BAT_FAULT)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;


	return POWER_SUPPLY_HEALTH_UNKNOWN;
}

static int bq25898_get_prop_status(struct bq25898_charger *chip)
{
	int val;

	if (!chip)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	val = bq25898_read_reg(chip->client, BQ25898_STATUS_REG);

	if ((!(val & CHARGER_STATUS1) && !(val & CHARGER_STATUS0)) ||
		((val & CHARGER_STATUS1) && (val & CHARGER_STATUS0)))
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if ((!(val & CHARGER_STATUS1) && (val & CHARGER_STATUS0)) ||
		((val & CHARGER_STATUS1) && !(val & CHARGER_STATUS0)))
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_UNKNOWN;
}


static int bq25898_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq25898_charger *chip = container_of(psy,
						struct bq25898_charger,
						psy_usb);

	if (!val || !chip) {
		dev_err(&chip->client->dev, "%s power_supply_propval:%p, chip: %p", __func__, val, chip);
		return -EINVAL;
	}

	mutex_lock(&chip->sysfs_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq25898_get_prop_status(chip);
		dev_dbg(&chip->client->dev, "%s prop status:%d", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq25898_get_prop_health(chip);
		dev_dbg(&chip->client->dev, "%s health:%d", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chip->temperature;
		dev_dbg(&chip->client->dev, "%s temperature:%d", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = chip->current_now;
		break;
	default:
		break;
	}
	mutex_unlock(&chip->sysfs_lock);

	return 0;
}

static int bq25898_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct bq25898_charger *chip = container_of(psy,
						struct bq25898_charger,
						psy_usb);

	if (!val || !chip)
		return -EINVAL;

	mutex_lock(&chip->sysfs_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP:
		dev_dbg(&chip->client->dev, "%s temperature:%d", __func__, val->intval);
		chip->temperature = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		dev_dbg(&chip->client->dev, "%s prop current:%d", __func__, val->intval);
		chip->current_now = val->intval;
		break;
	default:
		break;
	}
	mutex_unlock(&chip->sysfs_lock);

	return 0;
}

static int bq25898_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP:		/* tbd from where we get this information */
	case POWER_SUPPLY_PROP_CURRENT_NOW:	/* provided by Healthd from Fuel Gauge */
		return 1;
	default:
		return -EPERM;
	}
	return 0;
}

static int bq25898_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter;
	struct bq25898_charger *chip;
	int bq25898x_rev;
	int ret;

	dev_dbg(&client->dev, ">probe");
	pm_runtime_enable(&client->dev);

	adapter = to_i2c_adapter(client->dev.parent);

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "platform data is null");
		return -EFAULT;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
			"I2C adapter %s doesn't support SMBUS_BYTE_DATA\n",
			adapter->name);
		return -EIO;
	}

	bq25898x_rev = bq25898_read_reg(client, BQ25898_DEVREG_CTRL_REG);
	if (bq25898x_rev < 0) {
		dev_err(&client->dev,
			"error in reading ctrl reg %02x:%d\n", BQ25898_DEVREG_CTRL_REG, bq25898x_rev);
		return bq25898x_rev;
	}

	/* Disable watchdog for the POC, TBD add a watchdog kicker */
	ret = bq25898_wdt_configure(client, BQ25898_WDT_TIMER_DISABLE);
	dev_dbg(&client->dev, "disabling watchdog: 0x%02x\n", ret);
	if (ret < 0) {
		dev_err(&client->dev, "error disabling watchdog %d\n", ret);
		return ret;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, chip);

	chip->client = client;
	chip->pdata = client->dev.platform_data;
	chip->psy_usb.name = DEV_NAME;
	chip->psy_usb.type = POWER_SUPPLY_TYPE_USB;
	chip->psy_usb.properties = bq25898_battery_properties;
	chip->psy_usb.num_properties = ARRAY_SIZE(bq25898_battery_properties);
	chip->psy_usb.set_property = bq25898_set_property;
	chip->psy_usb.get_property = bq25898_get_property;
	chip->psy_usb.property_is_writeable = bq25898_property_is_writeable;
	chip->postcharge_duration = BQ25898_POSTCHARGE_DEFAULT_DURATION_MN;
	chip->current_now = 0;
	chip->irq_counter = 0;
	chip->revision = bq25898x_rev;

	strncpy(chip->model_name,
		MODEL_NAME,
		MODEL_NAME_SIZE);
	strncpy(chip->manufacturer, DEV_MANUFACTURER,
		DEV_MANUFACTURER_NAME_SIZE);

	mutex_init(&chip->stat_lock);
	mutex_init(&chip->sysfs_lock);

	INIT_DELAYED_WORK(&chip->sw_term_work, bq25898_sw_charge_term_worker);

	/* monitor battery status and react accordingly */
	INIT_DELAYED_WORK(&chip->batmon_work, bq25898_sw_batmon_worker);

	INIT_WORK(&chip->sw_config_work, bq25898_sw_config_worker);
	INIT_WORK(&chip->charge_status_work, bq25898_handle_charging_worker);

	/* default configuration */
	ret = bq25898_charger_configure(client);
	if (ret < 0)
		dev_err(&client->dev, "error configuring charger: %d\n", ret);

	bq25898_debugfs_init(chip);

	/* register for pmic notifications */
	ret = register_pmic_notification(chip);
	if (ret < 0) {
		dev_err(&client->dev, "error registering to PMIC notification: %d\n", ret);
		goto error0;
	}

	/* register for usb change */
	ret = register_otg_notification(chip);
	if (ret < 0) {
		dev_err(&client->dev, "error registering to OTG notification: %d\n", ret);
		goto error1;
	}

	ret = power_supply_register(&chip->client->dev, &chip->psy_usb);
	if (ret < 0)
		dev_err(&client->dev, "error registering power supply: %d\n", ret);

	dev_dbg(&client->dev, "<probe");

	return ret;

error1:
	unregister_pmic_notifier(&chip->pmic_notifier);
error0:
	bq25898_debugfs_exit(chip);

	pm_runtime_disable(&client->dev);

	return ret;
}

static int bq25898_remove(struct i2c_client *client)
{
	struct bq25898_charger *chip;

	if (!client)
		return -EINVAL;

	chip = i2c_get_clientdata(client);

	if (!chip)
		return -EINVAL;

	flush_scheduled_work();
	flush_work(&chip->sw_config_work);
	flush_work(&chip->charge_status_work);

	if (chip->transceiver)
		usb_unregister_notifier(chip->transceiver, &chip->otg_usb_change);

	unregister_pmic_notifier(&chip->pmic_notifier);

	power_supply_unregister(&chip->psy_usb);

	bq25898_debugfs_exit(chip);

	pm_runtime_disable(&client->dev);

	return 0;
}

static const struct dev_pm_ops bq25898_pm_ops = {
	.suspend = bq25898_suspend,
	.resume = bq25898_resume,
	.runtime_suspend = bq25898_runtime_suspend,
	.runtime_resume = bq25898_runtime_resume,
	.runtime_idle = bq25898_runtime_idle,
};

static const struct i2c_device_id bq25898_id[] = {
	{DEV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, bq25898_id);

static struct i2c_driver bq25898_driver = {
	.driver = {
		   .name = DEV_NAME,
		   .pm = &bq25898_pm_ops,
		   },
	.probe = bq25898_probe,
	.remove = bq25898_remove,
	.id_table = bq25898_id,
};

module_i2c_driver(bq25898_driver);

MODULE_AUTHOR("Kamel Slimani <kamel.slimani@intel.com>");
MODULE_AUTHOR("Marc Blassin <marc.blassin@intel.com>");
MODULE_DESCRIPTION("BQ25898 Charger Driver");
MODULE_LICENSE("GPL");
