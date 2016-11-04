/*
 * max17042_battery.h - Fuel gauge driver for Maxim 17042 / 8966 / 8997
 *  Note that Maxim 8966 and 8997 are mfd and this is its subdevice.
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __MAX17042_BATTERY_H_
#define __MAX17042_BATTERY_H_

/* No of cell characterization words to be written to max17042 */
#define CELL_CHAR_TBL_SAMPLES	48

#define BATTID_LEN		8
#define MAX_TABLE_NAME_LEN	8
#define MODEL_NAME_LEN		2
#define SERIAL_NUM_LEN		6

/* fuel gauge table type for DV10 platform */
#define MAX17042_TBL_TYPE_DV10	0xff

enum max17042_register {
	MAX17042_STATUS		= 0x00,
	MAX17042_VALRT_Th	= 0x01,
	MAX17042_TALRT_Th	= 0x02,
	MAX17042_SALRT_Th	= 0x03,
	MAX17042_AtRate		= 0x04,
	MAX17042_RepCap		= 0x05,
	MAX17042_RepSOC		= 0x06,
	MAX17042_Age		= 0x07,
	MAX17042_TEMP		= 0x08,
	MAX17042_VCELL		= 0x09,
	MAX17042_Current	= 0x0A,
	MAX17042_AvgCurrent	= 0x0B,
	MAX17042_Qresidual	= 0x0C,
	MAX17042_SOC		= 0x0D,
	MAX17042_AvSOC		= 0x0E,
	MAX17042_RemCap		= 0x0F,
	MAX17042_FullCAP	= 0x10,
	MAX17042_TTE		= 0x11,
	MAX17042_V_empty	= 0x12,

	MAX17042_RSLOW		= 0x14,

	MAX17042_AvgTA		= 0x16,
	MAX17042_Cycles		= 0x17,
	MAX17042_DesignCap	= 0x18,
	MAX17042_AvgVCELL	= 0x19,
	MAX17042_MinMaxTemp	= 0x1A,
	MAX17042_MinMaxVolt	= 0x1B,
	MAX17042_MinMaxCurr	= 0x1C,
	MAX17042_CONFIG		= 0x1D,
	MAX17042_ICHGTerm	= 0x1E,
	MAX17042_AvCap		= 0x1F,
	MAX17042_ManName	= 0x20,
	MAX17042_DevName	= 0x21,
	MAX17042_DevChem	= 0x22,
	MAX17042_FullCAPNom	= 0x23,

	MAX17042_TempNom	= 0x24,
	MAX17042_TempCold	= 0x25,
	MAX17042_TempHot	= 0x26,
	MAX17042_AIN		= 0x27,
	MAX17042_LearnCFG	= 0x28,
	MAX17042_SHFTCFG	= 0x29,
	MAX17042_RelaxCFG	= 0x2A,
	MAX17042_MiscCFG	= 0x2B,
	MAX17042_TGAIN		= 0x2C,
	MAx17042_TOFF		= 0x2D,
	MAX17042_CGAIN		= 0x2E,
	MAX17042_COFF		= 0x2F,

	MAX17042_MaskSOC	= 0x32,
	MAX17042_SOCempty	= 0x33,
	MAX17042_T_empty	= 0x34,
	MAX17042_FullCAP0	= 0x35,

	MAX17042_LAvg_empty	= 0x36,
	MAX17042_FCTC		= 0x37,
	MAX17042_RCOMP0		= 0x38,
	MAX17042_TempCo		= 0x39,
	MAX17042_ETC		= 0x3A,
	MAX17042_K_empty0	= 0x3B,
	MAX17042_TaskPeriod	= 0x3C,
	MAX17042_FSTAT		= 0x3D,

	MAX17042_SHDNTIMER	= 0x3F,

	MAX17042_dQacc		= 0x45,
	MAX17042_dPacc		= 0x46,
	MAX17042_VFSOC0         = 0x48,
	MAX17042_VFRemCap	= 0x4A,

	MAX17042_QH		= 0x4D,
	MAX17042_QL		= 0x4E,

	MAX17042_VFSOC0Enable	= 0x60,
	MAX17042_MLOCKReg1	= 0x62,
	MAX17042_MLOCKReg2	= 0x63,
	MAX17042_MODELChrTbl	= 0x80,
	MAX17042_OCV		= 0xEE,
	MAX17042_OCVInternal	= 0xFB,
	MAX17042_VFSOC		= 0xFF,

};

/* Registers specific to max17047/50 */
enum max17050_register {
	MAX17050_QRTbl00	= 0x12,
	MAX17050_FullSOCThr	= 0x13,
	MAX17050_QRTbl10	= 0x22,
	MAX17050_QRTbl20	= 0x32,
	MAX17050_V_empty	= 0x3A,
	MAX17050_QRTbl30	= 0x42,
};

enum max170xx_chip_type {
	MAXIM_DEVICE_TYPE_UNKNOWN	= 0,
	MAXIM_DEVICE_TYPE_MAX17042,
	MAXIM_DEVICE_TYPE_MAX17047,
	MAXIM_DEVICE_TYPE_MAX17050,

	MAXIM_DEVICE_TYPE_NUM
};

struct max17042_config_data {
	/*
	* if config_init is 0, which means new
	* configuration has been loaded in that case
	* we need to perform complete init of chip
	*/

	u8	table_type;
	u16	size;
	u16	rev;
	u8	table_name[MAX_TABLE_NAME_LEN];
	u8      battid[BATTID_LEN];
	u8	config_init;

	u16	rcomp0;
	u16	tempCo;
	u16	kempty0;
	u16	full_cap;
	u16	cycles;
	u16	full_capnom;

	u16	soc_empty;
	u16	ichgt_term;
	u16	design_cap;
	u16	etc;
	u16	rsense;
	u16	cfg;
	u16	learn_cfg;
	u16	filter_cfg;
	u16	relax_cfg;

	/* config data specific to max17050 */
	u16	qrtbl00;
	u16	qrtbl10;
	u16	qrtbl20;
	u16	qrtbl30;
	u16	full_soc_thr;
	u16	vempty;

	u16	cell_char_tbl[CELL_CHAR_TBL_SAMPLES];
	u16	lavg_empty;
} __packed;

struct max17042_platform_data {
	bool enable_current_sense;
	bool is_init_done;
	bool is_volt_shutdown;
	bool is_capacity_shutdown;
	bool is_lowbatt_shutdown;
	bool file_sys_storage_enabled;
	bool soc_intr_mode_enabled;
	bool reset_chip;
	bool valid_battery;
	bool en_vmax_intr;
	int technology;
	int fg_algo_model; /* maxim chip algorithm model */
	char battid[BATTID_LEN + 1];
	char model_name[MODEL_NAME_LEN + 1];
	char serial_num[2*SERIAL_NUM_LEN + 1];

	/* battery safety thresholds */
	int temp_min_lim;	/* in degrees centigrade */
	int temp_max_lim;	/* in degrees centigrade */
	int volt_min_lim;	/* milli volts */
	int volt_max_lim;	/* milli volts */
	int resv_cap;

	u16 tgain;
	u16 toff;

	int (*current_sense_enabled)(void);
	int (*battery_present)(void);
	int (*battery_health)(void);
	int (*battery_status)(void);
	int (*battery_pack_temp)(int *);
	int (*save_config_data)(const char *name, void *data, int len);
	int (*restore_config_data)(const char *name, void *data, int len);
	void (*reset_i2c_lines)(void);

	bool (*is_cap_shutdown_enabled)(void);
	bool (*is_volt_shutdown_enabled)(void);
	bool (*is_lowbatt_shutdown_enabled)(void);
	int (*get_vmin_threshold)(void);
	int (*get_vmax_threshold)(void);
};

#endif /* __MAX17042_BATTERY_H_ */
