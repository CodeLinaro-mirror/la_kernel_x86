/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
*
* File Name          : lis2dg_core.c
* Authors            : AMS - VMU - Application Team
*		     : Giuseppe Barba <giuseppe.barba@st.com>
*		     : Author is willing to be considered the contact and update
*		     : point for the driver.
* Version            : V.1.0.0
* Date               : 2015/Apr/17
* Description        : LIS2DG driver
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************/

#ifndef __LIS2DG_H
#define __LIS2DG_H

#include <linux/types.h>
#include <linux/iio/trigger.h>
#include <linux/platform_data/lis2dg.h>

#define LIS2DG_WHO_AM_I_ADDR			0x0f
#define LIS2DG_WHO_AM_I_DEF			0x43
#define LIS2DG_CTRL1_ADDR			0x20
#define LIS2DG_CTRL2_ADDR			0x21
#define LIS2DG_CTRL3_ADDR			0x22
#define LIS2DG_CTRL4_INT1_PAD_ADDR		0x23
#define LIS2DG_CTRL5_INT2_PAD_ADDR		0x24
#define LIS2DG_FIFO_CTRL_ADDR			0x25
#define LIS2DG_OUTX_L_ADDR			0x28
#define LIS2DG_OUTY_L_ADDR			0x2a
#define LIS2DG_OUTZ_L_ADDR			0x2c
#define LIS2DG_TAP_THS_6D_ADDR			0x31
#define LIS2DG_WAKE_UP_THS_ADDR			0x33
#define LIS2DG_FREE_FALL_ADDR			0x35
#define LIS2DG_STEP_C_MINTHS_ADDR		0x3a
#define LIS2DG_STEP_C_MINTHS_RST_NSTEP_MASK	0x80
#define LIS2DG_STEP_C_OUT_L_ADDR		0x3b
#define LIS2DG_FUNC_CTRL_ADDR			0x3f
#define LIS2DG_FUNC_CTRL_TILT_MASK		0x10
#define LIS2DG_FUNC_CTRL_SIGN_MOT_MASK		0x02
#define LIS2DG_FUNC_CTRL_STEP_CNT_MASK		0x01
#define LIS2DG_FUNC_CTRL_EV_MASK		(LIS2DG_FUNC_CTRL_TILT_MASK | \
						LIS2DG_FUNC_CTRL_SIGN_MOT_MASK | \
						LIS2DG_FUNC_CTRL_STEP_CNT_MASK)
#define LIS2DG_FIFO_THS_ADDR			0x2e
#define LIS2DG_FIFO_THS_MASK			0xff
#define LIS2DG_ODR_ADDR				LIS2DG_CTRL1_ADDR
#define LIS2DG_ODR_MASK				0xf0
#define LIS2DG_ODR_POWER_OFF_VAL		0x00
#define LIS2DG_ODR_1HZ_LP_VAL			0x08
#define LIS2DG_ODR_12HZ_LP_VAL			0x09
#define LIS2DG_ODR_25HZ_LP_VAL			0x0a
#define LIS2DG_ODR_50HZ_LP_VAL			0x0b
#define LIS2DG_ODR_100HZ_LP_VAL			0x0c
#define LIS2DG_ODR_200HZ_LP_VAL			0x0d
#define LIS2DG_ODR_400HZ_LP_VAL			0x0e
#define LIS2DG_ODR_800HZ_LP_VAL			0x0f
#define LIS2DG_ODR_LP_LIST_NUM			9

#define LIS2DG_ODR_12_5HZ_HR_VAL		0x01
#define LIS2DG_ODR_25HZ_HR_VAL			0x02
#define LIS2DG_ODR_50HZ_HR_VAL			0x03
#define LIS2DG_ODR_100HZ_HR_VAL			0x04
#define LIS2DG_ODR_200HZ_HR_VAL			0x05
#define LIS2DG_ODR_400HZ_HR_VAL			0x06
#define LIS2DG_ODR_800HZ_HR_VAL			0x07
#define LIS2DG_ODR_HR_LIST_NUM			8

#define LIS2DG_FS_ADDR				LIS2DG_CTRL1_ADDR
#define LIS2DG_FS_MASK				0x0c
#define LIS2DG_FS_2G_VAL			0x00
#define LIS2DG_FS_4G_VAL			0x02
#define LIS2DG_FS_8G_VAL			0x03
#define LIS2DG_FS_16G_VAL			0x01

/*
 * Sensitivity sets in LP mode [ug]
 */
#define LIS2DG_FS_2G_GAIN_LP			IIO_G_TO_M_S_2(3906)
#define LIS2DG_FS_4G_GAIN_LP			IIO_G_TO_M_S_2(7813)
#define LIS2DG_FS_8G_GAIN_LP			IIO_G_TO_M_S_2(15625)
#define LIS2DG_FS_16G_GAIN_LP			IIO_G_TO_M_S_2(31250)

/*
 * Sensitivity sets in HR mode [ug]
 */
#define LIS2DG_FS_2G_GAIN_HR			IIO_G_TO_M_S_2(244)
#define LIS2DG_FS_4G_GAIN_HR			IIO_G_TO_M_S_2(488)
#define LIS2DG_FS_8G_GAIN_HR			IIO_G_TO_M_S_2(976)
#define LIS2DG_FS_16G_GAIN_HR			IIO_G_TO_M_S_2(1952)

#define LIS2DG_MODE_DEFAULT			LIS2DG_LP_MODE
#define LIS2DG_INT1_S_TAP_MASK			0x40
#define LIS2DG_INT1_WAKEUP_MASK			0x20
#define LIS2DG_INT1_FREE_FALL_MASK		0x10
#define LIS2DG_INT1_TAP_MASK			0x08
#define LIS2DG_INT1_6D_MASK			0x04
#define LIS2DG_INT1_FTH_MASK			0x02
#define LIS2DG_INT1_DRDY_MASK			0x01
#define LIS2DG_INT1_EVENTS_MASK			(LIS2DG_INT1_S_TAP_MASK | \
						LIS2DG_INT1_WAKEUP_MASK | \
						LIS2DG_INT1_FREE_FALL_MASK | \
						LIS2DG_INT1_TAP_MASK | \
						LIS2DG_INT1_6D_MASK | \
						LIS2DG_INT1_FTH_MASK | \
						LIS2DG_INT1_DRDY_MASK)
#define LIS2DG_INT2_ON_INT1_MASK		0x20
#define LIS2DG_INT2_TILT_MASK			0x10
#define LIS2DG_INT2_SIG_MOT_DET_MASK		0x08
#define LIS2DG_INT2_STEP_DET_MASK		0x04
#define LIS2DG_INT2_FTH_MASK			0x02
#define LIS2DG_INT2_DRDY_MASK			0x01
#define LIS2DG_INT2_EVENTS_MASK			(LIS2DG_INT2_TILT_MASK | \
						LIS2DG_INT2_SIG_MOT_DET_MASK | \
						LIS2DG_INT2_STEP_DET_MASK | \
						LIS2DG_INT2_FTH_MASK | \
						LIS2DG_INT2_DRDY_MASK)
#define LIS2DG_WAKE_UP_THS_WU_MASK		0x3f
#define LIS2DG_WAKE_UP_THS_WU_DEFAULT		0x02
#define LIS2DG_FREE_FALL_THS_MASK		0x07
#define LIS2DG_FREE_FALL_DUR_MASK		0xF8
#define LIS2DG_FREE_FALL_THS_DEFAULT		0x01
#define LIS2DG_FREE_FALL_DUR_DEFAULT		0x01
#define LIS2DG_BDU_ADDR				LIS2DG_CTRL1_ADDR
#define LIS2DG_BDU_MASK				0x01
#define LIS2DG_SOFT_RESET_ADDR			LIS2DG_CTRL2_ADDR
#define LIS2DG_SOFT_RESET_MASK			0x40
#define LIS2DG_LIR_ADDR				LIS2DG_CTRL3_ADDR
#define LIS2DG_LIR_MASK				0x04
#define LIS2DG_TAP_AXIS_ADDR			LIS2DG_CTRL3_ADDR
#define LIS2DG_TAP_AXIS_MASK			0x38
#define LIS2DG_TAP_AXIS_ANABLE_ALL		0x07
#define LIS2DG_TAP_THS_ADDR			LIS2DG_TAP_THS_6D_ADDR
#define LIS2DG_TAP_THS_MASK			0x1f
#define LIS2DG_TAP_THS_DEFAULT			0x09
#define LIS2DG_INT2_ON_INT1_ADDR		LIS2DG_CTRL5_INT2_PAD_ADDR
#define LIS2DG_INT2_ON_INT1_MASK		0x20
#define LIS2DG_FIFO_MODE_ADDR			LIS2DG_FIFO_CTRL_ADDR
#define LIS2DG_FIFO_MODE_MASK			0xe0
#define LIS2DG_FIFO_MODE_BYPASS			0x00
#define LIS2DG_FIFO_MODE_CONTINUOS		0x06
#define LIS2DG_OUT_XYZ_SIZE			8

#define LIS2DG_SELFTEST_ADDR			LIS2DG_CTRL3_ADDR
#define LIS2DG_SELFTEST_MASK			0xc0
#define LIS2DG_SELFTEST_NORMAL			0x00
#define LIS2DG_SELFTEST_POS_SIGN		0x01
#define LIS2DG_SELFTEST_NEG_SIGN		0x02

#define LIS2DG_FIFO_SRC				0x2f
#define LIS2DG_FIFO_SRC_DIFF_MASK		0x20

#define LIS2DG_FIFO_NUM_AXIS			3
#define LIS2DG_FIFO_BYTE_X_AXIS			2
#define LIS2DG_FIFO_BYTE_FOR_SAMPLE		(LIS2DG_FIFO_NUM_AXIS * \
							LIS2DG_FIFO_BYTE_X_AXIS)
#define LIS2DG_TIMESTAMP_SIZE			8

#define LIS2DG_STATUS_ADDR			0x27
#define LIS2DG_STATUS_DUP_ADDR			0x36
#define LIS2DG_FUNC_CK_GATE_ADDR		0x3d
#define LIS2DG_FUNC_CK_GATE_TILT_INT_MASK	0x80
#define LIS2DG_FUNC_CK_GATE_SIGN_M_DET_MASK	0x10
#define LIS2DG_FUNC_CK_GATE_RST_SIGN_M_MASK	0x08
#define LIS2DG_FUNC_CK_GATE_RST_PEDO_MASK	0x04
#define LIS2DG_FUNC_CK_GATE_STEP_D_MASK	0x02
#define LIS2DG_FUNC_CK_GATE_MASK		(LIS2DG_FUNC_CK_GATE_TILT_INT_MASK | \
						LIS2DG_FUNC_CK_GATE_SIGN_M_DET_MASK | \
						LIS2DG_FUNC_CK_GATE_STEP_D_MASK)
#define LIS2DG_WAKE_UP_IA_MASK			0x40
#define LIS2DG_DOUBLE_TAP_MASK			0x10
#define LIS2DG_TAP_MASK				0x08
#define LIS2DG_6D_IA_MASK			0x04
#define LIS2DG_FF_IA_MASK			0x02
#define LIS2DG_DRDY_MASK			0x01
#define LIS2DG_EVENT_MASK			(LIS2DG_WAKE_UP_IA_MASK | \
						LIS2DG_DOUBLE_TAP_MASK | \
						LIS2DG_TAP_MASK | \
						LIS2DG_6D_IA_MASK | \
						LIS2DG_FF_IA_MASK)
#define LIS2DG_FIFO_SRC_ADDR			0x2f
#define LIS2DG_FIFO_SRC_FTH_MASK		0x80

#define LIS2DG_EN_BIT				0x01
#define LIS2DG_DIS_BIT				0x00
#define LIS2DG_ACCEL_ODR			1
#define LIS2DG_DEFAULT_ACCEL_FS			2
#define LIS2DG_FF_ODR				25
#define LIS2DG_STEP_D_ODR			25
#define LIS2DG_TILT_ODR				25
#define LIS2DG_SIGN_M_ODR			25
#define LIS2DG_TAP_ODR				400
#define LIS2DG_WAKEUP_ODR			25
#define LIS2DG_ACTIVITY_ODR			12
#define LIS2DG_MAX_FIFO_LENGHT			256
#define LIS2DG_MAX_CHANNEL_SPEC			4
#define LIS2DG_EVENT_CHANNEL_SPEC_SIZE		2

#define LIS2DG_DEV_NAME				"lis2dg"
#define SET_BIT(a, b)				{ a |= (1 << b); }
#define RESET_BIT(a, b)				{ a &= ~(1 << b); }
#define CHECK_BIT(a, b)				(a & (1 << b))

enum {
	LIS2DG_ACCEL = 0,
	LIS2DG_STEP_C,
	LIS2DG_TAP,
	LIS2DG_DOUBLE_TAP,
	LIS2DG_STEP_D,
	LIS2DG_TILT,
	LIS2DG_SIGN_M,
	LIS2DG_SENSORS_NUMB,
};

enum fifo_mode {
	BYPASS = 0,
	CONTINUOS,
};

#define LIS2DG_TX_MAX_LENGTH		12
#define LIS2DG_RX_MAX_LENGTH		8193

struct lis2dg_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LIS2DG_RX_MAX_LENGTH];
	u8 tx_buf[LIS2DG_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lis2dg_data;

struct lis2dg_transfer_function {
	int (*write)(struct lis2dg_data *cdata, u8 reg_addr, int len, u8 *data);
	int (*read)(struct lis2dg_data *cdata, u8 reg_addr, int len, u8 *data);
};

struct lis2dg_sensor_data {
	struct lis2dg_data *cdata;
	const char *name;
	s64 timestamp;
	u8 enabled;
	u32 odr;
	u32 gain;
	u8 sindex;
	u8 sample_to_discard;
};

struct lis2dg_data {
	const char *name;
	u8 drdy_int_pin;
	u8 selftest_status;
	u8 power_mode;
	u8 enabled_sensor;
	u32 common_odr;
	int irq;
	s64 timestamp;
	s64 accel_deltatime;
	s64 accel_timestamp;
	u8 *fifo_data;
	u16 fifo_size;
	struct device *dev;
	struct iio_dev *iio_sensors_dev[LIS2DG_SENSORS_NUMB];
	struct iio_trigger *iio_trig[LIS2DG_SENSORS_NUMB];
	struct mutex fifo_lock;
	struct mutex i2c_lock;
	struct work_struct data_work;
	const struct lis2dg_transfer_function *tf;
};

int lis2dg_common_probe(struct lis2dg_data *cdata, int irq);
#ifdef CONFIG_PM
int lis2dg_common_suspend(struct lis2dg_data *cdata);
int lis2dg_common_resume(struct lis2dg_data *cdata);
#endif
int lis2dg_allocate_rings(struct lis2dg_data *cdata);
int lis2dg_allocate_triggers(struct lis2dg_data *cdata,
			     const struct iio_trigger_ops *trigger_ops);
int lis2dg_trig_set_state(struct iio_trigger *trig, bool state);
int lis2dg_read_register(struct lis2dg_data *cdata, u8 reg_addr, int data_len,
							u8 *data);
int lis2dg_update_drdy_irq(struct lis2dg_sensor_data *sdata, bool state);
int lis2dg_set_enable(struct lis2dg_sensor_data *sdata, bool enable);
int lis2dg_update_fifo(struct lis2dg_data *cdata);
void lis2dg_common_remove(struct lis2dg_data *cdata, int irq);
void lis2dg_read_fifo(struct lis2dg_data *cdata, bool check_fifo_len);
void lis2dg_read_step_c(struct lis2dg_data *cdata);
void lis2dg_flush_works(void);
void lis2dg_deallocate_rings(struct lis2dg_data *cdata);
void lis2dg_deallocate_triggers(struct lis2dg_data *cdata);

#endif /* __LIS2DG_H */
