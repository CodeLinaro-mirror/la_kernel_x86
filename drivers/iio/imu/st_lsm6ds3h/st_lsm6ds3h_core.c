/*
 * STMicroelectronics lsm6ds3h core driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <asm/unaligned.h>

#include <linux/iio/common/st_sensors.h>
#include <linux/pm_runtime.h>
#include "st_lsm6ds3h.h"

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
#define ST_LSM6DS3H_DATA_FW		"st_lsm6ds3h_wrist_tilt_data.fw"
#define ST_LSM6DS3H_WRIST_TILT_THRESHOLD 0x10

static const u8 st_lsm6ds3h_fw[] = {
	#include "st_lsm6ds3h_wrist_tilt_data.fw"
};

DECLARE_BUILTIN_FIRMWARE(ST_LSM6DS3H_DATA_FW, st_lsm6ds3h_fw);
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

#define MS_TO_NS(msec)				((msec) * 1000 * 1000)

#ifndef MAX
#define MAX(a, b)				(((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b)				(((a) < (b)) ? (a) : (b))
#endif

#define MIN_BNZ(a, b)				(((a) < (b)) ? ((a == 0) ? \
						(b) : (a)) : ((b == 0) ? \
						(a) : (b)))

/* COMMON VALUES FOR ACCEL-GYRO SENSORS */
#define ST_LSM6DS3H_WAI_ADDRESS				0x0f
#define ST_LSM6DS3H_WAI_EXP				0x69
#define ST_LSM6DS3H_INT1_ADDR				0x0d
#define ST_LSM6DS3H_INT2_ADDR				0x0e
#define ST_LSM6DS3H_ACCEL_DRDY_IRQ_MASK			0x01
#define ST_LSM6DS3H_GYRO_DRDY_IRQ_MASK			0x02
#define ST_LSM6DS3H_MD1_ADDR				0x5e
#define ST_LSM6DS3H_MD2_ADDR				0x5f
#define ST_LSM6DS3H_ODR_LIST_NUM			6
#define ST_LSM6DS3H_ODR_POWER_OFF_VAL			0x00
#define ST_LSM6DS3H_ODR_13HZ_VAL			0x01
#define ST_LSM6DS3H_ODR_26HZ_VAL			0x02
#define ST_LSM6DS3H_ODR_52HZ_VAL			0x03
#define ST_LSM6DS3H_ODR_104HZ_VAL			0x04
#define ST_LSM6DS3H_ODR_208HZ_VAL			0x05
#define ST_LSM6DS3H_ODR_416HZ_VAL			0x06
#define ST_LSM6DS3H_FS_LIST_NUM				4
#define ST_LSM6DS3H_BDU_ADDR				0x12
#define ST_LSM6DS3H_BDU_MASK				0x40
#define ST_LSM6DS3H_EN_BIT				0x01
#define ST_LSM6DS3H_DIS_BIT				0x00
#define ST_LSM6DS3H_FUNC_EN_ADDR			0x19
#define ST_LSM6DS3H_FUNC_EN_MASK			0x04
#define ST_LSM6DS3H_FUNC_CFG_ACCESS_ADDR		0x01
#define ST_LSM6DS3H_FUNC_CFG_ACCESS_MASK		0x01
#define ST_LSM6DS3H_FUNC_CFG_ACCESS_MASK2		0x04
#define ST_LSM6DS3H_FUNC_CFG_REG2_MASK			0x80
#define ST_LSM6DS3H_FUNC_CFG_START1_ADDR		0x62
#define ST_LSM6DS3H_FUNC_CFG_START2_ADDR		0x63
#define ST_LSM6DS3H_FUNC_CFG_DATA_WRITE_ADDR		0x64
#define ST_LSM6DS3H_FUNC_CFG_DATA_READ_ADDR		0x65
#define ST_LSM6DS3H_SENSORHUB_ADDR			0x1a
#define ST_LSM6DS3H_SENSORHUB_MASK			0x01
#define ST_LSM6DS3H_SENSORHUB_TRIG_MASK			0x10
#define ST_LSM6DS3H_TRIG_INTERNAL			0x00
#define ST_LSM6DS3H_TRIG_EXTERNAL			0x01
#define ST_LSM6DS3H_SELFTEST_ADDR			0x14
#define ST_LSM6DS3H_SELFTEST_ACCEL_MASK			0x03
#define ST_LSM6DS3H_SELFTEST_GYRO_MASK			0x0c
#define ST_LSM6DS3H_SELF_TEST_DISABLED_VAL		0x00
#define ST_LSM6DS3H_SELF_TEST_POS_SIGN_VAL		0x01
#define ST_LSM6DS3H_SELF_TEST_NEG_ACCEL_SIGN_VAL	0x02
#define ST_LSM6DS3H_SELF_TEST_NEG_GYRO_SIGN_VAL		0x03
#define ST_LSM6DS3H_LIR_ADDR				0x58
#define ST_LSM6DS3H_LIR_MASK				0x01
#define ST_LSM6DS3H_TIMER_EN_ADDR			0x58
#define ST_LSM6DS3H_TIMER_EN_MASK			0x80
#define ST_LSM6DS3H_PEDOMETER_EN_ADDR			0x58
#define ST_LSM6DS3H_PEDOMETER_EN_MASK			0x40
#define ST_LSM6DS3H_INT2_ON_INT1_ADDR			0x13
#define ST_LSM6DS3H_INT2_ON_INT1_MASK			0x20
#define ST_LSM6DS3H_MIN_DURATION_MS			1638
#define ST_LSM6DS3H_XL_HM_MODE_ADDR			0x15
#define ST_LSM6DS3H_XL_HM_MODE_MASK			0x01
#define ST_LSM6DS3H_XL_HM_MODE_DISABLE			0x01
#define ST_LSM6DS3H_ROUNDING_ADDR			0x16
#define ST_LSM6DS3H_G_HM_MODE_DISABLE			0x80
#define ST_LSM6DS3H_ROUNDING_MASK			0x84
#define ST_LSM6DS3H_FIFO_MODE_ADDR			0x0a
#define ST_LSM6DS3H_FIFO_MODE_MASK			0x07
#define ST_LSM6DS3H_FIFO_MODE_BYPASS			0x00
#define ST_LSM6DS3H_FIFO_MODE_CONTINUOS			0x06
#define ST_LSM6DS3H_FIFO_THRESHOLD_IRQ_MASK		0x08
#define ST_LSM6DS3H_FIFO_ODR_MAX			0x40
#define ST_LSM6DS3H_FIFO_DECIMATOR_ADDR			0x08
#define ST_LSM6DS3H_FIFO_ACCEL_DECIMATOR_MASK		0x07
#define ST_LSM6DS3H_FIFO_GYRO_DECIMATOR_MASK		0x38
#define ST_LSM6DS3H_FIFO_DECIMATOR2_ADDR		0x09
#define ST_LSM6DS3H_FIFO_THR_L_ADDR			0x06
#define ST_LSM6DS3H_FIFO_THR_H_ADDR			0x07
#define ST_LSM6DS3H_FIFO_THR_MASK			0x0fff
#define ST_LSM6DS3H_FIFO_THR_IRQ_MASK			0x08
#define ST_LSM6DS3H_RESET_ADDR				0x12
#define ST_LSM6DS3H_RESET_MASK				0x01
#define ST_LSM6DS3H_TEST_REG_ADDR			0x00
#define ST_LSM6DS3H_START_INJECT_XL_MASK		0x08
#define ST_LSM6DS3H_INJECT_XL_X_ADDR			0x06
#define ST_LSM6DS3H_NS_AT_25HZ				40000000LL
#define ST_LSM6DS3H_26HZ_NS				(38461538LL)
#define ST_LSM6DS3H_SELFTEST_NA_MS			"na"
#define ST_LSM6DS3H_SELFTEST_FAIL_MS			"fail"
#define ST_LSM6DS3H_SELFTEST_PASS_MS			"pass"

/* VALUES TO UPLOAD FIRMWARE */
#define ST_LSM6DS3H_FIFO_CTRL4_ADDR			0x09
#define ST_LSM6DS3H_RESERVE_HALF_FIFO			0x80

/* CUSTOM VALUES FOR ACCEL SENSOR */
#define ST_LSM6DS3H_ACCEL_ODR_ADDR			0x10
#define ST_LSM6DS3H_ACCEL_ODR_MASK			0xf0
#define ST_LSM6DS3H_ACCEL_FS_ADDR			0x10
#define ST_LSM6DS3H_ACCEL_FS_MASK			0x0c
#define ST_LSM6DS3H_ACCEL_FS_2G_VAL			0x00
#define ST_LSM6DS3H_ACCEL_FS_4G_VAL			0x02
#define ST_LSM6DS3H_ACCEL_FS_8G_VAL			0x03
#define ST_LSM6DS3H_ACCEL_FS_16G_VAL			0x01
#define ST_LSM6DS3H_ACCEL_FS_2G_SENSITIVITY		61	/*  ug/LSB */
#define ST_LSM6DS3H_ACCEL_FS_4G_SENSITIVITY		122	/*  ug/LSB */
#define ST_LSM6DS3H_ACCEL_FS_8G_SENSITIVITY		244	/*  ug/LSB */
#define ST_LSM6DS3H_ACCEL_FS_16G_SENSITIVITY		488	/*  ug/LSB */
#define ST_LSM6DS3H_ACCEL_FS_2G_GAIN			IIO_G_TO_M_S_2(ST_LSM6DS3H_ACCEL_FS_2G_SENSITIVITY)
#define ST_LSM6DS3H_ACCEL_FS_4G_GAIN			IIO_G_TO_M_S_2(ST_LSM6DS3H_ACCEL_FS_4G_SENSITIVITY)
#define ST_LSM6DS3H_ACCEL_FS_8G_GAIN			IIO_G_TO_M_S_2(ST_LSM6DS3H_ACCEL_FS_8G_SENSITIVITY)
#define ST_LSM6DS3H_ACCEL_FS_16G_GAIN			IIO_G_TO_M_S_2(ST_LSM6DS3H_ACCEL_FS_16G_SENSITIVITY)
#define ST_LSM6DS3H_ACCEL_OUT_X_L_ADDR			0x28
#define ST_LSM6DS3H_ACCEL_OUT_Y_L_ADDR			0x2a
#define ST_LSM6DS3H_ACCEL_OUT_Z_L_ADDR			0x2c
#define ST_LSM6DS3H_ACCEL_STD_52HZ			1
#define ST_LSM6DS3H_ACCEL_STD_104HZ			2
#define ST_LSM6DS3H_ACCEL_STD_208HZ			3
#define ST_LSM6DS3H_SELFTEST_ACCEL_ADDR			0x10
#define ST_LSM6DS3H_SELFTEST_ACCEL_REG_VALUE		0x40
#define ST_LSM6DS3H_SELFTEST_ACCEL_MIN			1492
#define ST_LSM6DS3H_SELFTEST_ACCEL_MAX			27868

/* CUSTOM VALUES FOR GYRO SENSOR */
#define ST_LSM6DS3H_GYRO_ODR_ADDR			0x11
#define ST_LSM6DS3H_GYRO_ODR_MASK			0xf0
#define ST_LSM6DS3H_GYRO_FS_ADDR			0x11
#define ST_LSM6DS3H_GYRO_FS_MASK			0x0c
#define ST_LSM6DS3H_GYRO_FS_245_VAL			0x00
#define ST_LSM6DS3H_GYRO_FS_500_VAL			0x01
#define ST_LSM6DS3H_GYRO_FS_1000_VAL			0x02
#define ST_LSM6DS3H_GYRO_FS_2000_VAL			0x03
#define ST_LSM6DS3H_GYRO_FS_125_SENSITIVITY		4375    /* mdps/LSB */
#define ST_LSM6DS3H_GYRO_FS_245_SENSITIVITY		8750	/* mdps/LSB */
#define ST_LSM6DS3H_GYRO_FS_500_SENSITIVITY		17500	/* mdps/LSB */
#define ST_LSM6DS3H_GYRO_FS_1000_SENSITIVITY		35000	/* mdps/LSB */
#define ST_LSM6DS3H_GYRO_FS_2000_SENSITIVITY		70000	/* mdps/LSB */
#define ST_LSM6DS3H_GYRO_FS_125_GAIN                	IIO_DEGREE_TO_RAD(ST_LSM6DS3H_GYRO_FS_125_SENSITIVITY)
#define ST_LSM6DS3H_GYRO_FS_245_GAIN			IIO_DEGREE_TO_RAD(ST_LSM6DS3H_GYRO_FS_245_SENSITIVITY)
#define ST_LSM6DS3H_GYRO_FS_500_GAIN			IIO_DEGREE_TO_RAD(ST_LSM6DS3H_GYRO_FS_500_SENSITIVITY)
#define ST_LSM6DS3H_GYRO_FS_1000_GAIN			IIO_DEGREE_TO_RAD(ST_LSM6DS3H_GYRO_FS_1000_SENSITIVITY)
#define ST_LSM6DS3H_GYRO_FS_2000_GAIN			IIO_DEGREE_TO_RAD(ST_LSM6DS3H_GYRO_FS_2000_SENSITIVITY)
#define ST_LSM6DS3H_GYRO_OUT_X_L_ADDR			0x22
#define ST_LSM6DS3H_GYRO_OUT_Y_L_ADDR			0x24
#define ST_LSM6DS3H_GYRO_OUT_Z_L_ADDR			0x26
#define ST_LSM6DS3H_GYRO_STD_13HZ			2
#define ST_LSM6DS3H_GYRO_STD_52HZ			3
#define ST_LSM6DS3H_GYRO_STD_104HZ			5
#define ST_LSM6DS3H_GYRO_STD_208HZ			8
#define ST_LSM6DS3H_SELFTEST_GYRO_ADDR			0x11
#define ST_LSM6DS3H_SELFTEST_GYRO_REG_VALUE		0x4c
#define ST_LSM6DS3H_SELFTEST_GYRO_MIN			2142
#define ST_LSM6DS3H_SELFTEST_GYRO_MAX			10000
#define CALIBRATE_ODR_SET_VALUE					0X1C
/* CUSTOM VALUES FOR SIGNIFICANT MOTION SENSOR */
#define ST_LSM6DS3H_SIGN_MOTION_EN_ADDR			0x19
#define ST_LSM6DS3H_SIGN_MOTION_EN_MASK			0x01
#define ST_LSM6DS3H_SIGN_MOTION_DRDY_IRQ_MASK		0x40

/* CUSTOM VALUES FOR STEP DETECTOR SENSOR */
#define ST_LSM6DS3H_STEP_DETECTOR_DRDY_IRQ_MASK		0x80

/* CUSTOM VALUES FOR STEP COUNTER SENSOR */
#define ST_LSM6DS3H_STEP_COUNTER_DRDY_IRQ_MASK		0x80
#define ST_LSM6DS3H_STEP_COUNTER_OUT_L_ADDR		0x4b
#define ST_LSM6DS3H_STEP_COUNTER_RES_ADDR		0x19
#define ST_LSM6DS3H_STEP_COUNTER_RES_MASK		0x06
#define ST_LSM6DS3H_STEP_COUNTER_RES_ALL_EN		0x03
#define ST_LSM6DS3H_STEP_COUNTER_RES_FUNC_EN		0x02
#define ST_LSM6DS3H_STEP_COUNTER_DURATION_ADDR		0x15

/* CUSTOM VALUES FOR TILT SENSOR */
#define ST_LSM6DS3H_TILT_EN_ADDR			0x58
#define ST_LSM6DS3H_TILT_EN_MASK			0x20
#define ST_LSM6DS3H_TILT_DRDY_IRQ_MASK			0x02

/* CUSTOM VALUES FOR WRIST TILT SENSOR */
#define ST_LSM6DS3H_WRIST_TILT_EN_ADDR			0x11
#define ST_LSM6DS3H_WRIST_TILT_EN_MASK			0x01
#define ST_LSM6DS3H_WRIST_TILT_DRDY_IRQ_MASK		0x01
#define ST_LSM6DS3H_WRIST_TILT_FILTER_ADDR		0x27
#define ST_LSM6DS3H_WRIST_TILT_LATENCY_ADDR		0x28
#define ST_LSM6DS3H_WRIST_TILT_THS1_ADDR		0x29
#define ST_LSM6DS3H_WRIST_TILT_AXES_ADDR		0x2b
#define ST_LSM6DS3H_WRIST_TILT_AXES_XPOS_EN		0x80
#define ST_LSM6DS3H_WRIST_TILT_AXES_XNEG_EN		0x40
#define ST_LSM6DS3H_WRIST_TILT_AXES_YPOS_EN		0x20
#define ST_LSM6DS3H_WRIST_TILT_AXES_YNEG_EN		0x10
#define ST_LSM6DS3H_WRIST_TILT_AXES_ZPOS_EN		0x08
#define ST_LSM6DS3H_WRIST_TILT_AXES_ZNEG_EN		0x04

#define	ST_LSM6DS3H_STEP_COUNTER_THS_MIN_MASK		0x1f
#define	ST_LSM6DS3H_STEP_COUNTER_THS_MIN_DEF_VAL		0x19
#define	ST_LSM6DS3H_STEP_COUNTER_PEDO_THS_ADDR		0x0f
#define	ST_LSM6DS3H_STEP_COUNTER_DEB_STEP_MASK		0x07
#define	ST_LSM6DS3H_STEP_COUNTER_DEB_STEP_DEF_VAL	0x07
#define	ST_LSM6DS3H_STEP_COUNTER_PEDO_DEB_ADDR		0x14


#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
/* CUSTOM VALUES FOR TAP_TAP SENSOR */
#define ST_LSM6DS3H_TAP_CFG_ADDR			0x58
#define ST_LSM6DS3H_TAP_CFG_TAP_X_EN_MASK		0x08
#define ST_LSM6DS3H_TAP_CFG_TAP_Y_EN_MASK		0x04
#define ST_LSM6DS3H_TAP_CFG_TAP_Z_EN_MASK		0x02
#define ST_LSM6DS3H_TAP_THS_6D_ADDR			0x59
#define ST_LSM6DS3H_TAP_THS_MASK			0x1F
#define ST_LSM6DS3H_TAP_THS_DEF_VAL			0x0C /* 750 mg */
#define ST_LSM6DS3H_WAKE_UP_THS_ADDR			0x5B
#define ST_LSM6DS3H_WAKE_UP_THS_SINGLE_DOUBLE_TAP_MASK	0x80
#define ST_LSM6DS3H_INT_DUR2_ADDR			0x5A
#define ST_LSM6DS3H_INT_DUR2_MASK			0xFF
#define ST_LSM6DS3H_INT_DUR2_DEF_VAL			0x7F /* Duration, Quiet and Shock time */
#define ST_LSM6DS3H_MD1_CFG_INT1_DOUBLE_TAP_MASK	0x08
#define ST_LSM6DS3H_TAP_TAP_EN_ADDR			ST_LSM6DS3H_TAP_CFG_ADDR
#define ST_LSM6DS3H_TAP_TAP_EN_MASK			0x0E
#define ST_LSM6DS3H_TAP_TAP_DRDY_IRQ_MASK		ST_LSM6DS3H_MD1_CFG_INT1_DOUBLE_TAP_MASK
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

/* STATUS REGISTER */
#define ST_LSM6DS3H_STAT_REG_ADDR			0x1e
#define ST_LSM6DS3H_XLDA_MASK				0x01
#define ST_LSM6DS3H_GDA_MASK				0x02

#define ST_LSM6DS3H_ACCEL_SUFFIX_NAME			"accel"
#define ST_LSM6DS3H_GYRO_SUFFIX_NAME			"gyro"
#define ST_LSM6DS3H_STEP_COUNTER_SUFFIX_NAME		"step_c"
#define ST_LSM6DS3H_STEP_DETECTOR_SUFFIX_NAME		"step_d"
#define ST_LSM6DS3H_SIGN_MOTION_SUFFIX_NAME		"sign_motion"
#define ST_LSM6DS3H_TILT_SUFFIX_NAME			"tilt"
#define ST_LSM6DS3H_WRIST_TILT_SUFFIX_NAME		"wrist"
#define ST_LSM6DS3H_TAP_TAP_SUFFIX_NAME			"tap_tap"
#define ST_LSM6DS3H_WAKEUP_SUFFIX_NAME			"wk"

#define DELAY_FOR_OUT_STABLE				200/* 200ms */
#define MAX_WHILE_COUNTER				150
#define SAMPLE_WAIT_DELAY				39

/* Macro for calibrate */
#define CALIBRATE_SAMPLE_COUNT				50
#define GRAVITY_ACCEL_LSB_2G				(1000000 / ST_LSM6DS3H_ACCEL_FS_2G_SENSITIVITY)
#define SIGN_X_A			1
#define SIGN_Y_A			1
#define SIGN_Z_A			1
#define SIGN_X_G			1
#define SIGN_Y_G			1
#define SIGN_Z_G			1
#define ST_LSM6DS3H_ACCEL_FS_2G_SENSITIVITY		61      /*  ug/LSB */
#define ST_LSM6DS3H_DEV_ATTR_SAMP_FREQ() \
		IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO, \
			st_lsm6ds3h_sysfs_get_sampling_frequency, \
			st_lsm6ds3h_sysfs_set_sampling_frequency)

#define ST_LSM6DS3H_DEV_ATTR_SAMP_FREQ_AVAIL() \
		IIO_DEV_ATTR_SAMP_FREQ_AVAIL( \
			st_lsm6ds3h_sysfs_sampling_frequency_avail)

#define ST_LSM6DS3H_DEV_ATTR_SCALE_AVAIL(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			st_lsm6ds3h_sysfs_scale_avail, NULL , 0);

u8 threshold_value = ST_LSM6DS3H_STEP_COUNTER_THS_MIN_DEF_VAL;
#define RETRY_COUNTER_LIMITATION 10

static bool stay_wake;
static int accel_cal_data[3], gyro_cal_data[3];
static struct st_lsm6ds3h_selftest_table {
	char *string_mode;
	u8 accel_value;
	u8 gyro_value;
	u8 gyro_mask;
} st_lsm6ds3h_selftest_table[] = {
	[0] = {
		.string_mode = "disabled",
		.accel_value = ST_LSM6DS3H_SELF_TEST_DISABLED_VAL,
		.gyro_value = ST_LSM6DS3H_SELF_TEST_DISABLED_VAL,
	},
	[1] = {
		.string_mode = "positive-sign",
		.accel_value = ST_LSM6DS3H_SELF_TEST_POS_SIGN_VAL,
		.gyro_value = ST_LSM6DS3H_SELF_TEST_POS_SIGN_VAL
	},
	[2] = {
		.string_mode = "negative-sign",
		.accel_value = ST_LSM6DS3H_SELF_TEST_NEG_ACCEL_SIGN_VAL,
		.gyro_value = ST_LSM6DS3H_SELF_TEST_NEG_GYRO_SIGN_VAL
	},
};

struct st_lsm6ds3h_odr_reg {
	unsigned int hz;
	u8 value;
};

static struct st_lsm6ds3h_odr_table {
	u8 addr[4];
	u8 mask[4];
	struct st_lsm6ds3h_odr_reg odr_avl[ST_LSM6DS3H_ODR_LIST_NUM];
} st_lsm6ds3h_odr_table = {
	.addr[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_ODR_ADDR,
	.mask[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_ODR_MASK,
	.addr[ST_MASK_ID_ACCEL_WK] = ST_LSM6DS3H_ACCEL_ODR_ADDR,
	.mask[ST_MASK_ID_ACCEL_WK] = ST_LSM6DS3H_ACCEL_ODR_MASK,
	.addr[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_ODR_ADDR,
	.mask[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_ODR_MASK,
	.addr[ST_MASK_ID_GYRO_WK] = ST_LSM6DS3H_GYRO_ODR_ADDR,
	.mask[ST_MASK_ID_GYRO_WK] = ST_LSM6DS3H_GYRO_ODR_MASK,
	.odr_avl[0] = { .hz = 13, .value = ST_LSM6DS3H_ODR_13HZ_VAL },
	.odr_avl[1] = { .hz = 26, .value = ST_LSM6DS3H_ODR_26HZ_VAL },
	.odr_avl[2] = { .hz = 52, .value = ST_LSM6DS3H_ODR_52HZ_VAL },
	.odr_avl[3] = { .hz = 104, .value = ST_LSM6DS3H_ODR_104HZ_VAL },
	.odr_avl[4] = { .hz = 208, .value = ST_LSM6DS3H_ODR_208HZ_VAL },
	.odr_avl[5] = { .hz = 416, .value = ST_LSM6DS3H_ODR_416HZ_VAL },
};

struct st_lsm6ds3h_fs_reg {
	unsigned int gain;
	u8 value;
};

static struct st_lsm6ds3h_fs_table {
	u8 addr;
	u8 mask;
	struct st_lsm6ds3h_fs_reg fs_avl[ST_LSM6DS3H_FS_LIST_NUM];
} st_lsm6ds3h_fs_table[ST_INDIO_DEV_NUM] = {
	[ST_MASK_ID_ACCEL] = {
		.addr = ST_LSM6DS3H_ACCEL_FS_ADDR,
		.mask = ST_LSM6DS3H_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3H_ACCEL_FS_2G_GAIN,
					.value = ST_LSM6DS3H_ACCEL_FS_2G_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3H_ACCEL_FS_4G_GAIN,
					.value = ST_LSM6DS3H_ACCEL_FS_4G_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3H_ACCEL_FS_8G_GAIN,
					.value = ST_LSM6DS3H_ACCEL_FS_8G_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3H_ACCEL_FS_16G_GAIN,
					.value = ST_LSM6DS3H_ACCEL_FS_16G_VAL },
	},
	[ST_MASK_ID_ACCEL_WK] = {
		.addr = ST_LSM6DS3H_ACCEL_FS_ADDR,
		.mask = ST_LSM6DS3H_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3H_ACCEL_FS_2G_GAIN,
					.value = ST_LSM6DS3H_ACCEL_FS_2G_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3H_ACCEL_FS_4G_GAIN,
					.value = ST_LSM6DS3H_ACCEL_FS_4G_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3H_ACCEL_FS_8G_GAIN,
					.value = ST_LSM6DS3H_ACCEL_FS_8G_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3H_ACCEL_FS_16G_GAIN,
					.value = ST_LSM6DS3H_ACCEL_FS_16G_VAL },
	},
	[ST_MASK_ID_GYRO] = {
		.addr = ST_LSM6DS3H_GYRO_FS_ADDR,
		.mask = ST_LSM6DS3H_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3H_GYRO_FS_245_GAIN,
					.value = ST_LSM6DS3H_GYRO_FS_245_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3H_GYRO_FS_500_GAIN,
					.value = ST_LSM6DS3H_GYRO_FS_500_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3H_GYRO_FS_1000_GAIN,
					.value = ST_LSM6DS3H_GYRO_FS_1000_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3H_GYRO_FS_2000_GAIN,
					.value = ST_LSM6DS3H_GYRO_FS_2000_VAL },
	},
	[ST_MASK_ID_GYRO_WK] = {
		.addr = ST_LSM6DS3H_GYRO_FS_ADDR,
		.mask = ST_LSM6DS3H_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3H_GYRO_FS_245_GAIN,
					.value = ST_LSM6DS3H_GYRO_FS_245_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3H_GYRO_FS_500_GAIN,
					.value = ST_LSM6DS3H_GYRO_FS_500_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3H_GYRO_FS_1000_GAIN,
					.value = ST_LSM6DS3H_GYRO_FS_1000_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3H_GYRO_FS_2000_GAIN,
					.value = ST_LSM6DS3H_GYRO_FS_2000_VAL },
	}
};

static const struct iio_event_spec st_lsm6ds3h_event_spec[] = {
	{}
};

static const struct iio_chan_spec st_lsm6ds3h_accel_ch[] = {
	ST_LSM6DS3H_LSM_CHANNELS(IIO_ACCEL, 1, 0, IIO_MOD_X, IIO_LE,
				16, 16, ST_LSM6DS3H_ACCEL_OUT_X_L_ADDR, 's'),
	ST_LSM6DS3H_LSM_CHANNELS(IIO_ACCEL, 1, 1, IIO_MOD_Y, IIO_LE,
				16, 16, ST_LSM6DS3H_ACCEL_OUT_Y_L_ADDR, 's'),
	ST_LSM6DS3H_LSM_CHANNELS(IIO_ACCEL, 1, 2, IIO_MOD_Z, IIO_LE,
				16, 16, ST_LSM6DS3H_ACCEL_OUT_Z_L_ADDR, 's'),
	ST_LSM6DS3H_FLUSH_CHANNEL(IIO_ACCEL),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct iio_chan_spec st_lsm6ds3h_gyro_ch[] = {
	ST_LSM6DS3H_LSM_CHANNELS(IIO_ANGL_VEL, 1, 0, IIO_MOD_X, IIO_LE,
				16, 16, ST_LSM6DS3H_GYRO_OUT_X_L_ADDR, 's'),
	ST_LSM6DS3H_LSM_CHANNELS(IIO_ANGL_VEL, 1, 1, IIO_MOD_Y, IIO_LE,
				16, 16, ST_LSM6DS3H_GYRO_OUT_Y_L_ADDR, 's'),
	ST_LSM6DS3H_LSM_CHANNELS(IIO_ANGL_VEL, 1, 2, IIO_MOD_Z, IIO_LE,
				16, 16, ST_LSM6DS3H_GYRO_OUT_Z_L_ADDR, 's'),
	ST_LSM6DS3H_FLUSH_CHANNEL(IIO_ANGL_VEL),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct iio_chan_spec st_lsm6ds3h_sign_motion_ch[] = {
	ST_LSM6DS3H_EVENT_CHANNEL_WITH_MASK(IIO_SIGN_MOTION, 0),
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static const struct iio_chan_spec st_lsm6ds3h_step_c_ch[] = {
	{
		.type = IIO_STEP_COUNTER,
		.modified = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.address = ST_LSM6DS3H_STEP_COUNTER_OUT_L_ADDR,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static const struct iio_chan_spec st_lsm6ds3h_step_d_ch[] = {
	ST_LSM6DS3H_EVENT_CHANNEL_WITH_MASK(IIO_STEP_DETECTOR, 0),
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static const struct iio_chan_spec st_lsm6ds3h_tilt_ch[] = {
	ST_LSM6DS3H_EVENT_CHANNEL_WITH_MASK(IIO_TILT, 0),
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
static const struct iio_chan_spec st_lsm6ds3h_wrist_tilt_ch[] = {
	ST_LSM6DS3H_EVENT_CHANNEL_WITH_MASK(IIO_WRIST_TILT_GESTURE, 0),
	IIO_CHAN_SOFT_TIMESTAMP(1)
};
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
static const struct iio_chan_spec st_lsm6ds3h_tap_tap_ch[] = {
	ST_LSM6DS3H_EVENT_CHANNEL_WITH_MASK(IIO_TAP_TAP, 0),
	IIO_CHAN_SOFT_TIMESTAMP(1)
};
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */


int st_lsm6ds3h_write_data_with_mask(struct lsm6ds3h_data *cdata,
				u8 reg_addr, u8 mask, u8 data, bool b_lock)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}
EXPORT_SYMBOL(st_lsm6ds3h_write_data_with_mask);

static inline int st_lsm6ds3h_enable_embedded_page_regs(struct lsm6ds3h_data *cdata, bool enable)
{
	u8 value = 0x00;

	if (enable)
		value = ST_LSM6DS3H_FUNC_CFG_REG2_MASK;

#ifndef CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED
	if (cdata->fifo2_algo_available)
		value |= ST_LSM6DS3H_FUNC_CFG_ACCESS_MASK2;
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED */

	return cdata->tf->write(cdata, ST_LSM6DS3H_FUNC_CFG_ACCESS_ADDR, 1, &value, false);
}

int st_lsm6ds3h_write_embedded_registers(struct lsm6ds3h_data *cdata,
					u8 reg_addr, u8 *data, int len)
{
	int err = 0, err2;
	int retry_counter = 0;

	mutex_lock(&cdata->bank_registers_lock);

	if (cdata->enable_digfunc_mask) {
		err = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_FUNC_EN_ADDR,
					ST_LSM6DS3H_FUNC_EN_MASK,
					ST_LSM6DS3H_DIS_BIT, false);
		if (err < 0) {
			mutex_unlock(&cdata->bank_registers_lock);
			return err;
		}
	}

	udelay(100);

	err = st_lsm6ds3h_enable_embedded_page_regs(cdata, true);
	if (err < 0)
		goto restore_digfunc;

	udelay(100);

	err = cdata->tf->write(cdata, reg_addr, len, data, false);
	if (err < 0)
		goto restore_bank_regs;

	err = st_lsm6ds3h_enable_embedded_page_regs(cdata, false);
	if (err < 0)
		goto restore_bank_regs;

	udelay(100);

	if (cdata->enable_digfunc_mask) {
		err = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_FUNC_EN_ADDR,
					ST_LSM6DS3H_FUNC_EN_MASK,
					ST_LSM6DS3H_EN_BIT, false);
		if (err < 0)
			goto restore_digfunc;
	}

	mutex_unlock(&cdata->bank_registers_lock);

	return 0;

restore_bank_regs:
	do {
		msleep(200);
		err2 = st_lsm6ds3h_enable_embedded_page_regs(cdata, false);
	} while (err2 < 0 && retry_counter++ < RETRY_COUNTER_LIMITATION);

restore_digfunc:
	if (!cdata->enable_digfunc_mask) {
		err2 = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_FUNC_EN_ADDR,
					ST_LSM6DS3H_FUNC_EN_MASK,
					ST_LSM6DS3H_EN_BIT, false);
	}

	mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static int lsm6ds3h_set_watermark(struct lsm6ds3h_data *cdata)
{
	int err;
	u8 reg_value = 0;
	u16 fifo_watermark;
	unsigned int fifo_len, sip = 0, min_pattern = UINT_MAX;
	u16 hwfifo_watermark_accel = 0, hwfifo_watermark_gyro = 0;

	if (cdata->accel_on || cdata->accel_wk_on) {
		if (cdata->accel_on) {
			if (cdata->accel_wk_on) {
				hwfifo_watermark_accel = MIN(cdata->hwfifo_watermark[ST_MASK_ID_ACCEL],
					cdata->hwfifo_watermark[ST_MASK_ID_ACCEL_WK]);
			} else {
				hwfifo_watermark_accel = cdata->hwfifo_watermark[ST_MASK_ID_ACCEL];
			}
		} else {
			hwfifo_watermark_accel = cdata->hwfifo_watermark[ST_MASK_ID_ACCEL_WK];
		}
	}

	if (cdata->gyro_on || cdata->gyro_wk_on) {
		if (cdata->gyro_on) {
			if (cdata->gyro_wk_on) {
				hwfifo_watermark_gyro = MIN(cdata->hwfifo_watermark[ST_MASK_ID_GYRO],
					cdata->hwfifo_watermark[ST_MASK_ID_GYRO_WK]);
			} else {
				hwfifo_watermark_gyro = cdata->hwfifo_watermark[ST_MASK_ID_GYRO];
			}
		} else {
			hwfifo_watermark_gyro = cdata->hwfifo_watermark[ST_MASK_ID_GYRO_WK];
		}
	}

	if (cdata->fifo_output[ST_MASK_ID_ACCEL].sip > 0) {
		sip += cdata->fifo_output[ST_MASK_ID_ACCEL].sip;
		min_pattern = MIN(min_pattern,
			hwfifo_watermark_accel /
			cdata->fifo_output[ST_MASK_ID_ACCEL].sip);
	}

	if (cdata->fifo_output[ST_MASK_ID_GYRO].sip > 0) {
		sip += cdata->fifo_output[ST_MASK_ID_GYRO].sip;
		min_pattern = MIN(min_pattern,
			hwfifo_watermark_gyro /
			cdata->fifo_output[ST_MASK_ID_GYRO].sip);
	}

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	if (cdata->fifo_output[ST_MASK_ID_EXT0].sip > 0) {
		sip += cdata->fifo_output[ST_MASK_ID_EXT0].sip;
		min_pattern = MIN(min_pattern,
			cdata->hwfifo_watermark[ST_MASK_ID_EXT0] /
			cdata->fifo_output[ST_MASK_ID_EXT0].sip);
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

	if (sip == 0)
		return 0;

	if (min_pattern == 0)
		min_pattern = 1;

	min_pattern = MIN(min_pattern, ((unsigned int)ST_LSM6DS3H_MAX_FIFO_THRESHOLD / sip));

	fifo_len = min_pattern * sip * ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE;
	fifo_watermark = (fifo_len / 2);

	if (fifo_watermark < (ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE / 2))
		fifo_watermark = ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE / 2;

	if (fifo_watermark != cdata->fifo_watermark) {
		err = cdata->tf->read(cdata, ST_LSM6DS3H_FIFO_THR_H_ADDR, 1, &reg_value, true);
		if (err < 0)
			return err;

		fifo_watermark = (fifo_watermark & ST_LSM6DS3H_FIFO_THR_MASK) |
					((reg_value & ~ST_LSM6DS3H_FIFO_THR_MASK) << 8);

		err = cdata->tf->write(cdata, ST_LSM6DS3H_FIFO_THR_L_ADDR, 2,
						(u8 *)&fifo_watermark, true);
		if (err < 0)
			return err;

		cdata->fifo_watermark = fifo_watermark;
	}

	return 0;
}

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
/*
 * Configure Threshold, Duration, Quiet and Shock time.
 */
static int lsm6ds3h_config_tap_tap(struct lsm6ds3h_data *cdata)
{
	int err;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_TAP_THS_6D_ADDR,
				ST_LSM6DS3H_TAP_THS_MASK,
				ST_LSM6DS3H_TAP_THS_DEF_VAL, false);
	if (err < 0)
		return err;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_INT_DUR2_ADDR,
				ST_LSM6DS3H_INT_DUR2_MASK,
				ST_LSM6DS3H_INT_DUR2_DEF_VAL, false);
	if (err < 0)
		return err;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_WAKE_UP_THS_ADDR,
				ST_LSM6DS3H_WAKE_UP_THS_SINGLE_DOUBLE_TAP_MASK,
				ST_LSM6DS3H_EN_BIT, false);
	if (err < 0)
		return err;

	return 0;
}

int lsm6ds3h_get_fifo_odr_value(struct lsm6ds3h_data *cdata)
{
	int i, fifo_odr = 0, odr_value = 0;

	fifo_odr = cdata->fifo_odr;

	if (fifo_odr <= 0)
		return ST_LSM6DS3H_ODR_POWER_OFF_VAL;

	for (i = 0; i < ST_LSM6DS3H_ODR_LIST_NUM; i++) {
		if (st_lsm6ds3h_odr_table.odr_avl[i].hz == fifo_odr)
			break;
	}
	if (i == ST_LSM6DS3H_ODR_LIST_NUM)
		return -EINVAL;

	odr_value = st_lsm6ds3h_odr_table.odr_avl[i].value;

	return odr_value;
}
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

int st_lsm6ds3h_set_fifo_mode(struct lsm6ds3h_data *cdata, enum fifo_mode fm)
{
	int err;
	int fifo_odr = 0;
	u8 reg_value;
	bool enable_fifo;
	struct timespec ts;

	switch (fm) {
	case BYPASS:
		reg_value = ST_LSM6DS3H_FIFO_MODE_BYPASS;
		enable_fifo = false;
		break;
	case CONTINUOS:
		if (cdata->sensors_enabled & BIT(ST_MASK_ID_TAP_TAP)) {
			fifo_odr = lsm6ds3h_get_fifo_odr_value(cdata);
			if (fifo_odr < 0)
				return -EINVAL;
			reg_value = (ST_LSM6DS3H_FIFO_MODE_CONTINUOS | (fifo_odr << 3));
		} else {
			reg_value = ST_LSM6DS3H_FIFO_MODE_CONTINUOS | ST_LSM6DS3H_FIFO_ODR_MAX;
		}
		enable_fifo = true;
		break;
	default:
		return -EINVAL;
	}

	err = cdata->tf->write(cdata, ST_LSM6DS3H_FIFO_MODE_ADDR, 1, &reg_value, true);
	if (err < 0)
		return err;

	if (enable_fifo) {
		get_monotonic_boottime(&ts);
		cdata->timestamp = cdata->last_timestamp = timespec_to_ns(&ts);
		cdata->fifo_output[ST_MASK_ID_GYRO].timestamp = 0;
		cdata->fifo_output[ST_MASK_ID_ACCEL].timestamp = 0;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
		cdata->fifo_output[ST_MASK_ID_EXT0].timestamp = 0;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
	}

	cdata->fifo_status = fm;

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3h_set_fifo_mode);

static int lsm6ds3h_write_decimators(struct lsm6ds3h_data *cdata,
							u8 decimators[3])
{
	int i;
	u8 value[3], decimators_reg[2];

	for (i = 0; i < 3; i++) {
		switch (decimators[i]) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			value[i] = decimators[i];
			break;
		case 8:
			value[i] = 0x05;
			break;
		case 16:
			value[i] = 0x06;
			break;
		case 32:
			value[i] = 0x07;
			break;
		default:
			return -EINVAL;
		}
	}

	decimators_reg[0] = value[0] | (value[1] << 3);
	decimators_reg[1] = value[2];

#ifndef CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED
	if (cdata->fifo2_algo_available)
		decimators_reg[1] |= ST_LSM6DS3H_RESERVE_HALF_FIFO;
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED */

	return cdata->tf->write(cdata, ST_LSM6DS3H_FIFO_DECIMATOR_ADDR,
			ARRAY_SIZE(decimators_reg), decimators_reg, true);
}

static bool lsm6ds3h_calculate_fifo_decimators(struct lsm6ds3h_data *cdata,
				u8 decimators[3], u8 samples_in_pattern[3],
				unsigned int new_v_odr[ST_INDIO_DEV_NUM + 1],
				unsigned int new_hw_odr[ST_INDIO_DEV_NUM + 1],
				int64_t new_deltatime[ST_INDIO_DEV_NUM + 1],
				short new_fifo_decimator[ST_INDIO_DEV_NUM + 1])
{
	unsigned int trigger_odr;
	u8 min_decimator, max_decimator = 0;
	u8 accel_decimator = 0, gyro_decimator = 0;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	u8 ext_decimator = 0;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

	trigger_odr = new_hw_odr[ST_MASK_ID_ACCEL];
	if (trigger_odr < new_hw_odr[ST_MASK_ID_GYRO])
		trigger_odr = new_hw_odr[ST_MASK_ID_GYRO];

	if ((cdata->sensors_use_fifo & BIT(ST_MASK_ID_ACCEL)) &&
			(new_v_odr[ST_MASK_ID_ACCEL] != 0) && (cdata->accel_on || cdata->accel_wk_on))
		accel_decimator = trigger_odr / new_v_odr[ST_MASK_ID_ACCEL];

	if ((cdata->sensors_use_fifo & BIT(ST_MASK_ID_GYRO)) &&
				(new_v_odr[ST_MASK_ID_GYRO] != 0) &&
					(new_hw_odr[ST_MASK_ID_GYRO] > 0))
		gyro_decimator = trigger_odr / new_v_odr[ST_MASK_ID_GYRO];

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	if ((cdata->sensors_use_fifo & BIT(ST_MASK_ID_EXT0)) &&
			(new_v_odr[ST_MASK_ID_EXT0] != 0) && cdata->magn_on)
		ext_decimator = trigger_odr / new_v_odr[ST_MASK_ID_EXT0];

	new_fifo_decimator[ST_MASK_ID_EXT0] = 1;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

	new_fifo_decimator[ST_MASK_ID_ACCEL] = 1;
	new_fifo_decimator[ST_MASK_ID_GYRO] = 1;

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	if ((accel_decimator != 0) || (gyro_decimator != 0) || (ext_decimator != 0)) {
		min_decimator = MIN_BNZ(MIN_BNZ(accel_decimator, gyro_decimator), ext_decimator);
		max_decimator = MAX(MAX(accel_decimator, gyro_decimator), ext_decimator);
#else
	if ((accel_decimator != 0) || (gyro_decimator != 0)) {
		min_decimator = MIN_BNZ(accel_decimator, gyro_decimator);
		max_decimator = MAX(accel_decimator, gyro_decimator);
#endif
		if (min_decimator != 1) {
			if ((accel_decimator / min_decimator) == 1) {
				accel_decimator = 1;
				new_fifo_decimator[ST_MASK_ID_ACCEL] = min_decimator;
			} else if ((gyro_decimator / min_decimator) == 1) {
				gyro_decimator = 1;
				new_fifo_decimator[ST_MASK_ID_GYRO] = min_decimator;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
			} else if ((ext_decimator / min_decimator) == 1) {
				ext_decimator = 1;
				new_fifo_decimator[ST_MASK_ID_EXT0] = min_decimator;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
			}
			min_decimator = 1;
		}
		if ((accel_decimator > 4) && (accel_decimator < 8)) {
			new_fifo_decimator[ST_MASK_ID_ACCEL] = accel_decimator - 3;
			accel_decimator = 4;
		} else if ((accel_decimator > 8) && (accel_decimator < 16)) {
			new_fifo_decimator[ST_MASK_ID_ACCEL] = accel_decimator - 7;
			accel_decimator = 8;
		}
		if ((gyro_decimator > 4) && (gyro_decimator < 8)) {
			new_fifo_decimator[ST_MASK_ID_GYRO] = gyro_decimator - 3;
			gyro_decimator = 4;
		} else if ((gyro_decimator > 8) && (gyro_decimator < 16)) {
			new_fifo_decimator[ST_MASK_ID_GYRO] = gyro_decimator - 7;
			gyro_decimator = 8;
		}
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
		if ((ext_decimator > 4) && (ext_decimator < 8)) {
			new_fifo_decimator[ST_MASK_ID_EXT0] = ext_decimator - 3;
			ext_decimator = 4;
		} else if ((ext_decimator > 8) && (ext_decimator < 16)) {
			new_fifo_decimator[ST_MASK_ID_EXT0] = ext_decimator - 7;
			ext_decimator = 8;
		}
		max_decimator = MAX(MAX(accel_decimator, gyro_decimator), ext_decimator);
#else
		max_decimator = MAX(accel_decimator, gyro_decimator);
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
	}

	decimators[0] = accel_decimator;
	if (accel_decimator > 0) {
		new_deltatime[ST_MASK_ID_ACCEL] = accel_decimator *
						(1000000000U / trigger_odr);
		samples_in_pattern[0] = max_decimator / accel_decimator;
	} else
		samples_in_pattern[0] = 0;

	decimators[1] = gyro_decimator;
	if (gyro_decimator > 0) {
		new_deltatime[ST_MASK_ID_GYRO] = gyro_decimator *
						(1000000000U / trigger_odr);
		samples_in_pattern[1] = max_decimator / gyro_decimator;
	} else
		samples_in_pattern[1] = 0;

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	decimators[2] = ext_decimator;
	if (ext_decimator > 0) {
		new_deltatime[ST_MASK_ID_EXT0] = ext_decimator *
						(1000000000U / trigger_odr);
		samples_in_pattern[2] = max_decimator / ext_decimator;
	} else
		samples_in_pattern[2] = 0;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	if ((accel_decimator == cdata->hwfifo_decimator[ST_MASK_ID_ACCEL]) &&
			(ext_decimator == cdata->hwfifo_decimator[ST_MASK_ID_EXT0]) &&
			(gyro_decimator == cdata->hwfifo_decimator[ST_MASK_ID_GYRO])) {
#else /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
	if ((accel_decimator == cdata->hwfifo_decimator[ST_MASK_ID_ACCEL]) &&
			(gyro_decimator == cdata->hwfifo_decimator[ST_MASK_ID_GYRO])) {
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		return false;
	}

	return true;
}

int st_lsm6ds3h_flush_work_fifo(struct lsm6ds3h_data *cdata,
				bool disable_irq_and_flush)
{
	int err;

	dev_dbg(cdata->dev, "st_lsm6ds3h_flush_work_fifo!\n");

	if (disable_irq_and_flush) {
		disable_irq(cdata->irq);
		st_lsm6ds3h_flush_workqueue(cdata);
	}

	mutex_lock(&cdata->fifo_lock);
	err = st_lsm6ds3h_read_fifo(cdata, READ_FIFO_IN_COF_FIFO);
	if (err < 0)
		dev_warn(cdata->dev,"Read_fifo error in st_lsm6ds3h_flush_work_fifo!\n");
	mutex_unlock(&cdata->fifo_lock);

	if (disable_irq_and_flush)
		enable_irq(cdata->irq);

	return err;
}

int st_lsm6ds3h_set_drdy_irq(struct lsm6ds3h_sensor_data *sdata, bool state)
{
	int err;
	u16 *irq_mask = NULL;
	u8 reg_addr, mask = 0, value;
	u16 tmp_irq_enable_fifo_mask, tmp_irq_enable_accel_ext_mask;

	if (state)
		value = ST_LSM6DS3H_EN_BIT;
	else
		value = ST_LSM6DS3H_DIS_BIT;

	tmp_irq_enable_fifo_mask =
			sdata->cdata->irq_enable_fifo_mask & ~sdata->sindex;
	tmp_irq_enable_accel_ext_mask =
			sdata->cdata->irq_enable_accel_ext_mask & ~sdata->sindex;

	switch (sdata->sindex) {
	case ST_MASK_ID_ACCEL:
	case ST_MASK_ID_ACCEL_WK:
		reg_addr = ST_LSM6DS3H_INT1_ADDR;

		if (sdata->cdata->hwfifo_enabled[ST_MASK_ID_ACCEL]) {
			if (tmp_irq_enable_fifo_mask == 0)
				mask = ST_LSM6DS3H_FIFO_THR_IRQ_MASK;

			irq_mask = &sdata->cdata->irq_enable_fifo_mask;
		} else {
			if (tmp_irq_enable_accel_ext_mask == 0)
				mask = ST_LSM6DS3H_ACCEL_DRDY_IRQ_MASK;

			irq_mask = &sdata->cdata->irq_enable_accel_ext_mask;
		}

		break;
	case ST_MASK_ID_GYRO:
	case ST_MASK_ID_GYRO_WK:
		reg_addr = ST_LSM6DS3H_INT1_ADDR;

		if (sdata->cdata->hwfifo_enabled[ST_MASK_ID_GYRO]) {
			if (tmp_irq_enable_fifo_mask == 0)
				mask = ST_LSM6DS3H_FIFO_THR_IRQ_MASK;

			irq_mask = &sdata->cdata->irq_enable_fifo_mask;
		} else
			mask = ST_LSM6DS3H_GYRO_DRDY_IRQ_MASK;

		break;
	case ST_MASK_ID_SIGN_MOTION:
		reg_addr = ST_LSM6DS3H_INT1_ADDR;
		mask = ST_LSM6DS3H_SIGN_MOTION_DRDY_IRQ_MASK;
		break;
	case ST_MASK_ID_STEP_COUNTER:
		reg_addr = ST_LSM6DS3H_INT2_ADDR;
		mask = ST_LSM6DS3H_STEP_COUNTER_DRDY_IRQ_MASK;
		break;
	case ST_MASK_ID_STEP_DETECTOR:
		reg_addr = ST_LSM6DS3H_INT1_ADDR;
		mask = ST_LSM6DS3H_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case ST_MASK_ID_TILT:
		reg_addr = ST_LSM6DS3H_MD1_ADDR;
		mask = ST_LSM6DS3H_TILT_DRDY_IRQ_MASK;
		break;
#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	case ST_MASK_ID_TAP_TAP:
		reg_addr = ST_LSM6DS3H_MD1_ADDR;
		mask = ST_LSM6DS3H_TAP_TAP_DRDY_IRQ_MASK;
		break;
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */
#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	case ST_MASK_ID_WRIST_TILT:
		reg_addr = ST_LSM6DS3H_MD2_ADDR;
		mask = ST_LSM6DS3H_WRIST_TILT_DRDY_IRQ_MASK;
		break;
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	case ST_MASK_ID_EXT0:
		reg_addr = ST_LSM6DS3H_INT1_ADDR;

		if (sdata->cdata->hwfifo_enabled[ST_MASK_ID_EXT0]) {
			if (tmp_irq_enable_fifo_mask == 0)
				mask = ST_LSM6DS3H_FIFO_THR_IRQ_MASK;

			irq_mask = &sdata->cdata->irq_enable_fifo_mask;
		} else {
			if (tmp_irq_enable_accel_ext_mask == 0)
				mask = ST_LSM6DS3H_ACCEL_DRDY_IRQ_MASK;

			irq_mask = &sdata->cdata->irq_enable_accel_ext_mask;
		}

		break;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
	default:
		return -EINVAL;
	}

	if (mask > 0) {
		err =  st_lsm6ds3h_write_data_with_mask(sdata->cdata,
						reg_addr, mask, value, true);
		if (err < 0)
			return err;
	}

	if (irq_mask != NULL) {
		if (state)
			*irq_mask |= BIT(sdata->sindex);
		else
			*irq_mask &= ~BIT(sdata->sindex);
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3h_set_drdy_irq);

static int st_lsm6ds3h_set_odr(struct lsm6ds3h_sensor_data *sdata,
						unsigned int odr, bool force)
{
	u8 reg_value;
	int err, i = 0;
	bool scan_odr = true, fifo_conf_changed;
	u8 fifo_decimator[3] = { 0 }, samples_in_pattern[3] = { 0 };
	unsigned int temp_v_odr[ST_INDIO_DEV_NUM + 1];
	unsigned int temp_hw_odr[ST_INDIO_DEV_NUM + 1];
	int64_t new_deltatime[ST_INDIO_DEV_NUM + 1] = { 0 };
	short new_fifo_decimator[ST_INDIO_DEV_NUM + 1] = { 0 };

	dev_dbg(sdata->cdata->dev, "st_lsm6ds3h_set_odr: index=%d, odr=%d, force=%d\n",
		sdata->sindex, odr, force);

	if (odr == 0) {
		if (force)
			scan_odr = false;
		else
			return -EINVAL;
	}

	if (scan_odr) {
		for (i = 0; i < ST_LSM6DS3H_ODR_LIST_NUM; i++) {
			if (st_lsm6ds3h_odr_table.odr_avl[i].hz == odr)
				break;
		}
		if (i == ST_LSM6DS3H_ODR_LIST_NUM)
			return -EINVAL;

		if (sdata->cdata->hw_odr[sdata->sindex] == st_lsm6ds3h_odr_table.odr_avl[i].hz)
			reg_value = 0xff;
		else
			reg_value = st_lsm6ds3h_odr_table.odr_avl[i].value;
	} else {
		reg_value = ST_LSM6DS3H_ODR_POWER_OFF_VAL;
	}

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	if (((sdata->sindex == ST_MASK_ID_ACCEL)
		|| (sdata->sindex == ST_MASK_ID_ACCEL_WK))
		&& (reg_value != 0xff))
	{
		if (sdata->cdata->sensors_enabled & BIT(ST_MASK_ID_TAP_TAP)) {
				reg_value = ST_LSM6DS3H_ODR_416HZ_VAL;
		}
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

	if (sdata->cdata->sensors_use_fifo > 0) {
		/* someone is using fifo */
		if ((sdata->sindex == ST_MASK_ID_ACCEL) ||
			(sdata->sindex == ST_MASK_ID_ACCEL_WK)) {
			temp_v_odr[ST_MASK_ID_ACCEL] = odr;
			temp_v_odr[ST_MASK_ID_GYRO] = sdata->cdata->v_odr[ST_MASK_ID_GYRO];
			temp_hw_odr[ST_MASK_ID_ACCEL] = odr;
			temp_hw_odr[ST_MASK_ID_GYRO] = sdata->cdata->hw_odr[ST_MASK_ID_GYRO];
		} else {
			temp_v_odr[ST_MASK_ID_GYRO] = odr;
			temp_v_odr[ST_MASK_ID_ACCEL] = sdata->cdata->v_odr[ST_MASK_ID_ACCEL];
			temp_hw_odr[ST_MASK_ID_GYRO] = odr;
			temp_hw_odr[ST_MASK_ID_ACCEL] = sdata->cdata->hw_odr[ST_MASK_ID_ACCEL];
		}

		sdata->cdata->fifo_odr = MAX(temp_hw_odr[ST_MASK_ID_ACCEL], temp_hw_odr[ST_MASK_ID_GYRO]);

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
		temp_v_odr[ST_MASK_ID_EXT0] = sdata->cdata->v_odr[ST_MASK_ID_EXT0];
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

		fifo_conf_changed = lsm6ds3h_calculate_fifo_decimators(sdata->cdata,
				fifo_decimator, samples_in_pattern, temp_v_odr,
				temp_hw_odr, new_deltatime, new_fifo_decimator);
		if (fifo_conf_changed) {
			/* FIFO configuration changed, needs to write new decimators */
			disable_irq(sdata->cdata->irq);

			if (sdata->cdata->fifo_status != BYPASS) {
				mutex_lock(&sdata->cdata->fifo_lock);
				st_lsm6ds3h_read_fifo(sdata->cdata, READ_FIFO_IN_COF_FIFO);
				mutex_unlock(&sdata->cdata->fifo_lock);

				err = st_lsm6ds3h_set_fifo_mode(sdata->cdata, BYPASS);
				if (err < 0)
					goto reenable_fifo_irq;
			}

			err = lsm6ds3h_write_decimators(sdata->cdata, fifo_decimator);
			if (err < 0)
				goto reenable_fifo_irq;

			if (reg_value != 0xff) {
				err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
						st_lsm6ds3h_odr_table.addr[sdata->sindex],
						st_lsm6ds3h_odr_table.mask[sdata->sindex],
						reg_value, true);
				if (err < 0)
					goto reenable_fifo_irq;

				if ((sdata->sindex == ST_MASK_ID_ACCEL) ||
					(sdata->sindex == ST_MASK_ID_ACCEL_WK)) {
					switch (temp_hw_odr[ST_MASK_ID_ACCEL]) {
					case 13:
					case 26:
					case 52:
						sdata->cdata->samples_to_discard[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_STD_52HZ;
						break;
					case 104:
						sdata->cdata->samples_to_discard[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_STD_104HZ;
						break;
					default:
						sdata->cdata->samples_to_discard[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_STD_208HZ;
						break;
					}
				}

				switch (temp_hw_odr[ST_MASK_ID_GYRO]) {
				case 13:
					sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_13HZ;
					break;
				case 26:
				case 52:
					sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_52HZ;
					break;
				case 104:
					sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_104HZ;
					break;
				default:
					sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_208HZ;
					break;
				}
			}

			sdata->cdata->hwfifo_decimator[ST_MASK_ID_ACCEL] = fifo_decimator[0];
			sdata->cdata->hwfifo_decimator[ST_MASK_ID_GYRO] = fifo_decimator[1];

			sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].deltatime = new_deltatime[ST_MASK_ID_ACCEL];
			sdata->cdata->fifo_output[ST_MASK_ID_GYRO].deltatime = new_deltatime[ST_MASK_ID_GYRO];

			sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].decimator = new_fifo_decimator[ST_MASK_ID_ACCEL];
			sdata->cdata->fifo_output[ST_MASK_ID_GYRO].decimator = new_fifo_decimator[ST_MASK_ID_GYRO];

			sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].num_samples = new_fifo_decimator[ST_MASK_ID_ACCEL] - 1;
			sdata->cdata->fifo_output[ST_MASK_ID_GYRO].num_samples = new_fifo_decimator[ST_MASK_ID_GYRO] - 1;

			sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].sip = samples_in_pattern[0];
			sdata->cdata->fifo_output[ST_MASK_ID_GYRO].sip = samples_in_pattern[1];

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
			sdata->cdata->hwfifo_decimator[ST_MASK_ID_EXT0] = fifo_decimator[2];
			sdata->cdata->fifo_output[ST_MASK_ID_EXT0].deltatime = new_deltatime[ST_MASK_ID_EXT0];
			sdata->cdata->fifo_output[ST_MASK_ID_EXT0].decimator = new_fifo_decimator[ST_MASK_ID_EXT0];
			sdata->cdata->fifo_output[ST_MASK_ID_EXT0].num_samples = new_fifo_decimator[ST_MASK_ID_EXT0] - 1;
			sdata->cdata->fifo_output[ST_MASK_ID_EXT0].sip = samples_in_pattern[2];
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
			sdata->cdata->byte_in_pattern = (sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].sip+sdata->cdata->fifo_output[ST_MASK_ID_GYRO].sip+sdata->cdata->fifo_output[ST_MASK_ID_EXT0].sip)*ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE;
#else
			sdata->cdata->byte_in_pattern = (sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].sip+sdata->cdata->fifo_output[ST_MASK_ID_GYRO].sip)*ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE;
#endif

			err = lsm6ds3h_set_watermark(sdata->cdata);
			if (err < 0)
				goto reenable_fifo_irq;

			if ((samples_in_pattern[0] > 0) || (samples_in_pattern[1] > 0) || (samples_in_pattern[2] > 0)) {
				err = st_lsm6ds3h_set_fifo_mode(sdata->cdata, CONTINUOS);
				if (err < 0)
					goto reenable_fifo_irq;
			}

			enable_irq(sdata->cdata->irq);
		} else {
			/* FIFO configuration not changed */

			if (reg_value == 0xff)
				return 0;

			disable_irq(sdata->cdata->irq);

			if (sdata->cdata->fifo_status != BYPASS) {
				mutex_lock(&sdata->cdata->fifo_lock);
				st_lsm6ds3h_read_fifo(sdata->cdata, READ_FIFO_IN_COF_FIFO);
				mutex_unlock(&sdata->cdata->fifo_lock);

				err = st_lsm6ds3h_set_fifo_mode(sdata->cdata, BYPASS);
				if (err < 0)
					goto reenable_fifo_irq;
			}

			err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
					st_lsm6ds3h_odr_table.addr[sdata->sindex],
					st_lsm6ds3h_odr_table.mask[sdata->sindex],
					reg_value, true);
			if (err < 0)
				goto reenable_fifo_irq;

			if ((sdata->sindex == ST_MASK_ID_ACCEL) ||
				(sdata->sindex == ST_MASK_ID_ACCEL_WK)) {
				switch (temp_hw_odr[ST_MASK_ID_ACCEL]) {
				case 13:
				case 26:
				case 52:
					sdata->cdata->samples_to_discard[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_STD_52HZ;
					break;
				case 104:
					sdata->cdata->samples_to_discard[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_STD_104HZ;
					break;
				default:
					sdata->cdata->samples_to_discard[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_STD_208HZ;
					break;
				}
			}

			switch (temp_hw_odr[ST_MASK_ID_GYRO]) {
			case 13:
				sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_13HZ;
				break;
			case 26:
			case 52:
				sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_52HZ;
				break;
			case 104:
				sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_104HZ;
				break;
			default:
				sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_208HZ;
				break;
			}

			sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].deltatime = new_deltatime[ST_MASK_ID_ACCEL];
			sdata->cdata->fifo_output[ST_MASK_ID_GYRO].deltatime = new_deltatime[ST_MASK_ID_GYRO];
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
			sdata->cdata->fifo_output[ST_MASK_ID_EXT0].deltatime = new_deltatime[ST_MASK_ID_EXT0];
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
			sdata->cdata->byte_in_pattern = (sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].sip+sdata->cdata->fifo_output[ST_MASK_ID_GYRO].sip+sdata->cdata->fifo_output[ST_MASK_ID_EXT0].sip)*ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE;
#else
			sdata->cdata->byte_in_pattern = (sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].sip+sdata->cdata->fifo_output[ST_MASK_ID_GYRO].sip)*ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE;
#endif

			if ((sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].sip > 0) ||
					(sdata->cdata->fifo_output[ST_MASK_ID_GYRO].sip > 0)
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
						|| (sdata->cdata->fifo_output[ST_MASK_ID_EXT0].sip > 0)
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
			) {
				err = st_lsm6ds3h_set_fifo_mode(sdata->cdata, CONTINUOS);
				if (err < 0)
					goto reenable_fifo_irq;
			}

			enable_irq(sdata->cdata->irq);
		}

		if (!force) {
			sdata->cdata->v_odr[sdata->sindex] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
			sdata->cdata->accel_odr_dependency[0] = sdata->cdata->v_odr[sdata->sindex];
		}

		switch (sdata->sindex) {
		case ST_MASK_ID_ACCEL:
		case ST_MASK_ID_ACCEL_WK:
			if (odr == 0)
				sdata->cdata->hw_odr[ST_MASK_ID_ACCEL] = sdata->cdata->hw_odr[ST_MASK_ID_ACCEL_WK] = 0;
			else
				sdata->cdata->hw_odr[ST_MASK_ID_ACCEL] = sdata->cdata->hw_odr[ST_MASK_ID_ACCEL_WK] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
			break;
		case ST_MASK_ID_GYRO:
		case ST_MASK_ID_GYRO_WK:
			if (odr == 0)
				sdata->cdata->hw_odr[ST_MASK_ID_GYRO] = sdata->cdata->hw_odr[ST_MASK_ID_GYRO_WK] = 0;
			else
				sdata->cdata->hw_odr[ST_MASK_ID_GYRO] = sdata->cdata->hw_odr[ST_MASK_ID_GYRO_WK] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
			break;
		default:
			if (odr == 0)
				sdata->cdata->hw_odr[sdata->sindex] = 0;
			else
				sdata->cdata->hw_odr[sdata->sindex] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
			break;
		}
	} else {
		/* no one is using FIFO */

		disable_irq(sdata->cdata->irq);

		if ((odr != 0) && (sdata->cdata->hw_odr[sdata->sindex] == st_lsm6ds3h_odr_table.odr_avl[i].hz)) {
			if ((sdata->sindex == ST_MASK_ID_ACCEL) ||
				(sdata->sindex == ST_MASK_ID_ACCEL_WK)) {
				sdata->cdata->nofifo_decimation[ST_MASK_ID_ACCEL].decimator =
					sdata->cdata->hw_odr[ST_MASK_ID_ACCEL] / sdata->cdata->v_odr[ST_MASK_ID_ACCEL];
				sdata->cdata->nofifo_decimation[ST_MASK_ID_ACCEL].num_samples =
					sdata->cdata->nofifo_decimation[ST_MASK_ID_ACCEL].decimator - 1;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
				sdata->cdata->nofifo_decimation[ST_MASK_ID_EXT0].decimator =
					sdata->cdata->hw_odr[ST_MASK_ID_ACCEL] / sdata->cdata->v_odr[ST_MASK_ID_EXT0];
				sdata->cdata->nofifo_decimation[ST_MASK_ID_EXT0].num_samples =
					sdata->cdata->nofifo_decimation[ST_MASK_ID_EXT0].decimator - 1;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
			}

			enable_irq(sdata->cdata->irq);

			return 0;
		}

		err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
				st_lsm6ds3h_odr_table.addr[sdata->sindex],
				st_lsm6ds3h_odr_table.mask[sdata->sindex],
				reg_value, true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);
			return err;
		}

		if (!force) {
			sdata->cdata->v_odr[sdata->sindex] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
			sdata->cdata->accel_odr_dependency[0] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
		}

		switch (sdata->sindex) {
		case ST_MASK_ID_ACCEL:
		case ST_MASK_ID_ACCEL_WK:
			if (odr == 0)
				sdata->cdata->hw_odr[ST_MASK_ID_ACCEL] = sdata->cdata->hw_odr[ST_MASK_ID_ACCEL_WK] = 0;
			else
				sdata->cdata->hw_odr[ST_MASK_ID_ACCEL] = sdata->cdata->hw_odr[ST_MASK_ID_ACCEL_WK] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
			break;
		case ST_MASK_ID_GYRO:
		case ST_MASK_ID_GYRO_WK:
			if (odr == 0)
				sdata->cdata->hw_odr[ST_MASK_ID_GYRO] = sdata->cdata->hw_odr[ST_MASK_ID_GYRO_WK] = 0;
			else
				sdata->cdata->hw_odr[ST_MASK_ID_GYRO] = sdata->cdata->hw_odr[ST_MASK_ID_GYRO_WK] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
			break;
		default:
			if (odr == 0)
				sdata->cdata->hw_odr[sdata->sindex] = 0;
			else
				sdata->cdata->hw_odr[sdata->sindex] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
			break;
		}

		if ((sdata->sindex == ST_MASK_ID_ACCEL) ||
			(sdata->sindex == ST_MASK_ID_ACCEL_WK)) {
			switch (sdata->cdata->hw_odr[sdata->sindex]) {
			case 13:
			case 26:
			case 52:
				sdata->cdata->samples_to_discard[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_STD_52HZ;
				break;
			case 104:
				sdata->cdata->samples_to_discard[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_STD_104HZ;
				break;
			default:
				sdata->cdata->samples_to_discard[ST_MASK_ID_ACCEL] = ST_LSM6DS3H_ACCEL_STD_208HZ;
				break;
			}
		}

		switch (sdata->cdata->hw_odr[ST_MASK_ID_GYRO]) {
		case 13:
			sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_13HZ;
			break;
		case 26:
		case 52:
			sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_52HZ;
			break;
		case 104:
			sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_104HZ;
			break;
		default:
			sdata->cdata->samples_to_discard[ST_MASK_ID_GYRO] = ST_LSM6DS3H_GYRO_STD_208HZ;
			break;
		}

		if ((sdata->sindex == ST_MASK_ID_ACCEL) ||
			(sdata->sindex == ST_MASK_ID_ACCEL_WK)) {
			if (sdata->cdata->hw_odr[sdata->sindex] > 0) {
				sdata->cdata->nofifo_decimation[ST_MASK_ID_ACCEL].decimator =
					sdata->cdata->hw_odr[ST_MASK_ID_ACCEL] / sdata->cdata->v_odr[ST_MASK_ID_ACCEL];
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
				sdata->cdata->nofifo_decimation[ST_MASK_ID_EXT0].decimator =
					sdata->cdata->hw_odr[ST_MASK_ID_ACCEL] / sdata->cdata->v_odr[ST_MASK_ID_EXT0];
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
			} else {
				sdata->cdata->nofifo_decimation[ST_MASK_ID_ACCEL].decimator = 1;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
				sdata->cdata->nofifo_decimation[ST_MASK_ID_EXT0].decimator = 1;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
			}

			sdata->cdata->nofifo_decimation[ST_MASK_ID_ACCEL].num_samples =
				sdata->cdata->nofifo_decimation[ST_MASK_ID_ACCEL].decimator - 1;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
			sdata->cdata->nofifo_decimation[ST_MASK_ID_EXT0].num_samples =
				sdata->cdata->nofifo_decimation[ST_MASK_ID_EXT0].decimator - 1;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
		}

		enable_irq(sdata->cdata->irq);
	}

	sdata->cdata->trigger_odr = sdata->cdata->hw_odr[0] > sdata->cdata->hw_odr[1] ? sdata->cdata->hw_odr[0] : sdata->cdata->hw_odr[1];

	return 0;

reenable_fifo_irq:
	enable_irq(sdata->cdata->irq);
	return err;
}

/*
 * Enable / disable accelerometer
 */
static int lsm6ds3h_enable_accel(struct lsm6ds3h_data *cdata, enum st_mask_id id, int min_odr, bool enable)
{
	int odr, err;
	struct lsm6ds3h_sensor_data *sdata_accel = iio_priv(cdata->indio_dev[ST_MASK_ID_ACCEL]);
	struct lsm6ds3h_sensor_data *sdata_accel_wk = iio_priv(cdata->indio_dev[ST_MASK_ID_ACCEL_WK]);

	switch (id) {
	case ST_MASK_ID_ACCEL:
		cdata->accel_odr_dependency[0] = min_odr;
		cdata->accel_on = enable;

		break;
	case ST_MASK_ID_ACCEL_WK:
		cdata->accel_odr_dependency[1] = min_odr;
		cdata->accel_wk_on = enable;

		break;
	case ST_MASK_ID_SENSOR_HUB:
		cdata->accel_odr_dependency[2] = min_odr;
		cdata->magn_on = enable;

		break;
	case ST_MASK_ID_DIGITAL_FUNC:
		cdata->accel_odr_dependency[3] = min_odr;
		break;
	default:
		return -EINVAL;
	}

	if (MAX(cdata->accel_odr_dependency[0], cdata->accel_odr_dependency[1]) > cdata->accel_odr_dependency[2])
		odr = MAX(cdata->accel_odr_dependency[0], cdata->accel_odr_dependency[1]);
	else
		odr = cdata->accel_odr_dependency[2];

	if (cdata->accel_odr_dependency[3] > odr)
		odr = cdata->accel_odr_dependency[3];

#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	if (cdata->injection_mode)
		return 0;
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */

	if (id == ST_MASK_ID_ACCEL_WK) {
		err = st_lsm6ds3h_set_odr(sdata_accel_wk, odr, true);
		if (err < 0)
			return err;
	} else {
		err = st_lsm6ds3h_set_odr(sdata_accel, odr, true);
		if (err < 0)
			return err;
	}
	return 0;
}

/*
 * Enable / disable digital func
 */
static int lsm6ds3h_enable_digital_func(struct lsm6ds3h_data *cdata,
					bool enable, enum st_mask_id id)
{
	int err;

	if (enable) {
		if (cdata->enable_digfunc_mask == 0) {
			err = lsm6ds3h_enable_accel(cdata,
						ST_MASK_ID_DIGITAL_FUNC, 104, enable);
			if (err < 0)
				return err;

			err = st_lsm6ds3h_write_data_with_mask(cdata,
						ST_LSM6DS3H_FUNC_EN_ADDR,
						ST_LSM6DS3H_FUNC_EN_MASK,
						ST_LSM6DS3H_EN_BIT, true);
			if (err < 0)
				return err;
		}
		cdata->enable_digfunc_mask |= BIT(id);
	} else {
		if ((cdata->enable_digfunc_mask & ~BIT(id)) == 0) {
			err = st_lsm6ds3h_write_data_with_mask(cdata,
						ST_LSM6DS3H_FUNC_EN_ADDR,
						ST_LSM6DS3H_FUNC_EN_MASK,
						ST_LSM6DS3H_DIS_BIT, true);
			if (err < 0)
				return err;

			err = lsm6ds3h_enable_accel(cdata,
						ST_MASK_ID_DIGITAL_FUNC, 0, enable);
			if (err < 0)
				return err;
		}
		cdata->enable_digfunc_mask &= ~BIT(id);

	}

	return 0;
}

/*
 * Enable / disable HW pedometer
 */
static int lsm6ds3h_enable_pedometer(struct lsm6ds3h_data *cdata,
					bool enable, enum st_mask_id id)
{
	int err;

	if (enable) {
		if (cdata->enable_pedometer_mask == 0) {
			err =  st_lsm6ds3h_write_data_with_mask(cdata,
						ST_LSM6DS3H_PEDOMETER_EN_ADDR,
						ST_LSM6DS3H_PEDOMETER_EN_MASK,
						ST_LSM6DS3H_EN_BIT, true);
			if (err < 0)
				return err;

			err = lsm6ds3h_enable_digital_func(cdata,
						true, ST_MASK_ID_HW_PEDOMETER);
			if (err < 0)
				return err;
		}
		/* timestamp for step detector should be updated on
		 * system resume. Otherwise, event will be considered invalid.
		 */
		cdata->timestamp = ktime_to_ns(ktime_get_boottime());
		cdata->enable_pedometer_mask |= BIT(id);
	} else {
		if ((cdata->enable_pedometer_mask & ~BIT(id)) == 0) {
			err =  st_lsm6ds3h_write_data_with_mask(cdata,
						ST_LSM6DS3H_PEDOMETER_EN_ADDR,
						ST_LSM6DS3H_PEDOMETER_EN_MASK,
						ST_LSM6DS3H_DIS_BIT, true);
			if (err < 0)
				return err;

			err = lsm6ds3h_enable_digital_func(cdata,
						false, ST_MASK_ID_HW_PEDOMETER);
			if (err < 0)
				return err;
		}
		cdata->enable_pedometer_mask &= ~BIT(id);
	}

	return 0;
}

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
static int st_lsm6ds3h_set_wrist_tilt_threshold(struct lsm6ds3h_data *cdata)
{
	int err;
	u8 default_tilt_angle = ST_LSM6DS3H_WRIST_TILT_THRESHOLD;

	err = st_lsm6ds3h_write_embedded_registers(cdata, ST_LSM6DS3H_WRIST_TILT_THS1_ADDR,
					&default_tilt_angle, 1);
	if (err < 0)
		return err;

	return 0;
}
#endif

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
int st_lsm6ds3h_enable_sensor_hub(struct lsm6ds3h_data *cdata,
					bool enable, enum st_mask_id id)
{
	int err;

	if (enable) {
		if (cdata->enable_sensorhub_mask == 0) {
			err = lsm6ds3h_enable_digital_func(cdata,
						true, ST_MASK_ID_SENSOR_HUB);
			if (err < 0)
				return err;

			err = lsm6ds3h_enable_accel(cdata, ST_MASK_ID_SENSOR_HUB,
						cdata->v_odr[ST_MASK_ID_EXT0], enable);
			if (err < 0)
				return err;

			err = st_lsm6ds3h_write_data_with_mask(cdata,
						ST_LSM6DS3H_SENSORHUB_ADDR,
						ST_LSM6DS3H_SENSORHUB_MASK,
						ST_LSM6DS3H_EN_BIT, true);
			if (err < 0)
				return err;

		} else
			err = lsm6ds3h_enable_accel(cdata, ST_MASK_ID_SENSOR_HUB,
						cdata->v_odr[ST_MASK_ID_EXT0], enable);

		cdata->enable_sensorhub_mask |= BIT(id);
	} else {
		if ((cdata->enable_sensorhub_mask & ~BIT(id)) == 0) {
			err = st_lsm6ds3h_write_data_with_mask(cdata,
						ST_LSM6DS3H_SENSORHUB_ADDR,
						ST_LSM6DS3H_SENSORHUB_MASK,
						ST_LSM6DS3H_DIS_BIT, true);
			if (err < 0)
				return err;

			err = lsm6ds3h_enable_accel(cdata,
						ST_MASK_ID_SENSOR_HUB, 0, enable);
			if (err < 0)
				return err;

			err = lsm6ds3h_enable_digital_func(cdata,
						false, ST_MASK_ID_SENSOR_HUB);
			if (err < 0)
				return err;
		} else
			err = lsm6ds3h_enable_accel(cdata, ST_MASK_ID_SENSOR_HUB,
						cdata->v_odr[ST_MASK_ID_EXT0], enable);

		cdata->enable_sensorhub_mask &= ~BIT(id);
	}

	return err < 0 ? err : 0;
}
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
/*
 * Enable tap_tap event in all directions (X,Y,Z) for testing.
 * In production code only one direction will be enabled.
 */
static int lsm6ds3h_enable_tap_tap(struct lsm6ds3h_data *cdata, bool enable)
{
	int err;

	if (enable) {
		err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_TAP_TAP_EN_ADDR,
				ST_LSM6DS3H_TAP_CFG_TAP_X_EN_MASK,
				ST_LSM6DS3H_EN_BIT, true);
		if (err < 0)
			return err;

		err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_TAP_TAP_EN_ADDR,
				ST_LSM6DS3H_TAP_CFG_TAP_Y_EN_MASK,
				ST_LSM6DS3H_EN_BIT, true);
		if (err < 0)
			return err;

		err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_TAP_TAP_EN_ADDR,
				ST_LSM6DS3H_TAP_CFG_TAP_Z_EN_MASK,
				ST_LSM6DS3H_EN_BIT, true);
		if (err < 0)
			return err;

		cdata->sensors_enabled |= BIT(ST_MASK_ID_TAP_TAP);
		err = lsm6ds3h_enable_accel(cdata, ST_MASK_ID_ACCEL,
				cdata->hw_odr[ST_MASK_ID_ACCEL], enable);
		if (err < 0)
			return err;
	} else {
		err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_TAP_TAP_EN_ADDR,
				ST_LSM6DS3H_TAP_TAP_EN_MASK,
				ST_LSM6DS3H_DIS_BIT, true);
		if (err < 0)
			return err;

		cdata->sensors_enabled &= ~BIT(ST_MASK_ID_TAP_TAP);
		err = lsm6ds3h_enable_accel(cdata, ST_MASK_ID_ACCEL,
				cdata->hw_odr[ST_MASK_ID_ACCEL], enable);
		if (err < 0)
			return err;
	}

	return 0;
}
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

int st_lsm6ds3h_set_enable(struct lsm6ds3h_sensor_data *sdata, bool enable)
{
	int err;
	u8 reg_value;
	int en_odr, dis_odr;

	dev_dbg(sdata->cdata->dev, "st_lsm6ds3h_set_enable: index=%d, enable=%d\n",
		sdata->sindex, enable);

	switch (sdata->sindex) {
	case ST_MASK_ID_ACCEL:
		if (sdata->cdata->sensors_enabled & (1 << ST_MASK_ID_ACCEL_WK)) {
			en_odr = MAX(sdata->cdata->v_odr[ST_MASK_ID_ACCEL],
							sdata->cdata->v_odr[ST_MASK_ID_ACCEL_WK]);
			dis_odr = sdata->cdata->v_odr[ST_MASK_ID_ACCEL_WK];
		} else {
			en_odr = sdata->cdata->v_odr[ST_MASK_ID_ACCEL];
			dis_odr = 0;
		}
		err = lsm6ds3h_enable_accel(sdata->cdata, ST_MASK_ID_ACCEL,
			enable ? en_odr : dis_odr, enable);
		if (err < 0)
			return err;

		break;
	case ST_MASK_ID_ACCEL_WK:
		if (sdata->cdata->sensors_enabled & (1 << ST_MASK_ID_ACCEL)) {
			en_odr = MAX(sdata->cdata->v_odr[ST_MASK_ID_ACCEL_WK],
							sdata->cdata->v_odr[ST_MASK_ID_ACCEL]);
			dis_odr = sdata->cdata->v_odr[ST_MASK_ID_ACCEL];
		} else {
			en_odr = sdata->cdata->v_odr[ST_MASK_ID_ACCEL_WK];
			dis_odr = 0;
		}
		err = lsm6ds3h_enable_accel(sdata->cdata, ST_MASK_ID_ACCEL_WK,
			enable ? en_odr : dis_odr, enable);
		if (err < 0)
			return err;

		if (enable)
			stay_wake = true;
		/* both A_WK and G_WK disabled, the sensor can be suspended */
		else if (!(sdata->cdata->sensors_enabled & (1 << ST_MASK_ID_GYRO_WK)))
			stay_wake = false;

		break;
	case ST_MASK_ID_GYRO:
		if (sdata->cdata->sensors_enabled & (1 << ST_MASK_ID_GYRO_WK)) {
			en_odr = MAX(sdata->cdata->v_odr[ST_MASK_ID_GYRO],
							sdata->cdata->v_odr[ST_MASK_ID_GYRO_WK]);
			dis_odr = sdata->cdata->v_odr[ST_MASK_ID_GYRO_WK];
		} else {
			en_odr = sdata->cdata->v_odr[ST_MASK_ID_GYRO];
			dis_odr = 0;
		}

		if (enable)
			sdata->cdata->gyro_on = true;
		else
			sdata->cdata->gyro_on = false;

		err = st_lsm6ds3h_set_odr(sdata, enable ?
			en_odr : dis_odr, true);
		if (err < 0)
			return err;

		break;
	case ST_MASK_ID_GYRO_WK:
		if (sdata->cdata->sensors_enabled & (1 << ST_MASK_ID_GYRO)) {
			en_odr = MAX(sdata->cdata->v_odr[ST_MASK_ID_GYRO_WK],
							sdata->cdata->v_odr[ST_MASK_ID_GYRO]);
			dis_odr = sdata->cdata->v_odr[ST_MASK_ID_GYRO];
		} else {
			en_odr = sdata->cdata->v_odr[ST_MASK_ID_GYRO_WK];
			dis_odr = 0;
		}

		if (enable)
			sdata->cdata->gyro_wk_on= true;
		else
			sdata->cdata->gyro_wk_on = false;

		err = st_lsm6ds3h_set_odr(sdata, enable ?
			en_odr : dis_odr, true);
		if (err < 0)
			return err;

		if (enable)
			stay_wake = true;
		/* both A_WK and G_WK disabled, the sensor can be suspended */
		else if (!(sdata->cdata->sensors_enabled & (1 << ST_MASK_ID_ACCEL_WK)))
			stay_wake = false;

		break;
	case ST_MASK_ID_SIGN_MOTION:
		if (enable)
			reg_value = ST_LSM6DS3H_EN_BIT;
		else
			reg_value = ST_LSM6DS3H_DIS_BIT;

		err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3H_SIGN_MOTION_EN_ADDR,
					ST_LSM6DS3H_SIGN_MOTION_EN_MASK,
					reg_value, true);
		if (err < 0)
			return err;

		err = lsm6ds3h_enable_pedometer(sdata->cdata,
						enable, ST_MASK_ID_SIGN_MOTION);
		if (err < 0)
			return err;

		break;
	case ST_MASK_ID_STEP_COUNTER:
		if (enable)
			reg_value = ST_LSM6DS3H_EN_BIT;
		else
			reg_value = ST_LSM6DS3H_DIS_BIT;

		err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3H_TIMER_EN_ADDR,
					ST_LSM6DS3H_TIMER_EN_MASK,
					reg_value, true);
		if (err < 0)
			return err;

		err = lsm6ds3h_enable_pedometer(sdata->cdata,
					enable, ST_MASK_ID_STEP_COUNTER);
		if (err < 0)
			return err;

		break;
	case ST_MASK_ID_STEP_DETECTOR:
		err = lsm6ds3h_enable_pedometer(sdata->cdata,
					enable, ST_MASK_ID_STEP_DETECTOR);
		if (err < 0)
			return err;

		break;
	case ST_MASK_ID_TILT:
		if (enable)
			reg_value = ST_LSM6DS3H_EN_BIT;
		else
			reg_value = ST_LSM6DS3H_DIS_BIT;

		err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3H_TILT_EN_ADDR,
					ST_LSM6DS3H_TILT_EN_MASK,
					reg_value, true);
		if (err < 0)
			return err;

		err = lsm6ds3h_enable_digital_func(sdata->cdata,
						enable, ST_MASK_ID_TILT);
		if (err < 0)
			return err;

		break;
	case ST_MASK_ID_WRIST_TILT:
		if (enable)
			reg_value = ST_LSM6DS3H_EN_BIT;
		else
			reg_value = ST_LSM6DS3H_DIS_BIT;

		err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3H_WRIST_TILT_EN_ADDR,
					ST_LSM6DS3H_WRIST_TILT_EN_MASK,
					reg_value, true);
		if (err < 0)
			return err;

		err = lsm6ds3h_enable_digital_func(sdata->cdata,
						enable, ST_MASK_ID_WRIST_TILT);
		if (err < 0)
			return err;

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
		if (enable) {
			err = st_lsm6ds3h_set_wrist_tilt_threshold(sdata->cdata);
			if (err < 0)
				return err;
		}
#endif
		break;
#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	case ST_MASK_ID_TAP_TAP:
		err = lsm6ds3h_enable_tap_tap(sdata->cdata, enable);
		if (err < 0)
			return err;

		break;
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	case ST_MASK_ID_EXT0:
		break;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
	default:
		return -EINVAL;
	}

	err = st_lsm6ds3h_set_drdy_irq(sdata, enable);
	if (err < 0)
		return err;

	if (enable)
		sdata->cdata->sensors_enabled |= BIT(sdata->sindex);
	else
		sdata->cdata->sensors_enabled &= ~BIT(sdata->sindex);

	return 0;
}

static int st_lsm6ds3h_set_fs(struct lsm6ds3h_sensor_data *sdata,
							unsigned int gain)
{
	int err, i;

	for (i = 0; i < ST_LSM6DS3H_FS_LIST_NUM; i++) {
		if (st_lsm6ds3h_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}
	if (i == ST_LSM6DS3H_FS_LIST_NUM)
		return -EINVAL;

	err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
			st_lsm6ds3h_fs_table[sdata->sindex].addr,
			st_lsm6ds3h_fs_table[sdata->sindex].mask,
			st_lsm6ds3h_fs_table[sdata->sindex].fs_avl[i].value,
			true);
	if (err < 0)
		return err;

	sdata->c_gain[0] = gain;

	return 0;
}

static int st_lsm6ds3h_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int *val,
							int *val2, long mask)
{
	int err;
	u8 outdata[ST_LSM6DS3H_BYTE_FOR_CHANNEL];
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if ((sdata->sindex == ST_MASK_ID_ACCEL) ||
			(sdata->sindex == ST_MASK_ID_ACCEL_WK))
			msleep(40);

		if ((sdata->sindex == ST_MASK_ID_GYRO) ||
			(sdata->sindex == ST_MASK_ID_GYRO_WK))
			msleep(120);

		err = sdata->cdata->tf->read(sdata->cdata, ch->address,
				ST_LSM6DS3H_BYTE_FOR_CHANNEL, outdata, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		mutex_unlock(&indio_dev->mlock);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		if ((sdata->sindex == ST_MASK_ID_ACCEL) ||
			(sdata->sindex == ST_MASK_ID_ACCEL_WK)) {
			*val = accel_cal_data[ch->scan_index];
			return IIO_VAL_INT;
		}
		if ((sdata->sindex == ST_MASK_ID_GYRO) ||
			(sdata->sindex == ST_MASK_ID_GYRO_WK))	{
			*val = gyro_cal_data[ch->scan_index];
			return IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sdata->c_gain[0];
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int st_lsm6ds3h_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err;
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = st_lsm6ds3h_set_fs(sdata, val2);
		mutex_unlock(&indio_dev->mlock);
		break;
	case IIO_CHAN_INFO_OFFSET:
		err = 0;
		if ((sdata->sindex == ST_MASK_ID_ACCEL) ||
			(sdata->sindex == ST_MASK_ID_ACCEL_WK))
			accel_cal_data[chan->scan_index] = val;
		else if ((sdata->sindex == ST_MASK_ID_GYRO) ||
			(sdata->sindex == ST_MASK_ID_GYRO_WK))
			gyro_cal_data[chan->scan_index] = val;
		else
			err = -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	return err < 0 ? err : 0;
}

static int st_lsm6ds3h_reset_steps(struct lsm6ds3h_data *cdata)
{
	int err;
	u8 reg_value = 0x00;

	err = cdata->tf->read(cdata,
			ST_LSM6DS3H_STEP_COUNTER_RES_ADDR, 1, &reg_value, true);
	if (err < 0)
		return err;

	if (reg_value & ST_LSM6DS3H_FUNC_EN_MASK)
		reg_value = ST_LSM6DS3H_STEP_COUNTER_RES_FUNC_EN;
	else
		reg_value = ST_LSM6DS3H_DIS_BIT;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_STEP_COUNTER_RES_ADDR,
				ST_LSM6DS3H_STEP_COUNTER_RES_MASK,
				ST_LSM6DS3H_STEP_COUNTER_RES_ALL_EN, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_STEP_COUNTER_RES_ADDR,
				ST_LSM6DS3H_STEP_COUNTER_RES_MASK,
				reg_value, true);
	if (err < 0)
		return err;

	cdata->reset_steps = true;

	return 0;
}

static int st_lsm6ds3h_init_sensor(struct lsm6ds3h_data *cdata)
{
	int err;
	u8 default_reg_value = 0x00;

	/* Latch interrupts */
	err = st_lsm6ds3h_write_data_with_mask(cdata, ST_LSM6DS3H_LIR_ADDR,
				ST_LSM6DS3H_LIR_MASK, ST_LSM6DS3H_EN_BIT, true);
	if (err < 0)
		return err;

	/* Enable BDU for sensors data */
	err = st_lsm6ds3h_write_data_with_mask(cdata, ST_LSM6DS3H_BDU_ADDR,
				ST_LSM6DS3H_BDU_MASK, ST_LSM6DS3H_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_ROUNDING_ADDR,
					ST_LSM6DS3H_ROUNDING_MASK,
					ST_LSM6DS3H_G_HM_MODE_DISABLE | ST_LSM6DS3H_EN_BIT,
					true);
	if (err < 0)
		return err;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_XL_HM_MODE_ADDR,
					ST_LSM6DS3H_XL_HM_MODE_MASK,
					ST_LSM6DS3H_XL_HM_MODE_DISABLE,
					true);
	if (err < 0)
		return err;

	/* Redirect INT2 on INT1, all interrupt will be available on INT1 */
	err = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_INT2_ON_INT1_ADDR,
					ST_LSM6DS3H_INT2_ON_INT1_MASK,
					ST_LSM6DS3H_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3h_reset_steps(cdata);
	if (err < 0)
		return err;

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	err = lsm6ds3h_config_tap_tap(cdata);
	if (err < 0)
		return err;
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

	err = st_lsm6ds3h_write_embedded_registers(cdata,
					ST_LSM6DS3H_STEP_COUNTER_DURATION_ADDR,
					&default_reg_value, 1);
	if (err < 0)
		return err;

	mutex_lock(&cdata->bank_registers_lock);

	err = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3H_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3H_EN_BIT, false);
	if (err < 0)
		goto st_lsm6ds3h_init_sensor_mutex_unlock;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_STEP_COUNTER_PEDO_THS_ADDR,
					ST_LSM6DS3H_STEP_COUNTER_THS_MIN_MASK,
					ST_LSM6DS3H_STEP_COUNTER_THS_MIN_DEF_VAL, false);
	if (err < 0)
		goto st_lsm6ds3h_init_sensor_mutex_unlock;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_STEP_COUNTER_PEDO_DEB_ADDR,
					ST_LSM6DS3H_STEP_COUNTER_DEB_STEP_MASK,
					ST_LSM6DS3H_STEP_COUNTER_DEB_STEP_DEF_VAL, false);
	if (err < 0)
		goto st_lsm6ds3h_init_sensor_mutex_unlock;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3H_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3H_DIS_BIT, false);
	if (err < 0)
		goto st_lsm6ds3h_init_sensor_mutex_unlock;

	mutex_unlock(&cdata->bank_registers_lock);

	return 0;

st_lsm6ds3h_init_sensor_mutex_unlock:
	mutex_unlock(&cdata->bank_registers_lock);
	return err;
}

static int st_lsm6ds3h_set_selftest(struct lsm6ds3h_sensor_data *sdata, int index)
{
	u8 mode, mask;

	switch (sdata->sindex) {
	case ST_MASK_ID_ACCEL:
		mask = ST_LSM6DS3H_SELFTEST_ACCEL_MASK;
		mode = st_lsm6ds3h_selftest_table[index].accel_value;
		break;
	case ST_MASK_ID_GYRO:
		mask = ST_LSM6DS3H_SELFTEST_GYRO_MASK;
		mode = st_lsm6ds3h_selftest_table[index].gyro_value;
		break;
	default:
		return -EINVAL;
	}

	return st_lsm6ds3h_write_data_with_mask(sdata->cdata,
				ST_LSM6DS3H_SELFTEST_ADDR, mask, mode, true);
}

static ssize_t st_lsm6ds3h_sysfs_set_max_delivery_rate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 duration;
	int err;
	unsigned int max_delivery_rate;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtouint(buf, 10, &max_delivery_rate);
	if (err < 0)
		return -EINVAL;

	if (max_delivery_rate == sdata->cdata->v_odr[ST_MASK_ID_STEP_COUNTER])
		return size;

	duration = max_delivery_rate / ST_LSM6DS3H_MIN_DURATION_MS;

	err = st_lsm6ds3h_write_embedded_registers(sdata->cdata,
					ST_LSM6DS3H_STEP_COUNTER_DURATION_ADDR,
					&duration, 1);
	if (err < 0)
		return err;

	sdata->cdata->v_odr[ST_MASK_ID_STEP_COUNTER] = max_delivery_rate;

	return size;
}

static ssize_t st_lsm6ds3h_sysfs_get_max_delivery_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6ds3h_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n",
				sdata->cdata->v_odr[ST_MASK_ID_STEP_COUNTER]);
}

static ssize_t st_lsm6ds3h_sysfs_reset_counter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	struct lsm6ds3h_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	err = st_lsm6ds3h_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	return size;
}

static ssize_t st_lsm6ds3h_sysfs_get_sampling_frequency(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6ds3h_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->cdata->v_odr[sdata->sindex]);
}

static ssize_t st_lsm6ds3h_sysfs_set_sampling_frequency(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err, i;
	unsigned int odr;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;
	dev_dbg(sdata->cdata->dev, "st_lsm6ds3h_sysfs_set_sampling_frequency: odr=%d\n", odr);

	mutex_lock(&indio_dev->mlock);

	mutex_lock(&sdata->cdata->odr_lock);
#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	if (!((sdata->sindex & ST_MASK_ID_ACCEL) &&
					sdata->cdata->injection_mode)) {
		if (sdata->cdata->v_odr[sdata->sindex] != odr)
			err = st_lsm6ds3h_set_odr(sdata, odr, false);
	}
#else /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */
	if (sdata->cdata->v_odr[sdata->sindex] != odr) {
		for (i = 0; i < ST_LSM6DS3H_ODR_LIST_NUM; i++) {
			if (st_lsm6ds3h_odr_table.odr_avl[i].hz == odr)
				break;
		}
		if (i == ST_LSM6DS3H_ODR_LIST_NUM) {
			err = -EINVAL;
			goto sampling_freq_set_out;
		}

		sdata->cdata->v_odr[sdata->sindex] = st_lsm6ds3h_odr_table.odr_avl[i].hz;
		if ((sdata->cdata->sensors_enabled & BIT(sdata->sindex)) == 0) {
			err = size;
			goto sampling_freq_set_out;
		}
	}

	if (sdata->cdata->hw_odr[sdata->sindex] != odr) {
		switch (sdata->sindex) {
		case ST_MASK_ID_ACCEL:
			if (sdata->cdata->sensors_enabled & BIT(ST_MASK_ID_ACCEL)) {
				if (sdata->cdata->sensors_enabled & BIT(ST_MASK_ID_ACCEL_WK)) {
					if (odr > sdata->cdata->hw_odr[ST_MASK_ID_ACCEL])
						err = st_lsm6ds3h_set_odr(sdata, odr, false);
				} else
					err = st_lsm6ds3h_set_odr(sdata, odr, false);
			}
			break;
		case ST_MASK_ID_ACCEL_WK:
			if (sdata->cdata->sensors_enabled & BIT(ST_MASK_ID_ACCEL_WK)) {
				if (sdata->cdata->sensors_enabled & BIT(ST_MASK_ID_ACCEL)) {
					if (odr > sdata->cdata->hw_odr[ST_MASK_ID_ACCEL_WK])
						err = st_lsm6ds3h_set_odr(sdata, odr, false);
				} else
					err = st_lsm6ds3h_set_odr(sdata, odr, false);
			}
			break;
		case ST_MASK_ID_GYRO:
			if (sdata->cdata->sensors_enabled & BIT(ST_MASK_ID_GYRO)) {
				if (sdata->cdata->sensors_enabled & BIT(ST_MASK_ID_GYRO_WK)) {
					if (odr > sdata->cdata->hw_odr[ST_MASK_ID_GYRO])
						err = st_lsm6ds3h_set_odr(sdata, odr, false);
				} else
					err = st_lsm6ds3h_set_odr(sdata, odr, false);
			}
			break;
		case ST_MASK_ID_GYRO_WK:
			if (sdata->cdata->sensors_enabled & BIT(ST_MASK_ID_GYRO_WK)) {
				if (sdata->cdata->sensors_enabled & BIT(ST_MASK_ID_GYRO)) {
					if (odr > sdata->cdata->hw_odr[ST_MASK_ID_GYRO_WK])
						err = st_lsm6ds3h_set_odr(sdata, odr, false);
				} else
					err = st_lsm6ds3h_set_odr(sdata, odr, false);
			}
			break;
		default:
			err = st_lsm6ds3h_set_odr(sdata, odr, false);
			break;
		}
	}
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */
sampling_freq_set_out:
	mutex_unlock(&sdata->cdata->odr_lock);

	mutex_unlock(&indio_dev->mlock);

	return err < 0 ? err : size;
}

static ssize_t st_lsm6ds3h_sysfs_sampling_frequency_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 0; i < ST_LSM6DS3H_ODR_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
					st_lsm6ds3h_odr_table.odr_avl[i].hz);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6ds3h_sysfs_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	struct lsm6ds3h_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	for (i = 0; i < ST_LSM6DS3H_FS_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
			st_lsm6ds3h_fs_table[sdata->sindex].fs_avl[i].gain);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6ds3h_sysfs_get_selftest_available(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s, %s\n",
				st_lsm6ds3h_selftest_table[1].string_mode,
				st_lsm6ds3h_selftest_table[2].string_mode);
}

static ssize_t st_lsm6ds3h_sysfs_get_selftest_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t result;
	char *message;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	mutex_lock(&sdata->cdata->odr_lock);

	switch (sdata->sindex) {
	case ST_MASK_ID_ACCEL:
		result = sdata->cdata->accel_selftest_status;
		break;
	case ST_MASK_ID_GYRO:
		result = sdata->cdata->gyro_selftest_status;
		break;
	default:
		mutex_unlock(&sdata->cdata->odr_lock);
		return -EINVAL;
	}

	mutex_unlock(&sdata->cdata->odr_lock);

	if (result == 0)
		message = ST_LSM6DS3H_SELFTEST_NA_MS;
	else if (result < 0)
		message = ST_LSM6DS3H_SELFTEST_FAIL_MS;
	else if (result > 0)
		message = ST_LSM6DS3H_SELFTEST_PASS_MS;

	return sprintf(buf, "%s\n", message);
}

#if CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
int st_lsm6ds3h_wrist_tilt_conf(struct device *dev, u8 *value, char type)
{
	int err = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);
	struct lsm6ds3h_data *cdata = sdata->cdata;

	switch (type) {
	case 'l':
		err = st_lsm6ds3h_write_embedded_registers(cdata, ST_LSM6DS3H_WRIST_TILT_LATENCY_ADDR, value, 1);
		break;
	case 'a':
		err = st_lsm6ds3h_write_embedded_registers(cdata, ST_LSM6DS3H_WRIST_TILT_AXES_ADDR, value, 1);
		break;
	case 't':
		err = st_lsm6ds3h_write_embedded_registers(cdata, ST_LSM6DS3H_WRIST_TILT_THS1_ADDR, value, 1);
		break;
	case 'f':
		err = st_lsm6ds3h_write_embedded_registers(cdata, ST_LSM6DS3H_WRIST_TILT_FILTER_ADDR, value, 1);
		break;
	default:
		dev_err(dev, "Unknown command, failed to set embedded register for wrist tilt");
		err = -EINVAL;
	}

	return err;
}

static ssize_t st_lsm6ds3h_sysfs_set_wrist_tilt_latency(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	u8 latency_value;

	err = kstrtou8(buf, 10, &latency_value);
	if (err < 0)
		return -EINVAL;

	err = st_lsm6ds3h_wrist_tilt_conf(dev, &latency_value, 'l');
	if (err < 0)
		return err;

	return size;
}
static ssize_t st_lsm6ds3h_sysfs_set_wrist_tilt_axes(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	u8 axes_value;

	err = kstrtou8(buf, 10, &axes_value);
	if (err < 0)
		return -EINVAL;

	err = st_lsm6ds3h_wrist_tilt_conf(dev, &axes_value, 'a');
	if (err < 0)
		return err;

	return size;
}

static ssize_t st_lsm6ds3h_sysfs_set_wrist_tilt_threshold(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	u8 threshold_value;

	err = kstrtou8(buf, 10, &threshold_value);
	if (err < 0)
		return -EINVAL;

	err = st_lsm6ds3h_wrist_tilt_conf(dev, &threshold_value, 't');
	if (err < 0)
		return err;

	return size;
}

static ssize_t st_lsm6ds3h_sysfs_set_wrist_tilt_filter_timer(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	u8 filter_value;

	err = kstrtou8(buf, 10, &filter_value);
	if (err < 0)
		return -EINVAL;

	err = st_lsm6ds3h_wrist_tilt_conf(dev, &filter_value, 'f');
	if (err < 0)
		return err;

	return size;
}
#endif

static ssize_t st_lsm6ds3h_sysfs_start_selftest_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err, i, n;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);
	u8 reg_status, reg_addr, temp_reg_status, outdata[6];
	int x = 0, y = 0, z = 0, x_selftest = 0, y_selftest = 0, z_selftest = 0;

	mutex_lock(&sdata->cdata->odr_lock);

	switch (sdata->sindex) {
	case ST_MASK_ID_ACCEL:
		sdata->cdata->accel_selftest_status = 0;
		break;
	case ST_MASK_ID_GYRO:
		sdata->cdata->gyro_selftest_status = 0;
		break;
	default:
		mutex_unlock(&sdata->cdata->odr_lock);
		return -EINVAL;
	}

	if (sdata->cdata->sensors_enabled > 0) {
		mutex_unlock(&sdata->cdata->odr_lock);
		return -EBUSY;
	}

	for (n = 0; n < ARRAY_SIZE(st_lsm6ds3h_selftest_table); n++) {
		if (strncmp(buf, st_lsm6ds3h_selftest_table[n].string_mode,
								size - 2) == 0)
			break;
	}
	if (n == ARRAY_SIZE(st_lsm6ds3h_selftest_table)) {
		mutex_unlock(&sdata->cdata->odr_lock);
		return -EINVAL;
	}

	switch (sdata->sindex) {
	case ST_MASK_ID_ACCEL:
		reg_addr = ST_LSM6DS3H_SELFTEST_ACCEL_ADDR;
		temp_reg_status = ST_LSM6DS3H_SELFTEST_ACCEL_REG_VALUE;
		break;
	case ST_MASK_ID_GYRO:
		reg_addr = ST_LSM6DS3H_SELFTEST_GYRO_ADDR;
		temp_reg_status = ST_LSM6DS3H_SELFTEST_GYRO_REG_VALUE;
		break;
	default:
		mutex_unlock(&sdata->cdata->odr_lock);
		return -EINVAL;
	}

	err = sdata->cdata->tf->read(sdata->cdata,
					reg_addr, 1, &reg_status, true);
	if (err < 0) {
		mutex_unlock(&sdata->cdata->odr_lock);
		return err;
	}

	err = sdata->cdata->tf->write(sdata->cdata,
					reg_addr, 1, &temp_reg_status, false);
	if (err < 0) {
		mutex_unlock(&sdata->cdata->odr_lock);
		return err;
	}

	/* get data with selftest disabled */
	msleep(100);

	for (i = 0; i < 20; i++) {
		err = sdata->cdata->tf->read(sdata->cdata,
					sdata->data_out_reg, 6, outdata, true);
		if (err < 0) {
			i--;
			continue;
		}

		x += ((s16)*(u16 *)&outdata[0]) / 20;
		y += ((s16)*(u16 *)&outdata[2]) / 20;
		z += ((s16)*(u16 *)&outdata[4]) / 20;

		mdelay(10);
	}

	err = st_lsm6ds3h_set_selftest(sdata, n);
	if (err < 0) {
		mutex_unlock(&sdata->cdata->odr_lock);
		return err;
	}

	/* get data with selftest enabled */
	msleep(100);

	for (i = 0; i < 20; i++) {
		err = sdata->cdata->tf->read(sdata->cdata,
					sdata->data_out_reg, 6, outdata, true);
		if (err < 0) {
			i--;
			continue;
		}

		x_selftest += ((s16)*(u16 *)&outdata[0]) / 20;
		y_selftest += ((s16)*(u16 *)&outdata[2]) / 20;
		z_selftest += ((s16)*(u16 *)&outdata[4]) / 20;

		mdelay(10);
	}

	err = sdata->cdata->tf->write(sdata->cdata,
					reg_addr, 1, &reg_status, false);
	if (err < 0) {
		mutex_unlock(&sdata->cdata->odr_lock);
		return err;
	}

	err = st_lsm6ds3h_set_selftest(sdata, 0);
	if (err < 0) {
		mutex_unlock(&sdata->cdata->odr_lock);
		return err;
	}

	switch (sdata->sindex) {
	case ST_MASK_ID_ACCEL:
		if ((abs(x_selftest - x) < ST_LSM6DS3H_SELFTEST_ACCEL_MIN) ||
				(abs(x_selftest - x) > ST_LSM6DS3H_SELFTEST_ACCEL_MAX)) {
			sdata->cdata->accel_selftest_status = -1;
			goto selftest_failure;
		}

		if ((abs(y_selftest - y) < ST_LSM6DS3H_SELFTEST_ACCEL_MIN) ||
				(abs(y_selftest - y) > ST_LSM6DS3H_SELFTEST_ACCEL_MAX)) {
			sdata->cdata->accel_selftest_status = -1;
			goto selftest_failure;
		}

		if ((abs(z_selftest - z) < ST_LSM6DS3H_SELFTEST_ACCEL_MIN) ||
				(abs(z_selftest - z) > ST_LSM6DS3H_SELFTEST_ACCEL_MAX)) {
			sdata->cdata->accel_selftest_status = -1;
			goto selftest_failure;
		}

		sdata->cdata->accel_selftest_status = 1;
		break;
	case ST_MASK_ID_GYRO:
		if ((abs(x_selftest - x) < ST_LSM6DS3H_SELFTEST_GYRO_MIN) ||
				(abs(x_selftest - x) > ST_LSM6DS3H_SELFTEST_GYRO_MAX)) {
			sdata->cdata->gyro_selftest_status = -1;
			goto selftest_failure;
		}

		if ((abs(y_selftest - y) < ST_LSM6DS3H_SELFTEST_GYRO_MIN) ||
				(abs(y_selftest - y) > ST_LSM6DS3H_SELFTEST_GYRO_MAX)) {
			sdata->cdata->gyro_selftest_status = -1;
			goto selftest_failure;
		}

		if ((abs(z_selftest - z) < ST_LSM6DS3H_SELFTEST_GYRO_MIN) ||
				(abs(z_selftest - z) > ST_LSM6DS3H_SELFTEST_GYRO_MAX)) {
			sdata->cdata->gyro_selftest_status = -1;
			goto selftest_failure;
		}

		sdata->cdata->gyro_selftest_status = 1;
		break;
	default:
		mutex_unlock(&sdata->cdata->odr_lock);
		return -EINVAL;
	}

selftest_failure:
	mutex_unlock(&sdata->cdata->odr_lock);

	return size;
}

ssize_t st_lsm6ds3h_sysfs_flush_fifo(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u64 sensor_last_timestamp, event_type = 0;
	int stype = 0;
	int flags = READ_FIFO_IN_FLUSH;
	u64 timestamp_flush = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);
	int err;
	unsigned int sensor_handle;

	mutex_lock(&indio_dev->mlock);

	err = kstrtouint(buf, 10, &sensor_handle);
	if (err < 0) {
		mutex_unlock(&indio_dev->mlock);
		return -EINVAL;
	}
	dev_info(dev, "sensor_handle = %d", sensor_handle);

	if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
		mutex_lock(&sdata->cdata->odr_lock);
		disable_irq(sdata->cdata->irq);
	} else {
		mutex_unlock(&indio_dev->mlock);
		return -EINVAL;
	}

	if (sdata->sindex == ST_MASK_ID_ACCEL_WK)
		sensor_last_timestamp =
			sdata->cdata->fifo_output[ST_MASK_ID_ACCEL].timestamp_p;
	else if (sdata->sindex == ST_MASK_ID_GYRO_WK)
		sensor_last_timestamp =
			sdata->cdata->fifo_output[ST_MASK_ID_GYRO].timestamp_p;
	else
		sensor_last_timestamp =
			sdata->cdata->fifo_output[sdata->sindex].timestamp_p;

	mutex_lock(&sdata->cdata->fifo_lock);
	if (sdata->cdata->system_state & SF_RESUME)
		flags |= READ_FIFO_DISCARD_DATA;
	sdata->cdata->system_state = SF_NORMAL;
	st_lsm6ds3h_read_fifo(sdata->cdata, flags);
	mutex_unlock(&sdata->cdata->fifo_lock);

	if (sensor_last_timestamp ==
			sdata->cdata->fifo_output[sdata->sindex].timestamp_p)
		event_type = IIO_EV_DIR_FIFO_EMPTY;
	else
		event_type = IIO_EV_DIR_FIFO_DATA;

	timestamp_flush = sdata->cdata->fifo_output[sdata->sindex].timestamp_p;

	enable_irq(sdata->cdata->irq);

	switch (sdata->sindex) {
	case ST_MASK_ID_ACCEL:
	case ST_MASK_ID_ACCEL_WK:
		stype = IIO_ACCEL;
		break;

	case ST_MASK_ID_GYRO:
	case ST_MASK_ID_GYRO_WK:
		stype = IIO_ANGL_VEL;
		break;

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	case ST_MASK_ID_EXT0:
		stype = IIO_MAGN;
		break;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

	}

	iio_push_event(indio_dev, IIO_UNMOD_EVENT_CODE(stype,
				sensor_handle, IIO_EV_TYPE_FIFO_FLUSH, event_type),
				timestamp_flush);

	mutex_unlock(&sdata->cdata->odr_lock);
	mutex_unlock(&indio_dev->mlock);

	return size;
}

ssize_t st_lsm6ds3h_sysfs_get_hwfifo_enabled(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	return sprintf(buf, "%d\n",
				sdata->cdata->hwfifo_enabled[sdata->sindex]);
}

ssize_t st_lsm6ds3h_sysfs_set_hwfifo_enabled(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	bool enable = false;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
		err = -EBUSY;
		goto set_hwfifo_enabled_unlock_mutex;
	}

	err = strtobool(buf, &enable);
	if (err < 0)
		goto set_hwfifo_enabled_unlock_mutex;

	mutex_lock(&sdata->cdata->odr_lock);

	sdata->cdata->hwfifo_enabled[sdata->sindex] = enable;

	if (enable)
		sdata->cdata->sensors_use_fifo |= BIT(sdata->sindex);
	else
		sdata->cdata->sensors_use_fifo &= ~BIT(sdata->sindex);

	mutex_unlock(&sdata->cdata->odr_lock);
	mutex_unlock(&indio_dev->mlock);

	return size;

set_hwfifo_enabled_unlock_mutex:
	mutex_unlock(&indio_dev->mlock);
	return err;
}

ssize_t st_lsm6ds3h_sysfs_get_hwfifo_watermark(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	return sprintf(buf, "%d\n",
				sdata->cdata->hwfifo_watermark[sdata->sindex]);
}

ssize_t st_lsm6ds3h_sysfs_set_hwfifo_watermark(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0, watermark = 0, old_watermark;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &watermark);
	if (err < 0)
		return err;

	if ((watermark < 1) || (watermark > ST_LSM6DS3H_MAX_FIFO_LENGHT))
		return -EINVAL;

	mutex_lock(&sdata->cdata->odr_lock);

	if ((sdata->cdata->sensors_enabled & BIT(sdata->sindex)) &&
				(sdata->cdata->sensors_use_fifo & BIT(sdata->sindex))) {
		disable_irq(sdata->cdata->irq);

		if (sdata->cdata->fifo_status != BYPASS) {
			mutex_lock(&sdata->cdata->fifo_lock);
			st_lsm6ds3h_read_fifo(sdata->cdata, READ_FIFO_IN_COF_FIFO);
			mutex_unlock(&sdata->cdata->fifo_lock);
		}

		old_watermark = sdata->cdata->hwfifo_watermark[sdata->sindex];
		sdata->cdata->hwfifo_watermark[sdata->sindex] = watermark;

		err = lsm6ds3h_set_watermark(sdata->cdata);
		if (err < 0)
			sdata->cdata->hwfifo_watermark[sdata->sindex] = old_watermark;

		enable_irq(sdata->cdata->irq);
	} else
		sdata->cdata->hwfifo_watermark[sdata->sindex] = watermark;

	mutex_unlock(&sdata->cdata->odr_lock);

	return err < 0 ? err : size;
}

ssize_t st_lsm6ds3h_sysfs_get_hwfifo_watermark_max(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ST_LSM6DS3H_MAX_FIFO_LENGHT);
}

ssize_t st_lsm6ds3h_sysfs_get_hwfifo_watermark_min(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

static ssize_t st_lsm6ds3h_sysfs_set_pedo_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtou8(buf, 16, &threshold_value);
	if (err < 0)
		return -EINVAL;

	pr_info("pedo_threshold: threshold_value=0x%x", threshold_value);

	err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3H_STEP_COUNTER_PEDO_THS_ADDR,
					ST_LSM6DS3H_STEP_COUNTER_THS_MIN_MASK,
					threshold_value, false);
	if (err < 0)
		pr_info("pedo_threshold: err=%d", err);

	return size;
}

static ssize_t st_lsm6ds3h_sysfs_get_pedo_threshold(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", threshold_value);
}


#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
static ssize_t st_lsm6ds3h_sysfs_set_injection_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err, start;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);

	if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
		mutex_unlock(&indio_dev->mlock);
		return -EBUSY;
	}

	err = kstrtoint(buf, 10, &start);
	if (err < 0) {
		mutex_unlock(&indio_dev->mlock);
		return err;
	}

	mutex_lock(&sdata->cdata->odr_lock);

	if (start == 0) {
		hrtimer_cancel(&sdata->cdata->injection_timer);

		/* End injection */
		err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
				ST_LSM6DS3H_TEST_REG_ADDR,
				ST_LSM6DS3H_START_INJECT_XL_MASK, 0, true);
		if (err < 0) {
			mutex_unlock(&sdata->cdata->odr_lock);
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		 /* Force accel ODR to 26Hz if dependencies are enabled */
		 if (sdata->cdata->sensors_enabled > 0) {
			err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
				st_lsm6ds3h_odr_table.addr[sdata->sindex],
				st_lsm6ds3h_odr_table.mask[sdata->sindex],
				st_lsm6ds3h_odr_table.odr_avl[1].value, true);
			if (err < 0) {
				mutex_unlock(&sdata->cdata->odr_lock);
				mutex_unlock(&indio_dev->mlock);
				return err;
			}
		}

		sdata->cdata->injection_mode = false;
	} else {
		sdata->cdata->last_injection_timestamp = 0;
		sdata->cdata->injection_samples = 0;

		/* Force accel ODR to 26Hz */
		err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
				st_lsm6ds3h_odr_table.addr[sdata->sindex],
				st_lsm6ds3h_odr_table.mask[sdata->sindex],
				st_lsm6ds3h_odr_table.odr_avl[1].value, true);
		if (err < 0) {
			mutex_unlock(&sdata->cdata->odr_lock);
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		/* Set start injection */
		err = st_lsm6ds3h_write_data_with_mask(sdata->cdata,
				ST_LSM6DS3H_TEST_REG_ADDR,
				ST_LSM6DS3H_START_INJECT_XL_MASK, 1, true);
		if (err < 0) {
			mutex_unlock(&sdata->cdata->odr_lock);
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		sdata->cdata->injection_mode = true;
	}

	mutex_unlock(&sdata->cdata->odr_lock);
	mutex_unlock(&indio_dev->mlock);

	return size;
}

static ssize_t st_lsm6ds3h_sysfs_get_injection_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", sdata->cdata->injection_mode);
}

static ssize_t st_lsm6ds3h_sysfs_upload_xl_data(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int i;
	u8 sample[3];
	s64 timestamp;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);

	if (!sdata->cdata->injection_mode) {
		mutex_unlock(&indio_dev->mlock);
		return -EINVAL;
	}

	for (i = 0; i < 3; i++)
		sample[i] = *(s16 *)(&buf[i * 2]) >> 8;

	timestamp = *(s64 *)(buf + ALIGN(6, sizeof(s64)));

	if (timestamp < sdata->cdata->last_injection_timestamp +
						ST_LSM6DS3H_NS_AT_25HZ) {
		mutex_unlock(&indio_dev->mlock);
		return size;
	}

	while (sdata->cdata->injection_samples >= 10)
		msleep(200);

	spin_lock(&sdata->cdata->injection_spinlock);

	memcpy(&sdata->cdata->injection_data[
			sdata->cdata->injection_samples * 3], sample, 3);
	sdata->cdata->injection_samples++;

	spin_unlock(&sdata->cdata->injection_spinlock);

	sdata->cdata->last_injection_timestamp = timestamp;

	if (sdata->cdata->injection_samples >= 8)
		hrtimer_start(&sdata->cdata->injection_timer,
			ktime_set(0, ST_LSM6DS3H_26HZ_NS), HRTIMER_MODE_REL);

	mutex_unlock(&indio_dev->mlock);

	return size;
}

static void st_lsm6ds3h_injection_work(struct work_struct *work)
{
	int i, err;
	struct lsm6ds3h_data *cdata;

	cdata = container_of(work, struct lsm6ds3h_data, injection_work);

	if (cdata->injection_samples == 0)
		return;

	err = cdata->tf->write(cdata, ST_LSM6DS3H_INJECT_XL_X_ADDR,
					3, cdata->injection_data, false);
	if (err < 0)
		return;

	spin_lock(&cdata->injection_spinlock);

	for (i = 0; i < cdata->injection_samples - 1; i++)
		memcpy(&cdata->injection_data[i * 3],
				&cdata->injection_data[(i + 1) * 3], 3);

	cdata->injection_samples--;

	spin_unlock(&cdata->injection_spinlock);
}

static enum hrtimer_restart st_lsm6ds3h_injection_timer_func(
							struct hrtimer *timer)
{
	ktime_t now;
	struct lsm6ds3h_data *cdata;

	cdata = container_of(timer, struct lsm6ds3h_data, injection_timer);

	now = hrtimer_cb_get_time(timer);
	hrtimer_forward(timer, now, ktime_set(0, ST_LSM6DS3H_26HZ_NS));

	schedule_work(&cdata->injection_work);

	return HRTIMER_RESTART;
}

static ssize_t st_lsm6ds3h_sysfs_get_injection_sensors(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "lsm6ds3h_accel");
}
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */

int st_lsm6ds3h_average_sample(struct lsm6ds3h_sensor_data *sdata,
				s32 *out_data, int sample_count)
{
	int i, err = 0;
	u8 hw_data[ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE];
	struct lsm6ds3h_data *cdata = sdata->cdata;
	if (sample_count <= 0) {
		return -ERANGE;
	}
	msleep(DELAY_FOR_OUT_STABLE);

	/* first sample will be discarded */
	for (i = 0; i < (sample_count + 1); i++) {
		msleep(SAMPLE_WAIT_DELAY);
		/* Now data is ready */
		err = cdata->tf->read(cdata, cdata->indio_dev[sdata->sindex]->channels->address,
				ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE, hw_data, true);
		if (err < 0) {
			dev_err(cdata->dev, "failed to read out data.\n");
			return err;
		}
		dev_dbg(cdata->dev, "hw_data[x]=%x,hw_data[y]=%x, hw_data[z]=%x\n",
			(s16)((hw_data[1] << 8 | hw_data[0])),
			(s16)((hw_data[3] << 8 | hw_data[2])),
			(s16)((hw_data[5] << 8 | hw_data[4])));
		if (i != 0) {
			out_data[0] += (s16)((hw_data[1] << 8 | hw_data[0]));
			out_data[1] += (s16)((hw_data[3] << 8 | hw_data[2]));
			out_data[2] += (s16)((hw_data[5] << 8 | hw_data[4]));
		}
	}

	out_data[0] /= sample_count;
	out_data[1] /= sample_count;
	out_data[2] /= sample_count;
	dev_dbg(cdata->dev, "out_data[x]=%d, out_data[y]=%d, out_data[z]=%d\n", out_data[0], out_data[1], out_data[2]);

	return 0;
}


ssize_t st_lsm6ds3h_sysfs_do_calibrate(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	s32 no_cali[3] = {0};
	u8 calibrate_odr_reg = CALIBRATE_ODR_SET_VALUE;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	err = sdata->cdata->tf->write(sdata->cdata, ST_LSM6DS3H_GYRO_ODR_ADDR, 1, &calibrate_odr_reg, true);
	if (err < 0) {
		dev_err(sdata->cdata,"failed to write ST_LSM6DS3H_GYRO_ODR_ADDR\n");
	}
	err = st_lsm6ds3h_average_sample(sdata, no_cali, CALIBRATE_SAMPLE_COUNT);
	if (err < 0)
		return err;

	/* The 4th parameter used for data sum check */
	if (sdata->sindex == ST_MASK_ID_ACCEL)
		return sprintf(buf, "%d %d %d %d\n",
				0 * SIGN_X_A - no_cali[0],
				0 * SIGN_Y_A - no_cali[1],
				GRAVITY_ACCEL_LSB_2G * SIGN_Z_A - no_cali[2],
				0 * SIGN_X_A + 0 * SIGN_Y_A + GRAVITY_ACCEL_LSB_2G * SIGN_Z_A -
				no_cali[0] - no_cali[1] - no_cali[2]);
	else
		return sprintf(buf, "%d %d %d %d\n",
				0 * SIGN_X_G - no_cali[0],
				0 * SIGN_Y_G - no_cali[1],
				0 * SIGN_Z_G - no_cali[2],
				0 * SIGN_X_G + 0 * SIGN_Y_G + 0 * SIGN_Z_G -
				no_cali[0] - no_cali[1] - no_cali[2]);

}

static IIO_DEVICE_ATTR(do_calibrate, S_IRUGO,
				st_lsm6ds3h_sysfs_do_calibrate,
				NULL, 0);

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED
static inline int st_lsm6ds3h_upload_algo(struct lsm6ds3h_data *cdata)
{
	return 0;
}
#else /* CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED */
static int st_lsm6ds3h_upload_algo(struct lsm6ds3h_data *cdata)
{
	int err, err2, i;
	u8 data = 0x00, *fw_check_data;
	const struct firmware *fw;
#if (CONFIG_ST_LSM6DS3H_IIO_LIMIT_FIFO > 0)
	u16 address = 0;
	int remaining_byte, read_size = CONFIG_ST_LSM6DS3H_IIO_LIMIT_FIFO;
#endif /* CONFIG_ST_LSM6DS3H_IIO_LIMIT_FIFO */
	int retry_counter = 0;

	err = request_firmware(&fw, ST_LSM6DS3H_DATA_FW, cdata->dev);
	if (err < 0)
		return err;

	fw_check_data = kmalloc(fw->size, GFP_KERNEL);
	if (!fw_check_data) {
		err = -ENOMEM;
		goto release_firmware;
	}

	/* Stop current algo */
	err = cdata->tf->write(cdata,
			ST_LSM6DS3H_FUNC_CFG_ACCESS_ADDR, 1, &data, true);
	if (err < 0)
		goto free_fw_check_data;

	/* Reserve HALF FIFO for algo to be uploaded */
	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_FIFO_CTRL4_ADDR,
				ST_LSM6DS3H_RESERVE_HALF_FIFO,
				ST_LSM6DS3H_EN_BIT, true);
	if (err < 0)
		goto free_fw_check_data;

	mutex_lock(&cdata->bank_registers_lock);

	/* Start the upload algo procedure */
	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_FUNC_CFG_ACCESS_ADDR,
				ST_LSM6DS3H_FUNC_CFG_ACCESS_MASK,
				ST_LSM6DS3H_EN_BIT, false);
	if (err < 0)
		goto close_upload_procedure;

	/* Set upload start address (LSB) */
	err = cdata->tf->write(cdata,
			ST_LSM6DS3H_FUNC_CFG_START1_ADDR, 1, &data, false);
	if (err < 0)
		goto close_upload_procedure;

	/* Set upload start address (MSB) */
	err = cdata->tf->write(cdata,
			ST_LSM6DS3H_FUNC_CFG_START2_ADDR, 1, &data, false);
	if (err < 0)
		goto close_upload_procedure;

	/* upload algo */
	err = cdata->tf->write(cdata, ST_LSM6DS3H_FUNC_CFG_DATA_WRITE_ADDR,
					fw->size, (u8 *)fw->data, false);
	if (err < 0)
		goto close_upload_procedure;

#if (CONFIG_ST_LSM6DS3H_IIO_LIMIT_FIFO == 0)
	/* Set upload start address (LSB) */
	err = cdata->tf->write(cdata,
			ST_LSM6DS3H_FUNC_CFG_START1_ADDR, 1, &data, false);
	if (err < 0)
		goto close_upload_procedure;

	/* Set upload start address (MSB) */
	err = cdata->tf->write(cdata,
			ST_LSM6DS3H_FUNC_CFG_START2_ADDR, 1, &data, false);
	if (err < 0)
		goto close_upload_procedure;

	err = cdata->tf->read(cdata, ST_LSM6DS3H_WAI_ADDRESS,
					fw->size, fw_check_data, false);
	if (err < 0)
		goto close_upload_procedure;
#else /* CONFIG_ST_LSM6DS3H_IIO_LIMIT_FIFO */
	remaining_byte = fw->size;

	do {
		data = *(u8 *)&address;

		/* Set upload start address (LSB) */
		err = cdata->tf->write(cdata,
					ST_LSM6DS3H_FUNC_CFG_START1_ADDR,
					1, &data, false);
		if (err < 0)
			goto close_upload_procedure;

		data = *((u8 *)&address + 1);

		/* Set upload start address (MSB) */
		err = cdata->tf->write(cdata,
					ST_LSM6DS3H_FUNC_CFG_START2_ADDR,
					1, &data, false);
		if (err < 0)
			goto close_upload_procedure;

		if (remaining_byte < read_size)
			read_size = remaining_byte;

		err = cdata->tf->read(cdata,
				ST_LSM6DS3H_FUNC_CFG_DATA_READ_ADDR, read_size,
				&fw_check_data[fw->size - remaining_byte],
				false);
		if (err < 0)
			goto close_upload_procedure;

		remaining_byte -= read_size;
		address += read_size;
	} while (remaining_byte > 0);
#endif /* CONFIG_ST_LSM6DS3H_IIO_LIMIT_FIFO */

	/* Verify fw sent is same */
	for (i = 0; i < fw->size; i++) {
		if (fw_check_data[i] != fw->data[i]) {
			dev_err(cdata->dev, "uploaded fw not valid.\n");
			err = -EINVAL;
			goto close_upload_procedure;
		}
	}

	/* End the upload algo procedure */
	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_FUNC_CFG_ACCESS_ADDR,
				ST_LSM6DS3H_FUNC_CFG_ACCESS_MASK,
				ST_LSM6DS3H_DIS_BIT, false);
	if (err < 0)
		goto close_upload_procedure;

	mutex_unlock(&cdata->bank_registers_lock);

	/* Run the algo */
	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_FUNC_CFG_ACCESS_ADDR,
				ST_LSM6DS3H_FUNC_CFG_ACCESS_MASK2,
				ST_LSM6DS3H_EN_BIT, true);
	if (err < 0) {
		kfree(fw_check_data);
		goto release_firmware;
	}

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				st_lsm6ds3h_odr_table.addr[ST_MASK_ID_ACCEL],
				st_lsm6ds3h_odr_table.mask[ST_MASK_ID_ACCEL],
				st_lsm6ds3h_odr_table.odr_avl[3].value, false);
	if (err < 0) {
		kfree(fw_check_data);
		goto release_firmware;
	}

	err = st_lsm6ds3h_write_data_with_mask(cdata,
			ST_LSM6DS3H_FUNC_EN_ADDR,
			ST_LSM6DS3H_FUNC_EN_MASK,
			ST_LSM6DS3H_EN_BIT, true);
	if (err < 0) {
		kfree(fw_check_data);
		goto release_firmware;
	}

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_WRIST_TILT_EN_ADDR,
				ST_LSM6DS3H_WRIST_TILT_EN_MASK,
				ST_LSM6DS3H_EN_BIT, true);
	if (err < 0)
		return err;

	msleep(50);

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_WRIST_TILT_EN_ADDR,
				ST_LSM6DS3H_WRIST_TILT_EN_MASK,
				ST_LSM6DS3H_DIS_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				ST_LSM6DS3H_FUNC_EN_ADDR,
				ST_LSM6DS3H_FUNC_EN_MASK,
				ST_LSM6DS3H_DIS_BIT, true);
	if (err < 0) {
		kfree(fw_check_data);
		goto release_firmware;
	}

	err = st_lsm6ds3h_write_data_with_mask(cdata,
				st_lsm6ds3h_odr_table.addr[ST_MASK_ID_ACCEL],
				st_lsm6ds3h_odr_table.mask[ST_MASK_ID_ACCEL],
				ST_LSM6DS3H_ODR_POWER_OFF_VAL, false);
	if (err < 0) {
		kfree(fw_check_data);
		goto release_firmware;
	}

#ifndef CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED
	cdata->fifo2_algo_available = true;
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED */
#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	cdata->wrist_tilt_available = true;
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

	dev_info(cdata->dev, "algo upload completed\n");

	kfree(fw_check_data);

	release_firmware(fw);

	return 0;

close_upload_procedure:
	mutex_unlock(&cdata->bank_registers_lock);

	do {
		err2 = st_lsm6ds3h_write_data_with_mask(cdata,
					ST_LSM6DS3H_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3H_FUNC_CFG_ACCESS_MASK,
					ST_LSM6DS3H_DIS_BIT, true);
		msleep(200);
	} while (err2 < 0 && retry_counter++ < RETRY_COUNTER_LIMITATION);

free_fw_check_data:
	kfree(fw_check_data);
release_firmware:
	release_firmware(fw);
	return err;
}
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED */

static ST_LSM6DS3H_DEV_ATTR_SAMP_FREQ();
static ST_LSM6DS3H_DEV_ATTR_SAMP_FREQ_AVAIL();
static ST_LSM6DS3H_DEV_ATTR_SCALE_AVAIL(in_accel_scale_available);
static ST_LSM6DS3H_DEV_ATTR_SCALE_AVAIL(in_anglvel_scale_available);

static ST_LSM6DS3H_HWFIFO_ENABLED();
static ST_LSM6DS3H_HWFIFO_WATERMARK();
static ST_LSM6DS3H_HWFIFO_WATERMARK_MIN();
static ST_LSM6DS3H_HWFIFO_WATERMARK_MAX();
static ST_LSM6DS3H_HWFIFO_FLUSH();

static IIO_DEVICE_ATTR(reset_counter, S_IWUSR,
				NULL, st_lsm6ds3h_sysfs_reset_counter, 0);

static IIO_DEVICE_ATTR(pedo_threshold, S_IWUSR | S_IRUGO,
				st_lsm6ds3h_sysfs_get_pedo_threshold,
				st_lsm6ds3h_sysfs_set_pedo_threshold, 0);

static IIO_DEVICE_ATTR(max_delivery_rate, S_IWUSR | S_IRUGO,
				st_lsm6ds3h_sysfs_get_max_delivery_rate,
				st_lsm6ds3h_sysfs_set_max_delivery_rate, 0);

static IIO_DEVICE_ATTR(selftest_available, S_IRUGO,
				st_lsm6ds3h_sysfs_get_selftest_available,
				NULL, 0);

static IIO_DEVICE_ATTR(selftest, S_IWUSR | S_IRUGO,
				st_lsm6ds3h_sysfs_get_selftest_status,
				st_lsm6ds3h_sysfs_start_selftest_status, 0);

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
static IIO_DEVICE_ATTR(wrist_tilt_latency, S_IWUSR,
				NULL, st_lsm6ds3h_sysfs_set_wrist_tilt_latency, 0);
static IIO_DEVICE_ATTR(wrist_tilt_axes, S_IWUSR,
				NULL, st_lsm6ds3h_sysfs_set_wrist_tilt_axes, 0);
static IIO_DEVICE_ATTR(wrist_tilt_threshold, S_IWUSR,
				NULL, st_lsm6ds3h_sysfs_set_wrist_tilt_threshold, 0);
static IIO_DEVICE_ATTR(wrist_tilt_filter_timer, S_IWUSR,
				NULL, st_lsm6ds3h_sysfs_set_wrist_tilt_filter_timer, 0);
#endif

#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
static IIO_DEVICE_ATTR(injection_mode, S_IWUSR | S_IRUGO,
				st_lsm6ds3h_sysfs_get_injection_mode,
				st_lsm6ds3h_sysfs_set_injection_mode, 0);

static IIO_DEVICE_ATTR(in_accel_injection_raw, S_IWUSR, NULL,
				st_lsm6ds3h_sysfs_upload_xl_data, 0);

static IIO_DEVICE_ATTR(injection_sensors, S_IRUGO,
				st_lsm6ds3h_sysfs_get_injection_sensors,
				NULL, 0);
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */

static struct attribute *st_lsm6ds3h_accel_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	&iio_dev_attr_injection_mode.dev_attr.attr,
	&iio_dev_attr_in_accel_injection_raw.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */

	&iio_dev_attr_do_calibrate.dev_attr.attr,

	NULL,
};

static struct attribute *st_lsm6ds3h_accel_wk_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,

	NULL,
};

static const struct attribute_group st_lsm6ds3h_accel_attribute_group = {
	.attrs = st_lsm6ds3h_accel_attributes,
};

static const struct attribute_group st_lsm6ds3h_accel_wk_attribute_group = {
	.attrs = st_lsm6ds3h_accel_wk_attributes,
};

static const struct iio_info st_lsm6ds3h_accel_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_accel_attribute_group,
	.read_raw = &st_lsm6ds3h_read_raw,
	.write_raw = &st_lsm6ds3h_write_raw,
};

static const struct iio_info st_lsm6ds3h_accel_wk_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_accel_wk_attribute_group,
	.read_raw = &st_lsm6ds3h_read_raw,
	.write_raw = &st_lsm6ds3h_write_raw,
};

static struct attribute *st_lsm6ds3h_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_do_calibrate.dev_attr.attr,
	NULL,
};

static struct attribute *st_lsm6ds3h_gyro_wk_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3h_gyro_attribute_group = {
	.attrs = st_lsm6ds3h_gyro_attributes,
};

static const struct attribute_group st_lsm6ds3h_gyro_wk_attribute_group = {
	.attrs = st_lsm6ds3h_gyro_wk_attributes,
};

static const struct iio_info st_lsm6ds3h_gyro_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_gyro_attribute_group,
	.read_raw = &st_lsm6ds3h_read_raw,
	.write_raw = &st_lsm6ds3h_write_raw,
};

static const struct iio_info st_lsm6ds3h_gyro_wk_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_gyro_wk_attribute_group,
	.read_raw = &st_lsm6ds3h_read_raw,
	.write_raw = &st_lsm6ds3h_write_raw,
};

static struct attribute *st_lsm6ds3h_sign_motion_attributes[] = {
#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	&iio_dev_attr_injection_sensors.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */
	NULL,
};

static const struct attribute_group st_lsm6ds3h_sign_motion_attribute_group = {
	.attrs = st_lsm6ds3h_sign_motion_attributes,
};

static const struct iio_info st_lsm6ds3h_sign_motion_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_sign_motion_attribute_group,
};

static struct attribute *st_lsm6ds3h_step_c_attributes[] = {
	&iio_dev_attr_reset_counter.dev_attr.attr,
	&iio_dev_attr_max_delivery_rate.dev_attr.attr,
#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	&iio_dev_attr_injection_sensors.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */
	&iio_dev_attr_pedo_threshold.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3h_step_c_attribute_group = {
	.attrs = st_lsm6ds3h_step_c_attributes,
};

static const struct iio_info st_lsm6ds3h_step_c_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_step_c_attribute_group,
	.read_raw = &st_lsm6ds3h_read_raw,
};

static struct attribute *st_lsm6ds3h_step_d_attributes[] = {
#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	&iio_dev_attr_injection_sensors.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */
	NULL,
};

static const struct attribute_group st_lsm6ds3h_step_d_attribute_group = {
	.attrs = st_lsm6ds3h_step_d_attributes,
};

static const struct iio_info st_lsm6ds3h_step_d_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_step_d_attribute_group,
};

static struct attribute *st_lsm6ds3h_tilt_attributes[] = {
#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	&iio_dev_attr_injection_sensors.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */
	NULL,
};

static const struct attribute_group st_lsm6ds3h_tilt_attribute_group = {
	.attrs = st_lsm6ds3h_tilt_attributes,
};

static const struct iio_info st_lsm6ds3h_tilt_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_tilt_attribute_group,
};

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
static struct attribute *st_lsm6ds3h_wrist_tilt_attributes[] = {
	&iio_dev_attr_wrist_tilt_latency.dev_attr.attr,
	&iio_dev_attr_wrist_tilt_axes.dev_attr.attr,
	&iio_dev_attr_wrist_tilt_threshold.dev_attr.attr,
	&iio_dev_attr_wrist_tilt_filter_timer.dev_attr.attr,
#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	&iio_dev_attr_injection_sensors.dev_attr.attr,
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */
	NULL,
};

static const struct attribute_group st_lsm6ds3h_wrist_tilt_attribute_group = {
	.attrs = st_lsm6ds3h_wrist_tilt_attributes,
};

static const struct iio_info st_lsm6ds3h_wrist_tilt_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_wrist_tilt_attribute_group,
};
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
static struct attribute *st_lsm6ds3h_tap_tap_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6ds3h_tap_tap_attribute_group = {
	.attrs = st_lsm6ds3h_tap_tap_attributes,
};

static const struct iio_info st_lsm6ds3h_tap_tap_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3h_tap_tap_attribute_group,
};
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

#ifdef CONFIG_IIO_TRIGGER
static const struct iio_trigger_ops st_lsm6ds3h_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = ST_LSM6DS3H_TRIGGER_SET_STATE,
};
#define ST_LSM6DS3H_TRIGGER_OPS (&st_lsm6ds3h_trigger_ops)
#else
#define ST_LSM6DS3H_TRIGGER_OPS NULL
#endif

int st_lsm6ds3h_common_probe(struct lsm6ds3h_data *cdata, int irq)
{
	u8 wai = 0x00;
	int i, n, err;
	struct lsm6ds3h_sensor_data *sdata;
	u8 reset_value = ST_LSM6DS3H_RESET_MASK;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->fifo_lock);
	mutex_init(&cdata->tb.buf_lock);
	mutex_init(&cdata->odr_lock);

	cdata->fifo_watermark = 0;
	cdata->fifo_status = BYPASS;
	cdata->enable_digfunc_mask = 0;
	cdata->enable_pedometer_mask = 0;

#ifndef CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED
	cdata->fifo2_algo_available = false;
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_DISABLED */
#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	cdata->wrist_tilt_available = false;
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	cdata->enable_sensorhub_mask = 0;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

	cdata->irq_enable_fifo_mask = 0;
	cdata->irq_enable_accel_ext_mask = 0;

	for (i = 0; i < ST_INDIO_DEV_NUM + 1; i++) {
		cdata->hw_odr[i] = 0;
		cdata->v_odr[i] = 0;
		cdata->hwfifo_enabled[i] = false;
		cdata->hwfifo_decimator[i] = 0;
		cdata->hwfifo_watermark[i] = 1;
		cdata->nofifo_decimation[i].decimator = 1;
		cdata->nofifo_decimation[i].num_samples = 0;
		cdata->fifo_output[i].sip = 0;
		cdata->fifo_output[i].decimator = 1;
	}

	cdata->sensors_use_fifo = 0;
	cdata->sensors_enabled = 0;
	cdata->system_state = SF_NORMAL;

	cdata->gyro_selftest_status = 0;
	cdata->accel_selftest_status = 0;

	cdata->accel_on = false;
	cdata->accel_wk_on = false;
	cdata->gyro_on = false;
	cdata->gyro_wk_on = false;
	cdata->magn_on = false;

	cdata->reset_steps = false;

	cdata->accel_odr_dependency[0] = 0;
	cdata->accel_odr_dependency[1] = 0;
	cdata->accel_odr_dependency[2] = 0;
	cdata->accel_odr_dependency[3] = 0;

	cdata->trigger_odr = 0;
	cdata->fifo_odr = 0;

	cdata->fifo_data = kmalloc(ST_LSM6DS3H_MAX_FIFO_SIZE *
						sizeof(u8), GFP_KERNEL);
	if (!cdata->fifo_data)
		return -ENOMEM;

#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	cdata->injection_mode = false;
	cdata->last_injection_timestamp = 0;

	INIT_WORK(&cdata->injection_work, &st_lsm6ds3h_injection_work);
	hrtimer_init(&cdata->injection_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cdata->injection_timer.function = &st_lsm6ds3h_injection_timer_func;
	spin_lock_init(&cdata->injection_spinlock);
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */

	err = cdata->tf->read(cdata, ST_LSM6DS3H_WAI_ADDRESS, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		err = -EPROBE_DEFER;
		goto free_fifo_data;
	}
	if (wai != ST_LSM6DS3H_WAI_EXP) {
		dev_err(cdata->dev,
			"Who-Am-I value not valid. Expected %x, Found %x\n",
						ST_LSM6DS3H_WAI_EXP, wai);
		err = -ENODEV;
		goto free_fifo_data;
	}

	if (irq > 0) {
		cdata->irq = irq;
		dev_info(cdata->dev, "driver use DRDY int pin 1 at irq:%d\n", irq);
	} else {
		err = -EINVAL;
		dev_info(cdata->dev,
			"DRDY not available, curernt implementation needs irq!\n");
		goto free_fifo_data;
	}

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		cdata->indio_dev[i] = iio_device_alloc(sizeof(struct lsm6ds3h_sensor_data));
		if (!cdata->indio_dev[i]) {
			err = -ENOMEM;
			goto iio_device_free;
		}

		sdata = iio_priv(cdata->indio_dev[i]);
		sdata->cdata = cdata;
		sdata->sindex = i;

		switch (i) {
		case ST_MASK_ID_ACCEL:
		case ST_MASK_ID_ACCEL_WK:
			sdata->data_out_reg = st_lsm6ds3h_accel_ch[0].address;
			cdata->v_odr[i] = st_lsm6ds3h_odr_table.odr_avl[0].hz;
			sdata->c_gain[0] = st_lsm6ds3h_fs_table[i].fs_avl[0].gain;
			sdata->num_data_channels = 3;
			break;
		case ST_MASK_ID_GYRO:
		case ST_MASK_ID_GYRO_WK:
			sdata->data_out_reg = st_lsm6ds3h_gyro_ch[0].address;
			cdata->v_odr[i] = st_lsm6ds3h_odr_table.odr_avl[0].hz;
			sdata->c_gain[0] = st_lsm6ds3h_fs_table[i].fs_avl[0].gain;
			sdata->num_data_channels = 3;
			break;
		case ST_MASK_ID_STEP_COUNTER:
			sdata->data_out_reg = st_lsm6ds3h_step_c_ch[0].address;
			sdata->num_data_channels = 1;
			break;

		default:
			sdata->num_data_channels = 0;
			break;
		}

		cdata->indio_dev[i]->modes = INDIO_DIRECT_MODE;
	}

	cdata->indio_dev[ST_MASK_ID_ACCEL]->name =
			kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3H_ACCEL_SUFFIX_NAME);
	cdata->indio_dev[ST_MASK_ID_ACCEL]->info = &st_lsm6ds3h_accel_info;
	cdata->indio_dev[ST_MASK_ID_ACCEL]->channels = st_lsm6ds3h_accel_ch;
	cdata->indio_dev[ST_MASK_ID_ACCEL]->num_channels =
						ARRAY_SIZE(st_lsm6ds3h_accel_ch);

	cdata->indio_dev[ST_MASK_ID_ACCEL_WK]->name =
			kasprintf(GFP_KERNEL, "%s_%s_%s", cdata->name,
					ST_LSM6DS3H_ACCEL_SUFFIX_NAME, ST_LSM6DS3H_WAKEUP_SUFFIX_NAME);
	cdata->indio_dev[ST_MASK_ID_ACCEL_WK]->info = &st_lsm6ds3h_accel_wk_info;
	cdata->indio_dev[ST_MASK_ID_ACCEL_WK]->channels = st_lsm6ds3h_accel_ch;
	cdata->indio_dev[ST_MASK_ID_ACCEL_WK]->num_channels =
						ARRAY_SIZE(st_lsm6ds3h_accel_ch);

	cdata->indio_dev[ST_MASK_ID_GYRO]->name =
			kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3H_GYRO_SUFFIX_NAME);
	cdata->indio_dev[ST_MASK_ID_GYRO]->info = &st_lsm6ds3h_gyro_info;
	cdata->indio_dev[ST_MASK_ID_GYRO]->channels = st_lsm6ds3h_gyro_ch;
	cdata->indio_dev[ST_MASK_ID_GYRO]->num_channels =
						ARRAY_SIZE(st_lsm6ds3h_gyro_ch);

	cdata->indio_dev[ST_MASK_ID_GYRO_WK]->name =
			kasprintf(GFP_KERNEL, "%s_%s_%s", cdata->name,
					ST_LSM6DS3H_GYRO_SUFFIX_NAME, ST_LSM6DS3H_WAKEUP_SUFFIX_NAME);
	cdata->indio_dev[ST_MASK_ID_GYRO_WK]->info = &st_lsm6ds3h_gyro_wk_info;
	cdata->indio_dev[ST_MASK_ID_GYRO_WK]->channels = st_lsm6ds3h_gyro_ch;
	cdata->indio_dev[ST_MASK_ID_GYRO_WK]->num_channels =
						ARRAY_SIZE(st_lsm6ds3h_gyro_ch);

	cdata->indio_dev[ST_MASK_ID_SIGN_MOTION]->name =
			kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3H_SIGN_MOTION_SUFFIX_NAME);
	cdata->indio_dev[ST_MASK_ID_SIGN_MOTION]->info =
						&st_lsm6ds3h_sign_motion_info;
	cdata->indio_dev[ST_MASK_ID_SIGN_MOTION]->channels =
						st_lsm6ds3h_sign_motion_ch;
	cdata->indio_dev[ST_MASK_ID_SIGN_MOTION]->num_channels =
					ARRAY_SIZE(st_lsm6ds3h_sign_motion_ch);

	cdata->indio_dev[ST_MASK_ID_STEP_COUNTER]->name =
			kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3H_STEP_COUNTER_SUFFIX_NAME);
	cdata->indio_dev[ST_MASK_ID_STEP_COUNTER]->info =
						&st_lsm6ds3h_step_c_info;
	cdata->indio_dev[ST_MASK_ID_STEP_COUNTER]->channels =
						st_lsm6ds3h_step_c_ch;
	cdata->indio_dev[ST_MASK_ID_STEP_COUNTER]->num_channels =
					ARRAY_SIZE(st_lsm6ds3h_step_c_ch);

	cdata->indio_dev[ST_MASK_ID_STEP_DETECTOR]->name =
			kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3H_STEP_DETECTOR_SUFFIX_NAME);
	cdata->indio_dev[ST_MASK_ID_STEP_DETECTOR]->info =
						&st_lsm6ds3h_step_d_info;
	cdata->indio_dev[ST_MASK_ID_STEP_DETECTOR]->channels =
						st_lsm6ds3h_step_d_ch;
	cdata->indio_dev[ST_MASK_ID_STEP_DETECTOR]->num_channels =
					ARRAY_SIZE(st_lsm6ds3h_step_d_ch);

	cdata->indio_dev[ST_MASK_ID_TILT]->name =
			kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3H_TILT_SUFFIX_NAME);
	cdata->indio_dev[ST_MASK_ID_TILT]->info = &st_lsm6ds3h_tilt_info;
	cdata->indio_dev[ST_MASK_ID_TILT]->channels = st_lsm6ds3h_tilt_ch;
	cdata->indio_dev[ST_MASK_ID_TILT]->num_channels =
					ARRAY_SIZE(st_lsm6ds3h_tilt_ch);

	err = cdata->tf->write(cdata, ST_LSM6DS3H_RESET_ADDR, 1,
							&reset_value, true);
	if (err < 0)
		goto iio_device_free;

	msleep(200);

	err = st_lsm6ds3h_upload_algo(cdata);
	if (err < 0)
		dev_err(cdata->dev, "failed to upload fw\n");

	err = st_lsm6ds3h_init_sensor(cdata);
	if (err < 0)
		goto iio_device_free;

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (cdata->wrist_tilt_available) {
		cdata->indio_dev[ST_MASK_ID_WRIST_TILT] = iio_device_alloc(sizeof(struct lsm6ds3h_sensor_data));
		if (!cdata->indio_dev[ST_MASK_ID_WRIST_TILT]) {
			cdata->wrist_tilt_available = false;
			err = -ENOMEM;
			goto iio_device_free;
		}

		sdata = iio_priv(cdata->indio_dev[ST_MASK_ID_WRIST_TILT]);
		sdata->cdata = cdata;
		sdata->sindex = ST_MASK_ID_WRIST_TILT;
		sdata->num_data_channels = 0;

		cdata->indio_dev[ST_MASK_ID_WRIST_TILT]->modes = INDIO_DIRECT_MODE;

		cdata->indio_dev[ST_MASK_ID_WRIST_TILT]->name =
				kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3H_WRIST_TILT_SUFFIX_NAME);
		cdata->indio_dev[ST_MASK_ID_WRIST_TILT]->info =
						&st_lsm6ds3h_wrist_tilt_info;
		cdata->indio_dev[ST_MASK_ID_WRIST_TILT]->channels =
						st_lsm6ds3h_wrist_tilt_ch;
		cdata->indio_dev[ST_MASK_ID_WRIST_TILT]->num_channels =
					ARRAY_SIZE(st_lsm6ds3h_wrist_tilt_ch);
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

	st_lsm6ds3h_i2c_master_probe(cdata);

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
		cdata->indio_dev[ST_MASK_ID_TAP_TAP] = iio_device_alloc(sizeof(struct lsm6ds3h_sensor_data));
		if (!cdata->indio_dev[ST_MASK_ID_TAP_TAP]) {
			dev_err(cdata->dev, "failed to allocate TapTap device\n");
			err = -ENOMEM;
			goto iio_device_free;
		}

		sdata = iio_priv(cdata->indio_dev[ST_MASK_ID_TAP_TAP]);
		sdata->cdata = cdata;
		sdata->sindex = ST_MASK_ID_TAP_TAP;
		sdata->num_data_channels = 0;

		cdata->indio_dev[ST_MASK_ID_TAP_TAP]->modes = INDIO_DIRECT_MODE;

		cdata->indio_dev[ST_MASK_ID_TAP_TAP]->name =
				kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3H_TAP_TAP_SUFFIX_NAME);
		cdata->indio_dev[ST_MASK_ID_TAP_TAP]->info =
						&st_lsm6ds3h_tap_tap_info;
		cdata->indio_dev[ST_MASK_ID_TAP_TAP]->channels =
						st_lsm6ds3h_tap_tap_ch;
		cdata->indio_dev[ST_MASK_ID_TAP_TAP]->num_channels =
					ARRAY_SIZE(st_lsm6ds3h_tap_tap_ch);
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

	err = st_lsm6ds3h_allocate_rings(cdata);
	if (err < 0)
		goto iio_device_free;

	if (irq > 0) {
		err = st_lsm6ds3h_allocate_triggers(cdata,
							ST_LSM6DS3H_TRIGGER_OPS);
		if (err < 0)
			goto deallocate_ring;
	}

	for (n = 0; n < ST_INDIO_DEV_NUM; n++) {
		err = iio_device_register(cdata->indio_dev[n]);
		if (err)
			goto iio_device_unregister_and_trigger_deallocate;
	}

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (cdata->wrist_tilt_available) {
		err = iio_device_register(cdata->indio_dev[ST_MASK_ID_WRIST_TILT]);
		if (err)
			goto iio_device_unregister_and_trigger_deallocate;
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	err = iio_device_register(cdata->indio_dev[ST_MASK_ID_TAP_TAP]);
	if (err)
		goto iio_device_unregister_and_trigger_deallocate;
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

	device_init_wakeup(cdata->dev, true);

	pm_runtime_enable(cdata->dev);

	return 0;

iio_device_unregister_and_trigger_deallocate:
	for (n--; n >= 0; n--)
		iio_device_unregister(cdata->indio_dev[n]);

	if (irq > 0)
		st_lsm6ds3h_deallocate_triggers(cdata);
deallocate_ring:
	st_lsm6ds3h_deallocate_rings(cdata);
iio_device_free:
	for (i--; i >= 0; i--)
		iio_device_free(cdata->indio_dev[i]);
free_fifo_data:
	kfree(cdata->fifo_data);

	return err;
}
EXPORT_SYMBOL(st_lsm6ds3h_common_probe);

void st_lsm6ds3h_common_remove(struct lsm6ds3h_data *cdata, int irq)
{
	int i;

	pm_runtime_disable(cdata->dev);

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	iio_device_unregister(cdata->indio_dev[ST_MASK_ID_TAP_TAP]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (cdata->wrist_tilt_available)
		iio_device_unregister(cdata->indio_dev[ST_MASK_ID_WRIST_TILT]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_device_unregister(cdata->indio_dev[i]);

	if (irq > 0)
		st_lsm6ds3h_deallocate_triggers(cdata);

	st_lsm6ds3h_deallocate_rings(cdata);

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	iio_device_free(cdata->indio_dev[ST_MASK_ID_TAP_TAP]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (cdata->wrist_tilt_available)
		iio_device_free(cdata->indio_dev[ST_MASK_ID_WRIST_TILT]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_device_free(cdata->indio_dev[i]);

	kfree(cdata->fifo_data);

	st_lsm6ds3h_i2c_master_exit(cdata);
}
EXPORT_SYMBOL(st_lsm6ds3h_common_remove);

#ifdef CONFIG_PM
int st_lsm6ds3h_common_suspend(struct lsm6ds3h_data *cdata)
{
	disable_irq(cdata->irq);

	int err, i;
	u8 tmp_sensors_enabled;
	struct lsm6ds3h_sensor_data *sdata;

	tmp_sensors_enabled = cdata->sensors_enabled;

	dev_dbg(cdata->dev, "st_lsm6ds3h_common_suspend enabled=%d\n",
		cdata->sensors_enabled);

	for (i = 0; i < ST_INDIO_FULL_DEV_NUM; i++) {
		if (((1 << i) & ST_INDIO_DEV_AG_MASK) && stay_wake)
			continue;

		if ((1 << i) & ST_LSM6DS3H_WAKE_UP_SENSORS)
			continue;

		sdata = iio_priv(cdata->indio_dev[i]);

		err = st_lsm6ds3h_set_drdy_irq(sdata, false);
		if (err < 0)
			return err;
	}
	cdata->sensors_enabled = tmp_sensors_enabled;

	if (cdata->sensors_enabled & ST_LSM6DS3H_WAKE_UP_SENSORS) {
		if (device_may_wakeup(cdata->dev))
			enable_irq_wake(cdata->irq);
	}
	cdata->system_state = SF_SUSPEND;

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3h_common_suspend);

int st_lsm6ds3h_common_resume(struct lsm6ds3h_data *cdata)
{
	int err = 0, i;
	struct lsm6ds3h_sensor_data *sdata;

	dev_dbg(cdata->dev, "st_lsm6ds3h_common_resume enabled=%d\n",
		cdata->sensors_enabled);

	for (i = 0; i < ST_INDIO_FULL_DEV_NUM; i++) {
		if (((1 << i) & ST_INDIO_DEV_AG_MASK) && stay_wake)
			continue;

		if ((1 << i) & ST_LSM6DS3H_WAKE_UP_SENSORS)
			continue;

		sdata = iio_priv(cdata->indio_dev[i]);

		if (BIT(sdata->sindex) & cdata->sensors_enabled) {
			err = st_lsm6ds3h_set_drdy_irq(sdata, true);
			if (err < 0)
				goto out;
		}
	}

	cdata->system_state = SF_RESUME;

	if (cdata->sensors_enabled & ST_LSM6DS3H_WAKE_UP_SENSORS) {
		if (device_may_wakeup(cdata->dev))
			disable_irq_wake(cdata->irq);
	}

out:
	enable_irq(cdata->irq);
	return err;
}
EXPORT_SYMBOL(st_lsm6ds3h_common_resume);
#endif /* CONFIG_PM */

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3h core driver");
MODULE_LICENSE("GPL v2");
