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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/delay.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <asm/unaligned.h>
#include "lis2dg_core.h"


#define LIS2DG_FS_LIST_NUM			4
enum {
	LIS2DG_LP_MODE = 0,
	LIS2DG_HR_MODE,
	LIS2DG_MODE_COUNT,
};

#define LIS2DG_ADD_CHANNEL(device_type, modif, index, mod, endian, sbits, rbits, addr, s) \
{ \
	.type = device_type, \
	.modified = modif, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = s, \
		.realbits = rbits, \
		.shift = sbits - rbits, \
		.storagebits = sbits, \
		.endianness = endian, \
	}, \
}

struct lis2dg_odr_reg {
	u32 hz;
	u8 value;
};

static const struct lis2dg_odr_table_t {
	u8 addr;
	u8 mask;
	struct lis2dg_odr_reg odr_avl[LIS2DG_MODE_COUNT][LIS2DG_ODR_LP_LIST_NUM];
} lis2dg_odr_table = {
	.addr = LIS2DG_ODR_ADDR,
	.mask = LIS2DG_ODR_MASK,

	/*
	 * ODR values for Low Power Mode
	 */
	.odr_avl[LIS2DG_LP_MODE][0] = { .hz = 0, .value = LIS2DG_ODR_POWER_OFF_VAL, },
	.odr_avl[LIS2DG_LP_MODE][1] = { .hz = 1, .value = LIS2DG_ODR_1HZ_LP_VAL, },
	.odr_avl[LIS2DG_LP_MODE][2] = { .hz = 12, .value = LIS2DG_ODR_12HZ_LP_VAL, },
	.odr_avl[LIS2DG_LP_MODE][3] = { .hz = 25, .value = LIS2DG_ODR_25HZ_LP_VAL, },
	.odr_avl[LIS2DG_LP_MODE][4] = { .hz = 50, .value = LIS2DG_ODR_50HZ_LP_VAL, },
	.odr_avl[LIS2DG_LP_MODE][5] = { .hz = 100, .value = LIS2DG_ODR_100HZ_LP_VAL, },
	.odr_avl[LIS2DG_LP_MODE][6] = { .hz = 200, .value = LIS2DG_ODR_200HZ_LP_VAL, },
	.odr_avl[LIS2DG_LP_MODE][7] = { .hz = 400, .value = LIS2DG_ODR_400HZ_LP_VAL, },
	.odr_avl[LIS2DG_LP_MODE][8] = { .hz = 800, .value = LIS2DG_ODR_800HZ_LP_VAL, },

	/*
	 * ODR values for High Resolution Mode
	 */
	.odr_avl[LIS2DG_HR_MODE][0] = { .hz = 0, .value = LIS2DG_ODR_POWER_OFF_VAL, },
	.odr_avl[LIS2DG_HR_MODE][1] = { .hz = 12, .value = LIS2DG_ODR_12_5HZ_HR_VAL, },
	.odr_avl[LIS2DG_HR_MODE][2] = { .hz = 25, .value = LIS2DG_ODR_25HZ_HR_VAL, },
	.odr_avl[LIS2DG_HR_MODE][3] = { .hz = 50, .value = LIS2DG_ODR_50HZ_HR_VAL, },
	.odr_avl[LIS2DG_HR_MODE][4] = { .hz = 100, .value = LIS2DG_ODR_100HZ_HR_VAL, },
	.odr_avl[LIS2DG_HR_MODE][5] = { .hz = 200, .value = LIS2DG_ODR_200HZ_HR_VAL, },
	.odr_avl[LIS2DG_HR_MODE][6] = { .hz = 400, .value = LIS2DG_ODR_400HZ_HR_VAL, },
	.odr_avl[LIS2DG_HR_MODE][7] = { .hz = 800, .value = LIS2DG_ODR_800HZ_HR_VAL, },
};

struct lis2dg_fs_reg {
	unsigned int gain[LIS2DG_MODE_COUNT];
	u8 value;
	int urv;
};

static struct lis2dg_fs_table {
	u8 addr;
	u8 mask;
	struct lis2dg_fs_reg fs_avl[LIS2DG_FS_LIST_NUM];
} lis2dg_fs_table = {
	.addr = LIS2DG_FS_ADDR,
	.mask = LIS2DG_FS_MASK,
	.fs_avl[0] = {
		.gain = {LIS2DG_FS_2G_GAIN_LP, LIS2DG_FS_2G_GAIN_HR,},
		.value = LIS2DG_FS_2G_VAL,
		.urv = 2,
	},
	.fs_avl[1] = {
		.gain = {LIS2DG_FS_4G_GAIN_LP, LIS2DG_FS_4G_GAIN_HR,},
		.value = LIS2DG_FS_4G_VAL,
		.urv = 4,
	},
	.fs_avl[2] = {
		.gain = {LIS2DG_FS_8G_GAIN_LP, LIS2DG_FS_8G_GAIN_LP,},
		.value = LIS2DG_FS_8G_VAL,
		.urv = 8,
	},
	.fs_avl[3] = {
		.gain = {LIS2DG_FS_16G_GAIN_LP, LIS2DG_FS_16G_GAIN_HR,},
		.value = LIS2DG_FS_16G_VAL,
		.urv = 16,
	},
};

static const struct lis2dg_sensors_table {
	const char *name;
	const char *description;
	const u32 min_odr_hz;
	const u8 iio_channel_size;
	const struct iio_chan_spec iio_channel[LIS2DG_MAX_CHANNEL_SPEC];
} lis2dg_sensors_table[LIS2DG_SENSORS_NUMB] = {
	[LIS2DG_ACCEL] = {
		.name = "accel",
		.description = "ST LIS2DG Accelerometer Sensor",
		.min_odr_hz = LIS2DG_ACCEL_ODR,
		.iio_channel = {
			LIS2DG_ADD_CHANNEL(IIO_ACCEL, 1, 0, IIO_MOD_X, IIO_LE,
					16, 10, LIS2DG_OUTX_L_ADDR, 's'),
			LIS2DG_ADD_CHANNEL(IIO_ACCEL, 1, 1, IIO_MOD_Y, IIO_LE,
					16, 10, LIS2DG_OUTY_L_ADDR, 's'),
			LIS2DG_ADD_CHANNEL(IIO_ACCEL, 1, 2, IIO_MOD_Z, IIO_LE,
					16, 10, LIS2DG_OUTZ_L_ADDR, 's'),
			IIO_CHAN_SOFT_TIMESTAMP(3)
		},
		.iio_channel_size = LIS2DG_MAX_CHANNEL_SPEC,
	},
	[LIS2DG_STEP_C] = {
		.name = "step_c",
		.description = "ST LIS2DG Step Counter Sensor",
		.min_odr_hz = LIS2DG_STEP_D_ODR,
		.iio_channel = {
			{
				.type = IIO_STEP_COUNTER,
				.channel = 0,
				.modified = 0,
				.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
				.address = LIS2DG_STEP_C_OUT_L_ADDR,
				.scan_index = 0,
				.channel2 = IIO_NO_MOD,
				.scan_type = {
					.sign = 'u',
					.realbits = 16,
					.storagebits = 16,
					.endianness = IIO_LE,
				},
			},
			IIO_CHAN_SOFT_TIMESTAMP(1)
		},
		.iio_channel_size = LIS2DG_EVENT_CHANNEL_SPEC_SIZE,
	},
	[LIS2DG_TAP] = {
		.name = "tap",
		.description = "ST LIS2DG Tap Sensor",
		.min_odr_hz = LIS2DG_TAP_ODR,
		.iio_channel = {
			{
				.type = IIO_TAP,
				.channel = 0,
				.modified = 0,
				.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_RISING),
			},
			IIO_CHAN_SOFT_TIMESTAMP(1)
		},
		.iio_channel_size = LIS2DG_EVENT_CHANNEL_SPEC_SIZE,
	},
	[LIS2DG_DOUBLE_TAP] = {
		.name = "tap_tap",
		.description = "ST LIS2DG Double Tap Sensor",
		.min_odr_hz = LIS2DG_TAP_ODR,
		.iio_channel = {
			{
				.type = IIO_TAP_TAP,
				.channel = 0,
				.modified = 0,
				.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_RISING),
			},
			IIO_CHAN_SOFT_TIMESTAMP(1)
		},
		.iio_channel_size = LIS2DG_EVENT_CHANNEL_SPEC_SIZE,
	},
	[LIS2DG_STEP_D] = {
		.name = "step_d",
		.description = "ST LIS2DG Step Detector Sensor",
		.min_odr_hz = LIS2DG_STEP_D_ODR,
		.iio_channel = {
			{
				.type = IIO_STEP_DETECTOR,
				.channel = 0,
				.modified = 0,
				.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_RISING),
			},
			IIO_CHAN_SOFT_TIMESTAMP(1)
		},
		.iio_channel_size = LIS2DG_EVENT_CHANNEL_SPEC_SIZE,
	},
	[LIS2DG_TILT] = {
		.name = "tilt",
		.description = "ST LIS2DG Tilt Sensor",
		.min_odr_hz = LIS2DG_TILT_ODR,
		.iio_channel = {
			{
				.type = IIO_TILT,
				.channel = 0,
				.modified = 0,
				.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_RISING),
			},
			IIO_CHAN_SOFT_TIMESTAMP(1),
		},
		.iio_channel_size = LIS2DG_EVENT_CHANNEL_SPEC_SIZE,
	},
	[LIS2DG_SIGN_M] = {
		.name = "sign_motion",
		.description = "ST LIS2DG Significant Motion Sensor",
		.min_odr_hz = LIS2DG_SIGN_M_ODR,
		.iio_channel = {
			{
				.type = IIO_SIGN_MOTION,
				.channel = 0,
				.modified = 0,
				.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_RISING),
			},
			IIO_CHAN_SOFT_TIMESTAMP(1)
		},
		.iio_channel_size = LIS2DG_EVENT_CHANNEL_SPEC_SIZE,
	},
};

static const struct {
	char *mode_str;
	u8 streg_val;
} lis2dg_selftest_table[] = {
	{
		.mode_str = "normal-mode",
		.streg_val = LIS2DG_SELFTEST_NORMAL,
	},
	{
		.mode_str = "positive-sign",
		.streg_val = LIS2DG_SELFTEST_POS_SIGN,
	},
	{
		.mode_str = "negative-sign",
		.streg_val = LIS2DG_SELFTEST_NEG_SIGN,
	},
};

int lis2dg_read_register(struct lis2dg_data *cdata, u8 reg_addr, int data_len,
							u8 *data)
{
	return cdata->tf->read(cdata, reg_addr, data_len, data);
}

static int lis2dg_write_register(struct lis2dg_data *cdata, u8 reg_addr,
							u8 mask, u8 data)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = lis2dg_read_register(cdata, reg_addr, 1, &old_data);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));
	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data);
}

int lis2dg_set_axis_enable(struct lis2dg_sensor_data *sdata, u8 value)
{
	return 0;
}
EXPORT_SYMBOL(lis2dg_set_axis_enable);

int lis2dg_set_fifo_mode(struct lis2dg_data *cdata, enum fifo_mode fm)
{
	u8 reg_value;
	bool enable_fifo;
	struct timespec ts;

	switch (fm) {
	case BYPASS:
		reg_value = LIS2DG_FIFO_MODE_BYPASS;
		enable_fifo = false;
		break;
	case CONTINUOS:
		reg_value = LIS2DG_FIFO_MODE_CONTINUOS;
		enable_fifo = true;
		break;
	default:
		return -EINVAL;
	}

	get_monotonic_boottime(&ts);
	cdata->accel_timestamp = timespec_to_ns(&ts);

	return lis2dg_write_register(cdata, LIS2DG_FIFO_MODE_ADDR,
				LIS2DG_FIFO_MODE_MASK, reg_value);
}
EXPORT_SYMBOL(lis2dg_set_fifo_mode);

int lis2dg_update_event_functions(struct lis2dg_data *cdata)
{
	u8 reg_val = 0;

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_SIGN_M))
		reg_val |= LIS2DG_FUNC_CTRL_SIGN_MOT_MASK;

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_TILT))
		reg_val |= LIS2DG_FUNC_CTRL_TILT_MASK;

	if ((CHECK_BIT(cdata->enabled_sensor, LIS2DG_STEP_D)) ||
				(CHECK_BIT(cdata->enabled_sensor, LIS2DG_STEP_C)))
		reg_val |= LIS2DG_FUNC_CTRL_STEP_CNT_MASK;

	return lis2dg_write_register(cdata,
				LIS2DG_FUNC_CTRL_ADDR,
				LIS2DG_FUNC_CTRL_EV_MASK,
				reg_val >> __ffs(LIS2DG_FUNC_CTRL_EV_MASK));
}

int lis2dg_set_fs(struct lis2dg_sensor_data *sdata, unsigned int fs)
{
	int err, i;

	for (i = 0; i < LIS2DG_FS_LIST_NUM; i++) {
		if (lis2dg_fs_table.fs_avl[i].urv == fs)
			break;
	}

	if (i == LIS2DG_FS_LIST_NUM)
		return -EINVAL;

	err = lis2dg_write_register(sdata->cdata,
				lis2dg_fs_table.addr,
				lis2dg_fs_table.mask,
				lis2dg_fs_table.fs_avl[i].value);
	if (err < 0)
		return err;

	sdata->gain =
			lis2dg_fs_table.fs_avl[i].gain[sdata->cdata->power_mode];

	return 0;
}

static int lis2dg_set_selftest_mode(struct lis2dg_sensor_data *sdata, u8 index)
{
	return lis2dg_write_register(sdata->cdata, LIS2DG_SELFTEST_ADDR,
					LIS2DG_SELFTEST_MASK,
					lis2dg_selftest_table[index].streg_val);
}

u8 lis2dg_event_irq1_value(struct lis2dg_data *cdata)
{
	u8 value = 0x0;

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_DOUBLE_TAP))
		value |= LIS2DG_INT1_TAP_MASK;

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_TAP))
		value |= LIS2DG_INT1_S_TAP_MASK | LIS2DG_INT1_TAP_MASK;

	return value;
}

u8 lis2dg_event_irq2_value(struct lis2dg_data *cdata)
{
	u8 value = 0x0;

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_TILT))
		value |= LIS2DG_INT2_TILT_MASK;

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_SIGN_M))
		value |= LIS2DG_INT2_SIG_MOT_DET_MASK;

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_STEP_D) ||
				CHECK_BIT(cdata->enabled_sensor, LIS2DG_STEP_C))
		value |= LIS2DG_INT2_STEP_DET_MASK;

	return value;
}

int lis2dg_write_max_odr(struct lis2dg_sensor_data *sdata)
{
	int err, i;
	u32 max_odr = 0;
	u8 power_mode = sdata->cdata->power_mode;
	struct lis2dg_sensor_data *t_sdata;

	for (i = 0; i < LIS2DG_SENSORS_NUMB; i++)
		if (CHECK_BIT(sdata->cdata->enabled_sensor, i)) {
			t_sdata = iio_priv(sdata->cdata->iio_sensors_dev[i]);
			if (t_sdata->odr > max_odr)
				max_odr = t_sdata->odr;
		}

	if (max_odr != sdata->cdata->common_odr) {
		for (i = 0; i < LIS2DG_ODR_LP_LIST_NUM; i++) {
			if (lis2dg_odr_table.odr_avl[power_mode][i].hz >= max_odr)
				break;
		}
		if (i == LIS2DG_ODR_LP_LIST_NUM)
			return -EINVAL;

		err = lis2dg_write_register(sdata->cdata,
				lis2dg_odr_table.addr,
				lis2dg_odr_table.mask,
				lis2dg_odr_table.odr_avl[power_mode][i].value);
		if (err < 0)
			return err;

		sdata->cdata->common_odr = max_odr;
		sdata->cdata->accel_deltatime = (max_odr) ? 1000000000L / max_odr : 0;
	}

	return 0;
}

int lis2dg_update_drdy_irq(struct lis2dg_sensor_data *sdata, bool state)
{
	u8 reg_addr, reg_val, reg_mask;

	switch (sdata->sindex) {
	case LIS2DG_TAP:
	case LIS2DG_DOUBLE_TAP:
		reg_val = lis2dg_event_irq1_value(sdata->cdata);
		reg_addr = LIS2DG_CTRL4_INT1_PAD_ADDR;
		reg_mask = LIS2DG_INT1_EVENTS_MASK;

		break;

	case LIS2DG_SIGN_M:
	case LIS2DG_TILT:
	case LIS2DG_STEP_D:
	case LIS2DG_STEP_C:
		reg_val = lis2dg_event_irq2_value(sdata->cdata);
		reg_addr = LIS2DG_CTRL5_INT2_PAD_ADDR;
		reg_mask = LIS2DG_INT2_EVENTS_MASK;

		break;

	case LIS2DG_ACCEL:
		reg_addr = LIS2DG_CTRL4_INT1_PAD_ADDR;
		reg_mask = LIS2DG_INT1_FTH_MASK;
		if (state)
			reg_val = LIS2DG_EN_BIT;
		else
			reg_val = LIS2DG_DIS_BIT;

		break;

	default:
		return -EINVAL;
	}

	return lis2dg_write_register(sdata->cdata, reg_addr, reg_mask,
				reg_val);
}
EXPORT_SYMBOL(lis2dg_update_drdy_irq);

int lis2dg_update_fifo(struct lis2dg_data *cdata)
{
	int err;
	u8 fifo_len;
	int fifo_size;
	struct iio_dev *indio_dev;

	indio_dev = cdata->iio_sensors_dev[LIS2DG_ACCEL];
	fifo_len = (indio_dev->buffer->length);

	err = lis2dg_write_register(cdata, LIS2DG_FIFO_THS_ADDR,
				LIS2DG_FIFO_THS_MASK,
				fifo_len);
	if (err < 0)
		return err;

	kfree(cdata->fifo_data);
	cdata->fifo_data = 0;

	fifo_size = fifo_len * LIS2DG_FIFO_BYTE_FOR_SAMPLE;
	if (fifo_size > 0) {
		cdata->fifo_data = kmalloc(fifo_size, GFP_KERNEL);
		if (!cdata->fifo_data)
			return -ENOMEM;

		cdata->fifo_size = fifo_size;
	}

	return lis2dg_set_fifo_mode(cdata, CONTINUOS);
}
EXPORT_SYMBOL(lis2dg_update_fifo);

int lis2dg_set_enable(struct lis2dg_sensor_data *sdata, bool state)
{
	int err = 0;

	if (sdata->enabled == state)
		return 0;

	/*
	 * Start assuming the sensor enabled if state == true.
	 * It will be restored if an error occur.
	 */
	if (state) {
		SET_BIT(sdata->cdata->enabled_sensor, sdata->sindex);
	} else {
		RESET_BIT(sdata->cdata->enabled_sensor, sdata->sindex);
	}

	switch (sdata->sindex) {
	case LIS2DG_TAP:
		if (state && CHECK_BIT(sdata->cdata->enabled_sensor,
							LIS2DG_DOUBLE_TAP)) {
			err = -EINVAL;

			goto enable_sensor_error;
		}

		break;

	case LIS2DG_DOUBLE_TAP:
		if (state && CHECK_BIT(sdata->cdata->enabled_sensor,
							LIS2DG_TAP)) {
			err = -EINVAL;

			goto enable_sensor_error;
		}

		break;

	case LIS2DG_TILT:
	case LIS2DG_SIGN_M:
	case LIS2DG_STEP_D:
	case LIS2DG_STEP_C:
		err = lis2dg_update_event_functions(sdata->cdata);
		if (err < 0)
			goto enable_sensor_error;

		break;

	case LIS2DG_ACCEL:
		break;

	default:
		return -EINVAL;
	}

	err = lis2dg_update_drdy_irq(sdata, state);
	if (err < 0)
		goto enable_sensor_error;

	err = lis2dg_write_max_odr(sdata);
	if (err < 0)
		goto enable_sensor_error;

	sdata->enabled = state;

	return 0;

enable_sensor_error:
	if (state) {
		RESET_BIT(sdata->cdata->enabled_sensor, sdata->sindex);
	} else {
		SET_BIT(sdata->cdata->enabled_sensor, sdata->sindex);
	}

	return err;
}
EXPORT_SYMBOL(lis2dg_set_enable);

int lis2dg_init_sensors(struct lis2dg_data *cdata)
{
	int err, i;
	struct lis2dg_sensor_data *sdata;

	for (i = 0; i < LIS2DG_SENSORS_NUMB; i++) {
		sdata = iio_priv(cdata->iio_sensors_dev[i]);

		err = lis2dg_set_enable(sdata, false);
		if (err < 0)
			return err;

		if (sdata->sindex == LIS2DG_ACCEL) {
			err = lis2dg_set_fs(sdata, LIS2DG_DEFAULT_ACCEL_FS);
			if (err < 0)
				return err;
		}
	}

	cdata->selftest_status = 0;

	/*
	 * Soft reset the device on power on.
	 */
	err = lis2dg_write_register(cdata, LIS2DG_SOFT_RESET_ADDR,
				LIS2DG_SOFT_RESET_MASK,
				LIS2DG_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Enable latched interrupt mode.
	 */
	err = lis2dg_write_register(cdata, LIS2DG_LIR_ADDR,
				LIS2DG_LIR_MASK,
				LIS2DG_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Enable block data update feature.
	 */
	err = lis2dg_write_register(cdata, LIS2DG_BDU_ADDR,
				LIS2DG_BDU_MASK,
				LIS2DG_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Route interrupt from INT2 to INT1 pin.
	 */
	err = lis2dg_write_register(cdata, LIS2DG_INT2_ON_INT1_ADDR,
				LIS2DG_INT2_ON_INT1_MASK,
				LIS2DG_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Configure default free fall event threshold.
	 */
	err = lis2dg_write_register(sdata->cdata, LIS2DG_FREE_FALL_ADDR,
				LIS2DG_FREE_FALL_THS_MASK,
				LIS2DG_FREE_FALL_THS_DEFAULT);
	if (err < 0)
		return err;

	/*
	 * Configure default free fall event duration.
	 */
	err = lis2dg_write_register(sdata->cdata, LIS2DG_FREE_FALL_ADDR,
				LIS2DG_FREE_FALL_DUR_MASK,
				LIS2DG_FREE_FALL_DUR_DEFAULT);
	if (err < 0)
		return err;

	/*
	 * Configure Tap event recognition on all direction (X, Y and Z axes).
	 */
	err = lis2dg_write_register(sdata->cdata, LIS2DG_TAP_AXIS_ADDR,
				LIS2DG_TAP_AXIS_MASK,
				LIS2DG_TAP_AXIS_ANABLE_ALL);
	if (err < 0)
		return err;

	/*
	 * Configure default threshold for Tap event recognition.
	 */
	err = lis2dg_write_register(sdata->cdata, LIS2DG_TAP_THS_ADDR,
				LIS2DG_TAP_THS_MASK,
				LIS2DG_TAP_THS_DEFAULT);
	if (err < 0)
		return err;

	/*
	 * Configure default threshold for Wake Up event recognition.
	 */
	err = lis2dg_write_register(sdata->cdata, LIS2DG_WAKE_UP_THS_ADDR,
				LIS2DG_WAKE_UP_THS_WU_MASK,
				LIS2DG_WAKE_UP_THS_WU_DEFAULT);
	if (err < 0)
		return err;

	return 0;
}

static ssize_t lis2dg_get_sampling_frequency(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct lis2dg_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->odr);
}

ssize_t lis2dg_set_sampling_frequency(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	u8 power_mode;
	unsigned int odr, i;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lis2dg_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	if (sdata->odr == odr)
		return count;

	power_mode = sdata->cdata->power_mode;
	for (i = 0; i < LIS2DG_ODR_LP_LIST_NUM; i++) {
		if (lis2dg_odr_table.odr_avl[power_mode][i].hz >= odr)
			break;
	}
	if (i == LIS2DG_ODR_LP_LIST_NUM)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	sdata->odr = lis2dg_odr_table.odr_avl[power_mode][i].hz;
	mutex_unlock(&indio_dev->mlock);

	err = lis2dg_write_max_odr(sdata);
	if (err < 0)
		return err;

	return (err < 0) ? err : count;
}

static ssize_t lis2dg_get_sampling_frequency_avail(struct device *dev,
						struct device_attribute
						*attr, char *buf)
{
	int i, len = 0, mode_count, mode;
	struct lis2dg_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	mode = sdata->cdata->power_mode;
	mode_count = (mode == LIS2DG_LP_MODE) ?
				LIS2DG_ODR_LP_LIST_NUM : LIS2DG_ODR_HR_LIST_NUM;

	for (i = 0; i < mode_count; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				lis2dg_odr_table.odr_avl[mode][i].hz);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t lis2dg_get_scale_avail(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int i, len = 0, mode;
	struct lis2dg_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	mode = sdata->cdata->power_mode;
	for (i = 0; i < LIS2DG_FS_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
				lis2dg_fs_table.fs_avl[i].gain[mode]);
	}
	buf[len - 1] = '\n';

	return len;
}

ssize_t lis2dg_get_hw_fifo_lenght(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", LIS2DG_MAX_FIFO_LENGHT);
}

static int lis2dg_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int *val,
							int *val2, long mask)
{
	int err;
	u8 outdata[2];
	struct lis2dg_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = lis2dg_set_enable(sdata, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		msleep(40);

		err = lis2dg_read_register(sdata->cdata, ch->address, 2, outdata);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		err = lis2dg_set_enable(sdata, false);
		mutex_unlock(&indio_dev->mlock);

		if (err < 0)
			return err;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sdata->gain;

		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return 0;
}

static int lis2dg_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err;
	struct lis2dg_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = lis2dg_set_fs(sdata, val2);
		mutex_unlock(&indio_dev->mlock);

		break;

	default:
		return -EINVAL;
	}

	return err;
}

ssize_t lis2dg_sysfs_flush_fifo(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lis2dg_sensor_data *sdata = iio_priv(indio_dev);

	disable_irq(sdata->cdata->irq);
	lis2dg_flush_works();

	mutex_lock(&sdata->cdata->fifo_lock);
	lis2dg_read_fifo(sdata->cdata, true);
	mutex_unlock(&sdata->cdata->fifo_lock);

	enable_irq(sdata->cdata->irq);

	return size;
}

ssize_t lis2dg_reset_step_counter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lis2dg_sensor_data *sdata = iio_priv(indio_dev);

	return lis2dg_write_register(sdata->cdata,
					LIS2DG_STEP_C_MINTHS_ADDR,
					LIS2DG_STEP_C_MINTHS_RST_NSTEP_MASK,
					LIS2DG_EN_BIT);
}

static ssize_t lis2dg_get_selftest_avail(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s %s %s\n", lis2dg_selftest_table[0].mode_str,
					lis2dg_selftest_table[1].mode_str,
					lis2dg_selftest_table[2].mode_str);
}

static ssize_t lis2dg_get_selftest_status(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	u8 status;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lis2dg_sensor_data *sdata = iio_priv(indio_dev);

	status = sdata->cdata->selftest_status;
	return sprintf(buf, "%s\n", lis2dg_selftest_table[status].mode_str);
}

static ssize_t lis2dg_set_selftest_status(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int err, i;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lis2dg_sensor_data *sdata = iio_priv(indio_dev);

	for (i = 0; i < ARRAY_SIZE(lis2dg_selftest_table); i++) {
		if (strncmp(buf, lis2dg_selftest_table[i].mode_str,
								size - 2) == 0)
			break;
	}
	if (i == ARRAY_SIZE(lis2dg_selftest_table))
		return -EINVAL;

	err = lis2dg_set_selftest_mode(sdata, i);
	if (err < 0)
		return err;

	sdata->cdata->selftest_status = i;

	return size;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
					lis2dg_get_sampling_frequency,
					lis2dg_set_sampling_frequency);
static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(lis2dg_get_sampling_frequency_avail);
static IIO_DEVICE_ATTR(in_accel_scale_available, S_IRUGO,
					lis2dg_get_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hw_fifo_lenght, S_IRUGO,
					lis2dg_get_hw_fifo_lenght, NULL, 0);
static IIO_DEVICE_ATTR(flush, S_IWUSR, NULL, lis2dg_sysfs_flush_fifo, 0);
static IIO_DEVICE_ATTR(reset_counter, S_IWUSR,
					NULL, lis2dg_reset_step_counter, 0);
static IIO_DEVICE_ATTR(selftest_available, S_IRUGO,
					lis2dg_get_selftest_avail, NULL, 0);
static IIO_DEVICE_ATTR(selftest, S_IWUSR | S_IRUGO,
					lis2dg_get_selftest_status,
					lis2dg_set_selftest_status, 0);

static struct attribute *lis2dg_accel_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static struct attribute *lis2dg_step_c_attributes[] = {
	&iio_dev_attr_reset_counter.dev_attr.attr,
	NULL,
};
static struct attribute *lis2dg_step_tap_attributes[] = {
	NULL,
};
static struct attribute *lis2dg_step_double_tap_attributes[] = {
	NULL,
};
static struct attribute *lis2dg_step_d_attributes[] = {
	NULL,
};
static struct attribute *lis2dg_tilt_attributes[] = {
	NULL,
};
static struct attribute *lis2dg_sign_m_attributes[] = {
	NULL,
};

static const struct attribute_group lis2dg_accel_attribute_group = {
	.attrs = lis2dg_accel_attributes,
};
static const struct attribute_group lis2dg_step_c_attribute_group = {
	.attrs = lis2dg_step_c_attributes,
};
static const struct attribute_group lis2dg_tap_attribute_group = {
	.attrs = lis2dg_step_tap_attributes,
};
static const struct attribute_group lis2dg_double_tap_attribute_group = {
	.attrs = lis2dg_step_double_tap_attributes,
};
static const struct attribute_group lis2dg_step_d_attribute_group = {
	.attrs = lis2dg_step_d_attributes,
};
static const struct attribute_group lis2dg_tilt_attribute_group = {
	.attrs = lis2dg_tilt_attributes,
};
static const struct attribute_group lis2dg_sign_m_attribute_group = {
	.attrs = lis2dg_sign_m_attributes,
};


static const struct iio_info lis2dg_info[LIS2DG_SENSORS_NUMB] = {
	[LIS2DG_ACCEL] = {
		.driver_module = THIS_MODULE,
		.attrs = &lis2dg_accel_attribute_group,
		.read_raw = &lis2dg_read_raw,
		.write_raw = &lis2dg_write_raw,
	},
	[LIS2DG_STEP_C] = {
		.driver_module = THIS_MODULE,
		.attrs = &lis2dg_step_c_attribute_group,
		.read_raw = &lis2dg_read_raw,
	},
	[LIS2DG_TAP] = {
		.driver_module = THIS_MODULE,
		.attrs = &lis2dg_tap_attribute_group,
	},
	[LIS2DG_DOUBLE_TAP] = {
		.driver_module = THIS_MODULE,
		.attrs = &lis2dg_double_tap_attribute_group,
	},
	[LIS2DG_STEP_D] = {
		.driver_module = THIS_MODULE,
		.attrs = &lis2dg_step_d_attribute_group,
	},
	[LIS2DG_TILT] = {
		.driver_module = THIS_MODULE,
		.attrs = &lis2dg_tilt_attribute_group,
	},
	[LIS2DG_SIGN_M] = {
		.driver_module = THIS_MODULE,
		.attrs = &lis2dg_sign_m_attribute_group,
	},
};


#ifdef CONFIG_IIO_TRIGGER
static const struct iio_trigger_ops lis2dg_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = (&lis2dg_trig_set_state),
};
#define LIS2DG_TRIGGER_OPS (&lis2dg_trigger_ops)
#else
#define LIS2DG_TRIGGER_OPS NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id lis2dg_dt_id[] = {
	{.compatible = "st,lis2dg",},
	{},
};

MODULE_DEVICE_TABLE(of, lis2dg_dt_id);

static u32 lis2dg_parse_dt(struct lis2dg_data *cdata)
{
	u32 val;
	struct device_node *np;

	np = cdata->dev->of_node;
	if (!np)
		return -EINVAL;

	if (!of_property_read_u32(np, "st,drdy-int-pin", &val) &&
							(val <= 2) && (val > 0))
		cdata->drdy_int_pin = (u8) val;
	else
		cdata->drdy_int_pin = 1;

	return 0;
}

#else
#endif

int lis2dg_common_probe(struct lis2dg_data *cdata, int irq)
{
	u8 wai = 0;
	int32_t err, i, n;
	struct iio_dev *piio_dev;
	struct lis2dg_sensor_data *sdata;

	mutex_init(&cdata->i2c_lock);
	mutex_init(&cdata->fifo_lock);

	cdata->fifo_data = 0;

	err = lis2dg_read_register(cdata, LIS2DG_WHO_AM_I_ADDR, 1, &wai);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");

		return err;
	}
	if (wai != LIS2DG_WHO_AM_I_DEF) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");

		return -ENODEV;
	}

	if (irq > 0) {
		cdata->irq = irq;
#ifdef CONFIG_OF
		err = lis2dg_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct lis2dg_platform_data *)
					cdata->dev->platform_data)->drdy_int_pin;

			if ((cdata->drdy_int_pin > 2) || (cdata->drdy_int_pin < 1))
				cdata->drdy_int_pin = 1;
		} else
			cdata->drdy_int_pin = 1;
#endif /* CONFIG_OF */

		dev_info(cdata->dev, "driver use DRDY int pin %d\n",
						cdata->drdy_int_pin);
	}

	cdata->common_odr = 0;
	cdata->enabled_sensor = 0;

	/*
	 * Select sensor power mode operation.
	 *
	 * - LIS2DG_LP_MODE: Low Power. The output data are 10 bits encoded.
	 * - LIS2DG_HR_MODE: High Resolution. 14 bits output data encoding.
	 */
	cdata->power_mode = LIS2DG_MODE_DEFAULT;

	for (i = 0; i < LIS2DG_SENSORS_NUMB; i++) {
		piio_dev = iio_device_alloc(sizeof(struct lis2dg_sensor_data *));
		if (piio_dev == NULL) {
			err = -ENOMEM;

			goto iio_device_free;
		}

		cdata->iio_sensors_dev[i] = piio_dev;
		sdata = iio_priv(piio_dev);
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = lis2dg_sensors_table[i].name;
		sdata->odr = lis2dg_sensors_table[i].min_odr_hz;

		piio_dev->channels = lis2dg_sensors_table[i].iio_channel;
		piio_dev->num_channels = lis2dg_sensors_table[i].iio_channel_size;
		piio_dev->info = &lis2dg_info[i];
		piio_dev->modes = INDIO_DIRECT_MODE;
		piio_dev->name = kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
								sdata->name);
	}

	err = lis2dg_init_sensors(cdata);
	if (err < 0)
		goto iio_device_free;

	err = lis2dg_allocate_rings(cdata);
	if (err < 0)
		goto iio_device_free;

	if (irq > 0) {
		err = lis2dg_allocate_triggers(cdata, LIS2DG_TRIGGER_OPS);
		if (err < 0)
			goto deallocate_ring;
	}

	for (n = 0; n < LIS2DG_SENSORS_NUMB; n++) {
		err = iio_device_register(cdata->iio_sensors_dev[n]);
		if (err)
			goto iio_device_unregister_and_trigger_deallocate;
	}

	dev_info(cdata->dev, "%s: probed\n", LIS2DG_DEV_NAME);
	return 0;

iio_device_unregister_and_trigger_deallocate:
	for (n--; n >= 0; n--)
		iio_device_unregister(cdata->iio_sensors_dev[n]);

deallocate_ring:
	lis2dg_deallocate_rings(cdata);

iio_device_free:
	for (i--; i >= 0; i--)
		iio_device_free(cdata->iio_sensors_dev[i]);

	return err;
}
EXPORT_SYMBOL(lis2dg_common_probe);

void lis2dg_common_remove(struct lis2dg_data *cdata, int irq)
{
	int i;

	for (i = 0; i < LIS2DG_SENSORS_NUMB; i++)
		iio_device_unregister(cdata->iio_sensors_dev[i]);

	if (irq > 0)
		lis2dg_deallocate_triggers(cdata);

	lis2dg_deallocate_rings(cdata);

	for (i = 0; i < LIS2DG_SENSORS_NUMB; i++)
		iio_device_free(cdata->iio_sensors_dev[i]);
}
EXPORT_SYMBOL(lis2dg_common_remove);

#ifdef CONFIG_PM
int lis2dg_common_suspend(struct lis2dg_data *cdata)
{
	return 0;
}
EXPORT_SYMBOL(lis2dg_common_suspend);

int lis2dg_common_resume(struct lis2dg_data *cdata)
{
	return 0;
}
EXPORT_SYMBOL(lis2dg_common_resume);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("STMicroelectronics lis2dg i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
