/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
*
* File Name          : lis2dg_buffer.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "lis2dg_core.h"

#define LIS2DG_ACCEL_BUFFER_SIZE \
		ALIGN(LIS2DG_FIFO_BYTE_FOR_SAMPLE + LIS2DG_TIMESTAMP_SIZE, \
		      LIS2DG_TIMESTAMP_SIZE)
#define LIS2DG_STEP_C_BUFFER_SIZE \
		ALIGN(LIS2DG_FIFO_BYTE_X_AXIS + LIS2DG_TIMESTAMP_SIZE, \
		      LIS2DG_TIMESTAMP_SIZE)

void lis2dg_push_accel_fifo_data(struct lis2dg_data *cdata, u16 read_length)
{
	size_t offset;
	uint16_t i, j, k;
	u8 buffer[LIS2DG_ACCEL_BUFFER_SIZE], out_buf_index;
	struct iio_dev *indio_dev = cdata->iio_sensors_dev[LIS2DG_ACCEL];

	for (i = 0; i < read_length; i += LIS2DG_FIFO_BYTE_FOR_SAMPLE) {
		for (j = 0, out_buf_index = 0; j < LIS2DG_FIFO_NUM_AXIS; j++) {
			k = i + LIS2DG_FIFO_BYTE_X_AXIS * j;
			if (test_bit(j, indio_dev->active_scan_mask)) {
				memcpy(&buffer[out_buf_index], &cdata->fifo_data[k],
							LIS2DG_FIFO_BYTE_X_AXIS);
				out_buf_index += LIS2DG_FIFO_BYTE_X_AXIS;
			}
		}

		if (indio_dev->scan_timestamp) {
			offset = indio_dev->scan_bytes / sizeof(s64) - 1;
			((s64 *)buffer)[offset] = cdata->accel_timestamp;
			cdata->accel_timestamp += cdata->accel_deltatime;
		}

		iio_push_to_buffers(indio_dev, buffer);
	}
}

void lis2dg_read_fifo(struct lis2dg_data *cdata, bool check_fifo_len)
{
	int err;
	u8 fifo_src[2];
	u16 read_len = cdata->fifo_size;

	if (!cdata->fifo_data)
		return;

	if (check_fifo_len) {
		err = lis2dg_read_register(cdata, LIS2DG_FIFO_SRC, 2, fifo_src);
		if (err < 0)
			return;

		read_len = 0;
		read_len = (fifo_src[0] & LIS2DG_FIFO_SRC_DIFF_MASK) ?
								(1 << 8) : 0;
		read_len |= fifo_src[1];
		read_len *= LIS2DG_FIFO_BYTE_FOR_SAMPLE;

		if (read_len > cdata->fifo_size)
			read_len = cdata->fifo_size;
	}
	if (read_len == 0)
		return;

	err = lis2dg_read_register(cdata, LIS2DG_OUTX_L_ADDR, read_len,
							cdata->fifo_data);
	if (err < 0)
		return;

	cdata->accel_timestamp = cdata->timestamp;
	lis2dg_push_accel_fifo_data(cdata, read_len);
}

void lis2dg_read_step_c(struct lis2dg_data *cdata)
{
	int err;
	int64_t timestamp = 0;
	char buffer[LIS2DG_STEP_C_BUFFER_SIZE];
	struct iio_dev *indio_dev = cdata->iio_sensors_dev[LIS2DG_STEP_C];

	err = lis2dg_read_register(cdata, (u8)indio_dev->channels[0].address, 2,
					buffer);
	if (err < 0)
		goto lis2dg_step_counter_done;

	timestamp = cdata->timestamp;
	if (indio_dev->scan_timestamp)
		*(s64 *) ((u8 *) buffer +
			ALIGN(LIS2DG_FIFO_BYTE_X_AXIS, sizeof(s64))) = timestamp;

	iio_push_to_buffers(indio_dev, buffer);

lis2dg_step_counter_done:
	iio_trigger_notify_done(indio_dev->trig);
}

static inline irqreturn_t lis2dg_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}

int lis2dg_trig_set_state(struct iio_trigger *trig, bool state)
{
	int err;
	struct lis2dg_sensor_data *sdata;

	sdata = iio_priv(iio_trigger_get_drvdata(trig));
	err = lis2dg_update_drdy_irq(sdata, state);

	return (err < 0) ? err : 0;
}

static int lis2dg_buffer_preenable(struct iio_dev *indio_dev)
{
	int err;
	struct lis2dg_sensor_data *sdata = iio_priv(indio_dev);

	if (sdata->sindex == LIS2DG_ACCEL) {
		err = lis2dg_update_fifo(sdata->cdata);
		if (err < 0)
			return err;
	}

	err = lis2dg_set_enable(sdata, true);
	if (err < 0)
		return err;

	return iio_sw_buffer_preenable(indio_dev);
}

static int lis2dg_buffer_postdisable(struct iio_dev *indio_dev)
{
	int err, indx;
	struct lis2dg_sensor_data *sdata = iio_priv(indio_dev);

	err = lis2dg_set_enable(sdata, false);
	if (err < 0)
		return err;

	indx = sdata->sindex;
	if ((indx == LIS2DG_ACCEL) || (indx == LIS2DG_STEP_C)) {
		kfree(sdata->cdata->fifo_data);
		sdata->cdata->fifo_data = 0;
	}

	return 0;
}

static const struct iio_buffer_setup_ops lis2dg_buffer_setup_ops = {
	.preenable = &lis2dg_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &lis2dg_buffer_postdisable,
};

int lis2dg_allocate_rings(struct lis2dg_data *cdata)
{
	int err, i;

	for (i = 0; i < LIS2DG_SENSORS_NUMB; i++) {
		err = iio_triggered_buffer_setup(
				cdata->iio_sensors_dev[i],
				&lis2dg_handler_empty,
				NULL,
				&lis2dg_buffer_setup_ops);
		if (err < 0)
			goto buffer_cleanup;
	}

	return 0;

buffer_cleanup:
	for (i--; i >= 0; i--)
		iio_triggered_buffer_cleanup(cdata->iio_sensors_dev[i]);

	return err;
}

void lis2dg_deallocate_rings(struct lis2dg_data *cdata)
{
	int i;

	for (i = 0; i < LIS2DG_SENSORS_NUMB; i++)
		iio_triggered_buffer_cleanup(cdata->iio_sensors_dev[i]);
}

MODULE_DESCRIPTION("STMicroelectronics lis2dg i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
