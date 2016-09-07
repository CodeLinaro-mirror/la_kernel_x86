/*
 * STMicroelectronics lsm6ds3h buffer driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

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

#include "st_lsm6ds3h.h"

#define ST_LSM6DS3H_ENABLE_AXIS			0x07
#define ST_LSM6DS3H_FIFO_DIFF_L			0x3a
#define ST_LSM6DS3H_FIFO_DIFF_MASK		0x07ff /* 4kbyte fifo for lsm6ds3h */
#define ST_LSM6DS3H_FIFO_DATA_OUT_L		0x3e
#define ST_LSM6DS3H_FIFO_DATA_OVR		0x4000
#define ST_LSM6DS3H_FIFO_DATA_EMPTY		0x1000
#define ST_LSM6DS3H_FIFO_DATA_PATTERN_L		0x3c
#define ST_LSM6DS3H_FIFO_DATA_DEADLOCK_TRIGGER	10

static int st_lsm6ds3h_do_div(struct lsm6ds3h_data *cdata,
					u16 read_len,
					bool discard_data,
					int64_t *accel_deltatime, int64_t *gyro_deltatime
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
					, int64_t *ext0_deltatime
#endif
					)

{
	u8 gyro_sip, accel_sip;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	u8 ext0_sip;
#endif
	u16 byte_in_pattern = cdata->byte_in_pattern;
	u16 pattern_num;
	int64_t pattern_timestamp;

	accel_sip = cdata->fifo_output[ST_MASK_ID_ACCEL].sip;
	gyro_sip = cdata->fifo_output[ST_MASK_ID_GYRO].sip;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	ext0_sip = cdata->fifo_output[ST_MASK_ID_EXT0].sip;
#endif


	if (byte_in_pattern)
		pattern_num = read_len / byte_in_pattern;
	else {
		dev_err(cdata->dev, "st_lsm6ds3h_do_div byte_in_pattern equal 0\n");
		return -EINVAL;
	}
	if (pattern_num) {
		if (discard_data) {
			pattern_timestamp = accel_sip ? (*accel_deltatime * accel_sip) :
						(gyro_sip ? (*gyro_deltatime * gyro_sip) :
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
						(ext0_sip ? (*ext0_deltatime * ext0_sip) : 0));
#else
						0);
#endif
			cdata->last_timestamp = cdata->timestamp - pattern_timestamp * pattern_num;
		} else {
			pattern_timestamp = (cdata->timestamp - cdata->last_timestamp) / pattern_num;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
			if (ext0_sip)
				*ext0_deltatime = pattern_timestamp / ext0_sip;
#endif
			if (gyro_sip)
				*gyro_deltatime = pattern_timestamp / gyro_sip;
			if (accel_sip)
				*accel_deltatime = pattern_timestamp / accel_sip;
		}

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
		cdata->fifo_output[ST_MASK_ID_EXT0].timestamp =
#endif
		cdata->fifo_output[ST_MASK_ID_GYRO].timestamp = cdata->fifo_output[ST_MASK_ID_ACCEL].timestamp = cdata->last_timestamp;

		dev_dbg(cdata->dev, "st_lsm6ds3h_do_div"
					"[%d] [%d] [%d] [%d] [%d] [%lld] [%lld] [%lld] [%lld]\n",
					discard_data, gyro_sip, accel_sip, byte_in_pattern, read_len,
					*gyro_deltatime, *accel_deltatime,
					cdata->last_timestamp, cdata->timestamp);
	}

	return 0;
}


void st_lsm6ds3h_push_data_with_timestamp(struct lsm6ds3h_data *cdata,
					u8 index, u8 *data, int64_t timestamp)
{
	int i, n = 0;
	struct iio_chan_spec const *chs = cdata->indio_dev[index]->channels;
	uint16_t bfch, bfchs_out = 0, bfchs_in = 0;
	struct lsm6ds3h_sensor_data *sdata = iio_priv(cdata->indio_dev[index]);

	if (!sdata || !sdata->buffer_data) {
		return;
	}

	for (i = 0; i < sdata->num_data_channels; i++) {
		bfch = chs[i].scan_type.storagebits >> 3;

		if (test_bit(i, cdata->indio_dev[index]->active_scan_mask)) {
			memcpy(&sdata->buffer_data[bfchs_out],
							&data[bfchs_in], bfch);
			n++;
			bfchs_out += bfch;
		}

		bfchs_in += bfch;
	}

	if (cdata->indio_dev[index]->scan_timestamp)
		*(s64 *)((u8 *)sdata->buffer_data +
			ALIGN(bfchs_out, sizeof(s64))) = timestamp;

	iio_push_to_buffers(cdata->indio_dev[index], sdata->buffer_data);
}

static void st_lsm6ds3h_parse_fifo_data(struct lsm6ds3h_data *cdata, u16 read_len, bool discard_data,
		u16 fifo_offset)
{
	int deadlock_detector = 0;
	u8 gyro_sip, accel_sip;
	int64_t accel_deltatime;
	int64_t gyro_deltatime;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
	u8 ext0_sip;
	int64_t ext0_deltatime;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

	dev_dbg(cdata->dev, "st_lsm6ds3h_parse_fifo_data: sensors_enabled=0x%2x\n",
					cdata->sensors_enabled);

	if (st_lsm6ds3h_do_div(cdata, read_len, discard_data, &accel_deltatime, &gyro_deltatime
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
						, &ext0_deltatime
#endif
						) < 0)
			 return;


	while ((fifo_offset < read_len) && (deadlock_detector++ < ST_LSM6DS3H_FIFO_DATA_DEADLOCK_TRIGGER)) {
	        dev_dbg(cdata->dev, "st_lsm6ds3h_parse_fifo_data: deadlock=%d read_len=%d fifo_offset=%d\n", deadlock_detector, read_len, fifo_offset);
		gyro_sip = cdata->fifo_output[ST_MASK_ID_GYRO].sip;
		accel_sip = cdata->fifo_output[ST_MASK_ID_ACCEL].sip;
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
		ext0_sip = cdata->fifo_output[ST_MASK_ID_EXT0].sip;
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

		do {
			if (gyro_sip > 0) {
				if (cdata->samples_to_discard[ST_MASK_ID_GYRO] > 0)
					cdata->samples_to_discard[ST_MASK_ID_GYRO]--;
				else {
					cdata->fifo_output[ST_MASK_ID_GYRO].num_samples++;

					if (cdata->fifo_output[ST_MASK_ID_GYRO].num_samples >= cdata->fifo_output[ST_MASK_ID_GYRO].decimator) {
						cdata->fifo_output[ST_MASK_ID_GYRO].timestamp_p = cdata->fifo_output[ST_MASK_ID_GYRO].timestamp;
						cdata->fifo_output[ST_MASK_ID_GYRO].num_samples = 0;
						if (cdata->sensors_enabled & BIT(ST_MASK_ID_GYRO)) {
							st_lsm6ds3h_push_data_with_timestamp(
								cdata, ST_MASK_ID_GYRO,
								&cdata->fifo_data[fifo_offset],
								cdata->fifo_output[ST_MASK_ID_GYRO].timestamp);
						}
						if (cdata->sensors_enabled & BIT(ST_MASK_ID_GYRO_WK)) {
							st_lsm6ds3h_push_data_with_timestamp(
								cdata, ST_MASK_ID_GYRO_WK,
								&cdata->fifo_data[fifo_offset],
								cdata->fifo_output[ST_MASK_ID_GYRO].timestamp);
						}
					}
				}

				cdata->fifo_output[ST_MASK_ID_GYRO].timestamp += gyro_deltatime;
				fifo_offset += ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE;
				gyro_sip--;
				deadlock_detector = 0;
			}

			if (accel_sip > 0) {
				if (cdata->samples_to_discard[ST_MASK_ID_ACCEL] > 0)
					cdata->samples_to_discard[ST_MASK_ID_ACCEL]--;
				else {
					cdata->fifo_output[ST_MASK_ID_ACCEL].num_samples++;

					if (cdata->fifo_output[ST_MASK_ID_ACCEL].num_samples >= cdata->fifo_output[ST_MASK_ID_ACCEL].decimator) {
						cdata->fifo_output[ST_MASK_ID_ACCEL].timestamp_p = cdata->fifo_output[ST_MASK_ID_ACCEL].timestamp;
						cdata->fifo_output[ST_MASK_ID_ACCEL].num_samples = 0;
						if (cdata->sensors_enabled & BIT(ST_MASK_ID_ACCEL)) {
							st_lsm6ds3h_push_data_with_timestamp(
								cdata, ST_MASK_ID_ACCEL,
								&cdata->fifo_data[fifo_offset],
								cdata->fifo_output[ST_MASK_ID_ACCEL].timestamp);
						}
						if (cdata->sensors_enabled & BIT(ST_MASK_ID_ACCEL_WK)) {
							st_lsm6ds3h_push_data_with_timestamp(
								cdata, ST_MASK_ID_ACCEL_WK,
								&cdata->fifo_data[fifo_offset],
								cdata->fifo_output[ST_MASK_ID_ACCEL].timestamp);
						}
					}
				}

				cdata->fifo_output[ST_MASK_ID_ACCEL].timestamp += accel_deltatime;
				fifo_offset += ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE;
				accel_sip--;
				deadlock_detector = 0;
			}

#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
			if (ext0_sip > 0) {
				if (cdata->samples_to_discard[ST_MASK_ID_EXT0] > 0)
					cdata->samples_to_discard[ST_MASK_ID_EXT0]--;
				else {
					cdata->fifo_output[ST_MASK_ID_EXT0].num_samples++;

					if (cdata->fifo_output[ST_MASK_ID_EXT0].num_samples >= cdata->fifo_output[ST_MASK_ID_EXT0].decimator) {
						cdata->fifo_output[ST_MASK_ID_EXT0].timestamp_p = cdata->fifo_output[ST_MASK_ID_EXT0].timestamp;
						cdata->fifo_output[ST_MASK_ID_EXT0].num_samples = 0;
						st_lsm6ds3h_push_data_with_timestamp(
							cdata, ST_MASK_ID_EXT0,
							&cdata->fifo_data[fifo_offset],
							cdata->fifo_output[ST_MASK_ID_EXT0].timestamp);
					}
				}

				cdata->fifo_output[ST_MASK_ID_EXT0].timestamp += ext0_deltatime;
				fifo_offset += ST_LSM6DS3H_FIFO_ELEMENT_LEN_BYTE;
				ext0_sip--;
				deadlock_detector = 0;
			}

		} while ((accel_sip > 0) || (gyro_sip > 0) || (ext0_sip > 0));
#else /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
		} while ((accel_sip > 0) || (gyro_sip > 0));
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */
	}
}

int st_lsm6ds3h_read_fifo(struct lsm6ds3h_data *cdata, int flags)
{
	bool overrun_flag = false;
	bool want_to_discard = flags & READ_FIFO_DISCARD_DATA;
	bool discard_data = false;
	int err;
	u16 pattern, offset = 0, discard_len;
#if (CONFIG_ST_LSM6DS3H_IIO_LIMIT_FIFO > 0)
	u16 data_remaining, data_to_read, byte_in_pattern;
#endif /* CONFIG_ST_LSM6DS3H_IIO_LIMIT_FIFO */
	u16 read_len = cdata->fifo_watermark * ST_LSM6DS3H_BYTE_FOR_CHANNEL;

	dev_dbg(cdata->dev, "st_lsm6ds3h_read_fifo, flags=0x%2x\n", flags);

	byte_in_pattern = cdata->byte_in_pattern;

	if (byte_in_pattern == 0)
		return 0;

	dev_dbg(cdata->dev, "sip %d:%d byte_in_pattern:%d\n",
		   cdata->fifo_output[ST_MASK_ID_ACCEL].sip,
		   cdata->fifo_output[ST_MASK_ID_GYRO].sip, byte_in_pattern);

	err = cdata->tf->read(cdata, ST_LSM6DS3H_FIFO_DIFF_L,
				2, (u8 *)&read_len, true);
	if (err < 0)
		return err;

	dev_dbg(cdata->dev, "data fifo read_len=0x%x.\n", read_len);

	if (read_len & ST_LSM6DS3H_FIFO_DATA_OVR) {
		want_to_discard = true;
		overrun_flag = true;
		dev_err(cdata->dev,
			"data fifo overrun, read_len=%d.\n", read_len);

		if ((read_len & ST_LSM6DS3H_FIFO_DIFF_MASK) == 0)
			read_len = ST_LSM6DS3H_FIFO_DIFF_MASK;
	}

	if (read_len & ST_LSM6DS3H_FIFO_DATA_EMPTY) {
		dev_dbg(cdata->dev, "read_fifo data empty!\n");
		return 0;
	}

	if (flags != READ_FIFO_IN_INTERRUPT) {

		if (!(flags & READ_FIFO_IN_INTERRUPT)) {
			cdata->last_timestamp = cdata->timestamp;
			cdata->timestamp = ktime_to_ns(ktime_get_boottime());
		}
		if (want_to_discard) {
			dev_dbg(cdata->dev,
				"want_to_discard %d, %d.\n", read_len, cdata->fifo_watermark);
			read_len &= ST_LSM6DS3H_FIFO_DIFF_MASK;
			if (read_len > (cdata->fifo_watermark + byte_in_pattern) / ST_LSM6DS3H_BYTE_FOR_CHANNEL) {
				discard_len = read_len - cdata->fifo_watermark / ST_LSM6DS3H_BYTE_FOR_CHANNEL;
				discard_data = true;
			} else
				goto read_fifo_report;

			discard_len *= ST_LSM6DS3H_BYTE_FOR_CHANNEL;
			discard_len = (discard_len / byte_in_pattern) * byte_in_pattern;

			dev_dbg(cdata->dev,
				"prepare to discard %d data.\n", discard_len);
			err = cdata->tf->read(cdata, ST_LSM6DS3H_FIFO_DATA_OUT_L,
						discard_len, cdata->fifo_data, true);
			if (err < 0)
				return err;

			cdata->last_timestamp = cdata->timestamp;
			cdata->timestamp = ktime_to_ns(ktime_get_boottime());
			err = cdata->tf->read(cdata, ST_LSM6DS3H_FIFO_DIFF_L,
						2, (u8 *)&read_len, true);
			if (err < 0) {
				if (overrun_flag) {
					st_lsm6ds3h_set_fifo_mode(cdata, BYPASS);
					st_lsm6ds3h_set_fifo_mode(cdata, CONTINUOS);
				}
				return err;
			}
			dev_dbg(cdata->dev,
				"after discard data, read_len=%d.\n", read_len);
		}
	}

read_fifo_report:

	read_len &= ST_LSM6DS3H_FIFO_DIFF_MASK;
	read_len *= ST_LSM6DS3H_BYTE_FOR_CHANNEL;

	read_len = (read_len / byte_in_pattern) * byte_in_pattern;
	if (read_len == 0)
		return 0;

	dev_dbg(cdata->dev, "st_lsm6ds3h_read_fifo read:%d fifo_watermark:%d\n",
		read_len, cdata->fifo_watermark);

	err = cdata->tf->read(cdata, ST_LSM6DS3H_FIFO_DATA_OUT_L,
				read_len,
				cdata->fifo_data, true);
	if (err < 0)
		return err;


	if (overrun_flag) {
		err = cdata->tf->read(cdata, ST_LSM6DS3H_FIFO_DATA_PATTERN_L,
					2, (u8 *)&pattern, true);
		if (err < 0)
			return err;
		pattern &= 0x03FF;
		/* byte_in_pattern/ST_LSM6DS3H_BYTE_FOR_CHANNEL is the number
		 * of axis in a "pattern".
		 * offset is the number of bytes to be discarded when an
		 * overrun condition happens.
		 */
		offset = ((byte_in_pattern/ST_LSM6DS3H_BYTE_FOR_CHANNEL) - pattern)
			* ST_LSM6DS3H_BYTE_FOR_CHANNEL;
		dev_info(cdata->dev, "FIFO overrun, offset=%d", offset);
		if (offset != byte_in_pattern) {
			read_len -= byte_in_pattern;
		} else {
			offset = 0;
		}

		st_lsm6ds3h_set_fifo_mode(cdata, BYPASS);
		st_lsm6ds3h_set_fifo_mode(cdata, CONTINUOS);
	}

	st_lsm6ds3h_parse_fifo_data(cdata, read_len, discard_data, offset);

	return 0;
}

int lsm6ds3h_read_output_data(struct lsm6ds3h_data *cdata, int sindex, bool push)
{
	int err;
	u8 data[6];
	struct iio_dev *indio_dev = cdata->indio_dev[sindex];
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	err = cdata->tf->read(cdata, sdata->data_out_reg,
				ST_LSM6DS3H_BYTE_FOR_CHANNEL * 3, data, true);
	if (err < 0)
		return err;

	if (push)
		st_lsm6ds3h_push_data_with_timestamp(cdata, sindex,
							data, cdata->timestamp);

	return 0;
}
EXPORT_SYMBOL(lsm6ds3h_read_output_data);

static irqreturn_t st_lsm6ds3h_outdata_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static irqreturn_t st_lsm6ds3h_step_counter_trigger_handler(int irq, void *p)
{
	int err;
	struct timespec ts;
	int64_t timestamp = 0;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	dev_dbg(sdata->cdata->dev, "st_lsm6ds3h_step_counter_trigger_handler\n");
	if (!sdata->cdata->reset_steps) {
		err = sdata->cdata->tf->read(sdata->cdata,
					(u8)indio_dev->channels[0].address,
					ST_LSM6DS3H_BYTE_FOR_CHANNEL,
					sdata->buffer_data, true);
		if (err < 0)
			goto st_lsm6ds3h_step_counter_done;

		timestamp = sdata->cdata->timestamp;
	} else {
		memset(sdata->buffer_data, 0, ST_LSM6DS3H_BYTE_FOR_CHANNEL);
		get_monotonic_boottime(&ts);
		timestamp = timespec_to_ns(&ts);
		sdata->cdata->reset_steps = false;
	}

	if (indio_dev->scan_timestamp)
		*(s64 *)((u8 *)sdata->buffer_data +
				ALIGN(ST_LSM6DS3H_BYTE_FOR_CHANNEL,
						sizeof(s64))) = timestamp;

	iio_push_to_buffers(indio_dev, sdata->buffer_data);

st_lsm6ds3h_step_counter_done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static inline irqreturn_t st_lsm6ds3h_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}

int st_lsm6ds3h_trig_set_state(struct iio_trigger *trig, bool state)
{
	return 0;
}

static int st_lsm6ds3h_buffer_preenable(struct iio_dev *indio_dev)
{
#ifdef CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	if (sdata->cdata->injection_mode) {
		switch (sdata->sindex) {
		case ST_MASK_ID_ACCEL:
		case ST_MASK_ID_GYRO:
			return -EBUSY;

		default:
			break;
		}
	}
#endif /* CONFIG_ST_LSM6DS3H_XL_DATA_INJECTION */

	return iio_sw_buffer_preenable(indio_dev);
}

static int st_lsm6ds3h_buffer_postenable(struct iio_dev *indio_dev)
{
	int err, err2 = 0;
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	dev_dbg(sdata->cdata->dev, "st_lsm6ds3h_buffer_postenable: index=%d\n", sdata->sindex);

	switch (sdata->sindex) {
	case ST_MASK_ID_ACCEL:
	case ST_MASK_ID_ACCEL_WK:
	case ST_MASK_ID_GYRO:
	case ST_MASK_ID_GYRO_WK:
		if ((sdata->cdata->hwfifo_enabled[sdata->sindex]) &&
				(indio_dev->buffer->length <
					2 * ST_LSM6DS3H_MAX_FIFO_LENGHT))
			return -EINVAL;

		break;

	default:
		break;
	}

	sdata->buffer_data = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	if (!sdata->buffer_data)
		return -ENOMEM;

	mutex_lock(&sdata->cdata->odr_lock);

	err = st_lsm6ds3h_set_enable(sdata, true);
	if (err < 0) {
		mutex_unlock(&sdata->cdata->odr_lock);
		goto free_buffer_data;
	}

	mutex_unlock(&sdata->cdata->odr_lock);

	err = iio_triggered_buffer_postenable(indio_dev);
	if (err < 0)
		goto disable_sensor;

	if (sdata->sindex == ST_MASK_ID_STEP_COUNTER) {
		iio_trigger_poll_chained(
			sdata->cdata->trig[ST_MASK_ID_STEP_COUNTER], 0);
	}

	return 0;

disable_sensor:
	mutex_lock(&sdata->cdata->odr_lock);
	err2 = st_lsm6ds3h_set_enable(sdata, false);
	mutex_unlock(&sdata->cdata->odr_lock);
free_buffer_data:
	if (err2 >= 0) {
		kfree(sdata->buffer_data);
		sdata->buffer_data = NULL;
	}

	return err;
}

static int st_lsm6ds3h_buffer_predisable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3h_sensor_data *sdata = iio_priv(indio_dev);

	dev_dbg(sdata->cdata->dev, "st_lsm6ds3h_buffer_predisable: index=%d\n", sdata->sindex);

	mutex_lock(&sdata->cdata->odr_lock);

	err = st_lsm6ds3h_set_enable(sdata, false);
	if (err < 0) {
		mutex_unlock(&sdata->cdata->odr_lock);
		return err;
	}

	mutex_unlock(&sdata->cdata->odr_lock);

	err = iio_triggered_buffer_predisable(indio_dev);
	if (err < 0)
		goto reenable_sensor;

	err = st_lsm6ds3h_flush_work_fifo(sdata->cdata, true);
	if (err < 0) {
		dev_err(sdata->cdata->dev, "st_lsm6ds3h_flush_work_fifo return %d!\n", err);
		goto reenable_sensor;
	}

	kfree(sdata->buffer_data);
	sdata->buffer_data = NULL;

	return 0;

reenable_sensor:
	mutex_lock(&sdata->cdata->odr_lock);
	st_lsm6ds3h_set_enable(sdata, true);
	mutex_unlock(&sdata->cdata->odr_lock);

	return err;
}

static int st_lsm6ds3h_buffer_postdisable(struct iio_dev *indio_dev)
{
	return 0;
}

static const struct iio_buffer_setup_ops st_lsm6ds3h_buffer_setup_ops = {
	.preenable = &st_lsm6ds3h_buffer_preenable,
	.postenable = &st_lsm6ds3h_buffer_postenable,
	.predisable = &st_lsm6ds3h_buffer_predisable,
	.postdisable = &st_lsm6ds3h_buffer_postdisable,
};

int st_lsm6ds3h_allocate_rings(struct lsm6ds3h_data *cdata)
{
	int err;
	struct lsm6ds3h_sensor_data *sdata;

	sdata = iio_priv(cdata->indio_dev[ST_MASK_ID_ACCEL]);

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_MASK_ID_ACCEL],
				NULL, &st_lsm6ds3h_outdata_trigger_handler,
				&st_lsm6ds3h_buffer_setup_ops);
	if (err < 0)
		return err;

	sdata = iio_priv(cdata->indio_dev[ST_MASK_ID_ACCEL_WK]);

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_MASK_ID_ACCEL_WK],
				NULL, &st_lsm6ds3h_outdata_trigger_handler,
				&st_lsm6ds3h_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_accel;

	sdata = iio_priv(cdata->indio_dev[ST_MASK_ID_GYRO]);

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_MASK_ID_GYRO],
				NULL, &st_lsm6ds3h_outdata_trigger_handler,
				&st_lsm6ds3h_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_accel_wk;

	sdata = iio_priv(cdata->indio_dev[ST_MASK_ID_GYRO_WK]);

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_MASK_ID_GYRO_WK],
				NULL, &st_lsm6ds3h_outdata_trigger_handler,
				&st_lsm6ds3h_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_gyro;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_MASK_ID_SIGN_MOTION],
				&st_lsm6ds3h_handler_empty, NULL,
				&st_lsm6ds3h_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_gyro_wk;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_MASK_ID_STEP_COUNTER],
				NULL,
				&st_lsm6ds3h_step_counter_trigger_handler,
				&st_lsm6ds3h_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_sign_motion;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_MASK_ID_STEP_DETECTOR],
				&st_lsm6ds3h_handler_empty, NULL,
				&st_lsm6ds3h_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_step_counter;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_MASK_ID_TILT],
				&st_lsm6ds3h_handler_empty, NULL,
				&st_lsm6ds3h_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_step_detector;

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (cdata->wrist_tilt_available) {
		err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_MASK_ID_WRIST_TILT],
				&st_lsm6ds3h_handler_empty, NULL,
				&st_lsm6ds3h_buffer_setup_ops);
		if (err < 0)
			goto buffer_cleanup_tilt;
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_MASK_ID_TAP_TAP],
				&st_lsm6ds3h_handler_empty, NULL,
				&st_lsm6ds3h_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_tap_tap;
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

	return 0;

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
buffer_cleanup_tap_tap:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_MASK_ID_WRIST_TILT]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
buffer_cleanup_tilt:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_MASK_ID_TILT]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */
buffer_cleanup_step_detector:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_MASK_ID_STEP_DETECTOR]);
buffer_cleanup_step_counter:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_MASK_ID_STEP_COUNTER]);
buffer_cleanup_sign_motion:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_MASK_ID_SIGN_MOTION]);
buffer_cleanup_gyro_wk:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_GYRO_WK]);
buffer_cleanup_gyro:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_GYRO]);
buffer_cleanup_accel_wk:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_ACCEL_WK]);
buffer_cleanup_accel:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_ACCEL]);
	return err;
}

void st_lsm6ds3h_deallocate_rings(struct lsm6ds3h_data *cdata)
{
#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_TAP_TAP]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (cdata->wrist_tilt_available)
		iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_WRIST_TILT]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_TILT]);
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_MASK_ID_STEP_DETECTOR]);
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_MASK_ID_STEP_COUNTER]);
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_MASK_ID_SIGN_MOTION]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_ACCEL]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_ACCEL_WK]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_GYRO]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_MASK_ID_GYRO_WK]);
}

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3h buffer driver");
MODULE_LICENSE("GPL v2");
