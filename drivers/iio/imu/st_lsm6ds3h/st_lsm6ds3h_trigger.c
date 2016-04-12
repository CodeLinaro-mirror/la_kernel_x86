/*
 * STMicroelectronics lsm6ds3h trigger driver
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
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>

#include "st_lsm6ds3h.h"

#define ST_LSM6DS3H_DIS_BIT				0x00
#define ST_LSM6DS3H_SRC_FUNC_ADDR			0x53
#define ST_LSM6DS3H_FIFO_DATA_AVL_ADDR			0x3b
#define ST_LSM6DS3H_ACCEL_DATA_AVL_ADDR			0x1e

#define ST_LSM6DS3H_ACCEL_DATA_AVL			0x01
#define ST_LSM6DS3H_GYRO_DATA_AVL			0x02
#define ST_LSM6DS3H_SRC_STEP_DETECTOR_DATA_AVL		0x10
#define ST_LSM6DS3H_SRC_SIGN_MOTION_DATA_AVL		0x40
#define ST_LSM6DS3H_SRC_TILT_DATA_AVL			0x20
#define ST_LSM6DS3H_SRC_WRIST_TILT_DATA_AVL		0x02
#define ST_LSM6DS3H_SRC_STEP_COUNTER_DATA_AVL		0x80
#define ST_LSM6DS3H_FIFO_DATA_AVL			0x80
#define ST_LSM6DS3H_FIFO_DATA_OVR			0x40

static struct mutex lsm6ds3h_irq_mutex;
static struct workqueue_struct *st_lsm6ds3h_wq;

irqreturn_t lsm6ds3h_save_timestamp(int irq, void *private)
{
	struct lsm6ds3h_data *cdata = private;
	struct timespec ts;

	get_monotonic_boottime(&ts);
	cdata->timestamp = timespec_to_ns(&ts);
	queue_work(st_lsm6ds3h_wq, &cdata->data_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static void lsm6ds3h_irq_management(struct work_struct *data_work)
{
	int err;
	bool push;
	bool force_read_accel = false;
	struct lsm6ds3h_data *cdata;
	u8 src_accel_gyro = 0, src_dig_func = 0;

	cdata = container_of((struct work_struct*)data_work,
						struct lsm6ds3h_data, data_work);

	mutex_lock(&lsm6ds3h_irq_mutex);
	err = cdata->tf->read(cdata, ST_LSM6DS3H_SRC_FUNC_ADDR,
						1, &src_dig_func, true);
	if (err < 0) {
		mutex_unlock(&lsm6ds3h_irq_mutex);
		goto exit_irq;
	}

	mutex_unlock(&lsm6ds3h_irq_mutex);

	if ((cdata->sensors_enabled & ~cdata->sensors_use_fifo) &
			(BIT(ST_MASK_ID_ACCEL) | BIT(ST_MASK_ID_GYRO) |
						BIT(ST_MASK_ID_EXT0))) {
		err = cdata->tf->read(cdata, ST_LSM6DS3H_ACCEL_DATA_AVL_ADDR,
						1, &src_accel_gyro, true);
		if (err < 0)
			goto read_fifo_status;

		if (src_accel_gyro & ST_LSM6DS3H_ACCEL_DATA_AVL) {
#ifdef CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT
			if ((cdata->sensors_enabled & ~cdata->sensors_use_fifo)
						& BIT(ST_MASK_ID_EXT0)) {
				cdata->nofifo_decimation[ST_MASK_ID_EXT0].num_samples++;
				force_read_accel = true;

				if ((cdata->nofifo_decimation[ST_MASK_ID_EXT0].num_samples %
						cdata->nofifo_decimation[ST_MASK_ID_EXT0].decimator) == 0) {
					push = true;
					cdata->nofifo_decimation[ST_MASK_ID_EXT0].num_samples = 0;
				} else
					push = false;

					lsm6ds3h_read_output_data(cdata, ST_MASK_ID_EXT0, push);
			}
#endif /* CONFIG_ST_LSM6DS3H_IIO_MASTER_SUPPORT */

			if ((cdata->sensors_enabled & ~cdata->sensors_use_fifo) &
							BIT(ST_MASK_ID_ACCEL)) {
				cdata->nofifo_decimation[ST_MASK_ID_ACCEL].num_samples++;

				if ((cdata->nofifo_decimation[ST_MASK_ID_ACCEL].num_samples %
						cdata->nofifo_decimation[ST_MASK_ID_ACCEL].decimator) == 0) {
					push = true;
					cdata->nofifo_decimation[ST_MASK_ID_ACCEL].num_samples = 0;
				} else
					push = false;

				lsm6ds3h_read_output_data(cdata, ST_MASK_ID_ACCEL, push);
			} else {
				if (force_read_accel)
					lsm6ds3h_read_output_data(cdata, ST_MASK_ID_ACCEL, false);
			}

		}

		if (src_accel_gyro & ST_LSM6DS3H_GYRO_DATA_AVL) {
			if ((cdata->sensors_enabled & ~cdata->sensors_use_fifo) & BIT(ST_MASK_ID_GYRO))
				lsm6ds3h_read_output_data(cdata, ST_MASK_ID_GYRO, true);
		}
	}

read_fifo_status:
	if (cdata->sensors_use_fifo)
		st_lsm6ds3h_read_fifo(cdata);

	if (src_dig_func & ST_LSM6DS3H_SRC_STEP_DETECTOR_DATA_AVL)
		st_lsm6ds3h_push_data_with_timestamp(cdata,
			ST_MASK_ID_STEP_DETECTOR, NULL, cdata->timestamp);

	if (src_dig_func & ST_LSM6DS3H_SRC_SIGN_MOTION_DATA_AVL)
		iio_push_event(cdata->indio_dev[ST_MASK_ID_SIGN_MOTION],
				IIO_UNMOD_EVENT_CODE(IIO_SIGN_MOTION,
				0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER),
				cdata->timestamp);

	if (src_dig_func & ST_LSM6DS3H_SRC_STEP_COUNTER_DATA_AVL)
		iio_trigger_poll_chained(cdata->trig[ST_MASK_ID_STEP_COUNTER], 0);

	if (src_dig_func & ST_LSM6DS3H_SRC_TILT_DATA_AVL)
		st_lsm6ds3h_push_data_with_timestamp(cdata,
				ST_MASK_ID_TILT, NULL, cdata->timestamp);

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (src_dig_func & ST_LSM6DS3H_SRC_WRIST_TILT_DATA_AVL) {
		iio_push_event(cdata->indio_dev[ST_MASK_ID_WRIST_TILT],
				IIO_UNMOD_EVENT_CODE(IIO_WRIST_TILT_GESTURE,
				0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER),
				cdata->timestamp);
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

exit_irq:
	enable_irq(cdata->irq);
}

int st_lsm6ds3h_allocate_triggers(struct lsm6ds3h_data *cdata,
				const struct iio_trigger_ops *trigger_ops)
{
	int err, i, n;

	mutex_init(&lsm6ds3h_irq_mutex);

	if (!st_lsm6ds3h_wq)
		st_lsm6ds3h_wq = create_workqueue(cdata->name);

	if (!st_lsm6ds3h_wq)
		return -EINVAL;

	INIT_WORK(&cdata->data_work, lsm6ds3h_irq_management);

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		cdata->trig[i] = iio_trigger_alloc("%s-trigger",
						cdata->indio_dev[i]->name);
		if (!cdata->trig[i]) {
			dev_err(cdata->dev,
					"failed to allocate iio trigger.\n");
			err = -ENOMEM;
			goto deallocate_trigger;
		}
		iio_trigger_set_drvdata(cdata->trig[i], cdata->indio_dev[i]);
		cdata->trig[i]->ops = trigger_ops;
		cdata->trig[i]->dev.parent = cdata->dev;
	}

	err = request_threaded_irq(cdata->irq, lsm6ds3h_save_timestamp, NULL,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					cdata->name, cdata);
	if (err)
		goto deallocate_trigger;

	for (n = 0; n < ST_INDIO_DEV_NUM; n++) {
		err = iio_trigger_register(cdata->trig[n]);
		if (err < 0) {
			dev_err(cdata->dev,
					"failed to register iio trigger.\n");
			goto free_irq;
		}
		cdata->indio_dev[n]->trig = cdata->trig[n];
	}

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (cdata->wrist_tilt_available) {
		cdata->trig[ST_MASK_ID_WRIST_TILT] = iio_trigger_alloc("%s-trigger",
				cdata->indio_dev[ST_MASK_ID_WRIST_TILT]->name);
		if (!cdata->trig[ST_MASK_ID_WRIST_TILT]) {
			dev_err(cdata->dev,
					"failed to allocate iio trigger.\n");
			err = -ENOMEM;
			goto free_irq;
		}

		iio_trigger_set_drvdata(cdata->trig[ST_MASK_ID_WRIST_TILT], cdata->indio_dev[ST_MASK_ID_WRIST_TILT]);
		cdata->trig[ST_MASK_ID_WRIST_TILT]->ops = trigger_ops;
		cdata->trig[ST_MASK_ID_WRIST_TILT]->dev.parent = cdata->dev;

		err = iio_trigger_register(cdata->trig[ST_MASK_ID_WRIST_TILT]);
		if (err < 0) {
			dev_err(cdata->dev, "failed to register iio trigger.\n");
			goto free_irq;
		}
		cdata->indio_dev[ST_MASK_ID_WRIST_TILT]->trig = cdata->trig[ST_MASK_ID_WRIST_TILT];
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

	return 0;

free_irq:
	free_irq(cdata->irq, cdata);
	for (n--; n >= 0; n--)
		iio_trigger_unregister(cdata->trig[n]);
deallocate_trigger:
	for (i--; i >= 0; i--)
		iio_trigger_free(cdata->trig[i]);

	return err;
}
EXPORT_SYMBOL(st_lsm6ds3h_allocate_triggers);

void st_lsm6ds3h_deallocate_triggers(struct lsm6ds3h_data *cdata)
{
	int i;

	free_irq(cdata->irq, cdata);

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_trigger_unregister(cdata->trig[i]);

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (cdata->wrist_tilt_available)
		iio_trigger_unregister(cdata->trig[ST_MASK_ID_WRIST_TILT]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */
}
EXPORT_SYMBOL(st_lsm6ds3h_deallocate_triggers);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3h trigger driver");
MODULE_LICENSE("GPL v2");
