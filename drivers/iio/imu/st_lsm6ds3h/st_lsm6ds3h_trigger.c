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

#define ST_LSM6DS3H_TAP_SRC_ADDR			0x1C
#define ST_LSM6DS3H_TAP_SRC_SINGLE_TAP_MASK		0x20
#define ST_LSM6DS3H_TAP_SRC_DOUBLE_TAP_MASK		0x10
#define ST_LSM6DS3H_TAP_SRC_TAP_SIGN_MASK		0x08
#define ST_LSM6DS3H_TAP_SRC_X_TAP_MASK			0x04
#define ST_LSM6DS3H_TAP_SRC_Y_TAP_MASK			0x02
#define ST_LSM6DS3H_TAP_SRC_Z_TAP_MASK			0x01
#define ST_LSM6DS3H_TAP_SRC_ANY_TAP_MASK		0x77

static struct mutex lsm6ds3h_irq_mutex;
static struct workqueue_struct *st_lsm6ds3h_wq;

void st_lsm6ds3h_flush_workqueue(struct lsm6ds3h_data *cdata)
{
	dev_dbg(cdata->dev, "st_lsm6ds3h_flush_workqueue!\n");
	flush_workqueue(st_lsm6ds3h_wq);
}

irqreturn_t lsm6ds3h_save_timestamp(int irq, void *private)
{
	struct lsm6ds3h_data *cdata = private;
	struct timespec ts;

	cdata->last_timestamp = cdata->timestamp;
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
	u8 src_tap_tap = 0;
	int flags = READ_FIFO_IN_INTERRUPT;

	cdata = container_of((struct work_struct *)data_work,
						struct lsm6ds3h_data, data_work);

	mutex_lock(&lsm6ds3h_irq_mutex);
	err = cdata->tf->read(cdata, ST_LSM6DS3H_SRC_FUNC_ADDR,
						1, &src_dig_func, true);
	if (err < 0) {
		mutex_unlock(&lsm6ds3h_irq_mutex);
		goto exit_irq;
	}

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	err = cdata->tf->read(cdata, ST_LSM6DS3H_TAP_SRC_ADDR,
						1, &src_tap_tap, true);
	if (err < 0) {
		mutex_unlock(&lsm6ds3h_irq_mutex);
		goto exit_irq;
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

	mutex_unlock(&lsm6ds3h_irq_mutex);

	dev_dbg(cdata->dev, "lsm6ds3h_irq_management src_dig_func=%x\n", src_dig_func);

	if ((cdata->sensors_enabled & ~cdata->sensors_use_fifo) &
			(BIT(ST_MASK_ID_ACCEL) | BIT(ST_MASK_ID_ACCEL_WK) |
				BIT(ST_MASK_ID_GYRO) | BIT(ST_MASK_ID_GYRO_WK) |
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
					(BIT(ST_MASK_ID_ACCEL) | BIT(ST_MASK_ID_ACCEL_WK))) {
				cdata->nofifo_decimation[ST_MASK_ID_ACCEL].num_samples++;

				if ((cdata->nofifo_decimation[ST_MASK_ID_ACCEL].num_samples %
						cdata->nofifo_decimation[ST_MASK_ID_ACCEL].decimator) == 0) {
					push = true;
					cdata->nofifo_decimation[ST_MASK_ID_ACCEL].num_samples = 0;
				} else
					push = false;

				if ((cdata->sensors_enabled & ~cdata->sensors_use_fifo) & BIT(ST_MASK_ID_ACCEL))
					lsm6ds3h_read_output_data(cdata, ST_MASK_ID_ACCEL, push);
				if ((cdata->sensors_enabled & ~cdata->sensors_use_fifo) & BIT(ST_MASK_ID_ACCEL_WK))
					lsm6ds3h_read_output_data(cdata, ST_MASK_ID_ACCEL_WK, push);
			} else {
				if (force_read_accel)
					lsm6ds3h_read_output_data(cdata, ST_MASK_ID_ACCEL, false);
			}

		}

		if (src_accel_gyro & ST_LSM6DS3H_GYRO_DATA_AVL) {
			if ((cdata->sensors_enabled & ~cdata->sensors_use_fifo) & BIT(ST_MASK_ID_GYRO))
				lsm6ds3h_read_output_data(cdata, ST_MASK_ID_GYRO, true);
			if ((cdata->sensors_enabled & ~cdata->sensors_use_fifo) & BIT(ST_MASK_ID_GYRO_WK))
				lsm6ds3h_read_output_data(cdata, ST_MASK_ID_GYRO_WK, true);
		}
	}

read_fifo_status:
	if (cdata->sensors_use_fifo) {
		mutex_lock(&cdata->fifo_lock);
		if (cdata->system_state & SF_RESUME) {
			flags |= READ_FIFO_IN_RESUME | READ_FIFO_DISCARD_DATA;
			cdata->system_state = SF_NORMAL;
		}
		st_lsm6ds3h_read_fifo(cdata, flags);
		mutex_unlock(&cdata->fifo_lock);
	}

	if (src_dig_func & ST_LSM6DS3H_SRC_STEP_DETECTOR_DATA_AVL) {
		dev_dbg(cdata->dev, "ST_LSM6DS3H_SRC_STEP_DETECTOR_DATA_AVL\n");
		iio_push_event(cdata->indio_dev[ST_MASK_ID_STEP_DETECTOR],
				IIO_UNMOD_EVENT_CODE(IIO_STEP_DETECTOR,
				0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER),
				cdata->timestamp);

	}

	if (src_dig_func & ST_LSM6DS3H_SRC_SIGN_MOTION_DATA_AVL) {
		dev_dbg(cdata->dev, "ST_LSM6DS3H_SRC_SIGN_MOTION_DATA_AVL\n");
		iio_push_event(cdata->indio_dev[ST_MASK_ID_SIGN_MOTION],
				IIO_UNMOD_EVENT_CODE(IIO_SIGN_MOTION,
				0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER),
				cdata->timestamp);
	}

	if (src_dig_func & ST_LSM6DS3H_SRC_STEP_COUNTER_DATA_AVL) {
		dev_dbg(cdata->dev, "ST_LSM6DS3H_SRC_STEP_COUNTER_DATA_AVL\n");
		iio_trigger_poll_chained(cdata->trig[ST_MASK_ID_STEP_COUNTER], 0);
	}

	if (src_dig_func & ST_LSM6DS3H_SRC_TILT_DATA_AVL) {
		dev_dbg(cdata->dev, "ST_LSM6DS3H_SRC_TILT_DATA_AVL\n");
		iio_push_event(cdata->indio_dev[ST_MASK_ID_TILT],
				IIO_UNMOD_EVENT_CODE(IIO_TILT,
				0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER),
				cdata->timestamp);
	}

#ifdef CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT
	if (src_dig_func & ST_LSM6DS3H_SRC_WRIST_TILT_DATA_AVL) {
		dev_dbg(cdata->dev, "ST_LSM6DS3H_SRC_WRIST_TILT_DATA_AVL\n");
		iio_push_event(cdata->indio_dev[ST_MASK_ID_WRIST_TILT],
				IIO_UNMOD_EVENT_CODE(IIO_WRIST_TILT_GESTURE,
				0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER),
				cdata->timestamp);
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_ALGO_UPLOAD_WRIST_TILT */

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	if (src_tap_tap & ST_LSM6DS3H_TAP_SRC_ANY_TAP_MASK) {
		dev_dbg(cdata->dev, "ST_LSM6DS3H_TAP_TAP detected\n");
		iio_push_event(cdata->indio_dev[ST_MASK_ID_TAP_TAP],
				IIO_UNMOD_EVENT_CODE(IIO_TAP_TAP,
				0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER),
				cdata->timestamp);
	}
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

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

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	cdata->trig[ST_MASK_ID_TAP_TAP] = iio_trigger_alloc("%s-trigger",
			cdata->indio_dev[ST_MASK_ID_TAP_TAP]->name);
	if (!cdata->trig[ST_MASK_ID_TAP_TAP]) {
		dev_err(cdata->dev,
				"failed to allocate iio trigger for tap_tap.\n");
		err = -ENOMEM;
		goto free_trigger;
	}

	iio_trigger_set_drvdata(cdata->trig[ST_MASK_ID_TAP_TAP], cdata->indio_dev[ST_MASK_ID_TAP_TAP]);
	cdata->trig[ST_MASK_ID_TAP_TAP]->ops = trigger_ops;
	cdata->trig[ST_MASK_ID_TAP_TAP]->dev.parent = cdata->dev;

	err = iio_trigger_register(cdata->trig[ST_MASK_ID_TAP_TAP]);
	if (err < 0) {
		dev_err(cdata->dev, "failed to register iio trigger for tap_tap.\n");
		goto unregister_trigger;
	}
	cdata->indio_dev[ST_MASK_ID_TAP_TAP]->trig = cdata->trig[ST_MASK_ID_WRIST_TILT];
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */

	return 0;

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
unregister_trigger:
	iio_trigger_unregister(cdata->trig[ST_MASK_ID_TAP_TAP]);
	if (cdata->wrist_tilt_available)
		iio_trigger_unregister(cdata->trig[ST_MASK_ID_WRIST_TILT]);
free_trigger:
	iio_trigger_free(cdata->trig[ST_MASK_ID_TAP_TAP]);
	if (cdata->wrist_tilt_available)
		iio_trigger_free(cdata->trig[ST_MASK_ID_WRIST_TILT]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */
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

#ifdef CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED
	iio_trigger_unregister(cdata->trig[ST_MASK_ID_TAP_TAP]);
#endif /* CONFIG_ST_LSM6DS3H_IIO_TAP_TAP_ENABLED */
}
EXPORT_SYMBOL(st_lsm6ds3h_deallocate_triggers);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3h trigger driver");
MODULE_LICENSE("GPL v2");
