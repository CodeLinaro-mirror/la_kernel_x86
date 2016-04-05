/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
*
* File Name          : lis2dg_trigger.c
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
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>

#include "lis2dg_core.h"

static struct workqueue_struct *lis2dg_workqueue;

void lis2dg_flush_works()
{
	flush_workqueue(lis2dg_workqueue);
}
EXPORT_SYMBOL(lis2dg_flush_works);

irqreturn_t lis2dg_save_timestamp(int irq, void *private)
{
	struct timespec ts;
	struct lis2dg_data *cdata = private;

	get_monotonic_boottime(&ts);
	cdata->timestamp = timespec_to_ns(&ts);
	queue_work(lis2dg_workqueue, &cdata->data_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

void lis2dg_event_management(struct lis2dg_data *cdata, u8 int_reg_val,
			     u8 ck_gate_val)
{
	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_TAP) &&
				(int_reg_val & LIS2DG_TAP_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DG_TAP],
					IIO_UNMOD_EVENT_CODE(IIO_TAP, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_DOUBLE_TAP) &&
					(int_reg_val & LIS2DG_DOUBLE_TAP_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DG_DOUBLE_TAP],
					IIO_UNMOD_EVENT_CODE(IIO_TAP_TAP, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);

	if (ck_gate_val & LIS2DG_FUNC_CK_GATE_STEP_D_MASK) {
		if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_STEP_D))
			iio_push_event(cdata->iio_sensors_dev[LIS2DG_STEP_D],
					IIO_UNMOD_EVENT_CODE(IIO_STEP_DETECTOR, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);

		if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_STEP_C))
			lis2dg_read_step_c(cdata);
	}

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_TILT) &&
			(ck_gate_val & LIS2DG_FUNC_CK_GATE_TILT_INT_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DG_TILT],
					IIO_UNMOD_EVENT_CODE(IIO_TILT, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DG_SIGN_M) &&
			(ck_gate_val & LIS2DG_FUNC_CK_GATE_SIGN_M_DET_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DG_SIGN_M],
					IIO_UNMOD_EVENT_CODE(IIO_SIGN_MOTION, 0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					cdata->timestamp);
}

void lis2dg_irq_management(struct work_struct *data_work)
{
	struct lis2dg_data *cdata;
	u8 status[4], func[2], fifo;

	cdata = container_of((struct work_struct *)data_work,
			     struct lis2dg_data, data_work);

	cdata->tf->read(cdata, LIS2DG_STATUS_DUP_ADDR, 4, status);
	cdata->tf->read(cdata, LIS2DG_FUNC_CK_GATE_ADDR, 2, func);
	cdata->tf->read(cdata, LIS2DG_FIFO_SRC_ADDR, 1, &fifo);

	if (fifo & LIS2DG_FIFO_SRC_FTH_MASK)
		lis2dg_read_fifo(cdata, true);

	if (status[0] & (LIS2DG_EVENT_MASK | LIS2DG_FUNC_CK_GATE_MASK))
		lis2dg_event_management(cdata, status[0], func[0]);

	enable_irq(cdata->irq);
}

int lis2dg_allocate_triggers(struct lis2dg_data *cdata,
			     const struct iio_trigger_ops *trigger_ops)
{
	int err, i, n;

	if (!lis2dg_workqueue)
		lis2dg_workqueue = create_workqueue(cdata->name);

	if (!lis2dg_workqueue)
		return -EINVAL;

	INIT_WORK(&cdata->data_work, lis2dg_irq_management);

	for (i = 0; i < LIS2DG_SENSORS_NUMB; i++) {
		cdata->iio_trig[i] = iio_trigger_alloc("%s-trigger",
						cdata->iio_sensors_dev[i]->name);
		if (!cdata->iio_trig[i]) {
			dev_err(cdata->dev, "failed to allocate iio trigger.\n");
			err = -ENOMEM;

			goto deallocate_trigger;
		}
		iio_trigger_set_drvdata(cdata->iio_trig[i],
						cdata->iio_sensors_dev[i]);
		cdata->iio_trig[i]->ops = trigger_ops;
		cdata->iio_trig[i]->dev.parent = cdata->dev;
	}

	err = request_threaded_irq(cdata->irq, lis2dg_save_timestamp, NULL,
				   IRQF_TRIGGER_HIGH, cdata->name, cdata);
	if (err)
		goto deallocate_trigger;

	for (n = 0; n < LIS2DG_SENSORS_NUMB; n++) {
		err = iio_trigger_register(cdata->iio_trig[n]);
		if (err < 0) {
			dev_err(cdata->dev, "failed to register iio trigger.\n");

			goto free_irq;
		}
		cdata->iio_sensors_dev[n]->trig = cdata->iio_trig[n];
	}

	return 0;

free_irq:
	free_irq(cdata->irq, cdata);
	for (n--; n >= 0; n--)
		iio_trigger_unregister(cdata->iio_trig[n]);
deallocate_trigger:
	for (i--; i >= 0; i--)
		iio_trigger_free(cdata->iio_trig[i]);

	return err;
}
EXPORT_SYMBOL(lis2dg_allocate_triggers);

void lis2dg_deallocate_triggers(struct lis2dg_data *cdata)
{
	int i;

	free_irq(cdata->irq, cdata);

	for (i = 0; i < LIS2DG_SENSORS_NUMB; i++)
		iio_trigger_unregister(cdata->iio_trig[i]);
}
EXPORT_SYMBOL(lis2dg_deallocate_triggers);

MODULE_DESCRIPTION("STMicroelectronics lis2dg i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
