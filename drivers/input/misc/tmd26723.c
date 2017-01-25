/*
 *  Linux kernel modules for proximity sensor
 *
 *  Copyright (C) 2016 Intel Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/input/tmd26723.h>
#include <asm/intel_scu_flis.h>
#include <linux/pm_runtime.h>
#define TMD26723_DRV_NAME	"tmd26723"

#define TMD26723_INT		IRQ_EINT20

#define TMD_STARTUP_DELAY		5 /* ms */
#define TMD26723_PS_DETECTION_THRESHOLD		0x100
#define TMD26723_PS_HYSTERESIS_THRESHOLD	0x100

/*
 * Defines
 */

#define TMD26723_ENABLE_REG	0x00
#define TMD26723_PTIME_REG	0x02
#define TMD26723_WTIME_REG	0x03
#define TMD26723_PILTL_REG	0x08
#define TMD26723_PILTH_REG	0x09
#define TMD26723_PIHTL_REG	0x0A
#define TMD26723_PIHTH_REG	0x0B
#define TMD26723_PERS_REG	0x0C
#define TMD26723_CONFIG_REG	0x0D
#define TMD26723_PPCOUNT_REG	0x0E
#define TMD26723_CONTROL_REG	0x0F
#define TMD26723_ID_REG		0x12
#define TMD26723_STATUS_REG	0x13
#define TMD26723_PDATAL_REG	0x18
#define TMD26723_PDATAH_REG	0x19
#define TMD26723_POFFSET	0x1E

#define CMD_BYTE		0x80
#define CMD_WORD		0xA0
#define CMD_SPECIAL		0xE0
#define CMD_CLR_PS_INT		0xE5

#define I2C_RETRY_COUNT		0x05
#define TMD_CMD_CLEAR		0x00
#define SET_MAX_THRESHOLD	1023
#define SET_LOW_THRESHOLD	0
#define PTIME_REG_VALUE		0xff
#define WTIME_REG_VALUE		0xDB
#define PPCOUNT_REG_VALUE	0x04
#define CONFIG_REG_VALUE	0x00
#define CONTROL_REG_VALUE	0x5c
#define PERS_REG_VALUE		0x30
#define DISABLE_REG_VALUE	0x00
#define ENABLE_REG_VALUE	0x2d
#define PROXIMITY_INTERRUPT	0X20
/*
 * Structs
 */
struct tmd26723_settings {
	int proximity_odr;
};

struct tmd26723_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct mutex sysfs_lock;
        struct mutex change_ps_lock;
	struct mutex irq_lock;
	spinlock_t lock;
	struct delayed_work	dwork;	/* for PS interrupt */
	struct input_dev *input_dev_ps;
	struct proximity_sensor_platform_data  *pdata;
	struct tmd26723_settings tmd26723_settings;
	unsigned int enable;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;

	/* control flag from HAL */
	unsigned int enable_proximity_sensor;

	/* PS parameters */
	unsigned int ps_threshold;
	/* always lower than ps_threshold */
	unsigned int ps_hysteresis_threshold;
	/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_detection;
	/* to store PS data */
	int ps_data;
	int irq_enabled;
};

static int tmd26723_init_client(struct i2c_client *client);
/*
 * Global data
 */

static int tmd26723_read_byte(struct i2c_client *client, u8 reg)
{
	int retries = I2C_RETRY_COUNT;
	s32 ret;
	pm_runtime_get_sync(&client->dev);

	do {
		ret = i2c_smbus_read_byte_data(client, reg);
	} while ((ret < 0) && retries--);

	pm_runtime_put(&client->dev);

	return ret;
}

static int tmd26723_read_word(struct i2c_client *client, u8 reg)
{
	int retries = I2C_RETRY_COUNT;
	s32 ret;
	pm_runtime_get_sync(&client->dev);

	do {
		ret = i2c_smbus_read_word_data(client, reg);
	} while ((ret < 0) && retries--);

	pm_runtime_put(&client->dev);

	return ret;
}

static int tmd26723_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	int retries = I2C_RETRY_COUNT;
	s32 ret;
	pm_runtime_get_sync(&client->dev);

	do {
		ret = i2c_smbus_write_byte_data(client, reg, data);
	} while ((ret < 0) && retries--);

	pm_runtime_put(&client->dev);
	return ret;
}

static int tmd26723_write_word(struct i2c_client *client, u8 reg, u16 data)
{
	int retries = I2C_RETRY_COUNT;
	s32 ret;
	pm_runtime_get_sync(&client->dev);

	do {
		ret = i2c_smbus_write_word_data(client, reg, data);
	} while ((ret < 0) && retries--);

	pm_runtime_put(&client->dev);
	return ret;
}

static int tmd26723_set_command(struct i2c_client *client, int command)
{
	struct tmd26723_data *data = i2c_get_clientdata(client);
	int ret = 0;
	int clearInt;

	if (TMD_CMD_CLEAR == command) {
		clearInt = CMD_CLR_PS_INT;
		mutex_lock(&data->update_lock);
		ret = i2c_smbus_write_byte(client, clearInt);
		mutex_unlock(&data->update_lock);
	}

	return ret;
}

static int tmd26723_set_register(struct i2c_client *client, int reg, int value)
{
	struct tmd26723_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);

	if ((TMD26723_PILTL_REG == reg) || (TMD26723_PIHTL_REG == reg))
		ret = tmd26723_write_word(client, CMD_WORD | reg, value);
	else
		ret = tmd26723_write_byte(client, CMD_BYTE | reg, value);

	switch (reg)
	{
		case TMD26723_ENABLE_REG:
			data->enable = value;
			break;
		case TMD26723_PTIME_REG:
			data->ptime  = value;
			break;
		case TMD26723_WTIME_REG:
			data->wtime  = value;
			break;
		case TMD26723_PILTL_REG:
			data->pilt   = value;
			break;
		case TMD26723_PIHTL_REG:
			data->piht   = value;
			break;
		case TMD26723_PERS_REG:
			data->pers   = value;
			break;
		case TMD26723_CONFIG_REG:
			data->config = value;
			break;
		case TMD26723_PPCOUNT_REG:
			data->ppcount = value;
			break;
		case TMD26723_CONTROL_REG:
			data->control = value;
			break;
		default:
			break;
	}

	mutex_unlock(&data->update_lock);

	return ret;
}

static int tmd26723_disable_irq(struct i2c_client *client)
{
	struct tmd26723_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->irq_lock);
	if ((data->pdata->gpio_int >= 0) && (data->irq_enabled)) {
		data->irq_enabled = 0;
		disable_irq(client->irq);
	}
        mutex_unlock(&data->irq_lock);
}

static int tmd26723_enable_irq(struct i2c_client *client)
{
        struct tmd26723_data *data = i2c_get_clientdata(client);

        mutex_lock(&data->irq_lock);
        if ((data->pdata->gpio_int >= 0) && (!data->irq_enabled)) {
                data->irq_enabled = 1;
                enable_irq(client->irq);
        }
        mutex_unlock(&data->irq_lock);
}

static int tmd26723_change_ps_threshold(struct i2c_client *client)
{
	struct tmd26723_data *data = i2c_get_clientdata(client);
	int ret;

	data->ps_data =	tmd26723_read_word(client,
				CMD_WORD|TMD26723_PDATAL_REG);
	if ((data->ps_data < SET_LOW_THRESHOLD) || (data->ps_data > SET_MAX_THRESHOLD)) {
		dev_err(&client->dev, "%s: tmd26723 read adc failed, ps_data : %d\n", __func__, data->ps_data);
		return data->ps_data;
	}

	pr_info("%s:data->pilt = 0x%x.data->piht = 0x%x, data->psdata = 0x%x\n", __func__, data->pilt, data->piht, data->ps_data);

	if (data->ps_data >= data->ps_threshold) {
		/* far-to-near detected */
		data->ps_detection = 1;

		ret = tmd26723_write_word(client, CMD_WORD|TMD26723_PILTL_REG,
						data->ps_hysteresis_threshold);
		if (ret < 0) {
			dev_err(&client->dev,"%s: write failed to TMD26723_PILTL_REG\n", __func__);
			return ret;
		}
		ret = tmd26723_write_word(client, CMD_WORD|TMD26723_PIHTL_REG,
						 SET_MAX_THRESHOLD);
		if (ret < 0) {
			dev_err(&client->dev,"%s: write failed to TMD26723_PIHTL_REG\n", __func__);
			return ret;
		}

		/* FAR-to-NEAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
		input_sync(data->input_dev_ps);

		data->pilt = data->ps_hysteresis_threshold;
		data->piht = SET_MAX_THRESHOLD;

		dev_dbg(&client->dev,
			"%s far-to-near detected\n", TMD_26723_DEV_NAME);
	} else if (data->ps_data < data->ps_hysteresis_threshold) {
		/* near-to-far detected */
		data->ps_detection = 0;

		ret = tmd26723_write_word(client, CMD_WORD|TMD26723_PILTL_REG, 0);
		if (ret < 0) {
			dev_err(&client->dev,"%s: write failed to TMD26723_PILTL_REG\n", __func__);
			return ret;
		}
		ret = tmd26723_write_word(client, CMD_WORD|TMD26723_PIHTL_REG,
					data->ps_threshold);
		if (ret < 0) {
			dev_err(&client->dev,"%s: write failed to TMD26723_PIHTL_REG\n", __func__);
			return ret;
		}

		/* NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 5);
		input_sync(data->input_dev_ps);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		dev_dbg(&client->dev,
			"%s near-to-far detected\n", TMD_26723_DEV_NAME);
	}
	return 0;
}

/*
 * Management functions
 */
static void tmd26723_work_handler(struct work_struct *work)
{
	int ret;
	struct tmd26723_data *data = container_of(work,
					struct tmd26723_data, dwork.work);
	struct i2c_client *client = data->client;
	int status;

	status = tmd26723_read_byte(client, CMD_BYTE|TMD26723_STATUS_REG);
	if (status < 0) {
		dev_err(&client->dev,"%s:read failed from TMD26723_STATUS_REG\n", __func__);
	}
	/* disable 990x's ADC first */
	ret = tmd26723_write_byte(client, CMD_BYTE|TMD26723_ENABLE_REG, DISABLE_REG_VALUE);
	if (ret < 0) {
		dev_err(&client->dev,"%s:write failed to TMD26723_ENABLE_REG\n", __func__);
	}
	if (PROXIMITY_INTERRUPT == (status & data->enable & PROXIMITY_INTERRUPT)) {
		/* The device is asserting a proximity interruption */
		mutex_lock(&data->change_ps_lock);
		ret = tmd26723_change_ps_threshold(client);
		mutex_unlock(&data->change_ps_lock);
		if (ret < 0) {
			dev_err(&client->dev,"%s: tmd26723_change_ps_threshold\n", __func__);
		}

		/* 0 = CMD_CLR_PS_INT */
		tmd26723_set_command(client, TMD_CMD_CLEAR);
	}

	ret = tmd26723_write_byte(client, CMD_BYTE|TMD26723_ENABLE_REG,
					data->enable);
	if (ret < 0) {
		dev_err(&client->dev,"%s:write failed to TMD26723_ENABLE_REG\n", __func__);
	}
}

static void tmd26723_reschedule_work(struct tmd26723_data *data,
					  unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, delay);

	spin_unlock_irqrestore(&data->lock, flags);
}

 /* assume this is ISR */
static irqreturn_t tmd26723_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct tmd26723_data *data = i2c_get_clientdata(client);

	disable_irq_nosync(vec);
	tmd26723_reschedule_work(data, 0);
	enable_irq(vec);

	return IRQ_HANDLED;
}

static int tmd26723_power_off(struct tmd26723_data *data)
{
	int err = -1;

	mutex_lock(&data->update_lock);
	if (data->pdata->power_off) {
		err = data->pdata->power_off(&data->input_dev_ps->dev);
		if (err < 0)
			dev_err(&data->client->dev,
				"power_off failed: %d\n", err);
	}
	mutex_unlock(&data->update_lock);
	return err;
}
static int tmd26723_power_on(struct tmd26723_data *data)
{
	int err = -1;

	mutex_lock(&data->update_lock);
	if (data->pdata->power_on) {
		err = data->pdata->power_on(&data->input_dev_ps->dev);
		if (err < 0) {
			dev_err(&data->client->dev,
					"power_on failed: %d\n", err);
			mutex_unlock(&data->update_lock);
			return err;
		}
	}
	msleep(TMD_STARTUP_DELAY);
	mutex_unlock(&data->update_lock);

	return 0;
}

/*
 * SysFS support
 */

static ssize_t tmd26723_show_ps_detection(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmd26723_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ps_detection);
}

static DEVICE_ATTR(ps_detection, S_IRUGO,
			tmd26723_show_ps_detection,
			NULL);

static ssize_t tmd26723_show_enable_proximity_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmd26723_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_proximity_sensor);
}

static ssize_t tmd26723_store_enable_proximity_sensor(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmd26723_data *data = i2c_get_clientdata(client);
	int err;
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	dev_dbg(&client->dev,
		"%s enable ps sensor\n", TMD_26723_DEV_NAME);

	mutex_lock(&data->sysfs_lock);
	if (val == 1) {
		dev_info(&client->dev, "%s: Proximity sensor power-on\n", __func__);
		/*turn on p sensor */
		data->enable_proximity_sensor = 1;
		err = tmd26723_power_on(data);
		if (err < 0)
			goto exit_error;
		/* Initialize the TMD26723 chip */
		err = tmd26723_init_client(client);
		if (err < 0) {
			dev_err(&client->dev, "Device initialization Failed: %s\n",
			data->input_dev_ps->name);
			return err;
		}
		tmd26723_enable_irq(client);
	} else {
		/* turn off p sensor - 21 Apr 2016 .
		 * we can't turn off the entire sensor,
		 * the proximity sensor may be needed by HAL */
		dev_info(&client->dev, "%s: Proximity sensor power-off\n", __func__);
		data->enable_proximity_sensor = 0;

		err = tmd26723_set_register(client, TMD26723_ENABLE_REG,  DISABLE_REG_VALUE);
		if (err < 0) {
			dev_err(&client->dev, "%s: TMD26723 set TMD26723_DISABLE_REG is faild\n", __func__);
			goto exit_error;
		}
		tmd26723_disable_irq(client);
		err = tmd26723_power_off(data);
		if (err < 0) {
			dev_err(&client->dev, "%s: Proximity sensor power-off failed\n",  __func__);
			goto exit_error;
		}
	}

	mutex_unlock(&data->sysfs_lock);
	return count;

exit_error:
	mutex_unlock(&data->sysfs_lock);
	return err;
}

static DEVICE_ATTR(enable_proximity_sensor, 0666,
				tmd26723_show_enable_proximity_sensor,
				tmd26723_store_enable_proximity_sensor);

static int tmd26723_detect(struct i2c_client *client)
{
	int  id;

	id = tmd26723_read_byte(client, CMD_BYTE|TMD26723_ID_REG);
	if (id < 0) {
		dev_err(&client->dev, "ID read failed\n");
		return id;
	}
	dev_dbg(&client->dev,
		"%s chip id %x\n", TMD_26723_DEV_NAME, id);

	return id;
}

static ssize_t tmd26723_chip_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int id;

	id = tmd26723_detect(client);
	return sprintf(buf, "%s %x\n", "TMD26723 proximity sensor",
						id);
}

static DEVICE_ATTR(chip_id, S_IRUGO, tmd26723_chip_id_show, NULL);

static ssize_t tmd26723_reg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	ret = tmd26723_detect(client);
	if (ret < 0)
		return 0;
	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_ENABLE_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_ENABLE_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_PTIME_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_PTIME_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_WTIME_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_WTIME_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_PILTL_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_PILTL_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_PILTH_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_PILTH_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_PIHTL_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_PIHTL_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_PIHTH_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_PIHTH_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_PERS_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_PERS_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_CONFIG_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_CONFIG_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_PPCOUNT_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_PPCOUNT_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_CONTROL_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_CONTROL_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_ID_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_ID_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_STATUS_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_STATUS_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_PDATAL_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_PDATAL_REG read failed.%x\n", ret);

	ret = tmd26723_read_byte(client, CMD_BYTE|TMD26723_PDATAH_REG);
	if (ret < 0)
		dev_err(&client->dev, "TMD26723_PDATAH_REG read failed.%x\n", ret);

	return 0;
}

static ssize_t tmd26723_reg_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmd26723_data *data = i2c_get_clientdata(client);
	int ret;
	unsigned int val;
	int regid;
	u8 i2c_buf[2];

	if (sscanf(buf, "%x", &val) == 0) {
		dev_err(&client->dev,
			"Error in reading register setting!\n");
		return -EINVAL;
	}
	/*
		Format of input value:
			bit[31..16] = don't care
			bit[15..8]  = register address
			bit[8..0]   = value to write into register
	*/
	regid = (val >> 8) & 0xff;
	val &= 0xff;

	if (regid < TMD26723_ENABLE_REG || regid > TMD26723_POFFSET) {
		dev_err(&client->dev,
			"Register address out of range!\n");
		return -EINVAL;
	}

	i2c_buf[0] = (u8) regid;
	i2c_buf[1] = (u8) val;
	mutex_lock(&data->update_lock);
	ret = tmd26723_write_byte(client, CMD_BYTE|i2c_buf[0],
						i2c_buf[1]);
	if (ret < 0)
		dev_err(&client->dev, "Write failed\n");
	mutex_unlock(&data->update_lock);

	return size;
}

static DEVICE_ATTR(reg, S_IRUGO, tmd26723_reg_show, tmd26723_reg_set);

static ssize_t tmd26723_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd26723_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%d\n", data->tmd26723_settings.proximity_odr);
}

static ssize_t tmd26723_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmd26723_data *data = i2c_get_clientdata(client);
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;
	if (!value)
		return -EINVAL;

	mutex_lock(&data->update_lock);
	data->tmd26723_settings.proximity_odr = value;
	mutex_unlock(&data->update_lock);

	return len;
}
static DEVICE_ATTR(proximity_delay, S_IRUGO | S_IWUSR,
		tmd26723_delay_show, tmd26723_delay_store);

static struct attribute *tmd26723_attributes[] = {
	&dev_attr_enable_proximity_sensor.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_reg.attr,
	&dev_attr_proximity_delay.attr,
	&dev_attr_ps_detection.attr,
	NULL
};


static const struct attribute_group tmd26723_attr_group = {
	.attrs = tmd26723_attributes,
};

/*
 * Initialization function
 */

static int tmd26723_init_client(struct i2c_client *client)
{
	int ret;

	ret = tmd26723_set_register(client, TMD26723_ENABLE_REG,  DISABLE_REG_VALUE);
	if (ret < 0)
		return ret;

	/* 2.73ms Prox integration time */
	ret = tmd26723_set_register(client, TMD26723_PTIME_REG,   PTIME_REG_VALUE);
	if(ret < 0){
		dev_err(&client->dev,"%s TMD26723 set TMD26723_PTIME_REG is faild\n", __func__);
		return ret;
	}
	/* 2.72ms Wait time */
	ret = tmd26723_set_register(client, TMD26723_WTIME_REG,   WTIME_REG_VALUE);
	if(ret < 0){
		dev_err(&client->dev,"%s TMD26723 set TMD26723_WTIME_REG is faild\n", __func__);
		return ret;
	}
	/* 1-Pulse for proximity */
	ret = tmd26723_set_register(client, TMD26723_PPCOUNT_REG, PPCOUNT_REG_VALUE);
	if(ret < 0){
		dev_err(&client->dev,"%s TMD26723 set TMD26723_PPCOUNT_REG is faild\n", __func__);
		return ret;
	}
	/* no long wait */
	ret = tmd26723_set_register(client, TMD26723_CONFIG_REG,  CONFIG_REG_VALUE);
	if(ret < 0){
		dev_err(&client->dev,"%s TMD26723 set TMD26723_CONFIG_REG is faild\n", __func__);
		return ret;
	}
	/* 100mA, CH0+CH1 IR-diode, 1X AGAIN */
	ret = tmd26723_set_register(client, TMD26723_CONTROL_REG, CONTROL_REG_VALUE);
	if(ret < 0){
		dev_err(&client->dev,"%s TMD26723 set TMD26723_CONTROL_REG is faild\n", __func__);
		return ret;
	}
	/* 2 consecutive Interrupt persistence */
	ret = tmd26723_set_register(client, TMD26723_PERS_REG,    PERS_REG_VALUE);
	if(ret < 0){
		dev_err(&client->dev,"%s TMD26723 set TMD26723_PERS_REG is faild\n", __func__);
		return ret;
	}
	ret = tmd26723_set_register(client, TMD26723_ENABLE_REG,  ENABLE_REG_VALUE);
	if(ret < 0){
		dev_err(&client->dev,"%s TMD26723 set TMD26723_ENABLE_REG is faild\n", __func__);
		return ret;
	}
	/* sensor is in disabled mode but all the configurations are preset */

	return 0;
}


/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver tmd26723_driver;
static int tmd26723_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct tmd26723_data *data;
	int err = 0, chip_id;

	pr_info("%s: tmd26723 probe be called\n", TMD_26723_DEV_NAME);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct tmd26723_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	data->pdata = client->dev.platform_data;
	if (!data->pdata) {
		dev_err(&client->dev,
		"%s No Platform data\n", TMD_26723_DEV_NAME);
		err = -EINVAL;
		goto exit_kfree;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

	data->enable = 0;
        data->ps_threshold = TMD26723_PS_DETECTION_THRESHOLD;
        data->ps_hysteresis_threshold = TMD26723_PS_HYSTERESIS_THRESHOLD;
	data->ps_detection = 0;	/* default to no detection */
	data->enable_proximity_sensor = 0;	/* default to 0 */
	data->irq_enabled = 0;

	dev_dbg(&client->dev, "enable = %s\n",
			data->enable ? "1" : "0");

	mutex_init(&data->update_lock);
	spin_lock_init(&data->lock);
	mutex_init(&data->sysfs_lock);
	mutex_init(&data->change_ps_lock);
	mutex_init(&data->irq_lock);

	err = gpio_request(data->pdata->gpio_int, "PROX_INTR");
	if (err) {
		dev_err(&client->dev,
		"%s gpio request failed\n", TMD_26723_DEV_NAME);
		goto exit_kfree;
	}
	if(data->pdata->gpio_int){

		client->irq = gpio_to_irq(data->pdata->gpio_int);
		if (client->irq < 0) {
			dev_err(&client->dev,
			"%s gpio to irq failed\n", TMD_26723_DEV_NAME);
			err = client->irq;
			goto exit_free_gpio;
		}
		dev_dbg(&client->dev, "%s: %s has set irq to irq:"
					" %d mapped on gpio:%d\n",
					TMD_26723_DEV_NAME, __func__, client->irq,
					data->pdata->gpio_int);

		err = request_irq(client->irq, tmd26723_interrupt, IRQF_TRIGGER_FALLING,
				TMD26723_DRV_NAME, (void *)client);
		if (err) {
			dev_dbg(&client->dev,
				"tmd26723.c: Could not allocate TMD26723_INT !\n");
			goto exit_free_gpio;
		}

		disable_irq_nosync(client->irq);

		INIT_DELAYED_WORK(&data->dwork, tmd26723_work_handler);
	}
	/* Register to Input Device */
	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device ps\n");
		goto exit_free_irq;
	}
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
	data->input_dev_ps->name = "proximity_sensor";

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"Unable to register input device ps: %s\n",
				data->input_dev_ps->name);
		goto exit_free_dev_ps;
	}

	if (data->pdata->init) {
		err = data->pdata->init(&data->input_dev_ps->dev);
		if (err < 0) {
			dev_err(&client->dev, "Plat Init Failed: %s\n",
				data->input_dev_ps->name);
			goto exit_unregister_dev_ps;
		}
	}

	err = tmd26723_power_on(data);
	if (err < 0) {
		dev_err(&client->dev, "Power ON Failed: %s\n",
		       data->input_dev_ps->name);
		goto exit_plat_exit;
	}

        chip_id = tmd26723_read_byte(client, CMD_BYTE|TMD26723_ID_REG);
        if (chip_id == 0x00) {
                dev_dbg(&client->dev,
                "%s TMD27711 or TMD27715\n", TMD_26723_DEV_NAME);
        } else if (chip_id == 0x09) {
                dev_dbg(&client->dev,
                "%s TMD27713 or TMD27717\n", TMD_26723_DEV_NAME);
        } else if (chip_id == 0x3b){
                dev_dbg(&client->dev,
                "%s TMD26723\n", TMD_26723_DEV_NAME);
        } else {
                dev_dbg(&client->dev,
                "%s No TMD26723 chip detected\n", TMD_26723_DEV_NAME);
                err = -ENODEV;
                goto exit_power_off;
        }

        /* init threshold for proximity */
        err = tmd26723_set_register(client, TMD26723_PILTL_REG, 0);
        if(err < 0){
                dev_err(&client->dev,"%s TMD26723 set TMD26723_PILTL_REG is faild\n", __func__);
                goto exit_power_off;
        }
        err = tmd26723_set_register(client, TMD26723_PIHTL_REG, SET_MAX_THRESHOLD);
        if(err < 0){
                dev_err(&client->dev,"%s TMD26723 set TMD26723_PIHTL_REG is faild\n", __func__);
                goto exit_power_off;
        }

	/* Initialize the TMD26723 chip */
	err = tmd26723_init_client(client);
	if (err < 0) {
		dev_err(&client->dev, "Device initialization Failed: %s\n",
		       data->input_dev_ps->name);
		goto exit_power_off;
	}

	data->enable_proximity_sensor = 1;
	/* Read ps_data when proximity_sensor_enabled */
	mutex_lock(&data->change_ps_lock);
	err = tmd26723_change_ps_threshold(client);
	mutex_unlock(&data->change_ps_lock);
	if (err < 0)
		dev_err(&data->client->dev, "%s: tmd26723_change_ps_threshold\n", __func__);

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &tmd26723_attr_group);
	if (err)
		goto exit_power_off;
	pm_runtime_enable(&client->dev);

	return 0;

exit_power_off:
	tmd26723_power_off(data);
exit_plat_exit:
	if (data->pdata->exit)
		data->pdata->exit();
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
	goto exit_free_irq;
exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_free_irq:
	free_irq(client->irq, client);
exit_free_gpio:
	gpio_free(data->pdata->gpio_int);
exit_kfree:
	kfree(data);
exit:
	if (err >= 0)
		err = -EINVAL;
	return err;
}

static int tmd26723_remove(struct i2c_client *client)
{
	struct tmd26723_data *data = i2c_get_clientdata(client);
	pm_runtime_disable(&client->dev);

	/* Power down the device */
	tmd26723_set_register(client, TMD26723_ENABLE_REG, DISABLE_REG_VALUE);
	tmd26723_power_off(data);
	input_unregister_device(data->input_dev_ps);
	if(data->pdata->gpio_int){
		free_irq(client->irq, client);
		gpio_free(data->pdata->gpio_int);
	}
	sysfs_remove_group(&client->dev.kobj, &tmd26723_attr_group);

	if (data->pdata->exit)
		data->pdata->exit();
	kfree(data);


	return 0;
}

#if CONFIG_PM

static int tmd26723_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmd26723_data *data = i2c_get_clientdata(client);

	tmd26723_disable_irq(client);
	if (data->enable_proximity_sensor == 1) {
		tmd26723_set_register(client, TMD26723_ENABLE_REG, DISABLE_REG_VALUE);
		tmd26723_power_off(data);
	}

	return 0;
}

static int tmd26723_resume(struct device *dev)
{
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct tmd26723_data *data = i2c_get_clientdata(client);

	if (data->enable_proximity_sensor == 1) {
		err = tmd26723_power_on(data);
		if (err < 0) {
			dev_err(&client->dev, "Power ON Failed: %s\n",
				data->input_dev_ps->name);
		}
		/* Initialize the TMD26723 chip */
		err = tmd26723_init_client(client);
		if (err < 0) {
			dev_err(&client->dev, "Device initialization Failed: %s\n",
				data->input_dev_ps->name);
			tmd26723_power_off(data);
			goto out;
		}
		/* Read ps_data after resume */
		mutex_lock(&data->change_ps_lock);
		err = tmd26723_change_ps_threshold(client);
		mutex_unlock(&data->change_ps_lock);
		if (err < 0) {
			dev_err(&client->dev, "%s: tmd26723_change_ps_threshold\n", __func__);
			goto out;
		}
	}
	tmd26723_enable_irq(client);

out:
	return err;
}
static SIMPLE_DEV_PM_OPS(tsl_pm_ops, tmd26723_suspend, tmd26723_resume);
#define TMD_PM_OPS (&tsl_pm_ops)

#else

#define TMD_PM_OPS NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id tmd26723_id[] = {
	{ "tmd26723", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmd26723_id);

static struct i2c_driver tmd26723_driver = {
	.driver = {
		.name	= TMD26723_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm = TMD_PM_OPS,
	},
	.probe	= tmd26723_probe,
	.remove	= tmd26723_remove,
	.id_table = tmd26723_id,
};

static int __init tmd26723_init(void)
{
	pr_info("%s: tmd26723 sysfs driver init\n", TMD_26723_DEV_NAME);
	return i2c_add_driver(&tmd26723_driver);
}

static void __exit tmd26723_exit(void)
{
	pr_info("%s exit\n", TMD_26723_DEV_NAME);
	i2c_del_driver(&tmd26723_driver);
	return;
}

module_init(tmd26723_init);
module_exit(tmd26723_exit);

MODULE_AUTHOR("Qiang Kai <kaix.qiang@intel.com>");
MODULE_DESCRIPTION("TMD26723 proximity sensor driver");
MODULE_LICENSE("GPL");
