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

#ifndef __TMD_26723_H__
#define __TMD_26723_H__

#define TMD_26723_DEV_NAME		"tmd26723"

#define TMD_26723_I2C_SAD		0x39

struct proximity_sensor_platform_data {
	int (*power_on)(struct device *dev);
	int (*power_off)(struct device *dev);
	int (*init)(struct device *dev);
	void (*exit)(void);
	/* gpio ports for interrupt pads */
	int gpio_int;
	/* threshold initial values */
	int ps_threshold;
	int ps_hysteresis_threshold;
};
#endif  /* __TMD_26723_H__ */
