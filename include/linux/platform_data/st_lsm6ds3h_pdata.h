/*
 * STMicroelectronics lsm6ds3h platform-data driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef ST_LSM6DS3H_PDATA_H
#define ST_LSM6DS3H_PDATA_H

/**
 * struct st_lsm6ds3h_platform_data - Platform data for the ST lsm6ds3h sensor
 * @drdy_int_pin: Redirect DRDY on pin 1 (1) or pin 2 (2).
 */
struct st_lsm6ds3h_platform_data {
	int gpio_int1;
	int (*gpio_conf)(void);
};

#endif /* ST_LSM6DS3H_PDATA_H */
