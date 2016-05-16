/*
 * STMicroelectronics lps22hb platform-data driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef ST_LPS22HB_PDATA_H
#define ST_LPS22HB_PDATA_H

/**
 * struct st_lps22hb_platform_data - Platform data for the ST lps22hb sensor
 * @drdy_int_pin: Redirect DRDY on pin 1 (1) or pin 2 (2).
 */
struct st_lps22hb_platform_data {
   int gpio_int;
};

#endif /* ST_LPS22HB_PDATA_H */
