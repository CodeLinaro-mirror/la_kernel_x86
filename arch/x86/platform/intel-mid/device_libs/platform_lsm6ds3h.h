/*
 * platform_lsm6ds3h.h: lsm6ds3h platform data header file
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_LSM6DS3H_H_
#define _PLATFORM_LSM6DS3H_H_

extern void __init *lsm6ds3h_platform_data(void *info) __attribute__((weak));
#endif
