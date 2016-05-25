/*
 * platform_lp5562.c:  lp5562 platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/platform_data/leds-lp55xx.h>
#include <asm/intel-mid.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/i2c.h>
#include "platform_lp5562.h"

#define gpio_num get_gpio_by_name("LED_ENABLE")

static struct lp55xx_platform_data lp5562_platform_data;
static struct lp55xx_led_config lp5562_led_config[] = {
	{
		.name           = "R",
		.chan_nr        = 0,
		.led_current    = 20,
		.max_current    = 40,
	},
	{
		.name           = "G",
		.chan_nr        = 1,
		.led_current    = 20,
		.max_current    = 40,
	},
	{
		.name           = "B",
		.chan_nr        = 2,
		.led_current    = 20,
		.max_current    = 40,
	},
	{
		.name           = "W",
		.chan_nr        = 3,
		.led_current    = 20,
		.max_current    = 40,
	},
};

/* mode_1:  red color 4 flickers 250ms on 100ms off */
static const u8 mode_1[] = {
		0x40, 0xff, 0x50, 0x00, 0x40, 0x00, 0x46, 0x00, 0xA1, 0x80, 0xD0, 0x00};

/* mode_2: amber color always on */
static const u8 mode_2[] = {0x40, 0xFF, 0x4D, 0x00, 0xA2, 0x80, 0xD0, 0x00};

/* mode_3: steady green for battery charging full*/
static const u8 mode_3[] = {0x40, 0xFF, 0x4D, 0x00, 0xA2, 0x80, 0xD0, 0x00};

/*flash red*/
static const u8 mode_4[] = {0x40, 0xFF, 0x4D, 0x00, 0xA2, 0x80, 0xD0, 0x00};

/*White 2 blinks 250 on and 250 off*/
static const u8 mode_5[] = {0x40, 0xFF, 0x50, 0x00, 0x40, 0x00, 0x50, 0x00,
					0xA0, 0x80, 0xD0, 0x00};

/*White 2 blinks 500 on and 250 off*/
static const u8 mode_6[] = {0x40, 0xFF, 0x60, 0x00, 0x40, 0x00, 0x50, 0x00,
					0xA0, 0x80, 0xD0, 0x00};

/*White 2 blinks 500 on and 250 off*/
static const u8 mode_7[] = {0x40, 0xFF, 0x50, 0x00, 0x40, 0x00, 0x50, 0x00,
					0xD0, 0x00, 0x40, 0x00, 0x50, 0x00,
					0x40, 0x00, 0x50, 0x00};

/*White 2 blinks 500 on and 250 off */
static const u8 mode_8[] = {0x40, 0xFF, 0x50, 0x00, 0x40, 0x00, 0x50, 0x00,
					0xA0, 0x00, 0xD0, 0x00};

/*blue blinks */
static const u8 mode_9[] = {0x40, 0xFF, 0x4D, 0x00, 0x40, 0x00, 0x60, 0x00,
					0xA2, 0x00, 0xD0, 0x00, 0x00, 0x00,
					0x00, 0x00
			   };
/*green blinks */
static const u8 mode_10[] = {0x40, 0xFF, 0x4D, 0x00, 0x40, 0x00, 0x60, 0x00,
					0xA2, 0x10, 0xD0, 0x00, 0x00, 0x00,
					0x00, 0x00,
			    };
/*red Breathing*/
static const u8 mode_11[] = {0x0A, 0x00, 0x33, 0x13, 0x14, 0x31, 0x08, 0x31,
					0x04, 0x31, 0x0C, 0x31, 0x0C, 0xB1,
					0x08, 0xB1, 0x04, 0xB1, 0x08, 0xB1,
					0x22, 0x9D, 0x22, 0x9D,	0x73, 0x00,
					0xA0, 0xA0, 0xD0, 0x00
			    };
/*blue Breathing*/
static const u8 mode_12[] = {	0x0A, 0x00, 0x33, 0x13, 0x14, 0x31, 0x08, 0x31,
				0x04, 0x31, 0x0C, 0x31, 0x08, 0xB1, 0x08, 0xB1,
				0x04, 0xB1, 0x08, 0xB1, 0x22, 0x9D, 0x22, 0x9D,
				0x73, 0x00, 0xA0, 0x80, 0XD0, 0X00
			    };
/*green Breathing*/
static const u8 mode_13[] = {   0x0A, 0x00, 0x33, 0x13, 0x14, 0x31, 0x08, 0x31,
				0x04, 0x31, 0x0C, 0x31, 0x0C, 0xB1, 0x08, 0xB1,
				0x04, 0xB1, 0x08, 0xB1, 0x22, 0x9D, 0x22, 0x9D,
				0x73, 0x00, 0xA0, 0x90, 0xD0, 0x00
				};

struct lp55xx_predef_pattern board_led_patterns[] = {
	{
		.r = mode_1,
		.size_r = ARRAY_SIZE(mode_1),
	},
	{
		.r = mode_2,
		.g = mode_2,
		.size_r = ARRAY_SIZE(mode_2),
		.size_g = ARRAY_SIZE(mode_2),
	},
	{
		.g = mode_3,
		.size_g = ARRAY_SIZE(mode_3),
	},
	{
		.r = mode_4,
		.size_r = ARRAY_SIZE(mode_4),
	},
	{
		.r = mode_5,
		.g = mode_5,
		.b = mode_5,
		.size_r = ARRAY_SIZE(mode_5),
		.size_g = ARRAY_SIZE(mode_5),
		.size_b = ARRAY_SIZE(mode_5),
	},
	{
		.r = mode_6,
		.g = mode_6,
		.b = mode_6,
		.size_r = ARRAY_SIZE(mode_6),
		.size_g = ARRAY_SIZE(mode_6),
		.size_b = ARRAY_SIZE(mode_6),
	},
	{
		.r = mode_7,
		.g = mode_7,
		.b = mode_7,
		.size_r = ARRAY_SIZE(mode_7),
		.size_g = ARRAY_SIZE(mode_7),
		.size_b = ARRAY_SIZE(mode_7),
	},
	{
		.r = mode_8,
		.g = mode_8,
		.b = mode_8,
		.size_r = ARRAY_SIZE(mode_8),
		.size_g = ARRAY_SIZE(mode_8),
		.size_b = ARRAY_SIZE(mode_8),
	},
	{
		.b = mode_9,
		.size_b = ARRAY_SIZE(mode_9),
	},
	{
		.g = mode_10,
		.size_g = ARRAY_SIZE(mode_10),
	},
	{
		.r = mode_11, /* Red */
		.size_r = ARRAY_SIZE(mode_11),
	},
	{
		.b = mode_12,  /* Blue */
		.size_b = ARRAY_SIZE(mode_12),
	},
	{
		.g = mode_13,  /* Green */
		.size_g = ARRAY_SIZE(mode_13),
	},

};

static int lp5562_setup(void)
{
	/* setup HW resources */
	return gpio_request(gpio_num, "LED_ENABLE");
}

static void lp5562_release(void)
{
	/* Release HW resources */
	gpio_free(gpio_num);
}

static void lp5562_enable(bool state)
{
	/* Control of chip enable signal */
	if (state)
		gpio_set_value(gpio_num, 1);
	else
		gpio_set_value(gpio_num, 0);
}

static struct lp55xx_platform_data lp5562_platform_data = {
	.led_config		=	lp5562_led_config,
	.num_channels		=	ARRAY_SIZE(lp5562_led_config),
	.setup_resources	=	lp5562_setup,
	.release_resources	=	lp5562_release,
	.patterns		=	board_led_patterns,
	.num_patterns		=	ARRAY_SIZE(board_led_patterns),
	.enable			=	lp5562_enable,
	.current_mode		=	NULL,
};

void __init *lp55xx_platform_data(void *info)
{
	return &lp5562_platform_data;
}
