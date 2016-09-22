/*
 * platform_spidev.c: spidev platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Dan O'Donovan <dan@emutex.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/spi/spi.h>
#include <linux/spi/intel_mid_ssp_spi.h>
#include <asm/intel-mid.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include "platform_spidev.h"

static void tng_ssp_spi_cs_control(u32 command);

static int tng_ssp_spi2_FS_gpio = 111;

static struct intel_mid_ssp_spi_chip chip = {
	.burst_size = DFLT_FIFO_BURST_SIZE,
	.timeout = DFLT_TIMEOUT_VAL,
	/* SPI DMA is currently usable on Tangier */
	.dma_enabled = false,
	.cs_control = tng_ssp_spi_cs_control,
};

static void tng_ssp_spi_cs_control(u32 command)
{
	gpio_set_value(tng_ssp_spi2_FS_gpio, (command != 0) ? 1 : 0);
}

void __init *spidev_platform_data(void *info)
{
	struct spi_board_info *spi_info = info;

	if (!spi_info) {
		pr_err("%s: invalid info pointer\n", __func__);
		return NULL;
	}

	spi_info->mode = SPI_MODE_0;

	spi_info->controller_data = &chip;
	spi_info->bus_num = FORCE_SPI_BUS_NUM;

	return NULL;
}

static const struct devs_id spidev_dev_id __initconst = {
	.name = "spidev",
	.type = SFI_DEV_TYPE_SPI,
	.delay = 0,
	.get_platform_data = &spidev_platform_data,
};

sfi_device(spidev_dev_id);
