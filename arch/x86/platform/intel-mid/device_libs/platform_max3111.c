/*
 * platform_max3111.c: max3111 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/lnw_gpio.h>
#include <linux/serial_max3110.h>
#include <linux/spi/intel_mid_ssp_spi.h>
#include <asm/intel-mid.h>
#include "platform_max3111.h"

#ifdef CONFIG_SPI_INTEL_MID_SSP
static struct intel_mid_ssp_spi_chip mid_spi_chip = {
	.burst_size = DFLT_FIFO_BURST_SIZE,
	.timeout = DFLT_TIMEOUT_VAL,
	/* UART DMA is not supported in VP */
	.dma_enabled = false,
};
void *chip = &mid_spi_chip;
#else
void *chip;
#endif

void __init *max3111_platform_data(void *info)
{
	struct spi_board_info *spi_info = info;
	int intr;
	static struct plat_max3110 max3110_pdata;

	spi_info->mode = SPI_MODE_0;


		spi_info->controller_data = chip;
		spi_info->bus_num = FORCE_SPI_BUS_NUM;

		/* use fast_int_1 (IRQ 41) on MRFL */
		max3110_pdata.irq_edge_triggered = 0;

	return &max3110_pdata;
}

static const struct devs_id spi_max3111_dev_id __initconst = {
	        .name = "spi_max3111",
		.type = SFI_DEV_TYPE_SPI,
		.delay = 0,
		.get_platform_data = &max3111_platform_data,
};
sfi_device(spi_max3111_dev_id);
