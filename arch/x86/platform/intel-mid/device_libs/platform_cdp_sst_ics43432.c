/*
 * platform_cdp_sst_ics43432: Cloverdale Peak (CDP) audio
 * platform data initialization file
 *
 * (C) Copyright 2016 Intel Corporation
 * Author: Mattijs Korpershoek <mattijsx.korpershoek@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/sfi.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>

#include "platform_msic.h"
#include "platform_ipc.h"

void __weak *cdp_audio_platform_data(void *info)
{
	struct platform_device *pdev;

	/* platform / cpu dai */
	pdev = platform_device_register_simple("sst-mfld-platform", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("failed to register sst-mfld-platform device\n");
		return NULL;
	}

	/* codec driver */
	pdev = platform_device_register_simple("ics43432", -1, NULL, 0);
	if (IS_ERR(pdev)) {
	  pr_err("failed to register ics43432 codec\n");
	  return NULL;
	}

	/* machine driver */
	pdev = platform_device_register_simple("cdp_ics43432", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("failed to register cdp_ics43432 machine driver\n");
		return NULL;
	}

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_AUDIO);
}

static const struct devs_id cdp_audio_dev_id __initconst = {
	.name = "sst_ics43432",
	.type = SFI_DEV_TYPE_IPC,
	.delay = 1,
	.get_platform_data = &cdp_audio_platform_data,
	.device_handler = &ipc_device_handler,
};

sfi_device(cdp_audio_dev_id);
