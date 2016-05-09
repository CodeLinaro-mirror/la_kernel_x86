/*
 * Copyright (c)  2016 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors: Kevin Liu <kevin.liu@intel.com>
 *
 */


#ifndef HIMAX_CMD_H
#define HIMAX_CMD_H

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <asm/intel_scu_ipc.h>
#include "mdfld_output.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"

void himax_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs);

/* himax display unlock */
static u8 himax_unlock[] = {
	0xff, 0x70, 0x69
};

/* himax display gamma settings */
static u8 himax_RGamma[] = { 0xb5,
	0x01, 0xff, 0x01, 0x8e, 0x01, 0x72,
	0x01, 0x5a, 0x01, 0x36, 0x00, 0xff,
	0x00, 0x00, 0x00, 0x70, 0x00, 0x8c,
	0x00, 0xa4, 0x00, 0xc8, 0x00, 0xff
};
static u8 himax_GGamma[] = { 0xb6,
	0x01, 0xff, 0x01, 0x8e, 0x01, 0x72,
	0x01, 0x5a, 0x01, 0x36, 0x00, 0xff,
	0x00, 0x00, 0x00, 0x70, 0x00, 0x8c,
	0x00, 0xa4, 0x00, 0xc8, 0x00, 0xff
};
static u8 himax_BGamma[] = { 0xb7,
	0x01, 0xff, 0x01, 0x8e, 0x01, 0x72,
	0x01, 0x5a, 0x01, 0x36, 0x00, 0xff,
	0x00, 0x00, 0x00, 0x70, 0x00, 0x8c,
	0x00, 0xa4, 0x00, 0xc8, 0x00, 0xff
};

/* himax display CABC control */
static u8 himax_cabc[] = { 0xdd,
	0x1f, 0x00, 0x0f, 0x06, 0x88, 0x08
};

/* himax display VCOM&VCSTN settings */
static u8 himax_vcom_vcstn[] = {
	0xb2, 0x01, 0x16, 0x00, 0xff, 0x31
};

/* himax display vring settings */
static u8 himax_vring[] = {
	0xb4, 0xf9, 0x05
};

/* himax display mipi lane configuration */
static u8 himax_lane_conf = 0xc6;

/* himax display filp options */
static u8 himax_flip_conf = 0x36;

#endif
