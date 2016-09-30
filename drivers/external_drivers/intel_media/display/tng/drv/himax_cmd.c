/*
 * Copyright © 2016 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
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

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_esd.h"
#include "mdfld_dsi_dbi_dsr.h"
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <linux/lnw_gpio.h>
#include <linux/debugfs.h>
#include <linux/ctype.h>

#include "displays/himax_cmd.h"

#define HIMAX_DEFAULT_BRIGHTNESS 0x66

static int mipi_reset_gpio;
static int bias_en_gpio;
static int himax_conf_intf;
static int himax_boost_en;
static int himax_en_pwm;

typedef struct {
	struct dentry *dir;
	/* atomic ops */
	struct dentry *addr_set;
	struct dentry *send_lp;
	struct dentry *send_hs;
	struct dentry *read_lp;
	struct dentry *read_hs;
	unsigned int addr;
} dbgfs_t;


static dbgfs_t dbgfs;
static struct mdfld_dsi_config *dbgfs_dsi_config;

static
int himax_cmd_drv_ic_init(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
			mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	return 0;

ic_init_err:
	DRM_ERROR("Driver IC init failed\n");
	err = -EIO;
	return err;
}

static
void himax_cmd_controller_init(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
			&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/* mipi lane configuration */
	dsi_config->lane_count = 2;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

	/* DSI PLL 400 MHz, set it to 0 for 800 MHz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	/* settings for 2 mipi data lane */
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xFFFF;
	hw_ctx->turn_around_timeout = 0x1A;
	hw_ctx->high_low_switch_count = 0x10;
	hw_ctx->clk_lane_switch_time_cnt = 0x100008;
	hw_ctx->lp_byteclk = 0x2;
	hw_ctx->dphy_param = 0x120A2B0C;
	/* eot transmission supported */
	hw_ctx->eot_disable = 0x3;
	/* min time should be 100us(0x7d0) */
	hw_ctx->init_count = 0x7d0;
	/* test value */
	hw_ctx->dbi_bw_ctrl = 1390;
	hw_ctx->hs_ls_dbi_enable = 0x0;
	hw_ctx->dsi_func_prg = ((DBI_DATA_WIDTH_OPT2 << 13) |
				dsi_config->lane_count);
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE |
			BANDGAP_CHICKEN_BIT |
			TE_TRIGGER_GPIO_PIN |
			dsi_config->lane_config;

	return;
}

static
int himax_cmd_panel_connection_detect(
		struct mdfld_dsi_config *dsi_config)
{
	int status;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		status = MDFLD_DSI_PANEL_CONNECTED;
	} else {
		DRM_INFO("DO NOT support dual panel\n");
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static
int himax_cmd_power_on(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
			mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	u8 datatest = 0;
	int gpio_value = -EINVAL;

	PSB_DEBUG_ENTRY("\n");
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* PMIC should guarantee VDDD/VDDDSI/VDDA
	   RESET HIMAX display */

	//gpio_direction_output(bias_en_gpio, 1);
	gpio_direction_output(mipi_reset_gpio, 0);
	/* 0 for DSI, 1 for I2c */
	gpio_direction_output(himax_conf_intf, 0);
	gpio_direction_output(himax_boost_en, 1);
	gpio_direction_output(himax_en_pwm, 1);

	//gpio_set_value(bias_en_gpio, 1);
	gpio_set_value(himax_conf_intf, 0);

	gpio_set_value(himax_boost_en, 1);

	gpio_set_value(himax_en_pwm, 1);

	gpio_set_value(mipi_reset_gpio, 1);

	/* Set unlock */
	err = mdfld_dsi_send_mcs_long_lp(sender,
			himax_unlock, sizeof(himax_unlock),
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set unlock\n");
		goto power_on_err;
	}

	/* Gamma settings */
	err = mdfld_dsi_send_mcs_long_lp(sender,
			himax_RGamma, sizeof(himax_RGamma),
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set red gamma\n");
		goto power_on_err;
	}

	err = mdfld_dsi_send_mcs_long_lp(sender,
			himax_GGamma, sizeof(himax_GGamma),
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set green gamma\n");
		goto power_on_err;
	}

	err = mdfld_dsi_send_mcs_long_lp(sender,
			himax_BGamma, sizeof(himax_BGamma),
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set blue gamma\n");
		goto power_on_err;
	}

	/* himax display VCOM&VCSTN settings */
	err = mdfld_dsi_send_mcs_long_lp(sender,
			himax_vcom_vcstn, sizeof(himax_vcom_vcstn),
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set Vcom and VCSTN\n");
		goto power_on_err;
	}

	/* himax display vring settings */
	err = mdfld_dsi_send_mcs_long_lp(sender,
			himax_vring, sizeof(himax_vring),
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set Vring\n");
		goto power_on_err;
	}

	/* himax display sleep out */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			exit_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Sleep out\n");
		goto power_on_err;
	}
	msleep(120);

	/* himax single mipi data lane
	err = mdfld_dsi_send_mcs_short_lp(sender,
			himax_lane_conf, 0xc0, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set display single data lane\n");
		goto power_on_err;
	}
	*/
#ifdef HIMAX_DEBUG
	mdfld_dsi_read_mcs_lp(sender, himax_lane_conf, &datatest, 1);
	DRM_INFO("HIMAX: himax_lane_conf(0xc6) is %x\n", datatest);
#endif

	/* himax display flip horizontally */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			himax_flip_conf, 0x02, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set horizontal flip");
		goto power_on_err;
	}

	/* himax display on */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			set_display_on, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to bring Display on\n");
		goto power_on_err;
	}
	msleep(1);

	/* himax set brightness: default(0x66) */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_display_brightness, HIMAX_DEFAULT_BRIGHTNESS, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Backlight Brightness\n");
		goto power_on_err;
	}

	/* himax CABC enable */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_ctrl_cabc, 0x03, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to disable CABC\n");
		goto power_on_err;
	}

	/* himax TE enable with vblank info only */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			set_tear_on, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Tear On\n");
		goto power_on_err;
	}

	/* himax backlight on */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_ctrl_display, 0x2c, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Backlight On\n");
		goto power_on_err;
	}

	/* himax CABC control */
	err = mdfld_dsi_send_mcs_long_lp(sender,
			himax_cabc, sizeof(himax_cabc),
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set CABC control\n");
		goto power_on_err;
	}

#ifdef HIMAX_DEBUG
	mdfld_dsi_read_mcs_lp(sender, write_ctrl_display, &datatest, 1);
	DRM_INFO("HIMAX: write_ctrl_display(0x53) is %x\n", datatest);
#endif

	return 0;
power_on_err:
	err = -EIO;
	DRM_ERROR("Power on failed\n");
	return err;
}

static
int himax_cmd_power_off(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
			mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* himax backlight off */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_ctrl_display, 0x04, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Backlight Off\n");
		goto power_off_err;
	}

	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_ctrl_display, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Backlight Off\n");
		goto power_off_err;
	}

	/* set backlight brightness */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_display_brightness, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Backlight Brightness\n");
		goto power_off_err;
	}

	usleep_range(1000, 1100);

	/* himax TE disable */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			set_tear_off, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Tear Off\n");
		goto power_off_err;
	}

	usleep_range(1000, 1100);

	/* himax display off */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			set_display_off, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to set display off\n");
		goto power_off_err;
	}

	usleep_range(1000, 1100);
	/* himax sleep-in */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			enter_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Sleep in\n");
		goto power_off_err;
	}

	/* turn off BL driver */
	gpio_set_value(himax_boost_en, 0);
	gpio_set_value(himax_en_pwm, 0);

	/* assert panel reset 1 ms*/
	gpio_set_value(mipi_reset_gpio, 1);
	usleep_range(1000, 1100);

	//if (bias_en_gpio)
	//	gpio_set_value(bias_en_gpio, 0);

	return 0;

power_off_err:
	err = -EIO;
	DRM_ERROR("Power off failed\n");
	return err;
}

static
int himax_cmd_set_brightness(
		struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
			mdfld_dsi_get_pkg_sender(dsi_config);
	u8 duty_val = 0;
	int err = 0;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (0xFF * level) / 255;
	err = mdfld_dsi_send_mcs_short_hs(sender,
			write_display_brightness, duty_val, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Brightness\n");
		return -EIO;
	}

	return 0;
}

static
int himax_cmd_panel_reset(
		struct mdfld_dsi_config *dsi_config)
{
	PSB_DEBUG_ENTRY("\n");

	if (dbgfs_dsi_config == NULL)
		dbgfs_dsi_config = dsi_config;

	//gpio_direction_output(bias_en_gpio, 1);
	// Active high reset
	gpio_direction_output(mipi_reset_gpio, 0);

	//gpio_set_value(bias_en_gpio, 1);
	gpio_set_value(mipi_reset_gpio, 1);

	msleep(10);

	return 0;
}

static
struct drm_display_mode *himax_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 640;
	mode->hsync_start = 654;
	mode->hsync_end = 666;
	mode->htotal = 680;

	mode->vdisplay = 360;
	mode->vsync_start = 368;
	mode->vsync_end = 372;
	mode->vtotal = 380;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void himax_cmd_get_panel_info(
		int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = 40;
		pi->height_mm = 40;
	}
}

/* atomic operations from debugfs */
enum dbgfs_type { ADDR, HIGH_SPEED, LOW_POWER };

/* reading operations */
static ssize_t dbgfs_read(char __user *,
		size_t , loff_t *, enum dbgfs_type);
static ssize_t dbgfs_addr_read(struct file *,
		char __user *, size_t , loff_t *);
static ssize_t dbgfs_read_hs_read(struct file *,
		char __user *, size_t , loff_t *);
static ssize_t dbgfs_read_lp_read(struct file *,
		char __user *, size_t , loff_t *);

/* writting operations */
static int dbgfs_write(const char __user *buff,
		size_t , enum dbgfs_type);
static ssize_t dbgfs_addr_write(struct file *,
		const char __user *, size_t , loff_t *);
static ssize_t dbgfs_send_lp_write(struct file *,
		const char __user *, size_t , loff_t *);
static ssize_t dbgfs_send_hs_write(struct file *,
		const char __user *, size_t , loff_t *);

/* ops for atomic operations */
static const struct file_operations dbgfs_addr_ops = {
	.open		= nonseekable_open,
	.read		= dbgfs_addr_read,
	.write		= dbgfs_addr_write,
	.llseek		= no_llseek,
};
static const struct file_operations dbgfs_send_lp_ops = {
	.open		= nonseekable_open,
	.write		= dbgfs_send_lp_write,
	.llseek		= no_llseek,
};
static const struct file_operations dbgfs_send_hs_ops = {
	.open		= nonseekable_open,
	.write		= dbgfs_send_hs_write,
	.llseek		= no_llseek,
};
static const struct file_operations dbgfs_read_lp_ops = {
	.open		= nonseekable_open,
	.read		= dbgfs_read_lp_read,
	.llseek		= no_llseek,
};
static const struct file_operations dbgfs_read_hs_ops = {
	.open		= nonseekable_open,
	.read		= dbgfs_read_hs_read,
	.llseek		= no_llseek,
};

void himax_cmd_init(struct drm_device *dev,
				struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	bias_en_gpio = get_gpio_by_name("disp0_bias_en");
	if (bias_en_gpio <= 0)
		bias_en_gpio = 189;
	gpio_request(bias_en_gpio, "himax_display");

	mipi_reset_gpio = get_gpio_by_name("disp0_rst");
	if (mipi_reset_gpio <= 0)
		mipi_reset_gpio = 190;
	gpio_request(mipi_reset_gpio, "himax_display");

	himax_conf_intf = get_gpio_by_name("disp_ifsel");
	if (himax_conf_intf <= 0)
		himax_conf_intf = 188;
	gpio_request(himax_conf_intf, "himax_display");

	himax_boost_en = get_gpio_by_name("disp0_touch_rst");
	if (himax_boost_en <= 0)
		himax_boost_en = 191;
	gpio_request(himax_boost_en, "himax_display");

	himax_en_pwm = get_gpio_by_name("led_pwm");
	if (himax_en_pwm <= 0)
		himax_en_pwm = 182;
	gpio_request(himax_en_pwm, "himax_display");

	p_funcs->reset = himax_cmd_panel_reset;
	p_funcs->power_on = himax_cmd_power_on;
	p_funcs->power_off = himax_cmd_power_off;
	p_funcs->drv_ic_init = himax_cmd_drv_ic_init;
	p_funcs->get_config_mode = himax_cmd_get_config_mode;
	p_funcs->get_panel_info = himax_cmd_get_panel_info;
	p_funcs->dsi_controller_init = himax_cmd_controller_init;
	p_funcs->detect = himax_cmd_panel_connection_detect;
	p_funcs->set_brightness = himax_cmd_set_brightness;

	/* debugfs */
	dbgfs_dsi_config = NULL;

	dbgfs.dir = debugfs_create_dir("himax", NULL);
	if (dbgfs.dir == NULL) {
		DRM_ERROR("Cannot create debugfs directory\n");
		return;
	}

	/* atomic ops */
	dbgfs.addr_set = debugfs_create_file("addr",
				S_IFREG | S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP,
				dbgfs.dir, NULL, &dbgfs_addr_ops);
	if (dbgfs.addr_set == NULL)
		DRM_ERROR("Cannot create debugfs entry addr_set\n");

	dbgfs.send_lp = debugfs_create_file("send_lp",
				S_IFREG | S_IWUSR | S_IWGRP,
				dbgfs.dir, NULL, &dbgfs_send_lp_ops);
	if (dbgfs.send_lp == NULL)
		DRM_ERROR("Cannot create debugfs entry send_lp\n");

	dbgfs.send_hs = debugfs_create_file("send_hs",
				S_IFREG | S_IWUSR | S_IWGRP,
				dbgfs.dir, NULL, &dbgfs_send_hs_ops);
	if (dbgfs.send_hs == NULL)
		DRM_ERROR("Cannot create debugfs entry send_hs\n");

	dbgfs.read_lp = debugfs_create_file("read_lp",
				S_IFREG | S_IRUSR | S_IRGRP,
				dbgfs.dir, NULL, &dbgfs_read_lp_ops);
	if (dbgfs.read_lp == NULL)
		DRM_ERROR("Cannot create debugfs entry read_lp\n");

	dbgfs.read_hs = debugfs_create_file("read_hs",
				S_IFREG | S_IRUSR | S_IRGRP,
				dbgfs.dir, NULL, &dbgfs_read_hs_ops);
	if (dbgfs.read_hs == NULL)
		DRM_ERROR("Cannot create debugfs entry read_hs\n");
}

/* atomic operations */
static
ssize_t dbgfs_read(char __user *buff, size_t count,
				loff_t *ppos, enum dbgfs_type type)
{
	char *str;
	u8 data = 0;
	ssize_t len = 0;
	u32 power_island = 0;
	struct mdfld_dsi_pkg_sender *sender =
			mdfld_dsi_get_pkg_sender(dbgfs_dsi_config);

	/* setting display and MIPI bus in correct state for reading */
	if ((type == HIGH_SPEED) || (type == LOW_POWER)) {
		power_island = pipe_to_island(dbgfs_dsi_config->pipe);

		if (power_island & (OSPM_DISPLAY_A | OSPM_DISPLAY_C))
			power_island |= OSPM_DISPLAY_MIO;

		if (!power_island_get(power_island))
			return -EIO;

		mdfld_dsi_dsr_forbid(dbgfs_dsi_config);
	}

	str = kzalloc(count, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	switch (type) {
	case ADDR:
		len = sprintf(str, "addr = 0x%x\n", (u8)dbgfs.addr);
		break;
	case HIGH_SPEED:
		mdfld_dsi_read_mcs_hs(sender, (u8)dbgfs.addr, &data, 1);
		break;
	case LOW_POWER:
		mdfld_dsi_read_mcs_lp(sender, (u8)dbgfs.addr, &data, 1);
		break;
	}

	/* releasing display and MIPI bus */
	if ((type == HIGH_SPEED) || (type == LOW_POWER)) {
		len = sprintf(str, "addr = 0x%x, value = 0x%x\n",
					(u8)dbgfs.addr, data);
		mdfld_dsi_dsr_allow(dbgfs_dsi_config);
		power_island_put(power_island);
	}

	if (len < 0)
		DRM_ERROR("Can't read data\n");
	else
		len = simple_read_from_buffer(buff, count, ppos, str, len);

	kfree(str);

	return len;
}

static
ssize_t dbgfs_addr_read(struct file *file, char __user *buff,
					size_t count, loff_t *ppos)
{
	ssize_t len = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	len = dbgfs_read(buff, count, ppos, ADDR);

	return len;
}

static
ssize_t dbgfs_read_hs_read(struct file *file, char __user *buff,
					size_t count, loff_t *ppos)
{
	ssize_t len;

	if (*ppos < 0 || !count)
		return -EINVAL;

	len = dbgfs_read(buff, count, ppos, HIGH_SPEED);

	return len;
}

static
ssize_t dbgfs_read_lp_read(struct file *file, char __user *buff,
					size_t count, loff_t *ppos)
{
	ssize_t len;

	if (*ppos < 0 || !count)
		return -EINVAL;

	len = dbgfs_read(buff, count, ppos, LOW_POWER);

	return len;
}

static
int dbgfs_write(const char __user *buff, size_t count,
					enum dbgfs_type type)
{
	int err = 0, i = 0, ret = 0;
	unsigned int arg = 0;
	u32 power_island = 0;
	char *start, *str;
	struct mdfld_dsi_pkg_sender *sender =
			mdfld_dsi_get_pkg_sender(dbgfs_dsi_config);

	str = kzalloc(count, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	if (copy_from_user(str, buff, count)) {
		ret = -EFAULT;
		goto exit_dbgfs_write;
	}

	start = str;

	while (*start == ' ')
		start++;

	/* strip ending whitespace */
	for (i = count - 1; i > 0 && isspace(str[i]); i--)
		str[i] = 0;

	/* setting display and MIPI bus in correct state for writting */
	if ((type == HIGH_SPEED) || (type == LOW_POWER)) {
		power_island = pipe_to_island(dbgfs_dsi_config->pipe);

		if (power_island & (OSPM_DISPLAY_A | OSPM_DISPLAY_C))
			power_island |= OSPM_DISPLAY_MIO;

		if (!power_island_get(power_island)) {
			ret = -EIO;
			goto exit_dbgfs_write;
		}

		mdfld_dsi_dsr_forbid_locked(dbgfs_dsi_config);

		if (kstrtouint(start, 16, &arg)) {
			ret = -EINVAL;
			goto exit_dbgfs_write;
		}
	}

	switch (type) {
	case ADDR:
		if (kstrtouint(start, 16, &(dbgfs.addr)))
			ret = -EINVAL;
		break;
	case HIGH_SPEED:
		err = mdfld_dsi_send_mcs_short_hs(sender,
					(u8)dbgfs.addr, (u8)arg, 1,
					MDFLD_DSI_SEND_PACKAGE);
		break;
	case LOW_POWER:
		err = mdfld_dsi_send_mcs_short_lp(sender,
					(u8)dbgfs.addr, (u8)arg, 1,
					MDFLD_DSI_SEND_PACKAGE);
		break;
	}

	if (err) {
		DRM_ERROR("Write failed\n");
		ret = -EIO;
	}

	/* releasing display and MIPI bus */
	if ((type == HIGH_SPEED) || (type == LOW_POWER)) {
		mdfld_dsi_dsr_allow_locked(dbgfs_dsi_config);
		power_island_put(power_island);
	}

exit_dbgfs_write:
	kfree(str);
	return ret;
}

static
ssize_t dbgfs_addr_write(struct file *file, const char __user *buff,
						size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	int err = 0;

	ret = count;

	if (*ppos < 0 || !count)
		return -EINVAL;

	err = dbgfs_write(buff, count, ADDR);
	if (err < 0)
		ret = 0;

	*ppos += ret;

	return ret;
}

static
ssize_t dbgfs_send_hs_write(struct file *file, const char __user *buff,
						size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	int err = 0;

	ret = count;

	if (*ppos < 0 || !count)
		return -EINVAL;

	err = dbgfs_write(buff, count, HIGH_SPEED);
	if (err < 0)
		ret = 0;

	*ppos += ret;

	return count;
}

static
ssize_t dbgfs_send_lp_write(struct file *file, const char __user *buff,
						size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	int err = 0;

	ret = count;

	if (*ppos < 0 || !count)
		return -EINVAL;

	err = dbgfs_write(buff, count, LOW_POWER);
	if (err < 0)
		ret = 0;

	*ppos += ret;

	return count;
}
