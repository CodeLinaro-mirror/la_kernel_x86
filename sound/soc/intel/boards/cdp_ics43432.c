/*
 *  cdp_ics43432.c - ASoc Machine driver for Intel CloverDale Peak platform
 *
 *  Copyright (C) 2016 Intel Corp
 *  Author: Mattijs Korpershoek <mattijsx.korpershoek@intel.com>
 *          Sebastien Guiriec <sebastien.guiriec@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/dmi.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../atom/sst-atom-controls.h"
#include "../atom/sst-mfld-platform.h"

static const struct snd_soc_dapm_widget cdp_ics43432_widgets[] = {
	SND_SOC_DAPM_MIC("Dmic", NULL),
	SND_SOC_DAPM_OUTPUT("BT chip"),
};

static const struct snd_soc_dapm_route cdp_ics43432_audio_map[] = {
	{"modem_in", NULL, "ssp0 Rx"},
	{"ssp0 Rx", NULL, "Dmic"},
	{"BT chip", NULL, "ssp1 Tx"},
	{"ssp1 Tx", NULL, "bt_out"},
};

static int cdp_ics43432_dmic_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	struct sst_data *drv = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	int ret;

	/* The DSP will convert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP0 to 24-bit */
	params_set_format(params, SNDRV_PCM_FORMAT_S24_LE);

	/* As Default mode for SSP configuration is TDM 4 channel we need to override
	 * default setting in order to switch to I2S for Aduio DSP engine configuration */
	ret = snd_soc_dai_set_fmt(rtd->cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF
						| SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec I2S mode %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 24);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set SSP TDM slots in I2S mode %d\n", ret);
		return ret;
	}

	/* force frame_sync_width = 64 to trigger ASRC in DSP firmware */
	drv->ssp_cmd.frame_sync_width = 64;
	/* ASRC is 16.667 -> 48kHz, force freq to 16kHz */
	drv->ssp_cmd.frame_sync_frequency = SSP_FS_16_KHZ;

	return 0;
}

static int cdp_ics43432_bt_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	struct sst_data *drv = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	int ret;

	/* The DSP will convert the FE rate to 48k, stereo, 16bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP1 to 16-bit */
	params_set_format(params, SNDRV_PCM_FORMAT_S16_LE);

	/* As Default mode for SSP configuration is TDM 4 channel we need to override
	 * default setting in order to switch to I2S for Audio DSP engine configuration */
	ret = snd_soc_dai_set_fmt(rtd->cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF
						| SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set I2S for bt chip mode %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 16);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set slots for bt chip %d\n", ret);
		return ret;
	}

	return 0;
}

static int cdp_ics43432_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_single(substream->runtime,
			SNDRV_PCM_HW_PARAM_RATE, 48000);
}

static struct snd_soc_ops cdp_ics43432_aif1_ops = {
	.startup = cdp_ics43432_aif1_startup,
};

static struct snd_soc_dai_link cdp_ics43432_dais[] = {
	[MERR_DPCM_AUDIO] = {
		.name = "Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "media-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
		.ignore_suspend = 1,
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_capture = 1,
		.ops = &cdp_ics43432_aif1_ops,
	},
	[MERR_DPCM_DEEP_BUFFER] = {
		.name = "Deepbuffer",
		.stream_name = "Deepbuffer",
		.cpu_dai_name = "deepbuffer-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
		.ignore_suspend = 1,
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.ops = &cdp_ics43432_aif1_ops,
	},
	/* back ends */
	{
	  .name = "SSP0-DMIC",
	  .be_id = SSP_MODEM,
	  .cpu_dai_name = "ssp0-port",
	  .platform_name = "sst-mfld-platform",
	  .no_pcm = 1,
	  .codec_dai_name = "ics43432-hifi",
	  .codec_name = "ics43432",
	  .be_hw_params_fixup = cdp_ics43432_dmic_fixup,
	  .ignore_suspend = 1,
	  .nonatomic = true,
	  .dpcm_capture = 1,
	},
	{
	  .name = "SSP1-BT",
	  .be_id = SSP_BT,
	  .cpu_dai_name = "ssp1-port",
	  .platform_name = "sst-mfld-platform",
	  .no_pcm = 1,
	  .codec_dai_name = "snd-soc-dummy-dai",
	  .codec_name = "snd-soc-dummy",
	  .be_hw_params_fixup = cdp_ics43432_bt_fixup,
	  .ignore_suspend = 1,
	  .nonatomic = true,
	  .dpcm_playback = 1,
	},
};

/* SoC card */
static struct snd_soc_card cdp_ics43432_card = {
	.name = "cdp-ics43432",
	.owner = THIS_MODULE,
	.dai_link = cdp_ics43432_dais,
	.num_links = ARRAY_SIZE(cdp_ics43432_dais),
	.dapm_widgets = cdp_ics43432_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cdp_ics43432_widgets),
	.dapm_routes = cdp_ics43432_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cdp_ics43432_audio_map),
};

static int snd_cdp_ics43432_mc_probe(struct platform_device *pdev)
{
	int ret;

	/* register the soc card */
	cdp_ics43432_card.dev = &pdev->dev;

	ret = devm_snd_soc_register_card(&pdev->dev, &cdp_ics43432_card);

	if (ret) {
		dev_err(&pdev->dev, "devm_snd_soc_register_card failed %d\n",
			ret);
		return ret;
	}
	platform_set_drvdata(pdev, &cdp_ics43432_card);

	return ret;
}

static struct platform_driver snd_cdp_drv = {
	.driver = {
			.name = "cdp_ics43432",
			.pm = &snd_soc_pm_ops,
	},
	.probe = snd_cdp_ics43432_mc_probe,
};
module_platform_driver(snd_cdp_drv);


MODULE_DESCRIPTION("ASoC Intel(R) CDP MID Machine driver");
MODULE_AUTHOR("Mattijs Korpershoek <mattijsx.korpershoek@intel.com>");
MODULE_AUTHOR("Sebastien Guiriec <sebastien.guiriec@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cdp_dpcm_ics43432");
