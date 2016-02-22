/*
 *  ASoc DPCM Machine driver for Intel CDP MID platform
 *
 *  Copyright (C) 2014-2016 Intel Corp
 *  Author: Michael Soares <michaelx.soares@intel.com>
 *  Author: Mattijs Korpershoek <mattijsx.korpershoek@intel.com>
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
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <asm/intel_sst_mrfld.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

static int ssp0_config_fixup(struct snd_soc_dai_link *dai_link, struct snd_soc_dai *dai)
{
	int ret;

	/* tx_mask = 0 | rx_mask = 3 | 2 slots | 24 bits */
	ret = snd_soc_dai_set_tdm_slot(dai, 0x0, 0x3, 0x2, SNDRV_PCM_FORMAT_S24_LE);
	if (ret < 0) {
		pr_err("can't set codec pcm format: %d\n", ret);
		return ret;
	}

	/* I2S | DSP is master | framesync active high  */
	ret = snd_soc_dai_set_fmt(dai,
			SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_CBS_CFS |
			SND_SOC_DAIFMT_NB_IF);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration: %d\n", ret);
		return ret;
	}

	return ret;
}

static int ssp1_config_fixup(struct snd_soc_dai_link *dai_link, struct snd_soc_dai *dai)
{
	int ret;

	/* tx_mask = 3 | rx_mask = 0 | 2 slots | 16 bits */
	ret = snd_soc_dai_set_tdm_slot(dai, 0x3, 0x0, 0x2, SNDRV_PCM_FORMAT_S16_LE);
	if (ret < 0) {
		pr_err("can't set bt i2s format: %d\n", ret);
		return ret;
	}

	/* I2S | DSP is master | framesync active high  */
	ret = snd_soc_dai_set_fmt(dai,
			SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_CBS_CFS |
			SND_SOC_DAIFMT_NB_IF);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration: %d\n", ret);
		return ret;
	}

	return ret;
}

static const struct snd_soc_pcm_stream ssp0_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_PCM_RATE_16000,
	.rate_max = SNDRV_PCM_RATE_16000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream ssp1_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = SNDRV_PCM_RATE_48000,
	.rate_max = SNDRV_PCM_RATE_48000,
	.channels_min = 2,
	.channels_max = 2,
};

static struct snd_soc_dai_link dai_links[] = {
	/* front ends */
	[MERR_DPCM_AUDIO] = {
		.name = "Capture Audio Port",
		.stream_name = "CDP DMIC",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
	},
	[MERR_DPCM_DB] = {
		.name = "DB Audio Port",
		.stream_name = "Deep Buffer Audio",
		.cpu_dai_name = "Deepbuffer-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
	},
	/* back-end <-> back-end link */
	{
		.name = "DMIC-Loop Port",
		.stream_name = "CDP DMIC-Loop",
		.cpu_dai_name = "ssp0-port",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.params = &ssp0_dai_params,
		.be_fixup = ssp0_config_fixup,
		.dsp_loopback = true,
	},
	{
		.name = "BT-Loop Port",
		.stream_name = "CDP BT-Loop",
		.cpu_dai_name = "ssp1-port",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.params = &ssp1_dai_params,
		.be_fixup = ssp1_config_fixup,
		.dsp_loopback = true,
	},
	/* back-ends */
	{
		.name = "SSP0-DMIC",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.be_id = 3,
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
	{
		.name = "SSP1-BT",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.be_id = 2,
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
};

static struct snd_soc_dapm_route dapm_route[] = {
	{ "ssp0 Tx", NULL, "modem_out"},
	{ "modem_in", NULL, "ssp0 Rx" },
	{ "ssp1 Tx", NULL, "bt_fm_out"},
	{ "bt_fm_in", NULL, "ssp1 Rx" },
};

static struct snd_soc_card snd_cdp_card = {
	.name = "ics43432-dmic",
	.dai_link = dai_links,
	.num_links = ARRAY_SIZE(dai_links),
	.dapm_routes = dapm_route,
	.num_dapm_routes = ARRAY_SIZE(dapm_route),
};

#ifdef CONFIG_PM_SLEEP
static int snd_cdp_pm_prepare(struct device *dev)
{
	return snd_soc_suspend(dev);
}

static void snd_cdp_pm_complete(struct device *dev)
{
	snd_soc_resume(dev);
}

static int snd_cdp_pm_poweroff(struct device *dev)
{
	return snd_soc_poweroff(dev);
}
#else
#define snd_cdp_pm_prepare NULL
#define snd_cdp_pm_complete NULL
#define snd_cdp_pm_poweroff NULL
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops snd_cdp_pm_ops = {
	.prepare = snd_cdp_pm_prepare,
	.complete = snd_cdp_pm_complete,
	.poweroff = snd_cdp_pm_poweroff,
};

static int snd_cdp_probe(struct platform_device *pdev)
{
	int ret;

	snd_cdp_card.dev = &pdev->dev;
	ret = snd_soc_register_card(&snd_cdp_card);
	if (ret) {
		pr_err("snd_soc_register_card failed: %d\n", ret);
		return ret;
	}
	platform_set_drvdata(pdev, &snd_cdp_card);
	pr_info("%s successful\n", __func__);

	return ret;
}

static int snd_cdp_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver snd_cdp_drv = {
	.driver = {
			.owner = THIS_MODULE,
			.name = "cdp_dpcm_ics43432",
			.pm = &snd_cdp_pm_ops,
	},
	.probe = snd_cdp_probe,
	.remove = snd_cdp_remove,
};
module_platform_driver(snd_cdp_drv);

MODULE_DESCRIPTION("ASoC Intel(R) CDP MID Machine driver");
MODULE_AUTHOR("Michael Soares <michaelx.soares@intel.com>");
MODULE_AUTHOR("Mattijs Korpershoek <mattijsx.korpershoek@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cdp_dpcm_ics43432");
