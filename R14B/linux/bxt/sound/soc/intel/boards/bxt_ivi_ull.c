/*
 * Intel Broxton-P I2S ULL Machine Driver
 *
 * Copyright (C) 2017, Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

/* widgets*/
static const struct snd_soc_dapm_widget widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("DiranaCp", NULL),
	SND_SOC_DAPM_HP("DiranaPb", NULL),
	SND_SOC_DAPM_MIC("ANCMic", NULL),
#ifndef CONFIG_DIRANA_DAI_OPS
	SND_SOC_DAPM_MIC("HdmiIn", NULL),
	SND_SOC_DAPM_MIC("TestPinCp", NULL),
	SND_SOC_DAPM_HP("TestPinPb", NULL),
#else
	SND_SOC_DAPM_MIC("Dirana2Cp", NULL),
	SND_SOC_DAPM_MIC("MicRef2Cp", NULL),
	SND_SOC_DAPM_MIC("DmicCp", NULL),
#endif
	SND_SOC_DAPM_MIC("BtHfpDl", NULL),
	SND_SOC_DAPM_HP("BtHfpUl", NULL),
	SND_SOC_DAPM_MIC("ModemDl", NULL),
	SND_SOC_DAPM_HP("ModemUl", NULL),
};

/* route map */
static const struct snd_soc_dapm_route map[] = {
        /* ANC loop in */
        {"codec0_in", NULL, "ssp4-b Rx"},
        {"ssp4-b Rx", NULL, "ANCMic"},

#ifndef CONFIG_DIRANA_DAI_OPS
        /* Speaker BE connections */
        { "Speaker", NULL, "ssp4 Tx"},
        { "ssp4 Tx", NULL, "codec0_out"},
#else
	/* Dirana3 */
        { "dirana_in", NULL, "ssp2 Rx"},
        { "ssp2 Rx", NULL, "DiranaCp"},
#endif
#ifndef CONFIG_DIRANA_DAI_OPS
        { "dirana_aux_in", NULL, "ssp2 Rx"},
        { "ssp2 Rx", NULL, "DiranaCp"},

        { "dirana_tuner_in", NULL, "ssp2 Rx"},
        { "ssp2 Rx", NULL, "DiranaCp"},
#else
        /* ANC loop out */
        { "DiranaPb", NULL, "ssp2 Tx"},
        { "ssp2 Tx", NULL, "dirana_out"},
#endif
#ifndef CONFIG_DIRANA_DAI_OPS
        { "hdmi_ssp1_in", NULL, "ssp1 Rx"},
        { "ssp1 Rx", NULL, "HdmiIn"},
#else
   	/* Dirana3-2 */
        { "dirana2_in", NULL, "ssp3 Rx"},
        { "ssp3 Rx", NULL, "Dirana2Cp"},
	/*MicRef2 for PSSE */
        { "micref2", NULL, "ssp3-b Rx"},
        { "ssp3-b Rx", NULL, "MicRef2Cp"},
#endif
#ifndef CONFIG_DIRANA_DAI_OPS
        { "TestPin_ssp5_in", NULL, "ssp5 Rx"},
        { "ssp5 Rx", NULL, "TestPinCp"},

        { "TestPinPb", NULL, "ssp5 Tx"},
        { "ssp5 Tx", NULL, "TestPin_ssp5_out"},
#else
        /* DMIC */
        { "dmic_in", NULL, "ssp4 Rx"},
        { "ssp4 Rx", NULL, "DmicCp"},
#endif
	/* Bluetooth HFP */
        { "BtHfp_ssp0_in", NULL, "ssp0 Rx"},
        { "ssp0 Rx", NULL, "BtHfpDl"},

        { "BtHfpUl", NULL, "ssp0 Tx"},
        { "ssp0 Tx", NULL, "BtHfp_ssp0_out"},

	/* Modem Telit */
#ifndef CONFIG_DIRANA_DAI_OPS
        { "Modem_ssp3_in", NULL, "ssp3 Rx"},
        { "ssp3 Rx", NULL, "ModemDl"},

        { "ModemUl", NULL, "ssp3 Tx"},
        { "ssp3 Tx", NULL, "Modem_ssp3_out"},
#else
 	{ "Modem_ssp5_in", NULL, "ssp5 Rx"},
        { "ssp5 Rx", NULL, "ModemDl"},

        { "ModemUl", NULL, "ssp5 Tx"},
        { "ssp5 Tx", NULL, "Modem_ssp5_out"},
#endif
};

static const struct snd_soc_pcm_stream codec0_in_params = {
  .formats = SNDRV_PCM_FMTBIT_S32_LE,
  .rate_min = 48000,
  .rate_max = 48000,
  .channels_min = 8,
  .channels_max = 8,
};

static const struct snd_soc_pcm_stream media0_out_params = {
  .formats = SNDRV_PCM_FMTBIT_S32_LE,
  .rate_min = 48000,
  .rate_max = 48000,
  .channels_min = 8,
  .channels_max = 8,
};

static int bxt_tdf8532_ssp2_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	/* set SSP to 32 bit */
	snd_mask_none(fmt);
	snd_mask_set(fmt, SNDRV_PCM_FORMAT_S32_LE);

	return 0;
}

/* DAI (digital audio interface) glue- connect codec - CPU */
static struct snd_soc_dai_link dais[] = {
	/* Front End DAI links */
	{
		.name = "Speaker Port",
		.stream_name = "Speaker",
		.cpu_dai_name = "Speaker Pin",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_playback = 1,
	},
	{
		.name = "Dirana Capture Port",
		.stream_name = "Dirana Cp",
		.cpu_dai_name = "Dirana Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Dirana Playback Port",
		.stream_name = "Dirana Pb",
		.cpu_dai_name = "Dirana Pb Pin",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_playback = 1,
	},
#ifndef CONFIG_DIRANA_DAI_OPS
	{
		.name = "TestPin Capture Port",
		.stream_name = "TestPin Cp",
		.cpu_dai_name = "TestPin Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "TestPin Playback Port",
		.stream_name = "TestPin Pb",
		.cpu_dai_name = "TestPin Pb Pin",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_playback = 1,
	},
#endif
	{
		.name = "BtHfp Capture Port",
		.stream_name = "BtHfp Cp",
		.cpu_dai_name = "BtHfp Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "BtHfp Playback Port",
		.stream_name = "BtHfp Pb",
		.cpu_dai_name = "BtHfp Pb Pin",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_playback = 1,
	},
	{
		.name = "Modem Capture Port",
		.stream_name = "Modem Cp",
		.cpu_dai_name = "Modem Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Modem Playback Port",
		.stream_name = "Modem Pb",
		.cpu_dai_name = "Modem Pb Pin",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_playback = 1,
	},
	{
#ifndef CONFIG_DIRANA_DAI_OPS
		.name = "HDMI Capture Port",
		.stream_name = "HDMI Cp",
		.cpu_dai_name = "HDMI Cp Pin",
#else
		.name = "Dirana2 Cp Port",
		.stream_name = "Dirana2 Cp",
		.cpu_dai_name = "Dirana2 Cp Pin",
#endif
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
#ifndef CONFIG_DIRANA_DAI_OPS
		.name = "Dirana Aux Capture Port",
		.stream_name = "Dirana Aux Cp",
		.cpu_dai_name = "Dirana Aux Cp Pin",
#else
		.name = "Dmic Cp Port",
		.stream_name = "Dmic Cp",
		.cpu_dai_name = "Dmic Cp Pin",
#endif
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "MicRef2 Cp Port",
		.stream_name = "MicRef2 Cp",
		.cpu_dai_name = "MicRef2 Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
    /* loopbacks */
    {
        .name = "ANC loopback in port",
        .stream_name = "ANC loopback in",
        .cpu_dai_name = "SSP4-B Pin",
        .platform_name = "0000:00:0e.0",
        .codec_name = "snd-soc-dummy",
        .codec_dai_name = "snd-soc-dummy-dai",
        .params = &codec0_in_params,
        .dsp_loopback = 1,
    },
    {
        .name = "ANC loopback out Port",
        .stream_name = "ANC loopback out",
        .cpu_dai_name = "SSP2 Pin",
        .platform_name = "0000:00:0e.0",
        .codec_name = "snd-soc-dummy",
        .codec_dai_name = "snd-soc-dummy-dai",
        .params = &media0_out_params,
        .dsp_loopback = 1,
    },

#ifndef CONFIG_DIRANA_DAI_OPS
	{
		.name = "Dirana Tuner Capture Port",
		.stream_name = "Dirana Tuner Cp",
		.cpu_dai_name = "Dirana Tuner Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
#endif
	/* Probe DAI links*/
	{
		.name = "Bxt Compress Probe playback",
		.stream_name = "Probe Playback",
		.cpu_dai_name = "Compress Probe0 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.nonatomic = 1,
	},
	{
		.name = "Bxt Compress Probe capture",
		.stream_name = "Probe Capture",
		.cpu_dai_name = "Compress Probe1 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.nonatomic = 1,
	},
	/* Trace Buffer DAI links */
	{
		.name = "Bxt Trace Buffer0",
		.stream_name = "Core 0 Trace Buffer",
		.cpu_dai_name = "TraceBuffer0 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.capture_only = true,
		.ignore_suspend = 1,
	},
	{
		.name = "Bxt Trace Buffer1",
		.stream_name = "Core 1 Trace Buffer",
		.cpu_dai_name = "TraceBuffer1 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.capture_only = true,
		.ignore_suspend = 1,
	},
	/* Back End DAI links */
	{
		/* SSP0 - BT */
		.name = "SSP0-Codec",
		.id = 0,
		.cpu_dai_name = "SSP0 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP1 - HDMI-In */
		.name = "SSP1-Codec",
		.id = 1,
		.cpu_dai_name = "SSP1 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
	},
	{
		/* SSP2 - Dirana */
		.name = "SSP2-Codec",
		.id = 2,
		.cpu_dai_name = "SSP2 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
		.be_hw_params_fixup = bxt_tdf8532_ssp2_fixup,
	},
	{
		/* SSP3 - Modem */
		.name = "SSP3-Codec",
		.id = 3,
		.cpu_dai_name = "SSP3 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP4 - Amplifier */
		.name = "SSP4-Codec",
		.id = 4,
		.cpu_dai_name = "SSP4 Pin",
#ifndef CONFIG_DIRANA_DAI_OPS
		.codec_name = "i2c-INT34C3:00",
		.codec_dai_name = "tdf8532-hifi",
#else
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_capture = 1,
#endif
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP5 - TestPin */
		.name = "SSP5-Codec",
		.id = 5,
		.cpu_dai_name = "SSP5 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP3-B - PSSE */
		.name = "SSP3-B-Codec",
		.id = 6,
		.cpu_dai_name = "SSP3-B Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP4-B ANC mic loopback in*/
		.name = "SSP4-B for ANC mic in",
		.id = 7,
		.cpu_dai_name = "SSP4-B Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
};

/* pass all info to card */
static struct snd_soc_card audio_card = {
  .name = "broxton_ull",
  .owner = THIS_MODULE,
  .dai_link = dais,
  .num_links = ARRAY_SIZE(dais),
  .dapm_widgets = widgets,
  .num_dapm_widgets = ARRAY_SIZE(widgets),
  .dapm_routes = map,
  .num_dapm_routes = ARRAY_SIZE(map),
  .fully_routed = false,
};

static int broxton_audio_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s registering %s\n", __func__, pdev->name);
	audio_card.dev = &pdev->dev;
	return snd_soc_register_card(&audio_card);
}

static int broxton_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&audio_card);
	return 0;
}

static struct platform_driver broxton_audio = {
	.probe = broxton_audio_probe,
	.remove = broxton_audio_remove,
	.driver = {
		.name = "bxt_ivi_ull",
		.pm = &snd_soc_pm_ops,
	},
};

module_platform_driver(broxton_audio)

/* Module information */
MODULE_DESCRIPTION("Intel SST Audio for Broxton ULL Machine");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bxtp_i2s_ull");
MODULE_AUTHOR("Jakub Dorzak <jakubx.dorzak@intel.com>");
