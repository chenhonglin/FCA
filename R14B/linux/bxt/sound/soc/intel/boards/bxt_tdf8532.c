/*
 * Intel Broxton-P I2S Machine Driver for IVI reference platform
 * Copyright (c) 2017, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

static const struct snd_kcontrol_new broxton_tdf8532_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

#ifdef CONFIG_SND_SOC_GWM_ALSA
static const struct snd_soc_dapm_widget broxton_tdf8532_widgets[] = {
	SND_SOC_DAPM_MIC("DiranaTunerCp", NULL),
	SND_SOC_DAPM_HP("DiranaPb", NULL),
	SND_SOC_DAPM_MIC("BtHfpDl", NULL),
	SND_SOC_DAPM_HP("BtHfpUl", NULL),
	SND_SOC_DAPM_MIC("micCp", NULL),
	SND_SOC_DAPM_MIC("tboxCp", NULL),
	SND_SOC_DAPM_MIC("auxCp", NULL),
	SND_SOC_DAPM_MIC("tunnerCp", NULL),
	SND_SOC_DAPM_MIC("sxmCp", NULL),
	SND_SOC_DAPM_MIC("mic1Cp", NULL),
	SND_SOC_DAPM_MIC("DiranaTunerCp1", NULL),
        SND_SOC_DAPM_MIC("refCp", NULL),
};
#elif defined(CONFIG_SND_SOC_BJEVN60_ALSA)
static const struct snd_soc_dapm_widget broxton_tdf8532_widgets[] = {
	SND_SOC_DAPM_MIC("SilabsTunerCp", NULL),
	SND_SOC_DAPM_HP("SilabsPb", NULL),
	SND_SOC_DAPM_MIC("BtHfpDl", NULL),
	SND_SOC_DAPM_HP("BtHfpUl", NULL),
	SND_SOC_DAPM_MIC("micCp", NULL),
	SND_SOC_DAPM_MIC("tboxCp", NULL),
	SND_SOC_DAPM_MIC("auxCp", NULL),
	SND_SOC_DAPM_MIC("tunnerCp", NULL),
	SND_SOC_DAPM_MIC("sxmCp", NULL),
	SND_SOC_DAPM_HP("ClusterChimePb", NULL),
};
#elif defined(CONFIG_SND_SOC_GWM_V3_ALSA)
static const struct snd_soc_dapm_widget broxton_tdf8532_widgets[] = {
	SND_SOC_DAPM_MIC("SilabsTunerCp", NULL),
	SND_SOC_DAPM_HP("SilabsPb", NULL),
	SND_SOC_DAPM_MIC("BtHfpDl", NULL),
	SND_SOC_DAPM_HP("BtHfpUl", NULL),
	SND_SOC_DAPM_HP("micCp", NULL),
	SND_SOC_DAPM_HP("tboxCp", NULL),
	SND_SOC_DAPM_HP("auxCp", NULL),
	SND_SOC_DAPM_HP("tunnerCp", NULL),
	SND_SOC_DAPM_HP("sxmCp", NULL),
	SND_SOC_DAPM_HP("ClusterChimePb", NULL),
};
#else
static const struct snd_soc_dapm_widget broxton_tdf8532_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("DiranaCp", NULL),
	SND_SOC_DAPM_HP("DiranaPb", NULL),
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
#endif

#ifdef CONFIG_SND_SOC_GWM_ALSA
static const struct snd_soc_dapm_route broxton_tdf8532_map[] = {
	{ "dirana_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx",   NULL, "DiranaTunerCp"},

	{ "dirana_in1", NULL, "ssp3 Rx"},
	{ "ssp3 Rx",   NULL, "DiranaTunerCp1"},

	{ "mic_in",  NULL,  "ssp3-b Rx"},
	{ "ssp3-b Rx", NULL,  "micCp"},

	{ "tbox_in", NULL,  "ssp2-b Rx"},
	{ "ssp2-b Rx", NULL,  "tboxCp"},

	{ "aux_in", NULL, "ssp2-c Rx"},
	{ "ssp2-c Rx",NULL, "auxCp"},

	{ "tunner_in", NULL, "ssp2-d Rx"},
	{ "ssp2-d Rx",   NULL, "tunnerCp"},

	{ "sxm_in", NULL,"ssp2-e Rx"},
	{ "ssp2-e Rx",NULL,"sxmCp"},

	{ "mic1_in", NULL, "ssp3-c Rx"},
	{ "ssp3-c Rx", NULL, "mic1Cp"},

	{ "DiranaPb", NULL, "ssp2 Tx"},
	{ "ssp2 Tx",  NULL, "dirana_out"},

        { "ref_in", NULL, "ssp3-d Rx"},
        { "ssp3-d Rx", NULL, "refCp"},

	{ "BtHfp_ssp0_in", NULL, "ssp0 Rx"},
	{ "ssp0 Rx",       NULL, "BtHfpDl"},

	{ "BtHfpUl", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "BtHfp_ssp0_out"},
};
#elif defined(CONFIG_SND_SOC_BJEVN60_ALSA)
static const struct snd_soc_dapm_route broxton_tdf8532_map[] = {
	{ "silabs_in", NULL, "ssp5 Rx"},
	{ "ssp5 Rx",   NULL, "SilabsTunerCp"},

	{ "mic_in",  NULL,  "ssp5-b Rx"},
	{ "ssp5-b Rx", NULL,  "micCp"},

	{ "tbox_in", NULL,  "ssp5-c Rx"},
	{ "ssp5-c Rx", NULL,  "tboxCp"},

	{ "aux_in", NULL, "ssp5-d Rx"},
	{ "ssp5-d Rx", NULL, "auxCp"},

	{ "tunner_in", NULL, "ssp5-e Rx"},
	{ "ssp5-e Rx",   NULL, "tunnerCp"},

	{ "sxm_in", NULL,"ssp1 Rx"},
	{ "ssp1 Rx",NULL,"sxmCp"},

	{ "ClusterChimePb", NULL, "ssp4 Tx"},
	{ "ssp4 Tx", NULL, "cluster_chime"},

	{ "SilabsPb", NULL, "ssp5 Tx"},
	{ "ssp5 Tx",  NULL, "silabs_out"},

	{ "BtHfp_ssp0_in", NULL, "ssp0 Rx"},
	{ "ssp0 Rx",       NULL, "BtHfpDl"},

	{ "BtHfpUl", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "BtHfp_ssp0_out"},

};
#elif defined(CONFIG_SND_SOC_GWM_V3_ALSA)
static const struct snd_soc_dapm_route broxton_tdf8532_map[] = {
	{ "silabs_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx",   NULL, "SilabsTunerCp"},

	{ "mic_in",  NULL,  "ssp2 Rx"},
	{ "ssp2 Rx", NULL,  "micCp"},

	{ "tbox_in", NULL,  "ssp2 Rx"},
	{ "ssp2 Rx", NULL,  "tboxCp"},

	{ "aux_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx",NULL, "auxCp"},

	{ "tunner_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx",   NULL, "tunnerCp"},

	{ "sxm_in", NULL,"ssp1 Rx"},
	{ "ssp1 Rx",NULL,"sxmCp"},

	{ "ClusterChimePb", NULL, "ssp4 Tx"},
	{ "ssp4 Tx", NULL, "cluster_chime"},

	{ "SilabsPb", NULL, "ssp2 Tx"},
	{ "ssp2 Tx",  NULL, "silabs_out"},

	{ "BtHfp_ssp0_in", NULL, "ssp0 Rx"},
	{ "ssp0 Rx",       NULL, "BtHfpDl"},

	{ "BtHfpUl", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "BtHfp_ssp0_out"},

};
#else
static const struct snd_soc_dapm_route broxton_tdf8532_map[] = {
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

	/* MicRef2 */
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
#endif
static int bxt_tdf8532_ssp2_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	/* set SSP to 32 bit */
	snd_mask_none(fmt);
	snd_mask_set(fmt, SNDRV_PCM_FORMAT_S32_LE);

	return 0;
}

#ifdef CONFIG_SND_SOC_GWM_ALSA
/* broxton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link broxton_tdf8532_dais[] = {
	/* Front End DAI links */
	{
		.name = "Dirana main Port",
		.stream_name = "Dirana main",
		.cpu_dai_name = "Dirana main pin",
		.platform_name = "0000:00:0e.0",
        .init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_playback = 1,
        .dpcm_capture = 1,
	},
	{
		.name = "BtHfp Chip Port",
		.stream_name = "BtHfp chip",
		.cpu_dai_name = "BtHfp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
        .dpcm_playback = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Dirana Mic Cp Port",
		.stream_name = "Mic Cp",
		.cpu_dai_name = "mic pin",
		.platform_name = "0000:00:0e.0",
        .init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_capture = 1,
	},
    {
        .name = "Dirana TBox Cp Port",
		.stream_name = "Tbox Cp",
		.cpu_dai_name = "tbox pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
    },
    {
     	.name = "Dirana Aux Cp Port",
		.stream_name = "Aux Cp",
		.cpu_dai_name = "aux pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
    },
    {
     	.name = "Dirana Tunner Cp Port",
		.stream_name = "Tunner Cp",
		.cpu_dai_name = "tunner pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
    },
    {
     	.name = "Dirana SXM Cp Port",
		.stream_name = "SXM Cp",
		.cpu_dai_name = "sxm pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
    },
    {
        .name = "Dirana Mic1 Cp Port",
		.stream_name = "Mic1 Cp",
		.cpu_dai_name = "mic1 pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
    },
    {
	.name = "Dirana Second Capture Port",
		.stream_name = "Dirana Second Cp",
		.cpu_dai_name = "Dirana second pin",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_capture = 1,
    },
    {
        .name = "Dirana Ref Cp Port",
                .stream_name = "Ref Cp",
                .cpu_dai_name = "ref pin",
                .platform_name = "0000:00:0e.0",
                .init = NULL,
                .nonatomic = 1,
                .dynamic = 1,
                .codec_name = "snd-soc-dummy",
                .codec_dai_name = "snd-soc-dummy-dai",
                .dpcm_capture = 1,
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
		/* SSP3 */
		.name = "SSP3-B-Codec",
		.id = 3,
		.cpu_dai_name = "SSP3-B Pin",
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
		/* SSP2 */
		.name = "SSP2-B-Codec",
		.id = 4,
		.cpu_dai_name = "SSP2-B Pin",
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
		/* SSP2 */
		.name = "SSP2-C-Codec",
		.id = 5,
		.cpu_dai_name = "SSP2-C Pin",
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
		/* SSP2 */
		.name = "SSP2-D-Codec",
		.id = 6,
		.cpu_dai_name = "SSP2-D Pin",
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
		/* SSP2 */
		.name = "SSP2-E-Codec",
		.id = 7,
		.cpu_dai_name = "SSP2-E Pin",
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
		/* SSP3 */
		.name = "SSP3-C-Codec",
		.id = 8,
		.cpu_dai_name = "SSP3-C Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP3 */
		.name = "SSP3-D-Codec",
		.id = 9,
		.cpu_dai_name = "SSP3-D Pin",
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
		.id = 10,
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
		.id = 11,
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
		.id = 12,
		.cpu_dai_name = "SSP5 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
};
#elif defined(CONFIG_SND_SOC_BJEVN60_ALSA)
/* broxton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link broxton_tdf8532_dais[] = {
	/* Front End DAI links */
	{
		.name = "Silabs main Port",
		.stream_name = "Silabs main",
		.cpu_dai_name = "Silabs main pin",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "BtHfp Chip Port",
		.stream_name = "BtHfp chip",
		.cpu_dai_name = "BtHfp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs Mic Cp Port",
		.stream_name = "Mic Cp",
		.cpu_dai_name = "mic pin",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_capture = 1,
	},
	{
		.name = "Silabs TBox Cp Port",
		.stream_name = "Tbox Cp",
		.cpu_dai_name = "tbox pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs Aux Cp Port",
		.stream_name = "Aux Cp",
		.cpu_dai_name = "aux pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs Tunner Cp Port",
		.stream_name = "Tunner Cp",
		.cpu_dai_name = "tunner pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs SXM Cp Port",
		.stream_name = "SXM Cp",
		.cpu_dai_name = "sxm pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs Cluster Chime Playback Port",
		.stream_name = "Cluster Chime Pb",
		.cpu_dai_name = "Cluster Chime Pb pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_playback = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},

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
		/* SSP1 - SXM-In */
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
		/* SSP2 */
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
		/* SSP4 - Cluster */
		.name = "SSP4-Codec",
		.id = 4,
		.cpu_dai_name = "SSP4 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_capture = 1,
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP5 */
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
		.be_hw_params_fixup = bxt_tdf8532_ssp2_fixup,
	},
	{
		/* SSP5-B */
		.name = "SSP5-B-Codec",
		.id = 6,
		.cpu_dai_name = "SSP5-B Pin",
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
		/* SSP5-C */
		.name = "SSP5-C-Codec",
		.id = 7,
		.cpu_dai_name = "SSP5-C Pin",
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
		/* SSP5-D */
		.name = "SSP5-D-Codec",
		.id = 8,
		.cpu_dai_name = "SSP5-D Pin",
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
		/* SSP5-E */
		.name = "SSP5-E-Codec",
		.id = 9,
		.cpu_dai_name = "SSP5-E Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
		.be_hw_params_fixup = bxt_tdf8532_ssp2_fixup,
	},
};
#elif defined(CONFIG_SND_SOC_GWM_V3_ALSA)
/* broxton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link broxton_tdf8532_dais[] = {
	/* Front End DAI links */
	{
		.name = "Silabs main Port",
		.stream_name = "Silabs main",
		.cpu_dai_name = "Silabs main pin",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "BtHfp Chip Port",
		.stream_name = "BtHfp chip",
		.cpu_dai_name = "BtHfp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs Mic Cp Port",
		.stream_name = "Mic Cp",
		.cpu_dai_name = "mic pin",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_capture = 1,
	},
	{
		.name = "Silabs TBox Cp Port",
		.stream_name = "Tbox Cp",
		.cpu_dai_name = "tbox pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs Aux Cp Port",
		.stream_name = "Aux Cp",
		.cpu_dai_name = "aux pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs Tunner Cp Port",
		.stream_name = "Tunner Cp",
		.cpu_dai_name = "tunner pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs SXM Cp Port",
		.stream_name = "SXM Cp",
		.cpu_dai_name = "sxm pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Silabs Cluster Chime Playback Port",
		.stream_name = "Cluster Chime Pb",
		.cpu_dai_name = "Cluster Chime Pb pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_playback = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},

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
		/* SSP1 - SXM-In */
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
		/* SSP2 */
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
		/* SSP4 - Cluster */
		.name = "SSP4-Codec",
		.id = 4,
		.cpu_dai_name = "SSP4 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_capture = 1,
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP5 - Silabs */
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
		.be_hw_params_fixup = bxt_tdf8532_ssp2_fixup,
	},
};
#else
/* broxton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link broxton_tdf8532_dais[] = {
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
#endif
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
};
#endif
static int bxt_add_dai_link(struct snd_soc_card *card,
			struct snd_soc_dai_link *link)
{
	link->platform_name = "0000:00:0e.0";
	link->nonatomic = 1;
	return 0;
}

/* broxton audio machine driver for TDF8532 */
static struct snd_soc_card broxton_tdf8532 = {
	.name = "broxton_tdf8532",
	.dai_link = broxton_tdf8532_dais,
	.num_links = ARRAY_SIZE(broxton_tdf8532_dais),
	.controls = broxton_tdf8532_controls,
	.num_controls = ARRAY_SIZE(broxton_tdf8532_controls),
	.dapm_widgets = broxton_tdf8532_widgets,
	.num_dapm_widgets = ARRAY_SIZE(broxton_tdf8532_widgets),
	.dapm_routes = broxton_tdf8532_map,
	.num_dapm_routes = ARRAY_SIZE(broxton_tdf8532_map),
	.fully_routed = true,
	.add_dai_link = bxt_add_dai_link,
};

static int broxton_tdf8532_audio_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s registering %s\n", __func__, pdev->name);
	broxton_tdf8532.dev = &pdev->dev;
	return snd_soc_register_card(&broxton_tdf8532);
}

static int broxton_tdf8532_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&broxton_tdf8532);
	return 0;
}

static struct platform_driver broxton_tdf8532_audio = {
	.probe = broxton_tdf8532_audio_probe,
	.remove = broxton_tdf8532_audio_remove,
	.driver = {
		.name = "bxt_tdf8532",
		.pm = &snd_soc_pm_ops,
	},
};

module_platform_driver(broxton_tdf8532_audio)

/* Module information */
MODULE_DESCRIPTION("Intel SST Audio for Broxton GP MRB");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:gpmrb_machine");
