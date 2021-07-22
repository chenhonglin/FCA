/*
 * Copyright (c) 2016--2017 Intel Corporation.
 *
 * Author: Jouni Ukkonen <jouni.ukkonen@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#include <media/intel-ipu4-isys.h>
#include "../../../../include/media/crlmodule.h"
#include "../../pci/intel-ipu4/intel-ipu4.h"
#if IS_ENABLED(CONFIG_VIDEO_MAX9288)
#include "../../../../include/media/max9288.h"
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9286)
#include "../../../../include/media/max9286.h"
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9296)
#include "../../../../include/media/max9296.h"
#endif

#ifdef CONFIG_INTEL_IPU4_TI953_SENSOR
#include "../../../../include/media/ti954.h"
#endif

#define ADV7481_HDMI_LANES      4
#define ADV7481_HDMI_I2C_ADDRESS 0xe0

#define ADV7481_LANES		1
/*
 * below i2c address is dummy one, to be able to register single
 * ADV7481 chip as two sensors
 */
#define ADV7481_I2C_ADDRESS	0xe1


#define GPIO_BASE		434

#ifdef CONFIG_INTEL_IPU4_ADV7281
#define ADV7281_IRQ_PIN                 434 /* GPIO[0], base 434 + 0 */
#define ADV7281_CRL_LANES		1
#define ADV7281_CRL_I2C_ADDRESS 	0x42
#define ADV7281_CRL_I2C_ADAPTER 	6
#define ADV7281_CSI_PORT_ID             0
#endif

#ifdef CONFIG_INTEL_IPU4_DS90UB954_MOPAR
#define DS90UB954_IRQ_PIN               455 /* GPIO[21], base 434 + 21 */
#define DS90UB954_LANES                 2
#define DS90UB954_I2C_ADDRESS           0x30
#define CAM_LVDS_RST_GPIO               10
#define CAM_CHNL_SELECT_GPIO            18

#ifdef CONFIG_DV2_CAMERA
#define DS90UB954_CSI_PORT_ID           4
#endif

#ifdef CONFIG_DV1_CAMERA
#define DS90UB954_CSI_PORT_ID           2
#endif

#define DS90UB954_I2C_ADAPTER           6
#endif

#ifdef CONFIG_INTEL_IPU4_DS90UB954_MOPAR
/* The following ds90ub964 pdata for Deserializer */
static struct crlmodule_platform_data ds90ub954_pdata = {
	/* Since power on pin is pulled to 1.8v, gpio not required
	for power on / shut down */
	.lanes = DS90UB954_LANES,
	.ext_clk = 25000000,
	.op_sys_clock = (uint64_t []){ 400000000 },
	.module_name = "DS90UB954",
	.crl_irq_pin = DS90UB954_IRQ_PIN, 
	.irq_pin_flags = (IRQF_TRIGGER_FALLING | IRQF_ONESHOT), /* Active low with ext pullup */
	.irq_pin_name = "DS90UB954_IRQ",
};

static struct intel_ipu4_isys_csi2_config ds90ub954_csi2_cfg = {
       .nlanes = DS90UB954_LANES,
       .port = DS90UB954_CSI_PORT_ID,
};

static struct intel_ipu4_isys_subdev_info ds90ub954_crl_sd = {
       .csi2 = &ds90ub954_csi2_cfg,
       .i2c = {
               .board_info = {
                         I2C_BOARD_INFO(CRLMODULE_NAME, DS90UB954_I2C_ADDRESS),
                         .platform_data = &ds90ub954_pdata,
               },
               .i2c_adapter_id = DS90UB954_I2C_ADAPTER,
       }
};
#endif

#define BJEV_FACE_DETECT_NO_CSI_LANES		4
#define BJEV_FACE_DETECT_BXT_CSI_PORT		4

#ifndef CONFIG_BJEVN60_VIRTUAL
#define BJEV_FACE_DETECT_I2C_ADAPTER		0x3
#else
#define BJEV_FACE_DETECT_I2C_ADAPTER		0x2
#endif

#define BJEV_FACE_DETECT_I2C_DEVICE_ADDRESS	0x30

static struct crlmodule_platform_data bjev_face_detect_pdata = {
	.lanes = BJEV_FACE_DETECT_NO_CSI_LANES,
	.ext_clk = 25000000,
	.op_sys_clock = (uint64_t []){ 400000000 },
	.module_name = "BJEV_FACE_DETECT"
};

static struct intel_ipu4_isys_csi2_config bjev_face_detect_csi2_cfg = {
	.nlanes = BJEV_FACE_DETECT_NO_CSI_LANES,
	.port = BJEV_FACE_DETECT_BXT_CSI_PORT,
};

static struct intel_ipu4_isys_subdev_info bjev_face_detect_crl_sd = {
	.csi2 = &bjev_face_detect_csi2_cfg,
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(CRLMODULE_NAME,
				BJEV_FACE_DETECT_I2C_DEVICE_ADDRESS),
			.platform_data = &bjev_face_detect_pdata,
		},
		.i2c_adapter_id = BJEV_FACE_DETECT_I2C_ADAPTER,
	}
};


/* ti ds90ub953 register configurations*/
#ifdef CONFIG_INTEL_IPU4_TI953_SENSOR
#define TI953_SENSOR_LANES              4
#define TI953_SENSOR_I2C_ADDRESS        0x10
#define TI953_SENSOR_A_I2C_ADDRESS      (TI953_SENSOR_I2C_ADDRESS + 1)
#define TI953_SENSOR_B_I2C_ADDRESS      (TI953_SENSOR_I2C_ADDRESS + 2)

#define DS_TI954_CSI_PORT_ID		4
#define DS_TI954_LANES		        4

#ifndef CONFIG_BJEVN60_VIRTUAL
#define DS_TI954_I2C_ADAPTER_B	    3
#else
#define DS_TI954_I2C_ADAPTER_B	    2
#endif

#define DS_TI954_I2C_ADDRESS		0x30


static struct crlmodule_platform_data ti953_sensor_pdata = {
        .lanes = TI953_SENSOR_LANES,
        .ext_clk = 25000000,
        .op_sys_clock = (uint64_t []){ 400000000 },
        .module_name = "TI953_SENSOR",
};

static struct intel_ipu4_isys_csi2_config ti954_b_csi2_cfg = {
	.nlanes = DS_TI954_LANES,
	.port = DS_TI954_CSI_PORT_ID,
};

struct ti954_subdev_i2c_info ti954_b_subdevs[] = {
        {
                .board_info = {
                        .type = CRLMODULE_NAME,
                        .addr = TI953_SENSOR_A_I2C_ADDRESS,
                        .platform_data = &ti953_sensor_pdata,
                },
                .i2c_adapter_id = DS_TI954_I2C_ADAPTER_B,
        },
        {
                .board_info = {
                        .type = CRLMODULE_NAME,
                        .addr = TI953_SENSOR_B_I2C_ADDRESS,
                        .platform_data = &ti953_sensor_pdata,
                },
                .i2c_adapter_id = DS_TI954_I2C_ADAPTER_B,
        },
};

static struct ti954_pdata ti954_b_pdata = {
	.subdev_info = ti954_b_subdevs,
	.subdev_num = ARRAY_SIZE(ti954_b_subdevs),
};

static struct intel_ipu4_isys_subdev_info ti954_b_sd = {
	.csi2 = &ti954_b_csi2_cfg,
	.i2c = {
			.board_info = {

			.type = "ti-ds90ub954",
			.addr = DS_TI954_I2C_ADDRESS,
			.platform_data = &ti954_b_pdata,
			},
		.i2c_adapter_id = DS_TI954_I2C_ADAPTER_B,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_ADV7281
static struct crlmodule_platform_data adv7281_cvbs_pdata = {
        .ext_clk = 28636360,
#ifndef CONFIG_FCA_MULTI_DISP_TOUCH_ENABLE
        .xshutdown = GPIO_BASE + 5, /* pwrdwn pin, active low */
#endif
        .lanes = ADV7281_CRL_LANES,
        .module_name = "ADV7281 CVBS",
	.crl_irq_pin = ADV7281_IRQ_PIN, 
	.irq_pin_flags = (IRQF_TRIGGER_FALLING | IRQF_ONESHOT), /* Active low with ext pullup */
	.irq_pin_name = "ADV7281_IRQ",
};

static struct intel_ipu4_isys_csi2_config adv7281_cvbs_csi2_cfg = {
        .nlanes = ADV7281_CRL_LANES,
        .port = ADV7281_CSI_PORT_ID,
};

static struct intel_ipu4_isys_subdev_info adv7281_cvbs_crl_sd = {
        .csi2 = &adv7281_cvbs_csi2_cfg,
        .i2c = {
               .board_info = {
                         I2C_BOARD_INFO(CRLMODULE_NAME, ADV7281_CRL_I2C_ADDRESS),
                         .platform_data = &adv7281_cvbs_pdata,
               },
               .i2c_adapter_id = ADV7281_CRL_I2C_ADAPTER,
        }
};
#endif

/* AR0143 register configurations for MAXIM */
#ifdef CONFIG_INTEL_IPU4_AR0143_AVM
#define AR0143_AVM_LANES             4
#define AR0143_AVM_I2C_ADDRESS       0x30 //0x5d
#define AR0143A_AVM_I2C_ADDRESS      (AR0143_AVM_I2C_ADDRESS + 1)
#define AR0143B_AVM_I2C_ADDRESS      (AR0143_AVM_I2C_ADDRESS + 2)
#define AR0143C_AVM_I2C_ADDRESS      (AR0143_AVM_I2C_ADDRESS + 3)
#define AR0143D_AVM_I2C_ADDRESS      (AR0143_AVM_I2C_ADDRESS + 4)

static struct crlmodule_platform_data AR0143_AVM_pdata = {
	.lanes = AR0143_AVM_LANES,
	.ext_clk = 25000000,
	.op_sys_clock = (uint64_t []){ 288000000 },
	.module_name = "AR0143_AVM",
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_MAX9286)
#define DS_MAX9286_RESET_GPIO           (GPIO_BASE + 10)
#define DS_MAX9286_CSI_PORT_ID          0
#define DS_MAX9286_LANES		4
#define DS_MAX9286_I2C_ADAPTER_B        6
#define DS_MAX9286_I2C_ADDRESS          0x48

static struct intel_ipu4_isys_csi2_config max9286_b_csi2_cfg = {
	.nlanes = DS_MAX9286_LANES,
	.port = DS_MAX9286_CSI_PORT_ID,
};

struct max9286_subdev_i2c_info max9286_b_subdevs[] = {
#ifdef CONFIG_INTEL_IPU4_AR0143_AVM
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = AR0143A_AVM_I2C_ADDRESS,
			.platform_data = &AR0143_AVM_pdata,
        },
		.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER_B,
    },
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = AR0143B_AVM_I2C_ADDRESS,
			.platform_data = &AR0143_AVM_pdata,
		},
		.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER_B,
    },
    {
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = AR0143C_AVM_I2C_ADDRESS,
			.platform_data = &AR0143_AVM_pdata,
		},
		.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER_B,
    },
    {
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = AR0143D_AVM_I2C_ADDRESS,
			.platform_data = &AR0143_AVM_pdata,
		},
		.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER_B,
    },
#endif
};

static struct max9286_pdata max9286_b_pdata = {
	.subdev_info = max9286_b_subdevs,
	.subdev_num = ARRAY_SIZE(max9286_b_subdevs),
	.reset_gpio = DS_MAX9286_RESET_GPIO, /* GPIO_10 */
};

static struct intel_ipu4_isys_subdev_info max9286_b_sd = {
	.csi2 = &max9286_b_csi2_cfg,
	.i2c = {
		.board_info = {
			.type = "max9286",
			.addr = DS_MAX9286_I2C_ADDRESS,
			.platform_data = &max9286_b_pdata,
		},
		.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER_B,
	}
};
#endif

static struct crlmodule_platform_data adv7481_cvbs_pdata = {
	.ext_clk = 286363636,
	.xshutdown = GPIO_BASE + 64, /*dummy for now*/
	.lanes = ADV7481_LANES,
	.module_name = "ADV7481 CVBS"
};

static struct intel_ipu4_isys_csi2_config adv7481_cvbs_csi2_cfg = {
	.nlanes = ADV7481_LANES,
	.port = 4,
};

static struct intel_ipu4_isys_subdev_info adv7481_cvbs_crl_sd = {
	.csi2 = &adv7481_cvbs_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481_I2C_ADDRESS,
			 .platform_data = &adv7481_cvbs_pdata,
		},
		.i2c_adapter_id = 0,
	}
};

static struct crlmodule_platform_data adv7481_hdmi_pdata = {
	/* FIXME: may need to revisit */
	.ext_clk = 286363636,
	.xshutdown = GPIO_BASE + 30,
	.lanes = ADV7481_HDMI_LANES,
	.module_name = "ADV7481 HDMI",
	.crl_irq_pin = GPIO_BASE + 22,
	.irq_pin_flags = (IRQF_TRIGGER_RISING | IRQF_ONESHOT),
	.irq_pin_name = "ADV7481_HDMI_IRQ",
};

static struct intel_ipu4_isys_csi2_config adv7481_hdmi_csi2_cfg = {
	.nlanes = ADV7481_HDMI_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info adv7481_hdmi_crl_sd = {
	.csi2 = &adv7481_hdmi_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481_HDMI_I2C_ADDRESS,
			 .platform_data = &adv7481_hdmi_pdata,
		},
		.i2c_adapter_id = 0,
	}
};

#ifdef CONFIG_INTEL_IPU4_AR0143_RVC
#define AR0143_RVC_LANES             1
#define AR0143_RVC_I2C_ADDRESS      (ADDR_AR0143_RVC_BASE + 1)

static struct crlmodule_platform_data ar0143_rvc_pdata = {
	.lanes = AR0143_RVC_LANES,
	.ext_clk = 25000000,
	.op_sys_clock = (uint64_t []){ 288000000 },
	.module_name = "AR0143_RVC",
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_MAX9288)
#define DS_MAX9288_RESET_GPIO		 (GPIO_BASE + 10)
#define DS_MAX9288_CSI_PORT_ID           4
#define DS_MAX9288_LANES                       1
#define DS_MAX9288_I2C_ADAPTER_B     6
#define DS_MAX9288_I2C_ADDRESS          0x68

struct max9288_subdev_i2c_info max9288_b_subdevs[] = {
#ifdef CONFIG_INTEL_IPU4_AR0143_RVC
{
	.board_info = {
		.type = CRLMODULE_NAME,
		.addr = AR0143_RVC_I2C_ADDRESS,
		.platform_data = &ar0143_rvc_pdata,
	},
	.i2c_adapter_id = DS_MAX9288_I2C_ADAPTER_B,
},
#endif
};

static struct max9288_pdata max9288_b_pdata = {
	.subdev_info = max9288_b_subdevs,
	.subdev_num = ARRAY_SIZE(max9288_b_subdevs),
	.reset_gpio = DS_MAX9288_RESET_GPIO, /* GPIO_10 */
};


static struct intel_ipu4_isys_csi2_config max9288_b_csi2_cfg = {
	.nlanes = DS_MAX9288_LANES,
	.port = DS_MAX9288_CSI_PORT_ID,
};

static struct intel_ipu4_isys_subdev_info max9288_b_sd = {
	.csi2 = &max9288_b_csi2_cfg,
	.i2c = {
		.board_info = {
		.type = "max9288",
		.addr = DS_MAX9288_I2C_ADDRESS,
		.platform_data = &max9288_b_pdata,
		},
		.i2c_adapter_id = DS_MAX9288_I2C_ADAPTER_B,
	},
};
#endif

#ifdef CONFIG_INTEL_IPU4_APTINA_RVC
#define APTINA_RVC_LANES             4
#define APTINA_RVC_I2C_ADDRESS       0x30
#define APTINA1_RVC_I2C_ADDRESS      (APTINA_RVC_I2C_ADDRESS + 1)
#define APTINA2_RVC_I2C_ADDRESS      (APTINA_RVC_I2C_ADDRESS + 2)

static struct crlmodule_platform_data aptina_rvc_pdata = {
	.lanes = APTINA_RVC_LANES,
	.ext_clk = 25000000,
	.op_sys_clock = (uint64_t []){ 300000000 },
	.module_name = "APTINA_RVC",
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_MAX9296)
#define DS_MAX9296_RESET_GPIO            (GPIO_BASE + 36)
#define DS_MAX9296_CSI_PORT_ID		0
#define DS_MAX9296_LANES		4
#define DS_MAX9296_I2C_ADAPTER_B	2
#define DS_MAX9296_I2C_ADDRESS		0x48

static struct intel_ipu4_isys_csi2_config max9296_b_csi2_cfg = {
	.nlanes = DS_MAX9296_LANES,
	.port = DS_MAX9296_CSI_PORT_ID,
};

struct max9296_subdev_i2c_info max9296_b_subdevs[] = {
#ifdef CONFIG_INTEL_IPU4_APTINA_RVC
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = APTINA1_RVC_I2C_ADDRESS,
			.platform_data = &aptina_rvc_pdata,
		},
		.i2c_adapter_id = DS_MAX9296_I2C_ADAPTER_B,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = APTINA2_RVC_I2C_ADDRESS,
			.platform_data = &aptina_rvc_pdata,
		},
		.i2c_adapter_id = DS_MAX9296_I2C_ADAPTER_B,
	},
#endif
};

static struct max9296_pdata max9296_b_pdata = {
	.subdev_info = max9296_b_subdevs,
	.subdev_num = ARRAY_SIZE(max9296_b_subdevs),
	.reset_gpio = DS_MAX9296_RESET_GPIO, /* GPIO_36 */
};

static struct intel_ipu4_isys_subdev_info max9296_b_sd = {
	.csi2 = &max9296_b_csi2_cfg,
	.i2c = {
		.board_info = {
			.type = "max9296",
			.addr = DS_MAX9296_I2C_ADDRESS,
			.platform_data = &max9296_b_pdata,
		},
		.i2c_adapter_id = DS_MAX9296_I2C_ADAPTER_B,
	}
};
#endif


/*
 * Map buttress output sensor clocks to sensors -
 * this should be coming from ACPI, in Gordon Peak
 * ADV7481 have its own oscillator, no buttres clock
 * needed.
 */
struct intel_ipu4_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};


#if defined (CONFIG_DV2_CAMERA)

static struct intel_ipu4_isys_subdev_pdata pdata = {
	.subdevs = (struct intel_ipu4_isys_subdev_info *[]) {
#ifdef CONFIG_INTEL_IPU4_ADV7481
		&adv7481_hdmi_crl_sd,
		&adv7481_cvbs_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_ADV7281
		&adv7281_cvbs_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_DS90UB954_MOPAR
                &ds90ub954_crl_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9288)
		&max9288_b_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9286)
		&max9286_b_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_BJEV_FD_CAM
		&bjev_face_detect_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_TI953_SENSOR
		&ti954_b_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9296)
		&max9296_b_sd,
#endif
		NULL,
	},
	.clk_map = clk_mapping,
};

static void ipu4_quirk(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = &pdata;
}

#elif defined (CONFIG_DV1_CAMERA)

static struct intel_ipu4_isys_subdev_pdata pdata_analog = {
        .subdevs = (struct intel_ipu4_isys_subdev_info *[]) {
#ifdef CONFIG_INTEL_IPU4_ADV7481
                &adv7481_hdmi_crl_sd,
                &adv7481_cvbs_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_ADV7281
                &adv7281_cvbs_crl_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9288)
                &max9288_b_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9286)
                &max9286_b_sd,
#endif
                NULL,
        },
        .clk_map = clk_mapping,
};

static struct intel_ipu4_isys_subdev_pdata pdata_digital = {
        .subdevs = (struct intel_ipu4_isys_subdev_info *[]) {
#ifdef CONFIG_INTEL_IPU4_DS90UB954_MOPAR
                &ds90ub954_crl_sd,
#endif
                NULL,
        },
        .clk_map = clk_mapping,
};

static void ipu4_quirk(struct pci_dev *pci_dev)
{
#ifdef CONFIG_INTEL_IPU4_DS90UB954_MOPAR
	extern unsigned char oem_camera_type; // from platform data
	if ( oem_camera_type != 0 ) {
		pci_dev->dev.platform_data = &pdata_digital;
		printk(KERN_INFO "LINE:[%d] Func:[%s] oem_camera_type = 0x%x \n",__LINE__,__func__,oem_camera_type);
	}
	else
#endif
	{
		printk(KERN_INFO "LINE:[%d] Func:[%s] oem_camera_type = 0x%x \n",__LINE__,__func__,oem_camera_type);
		pci_dev->dev.platform_data = &pdata_analog;
	}
}

#else

static struct intel_ipu4_isys_subdev_pdata pdata_analog = {
        .subdevs = (struct intel_ipu4_isys_subdev_info *[]) {
#ifdef CONFIG_INTEL_IPU4_ADV7481
                &adv7481_hdmi_crl_sd,
                &adv7481_cvbs_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_ADV7281
                &adv7281_cvbs_crl_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9288)
                &max9288_b_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9286)
                &max9286_b_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_BJEV_FD_CAM
                &bjev_face_detect_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_TI953_SENSOR
		&ti954_b_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9296)
                &max9296_b_sd,
#endif
                NULL,
        },
        .clk_map = clk_mapping,
};

static void ipu4_quirk(struct pci_dev *pci_dev)
{
#ifdef CONFIG_INTEL_IPU4_DS90UB954_MOPAR
	pci_dev->dev.platform_data = &pdata_digital;
#endif
	pci_dev->dev.platform_data = &pdata_analog;
}
#endif

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, INTEL_IPU4_HW_BXT_P,
			ipu4_quirk);
