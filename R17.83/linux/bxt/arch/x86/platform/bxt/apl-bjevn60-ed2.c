/*
 * Copyright (c) 2016, Intel Corporation
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/i2c/r1_display.h>
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
#include <linux/delay.h>
#include <linux/input/synaptics_dsx.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>
#include <linux/cyttsp4_core.h>
#include <linux/cyttsp4_platform.h>
#endif

#include <linux/i2c/bjev_ivi_display.h>
#include <linux/i2c/bjev_vtp_display.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/ipc_i2c.h>


#include "camera_init.h"
#define DRVNAME			"apl-bjevn60-board"

#define TI954_PDB	(434+10)

/*947 VTP/TDS */
#define BJEV_VTP_SERIALIZER_ADDR      0x1A
#define BJEV_VTP_I2C_ADAPTER	      4
#define VTP_TOUCH_PDB_GPIO		     (434+23)
#define VTP_TOUCH_INT_GPIO_NORMAL    (267+27)
#define BJEV_VTP_DESERIALIZER_ADDR     0x2C


/*949 IVI/CDS */
#define BJEV_IVI_SERIALIZER_ADDR      0x0C
#define BJEV_IVI_I2C_ADAPTER	      6
#define IVI_TOUCH_PDB_GPIO	      (434+0)
#define IVI_TOUCH_INT_GPIO            (267+22)
#define BJEV_IVI_DESERIALIZER_ADDR     0x2C
/* CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE */
#ifdef  CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
#define BJEV_VTP_SYNAPTICS_ADDR        0x20
#define BJEV_IVI_SYNAPTICS_ADDR        0x20
#endif

/* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4 */
#ifdef  CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
#define CYTTSP6_I2C_TCH_ADR 0x24
#define CYTTSP6_LDR_TCH_ADR 0x24

#define CYTTSP6_2_I2C_TCH_ADR 0x24
#define CYTTSP6_2_LDR_TCH_ADR 0x24

/* VTP setting */
#define CY_VKEYS_X_VTP 1920
#define CY_VKEYS_Y_VTP 1080
#define CY_MAXX_VTP 1920
#define CY_MAXY_VTP 1080
#define CY_MINX_VTP 0
#define CY_MINY_VTP 0

#define CY_ABS_MIN_X_VTP CY_MINX_VTP
#define CY_ABS_MIN_Y_VTP CY_MINY_VTP
#define CY_ABS_MAX_X_VTP CY_MAXX_VTP
#define CY_ABS_MAX_Y_VTP CY_MAXY_VTP

/* IVI setting */
#define CY_VKEYS_X_IVI 3840
#define CY_VKEYS_Y_IVI 720
#define CY_MAXX_IVI 3840
#define CY_MAXY_IVI 720
#define CY_MINX_IVI 0
#define CY_MINY_IVI 0

#define CY_ABS_MIN_X_IVI CY_MINX_IVI
#define CY_ABS_MIN_Y_IVI CY_MINY_IVI
#define CY_ABS_MAX_X_IVI CY_MAXX_IVI
#define CY_ABS_MAX_Y_IVI CY_MAXY_IVI

#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255
#define CY_PROXIMITY_MIN_VAL    0
#define CY_PROXIMITY_MAX_VAL    1

#define CY_ABS_MIN_T 0
#define CY_ABS_MAX_T 15
#define CYTTSP6_I2C_MAX_XFER_LEN 256
#endif

#define	BJEV_VTP_DU_ADDR		0x01
#define	BJEV_IVI_DU_ADDR		0x01

#define FPGA_RESET_GPIO              (434+24)
#define FPGA_DONE_GPIO               (434+8)


static struct pxa2xx_spi_chip chip_data = {
	.gpio_cs = -EINVAL,
	.dma_burst_size = 1,
};

static struct spi_board_info apl_spi_slaves[] = {
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 1,
		.chip_select = 0,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 1,
		.chip_select = 1,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 2,
		.chip_select = 0,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 2,
		.chip_select = 1,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "dabplugin",
		.max_speed_hz = 2000000,
		.bus_num = 3,
		.chip_select = 0,
		.controller_data = &chip_data,
		.mode = SPI_MODE_1,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 3,
		.chip_select = 1,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 3,
		.chip_select = 2,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	}
};

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
static struct synaptics_dsx_button_map _tag_cap_button_map ={
		.nbuttons = 0,
		.map      = NULL,
};

static struct synaptics_dsx_button_map _tag_vir_button_map ={
		.nbuttons = 0,
		.map      = NULL,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4

static struct cyttsp4_core_platform_data _cyttsp6_ivi_core_platform_data = {
//	.serializer_i2c_addr   = BJEV_VTP_SERIALIZER_ADDR,
//	.deserializer_i2c_addr = BJEV_VTP_DESERIALIZER_ADDR,
	.irq_gpio = IVI_TOUCH_INT_GPIO,
	.rst_gpio = 0,
	.err_gpio = 0,
	.max_xfer_len = CYTTSP6_I2C_MAX_XFER_LEN,
	.xres = NULL,
	.init = cyttsp4_init,
	.power = cyttsp4_power,
	.detect = cyttsp4_detect,
	.irq_stat = cyttsp4_irq_stat,
	.error_stat = cyttsp4_error_stat,
	.sett = {
		NULL,   /* Reserved */
		NULL,   /* Command Registers */
		NULL,   /* Touch Report */
		NULL,   /* Cypress Data Record */
		NULL,   /* Test Record */
		NULL,   /* Panel Configuration Record */
		NULL,   /* &cyttsp6_sett_param_regs, */
		NULL,   /* &cyttsp6_sett_param_size, */
		NULL,   /* Reserved */
		NULL,   /* Reserved */
		NULL,   /* Operational Configuration Record */
		NULL, /* &cyttsp6_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp6_sett_mdata, *//* Manufacturing Data Record */
		NULL,   /* Config and Test Registers */
		NULL, /* button-to-keycode table */
	},
//        .flags = CY_CORE_FLAG_WAKE_ON_GESTURE,
//       .easy_wakeup_gesture = CY_CORE_EWG_TAP_TAP
//                | CY_CORE_EWG_TWO_FINGER_SLIDE,
};

static struct cyttsp4_core_platform_data _cyttsp6_vtp_core_platform_data = {
//	.serializer_i2c_addr   = BJEV_VTP_SERIALIZER_ADDR,
//	.deserializer_i2c_addr = BJEV_VTP_DESERIALIZER_ADDR,
	.irq_gpio = VTP_TOUCH_INT_GPIO_NORMAL,
	.rst_gpio = 0,
	.err_gpio = 0,
	.max_xfer_len = CYTTSP6_I2C_MAX_XFER_LEN,
	.xres = NULL,
	.init = cyttsp4_init,
	.power = cyttsp4_power,
	.detect = cyttsp4_detect,
	.irq_stat = cyttsp4_irq_stat,
	.error_stat = cyttsp4_error_stat,
	.sett = {
                NULL,   /* Reserved */
                NULL,   /* Command Registers */
                NULL,   /* Touch Report */
                NULL,   /* Cypress Data Record */
                NULL,   /* Test Record */
                NULL,   /* Panel Configuration Record */
                NULL,   /* &cyttsp6_sett_param_regs, */
                NULL,   /* &cyttsp6_sett_param_size, */
                NULL,   /* Reserved */
                NULL,   /* Reserved */
                NULL,   /* Operational Configuration Record */
                NULL, /* &cyttsp6_sett_ddata, *//* Design Data Record */
                NULL, /* &cyttsp6_sett_mdata, *//* Manufacturing Data Record */
                NULL,   /* Config and Test Registers */
                NULL, /* button-to-keycode table */
        },
//        .flags = CY_CORE_FLAG_WAKE_ON_GESTURE,
//       .easy_wakeup_gesture = CY_CORE_EWG_TAP_TAP
//                | CY_CORE_EWG_TWO_FINGER_SLIDE,
};

static const int16_t cyttsp4_vtp_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X_VTP, CY_ABS_MAX_X_VTP, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y_VTP, CY_ABS_MAX_Y_VTP, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -127, 127, 0, 0,
	ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
	ABS_DISTANCE, 0, 255, 0, 0,     /* Used with hover */
};

static const int16_t cyttsp4_ivi_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X_IVI, CY_ABS_MAX_X_IVI, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y_IVI, CY_ABS_MAX_Y_IVI, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -127, 127, 0, 0,
	ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
	ABS_DISTANCE, 0, 255, 0, 0,     /* Used with hover */
};


struct touch_framework cyttsp4_vtp_framework = {
	.abs = cyttsp4_vtp_abs,
	.size = ARRAY_SIZE(cyttsp4_vtp_abs),
	.enable_vkeys = 0,
};

struct touch_framework cyttsp4_ivi_framework = {
	.abs = cyttsp4_ivi_abs,
	.size = ARRAY_SIZE(cyttsp4_ivi_abs),
	.enable_vkeys = 0,
};


static struct cyttsp4_mt_platform_data _cyttsp6_mt_vtp_platform_data = {
	.frmwrk = &cyttsp4_vtp_framework,
	//.flags = CY_MT_FLAG_INV_Y,
	.inp_dev_name = CYTTSP4_MT_NAME_VTP,
	.vkeys_x = CY_VKEYS_X_VTP,
	.vkeys_y = CY_VKEYS_Y_VTP,
};

static struct cyttsp4_mt_platform_data _cyttsp4_mt_ivi_platform_data = {
	.frmwrk = &cyttsp4_ivi_framework,
	//.flags = CY_MT_FLAG_INV_Y,
	.inp_dev_name = CYTTSP4_MT_NAME_IVI,
	.vkeys_x = CY_VKEYS_X_IVI,
	.vkeys_y = CY_VKEYS_Y_IVI,
};

static struct cyttsp4_btn_platform_data _cyttsp4_btn_platform_data = {
	.inp_dev_name = CYTTSP4_BTN_NAME,
};

static const int16_t cyttsp5_prox_abs[] = {
	ABS_DISTANCE, CY_PROXIMITY_MIN_VAL, CY_PROXIMITY_MAX_VAL, 0, 0,
};

struct touch_framework cyttsp5_prox_framework = {
	.abs = cyttsp5_prox_abs,
	.size = ARRAY_SIZE(cyttsp5_prox_abs),
};

static struct cyttsp4_proximity_platform_data
	_cyttsp5_proximity_platform_data = {
	.frmwrk = &cyttsp5_prox_framework,
	.inp_dev_name = CYTTSP5_PROXIMITY_NAME,
};

static struct cyttsp4_adapter_platform_data _cyttsp6_vtp_adapter_data = {
	.bl_i2c_client_addr = 0U,
};

static struct cyttsp4_adapter_platform_data _cyttsp6_ivi_adapter_data = {
	.bl_i2c_client_addr = 0U,
};
#endif

static struct bjev_vtp_display_platform_data bjev_vtp_pdata = {

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
	.synaptics_platform_data = {
		.serializer_i2c_addr   = BJEV_VTP_SERIALIZER_ADDR,
		.deserializer_i2c_addr = BJEV_VTP_DESERIALIZER_ADDR,
		.irq_gpio              = VTP_TOUCH_INT_GPIO_NORMAL,
		.irq_flags             = IRQF_ONESHOT | IRQF_TRIGGER_LOW,
		.irq_on_state          = 0,
		.power_gpio            = -1,
		.power_delay_ms        = 200,
		.power_on_state        = 1,
		.reset_gpio            = -1,
		.reset_delay_ms        = 200,
		.reset_on_state        = 0,
		.reset_active_ms       = 20,
		.pwr_reg_name          = NULL,
		.bus_reg_name          = NULL,
		.max_y_for_2d          = -1,
		.swap_axes             = 0,
		.x_flip                = 0,
		.y_flip                = 0,
		.ub_i2c_addr           = -1,
		.cap_button_map        = &_tag_cap_button_map,
		.vir_button_map        = &_tag_vir_button_map,
		.pwr_reg_name          = NULL,
		.panel_x               = 1920,
		.panel_y               = 1080,
	},
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
	.cypress_platform_data  = {
		.core_pdata = &_cyttsp6_vtp_core_platform_data,
		.adap_pdata = &_cyttsp6_vtp_adapter_data,
		.mt_pdata = &_cyttsp6_mt_vtp_platform_data,
		.btn_pdata = &_cyttsp4_btn_platform_data,
		.prox_pdata = &_cyttsp5_proximity_platform_data,
		.loader_pdata = &_cyttsp6_loader_platform_data,

	},
#endif
};

static struct bjev_ivi_display_platform_data bjev_ivi_pdata = {

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
	.synaptics_platform_data = {
		.serializer_i2c_addr   = BJEV_IVI_SERIALIZER_ADDR,
		.deserializer_i2c_addr = BJEV_IVI_DESERIALIZER_ADDR,
		.irq_gpio              = IVI_TOUCH_INT_GPIO,
		.irq_flags             = IRQF_ONESHOT | IRQF_TRIGGER_LOW,
		.irq_on_state          = 0,
		.power_gpio            = -1,
		.power_delay_ms        = 200,
		.power_on_state        = 1,
		.reset_gpio            = -1,
		.reset_delay_ms        = 200,
		.reset_on_state        = 0,
		.reset_active_ms       = 20,
		.pwr_reg_name          = NULL,
		.bus_reg_name          = NULL,
		.max_y_for_2d          = -1,
		.swap_axes             = 0,
		.x_flip                = 0,
		.y_flip                = 0,
		.ub_i2c_addr           = -1,
		.cap_button_map        = &_tag_cap_button_map,
		.vir_button_map        = &_tag_vir_button_map,
		.pwr_reg_name          = NULL,
		.panel_x               = 1920,
		.panel_y               = 1080,
	},
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
	.cypress_platform_data  = {
		.core_pdata = &_cyttsp6_ivi_core_platform_data,
		.adap_pdata = &_cyttsp6_ivi_adapter_data,
		.mt_pdata = &_cyttsp4_mt_ivi_platform_data,
		.btn_pdata = &_cyttsp4_btn_platform_data,
		.prox_pdata = &_cyttsp5_proximity_platform_data,
		.loader_pdata = &_cyttsp6_loader_platform_data,
	},
#endif
};

static struct i2c_board_info bjev_i2c_devs4[] __initdata = {
	{ I2C_BOARD_INFO("bjev_vtp_display", BJEV_VTP_SERIALIZER_ADDR),
	.platform_data = &bjev_vtp_pdata },
	{ I2C_BOARD_INFO("ipc_i2c_vtp", BJEV_VTP_DU_ADDR),
	},
};

static struct i2c_board_info bjev_i2c_devs6[] __initdata = {
	{ I2C_BOARD_INFO("bjev_ivi_display", BJEV_IVI_SERIALIZER_ADDR),
	.platform_data = &bjev_ivi_pdata},
	{ I2C_BOARD_INFO("ipc_i2c", BJEV_IVI_DU_ADDR),
	},
};

static int apl_i2c_board_setup(void)
{
	int ret;
	ret = i2c_register_board_info(BJEV_VTP_I2C_ADAPTER, bjev_i2c_devs4,
						ARRAY_SIZE(bjev_i2c_devs4));
	if (ret)
		pr_warn(DRVNAME "failed to register VTP serializer & ipc-i2c (ret: %d)\n",ret);
	else
		pr_debug(DRVNAME "successfully registered VTP serializer & ipc-i2c\n");

	ret = i2c_register_board_info(BJEV_IVI_I2C_ADAPTER, bjev_i2c_devs6,
						ARRAY_SIZE(bjev_i2c_devs6));
	if (ret)
		pr_warn(DRVNAME "failed to register IVI serializer & ipc-i2c (ret: %d)\n",ret);
	else
		pr_debug(DRVNAME "successfully registered IVI serializer & ipc-i2c\n");

	return ret;

}

static int apl_spi_board_setup(void)
{
	int ret = -1;

	/* Register the SPI devices */
	ret = spi_register_board_info(apl_spi_slaves,
				ARRAY_SIZE(apl_spi_slaves));
	if (ret)
		pr_warn(DRVNAME ": failed to register the SPI slaves...\n");
	else
		pr_debug(DRVNAME ": successfully registered the SPI slaves...\n");
	return ret;
}

static int __init apl_board_init(void)
{
	int ret;

	camera_init();
	pr_info(DRVNAME ": >>> oem_camera_type = 0x%x\n", oem_camera_type);

	pr_debug(DRVNAME ": registering APL SPI devices...\n");
	ret = apl_spi_board_setup();
	if (ret)
		goto exit;
	pr_debug(DRVNAME ": registering APL I2C devices...\n");
	ret = apl_i2c_board_setup();

exit:
	return ret;
}
arch_initcall(apl_board_init);


static irqreturn_t fpga_done_irq(int irq, void *data)
{
	gpio_direction_output(FPGA_RESET_GPIO, 1);
	/*printk(KERN_ERR"%s: hubert--log\n",__func__);*/
	return IRQ_HANDLED;
}

static int fpga_init(void)
{
	int retval=0;

	if(gpio_get_value(FPGA_DONE_GPIO)) {
		gpio_direction_output(FPGA_RESET_GPIO, 1);
	} else {
		retval = request_threaded_irq(gpio_to_irq(FPGA_DONE_GPIO),  NULL,
			fpga_done_irq, (IRQF_ONESHOT | IRQF_TRIGGER_RISING),	"FPGA_DONE_IRQ", NULL);
		if (retval < 0)
			printk(KERN_ERR"%s: Failed to create irq thread\n",__func__);
	}
	return retval;
}

static void vtp_touch_setup(void)
{
	gpio_request(VTP_TOUCH_PDB_GPIO, "VTP_TOUCH_PDB");
	/*gpio_direction_output(VTP_TOUCH_PDB_GPIO, 0);
	mdelay(30);*/
	gpio_direction_output(VTP_TOUCH_PDB_GPIO, 1);
	gpio_export(VTP_TOUCH_PDB_GPIO, 1);

	gpio_request(IVI_TOUCH_PDB_GPIO, "IVI_TOUCH_PDB");
	gpio_direction_output(IVI_TOUCH_PDB_GPIO, 1);
	gpio_export(IVI_TOUCH_PDB_GPIO, 1);

	gpio_request(FPGA_RESET_GPIO, "FPGA_RESET_GPIO");
	/*gpio_direction_output(FPGA_RESET_GPIO, 0);
	mdelay(30);*/
	gpio_direction_output(FPGA_RESET_GPIO, 1);
	gpio_export(FPGA_RESET_GPIO, 1);
}

static void ti954_setup(void)
{
	gpio_request(TI954_PDB, "ti954_pdb");
	gpio_direction_output(TI954_PDB, 1);
	gpio_export(TI954_PDB, 1);
}

static int __init bjevn60_gpio_init(void)
{
	ti954_setup();
	vtp_touch_setup();
	fpga_init();

	return 0;
}
subsys_initcall_sync(bjevn60_gpio_init);

MODULE_LICENSE("GPL v2");
