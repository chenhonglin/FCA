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
#include <linux/i2c/gwm_display.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input/synaptics_dsx.h>
#include "camera_init.h"

#define DRVNAME			"apl-gwmv3-board"
#define GWM_SERIALIZER_ADDR         0x0C
#define GWM_DISPLAY_I2C_ADAPTER     6
#define GWM_TOUCH_INT_GPIO        (357 + 40)

static const struct pxa2xx_spi_chip chip_data = {
	.gpio_cs = -EINVAL,
	.dma_burst_size = 1,
};

static const struct spi_board_info apl_spi_slaves[] = {
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 2,
		.chip_select = 0,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
};

static struct synaptics_dsx_button_map _tag_cap_button_map = {
	.nbuttons = 0,
	.map      = NULL,
};

static struct synaptics_dsx_button_map _tag_vir_button_map = {
	.nbuttons = 0,
	.map      = NULL,
};

static struct gwm_display_platform_data gw_display_pdata = {
	.mxt_platform_data = {
		.input_name = "atmel_mxt_ts",
		.irqflags = IRQF_TRIGGER_LOW,
		.t19_num_keys = 0,
		.t15_num_keys = 0,
		.t19_keymap = NULL,
		.t15_keymap = NULL,
		.gpio_int = 397,
		.cfg_name = "gw_maxtouch.cfg",
		.regulator_dis = 1,
	},
	.synaptics_platform_data = {
		.irq_gpio = GWM_TOUCH_INT_GPIO,
		.irq_flags = IRQF_ONESHOT | IRQF_TRIGGER_LOW,
		.irq_on_state = 0,
		.power_gpio = -1,
		.power_delay_ms = 200,
		.power_on_state = 1,
		.reset_gpio = -1,
		.reset_delay_ms = 200,
		.reset_on_state = 0,
		.reset_active_ms = 20,
		.pwr_reg_name = NULL,
		.bus_reg_name = NULL,
		.max_y_for_2d = -1,
		.swap_axes = 0,
		.x_flip = 0,
		.y_flip	= 0,
		.ub_i2c_addr = -1,
		.cap_button_map = &_tag_cap_button_map,
		.vir_button_map = &_tag_vir_button_map,
		.pwr_reg_name = NULL,
		.panel_x = 1920,
		.panel_y = 720,
	},
};

static struct i2c_board_info gw_i2c_devs6[] __initdata = {
	{ I2C_BOARD_INFO("gw_display", GWM_SERIALIZER_ADDR),
		.platform_data = &gw_display_pdata },
};
static int apl_i2c_board_setup(void)
{
	int ret;
	ret = i2c_register_board_info(GWM_DISPLAY_I2C_ADAPTER, gw_i2c_devs6,
			ARRAY_SIZE(gw_i2c_devs6));
	if (ret)
		pr_warn(DRVNAME "failed to register i2c (ret: %d)\n", ret);
	else
		pr_debug(DRVNAME "successfully registered the i2c slaves...\n");

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
	if (ret)
		goto exit;
exit:
	return ret;

}
arch_initcall(apl_board_init);

MODULE_LICENSE("GPL v2");
