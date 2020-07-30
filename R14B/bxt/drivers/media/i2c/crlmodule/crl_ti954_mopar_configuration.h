/*
 * crl_ti954_mopar_configuration.h
 * Copyright (C) 2016 Harman International Ltd,
 *
 * Author: Sreeju Arumugan Selvaraj <sreeju.selvaraj@harman.com>
 *       : Kiruthika Varadarajan <kiruthika.varadarajan@harman.com>
 * Created on: 15-06-2018
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

#ifndef __CRLMODULE_TI954_MOPAR_CONFIGURATION_H__
#define __CRLMODULE_TI954_MOPAR_CONFIGURATION_H__

#include "crlmodule-sensor-ds.h"
#include "serdes-regs.h"


irqreturn_t crl_ds90ub954_threaded_irq_fn(int irq, void *sensor_struct);

static struct crl_register_write_rep ti954_mopar_onetime_init_regset[] = {
};

static struct crl_register_write_rep ti954_mopar_streamon_regs[] = {
};

static struct crl_register_write_rep ti954_mopar_streamoff_regs[] = {
};

static struct crl_register_read_rep ti954_mopar_status_regset[] = {
	{ 0x04, CRL_REG_LEN_08BIT, 0xff }, /* 954 general status */
};

const s64 ti954_mopar_op_sys_clock[] =  { 400000000 };

static struct crl_pll_configuration ti954_mopar_pll_configurations[] = {
        {
                .input_clk = 25000000,
		.op_sys_clk = 400000000, /* Deserializer is configured for 800Mbps */
                .pixel_rate_csi = 200000000, /* 800Mbps/4 */
                .pixel_rate_pa = 200000000,
                .bitsperpixel = 16,
                .comp_items = 0,
                .ctrl_data = 0,
                .pll_regs_items = 0,
                .pll_regs = NULL,
		.csi_lanes = 2,
         },
};

static struct crl_subdev_rect_rep ti954_mopar_1280x800_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 1280,
		.in_rect.height = 800,
		.out_rect.left = 0,
		.out_rect.top = 0,
		.out_rect.width = 1280,
		.out_rect.height = 800,
	 },

        {
                .subdev_type = CRL_SUBDEV_TYPE_BINNER,
                .in_rect.left = 0,
                .in_rect.top = 0,
                .in_rect.width = 1280,
                .in_rect.height = 800,
                .out_rect.left = 0,
                .out_rect.top = 0,
                .out_rect.width = 1280,
                .out_rect.height = 800,
        },
};

static struct crl_mode_rep ti954_mopar_modes[] = {
	{
		.sd_rects_items = ARRAY_SIZE(ti954_mopar_1280x800_rects),
		.sd_rects = ti954_mopar_1280x800_rects,
		.binn_hor = 1,
		.binn_vert = 1,
		.scale_m = 1,
		.width = 1280,
		.height = 800,
                .min_llp = 1905,
                .min_fll = 840,
		.comp_items = 0,
		.ctrl_data = 0,
	 },
};


static struct crl_sensor_subdev_config ti954_mopar_sensor_subdevs[] = {
	{
                .subdev_type = CRL_SUBDEV_TYPE_BINNER,
                .name = "mopar binner",
	},

	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "mopar pixel array",
	},
};

static struct crl_sensor_limits ti954_mopar_sensor_limits = {
	.x_addr_min = 0,
	.y_addr_min = 0,
	.x_addr_max = 1280,
	.y_addr_max = 800,
	.min_frame_length_lines = 280,
	.max_frame_length_lines = 840,
	.min_line_length_pixels = 440,
	.max_line_length_pixels = 1905,
        .scaler_m_min = 1,
        .scaler_m_max = 1,
        .scaler_n_min = 1,
        .scaler_n_max = 1,
        .min_even_inc = 1,
        .max_even_inc = 1,
        .min_odd_inc = 1,
        .max_odd_inc = 1,
};



static struct crl_csi_data_fmt ti954_mopar_crl_csi_data_fmt[] = {
        {
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
                .pixel_order = CRL_PIXEL_ORDER_GRBG,
                .bits_per_pixel = 16,
                .regs_items = 0,
                .regs = 0,
        },
};


static struct crl_v4l2_ctrl ti954_mopar_v4l2_ctrls[] = {

        {
                .sd_type = CRL_SUBDEV_TYPE_BINNER,
                .op_type = CRL_V4L2_CTRL_SET_OP,
                .context = SENSOR_IDLE,
                .ctrl_id = V4L2_CID_LINK_FREQ,
                .name = "V4L2_CID_LINK_FREQ",
                .type = CRL_V4L2_CTRL_TYPE_MENU_INT,
                .data.v4l2_int_menu.def = 0,
                .data.v4l2_int_menu.max = ARRAY_SIZE(ti954_mopar_pll_configurations) - 1,
                .data.v4l2_int_menu.menu = ti954_mopar_op_sys_clock,
                .flags = 0,
                .impact = CRL_IMPACTS_NO_IMPACT,
                .regs_items = 0,
                .regs = 0,
                .dep_items = 0,
                .dep_ctrls = 0,
        },
	{
                .sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
                .op_type = CRL_V4L2_CTRL_GET_OP,
                .context = SENSOR_POWERED_ON,
                .ctrl_id = V4L2_CID_PIXEL_RATE,
                .name = "V4L2_CID_PIXEL_RATE_PA",
                .type = CRL_V4L2_CTRL_TYPE_INTEGER,
                .data.std_data.min = 0,
                .data.std_data.max = 0,
                .data.std_data.step = 1,
                .data.std_data.def = 0,
                .flags = 0,
                .impact = CRL_IMPACTS_NO_IMPACT,
                .regs_items = 0,
                .regs = 0,
                .dep_items = 0,
                .dep_ctrls = 0,
        },
	{
                .sd_type = CRL_SUBDEV_TYPE_BINNER,
                .op_type = CRL_V4L2_CTRL_GET_OP,
                .context = SENSOR_POWERED_ON,
                .ctrl_id = V4L2_CID_PIXEL_RATE,
                .name = "V4L2_CID_PIXEL_RATE_CSI",
                .type = CRL_V4L2_CTRL_TYPE_INTEGER,
                .data.std_data.min = 0,
                .data.std_data.max = 0,
                .data.std_data.step = 1,
                .data.std_data.def = 0,
                .flags = 0,
                .impact = CRL_IMPACTS_NO_IMPACT,
                .regs_items = 0,
                .regs = 0,
                .dep_items = 0,
                .dep_ctrls = 0,
        },
        {
                .sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
                .op_type = CRL_V4L2_CTRL_SET_OP,
                .context = SENSOR_POWERED_ON,
                .ctrl_id = CRL_CID_SENSOR_MODE,
                .name = "CRL_CID_SENSOR_MODE",
                .type = CRL_V4L2_CTRL_TYPE_CUSTOM,
                .data.std_data.min = 0,
                .data.std_data.max = ARRAY_SIZE(ti954_mopar_modes) - 1,
                .data.std_data.step = 1,
                .data.std_data.def = 0,
                .flags = V4L2_CTRL_FLAG_UPDATE,
                .impact = CRL_IMPACTS_MODE_SELECTION,
                .ctrl = 0,
                .regs_items = 0,
                .regs = 0,
                .dep_items = 0,
                .dep_ctrls = 0,
                .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
        },
};


int ds90ub954_custom_init(struct i2c_client *);
int ds90ub954_cleanup(struct i2c_client *);
int ds90ub954_streamon_callback(struct i2c_client* client);
int ds90ub954_streamoff_callback(struct i2c_client* client);
int ds90ub954_suspend_callback(struct i2c_client* client);
int ds90ub954_resume_callback(struct i2c_client* client);

void ds90ub954_get_modes(struct i2c_client*, struct crl_mode_rep **modes, unsigned int *modes_items);
struct crl_sensor_limits * ds90ub954_get_limits(struct i2c_client*);

struct crl_sensor_configuration ti954_mopar_crl_configuration = {

        .sensor_init = ds90ub954_custom_init,  
	.sensor_cleanup = ds90ub954_cleanup,
	//.onetime_init_regs_items = ARRAY_SIZE(ti954_mopar_onetime_init_regset),
	//.onetime_init_regs = ti954_mopar_onetime_init_regset,
	.poweroff_regs_items = 0,
	.poweroff_regs = 0,
	.subdev_items = ARRAY_SIZE(ti954_mopar_sensor_subdevs),
	.subdevs = ti954_mopar_sensor_subdevs,

	.sensor_limits = &ti954_mopar_sensor_limits,

	.status_regs_items = ARRAY_SIZE(ti954_mopar_status_regset),
	.status_regs = ti954_mopar_status_regset,

	.pll_config_items = ARRAY_SIZE(ti954_mopar_pll_configurations),
	.pll_configs = ti954_mopar_pll_configurations,

	.modes_items = ARRAY_SIZE(ti954_mopar_modes),
	.modes = ti954_mopar_modes,

	.streamon_regs_items = ARRAY_SIZE(ti954_mopar_streamon_regs),
	.streamon_regs = ti954_mopar_streamon_regs,

	.streamoff_regs_items = ARRAY_SIZE(ti954_mopar_streamoff_regs),
	.streamoff_regs = ti954_mopar_streamoff_regs,

	.streamon_callback = ds90ub954_streamon_callback,
	.streamoff_callback = ds90ub954_streamoff_callback,

	.suspend_callback  = ds90ub954_suspend_callback,
	.resume_callback  = ds90ub954_resume_callback,

	.v4l2_ctrls_items = ARRAY_SIZE(ti954_mopar_v4l2_ctrls),
	.v4l2_ctrl_bank = ti954_mopar_v4l2_ctrls,

	.csi_fmts_items = ARRAY_SIZE(ti954_mopar_crl_csi_data_fmt),
	.csi_fmts = ti954_mopar_crl_csi_data_fmt,

	.addr_len = CRL_ADDR_8BIT,

        .get_modes = ds90ub954_get_modes,
        .get_limits = ds90ub954_get_limits,

	.irq_in_use = true,
	.crl_irq_fn = NULL,
	.crl_threaded_irq_fn = crl_ds90ub954_threaded_irq_fn,
};

#endif  /* __CRLMODULE_TI954_MOPAR_CONFIGURATION_H__ */
