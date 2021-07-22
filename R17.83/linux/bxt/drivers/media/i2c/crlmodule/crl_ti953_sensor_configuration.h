/*
 * crl_ti953_sensor_configuration.h
 * Copyright (C) 2016 Harman International Ltd,
 *
 * Author: Zhu Weitao <weitao.zhu@harman.com>
 * Created on: 22-04-2019
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

#ifndef __CRLMODULE_TI953_SENSOR_CONFIGURATION_H__
#define __CRLMODULE_TI953_SENSOR_CONFIGURATION_H__

#include "crlmodule-sensor-ds.h"
#include "serdes-regs.h"

#define DELAY_10MS 10
#define DELAY_100MS 100

#define TI953_SENSOR_CSI_DATA_LANES		       4


struct crl_ctrl_data_pair ctrl_ti953_sensor_data_lanes[] = {
	{
		.ctrl_id = V4L2_CID_MIPI_LANES,
		.data = TI953_SENSOR_CSI_DATA_LANES,
	},
};

static struct crl_register_write_rep ti953_sensor_onetime_init_regset[] = {
	{0x00, CRL_REG_LEN_DELAY, SER_DES_INIT_DELAY, 0x00},
};



static struct crl_register_write_rep ti953_sensor_streamon_regs[] = {
	{0x02, CRL_REG_LEN_08BIT, 0x72, 0x00},
	{0x00, CRL_REG_LEN_DELAY, SER_DES_INIT_DELAY, 0x00},
};

static struct crl_register_write_rep ti953_sensor_streamoff_regs[] = {
	/* dummy */
	{0x00, CRL_REG_LEN_DELAY, SER_DES_INIT_DELAY, 0x00},
};

const s64 ti953_sensor_op_sys_clock[] =  {400000000};

static struct crl_pll_configuration ti953_sensor_pll_configurations[] = {
        {
                .input_clk = 25000000,
				.op_sys_clk = 400000000, /* Deserializer is configured for 800Mbps */
                .pixel_rate_csi = 200000000,
                .pixel_rate_pa = 200000000,
                .bitsperpixel = 16,
                .comp_items = 0,
                .ctrl_data = 0,
                .pll_regs_items = 0,
                .pll_regs = NULL,
				.csi_lanes = TI953_SENSOR_CSI_DATA_LANES,
         },
};

static struct crl_subdev_rect_rep ti953_sensor_1080p_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 1920,
		.in_rect.height = 1080,
		.out_rect.left = 0,
		.out_rect.top = 0,
		.out_rect.width = 1920,
		.out_rect.height = 1080,
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_BINNER,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 1920,
		.in_rect.height = 1080,
		.out_rect.left = 0,
		.out_rect.top = 0,
		.out_rect.width = 1920,
		.out_rect.height = 1080,
	},
};

static struct crl_mode_rep ti953_sensor_modes[] = {
	{
		.sd_rects_items = ARRAY_SIZE(ti953_sensor_1080p_rects),
		.sd_rects = ti953_sensor_1080p_rects,
		.binn_hor = 1,
		.binn_vert = 1,
		.scale_m = 1,
		.width = 1920,
		.height = 1080,
		.min_llp = 1905,
		.min_fll = 840,
		.comp_items = 1,
		.ctrl_data = &ctrl_ti953_sensor_data_lanes[0],
	},
};

static struct crl_sensor_subdev_config ti953_sensor_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_BINNER,
		.name = "ti953_sensor binner",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "ti953_sensor pixel array",
	},
};

static struct crl_sensor_limits ti953_sensor_sensor_limits = {
	.x_addr_min = 0,
	.y_addr_min = 0,
	.x_addr_max = 1920,
	.y_addr_max = 1080,
	.min_frame_length_lines = 280,
	.max_frame_length_lines = 2160,
	.min_line_length_pixels = 440,
	.max_line_length_pixels = 3840,
	.scaler_m_min = 1,
	.scaler_m_max = 1,
	.scaler_n_min = 1,
	.scaler_n_max = 1,
	.min_even_inc = 1,
	.max_even_inc = 1,
	.min_odd_inc = 1,
	.max_odd_inc = 1,
};

static struct crl_register_write_rep sensor_fmt_yuyv8[] = {
	/* dummy */
	{0x00, CRL_REG_LEN_DELAY, SER_DES_INIT_DELAY, 0x00},
};

static struct crl_register_write_rep sensor_fmt_uyvy8[] = {
	/* dummy */
	{0x00, CRL_REG_LEN_DELAY, SER_DES_INIT_DELAY, 0x00},
};

static struct crl_csi_data_fmt ti953_sensor_crl_csi_data_fmt[] = {
        {
                .code = MEDIA_BUS_FMT_UYVY8_1X16,
                .pixel_order = CRL_PIXEL_ORDER_GRBG,
                .bits_per_pixel = 16,
                .regs_items = ARRAY_SIZE(sensor_fmt_uyvy8),
                .regs = sensor_fmt_uyvy8,
        },
};

static struct crl_v4l2_ctrl ti953_sensor_v4l2_ctrls[] = {
	{
		.sd_type = CRL_SUBDEV_TYPE_BINNER,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_IDLE,
		.ctrl_id = V4L2_CID_LINK_FREQ,
		.name = "V4L2_CID_LINK_FREQ",
		.type = CRL_V4L2_CTRL_TYPE_MENU_INT,
		.data.v4l2_int_menu.def = 0,
		.data.v4l2_int_menu.max = ARRAY_SIZE(ti953_sensor_pll_configurations) - 1,
		.data.v4l2_int_menu.menu = ti953_sensor_op_sys_clock,
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
		.data.std_data.max = ARRAY_SIZE(ti953_sensor_modes) - 1,
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


struct crl_sensor_configuration ti953_sensor_crl_configuration = {
	.onetime_init_regs_items = ARRAY_SIZE(ti953_sensor_onetime_init_regset),
	.onetime_init_regs = ti953_sensor_onetime_init_regset,
	.poweroff_regs_items = 0,
	.poweroff_regs = 0,

	.subdev_items = ARRAY_SIZE(ti953_sensor_sensor_subdevs),
	.subdevs = ti953_sensor_sensor_subdevs,

	.sensor_limits = &ti953_sensor_sensor_limits,

	.pll_config_items = ARRAY_SIZE(ti953_sensor_pll_configurations),
	.pll_configs = ti953_sensor_pll_configurations,

	.modes_items = ARRAY_SIZE(ti953_sensor_modes),
	.modes = ti953_sensor_modes,

	.streamon_regs_items = ARRAY_SIZE(ti953_sensor_streamon_regs),
	.streamon_regs = ti953_sensor_streamon_regs,

	.streamoff_regs_items = ARRAY_SIZE(ti953_sensor_streamoff_regs),
	.streamoff_regs = ti953_sensor_streamoff_regs,

	.v4l2_ctrls_items = ARRAY_SIZE(ti953_sensor_v4l2_ctrls),
	.v4l2_ctrl_bank = ti953_sensor_v4l2_ctrls,

	.csi_fmts_items = ARRAY_SIZE(ti953_sensor_crl_csi_data_fmt),
	.csi_fmts = ti953_sensor_crl_csi_data_fmt,

	.addr_len = CRL_ADDR_8BIT,
};

#endif   /* __CRLMODULE_TI953_SENSOR_CONFIGURATION_H__  */
