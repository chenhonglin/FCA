/*
 * crl_ar0143_avm_configuration_avm.h
 * Copyright (C) 2016 Harman International Ltd,
 *
 * Author: Sreeju Arumugan Selvaraj <sreeju.selvaraj@harman.com>
 * Created on: 18-08-2016
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

#ifndef __CRLMODULE_AR0143_AVM_CONFIGURATION_H__
#define __CRLMODULE_AR0143_AVM_CONFIGURATION_H__

#include "crlmodule-sensor-ds.h"
#include "ar0143-reg.h"

struct crl_ctrl_data_pair ctrl_ar0143_avm_data_lanes[] = {
	{
		.ctrl_id = V4L2_CID_MIPI_LANES,
		.data = 4,
	},
};

static struct crl_register_write_rep ar0143_avm_onetime_init_regset[] = {
	{0x00, CRL_REG_LEN_DELAY, 10, 0x00},
};


/* PCLK = 72Mhz, CSI clock for 4 cameras = 288Mhz */
const s64 ar0143_avm_op_sys_clock[] =  { 288000000 };

static struct crl_pll_configuration ar0143_avm_pll_configurations[] = {
	{
		/* PCLK = 72Mhz, Total CSI Clock speed (4 cameras) = 72Mhz * 4,
		Data rate = 576Mbps, pixel rate per lane = 576/4 = 144000000  */
		.input_clk = 25000000,
		.op_sys_clk = 288000000,
		.pixel_rate_csi = 144000000,
		.pixel_rate_pa = 144000000,
		.bitsperpixel = 16,
		.comp_items = 0,
		.ctrl_data = 0,
		.pll_regs_items = 0,
		.pll_regs = NULL,
		.csi_lanes = 4,
	},
};

static struct crl_register_write_rep ar0143_avm_1280_960[] = {
	{ 0xCA80, CRL_REG_LEN_16BIT, 0x0002 },
	/* cam_fov_calib_x_offset */
	{ 0xC8A8, CRL_REG_LEN_08BIT, 0x00 },
	/* cam_fov_calib_y_offset */
	{ 0xC8A9, CRL_REG_LEN_08BIT, 0x00 },
	/* y_addr_start */
	{ 0xC804, CRL_REG_LEN_16BIT, 0x0008 },
	/* y_addr_end */
	{ 0xC808, CRL_REG_LEN_16BIT, 0x03C7 },
	/* frame_len_lines */
	{ 0xC814, CRL_REG_LEN_16BIT, 0x045B },
	/* crop_height */
	{ 0xC8A2, CRL_REG_LEN_16BIT, 0x03C0 },
	/* output_height */
	{ 0xCA92, CRL_REG_LEN_16BIT, 0x03C0 },
	{ 0xFC00, CRL_REG_LEN_16BIT, 0x2800 },
	{ 0x0040, CRL_REG_LEN_16BIT, 0x8100 },
	{ 0x0040, CRL_REG_LEN_16BIT, 0x8606 },
};

static struct crl_subdev_rect_rep ar0143_avm_720p_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 1280,
		.in_rect.height = 720,
		.out_rect.left = 0,
		.out_rect.top = 0,
		.out_rect.width = 1280,
		.out_rect.height = 720,
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_BINNER,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 1280,
        .in_rect.height = 720,
        .out_rect.left = 0,
        .out_rect.top = 0,
        .out_rect.width = 1280,
        .out_rect.height = 720,
	},
};


static struct crl_subdev_rect_rep ar0143_avm_1280_960_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 1280,
		.in_rect.height = 960,
		.out_rect.left = 0,
		.out_rect.top = 0,
		.out_rect.width = 1280,
		.out_rect.height = 960,
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_BINNER,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 1280,
        .in_rect.height = 960,
        .out_rect.left = 0,
        .out_rect.top = 0,
        .out_rect.width = 1280,
        .out_rect.height = 960,
	},
};

static struct crl_mode_rep ar0143_avm_modes[] = {
	{
		.sd_rects_items = ARRAY_SIZE(ar0143_avm_720p_rects),
		.sd_rects = ar0143_avm_720p_rects,
		.binn_hor = 1,
		.binn_vert = 1,
		.scale_m = 1,
		.width = 1280,
		.height = 720,
		.min_llp = 1905,
        .min_fll = 840,
		.comp_items = 1,
		.ctrl_data = &ctrl_ar0143_avm_data_lanes[0],
	},
	{
		.sd_rects_items = ARRAY_SIZE(ar0143_avm_1280_960_rects),
		.sd_rects = ar0143_avm_1280_960_rects,
		.binn_hor = 1,
		.binn_vert = 1,
		.scale_m = 1,
		.width = 1280,
		.height = 960,
		.min_llp = 2250,
		.min_fll = 1320,
		.comp_items = 1,
		.ctrl_data = &ctrl_ar0143_avm_data_lanes[0],
		.mode_regs_items = ARRAY_SIZE(ar0143_avm_1280_960),
		.mode_regs = ar0143_avm_1280_960,
	},
};

static struct crl_sensor_subdev_config ar0143_avm_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_BINNER,
		.name = "ar0143-avm binner",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "ar0143-avm pixel array",
	},
};

static struct crl_sensor_limits ar0143_avm_sensor_limits = {
	.x_addr_min = 0,
	.y_addr_min = 0,
	.x_addr_max = 1344,
	.y_addr_max = 968,
	.min_frame_length_lines = 280,
	.max_frame_length_lines = 1115,
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

static struct crl_register_write_rep ar0143_avm_mode_uyvy_regs[] = {
   {0xCA9C, CRL_REG_LEN_16BIT, 0x0405},
   {0xFC00, CRL_REG_LEN_16BIT, 0x2800},
   {0x0040, CRL_REG_LEN_16BIT, 0x8100},
   {0x0000, CRL_REG_LEN_DELAY, 10}, /* 10ms delay */
};

static struct crl_register_write_rep ar0143_avm_mode_yuyv_regs[] = {
   {0xCA9C, CRL_REG_LEN_16BIT, 0x0605},
   {0xFC00, CRL_REG_LEN_16BIT, 0x2800},
   {0x0040, CRL_REG_LEN_16BIT, 0x8100},
   {0x0000, CRL_REG_LEN_DELAY, 10}, /* 10 ms delay */
};

static struct crl_csi_data_fmt ar0143_avm_crl_csi_data_fmt[] = {
	{
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.pixel_order = CRL_PIXEL_ORDER_GRBG,
		.bits_per_pixel = 16,
		.regs_items = ARRAY_SIZE(ar0143_avm_mode_yuyv_regs),
		.regs = ar0143_avm_mode_yuyv_regs,
	},
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.pixel_order = CRL_PIXEL_ORDER_GRBG,
		.bits_per_pixel = 16,
		.regs_items = ARRAY_SIZE(ar0143_avm_mode_uyvy_regs),
		.regs = ar0143_avm_mode_uyvy_regs,
	},
};


static struct crl_v4l2_ctrl ar0143_avm_v4l2_ctrls[] = {
	{
		.sd_type = CRL_SUBDEV_TYPE_BINNER,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_IDLE,
        .ctrl_id = V4L2_CID_LINK_FREQ,
		.name = "V4L2_CID_LINK_FREQ",
		.type = CRL_V4L2_CTRL_TYPE_MENU_INT,
		.data.v4l2_int_menu.def = 0,
		.data.v4l2_int_menu.max = ARRAY_SIZE(ar0143_avm_pll_configurations) - 1,
		.data.v4l2_int_menu.menu = ar0143_avm_op_sys_clock,
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
		.data.std_data.max = ARRAY_SIZE(ar0143_avm_modes) - 1,
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


struct crl_sensor_configuration ar0143_avm_crl_configuration = {
	.onetime_init_regs_items = ARRAY_SIZE(ar0143_avm_onetime_init_regset),
	.onetime_init_regs = ar0143_avm_onetime_init_regset,
	.poweroff_regs_items = 0,
	.poweroff_regs = 0,

	.subdev_items = ARRAY_SIZE(ar0143_avm_sensor_subdevs),
	.subdevs = ar0143_avm_sensor_subdevs,

	.sensor_limits = &ar0143_avm_sensor_limits,

	.pll_config_items = ARRAY_SIZE(ar0143_avm_pll_configurations),
	.pll_configs = ar0143_avm_pll_configurations,

	.modes_items = ARRAY_SIZE(ar0143_avm_modes),
	.modes = ar0143_avm_modes,

	.v4l2_ctrls_items = ARRAY_SIZE(ar0143_avm_v4l2_ctrls),
	.v4l2_ctrl_bank = ar0143_avm_v4l2_ctrls,

	.csi_fmts_items = ARRAY_SIZE(ar0143_avm_crl_csi_data_fmt),
	.csi_fmts = ar0143_avm_crl_csi_data_fmt,
};

#endif  /* __CRLMODULE_AR0143_AVM_CONFIGURATION_H__ */
