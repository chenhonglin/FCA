/*
 * crl_bjev_face_detect_configuration.h
 * Copyright (C) 2016 Harman International Ltd,
 *
 * Author: Sreeju Arumugan Selvaraj <sreeju.selvaraj@harman.com>
 *       : Kiruthika Varadarajan <kiruthika.varadarajan@harman.com>
 *       : Davidson Kumarasan <davidson.kumarasan@harman.com>
 * Created on: 04-09-2018
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

#ifndef __CRLMODULE_BJEV_FACE_DETECT_CONFIGURATION_H__
#define __CRLMODULE_BJEV_FACE_DETECT_CONFIGURATION_H__

#include "crlmodule-sensor-ds.h"
#include "serdes-regs.h"

static struct crl_register_write_rep bjev_face_detect_onetime_init_regset[] = {
	/* Enable only RX2 */
	{0x0c, CRL_REG_LEN_08BIT | CRL_REG_READ_AND_UPDATE, 1<<1, 0x00, 1<<1},
	/* Enable initial skew calibration sequence */
	{0x33, CRL_REG_LEN_08BIT | CRL_REG_READ_AND_UPDATE, 1<<6, 0x00, 1<<6},
	/* Enable Read RX2 port regs */
	{0x4c, CRL_REG_LEN_08BIT, 0x12},
	/* Enable back channel communication */
	{0x58, CRL_REG_LEN_08BIT, 0x5e},
	/* Set alias for serializer */
	{0x5c, CRL_REG_LEN_08BIT, 0x22},
	/* FPD3 mode - COAX CSI */
	{0x6d, CRL_REG_LEN_08BIT, 0x7c},
};

static struct crl_register_write_rep bjev_face_detect_streamon_regs[] = {
	/* Enable continous clock in 953 */
	{0x02, CRL_REG_LEN_08BIT, 0x72, 0x11},
	/* start forwarding from the receiver port 2 */
	{0x20, CRL_REG_LEN_08BIT | CRL_REG_READ_AND_UPDATE, 0<<5, 0x00, 1<<5},
	/* enable CSI port */
	{0x33, CRL_REG_LEN_08BIT | CRL_REG_READ_AND_UPDATE, 1<<0, 0x00, 1<<0},
};

static struct crl_register_write_rep bjev_face_detect_streamoff_regs[] = {
	/* disable CSI port */
	{0x33, CRL_REG_LEN_08BIT | CRL_REG_READ_AND_UPDATE, 0<<0, 0x00, 1<<0},
	/* stop forwarding from the receiver port 2 */
	{0x20, CRL_REG_LEN_08BIT | CRL_REG_READ_AND_UPDATE, 1<<5, 0x00, 1<<5},
};

/* 954 deser operates at 800Mbps and hence configuring it as 400MHz */
const s64 bjev_face_detect_op_sys_clock[] =  { 400000000 };

static struct crl_pll_configuration bjev_face_detect_pll_configurations[] = {
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
		.csi_lanes = 4,
	},
};

static struct crl_subdev_rect_rep bjev_face_detect_1080p_rects[] = {
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

static struct crl_mode_rep bjev_face_detect_modes[] = {
	{
		.sd_rects_items = ARRAY_SIZE(bjev_face_detect_1080p_rects),
		.sd_rects = bjev_face_detect_1080p_rects,
		.binn_hor = 1,
		.binn_vert = 1,
		.scale_m = 1,
		.width = 1920,
		.height = 1080,
		.min_llp = 1905,
		.min_fll = 840,
		.comp_items = 0,
		.ctrl_data = 0,
	},
};

static struct crl_sensor_subdev_config bjev_face_detect_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_BINNER,
		.name = "bjev_fd binner",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "bjev_fd pixel array",
	},
};

static struct crl_sensor_limits bjev_face_detect_sensor_limits = {
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

static struct crl_csi_data_fmt bjev_face_detect_crl_csi_data_fmt[] = {
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.pixel_order = CRL_PIXEL_ORDER_GRBG,
		.bits_per_pixel = 16,
		.regs_items = 0,
		.regs = 0,
	},
};

static struct crl_v4l2_ctrl bjev_face_detect_v4l2_ctrls[] = {
	{
		.sd_type = CRL_SUBDEV_TYPE_BINNER,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_IDLE,
		.ctrl_id = V4L2_CID_LINK_FREQ,
		.name = "V4L2_CID_LINK_FREQ",
		.type = CRL_V4L2_CTRL_TYPE_MENU_INT,
		.data.v4l2_int_menu.def = 0,
		.data.v4l2_int_menu.max = ARRAY_SIZE(bjev_face_detect_pll_configurations) - 1,
		.data.v4l2_int_menu.menu = bjev_face_detect_op_sys_clock,
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
};

struct crl_sensor_configuration bjev_face_detect_crl_configuration = {
	.onetime_init_regs_items = ARRAY_SIZE(bjev_face_detect_onetime_init_regset),
	.onetime_init_regs = bjev_face_detect_onetime_init_regset,

	.poweroff_regs_items = 0,
	.poweroff_regs = 0,

	.subdev_items = ARRAY_SIZE(bjev_face_detect_sensor_subdevs),
	.subdevs = bjev_face_detect_sensor_subdevs,

	.sensor_limits = &bjev_face_detect_sensor_limits,

	.pll_config_items = ARRAY_SIZE(bjev_face_detect_pll_configurations),
	.pll_configs = bjev_face_detect_pll_configurations,

	.modes_items = ARRAY_SIZE(bjev_face_detect_modes),
	.modes = bjev_face_detect_modes,

	.streamon_regs_items = ARRAY_SIZE(bjev_face_detect_streamon_regs),
	.streamon_regs = bjev_face_detect_streamon_regs,

	.streamoff_regs_items = ARRAY_SIZE(bjev_face_detect_streamoff_regs),
	.streamoff_regs = bjev_face_detect_streamoff_regs,

	.v4l2_ctrls_items = ARRAY_SIZE(bjev_face_detect_v4l2_ctrls),
	.v4l2_ctrl_bank = bjev_face_detect_v4l2_ctrls,

	.csi_fmts_items = ARRAY_SIZE(bjev_face_detect_crl_csi_data_fmt),
	.csi_fmts = bjev_face_detect_crl_csi_data_fmt,

	.addr_len = CRL_ADDR_8BIT,
};

#endif  /* __CRLMODULE_BJEV_FACE_DETECT_CONFIGURATION_H__  */
