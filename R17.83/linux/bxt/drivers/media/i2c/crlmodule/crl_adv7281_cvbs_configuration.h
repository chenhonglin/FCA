/*
 * Copyright (c) 2015--2016 Harman International. All Rights Reserved.
 *
 * Author: Sreeju Arumugan Selvaraj <sreeju.selvaraj@harman.com>
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

#ifndef __CRLMODULE_ADV7281_CVBS_CONFIGURATION_H__
#define __CRLMODULE_ADV7281_CVBS_CONFIGURATION_H__

#include "crlmodule-sensor-ds.h"

static struct crl_pll_configuration adv7281_cvbs_pll_configurations[] = {
	{
		.input_clk = 28636360,
		.op_sys_clk = 216000000,
		.bitsperpixel = 16,
		.pixel_rate_csi = 27000000,
		.pixel_rate_pa = 27000000,
		.csi_lanes = 1,
	 },
	 {
		.input_clk = 24000000,
		.op_sys_clk = 130000000,
		.bitsperpixel = 16,
		.pixel_rate_csi = 130000000,
		.pixel_rate_pa = 130000000,
		.csi_lanes = 1,
	 },
};

static struct crl_subdev_rect_rep adv7281_cvbs_ntsc_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 720,
		.in_rect.height = 243,
		.out_rect.left = 0,
		.out_rect.top = 0,
		.out_rect.width = 720,
		.out_rect.height = 243,
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_BINNER,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 720,
		.in_rect.height = 243,
		.out_rect.left = 0,
		.out_rect.top = 0,
		.out_rect.width = 720,
		.out_rect.height = 243,
	},
};

static struct crl_mode_rep adv7281_cvbs_modes[] = {
	{
		.sd_rects_items = ARRAY_SIZE(adv7281_cvbs_ntsc_rects),
		.sd_rects = adv7281_cvbs_ntsc_rects,
		.binn_hor = 1,
		.binn_vert = 1,
		.scale_m = 1,
		.width = 720,
		.height = 243,
	},
};

static struct crl_sensor_subdev_config adv7281_cvbs_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_BINNER,
		.name = "adv7281 cvbs binner",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "adv7281 cvbs pixel array",
	},
};

static struct crl_sensor_limits adv7281_cvbs_sensor_limits = {
	.x_addr_min = 0,
	.y_addr_min = 0,
	.x_addr_max = 720,
	.y_addr_max = 243,
	.min_frame_length_lines = 160,
	.max_frame_length_lines = 65535,
	.min_line_length_pixels = 6024,
	.max_line_length_pixels = 32752,
	.scaler_m_min = 1,
	.scaler_m_max = 1,
	.scaler_n_min = 1,
	.scaler_n_max = 1,
	.min_even_inc = 1,
	.max_even_inc = 1,
	.min_odd_inc = 1,
	.max_odd_inc = 1,
};

static struct crl_csi_data_fmt adv7281_cvbs_crl_csi_data_fmt[] = {
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.pixel_order = CRL_PIXEL_ORDER_GRBG,
		.bits_per_pixel = 16,
	},
};

static struct crl_v4l2_ctrl adv7281_cvbs_v4l2_ctrls[] = {
	{
		.sd_type = CRL_SUBDEV_TYPE_BINNER,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_IDLE,
		.ctrl_id = V4L2_CID_LINK_FREQ,
		.name = "V4L2_CID_LINK_FREQ",
		.type = CRL_V4L2_CTRL_TYPE_MENU_INT,
		.impact = CRL_IMPACTS_NO_IMPACT,
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
		.impact = CRL_IMPACTS_NO_IMPACT,
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
		.impact = CRL_IMPACTS_NO_IMPACT,
	},
};

int adv7281_init(struct i2c_client *);
int adv7281_cleanup(struct i2c_client *);
int adv7281_streamon_callback(struct i2c_client* client);
int adv7281_streamoff_callback(struct i2c_client* client);
int adv7281_suspend_callback(struct i2c_client* client);
int adv7281_resume_callback(struct i2c_client* client);
int adv7281_sensor_status(struct i2c_client* client, int *status);
irqreturn_t adv7281_threaded_irq_fn(int irq, void *sensor_struct);

static struct crl_sensor_configuration adv7281_cvbs_crl_configuration = {
	.sensor_init    = adv7281_init,
	.sensor_cleanup = adv7281_cleanup,
	.streamon_callback = adv7281_streamon_callback,
	.streamoff_callback = adv7281_streamoff_callback,

	.suspend_callback  = adv7281_suspend_callback,
	.resume_callback  = adv7281_resume_callback,

	.subdev_items = ARRAY_SIZE(adv7281_cvbs_sensor_subdevs),
	.subdevs = adv7281_cvbs_sensor_subdevs,

	.sensor_limits = &adv7281_cvbs_sensor_limits,

	.pll_config_items = ARRAY_SIZE(adv7281_cvbs_pll_configurations),
	.pll_configs = adv7281_cvbs_pll_configurations,

	.modes_items = ARRAY_SIZE(adv7281_cvbs_modes),
	.modes = adv7281_cvbs_modes,

	.v4l2_ctrls_items = ARRAY_SIZE(adv7281_cvbs_v4l2_ctrls),
	.v4l2_ctrl_bank = adv7281_cvbs_v4l2_ctrls,

	.csi_fmts_items = ARRAY_SIZE(adv7281_cvbs_crl_csi_data_fmt),
	.csi_fmts = adv7281_cvbs_crl_csi_data_fmt,

	.addr_len = CRL_ADDR_7BIT,

	.irq_in_use = true,
	.crl_irq_fn = NULL,
	.crl_threaded_irq_fn = adv7281_threaded_irq_fn,

	.sensor_status = adv7281_sensor_status
};

#endif  /* __CRLMODULE_ADV7281_CVBS_CONFIGURATION_H__ */
