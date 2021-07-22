/*
 * Copyright (c) 2017 Intel Corporation.
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

#ifndef MAX9296_H
#define MAX9296_H

#include <linux/i2c.h>
#include <linux/regmap.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define MAX9296_NAME		"max9296"

#define PIXEL_ORDER_GRBG	0
#define PIXEL_ORDER_RGGB	1
#define PIXEL_ORDER_BGGR	2
#define PIXEL_ORDER_GBRG	3

#define NR_OF_MAX_STREAMS	2
#define NR_OF_MAX_SOURCE_PADS	1
#define NR_OF_MAX_SINK_PADS	2
#define NR_OF_MAX_PADS		(NR_OF_MAX_SOURCE_PADS + NR_OF_MAX_SINK_PADS)

#define MAX_PAD_SOURCE		2

#define MAX9296_MIN_WIDTH	640
#define MAX9296_MIN_HEIGHT	480
#define MAX9296_MAX_WIDTH	1280
#define MAX9296_MAX_HEIGHT	800

struct max9296_csi_data_format {
	u32 code;
	u8 width;
	u8 compressed;
	u8 pixel_order;
	u8 mipi_dt_code;
};

struct max9296_subdev_i2c_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
};

struct max9296_pdata {
	unsigned int subdev_num;
	struct max9296_subdev_i2c_info *subdev_info;
	unsigned int reset_gpio;
};

#endif

