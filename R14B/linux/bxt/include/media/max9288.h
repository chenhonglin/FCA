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

#ifndef MAX9288_H
#define MAX9288_H

#include <linux/i2c.h>
#include <linux/regmap.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define MAX9288_NAME		"max9288"

#define PIXEL_ORDER_GRBG	0
#define PIXEL_ORDER_RGGB	1
#define PIXEL_ORDER_BGGR	2
#define PIXEL_ORDER_GBRG	3

/* TODO: Revisit these macro values, max9288 has only one
 * input port, so streams also 1 */
#define NR_OF_MAX9288_STREAMS	1
#define NR_OF_MAX9288_SOURCE_PADS	1
#define NR_OF_MAX9288_SINK_PADS	1
#define NR_OF_MAX9288_PADS		(NR_OF_MAX9288_SOURCE_PADS + NR_OF_MAX9288_SINK_PADS)

#define MAX9288_PAD_SOURCE		1

#define MAX9288_MIN_WIDTH	640
#define MAX9288_MIN_HEIGHT	480
#define MAX9288_MAX_WIDTH	1280
#define MAX9288_MAX_HEIGHT	800

#define ADDR_AR0143_RVC_BASE    0x20
#define ADDR_AR0143_SENSOR      0x5d

struct max9288_csi_data_format {
	u32 code;
	u8 width;
	u8 compressed;
	u8 pixel_order;
	u8 mipi_dt_code;
};

struct max9288_subdev_i2c_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
};

struct max9288_pdata {
	unsigned int subdev_num;
	struct max9288_subdev_i2c_info *subdev_info;
	unsigned int reset_gpio;
};

#endif
