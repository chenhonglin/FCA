/*
 * Copyright (c) 2017 Harman International Corporation.
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

#ifndef MAX9288_REG_H
#define MAX9288_REG_H

#include <media/max9288.h>

#define DSER_ADDR_MAX9288		0x68

#define DSER_MAX9288_DEVID		0x1E
#define DSER_BWS				0x17
#define	DSER_I2C_CTRL			0x1C
#define DSER_AUDIO				0x02
#define DSER_CSI_DATA			0x60
#define DSER_CSI_LANE_SEL		0x65
#define DSER_CSI_LANE_MAP		0x66
#define DSER_AUTOPPL			0x09
#define DSER_CHNL_CTRL			0x04

#endif
