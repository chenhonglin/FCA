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

#ifndef MAX9286_REG_H
#define MAX9286_REG_H

#include <media/max9286.h>

#define DS_ADDR_MAX9286			0x48
#define S_ADDR_MAX96705			0x40
#define S_ADDR_MAX96705_BROADCAST	(S_ADDR_MAX96705 + NR_OF_MAX_STREAMS + 1)

#define ADDR_OV10635_SENSOR		0x30
#define ADDR_AR0143_SENSOR_BASE	0x30
#define ADDR_AR0143_SENSOR		0x5d

/* Deserializer: MAX9286 registers */
#define DS_MAX9286_DEVID		0x1E
#define DS_ENCRC_FPL			0x3F
#define DS_FPL_RT			0x3B
#define DS_CSI_VC_CTL			0x15
#define DS_CSI_DBL_DT			0x12
#define DS_FSYNCMODE			0x01
#define DS_CONFIGL_VIDEOL_DET		0x49
#define DS_LINK_OUTORD			0x0B
#define DS_LINK_ENABLE			0x00
#define DS_FWDCCEN_REVCCEN		0x0A
#define DS_I2CLOCACK			0x34
#define DS_ENEQ				0x1B
#define DS_FSYNC_LOCKED			0x31
#define DS_VSYNCDET			0x27

/* Serializer: MAX96705 registers */
#define S_MAIN_CTL			0x04
#define S_RSVD_8			0x08
#define S_RSVD_97			0x97
#define S_CONFIG			0x07
#define S_SERADDR			0x00
#define S_I2C_SOURCE_IS			0x09
#define S_I2C_DST_IS			0x0A
#define S_I2C_SOURCE_SER		0x0B
#define S_I2C_DST_SER			0x0C
#define S_INPUT_STATUS			0x15
#define S_CMLLVL_PREEMP			0x06

#endif
