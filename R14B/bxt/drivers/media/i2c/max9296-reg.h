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

#ifndef MAX9296_REG_H
#define MAX9296_REG_H

#include <media/max9296.h>

#define DS_ADDR_MAX9296			0x48
#define S_ADDR_MAX96705			0x40
#define S_ADDR_MAX96705_BROADCAST (S_ADDR_MAX96705 + NR_OF_MAX_STREAMS + 1)

/* Deserializer: MAX9296 registers */
#define DS_MAX9296_DEVID		0x000D
#define DS_LINKA_LOCKED			0x0BCB
#define DS_LINKB_LOCKED			0x0CCB
#define DS_PIPEX_LOCKED			0x01DC
#define DS_I2CLOCACK_LINKA		0x0B0D
#define DS_I2CLOCACK_LINKB		0x0C0D
#define DS_CSI_VC_CTL			0x0313

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
#define S_DBL_ALIGN			0x67
#define S_GPIO_OUT			0x0F

#define ADDR_AR1043_RVC_BASE		0x30
#define ADDR_AR1043_SENSOR		0x1a

#endif
