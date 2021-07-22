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

#ifndef MAX96705_REG_H
#define MAX96705_REG_H

#define SER_ADDR_MAX96705			0x40

/* Serializer: MAX96705 registers */
#define SER_MAIN_CTL			0x04
#define SER_CONFIG				0x07
#define SER_GPIO_OUT			0x0F
#define SER_CROSSBAR_0			0x20
#define SER_CROSSBAR_1			0x21
#define SER_CROSSBAR_2			0x22
#define SER_CROSSBAR_3			0x23
#define SER_CROSSBAR_4			0x24
#define SER_CROSSBAR_7			0x27
#define SER_CROSSBAR_8			0x28
#define SER_CROSSBAR_9			0x29
#define SER_CROSSBAR_16			0x30
#define SER_CROSSBAR_17			0x31
#define SER_CROSSBAR_18			0x32
#define SER_CROSSBAR_19			0x33
#define SER_CROSSBAR_20			0x34
#define SER_CROSSBAR_21			0x35
#define SER_CROSSBAR_22			0x36
#define SER_CROSSBAR_23			0x37
#define SER_CROSSBAR_24			0x38
#define SER_CROSSBAR_25			0x39
#define SER_CROSSBAR_26			0x3A
#define SER_I2C_SOURCE_IS		0x09
#define SER_I2C_DST_IS			0x0A
#define SER_DBL_ALIGN			0x67
#define	SER_MAX96705_DEVID		0x1E

#endif
