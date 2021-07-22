/*
 * Copyright (c) 2016 Intel Corporation.
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
#ifndef TI954_REG_H
#define TI954_REG_H

#define TI954_RX_PORT_USED_MASK 0x03
#define TI954_RX_PORT_NUM       2
#define TI954_RX_PORT_0         0
#define TI954_RX_PORT_1         1
#define DS_ADDR_TI954		    0x30
#define TI954_DEVID		        0x00
#define TI954_RESET		        0x01
#define TI954_CSI_PLL_CTL	    0x1f
#define TI954_FS_CTL		    0x18
#define TI954_FWD_CTL1		    0x20
#define TI954_RX_PORT_SEL	    0x4c
#define TI954_RX_PORT_STS1      0x4D
#define TI954_SLAVE_ID0		    0x5d
#define TI954_SLAVE_ALIAS_ID0	0x65
#define TI954_PORT_CONFIG	    0x6d
#define TI954_BC_GPIO_CTL0	    0x6e
#define TI954_PORT_CONFIG2	    0x7c
#define TI954_IND_ACC_DATA	    0xb2
#define TI954_CSI_CTL           0x33

#define TI954_POWER_ON		    0x1
#define TI954_POWER_OFF		    0x20
#define TI954_FPD3_RAW10_100MHz	0x7f
#define TI954_FPD3_RAW12_50MHz	0x7d
#define TI954_FPD3_RAW12_75MHz	0x7e
#define TI954_RAW10_NORMAL	    0x1
#define TI954_RAW10_8BIT	    0x81
#define TI954_GPIO0_HIGH	    0x09
#define TI954_GPIO0_LOW		    0x08
#define TI954_GPIO1_HIGH	    0x90
#define TI954_GPIO1_LOW		    0x80
#define TI954_GPIO0_FSIN	    0x0a
#define TI954_GPIO1_FSIN	    0xa0
#define TI954_GPIO0_MASK	    0x0f
#define TI954_GPIO1_MASK	    0xf0
#define TI954_MIPI_800MBPS	    0x2
#define TI954_MIPI_1600MBPS	    0x0
#define TI954_CSI_ENABLE	    0x1
#define TI954_RX_PORT1_EN	    0x10
#define TI954_RX_PORT0_EN	    0x00
#define TI954_RX_LOCK_STS       0x01
#define TI954_CSI_SKEWCAL	    0x40
#define TI954_FSIN_ENABLE	    0x1

#endif
