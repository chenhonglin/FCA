
/*
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


#ifndef __CRLMODULE_SERDES_REGS_H_
#define __CRLMODULE_SERDES_REGS_H_


#define SER_ALIAS_ID                    0x12
#define SLAVE_ALIAS_ID_0                0x22
#define SLAVE_ID_0                      0x30
#define OV10635_I2C_ADDR		SLAVE_ALIAS_ID_0

#define SER_DES_INIT_DELAY              100  /* 100ms */

/* DS90UB960 Deserializer Registers */

#define FPD3_DES_RESET                  0x01 /* value: 0x03 -> Reset */
#define FPD3_DES_CSI_PORT_SEL           0x32 /* value: 0x01 -> Select CSI 0 */
#define FPD3_DES_CSI_PLL_CTL            0x1F /* value: 0x05 -> CSI2 800Mbps/lane and BCC 2.5Mbps */
#define FPD3_DES_CSI_CTL                0x33 /* value: 0x01, Enable CSI0 */
#define FPD3_DES_FWD_CTL1               0x20 /* value: 0xF0, */
#define FPD3_DES_FWD_CTL2               0x21 /* value: 0x, */
#define FPD3_DES_PORT_SEL               0x4C /* Value: 0x01 -> select Rx Port 0*/
#define FPD3_DES_BCC_CONFIG             0x58 /* Value: 0x58 */
#define FPD3_DES_SER_ALIAS_ID           0x5C /* Value: SER_ALIAS_ID */
#define FPD3_DES_PORT_CONFIG            0x6D /* Value: 0x7F */
#define FPD3_DES_SLAVE_ID_0             0x5D /* Value: SLAVE_ID_0 */
#define FPD3_DES_SLAVE_ALIAS_ID_0       0x65 /* Value: SLAVE_ALIAS_ID_0 */
#define FPD3_DES_SER_ID			0x5B /* Value: read autoloaded value */
#define FPD3_DES_FS_CTL			0x18 /* Value: 0x01 */
#define FPD3_DES_INTERRUPT_CTL		0x23 /* value: 0x91 */

/* DS90UB913 Serializer Registers */
#define FPD3_SER_DEV_ID                 0x00
#define FPD3_SER_RESET                  0x01
#define FPD3_SER_CONFIG0                0x02
#define FPD3_SER_CONFIG1                0x03
#define FPD3_SER_CONFIG2                0x04
#define FPD3_SER_MODE_SELECT            0x05
#define FPD3_SER_GPO0_1	                0x0D
#define FPD3_SER_GPO2_3	                0x0E

#define FPD3_SER_GENERAL_STATUS		0x0C
#define FPD3_DESER_CSI_STS		0x34
#define FPD3_DESER_RX_PORT_STS1		0x4D
#define FPD3_DESER_RX_PORT_STS2		0x4E
#define FPD3_DESER_CSI_RX_STS		0x7A

#define FPD3_SER_MODE_SELECT		0x05

#define FPD3_SER_SCL_HIGH_TIME		0x11
#define FPD3_SER_SCL_LOW_TIME		0x12

#endif
