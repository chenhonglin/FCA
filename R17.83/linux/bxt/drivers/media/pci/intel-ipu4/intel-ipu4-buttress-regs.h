/*
 * Copyright (c) 2014--2017 Intel Corporation.
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

#ifndef INTEL_IPU4_BUTTRESS_REGS_H
#define INTEL_IPU4_BUTTRESS_REGS_H

#define BUTTRESS_REG_WDT			0x8
#define BUTTRESS_REG_BTRS_CTRL			0xc
#define BUTTRESS_REG_BTRS_CTRL_STALL_MODE_VC0	BIT(0)
#define BUTTRESS_REG_BTRS_CTRL_STALL_MODE_VC1	BIT(1)

#define BUTTRESS_REG_FW_RESET_CTL	0x30
#define BUTTRESS_FW_RESET_CTL_START_SHIFT	0
#define BUTTRESS_FW_RESET_CTL_DONE_SHIFT	1

#define BUTTRESS_REG_IS_FREQ_CTL	0x34

#define BUTTRESS_IS_FREQ_CTL_DIVISOR_MASK	0xf

#define BUTTRESS_REG_PS_FREQ_CTL	0x38

#define BUTTRESS_PS_FREQ_CTL_RATIO_MASK		0xff

#define BUTTRESS_FREQ_CTL_START_SHIFT		31
#define BUTTRESS_FREQ_CTL_QOS_FLOOR_SHIFT	8
#define BUTTRESS_FREQ_CTL_QOS_FLOOR_MASK	(0xff << 8)

#define BUTTRESS_REG_PWR_STATE	0x5c

#define BUTTRESS_PWR_STATE_IS_PWR_SHIFT	4
#define BUTTRESS_PWR_STATE_IS_PWR_MASK	(0x7 << 4)

#define BUTTRESS_PWR_STATE_PS_PWR_SHIFT	8
#define BUTTRESS_PWR_STATE_PS_PWR_MASK	(0x7 << 8)

#define BUTTRESS_PWR_STATE_RESET		0x0
#define BUTTRESS_PWR_STATE_PWR_ON_DONE		0x1
#define BUTTRESS_PWR_STATE_PWR_RDY		0x3
#define BUTTRESS_PWR_STATE_PWR_IDLE		0x4

#define BUTTRESS_PWR_STATE_HH_STATUS_SHIFT	12
#define BUTTRESS_PWR_STATE_HH_STATUS_MASK	(0x3 << 12)

enum {
	BUTTRESS_PWR_STATE_HH_STATE_IDLE,
	BUTTRESS_PWR_STATE_HH_STATE_IN_PRGS,
	BUTTRESS_PWR_STATE_HH_STATE_DONE,
	BUTTRESS_PWR_STATE_HH_STATE_ERR,
};

#define BUTTRESS_PWR_STATE_IS_PWR_FSM_SHIFT	20
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_MASK	(0xf << 20)

#define BUTTRESS_PWR_STATE_IS_PWR_FSM_IDLE			0x0
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_WAIT_4_PLL_CMP		0x1
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_WAIT_4_CLKACK		0x2
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_WAIT_4_PG_ACK		0x3
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_RST_ASSRT_CYCLES		0x4
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_STOP_CLK_CYCLES1		0x5
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_STOP_CLK_CYCLES2		0x6
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_RST_DEASSRT_CYCLES	0x7
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_WAIT_4_FUSE_WR_CMP	0x8
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_BRK_POINT			0x9
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_IS_RDY			0xa
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_HALT_HALTED		0xb
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_RST_DURATION_CNT3		0xc
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_WAIT_4_CLKACK_PD		0xd
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_PD_BRK_POINT		0xe
#define BUTTRESS_PWR_STATE_IS_PWR_FSM_WAIT_4_PD_PG_ACK0		0xf

#define BUTTRESS_PWR_STATE_PS_PWR_FSM_SHIFT	24
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_MASK	(0x1f << 24)

#define BUTTRESS_PWR_STATE_PS_PWR_FSM_IDLE			0x0
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_PU_PLL_IP_RDY	0x1
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_RO_PRE_CNT_EXH	0x2
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_PU_VGI_PWRGOOD	0x3
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_RO_POST_CNT_EXH	0x4
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WR_PLL_RATIO		0x5
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_PU_PLL_CMP		0x6
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_PU_CLKACK		0x7
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_RST_ASSRT_CYCLES		0x8
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_STOP_CLK_CYCLES1		0x9
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_STOP_CLK_CYCLES2		0xa
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_RST_DEASSRT_CYCLES	0xb
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_PU_BRK_PNT		0xc
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_FUSE_ACCPT		0xd
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_PS_PWR_UP			0xf
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_4_HALTED		0x10
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_RESET_CNT3		0x11
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_PD_CLKACK		0x12
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_PD_OFF_IND		0x13
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_DVFS_PH4		0x14
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_DVFS_PLL_CMP		0x15
#define BUTTRESS_PWR_STATE_PS_PWR_FSM_WAIT_DVFS_CLKACK		0x16

#define BUTTRESS_REG_SECURITY_CTL	0x300

#define BUTTRESS_SECURITY_CTL_FW_SECURE_MODE_SHIFT	16
#define BUTTRESS_SECURITY_CTL_FW_SETUP_SHIFT		0
#define BUTTRESS_SECURITY_CTL_FW_SETUP_MASK		0x1f

#define BUTTRESS_SECURITY_CTL_FW_SETUP_DONE		0x1
#define BUTTRESS_SECURITY_CTL_AUTH_DONE			0x2
#define BUTTRESS_SECURITY_CTL_AUTH_FAILED			0x8

#define BUTTRESS_REG_SENSOR_FREQ_CTL	0x16c

#define BUTTRESS_SENSOR_FREQ_CTL_OSC_OUT_FREQ_DEFAULT_B0(i) \
					(0x1b << ((i) * 10))
#define BUTTRESS_SENSOR_FREQ_CTL_OSC_OUT_FREQ_SHIFT_B0(i)	((i) * 10)
#define BUTTRESS_SENSOR_FREQ_CTL_OSC_OUT_FREQ_MASK_B0(i) \
					(0x1ff << ((i) * 10))

#define BUTTRESS_SENSOR_CLK_FREQ_6P75MHZ	0x176
#define BUTTRESS_SENSOR_CLK_FREQ_8MHZ		0x164
#define BUTTRESS_SENSOR_CLK_FREQ_9P6MHZ		0x2
#define BUTTRESS_SENSOR_CLK_FREQ_12MHZ		0x1b2
#define BUTTRESS_SENSOR_CLK_FREQ_13P6MHZ	0x1ac
#define BUTTRESS_SENSOR_CLK_FREQ_14P4MHZ	0x1cc
#define BUTTRESS_SENSOR_CLK_FREQ_15P8MHZ	0x1a6
#define BUTTRESS_SENSOR_CLK_FREQ_16P2MHZ	0xca
#define BUTTRESS_SENSOR_CLK_FREQ_17P3MHZ	0x12e
#define BUTTRESS_SENSOR_CLK_FREQ_18P6MHZ	0x1c0
#define BUTTRESS_SENSOR_CLK_FREQ_19P2MHZ	0x0
#define BUTTRESS_SENSOR_CLK_FREQ_24MHZ		0xb2
#define BUTTRESS_SENSOR_CLK_FREQ_26MHZ		0xae
#define BUTTRESS_SENSOR_CLK_FREQ_27MHZ		0x196

#define BUTTRESS_SENSOR_FREQ_CTL_LJPLL_FB_RATIO_MASK		0xff
#define BUTTRESS_SENSOR_FREQ_CTL_SEL_MIPICLK_A_SHIFT		8
#define BUTTRESS_SENSOR_FREQ_CTL_SEL_MIPICLK_A_MASK		(0x2 << 8)
#define BUTTRESS_SENSOR_FREQ_CTL_SEL_MIPICLK_C_SHIFT		10
#define BUTTRESS_SENSOR_FREQ_CTL_SEL_MIPICLK_C_MASK		(0x2 << 10)
#define BUTTRESS_SENSOR_FREQ_CTL_LJPLL_FORCE_OFF_SHIFT		12
#define BUTTRESS_SENSOR_FREQ_CTL_LJPLL_REF_RATIO_SHIFT		14
#define BUTTRESS_SENSOR_FREQ_CTL_LJPLL_REF_RATIO_MASK		(0x2 << 14)
#define BUTTRESS_SENSOR_FREQ_CTL_LJPLL_PVD_RATIO_SHIFT		16
#define BUTTRESS_SENSOR_FREQ_CTL_LJPLL_PVD_RATIO_MASK		(0x2 << 16)
#define BUTTRESS_SENSOR_FREQ_CTL_LJPLL_OUTPUT_RATIO_SHIFT	18
#define BUTTRESS_SENSOR_FREQ_CTL_LJPLL_OUTPUT_RATIO_MASK	(0x2 << 18)
#define BUTTRESS_SENSOR_FREQ_CTL_START_SHIFT			31

#define BUTTRESS_REG_SENSOR_CLK_CTL	0x170

/* 0 <= i <= 2 */
#define BUTTRESS_SENSOR_CLK_CTL_OSC_CLK_OUT_EN_SHIFT(i)		((i) * 2)
#define BUTTRESS_SENSOR_CLK_CTL_OSC_CLK_OUT_SEL_SHIFT(i)	((i) * 2 + 1)

#define BUTTRESS_REG_FW_SOURCE_BASE_LO	0x78
#define BUTTRESS_REG_FW_SOURCE_BASE_HI	0x7C
#define BUTTRESS_REG_FW_SOURCE_SIZE	0x80

#define BUTTRESS_REG_ISR_STATUS		0x90
#define BUTTRESS_REG_ISR_ENABLED_STATUS	0x94
#define BUTTRESS_REG_ISR_ENABLE		0x98
#define BUTTRESS_REG_ISR_CLEAR		0x9C

#define BUTTRESS_ISR_IS_IRQ			(1 << 0)
#define BUTTRESS_ISR_PS_IRQ			(1 << 1)
#define BUTTRESS_ISR_IPC_EXEC_DONE_BY_CSE	(1 << 2)
#define BUTTRESS_ISR_IPC_EXEC_DONE_BY_ISH	(1 << 3)
#define BUTTRESS_ISR_IPC_FROM_CSE_IS_WAITING	(1 << 4)
#define BUTTRESS_ISR_IPC_FROM_ISH_IS_WAITING	(1 << 5)
#define BUTTRESS_ISR_CSE_CSR_SET		(1 << 6)
#define BUTTRESS_ISR_ISH_CSR_SET		(1 << 7)
#define BUTTRESS_ISR_SPURIOUS_CMP		(1 << 8)
#define BUTTRESS_ISR_WATCHDOG_EXPIRED		(1 << 9)
#define BUTTRESS_ISR_PUNIT_2_IUNIT_IRQ		(1 << 10)
#define BUTTRESS_ISR_SAI_VIOLATION		(1 << 11)

#define BUTTRESS_REG_IU2CSEDB0	0x100

#define BUTTRESS_IU2CSEDB0_BUSY_SHIFT		31
#define BUTTRESS_IU2CSEDB0_SHORT_FORMAT_SHIFT	27
#define BUTTRESS_IU2CSEDB0_CLIENT_ID_SHIFT	10
#define BUTTRESS_IU2CSEDB0_IPC_CLIENT_ID_VAL	2

#define BUTTRESS_REG_IU2CSEDATA0	0x104

#define BUTTRESS_IU2CSEDATA0_IPC_BOOT_LOAD		1
#define BUTTRESS_IU2CSEDATA0_IPC_AUTHENTICATE_RUN	2
#define BUTTRESS_IU2CSEDATA0_IPC_AUTHENTICATE_REPLACE	3
#define BUTTRESS_IU2CSEDATA0_IPC_UPDATE_SECURE_TOUCH	16

#define BUTTRESS_REG_IU2CSECSR		0x108

#define BUTTRESS_IU2CSECSR_IPC_PEER_COMP_ACTIONS_RST_PHASE1		(1 << 0)
#define BUTTRESS_IU2CSECSR_IPC_PEER_COMP_ACTIONS_RST_PHASE2		(1 << 1)
#define BUTTRESS_IU2CSECSR_IPC_PEER_QUERIED_IP_COMP_ACTIONS_RST_PHASE	(1 << 2)
#define BUTTRESS_IU2CSECSR_IPC_PEER_ASSERTED_REG_VALID_REQ		(1 << 3)
#define BUTTRESS_IU2CSECSR_IPC_PEER_ACKED_REG_VALID			(1 << 4)
#define BUTTRESS_IU2CSECSR_IPC_PEER_DEASSERTED_REG_VALID_REQ		(1 << 5)

#define BUTTRESS_REG_CSE2IUDB0		0x304
#define BUTTRESS_REG_CSE2IUCSR		0x30C
#define BUTTRESS_REG_CSE2IUDATA0	0x308

/* 0x20 == NACK, 0xf == unknown command */
#define BUTTRESS_CSE2IUDATA0_IPC_NACK      0xf20
#define BUTTRESS_CSE2IUDATA0_IPC_NACK_MASK 0xffff

#define BUTTRESS_REG_ISH2IUCSR		0x50
#define BUTTRESS_REG_ISH2IUDB0		0x54
#define BUTTRESS_REG_ISH2IUDATA0	0x58

#define BUTTRESS_REG_IU2ISHDB0		0x10C
#define BUTTRESS_REG_IU2ISHDATA0	0x110
#define BUTTRESS_REG_IU2ISHDATA1	0x114
#define BUTTRESS_REG_IU2ISHCSR		0x118

#define BUTTRESS_REG_ISH_START_DETECT		0x198
#define BUTTRESS_REG_ISH_START_DETECT_MASK	0x19C

#define BUTTRESS_REG_FABRIC_CMD	0x88

#define BUTTRESS_FABRIC_CMD_START_TSC_SYNC	(1 << 0)
#define BUTTRESS_FABRIC_CMD_IS_DRAIN		(1 << 4)

#define BUTTRESS_REG_TSW_CTL		0x120
#define BUTTRESS_TSW_CTL_SOFT_RESET	(1 << 8)

#define BUTTRESS_REG_TSC_LO	0x164
#define BUTTRESS_REG_TSC_HI	0x168

#define BUTTRESS_REG_CSI2_PORT_CONFIG_AB		0x200
#define BUTTRESS_CSI2_PORT_CONFIG_AB_MUX_MASK		0x1f
#define BUTTRESS_CSI2_PORT_CONFIG_AB_COMBO_SHIFT_B0	16

#define BUTTRESS_REG_PS_FREQ_CAPABILITIES			0xf7498

#define BUTTRESS_PS_FREQ_CAPABILITIES_LAST_RESOLVED_RATIO_SHIFT	24
#define BUTTRESS_PS_FREQ_CAPABILITIES_LAST_RESOLVED_RATIO_MASK	(0xff << 24)
#define BUTTRESS_PS_FREQ_CAPABILITIES_MAX_RATIO_SHIFT		16
#define BUTTRESS_PS_FREQ_CAPABILITIES_MAX_RATIO_MASK		(0xff << 16)
#define BUTTRESS_PS_FREQ_CAPABILITIES_EFFICIENT_RATIO_SHIFT	8
#define BUTTRESS_PS_FREQ_CAPABILITIES_EFFICIENT_RATIO_MASK	(0xff << 8)
#define BUTTRESS_PS_FREQ_CAPABILITIES_MIN_RATIO_SHIFT		0
#define BUTTRESS_PS_FREQ_CAPABILITIES_MIN_RATIO_MASK		(0xff)

#endif /* INTEL_IPU4_BUTTRESS_REGS_H */
