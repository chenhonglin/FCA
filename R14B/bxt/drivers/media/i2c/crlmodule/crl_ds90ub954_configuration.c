
/*
 * crl_ds90ub954_configuration.c
 * Copyright (C) 2016 Harman International Ltd,
 
 * Author: Sreeju Arumugan Selvaraj <sreeju.selvaraj@harman.com>
	 : Kiruthika Varadarajan <kiruthika.varadarajan@harman.com>
 * Created on: 15-06-2018
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

//#define DEBUG
//#define DEBUG_954_STREAM_ONOFF

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include "crlmodule.h"
#include "crlmodule-regs.h"
#include "serdes-regs.h"
#include "crlmodule-sensor-ds.h"

#define CAM_LVDS_RST	444

/* Registers used for reset detection... set these to some dummy/sentinel
 * value. If they ever change: re-configure chip! */
#define TI954_RESET_DETECT_REG0 (0x71) /* RAW-12 ID register is unused */
#define TI954_RESET_DETECT_VAL0 (0x3f) /* Default value is 0x2C */
#define TI954_RESET_DETECT_REG1 (0x78) /* MAILBOX_1 - UNUSED */
#define TI954_RESET_DETECT_VAL1 (0x13) /* Default value is 0x00 */
#define TI954_RESET_DETECT_REG2 (0x79) /* RAW-12 ID register is unused */
#define TI954_RESET_DETECT_VAL2 (0x37) /* MAILBOX_2 - UNUSED */

#define TI954_FWD_CTL1	(0x20)
#define TI954_FWD_CTL1_PORT1_DIS (0x20)
#define TI954_FWD_CTL1_PORT0_DIS (0x10)
#define TI954_FWD_CTL1_DISABLE_ALL (TI954_FWD_CTL1_PORT0_DIS | TI954_FWD_CTL1_PORT1_DIS)

#define TI954_CSI_CTL  (0x33)
#define TI954_CSI_CTL_CALEN  (0x40)
#define TI954_CSI_CTL_4LANE  (0x00)
#define TI954_CSI_CTL_3LANE  (0x10)
#define TI954_CSI_CTL_2LANE  (0x20)
#define TI954_CSI_CTL_1LANE  (0x30)
#define TI954_CSI_CTL_ENABLE (0x01)

#define TI954_CSI_CTL_STREAMON  (TI954_CSI_CTL_CALEN | TI954_CSI_CTL_2LANE | TI954_CSI_CTL_ENABLE)
#define TI954_CSI_CTL_STREAMOFF (TI954_CSI_CTL_CALEN | TI954_CSI_CTL_2LANE)

#define TI954_STS1_PASS (0x02)
#define TI954_STS1_LOCK (0x01)

/* Bank switching registers and modes */
#define TI954_REGSEL       (0x4C)
#define TI954_REGSEL_W_ALL (0x03)
#define TI954_REGSEL_RW_P0 (0x01)
#define TI954_REGSEL_RW_P1 (0x12)


#define TI954_INT_CTL      (0x23)
#define TI954_INT_CTL_ALL  (0x93)
#define TI954_INT_CTL_NONE (0x00)

/* FDP-Link mode */
#define TI954_PORT_CONFIG    (0x6d)
#define TI954_FPD_MODE_CSI   0x00
#define TI954_FPD_MODE_RAW10 0x03
#define TI954_FPD_MODE_MASK  0x03
/* Compute port_config register by FPD-Link mode. */
#define TI954_PORT_CONFIG_VAL(m)  (0x7C | (m & TI954_FPD_MODE_MASK))

/* Compute bcc_config register based on FPD-Link mode */
#define TI954_BCC_CONFIG    (0x58)
#define TI954_BCC_CONFIG_VAL(m)  (((m) == TI954_FPD_MODE_CSI) ? 0x1a : 0x58)


/* Hardware states */
enum {
	/* Report this state internally when hardware is in an UNKNOWN state do
	 * to failed hardware operation.  Note: zero value ALSO used to
	 * indicate when there are no state change requests pending */
	DS90UB954_HW_STATE_UNKNOWN = 0,  

	/* Hardware is known to be in reset state as a result of either hard or
	 * soft reset operation */
	DS90UB954_HW_STATE_RESET,

	/* Hardware has been initialized and is READY for streaming, but not
	 * actively sending CSI packets to the SOC. */
	DS90UB954_HW_STATE_READY,

	/* Video decoder HW is expected to be actively streaming something to
	 * SOC (barring excetpions for HW failure/unexpected behavior).  */
	DS90UB954_HW_STATE_STREAMING,

	/* Used for suspend-to-ram / pm_suspend state */
	DS90UB954_HW_STATE_SUSPENDED
};

/* For "misc" sysfs => hwtask requests. */
enum {
	MISC_REQ_NONE = 0,      /* No request pending */
	MISC_REQ_INPUT_SELECT,  /* Request hw thread to update input_select */
	MISC_REQ_BIT_ORDER,     /* Change bit-order selection */
	MISC_REQ_P0_FPDMODE,    /* Change port-0 FPD-link mode */
	MISC_REQ_P1_FPDMODE,    /* Change port-1 FPD-link mode */
#if 0
	MISC_REQ_REGISTER_DUMP, /* Request hw thread to dump all registers when bus is free */
	MISC_REQ_FORCE_RESET,   /* Request forcing a reset of the chip */
#endif
};

struct ti954_port_status 
{
	uint8_t  sts1;         /* reg 0x4d */
	uint8_t  sts2;	       /* reg 0x4e */
	uint32_t rx_freq;      /* Receive freq in Hz [0x4f-0x50] */
	uint16_t line_count;   /* line count registers  [x73-74] */
	uint16_t line_len;     /* line length registers [0x75-76] */
	uint8_t  dg_bit_order; /* bit order for this port. */
	uint8_t  fpd_mode;     /* FPD-Link3 mode. */
};



#ifdef DEBUG_954_STREAM_ONOFF
#define DEBUG_MAX_REGS 64 
struct debug_regset {
	struct crl_register_write_rep regs[DEBUG_MAX_REGS];
	size_t numregs;
};
#endif

/* Structure for storing sensor specific data,
 * accessed through crl_sensor.sensor_specific_data */
struct crl_ds90ub954 {
	/* HW Mutex used to synchronize access to functions that request a
	 * change in HW state or request to HW mgmt thread. */
	struct mutex hw_mutex;

	/* Spin-lock for protecting members of this struct */
	spinlock_t slock;

	/* Input select */
	int input_select;  /* Determine ACTIVE port */

	/* For selecting expected sensor type */
	int camera_type;

	/* Port status, tracked idependently */
	struct ti954_port_status  port0_info;
	struct ti954_port_status  port1_info;
	struct ti954_port_status* active_port; 

	/* stats counter */
	uint32_t streamon_count;
	uint32_t streamoff_count;
	uint32_t interrupt_count;
	uint32_t input_change_count;

	/**
	 * Single thread for ALL hw access. Allows for simple & self-contained
	 * retry mechanims, and easier logic to prevent concurrent access from
	 * multiple execution contexts without.
	 */
	struct task_struct * hw_task;

	/* Last KNOWN HW state (to best of our knowledge for SoC) */
	int	hw_state;

	/* Last REQUESTED hardware state. */
	int     requested_hw_state;

	/* Notification to wake HW thread */
	int     irq_wake;

	/* For miscelaneous requests to HW thread */
	int     misc_request;

	/* Should the HW thread be suspended?? */
	int     should_suspend;

	/* IS the HW thread suspended */
	int     suspended;

	/* Wait-queue for HW thread to wait for events from crlmodule core or
	 * userspace (e.g. sysfs) */
	wait_queue_head_t hw_wait;

	/* Wait-queue for waking "user" thread contexts, i.e. calls that will
	 * come in through various V4L ioctls like stream on and streamoff, or
	 * calls coming in from sysfs writes */
	wait_queue_head_t usr_wait;

	/* Pointer back to i2c_client... mostly for calling dev_X logging funcs
	 * from functions deeper in the call stack */
	struct i2c_client *client;

#ifdef DEBUG_954_STREAM_ONOFF
	struct debug_regset rin1_streamon;
	struct debug_regset rin0_streamon;
	struct debug_regset streamoff_regs;
#endif
};


static int ds90ub954_dg_bit_order(struct i2c_client *client);
static void ds90ub954_update_fpd_mode(struct i2c_client *client, int port);
static int ds90ub954_i2c_write(struct i2c_client *client, u16 i2c_addr, u16 reg, u8 val);
static int ds90ub954_i2c_read(struct i2c_client *client, u16 i2c_addr, u16 reg, u32 * val);
static int ds90ub954_sw_reset(struct i2c_client *client);
static int ds90ub954_hw_streamon(struct i2c_client *client);
static int ds90ub954_hw_streamoff(struct i2c_client *client);
static int ds90ub954_hw_op_init(struct i2c_client *client);
static int ds90ub954_read_isr_status(struct i2c_client *client, int force_read);
static void ds90ub954_periodic_service(struct i2c_client *client);


// state & flag mgmt.
static int get_hwstate(struct crl_ds90ub954 * sensor_data);
static void set_hwstate(struct crl_ds90ub954 * sensor_data, int new_state);
static int set_miscreq(struct crl_ds90ub954 * sensor_data, int req);
static int get_miscreq(struct crl_ds90ub954 * sensor_data);
static void get_active_port_status(struct crl_ds90ub954 * sensor_data, struct ti954_port_status *p);


static struct crl_subdev_rect_rep ti954_mopar_1280x800_rects[] = {
	{
	 .subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
	 .in_rect.left = 0,
	 .in_rect.top = 0,
	 .in_rect.width = 1280,
	 .in_rect.height = 800,
	 .out_rect.left = 0,
	 .out_rect.top = 0,
	 .out_rect.width = 1280,
	 .out_rect.height = 800,
	 },

	{
	 .subdev_type = CRL_SUBDEV_TYPE_BINNER,
	 .in_rect.left = 0,
	 .in_rect.top = 0,
	 .in_rect.width = 1280,
	 .in_rect.height = 800,
	 .out_rect.left = 0,
	 .out_rect.top = 0,
	 .out_rect.width = 1280,
	 .out_rect.height = 800,
	 },
};

static struct crl_mode_rep ti954_mopar_modes[] = {
	{
		.sd_rects_items = ARRAY_SIZE(ti954_mopar_1280x800_rects),
		.sd_rects = ti954_mopar_1280x800_rects,
		.binn_hor = 1,
		.binn_vert = 1,
		.scale_m = 1,
		.width = 1280,
		.height = 800,
                .min_llp = 1905,
                .min_fll = 840,
		.comp_items = 0,
		.ctrl_data = 0,
	 },
};

static struct crl_sensor_limits ti954_mopar_sensor_limits = {
	.x_addr_min = 0,
	.y_addr_min = 0,
	.x_addr_max = 1280,
	.y_addr_max = 800,
	.min_frame_length_lines = 280,
	.max_frame_length_lines = 840,
	.min_line_length_pixels = 440,
	.max_line_length_pixels = 1905,
        .scaler_m_min = 1,
        .scaler_m_max = 1,
        .scaler_n_min = 1,
        .scaler_n_max = 1,
        .min_even_inc = 1,
        .max_even_inc = 1,
        .min_odd_inc = 1,
        .max_odd_inc = 1,
};

static struct crl_subdev_rect_rep ti954_mopar_1024x768_rects[] = {
	{
	 .subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
	 .in_rect.left = 0,
	 .in_rect.top = 0,
	 .in_rect.width = 1024,
	 .in_rect.height = 768,
	 .out_rect.left = 0,
	 .out_rect.top = 0,
	 .out_rect.width = 1024,
	 .out_rect.height = 768,
	 },

	{
	 .subdev_type = CRL_SUBDEV_TYPE_BINNER,
	 .in_rect.left = 0,
	 .in_rect.top = 0,
	 .in_rect.width = 1024,
	 .in_rect.height = 768,
	 .out_rect.left = 0,
	 .out_rect.top = 0,
	 .out_rect.width = 1024,
	 .out_rect.height = 768,
	 },
};

static struct crl_mode_rep ti954_mopar_modes_1024x768[] = {
	{
	 .sd_rects_items = ARRAY_SIZE(ti954_mopar_1024x768_rects),
	 .sd_rects = ti954_mopar_1024x768_rects,
	 .binn_hor = 1,
	 .binn_vert = 1,
	 .scale_m = 1,
	 .width = 1024,
	 .height = 768,
	 },
};

static struct crl_sensor_limits ti954_mopar_sensor_limits_1024x768 = {
	.x_addr_min = 0,
	.y_addr_min = 0,
	.x_addr_max = 1024,
	.y_addr_max = 768,
	.min_frame_length_lines = 280,
	.max_frame_length_lines = 840,
	.min_line_length_pixels = 440,
	.max_line_length_pixels = 1905,
	.scaler_m_min = 1,
	.scaler_m_max = 1,
	.scaler_n_min = 1,
	.scaler_n_max = 1,
	.min_even_inc = 1,
	.max_even_inc = 1,
	.min_odd_inc = 1,
	.max_odd_inc = 1,
};

static int ds90ub954_hw_streamon(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int ret;

	struct crl_register_write_rep *regs = NULL;
	int len = 0;
	static struct crl_register_write_rep fwd_rin1_regs[] = {
		{0x0C, CRL_REG_LEN_08BIT, 0x97}, /* Enable RX-Port 1, set PASS_SEL, LOCK_SEL */
		{TI954_CSI_CTL, CRL_REG_LEN_08BIT, TI954_CSI_CTL_STREAMON},
		{TI954_FWD_CTL1, CRL_REG_LEN_08BIT, TI954_FWD_CTL1_PORT0_DIS},
		{TI954_INT_CTL, CRL_REG_LEN_08BIT, TI954_INT_CTL_ALL},	/* 954 global interrupt enable */
		{0, CRL_REG_LEN_DELAY, 66, 0}, /* wait 2x frame periods to avoid pink-screen */
	};

	static struct crl_register_write_rep fwd_rin0_regs[] = {
		{0x0C, CRL_REG_LEN_08BIT, 0x83}, /* Enable RX-Port 0, set PASS_SEL, LOCK_SEL */
		{TI954_CSI_CTL, CRL_REG_LEN_08BIT, TI954_CSI_CTL_STREAMON},
		{TI954_FWD_CTL1, CRL_REG_LEN_08BIT, TI954_FWD_CTL1_PORT1_DIS},
		{TI954_INT_CTL, CRL_REG_LEN_08BIT, TI954_INT_CTL_ALL},	/* 954 global interrupt enable */
		{0, CRL_REG_LEN_DELAY, 66, 0}, /* wait 2x frame periods to avoid pink-screen */
	};

	if (sensor_data->input_select == 1) {
		regs = fwd_rin1_regs;
		len = ARRAY_SIZE(fwd_rin1_regs);
#ifdef DEBUG_954_STREAM_ONOFF
		if ( sensor_data->rin1_streamon.numregs > 0){
			dev_info(&client->dev, "%s: using debug regset for rin1\n", __func__);
			regs = sensor_data->rin1_streamon.regs;
			len = sensor_data->rin1_streamon.numregs;
		} else {
			dev_info(&client->dev, "%s: using builtin regset for rin1\n", __func__);
		}
#endif
	} else if (sensor_data->input_select == 0) {
		regs = fwd_rin0_regs;
		len = ARRAY_SIZE(fwd_rin0_regs);
#ifdef DEBUG_954_STREAM_ONOFF
		if ( sensor_data->rin0_streamon.numregs > 0 ){
			dev_info(&client->dev, "%s: using debug regset for rin0\n", __func__);
			regs = sensor_data->rin0_streamon.regs;
			len = sensor_data->rin0_streamon.numregs;
		} else {
			dev_info(&client->dev, "%s: using builtin regset for rin0\n", __func__);
		}
#endif
	} else {
		dev_err(&client->dev,
			"%s: invalid input_select (should never get here!)\n",
			__func__);
		return -EINVAL;
	}

	ret = crlmodule_write_regs(sensor, regs, len);
	if (ret != 0) {
		dev_err(&client->dev, "%s: register write failed: %d\n",
			__func__, ret);
		return -EIO;
	}

	if ( ret == 0 ){
		set_hwstate(sensor_data, DS90UB954_HW_STATE_STREAMING);
	}
	dev_info(&client->dev, "%s: complete, ret=%d\n", __func__, ret);
	return ret;
}

static int ds90ub954_hw_streamoff(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;

	static struct crl_register_write_rep regs[] = {
		{TI954_FWD_CTL1, CRL_REG_LEN_08BIT, TI954_FWD_CTL1_DISABLE_ALL},
		{0, CRL_REG_LEN_DELAY, 33, 0},
		{TI954_CSI_CTL, CRL_REG_LEN_08BIT, TI954_CSI_CTL_STREAMOFF},
		{0, CRL_REG_LEN_DELAY, 10, 0},
	};
	
	int ret;

#ifdef DEBUG_954_STREAM_ONOFF
	if (sensor_data->streamoff_regs.numregs > 0){
		dev_info(&client->dev, "%s: using debug regset for streamoff\n", __func__);
		ret = crlmodule_write_regs(sensor, sensor_data->streamoff_regs.regs, 
			sensor_data->streamoff_regs.numregs);
	} else {
		dev_info(&client->dev, "%s: using builtin regset for streamoff\n", __func__);
		ret = crlmodule_write_regs(sensor, regs, ARRAY_SIZE(regs));
	}
#else
	ret = crlmodule_write_regs(sensor, regs, ARRAY_SIZE(regs));
#endif
	if ( ret == 0 ){
		set_hwstate(sensor_data, DS90UB954_HW_STATE_READY);
	} else {
		dev_err(&client->dev, "%s:%d unknown hw state detected\n",
				__func__, __LINE__);
		set_hwstate(sensor_data, DS90UB954_HW_STATE_UNKNOWN);
	}
	dev_info(&client->dev, "%s: complete, ret=%d\n", __func__, ret);
	return ret;
}

/***
 * SW Reset sequence for DS90UB954 chip.
 */
static int ds90ub954_sw_reset(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;

	dev_err(&client->dev, "%s reset started\n", __func__);
	set_hwstate(sensor_data, DS90UB954_HW_STATE_UNKNOWN);

	gpio_set_value(CAM_LVDS_RST, 0);
	usleep_range(2000, 5000); /* datasheet says 2ms, old code was 5.000 - 5.010 ms. */
	gpio_set_value(CAM_LVDS_RST, 1);
	usleep_range(5000, 5000 + 10); /* not sure if needed... */

	set_hwstate(sensor_data, DS90UB954_HW_STATE_RESET);
	dev_dbg(&client->dev, "%s: complete, ret=0\n", __func__);
	return 0;

}
/***
 * This does NOT enable CSI tx right away, but rather defers that to 
 * "stream on" operation.
 */
static int ds90ub954_hw_op_init(struct i2c_client *client){
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	u32 regval;
	int ret;
	unsigned long flags;
	int input_select, p0_bo, p1_bo;
	int p0_fpd, p1_fpd; // fpd-mode

	/* quirk to apply to prototype silicon only */
	struct crl_register_write_rep prototype_quirks[] = {
		{0xb0, CRL_REG_LEN_08BIT, 0x1c, 0x00},
		{0xb1, CRL_REG_LEN_08BIT, 0x15, 0x00},
		{0xb2, CRL_REG_LEN_08BIT, 0x30, 0x00},
		{0xb0, CRL_REG_LEN_08BIT, 0x1c, 0x00},
		{0xb1, CRL_REG_LEN_08BIT, 0x15, 0x00},
		{0xb2, CRL_REG_LEN_08BIT, 0x00, 0x00}
	};

	struct crl_register_write_rep irq_enable_regs[] = {
		{TI954_INT_CTL, CRL_REG_LEN_08BIT, TI954_INT_CTL_ALL, 0x00}
	};

	/* Get bit order & input select per port */
	spin_lock_irqsave(&sensor_data->slock, flags);
	input_select = sensor_data->input_select;
	p0_bo = sensor_data->port0_info.dg_bit_order;
	p1_bo = sensor_data->port1_info.dg_bit_order;
	p0_fpd = sensor_data->port0_info.fpd_mode;
	p1_fpd = sensor_data->port1_info.fpd_mode;
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	{ 
		/* scope to declare powerup reg values based on local-var parameters
		   that are acquired through function calls */
		struct crl_register_write_rep powerup_regset[] = {
			/* Enable Read RX1 port regs, write to both RX1 & RX0 */
			{TI954_REGSEL, CRL_REG_LEN_08BIT, TI954_REGSEL_W_ALL, 0x00},

			/* Set FV MIN TIME */
			{0xbc, CRL_REG_LEN_08BIT, 0x00, 0x00},

			/* Enable BOTH rx ports */
			{0x0c, CRL_REG_LEN_08BIT, 0x97, 0x00},

			/* Setup "reset detect" registers for later monitoring */
			{TI954_RESET_DETECT_REG0, CRL_REG_LEN_08BIT, TI954_RESET_DETECT_VAL0, 0},
			{TI954_RESET_DETECT_REG1, CRL_REG_LEN_08BIT, TI954_RESET_DETECT_VAL1, 0},
			{TI954_RESET_DETECT_REG2, CRL_REG_LEN_08BIT, TI954_RESET_DETECT_VAL2, 0},

			/* RAW10,  VC = 0 */
			{0x70, CRL_REG_LEN_08BIT, 0x1e, 0x00},

			/* RIN-0 Port specific regs */
			{TI954_REGSEL, CRL_REG_LEN_08BIT, TI954_REGSEL_RW_P0, 0x00},
			{0x5c, CRL_REG_LEN_08BIT, 0xba, 0x00}, /* ser_alias_id=0x5d  */
			{0x65, CRL_REG_LEN_08BIT, 0xbc, 0x00}, /* slaveAlias[0]=0x5e */
			{0x7c, CRL_REG_LEN_08BIT, p0_bo ? 0xb8 : 0xf8, 0x00},

			/* RIN-1 Port specific regs */
			{TI954_REGSEL, CRL_REG_LEN_08BIT, TI954_REGSEL_RW_P1, 0x00},
			{0x5c, CRL_REG_LEN_08BIT, 0xda, 0x00}, /* ser_alias_id=0x6d  */
			{0x65, CRL_REG_LEN_08BIT, 0xdc, 0x00}, /* slaveAlias[0]=0x6e */
			{0x7c, CRL_REG_LEN_08BIT, p1_bo ? 0xb8 : 0xf8, 0x00},

			/* Go back to writing BOTH ports */
			{TI954_REGSEL, CRL_REG_LEN_08BIT, TI954_REGSEL_W_ALL, 0x00},

			/* Error detection & PASS behavior
			 *  NOTE:  0xbf was arrived at w/ Alex Ried from HW-team.
			 *  Changed to '0x00' to prevent -momentary- loss of PASS
			 *  status on cameras that "glitch" upon changing zoom view.
			 */
			{0x7d, CRL_REG_LEN_08BIT, 0x00, 0x00},

			/* Disable All port forwarding */
			{TI954_FWD_CTL1, CRL_REG_LEN_08BIT, TI954_FWD_CTL1_DISABLE_ALL, 0x00},

			/* Enable CSI */
			/* {0x32, CRL_REG_LEN_08BIT, 0x01, 0x00}, <= reserved reg, but in old code?? */
			{0x34, CRL_REG_LEN_08BIT, 0x00, 0x00},
			{TI954_CSI_CTL, CRL_REG_LEN_08BIT, TI954_CSI_CTL_STREAMOFF, 0x00},

			/* Enable CSI Replicate */
			{0x21, CRL_REG_LEN_08BIT, 0x81, 0x00},

			/* Set FPD-Link mode AND backchannel settings based on 'fpd-mode' setting */
			{TI954_REGSEL, CRL_REG_LEN_08BIT, TI954_REGSEL_RW_P1, 0x00},
			{TI954_PORT_CONFIG, CRL_REG_LEN_08BIT, TI954_PORT_CONFIG_VAL(p1_fpd), 0x00},
			{TI954_BCC_CONFIG, CRL_REG_LEN_08BIT, TI954_BCC_CONFIG_VAL(p1_fpd), 0x00},
			{TI954_REGSEL, CRL_REG_LEN_08BIT, TI954_REGSEL_RW_P0, 0x00},
			{TI954_PORT_CONFIG, CRL_REG_LEN_08BIT, TI954_PORT_CONFIG_VAL(p0_fpd), 0x00},
			{TI954_BCC_CONFIG, CRL_REG_LEN_08BIT, TI954_BCC_CONFIG_VAL(p0_fpd), 0x00},
			{TI954_REGSEL, CRL_REG_LEN_08BIT, TI954_REGSEL_W_ALL, 0x00},

			/* Back Channel GPIO0 Select.Camera switch to 15FPS after
			 * updating the camera firmware*/
			{0x6E, CRL_REG_LEN_08BIT, 0x80, 0x00},

			/* Setup port-level ISR mask registers */
			{0xd8, CRL_REG_LEN_08BIT, 0x07, 0x00}, 
			{0xd9, CRL_REG_LEN_08BIT, 0x77, 0x00}, 

			/* Set slaveId[0]=0xbc for both RX ports */
			{0x5d, CRL_REG_LEN_08BIT, 0xbc, 0x00},

			/* Point bank-switching back at the expected "active" port */
			{TI954_REGSEL, CRL_REG_LEN_08BIT, 
				(input_select == 1 ? TI954_REGSEL_RW_P1 : TI954_REGSEL_RW_P0), 
				0x00},
		};


		ret =  crlmodule_write_regs(sensor, powerup_regset,
					   ARRAY_SIZE(powerup_regset));
	
	} /* end powerup reg's scope */

	/* Check silicon revision before appyling prototype-only errata */
	regval = 0x00;
	ds90ub954_i2c_read(client,0,0x03,&regval);
	if ((regval >> 4) < 0x2){ 
		dev_info(&client->dev, "Detected prototype silicon! Applying erata fixes\n");
		ret =  crlmodule_write_regs(sensor, prototype_quirks,
					   ARRAY_SIZE(prototype_quirks));
	}

	/* Write IRQ enable registers set */
	if (ret == 0) {
		ret =  crlmodule_write_regs(sensor, irq_enable_regs,
					   ARRAY_SIZE(irq_enable_regs));
	}

	/* Set hardware state to "ready" only upon successful initialization */
	if (ret == 0) {
		set_hwstate(sensor_data, DS90UB954_HW_STATE_READY);
	}

	dev_info(&client->dev, "%s: complete, ret=%d\n", __func__, ret);
	return ret;
}


/* For use by HW thread to set last known hardware state */
static void set_hwstate(struct crl_ds90ub954 * sensor_data, int new_state)
{
	unsigned long flags;
	spin_lock_irqsave(&sensor_data->slock, flags);
	sensor_data->hw_state = new_state;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	dev_dbg(&sensor_data->client->dev, "new hw-state: %d\n", new_state);
}

/* For use within or outside of hw thread to RETURN last known hardware state */
static int get_hwstate(struct crl_ds90ub954 * sensor_data)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&sensor_data->slock, flags);
	ret = sensor_data->hw_state;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	return ret;
}

static int set_miscreq(struct crl_ds90ub954 * sensor_data, int req)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&sensor_data->slock, flags);
	sensor_data->misc_request = req;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	return ret;
}

static int get_miscreq(struct crl_ds90ub954 * sensor_data)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&sensor_data->slock, flags);
	ret = sensor_data->misc_request;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	return ret;
}

static void get_active_port_status(struct crl_ds90ub954 *sensor_data, struct ti954_port_status *port)
{
	unsigned long flags;
	spin_lock_irqsave(&sensor_data->slock, flags);
	memcpy(port, sensor_data->active_port, sizeof(*port));
	spin_unlock_irqrestore(&sensor_data->slock, flags);
}


/* Get lock status to report to userspace.  This does NOT reflect "raw" locked
 * state, but rather takes into acct the overall hw state too... this prevents
 * FALSE reports of "locked" when HW is in bad/unknown state or in reset */
static int get_user_locked_status(struct crl_ds90ub954 * sensor_data)
{
	unsigned long flags;
	int status, hwstate;

	spin_lock_irqsave(&sensor_data->slock, flags);
	status = !!(sensor_data->active_port->sts1 & TI954_STS1_LOCK);
	hwstate = sensor_data->hw_state;
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	return !!( (status  == 1) && \
	           ((hwstate == DS90UB954_HW_STATE_READY || \
		    hwstate == DS90UB954_HW_STATE_STREAMING)));
}

/* Get lock status to report to userspace.  This does NOT reflect "raw" locked
 * state, but rather takes into acct the overall hw state too... this prevents
 * FALSE reports of "locked" when HW is in bad/unknown state or in reset */
static int get_user_pass_status(struct crl_ds90ub954 * sensor_data)
{
	unsigned long flags;
	int status, hwstate;

	spin_lock_irqsave(&sensor_data->slock, flags);
	status = !!(sensor_data->active_port->sts1 & TI954_STS1_PASS);
	hwstate = sensor_data->hw_state;
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	return !!( (status  == 1) && \
	           ((hwstate == DS90UB954_HW_STATE_READY || \
		    hwstate == DS90UB954_HW_STATE_STREAMING)));
}

/* Take a single 'pass' at attempting to configure hardware for a given target
 * state based on previous state + on failure handle retry behavior.  */
static int ds90ub954_configure_hw(struct i2c_client *client, int prevstate, int target)
{
	int ret = 0;
	switch(target){
	case DS90UB954_HW_STATE_RESET:
		ret = ds90ub954_sw_reset(client);
		break;
	case DS90UB954_HW_STATE_READY:
		if ( prevstate == DS90UB954_HW_STATE_STREAMING ){
			ret = ds90ub954_hw_streamoff(client);
		} else {
			if( 0 == (ret=ds90ub954_sw_reset(client))){
				ret = ds90ub954_hw_op_init(client);
			}
		}
		break;
	case DS90UB954_HW_STATE_STREAMING:
		if ( prevstate == DS90UB954_HW_STATE_READY ){
			ret = ds90ub954_hw_streamon(client);
		}
		else {
			if( 0 == (ret=ds90ub954_sw_reset(client))){
				if ( 0 == (ret = ds90ub954_hw_op_init(client))){
					ret = ds90ub954_hw_streamon(client);
				}
			}
		}
		break;
	default:
		ret = ds90ub954_sw_reset(client);
		break;
	}
	dev_info(&client->dev, "%s: prevstate=%d, target=%d, ret=%d\n", 
		__func__, prevstate, target, ret);
	return ret;
}


static void ds90ub954_periodic_service(struct i2c_client *client)
{
	/* Check to see if module in a bad state, 
	 *    setup for recovery if so.... */
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int hwstate = get_hwstate(sensor_data);
	static int err_cnt  = 0;
	int ii;
	struct {uint8_t reg; uint8_t val;} rv_pairs[] = {
		{TI954_RESET_DETECT_REG0, TI954_RESET_DETECT_VAL0},
		{TI954_RESET_DETECT_REG1, TI954_RESET_DETECT_VAL1},
		{TI954_RESET_DETECT_REG2, TI954_RESET_DETECT_VAL2},
	};

	dev_dbg(&client->dev, "%s hwstate=%d\n", __func__, hwstate);

	if(!(hwstate == DS90UB954_HW_STATE_READY || 
	     hwstate == DS90UB954_HW_STATE_STREAMING)){
		/* nothing to do if already in initializing  or unknown state .. */
		return; 
	}

	for (ii=0; ii < ARRAY_SIZE(rv_pairs); ++ii){
		uint32_t val; 
		int ret;
		if ( 0 != (ret = ds90ub954_i2c_read(client,0, rv_pairs[ii].reg, &val))){
			dev_warn(&client->dev, "%s: i2c read failed: %d\n", __func__, ret);
			err_cnt++;
		} else if ( val != rv_pairs[ii].val ){
			dev_warn(&client->dev, "%s: REGISTER RESET DETECTED (r=%x,ev=%x,v=%x)!\n", __func__,
				rv_pairs[ii].reg, rv_pairs[ii].val, val);
			dev_err(&client->dev, "%s:%d unknown hw state detected\n",
					__func__, __LINE__);
			set_hwstate(sensor_data, DS90UB954_HW_STATE_UNKNOWN);
		} else {
			err_cnt = 0; /* reset err count on succesful read */
		}
	}

	/* If we've been through this sanity check twice with no valid register
	 * reads, issue a hardware reset (by setting state=UNKNOWN) */
	if ( err_cnt >= 2 * ARRAY_SIZE(rv_pairs)) {
		dev_warn(&client->dev, 
			"%s: too many register read fails, reset device\n", __func__);
		dev_err(&client->dev, "%s:%d unknown hw state detected\n",
				__func__, __LINE__);
		set_hwstate(sensor_data, DS90UB954_HW_STATE_UNKNOWN);
	}

	return;
}

/***
 * Check if either IRQ work or requested state change is pending AND
 *  return values that caused waking
 */
static int __inline hw_work_pending (struct crl_ds90ub954 *sensor_data, int* req,
					int *irq, int* misc_req, int *suspend)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&sensor_data->slock, flags);
	*irq = sensor_data->irq_wake;
	*req = sensor_data->requested_hw_state;
	*misc_req = sensor_data->misc_request;
	*suspend  = sensor_data->should_suspend;
	sensor_data->irq_wake = 0;
	sensor_data->requested_hw_state = 0;
	sensor_data->misc_request = 0;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	ret =  (*irq || *req || *misc_req || *suspend);
	return ret;
}


/* Debug/test code for doing full register dump.
 * should only be called from hw_tast context. */
#if 0
static void ds90ub954_dump_regs(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int ret;

	struct i2c_msg msg[2];
	static uint8_t reg_addr[1] = {0x00}; /* address to dump from */
	static uint8_t reg_buffer[256]; 

	msg[0].addr = client->addr;
	msg[1].addr = client->addr;

	msg[0].flags = 0;
	msg[0].buf = reg_addr;
	msg[0].len = 1;

	msg[1].flags = I2C_M_RD;;
	msg[1].buf   = reg_buffer;
	msg[1].len   = sizeof(reg_buffer);

	ret = i2c_transfer(client->adapter, msg, 2);
	if ( ret < 0){
		dev_err(&client->dev, "i2c xfer error: %d\n", ret);
		return;
	}

	dev_info(&client->dev, "954-regdump : START\n");
	print_hex_dump(KERN_INFO, "954-regdump:", DUMP_PREFIX_OFFSET, 
		16, 1, reg_buffer, sizeof(reg_buffer), false);

	{
		int lc, ll;
		lc = ((uint16_t) reg_buffer[0x73] << 8) | (reg_buffer[0x74]);
		ll = ((uint16_t) reg_buffer[0x75] << 8) | (reg_buffer[0x76]);
		dev_info(&client->dev, 
			"954-regdump: sts1=%02x sts2=%02x lc=%d%s ll=%d%s%s fr=%dHz p=%d l=%d %s\n",
			reg_buffer[0x4d],
			reg_buffer[0x4e],
			lc,
			(reg_buffer[0x4e] & 1) ? "c" : "",
			ll,
			(reg_buffer[0x4e] & 0x40) ? "c" : "",
			(reg_buffer[0x4e] & 0x80) ? "u" : "",
			((0xff & reg_buffer[0x4f]) * 1000000) + ((0xff & reg_buffer[0x50]) * 1000000/256),
			(reg_buffer[0x4d] & 0x02),
			(reg_buffer[0x4d] & 0x01),
			((ll!=2560) || (lc!=800)) ? "GREP4ME" : "");
	}

	dev_info(&client->dev, "954-regdump : DONE\n");

	return;

}


static void do_regdump(struct i2c_client *client) {
	dev_info(&client->dev, "RX PORT-0 REGISTER DUMP\n");
	ds90ub954_i2c_write(client, 0, TI954_REGSEL, TI954_REGSEL_RW_P0);
	ds90ub954_dump_regs(client);

	dev_info(&client->dev, "RX PORT-1 REGISTER DUMP\n");
	ds90ub954_i2c_write(client, 0, TI954_REGSEL, TI954_REGSEL_RW_P1);
	ds90ub954_dump_regs(client);

}
#endif

static int ds90ub954_hw_task(void * client_vp)
{
	struct i2c_client *client = client_vp;
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;

	unsigned long wait_time = msecs_to_jiffies(1000); 
	int target_state      = DS90UB954_HW_STATE_READY;  /* initial target state */
	struct ti954_port_status last_port_info;
	struct ti954_port_status cur_port_info;

	// Local 'copies' of request& state variables
	int req_state;
	int irq_wake;
	int timeout;
	int hwstate;
	int suspend;
	int misc;

	// "clear" last port info.
	memset(&last_port_info, 0, sizeof(last_port_info));

	// initialize hw at thread startup
	ds90ub954_configure_hw(client, DS90UB954_HW_STATE_UNKNOWN, target_state);
	
	while ( !kthread_should_stop() ) {

		/* Wait until there is work for HW task to do.*/
		timeout = !wait_event_interruptible_timeout( 
				sensor_data->hw_wait, 
				hw_work_pending(sensor_data, &req_state, &irq_wake, &misc, &suspend),
				wait_time);

		if (!timeout){
			dev_dbg(&client->dev, "%s wakeup: req_state=%d, irq_wake=%d, misc=%d, suspend=%d, timeout=%d\n", 
				__func__, req_state, irq_wake, misc, suspend, timeout);
		}

		/* Check to see if HW thread should be suspended... */
		if (suspend) {
			/* On resume: 'configure_hw' will restore previous target hw state */ 
			set_hwstate(sensor_data, DS90UB954_HW_STATE_SUSPENDED);
			gpio_set_value(CAM_LVDS_RST, 0); /* put hw in reset state */

			/* Notify caller that this thread is about to suspend here (until
			 * resume is called and "should_suspend" flag clears */
			sensor_data->suspended = 1;
			wake_up_interruptible(&sensor_data->usr_wait);

			/* Wait until "should_supsend" status clears */
			wait_event_interruptible(sensor_data->hw_wait, 
				(sensor_data->should_suspend == 0));

			sensor_data->suspended = 0;
		}

#if 0
		if ( irq_wake ){
			do_regdump(client);
		}
#endif

		/*  Handle periodic work for a given state. 
		 *  Put this BEFORE state transitions in order to detect errors
		 *  + recover faster */
		ds90ub954_periodic_service(client);

		/* Next, check for a new state-change request */
		if (req_state && (req_state != target_state)) {
			target_state = req_state;
		}

		/* Any time target state != current state, try to change to the
		 * target state. */
		hwstate = get_hwstate(sensor_data);
		if (hwstate != target_state){
			if ( 0 != ds90ub954_configure_hw(client, hwstate, target_state)) {
				dev_warn(&client->dev, 
					"Failed to change DS90UB954 to target state: resetting!!!\n");
				ds90ub954_sw_reset(client);
			}
		}

		/* Handle "misc" requests... i.e. those that didn't fit into a
		 * state change request or IRQ processing */
		if ( misc == MISC_REQ_INPUT_SELECT ){
			/* Applying "stream_on" register changes is all that's
			 * needed for input_select change. Set state to unknown
			 * on error. If already in stream-off: no action to
			 * apply here.*/
			if (get_hwstate(sensor_data) == DS90UB954_HW_STATE_STREAMING){
				if ( 0 != ds90ub954_hw_streamon(client)){
					dev_err(&client->dev, "%s:%d unknown hw state detected\n",
							__func__, __LINE__);
					set_hwstate(sensor_data, DS90UB954_HW_STATE_UNKNOWN);
				}
			}
		} else if ( misc == MISC_REQ_BIT_ORDER ){
			ds90ub954_dg_bit_order(client);	
		} else if ( misc == MISC_REQ_P0_FPDMODE) {   
			ds90ub954_update_fpd_mode(client, 0);
		} else if ( misc == MISC_REQ_P1_FPDMODE) {   
			ds90ub954_update_fpd_mode(client, 1);
		} 
#if 0
		else if ( misc == MISC_REQ_REGISTER_DUMP ) {
			do_regdump(client);
		}
#endif

		/* Do IRQ handling on every wakeup since this does basic lock
		 * status check AND clears the HW registers that trigger
		 * interrupt processing. */
		ds90ub954_read_isr_status(client, 1);

		/* Wake user thread if needed to finish ioctl or sysfs request */
		if ( req_state ||  misc || suspend ) {
			dev_dbg(&client->dev, "%s: wake up usr_wait\n", __func__);
			wake_up_interruptible(&sensor_data->usr_wait);
		}

		/* Check for sysfs "push" notifications for userspace */
		get_active_port_status(sensor_data, &cur_port_info);
		if ( (cur_port_info.sts1 & TI954_STS1_LOCK) != 
		     (last_port_info.sts1 & TI954_STS1_LOCK))
		{
			dev_dbg(&client->dev, "notify sysfs change: lock=%d\n", 
				!!(cur_port_info.sts1 & TI954_STS1_LOCK));
			sysfs_notify(&client->dev.kobj, NULL, "lock");
		}

		if ( (cur_port_info.sts1 & TI954_STS1_PASS) != 
		     (last_port_info.sts1 & TI954_STS1_PASS))
		{
			dev_dbg(&client->dev, "notify sysfs change: pass=%d\n", 
				!!(cur_port_info.sts1 & TI954_STS1_PASS));
			sysfs_notify(&client->dev.kobj, NULL, "pass");
		}

		/* Store port info for next pass through loop / comparison. */
		memcpy(&last_port_info, &cur_port_info, sizeof(last_port_info));
	}
	return 0;
}

static int ds90ub954_do_hwmisc_req(struct crl_ds90ub954 *sensor_data, int misc_req){
	if(!mutex_lock_interruptible(&sensor_data->hw_mutex)){
		unsigned long wait_time = msecs_to_jiffies(1000); 

		/* Set misc request flag */
		set_miscreq(sensor_data, misc_req);
		wake_up_interruptible(&sensor_data->hw_wait);

		/* Wait for request completion */
		wait_event_interruptible_timeout(sensor_data->usr_wait, 
			get_miscreq(sensor_data) == MISC_REQ_NONE, 
			wait_time);

		mutex_unlock(&sensor_data->hw_mutex);
	}
	return 0;
}


static int ds90ub954_i2c_write(struct i2c_client *client, u16 i2c_addr,
			       u16 reg, u8 val)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);

	return crlmodule_write_reg(sensor, i2c_addr, reg, 1, 0xFF, val);
}

static int ds90ub954_i2c_read(struct i2c_client *client, u16 i2c_addr,
			      u16 reg, u32 * val)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_register_read_rep read_reg;

	read_reg.address = reg;
	read_reg.len = CRL_REG_LEN_08BIT;
	read_reg.dev_i2c_addr = i2c_addr;
	return crlmodule_read_reg(sensor, read_reg, val);
}

static ssize_t ds90ub954_dg_bit_order_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int bit_order;
	unsigned long flags;

	spin_lock_irqsave(&sensor_data->slock, flags);
	bit_order = sensor_data->active_port->dg_bit_order;
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bit_order);
}

static ssize_t ds90ub954_dg_bit_order_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	ssize_t res = count;
	int bit_order = -1;
	unsigned long flags;

	if (kstrtoint(buf, 0, &bit_order) || !(bit_order == 0 || bit_order == 1)) {
		dev_err(dev,
			"%s: setting dg_bit_order failed: must be 0 or 1 (got %d)\n",
			__func__, bit_order);
		return -ENXIO;
	}

	if( !mutex_lock_interruptible(&sensor->mutex)){
		spin_lock_irqsave(&sensor_data->slock, flags);
		if (bit_order != sensor_data->active_port->dg_bit_order) {
			sensor_data->active_port->dg_bit_order = bit_order;
			spin_unlock_irqrestore(&sensor_data->slock, flags);
			/* wait for HW thread to make this change take effect */
			ds90ub954_do_hwmisc_req(sensor_data, MISC_REQ_BIT_ORDER);
		} else {
			spin_unlock_irqrestore(&sensor_data->slock, flags);
		}
		mutex_unlock(&sensor->mutex);
	}

	return res;
}

static DEVICE_ATTR(dg_bit_order, S_IWUSR | S_IWGRP | S_IRUGO,
			ds90ub954_dg_bit_order_show,
			ds90ub954_dg_bit_order_store);

static ssize_t ds90ub954_input_select_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int port;
	unsigned long flags;

	spin_lock_irqsave(&sensor_data->slock, flags);
	port = sensor_data->input_select;
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	return scnprintf(buf, PAGE_SIZE, "%d\n", port);
}


static ssize_t ds90ub954_input_select_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	ssize_t res = count;
	int port = -1;

	if (kstrtoint(buf, 0, &port) || !(port == 0 || port == 1)) {
		dev_err(dev,
			"%s: setting input_select failed: must be 0 or 1 (got %d)\n",
			__func__, port);
		return -ENXIO;
	}

	if( !mutex_lock_interruptible(&sensor->mutex)){
		unsigned long flags;
		/* switch active port here from calling context... */
		spin_lock_irqsave(&sensor_data->slock, flags);
		if ( port != sensor_data->input_select) {
			sensor_data->input_change_count++;
			sensor_data->input_select = port;
			if (port == 0){
				sensor_data->active_port = &sensor_data->port0_info;
			} else {
				sensor_data->active_port = &sensor_data->port1_info;
			}
		}
		spin_unlock_irqrestore(&sensor_data->slock, flags);

		/* wait for HW thread to make this change take effect */
		ds90ub954_do_hwmisc_req(sensor_data, MISC_REQ_INPUT_SELECT);
		mutex_unlock(&sensor->mutex);
	}

	return res;
}

static DEVICE_ATTR(input_select, S_IWUSR | S_IWGRP | S_IRUGO,
		   ds90ub954_input_select_show,
		   ds90ub954_input_select_store);


static ssize_t ds90ub954_fpd_mode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int mode = -1;
	unsigned long flags;

	if (!strcmp(attr->attr.name, "fpd_mode")){
		spin_lock_irqsave(&sensor_data->slock, flags);
		mode = sensor_data->active_port->fpd_mode;
		spin_unlock_irqrestore(&sensor_data->slock, flags);
	} else if ( !strcmp(attr->attr.name, "port0_fpd_mode")){
		spin_lock_irqsave(&sensor_data->slock, flags);
		mode = sensor_data->port0_info.fpd_mode;
		spin_unlock_irqrestore(&sensor_data->slock, flags);
	} else if ( !strcmp(attr->attr.name, "port1_fpd_mode")){
		spin_lock_irqsave(&sensor_data->slock, flags);
		mode = sensor_data->port1_info.fpd_mode;
		spin_unlock_irqrestore(&sensor_data->slock, flags);
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", mode);
}

/* Store FPD-link mode, return 1 on change.  
 *   port:  pass in -1 (for active), 0, or 1.  If -1, will be updated
 *    on return.
 */
static int store_fpd_mode(struct crl_ds90ub954 *sensor_data, int mode, int* port)
{
	unsigned long flags;
	int changed = 0;
	struct ti954_port_status* pinfo = NULL;

	if ( *port == 0 ){
		pinfo = &sensor_data->port0_info;
	}else if( *port == 1){
		pinfo = &sensor_data->port1_info;
	}

	spin_lock_irqsave(&sensor_data->slock, flags);
		if ( !pinfo ) {
			pinfo = sensor_data->active_port; 
		}
		changed = (mode != pinfo->fpd_mode);
		pinfo->fpd_mode = mode;
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	if ( pinfo == &sensor_data->port0_info ){
		*port = 0;
	} else {
		*port = 1;
	}
	return changed;
}

static ssize_t ds90ub954_fpd_mode_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	ssize_t res = count;
	int mode = -1;
	int port = -1;


	if (kstrtoint(buf, 0, &mode) || !(mode == TI954_FPD_MODE_CSI || mode == TI954_FPD_MODE_RAW10)) {
		dev_err(dev,
			"%s: setting fpd_mode failed: must be 0 (csi) or 3 (raw10) (got %d)\n",
			__func__, port);
		return -ENXIO;
	}

	/* switch active port here from calling context... */
	if ( !strcmp(attr->attr.name, "port0_fpd_mode")){
		port = 0;
	} else if ( !strcmp(attr->attr.name, "port1_fpd_mode")){
		port = 1;
	} else {
		port = -1;
	}

	if( !mutex_lock_interruptible(&sensor->mutex)){
		/* wait for HW thread to make this change take effect */
		store_fpd_mode(sensor_data, mode, &port);
		if ( port == 0 ){
			ds90ub954_do_hwmisc_req(sensor_data, MISC_REQ_P0_FPDMODE);
		} else {
			ds90ub954_do_hwmisc_req(sensor_data, MISC_REQ_P1_FPDMODE);
		}
		mutex_unlock(&sensor->mutex);
	}
	return res;
}

static DEVICE_ATTR(fpd_mode, S_IWUSR | S_IWGRP | S_IRUGO,
                   ds90ub954_fpd_mode_show, ds90ub954_fpd_mode_store);
static DEVICE_ATTR(port0_fpd_mode, S_IWUSR | S_IWGRP | S_IRUGO, 
                   ds90ub954_fpd_mode_show, ds90ub954_fpd_mode_store);
static DEVICE_ATTR(port1_fpd_mode, S_IWUSR | S_IWGRP | S_IRUGO, 
                   ds90ub954_fpd_mode_show, ds90ub954_fpd_mode_store);

static ssize_t ds90ub954_camera_type_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int type;

	mutex_lock(&sensor->mutex);
	type = sensor_data->camera_type;
	mutex_unlock(&sensor->mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", type);
}

static ssize_t ds90ub954_camera_type_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	ssize_t res = count;
	int type = -1;

	if (kstrtoint(buf, 0, &type) || !(type == 1 || type == 2)) {
		dev_err(dev,
			"%s: setting camera_type failed: must be 1 or 2 (got %d)\n",
			__func__, type);
		return -ENXIO;
	}

	mutex_lock(&sensor->mutex);
	if (type != sensor_data->camera_type) {
		/* Prevent on-the-fly changes while streaming */
		if (sensor->streaming) {
			dev_info(dev,
				 "%s: cannot change camera type while streaming\n",
				 __func__);
			res = -ENXIO;
		} else {
			struct crl_mode_rep **modes = &sensor->sensor_ds->modes;
			struct crl_sensor_limits **limits = &sensor->sensor_ds->sensor_limits;
			unsigned int *modes_items = (unsigned int *)&sensor->sensor_ds->modes_items;
			sensor_data->camera_type = type;
			if (sensor_data->camera_type == 2) {
				*modes = ti954_mopar_modes_1024x768;
				*limits = &ti954_mopar_sensor_limits_1024x768;
				*modes_items = ARRAY_SIZE(ti954_mopar_modes_1024x768);
			} else {
				*modes = ti954_mopar_modes;
				*limits = &ti954_mopar_sensor_limits;
				*modes_items = ARRAY_SIZE(ti954_mopar_modes);
			}
			printk(KERN_ERR "sensor = %p modes = %p limits = %p", sensor, sensor->sensor_ds->modes, sensor->sensor_ds->sensor_limits);
			dev_dbg(dev, "%s: camera_type = new value (%d), change",
				__func__, type);
		}
	} else {
		dev_dbg(dev, "%s: camera_type == new value (%d), no change",
			__func__, type);
	}
	mutex_unlock(&sensor->mutex);

	if (res < 0)
		res = -ENXIO;
	return res;
}

static DEVICE_ATTR(camera_type, S_IWUSR | S_IWGRP | S_IRUGO,
		   ds90ub954_camera_type_show,
		   ds90ub954_camera_type_store);

static ssize_t ds90ub954_lock_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	ssize_t ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", get_user_locked_status(sensor_data));
	return ret;
}

static DEVICE_ATTR(lock, S_IRUGO, ds90ub954_lock_show, NULL);

static ssize_t ds90ub954_pass_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	ssize_t ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", get_user_pass_status(sensor_data));
	return ret;
}

static DEVICE_ATTR(pass, S_IRUGO, ds90ub954_pass_show, NULL);


static ssize_t ds90ub954_debug_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	char *optr = buf;
	char *eptr = buf + PAGE_SIZE;

	mutex_lock(&sensor->mutex);
	optr += scnprintf(optr, eptr - optr, "streamon_count=%d\n", sensor_data->streamon_count);
	optr += scnprintf(optr, eptr - optr, "streamoff_count=%d\n", sensor_data->streamoff_count);
	optr += scnprintf(optr, eptr - optr, "interrupt_count=%d\n", sensor_data->interrupt_count);
	optr += scnprintf(optr, eptr - optr, "input_change_count=%d\n", sensor_data->input_change_count);
	mutex_unlock(&sensor->mutex);
	return optr - buf;
}

static DEVICE_ATTR(debug, S_IRUGO, ds90ub954_debug_show, NULL);

static ssize_t ds90ub954_portX_status_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf, int port)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	char *optr = buf;
	char *eptr = buf + PAGE_SIZE;
	struct ti954_port_status status;
	struct ti954_port_status* source_ptr;
	unsigned long flags;
	
	if (port == 1) { 
		source_ptr = &sensor_data->port1_info;
	} else {
		source_ptr = &sensor_data->port0_info;
	}
	

	spin_lock_irqsave(&sensor_data->slock, flags);
	memcpy(&status, source_ptr, sizeof(status));
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	optr += scnprintf(optr, eptr - optr, "%u %u %u %u %u",
				status.sts1, 
				status.sts2, 
				status.rx_freq, 
				status.line_count,
				status.line_len);

	dev_dbg(dev, "port %d status: %s\n", port, buf);
	return optr - buf;
}

static ssize_t ds90ub954_port0_status_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	return ds90ub954_portX_status_show(dev, attr, buf, 0);
}

static ssize_t ds90ub954_port1_status_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	return ds90ub954_portX_status_show(dev, attr, buf, 1);
}

static DEVICE_ATTR(port0_status, S_IRUGO, ds90ub954_port0_status_show, NULL);
static DEVICE_ATTR(port1_status, S_IRUGO, ds90ub954_port1_status_show, NULL);

#ifdef DEBUG_954_STREAM_ONOFF
static size_t print_debug_regs(char* buf, size_t len, struct debug_regset *regs)
{
	size_t ii;
	char *optr = buf;
	char *eptr = buf + len;
	pr_info("%s: print %p (num regs=%d)\n", __func__, regs, (int) regs->numregs);
	for(ii=0; ii < regs->numregs; ++ii){
		if ( regs->regs[ii].len == CRL_REG_LEN_DELAY ){
			optr += scnprintf(optr, eptr - optr, "msleep %d\n", regs->regs[ii].val);
			pr_info("%s: msleep %d\n", __func__, regs->regs[ii].val);
		} else {
			optr += scnprintf(optr, eptr - optr, "%x %x\n", 
				regs->regs[ii].address, (0xFF & regs->regs[ii].val));
			pr_info("%s: %x %x\n", __func__, regs->regs[ii].address, 
				(0xFF & regs->regs[ii].val));
		}
	}
	return optr - buf;
}

static ssize_t ds90ub954_stream_ctrl_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	char *optr = buf;
	char *eptr = buf + PAGE_SIZE;

	if( !mutex_lock_interruptible(&sensor->mutex)){
		optr += scnprintf(optr, eptr - optr, "rin0: stream_on regs\n");
		optr += print_debug_regs(optr, eptr - optr, &sensor_data->rin0_streamon);
		optr += scnprintf(optr, eptr - optr, "rin1: stream_on regs\n");
		optr += print_debug_regs(optr, eptr - optr, &sensor_data->rin1_streamon);
		optr += scnprintf(optr, eptr - optr, "stream_off regs\n");
		optr += print_debug_regs(optr, eptr - optr, &sensor_data->streamoff_regs);
		mutex_unlock(&sensor->mutex);
	}
	return optr - buf;
}

static void set_dbg_reg(struct device * dev, struct debug_regset* regs, int idx, 
	const char* a_str, const char* v_str)
{
	int addr;
	int value;
	int len = CRL_REG_LEN_08BIT;

	if ( 0 != kstrtoint(v_str, 0, &value) || 0 != kstrtoint(a_str, 0, &addr)) {
		dev_info(dev, "%s: error parsing addr or val\n", __func__);
	}

	if ( !strcmp(a_str, "msleep") ){
		len = CRL_REG_LEN_DELAY;
	}

	if ( idx > regs->numregs  || idx > (sizeof(regs->regs)/sizeof(regs->regs[0]))) {
		dev_info(dev, "%s: out of bounds regsiter index!\n", __func__);
	} else {
		regs->regs[idx].val      = (uint32_t) value;
		regs->regs[idx].address  = (uint16_t) addr;
		regs->regs[idx].len      = (uint8_t)  len;
		dev_info(dev, "%s: %p[%d]: %x <%d> = %x\n", __func__, regs, idx, addr, len, value);
	}
}

static int process_regset_cmd(struct device *dev, struct crl_ds90ub954 *sensor_data, char* line)
{
	struct debug_regset *regs = NULL;
	char* cmd;
	char* id;

	/* Ignore comments and blanks */
	if ( !line || strlen(line) < 2 || *line == '#' ){
		return 0;
	}

	cmd  = strsep(&line, " \n");
	id  = strsep(&line, " \n");
	dev_info(dev, "%s: cmd=\"%s\"\n",  __func__, line);

	if (!cmd) cmd = "";
	if (!id)  id = "";
	/* Get register set */
	if ( !strcmp(id, "rin0")){
		regs = &sensor_data->rin0_streamon;
	}else if ( !strcmp(id, "rin1")) {
		regs = &sensor_data->rin1_streamon;
	} else if ( !strcmp(id, "off")) {
		regs = &sensor_data->streamoff_regs;
	} else {
		dev_info(dev, "%s: bad regset: \"%s\"\n", __func__, id);
		return -1;
	}

	/* Process clear / add / set command */
	if ( !strcmp(cmd, "clear") ){
		memset(regs->regs, 0, sizeof(regs->regs));
		regs->numregs = 0;
	} else if ( !strcmp(cmd, "add") ){
		char *a_str = strsep(&line, " ");
		char *v_str = strsep(&line, " ");
		if ( a_str && v_str && regs->numregs < DEBUG_MAX_REGS) {
			int idx = (int) regs->numregs;
			set_dbg_reg(dev, regs, idx, a_str, v_str);
			regs->numregs++;
		} else {
			dev_info(dev, "%s: bad args for \"%s\"\n", __func__, cmd);
			return -1;
		}
	} else if ( !strcmp(cmd, "set") ){
		char *i_str = strsep(&line, " ");
		char *a_str = strsep(&line, " ");
		char *v_str = strsep(&line, " ");
		if ( i_str && a_str && v_str )
		{
			int idx=0;
			if ( 0 == kstrtoint(i_str, 0, &idx) &&  
			     a_str && v_str && idx < regs->numregs) 
			{
				set_dbg_reg(dev, regs, idx, a_str, v_str);
			} else {
				dev_info(dev, "%s: bad set-cmd index\n", __func__);
				return -1;
			}
		} else {
			dev_info(dev, "%s: bad args for \"%s\"\n", __func__, cmd);
			return -1;
		}
	} else {
		dev_info(dev, "%s: bad command: \"%s\"\n", __func__, cmd);
		return -1;
	}
	return 0;
}



static ssize_t ds90ub954_stream_ctrl_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	ssize_t res = count;

	if( !mutex_lock_interruptible(&sensor->mutex)){
		/* Make a working copy of the string. use static to save stack... should be
		 * safe since we're holding lock. */
		static char in_str[PAGE_SIZE];
		char  *in_ptr, *line;
		int    parse_err = 0;
		int    line_no=0;
		strncpy(in_str, buf, PAGE_SIZE-1);
		in_str[PAGE_SIZE-1] = '\0'; /* "buf" should already have nul-term. this is just paranoia! */
		in_ptr = in_str;

		while (!parse_err && NULL != (line = strsep(&in_ptr, "\n"))) {
			parse_err = process_regset_cmd(dev, sensor_data, line);
			if ( parse_err != 0) dev_info(dev, "%s: parse err on line: %d\n", __func__, line_no);
			line_no++;
		}

		dev_info( dev, "%s: parsed %d lines in %d bytes)\n",
					__func__, line_no, (int) count);

		mutex_unlock(&sensor->mutex);
	}
	return res;
}

static DEVICE_ATTR(stream_ctrl, S_IWUSR | S_IWGRP | S_IRUGO,
			ds90ub954_stream_ctrl_show,
			ds90ub954_stream_ctrl_store);

#endif // DEBUG_954_STREAM_ONOFF

static struct attribute *ds90ub954_attributes[] = {
	&dev_attr_input_select.attr,
	&dev_attr_dg_bit_order.attr,
	&dev_attr_camera_type.attr,
	&dev_attr_pass.attr,
	&dev_attr_lock.attr,
	&dev_attr_debug.attr,
	&dev_attr_port0_status.attr,
	&dev_attr_port1_status.attr,
	&dev_attr_fpd_mode.attr, 
	&dev_attr_port0_fpd_mode.attr,
	&dev_attr_port1_fpd_mode.attr,
#ifdef DEBUG_954_STREAM_ONOFF
	&dev_attr_stream_ctrl.attr,
#endif
	NULL
};

static const struct attribute_group ds90ub954_attr_group = {
	.attrs = ds90ub954_attributes,
};


int ds90ub954_custom_init(struct i2c_client *client)
{
	int rval;
	struct crl_ds90ub954 *sensor_data;	/* sensor specific data */
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	extern unsigned char oem_camera_type;

	sensor_data = devm_kzalloc(&client->dev,
				   sizeof(*sensor_data), GFP_KERNEL);

	if (!sensor_data)
		return -ENOMEM;

	sensor_data->input_select = 1;	/* default to port 1 for RVC on FCA-R1 */
	sensor_data->active_port = &sensor_data->port1_info;

	/* initialize 'fpd mode' for RAW10 on both ports. */
	sensor_data->port0_info.fpd_mode = TI954_FPD_MODE_RAW10;
	sensor_data->port1_info.fpd_mode = TI954_FPD_MODE_RAW10;

	/* Initialize the module param that sets camera type */
	if (oem_camera_type == 1 || oem_camera_type == 2){
		sensor_data->camera_type  = oem_camera_type;
	} else {
		sensor_data->camera_type  = 1;
	}

	sensor->sensor_specific_data = sensor_data;

	if (devm_gpio_request_one(&client->dev, CAM_LVDS_RST, 0,
				  "CRL-954 PDB") != 0) {
		dev_err(&client->dev,
			"unable to acquire: CRL-954 PDB %d\n",
			CAM_LVDS_RST);
		return -ENODEV;
	}

	/* Extra locking + WQ's */
	spin_lock_init(&sensor_data->slock);
	mutex_init(&sensor_data->hw_mutex);
	init_waitqueue_head(&sensor_data->hw_wait);
	init_waitqueue_head(&sensor_data->usr_wait);

	/* setup pointers */
	sensor->sensor_specific_data = sensor_data;
	sensor_data->client = client;

	sensor_data->hw_task = kthread_run(ds90ub954_hw_task, client, "ds90ub954-hwtask");
	if (IS_ERR(sensor_data->hw_task)){
		sensor_data->hw_task = NULL;
		dev_err(&client->dev, "Failed to create hw_task!!!\n");
		return -ENOMEM;
	}

	rval =
	    sysfs_create_group(&client->dev.kobj, &ds90ub954_attr_group);
	if (rval) {
		dev_info(&client->dev, "sysfs_create_group failed: %d\n",
			 rval);
		return rval;
	} else {
		dev_dbg(&client->dev, "sysfs_create_group success!\n");
	}

	return 0;
}

int ds90ub954_cleanup(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);

	/* This can be NULL if crlmodule_registered call failed before
	 * sensor_init call.  */
	if (!sensor->sensor_specific_data)
		return 0;

	sysfs_remove_group(&client->dev.kobj, &ds90ub954_attr_group);
	return 0;
}


void ds90ub954_get_modes(struct i2c_client *client,
			 struct crl_mode_rep **modes,
			 unsigned int *modes_items)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	extern unsigned char oem_camera_type;
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;

	if (sensor_data->camera_type == 2) {
		*modes = ti954_mopar_modes_1024x768;
		*modes_items = ARRAY_SIZE(ti954_mopar_modes_1024x768);
	} else {
		*modes = sensor->sensor_ds->modes;
		*modes_items = sensor->sensor_ds->modes_items;
	}

}

struct crl_sensor_limits *ds90ub954_get_limits(struct i2c_client
						     *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	extern unsigned char oem_camera_type;
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;

	if (sensor_data->camera_type == 2) {
		return &ti954_mopar_sensor_limits_1024x768;
	} else {
		return sensor->sensor_ds->sensor_limits;
	}
}


/**  Read + store  RX port status registers.  */
static void ds90ub954_refresh_port_status(struct i2c_client *client, int port){
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;

	uint32_t regs1; /* for regs 0x4d - 0x50  (port sts & rx-freq) */
	uint32_t regs2; /* for regs 0x73 - 0x76  (line len/count)*/
	uint32_t savePortSwitch; /* save the port-switch register value (0x4C)*/
	struct crl_register_read_rep read_reg;
	int err = 0;
	unsigned long flags;
	struct ti954_port_status *info;

	if ( port == 1 ){
		info = &sensor_data->port1_info;
	} else {
		info = &sensor_data->port0_info;
	}

	/* save bank switching register, then change to target port */
	err |= ds90ub954_i2c_read(client, 0, TI954_REGSEL, &savePortSwitch);
	err |= ds90ub954_i2c_write(client, 0, TI954_REGSEL, 
		(port == 1 ? TI954_REGSEL_RW_P1 : TI954_REGSEL_RW_P0));

	memset(&read_reg, 0, sizeof(read_reg));
	read_reg.address = 0x4d;
	read_reg.len = CRL_REG_LEN_32BIT;
	err |= crlmodule_read_reg(sensor, read_reg, &regs1);

	memset(&read_reg, 0, sizeof(read_reg));
	read_reg.address = 0x73;
	read_reg.len = CRL_REG_LEN_32BIT;
	err |= crlmodule_read_reg(sensor, read_reg, &regs2);

	/* restore bank switching register */
	err |= ds90ub954_i2c_write(client, 0, TI954_REGSEL, savePortSwitch);

	/* On error: this will assume the worst - inform video unlocked
	   and chip needs reset. */
	if ( err ){
		dev_err(&client->dev, "Failed to refresh status regs!\n");
		regs1=0; regs2=0;
		dev_err(&client->dev, "%s:%d unknown hw state detected\n",
				__func__, __LINE__);
		set_hwstate(sensor_data, DS90UB954_HW_STATE_UNKNOWN);
	}

	spin_lock_irqsave(&sensor_data->slock, flags);
	/* Unpack regs 0x4d - 0x50 */
	info->sts1        = (0xffu & (regs1 >> 24));
	info->sts2        = (0xffu & (regs1 >> 16));
	info->rx_freq     = ((0xffu & (regs1 >> 8))*1000000 ) + ((0xFFu & regs1) * 1000000 / 256);
	/* Unpack regs 0x73 - 0x76 */
	info->line_count  = (0xffffu & (regs2 >> 16));
	info->line_len    = (0xffffu & regs2);
	spin_unlock_irqrestore(&sensor_data->slock, flags);
}

/* Read ISR status regs. Update internal state accordingly.
 * Must be called either from threaded irq or when irq is disabled. */
static int ds90ub954_read_isr_status(struct i2c_client *client,
				     int force_read)
{
	int ret;
	uint32_t interrupt_sts;

	if (0 !=
	    (ret = ds90ub954_i2c_read(client, 0, 0x24, &interrupt_sts))) {
		dev_err(&client->dev,
			"%s: failed to read INTERRUPT_STS:%d\n", __func__,
			ret);
		return ret;
	}

	if (interrupt_sts & 0x80)
		dev_dbg(&client->dev, "%s: global interrupt\n", __func__);
	if (interrupt_sts & 0x10)
		dev_dbg(&client->dev, "%s: CSI TX interrupt\n", __func__);
	if (interrupt_sts & 0x02)
		dev_dbg(&client->dev, "%s: RX-1 interrupt\n", __func__);
	if (interrupt_sts & 0x01)
		dev_dbg(&client->dev, "%s: RX-0 interrupt\n", __func__);

	/* Reading PORT_ISRx regs tell what triggered the interrupt, Reading
	 * PORT_STSx regs clear the ISR status for any 'edge triggered' events. */
	if ( (interrupt_sts & 0x01) || force_read) {
		ds90ub954_refresh_port_status(client,0);
	}
	if ( (interrupt_sts & 0x02) || force_read) {
		ds90ub954_refresh_port_status(client,1);
	}
	return 0;
}


/* Select dg_bit_order on the DS90UB954. */
static int ds90ub954_dg_bit_order(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int ret = 0;
	unsigned long flags;
	int bit_order;
	int port;

	spin_lock_irqsave(&sensor_data->slock, flags);
	bit_order = sensor_data->active_port->dg_bit_order;
	port      = sensor_data->input_select;
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	{
		struct crl_register_write_rep regs[] = {
			{ TI954_REGSEL, CRL_REG_LEN_08BIT, (port == 1 ? 
				TI954_REGSEL_RW_P1 : TI954_REGSEL_RW_P0)},

			{ 0x7C, CRL_REG_LEN_08BIT, (bit_order ? 
				0xb8 :	/* 8 bit format, Set FV, LV Polarity Higher 8 Bit Processing. */
				0xf8)	/* 8 bit format, Set FV, LV Polarity Lower 8 Bit Processing. */
			}
		};
		ret = crlmodule_write_regs(sensor, regs, ARRAY_SIZE(regs));
	};

	if (ret != 0) {
		dev_err(&client->dev, "%s: register write failed: %d\n",
			__func__, ret);
		return -EIO;
	}

	return ret;
}

/** Change fpd-link mode for specified port (in hardware) */
static void ds90ub954_update_fpd_mode(struct i2c_client *client, int port)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int ret = 0;
	unsigned long flags;
	int mode;

	spin_lock_irqsave(&sensor_data->slock, flags);
	if ( port == 0 ){
		mode = sensor_data->port0_info.fpd_mode;
	} else {
		mode = sensor_data->port1_info.fpd_mode;
	}
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	{
		struct crl_register_write_rep regs[] = {
			{ TI954_REGSEL, CRL_REG_LEN_08BIT, (port == 1 ? 
				TI954_REGSEL_RW_P1 : TI954_REGSEL_RW_P0)},
			{TI954_PORT_CONFIG, CRL_REG_LEN_08BIT, TI954_PORT_CONFIG_VAL(mode)},
			{TI954_BCC_CONFIG, CRL_REG_LEN_08BIT, TI954_BCC_CONFIG_VAL(mode)},
		};
		ret = crlmodule_write_regs(sensor, regs, ARRAY_SIZE(regs));

	}

	if ( ret != 0){
		dev_err(&client->dev, "%s: register write falied: %d\n", __func__, ret);
		set_hwstate(sensor_data, DS90UB954_HW_STATE_UNKNOWN);
	}
}

/***
 * Function used to request a new hardware state from hardware thread + optionally
 * wait for response/completion from userspace.
 */
int ds90ub954_request_hw_state(struct i2c_client *client, int state, int block)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	unsigned long flags;
	int state_change = 0;
	int new_state;
	static unsigned long wait_time = 0;

	dev_dbg(&client->dev, "%s: req=%d, block=%d\n", __func__, state, block);

	/* Calc wait time for handshake w/ HW thread.  Should not ever wait
	 * this long... but provides an "exit" point just incase something goes
	 * horribly wrong in the hw thread. */
	wait_time = msecs_to_jiffies(2000);

	if(!mutex_lock_interruptible(&sensor_data->hw_mutex)){
		spin_lock_irqsave(&sensor_data->slock, flags);
		if ( sensor_data->hw_state != state ) {
			sensor_data->requested_hw_state = state;
			state_change = 1;
		}
		spin_unlock_irqrestore(&sensor_data->slock, flags);

		if (state_change){
			dev_dbg(&client->dev, "wake hw thread\n");
			wake_up_interruptible(&sensor_data->hw_wait);

			if (block){
				wait_event_interruptible_timeout( sensor_data->usr_wait,
					((new_state = get_hwstate(sensor_data)) == state), 
					wait_time);

				dev_dbg(&client->dev, "usr wake: req state=%d, actual state=%d\n", 
					state, new_state);
			}
		}
		mutex_unlock(&sensor_data->hw_mutex);
	}

	if  (block && new_state != state) {
		return -EIO;
	}

	return 0;
}

int ds90ub954_streamon_callback(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int ret;

	dev_dbg(&client->dev, "enter %s\n", __func__);
	ret =  ds90ub954_request_hw_state(client, DS90UB954_HW_STATE_STREAMING, 1);
	if (ret != 0){
		dev_warn(&client->dev, "Failed or timed out starting streaming! (reset hw to ready state)\n");
		ds90ub954_request_hw_state(client, DS90UB954_HW_STATE_READY, 0);
	}
	dev_dbg(&client->dev, "exit %s\n", __func__);

	if (ret == 0){
		sensor_data->streamon_count++;
	}
	return ret;
}

int ds90ub954_streamoff_callback(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	int ret;
	dev_dbg(&client->dev, "enter %s\n", __func__);
	ret = ds90ub954_request_hw_state(client, DS90UB954_HW_STATE_READY, 1);
	dev_dbg(&client->dev, "exit %s\n", __func__);
	if ( ret == 0 ){
		sensor_data->streamoff_count++;
	}
	return ret;
}


int ds90ub954_suspend_callback(struct i2c_client* client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;

	sensor_data->should_suspend = 1;
	wake_up_interruptible(&sensor_data->hw_wait);
	wait_event_interruptible(sensor_data->usr_wait, (sensor_data->suspended));
	return 0;
}

int ds90ub954_resume_callback(struct i2c_client* client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;

	sensor_data->should_suspend = 0;
	wake_up_interruptible(&sensor_data->hw_wait);
	wait_event_interruptible(sensor_data->usr_wait, (!sensor_data->suspended));
	return 0;
}

irqreturn_t crl_ds90ub954_threaded_irq_fn(int irq, void *sensor_struct)
{
	struct crl_sensor *sensor = sensor_struct;
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);
	struct crl_ds90ub954 *sensor_data = sensor->sensor_specific_data;
	unsigned long flags;

	if (!sensor_data)
		goto error_exit;

	dev_dbg(&client->dev, "%s\n", __func__);
	sensor_data->interrupt_count++;

	/* Just wake the main 'hwtask' on IRQ events. */
	spin_lock_irqsave(&sensor_data->slock, flags);
	sensor_data->irq_wake = 1;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	wake_up_interruptible(&sensor_data->hw_wait);

error_exit:
	return IRQ_HANDLED;
}

