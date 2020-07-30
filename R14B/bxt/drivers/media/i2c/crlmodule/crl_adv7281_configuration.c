/*
 * crl_adv7281_configuration.c
 * Copyright (C) 2018 Harman International Ltd,
 *
 * Author: Brian McFarland <brian.mcfarland@harman.com>
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

#define DEBUG
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include "crlmodule.h"
#include "crlmodule-regs.h"
#include "crlmodule-sensor-ds.h"

#define DELAY_10MS 10
#define DELAY_25MS 25
#define DELAY_100MS 100
#define TEST_PATTERN	0

/* The ADV7281 has 3x "submaps" / register banks on the same I2C slave
 * address.  Register address 0x0e selects the active submap. */
#define ADV7281_REG_SUBEN (0x0e)
#define SUBEN_USER        (0x00)
#define SUBEN_INT_VDP     (0x20)
#define SUBEN_USER2       (0x40)

/****** USER SUBMAP REGISTERS ******/

#define ADV7281_REG_PWRCTL (0xf)
#define ADV7281_PWR_RESET_FLAG  (0x80)
#define ADV7281_PWR_PWRDWN_FLAG (0x40)

#define ADV7281_REG_INSEL (0x0)
#define INSEL_MASK        (0x1f)
#define INSEL_DIFF_AIN12  (0xe)	/* Analog rvc on FCA */
#define INSEL_DIFF_AIN34  (0xf)	/* Analog 'Japan' in on FCA */
#define INSEL_DEFAULT (INSEL_DIFF_AIN12)

#define ADV7281_REG_STATUS1 (0x10)
#define STATUS1_IN_LOCK     (0x01)

/* GPIO output bits (only used in reset sequence as a detection mechanism) */
#define ADV7281_REG_GPO     (0x59)

/* Short-to-battery diagnostic control */
#define ADV7281_REG_DIAG1_CTL (0x5d)
#define ADV7281_REG_DIAG2_CTL (0x5e)
#define DIAG_RSVD             (0x21)	/* Reserved bits that should be set = 1 */
#define DIAG_SLICE_LVL_975    (0x6 << 2)	/* Set 'trip' level for STB */
#define DIAG_SLICE_PWRDN      (0x40)	/* set to turn OFF */

/****** INTERRUPT / VDP SUBMAP REGISTERS ******/
#define ADV7281_REG_INTCFG   (0x40)
/* bits [1:0] Rovi int select */
#define INTCFG_OPENDRAIN     (0x00)
#define INTCFG_ACTIVE_LOW    (0x01)
#define INTCFG_ACTIVE_HIGH   (0x02)
/* bits [5:4] Rovi int select */
#define INTCFG_PSEUDOSYNC    (0x10)
/* bits [7:6] duration */
#define INTCFG_DUR_3XTAL     (0x00)
#define INTCFG_DUR_15XTAL    (0x40)
#define INTCFG_DUR_63XTAL    (0x80)
#define INTCFG_DUR_UNTIL_CLR (0xC0)

/* Interrupt status reg addrs & bit masks. In the Int/VDP I2C map. */
#define ADV7281_REG_INT_STS1    (0x42)
#define ADV7281_REG_INT_CLR1    (0x43)
#define ADV7281_REG_INT_MASK1   (0x44)
/* Status / clear / mask bit pos. */
#define INT1_SD_LOCK            (0x01)
#define INT1_SD_UNLOCK          (0x02)

#define ADV7281_REG_INT_STS5    (0x53)
#define ADV7281_REG_INT_CLR5    (0x54)
#define ADV7281_REG_INT_MASK5   (0x55)
#define INT5_DIAG_TRI1_L1       (0x02)	/* On FCA - 'rvc' STB status */
#define INT5_DIAG_TRI2_L1       (0x08)	/* On FCA - 'Japan' STB status */


/* Hardware states */
enum {
	/* Report this state internally when hardware is in an UNKNOWN state do
	 * to failed hardware operation.  Note: zero value ALSO used to
	 * indicate when there are no state change requests pending */
	ADV7281_HW_STATE_UNKNOWN = 0,  

	/* Hardware is known to be in reset state as a result of either hard or
	 * soft reset operation */
	ADV7281_HW_STATE_RESET,

	/* Hardware has been initialized and is READY for streaming, but not
	 * actively sending CSI packets to the SOC. */
	ADV7281_HW_STATE_READY,

	/* Video decoder HW is expected to be actively streaming something to
	 * SOC (barring excetpions for HW failure/unexpected behavior).  */
	ADV7281_HW_STATE_STREAMING,

	/* Used for suspend-to-ram / pm_suspend state */
	ADV7281_HW_STATE_SUSPENDED
};

enum {
	MISC_REQ_NONE = 0,      /* No request pending */
	MISC_REQ_INPUT_SELECT,  /* Request hw thread to update input_select */
	MISC_REQ_REGISTER_DUMP, /* Request hw thread to dump all registers when bus is free */
#if 0
	MISC_REQ_FORCE_RESET,   /* Request forcing a reset of the chip */
#endif
};

struct crl_adv7281 {
	/* HW Mutex used to synchronize access to functions that request a
	 * change in HW state or request to HW mgmt thread. */
	struct mutex hw_mutex;

	/* Spin-lock for protecting members of this struct */
	spinlock_t slock;

	/* Currently chosen video input (stored in terms of raw value written
	 * to INSEL register). */
	int input_select;

	/* Last known lock status. Read back 1 on lock, 0 when not locked */
	int lock_sts;

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
};

static int adv7281_update_lock_status(struct i2c_client *client);
static int adv7281_i2c_write(struct i2c_client *client, u16 reg, u8 val);
static int adv7281_i2c_read(struct i2c_client *client, u16 reg, u32 * val);
static int adv7281_sw_reset(struct i2c_client *client);
static int adv7281_hw_streamon(struct i2c_client *client);
static int adv7281_hw_streamoff(struct i2c_client *client);
static int adv7281_hw_op_init(struct i2c_client *client);
static int adv7281_enable_irq(struct i2c_client *client);

/* For use by HW thread to set last known hardware state */
static void set_hwstate(struct crl_adv7281 * sensor_data, int new_state)
{
	unsigned long flags;
	spin_lock_irqsave(&sensor_data->slock, flags);
	sensor_data->hw_state = new_state;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	dev_dbg(&sensor_data->client->dev, "new hw-state: %d\n", new_state);
}

/* For use within or outside of hw thread to RETURN last known hardware state */
static int get_hwstate(struct crl_adv7281 * sensor_data)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&sensor_data->slock, flags);
	ret = sensor_data->hw_state;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	return ret;
}

static int set_miscreq(struct crl_adv7281 * sensor_data, int req)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&sensor_data->slock, flags);
	sensor_data->misc_request = req;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	return ret;
}

static int get_miscreq(struct crl_adv7281 * sensor_data)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&sensor_data->slock, flags);
	ret = sensor_data->misc_request;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	return ret;
}

/* Get lock status to report to userspace.  This does NOT reflect "raw" locked
 * state, but rather takes into acct the overall hw state too... this prevents
 * FALSE reports of "locked" when HW is in bad/unknown state or in reset */
static int get_user_locked_status(struct crl_adv7281 * sensor_data)
{
	unsigned long flags;
	int status, hwstate;

	spin_lock_irqsave(&sensor_data->slock, flags);
	status = sensor_data->lock_sts;
	hwstate = sensor_data->hw_state;
	spin_unlock_irqrestore(&sensor_data->slock, flags);

	return !!( (status  == 1) && \
	           ((hwstate == ADV7281_HW_STATE_READY || \
		    hwstate == ADV7281_HW_STATE_STREAMING)));
}

static int adv7281_hw_streamon(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	static struct crl_register_write_rep regs[] = {
		//{0x00, CRL_REG_LEN_08BIT, 0x80, 0x88},  /* CSI Tx Off */
		//{0x00, CRL_REG_LEN_DELAY, DELAY_100MS}, /* 100ms delay */
		{0x00, CRL_REG_LEN_08BIT, 0x00, 0x88},  /* CSI Tx On */
		{0x00, CRL_REG_LEN_DELAY, DELAY_10MS}, /* 10 ms*/
	};
	int ret;

	ret = crlmodule_write_regs(sensor, regs, ARRAY_SIZE(regs));
	if ( ret == 0 ){
		set_hwstate(sensor_data, ADV7281_HW_STATE_STREAMING);
	}
	dev_dbg(&client->dev, "%s: ret=%d\n", __func__, ret);
	return ret;
}

static int adv7281_hw_streamoff(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	static struct crl_register_write_rep regs[] = {
		{0x00, CRL_REG_LEN_08BIT, 0x80, 0x88}, /* CSI Tx Off */
	};
	int ret;
	ret = crlmodule_write_regs(sensor, regs, ARRAY_SIZE(regs));
	if ( ret == 0 ){
		set_hwstate(sensor_data, ADV7281_HW_STATE_READY);
	}
	dev_dbg(&client->dev, "%s: ret=%d\n", __func__, ret);
	return ret;
}


/***
 * SW Reset sequence for ADV7281 chip.
 */
static int adv7281_sw_reset(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	int ret; 
	uint32_t val;
	int read_retries = 3;

	dev_dbg(&client->dev, "%s: entry\n", __func__);

	/* This sequence may leave the HW in an unknown state... 
	 * keep track of that here accordingly.  */
	set_hwstate(sensor_data, ADV7281_HW_STATE_UNKNOWN);

	/* Make sure we're on the right register submap */
	ret = adv7281_i2c_write(client, ADV7281_REG_SUBEN, SUBEN_USER);
	if ( ret != 0){
		dev_err(&client->dev, "%s: Failed to set submap\n", __func__);
		return ret;
	}

	/* SET GPO pin control register... this is to check if reset is 
	 * successful (which clears the register) since writing the power
	 * control register causes a failure on i2c bus. */
	ret = adv7281_i2c_write(client, 0x59, 0x7);
	if ( ret != 0){
		dev_err(&client->dev, "%s: Failed to set GPO marker bits\n", __func__);
		return ret;
	}

	/* Issue reset command: An I2C NAK is EXPECTED here. This will return
	 * error code -EAGAIN or -EREMOTEIO. If a different error number is received,
	 * abort... otherwise ignore those errors. */
	ret = adv7281_i2c_write(client, ADV7281_REG_PWRCTL, ADV7281_PWR_RESET_FLAG);
	if ( ret != 0){
		if ( ret != -EREMOTEIO  && ret != -EAGAIN ){
			dev_warn(&client->dev, 
				"%s: Unexpected error code: %d\n", __func__, ret);
			return ret;
		}
	}

	/* Try to read back a different register for up to 10ms time.
	 * Datasheet says reset should be done within 2ms, but ADV FAE
	 * support recommends up to 5ms for safety margin. Doubling that to
	 * 10ms as extra precaution */
	do {
		udelay(3000);
		ret = adv7281_i2c_read(client, 0x59, &val);
	}while( ret && read_retries--);

	/* If GPO bits come back zero, take that as a sign that reset was
	 * succesful. */
	if (ret == 0 && val == 0) {
		set_hwstate(sensor_data, ADV7281_HW_STATE_RESET);
	} else {
		dev_err(&client->dev, "%s: SW RESET FAILED!\n", __func__);
	}
	return ret;
}

/***
 * HW init sequence that has been officially "approved" by AD with 1x minor
 * modification:
 *  This does NOT enable CSI tx right away, but rather defers that to 
 *  "stream on" operation.
 */
static int adv7281_hw_op_init(struct i2c_client *client){
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	struct crl_register_write_rep adv7281_cvbs_powerup_regset[] = {
		{0x0f, CRL_REG_LEN_08BIT, 0x00},       /* Exit power down mode */
		{0x52, CRL_REG_LEN_08BIT, 0xc0},       /* Diff_CVBS AFE IBIAS  */
		{0x00, CRL_REG_LEN_08BIT, 0x10},       /* INSEL =unconnected input [INSEL Switch] */
		{0x00, CRL_REG_LEN_08BIT, sensor_data->input_select}, /* CVBS_P in on Ain 1, CVBS_N in on Ain2 */
		{0x0e, CRL_REG_LEN_08BIT, 0x80},       /* ADI Required Write  */
		{0x9c, CRL_REG_LEN_08BIT, 0x00},       /* Reset Current Clamp Circuitry [step1] */
		{0x9c, CRL_REG_LEN_08BIT, 0xff},       /* Reset Current Clamp Circuitry [step2] */
		{0x0e, CRL_REG_LEN_08BIT, 0x00},       /* Enter User Sub Map */
		{0x5a, CRL_REG_LEN_08BIT, 0x90},       /* ADI Required Write [common mode clamp setup] */
		{0x60, CRL_REG_LEN_08BIT, 0xa0},       /* ADI Required Write [common mode clamp setup] */
		{0x00, CRL_REG_LEN_DELAY, DELAY_25MS}, /* Force common mode clamps on for 25 ms */
		{0x60, CRL_REG_LEN_08BIT, 0xb0},       /* ADI Required Writes [common mode clamp setup] */
		{0x5f, CRL_REG_LEN_08BIT, 0xa8},       /* SHA gain for Div4 */
		{0x0e, CRL_REG_LEN_08BIT, 0x80},       /* ADI Required Writes */
		{0xb6, CRL_REG_LEN_08BIT, 0x08},       /* ADI Required Writes [differential CVBS required write] */
		{0xc0, CRL_REG_LEN_08BIT, 0xa0},       /* ADI Required Writes [differential CVBS required write] */
		{0x0e, CRL_REG_LEN_08BIT, 0x00},       /* Enter User Map  */
		{0x80, CRL_REG_LEN_08BIT, 0x51},       /* ADI Required Write */
		{0x81, CRL_REG_LEN_08BIT, 0x51},       /* ADI Required Write */
		{0x82, CRL_REG_LEN_08BIT, 0x68},       /* ADI Required Write */
		{0x17, CRL_REG_LEN_08BIT, 0x41},       /* Enable SH1 */
		{0x03, CRL_REG_LEN_08BIT, 0x4e},       /* Power down unused pads */
                {0x04, CRL_REG_LEN_08BIT, 0xD7},       /* Changing output control to emit Active video   */
		{0x13, CRL_REG_LEN_08BIT, 0x00},       /* Enable ADV7281M for 28_63636MHz crystal */
		{0x1d, CRL_REG_LEN_08BIT, 0xc0},       /* Tri-State LLC output driver */
		{0xfe, CRL_REG_LEN_08BIT, 0x88},       /* Set CSI Map Address */
	};

	struct crl_register_write_rep adv7281_csi_init_regset[] = {
		{0xde, CRL_REG_LEN_08BIT, 0x02, 0x88}, /* Power up MIPI D-PHY */
		{0xd2, CRL_REG_LEN_08BIT, 0xf7, 0x88}, /* ADI Required Write */
		{0xd8, CRL_REG_LEN_08BIT, 0x65, 0x88}, /* ADI Required Write */
		{0xe0, CRL_REG_LEN_08BIT, 0x09, 0x88}, /* ADI Required Write */
		{0x2c, CRL_REG_LEN_08BIT, 0x00, 0x88}, /* ADI Required Write */
#if 0
		/* Don't do this until streamon... allows avoiding extra delay! */
		// {0x00, CRL_REG_LEN_08BIT, 0x00, 0x88}, /* Power up MIPI CSI-2 Tx [All ADV7281-M writes complete] */
#endif
	};

	int ret;
	ret =  crlmodule_write_regs(sensor, adv7281_cvbs_powerup_regset,
				   ARRAY_SIZE(adv7281_cvbs_powerup_regset));

	/* Write the CSI configuration registers with retries. This seems to fail often 
	 * on the first attempt to configure it for unexplained reasons. Discovered at 
	 * first during vehicle testing. ADI never gave a clear answer why it
	 * happens but retrying seems to correct it. */
	if (ret == 0){
		int retries = 3;
		while(!ret && retries--){
			ret =  crlmodule_write_regs(sensor, adv7281_csi_init_regset,
						   ARRAY_SIZE(adv7281_csi_init_regset));
			if (ret && retries){
				msleep(10);
				dev_dbg(&client->dev, "retrying csi register init (ret=%d)\n", ret);
			}
		}

	}

	/* Write IRQ enable registes *before* CSI register set */
	if (ret == 0) {
		ret = adv7281_enable_irq(client);
	}


	/* Set hardware state to "ready" only upon successful initialization */
	if (ret == 0) {
		set_hwstate(sensor_data, ADV7281_HW_STATE_READY);
	}
	return ret;
}


static int adv7281_enable_irq(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	static struct crl_register_write_rep enable_irq_regs[] = {
		/* Select user submap */
		{ADV7281_REG_SUBEN, CRL_REG_LEN_08BIT, SUBEN_USER},
		/* Enable "diag slice" circuit for short-to-battery detection */
		{ADV7281_REG_DIAG1_CTL, CRL_REG_LEN_08BIT, DIAG_RSVD |
		 DIAG_SLICE_LVL_975},
		{ADV7281_REG_DIAG2_CTL, CRL_REG_LEN_08BIT, DIAG_RSVD |
		 DIAG_SLICE_LVL_975},
		/* Select interrupt submap */
		{ADV7281_REG_SUBEN, CRL_REG_LEN_08BIT, SUBEN_INT_VDP},
		/* Set interrupt masks */
		{ADV7281_REG_INT_MASK1, CRL_REG_LEN_08BIT, INT1_SD_LOCK |
		 INT1_SD_UNLOCK},
		/* clear interrupts */
		{ADV7281_REG_INT_CLR1, CRL_REG_LEN_08BIT, INT1_SD_LOCK |
		 INT1_SD_UNLOCK},
		/* INTRQ Pin configuration */
		{ADV7281_REG_INTCFG, CRL_REG_LEN_08BIT, INTCFG_ACTIVE_LOW |
		 INTCFG_PSEUDOSYNC | INTCFG_DUR_UNTIL_CLR},
		{ADV7281_REG_SUBEN, CRL_REG_LEN_08BIT, SUBEN_USER},
	};
	int ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	ret = crlmodule_write_regs(sensor, enable_irq_regs,
				   ARRAY_SIZE(enable_irq_regs));
	if (ret != 0) {
		dev_err(&client->dev, "%s: failed to write regs\n",
			__func__);
		return ret;
	}

	/* Checking lock status once seems necessary to  "arm" the interrupt
	 * hardware for lock status IRQs */
	return adv7281_update_lock_status(client);
}

int adv7281_handle_irq(struct i2c_client* client)
{
	int ret;
	uint32_t irqsts1;

	/* Activate Interrupt/VDP submap */
	if (0 != (ret = adv7281_i2c_write(client, 0x0e, 0x20))) {
		dev_err(&client->dev,
			"%s: failed to set submap: %d\n", __func__, ret);
		goto error_exit;
	}

	/* Read status1 register for lock */
	if (0 != (ret = adv7281_i2c_read(client, ADV7281_REG_INT_STS1,
					 &irqsts1))) {
		dev_err(&client->dev,
			"%s: failed to read irq status: %d\n", __func__, ret);
		goto error_exit;
	}

	//if (irqsts1 & (INT1_SD_LOCK | INT1_SD_UNLOCK)) {
		adv7281_update_lock_status(client);
	//}

	/* Clear the lock status IRQs */
	if (0 !=
	    (ret =
	     adv7281_i2c_write(client, ADV7281_REG_INT_CLR1, irqsts1))) {
		goto error_exit;
	}
error_exit:
	/* Activate User submap */
	if (0 != (ret = adv7281_i2c_write(client, 0x0e, 0x00))) {
		dev_err(&client->dev,
			"%s: failed to set submap: %d\n", __func__, ret);
	}

	return ret;
}

/* Check status of the hw to check for reset, pwrdwn or other 
 * generally "weird" states */
static int adv7821_periodic_service(struct i2c_client *client)
{
	int ret;
	uint32_t val; 
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	int hwstate = get_hwstate(sensor_data);

	//dev_dbg(&client->dev, "%s hwstate=%d\n", __func__, hwstate);

	if(!(hwstate == ADV7281_HW_STATE_READY || 
	     hwstate == ADV7281_HW_STATE_STREAMING)){
		/* nothing to do here.. */
		return 0;
	}

	/* Set User submap */
	if (0 != (ret = adv7281_i2c_write(client, 0x0e, 0x00))) {
		dev_err(&client->dev,
			"%s: failed to set submap: %d\n", __func__, ret);
		return -1;
	}

	/* Check to make sure pwrdwn bit is NOT set */
	if (0 != (ret = adv7281_i2c_read(client, ADV7281_REG_PWRCTL, &val))) {
		return -1;
	}

	if ( (val &  ADV7281_PWR_PWRDWN_FLAG) ){
		dev_err(&client->dev, "%s: Device in PWRDWN!\n", __func__);
		set_hwstate(sensor_data, ADV7281_HW_STATE_UNKNOWN);
		return -1;
	}
	return 0;

}

/* Take a single 'pass' at attempting to configure hardware for a given target
 * state based on previous state + on failure handle retry behavior.  */
static int adv7281_configure_hw(struct i2c_client *client, int prevstate, int target)
{
	int ret = 0;
	switch(target){
	case ADV7281_HW_STATE_RESET:
		ret = adv7281_sw_reset(client);
		break;
	case ADV7281_HW_STATE_READY:
		if ( prevstate == ADV7281_HW_STATE_STREAMING ){
			ret = adv7281_hw_streamoff(client);
		} else {
			if( 0 == (ret=adv7281_sw_reset(client))){
				ret = adv7281_hw_op_init(client);
			}
		}
		break;
	case ADV7281_HW_STATE_STREAMING:
		if ( prevstate == ADV7281_HW_STATE_READY ){
			ret = adv7281_hw_streamon(client);
		}
		else {
			if( 0 == (ret=adv7281_sw_reset(client))){
				if ( 0 == (ret = adv7281_hw_op_init(client))){
					ret = adv7281_hw_streamon(client);
				}
			}
		}
		break;
	default:
		ret = adv7281_sw_reset(client);
		break;
	}
	return ret;
}

/***
 * Dump registers to dmesg log
 */
static void adv7281_dump_regs(struct i2c_client *client, int req_type)
{
	int idx;

	// addresses to dump for user submap
	uint8_t user_regs [] = {
		0x0, 0x1, 0x2, 0x3, 0x4, 0x7, 0xf, 0x10, 0x11, 0x12, 0x13, 0xfd, 0xfe
	};

	for ( idx=0; idx < sizeof(user_regs); ++idx){
		uint32_t val;
		int ret;
		ret = adv7281_i2c_read(client, user_regs[idx], &val);
		dev_info(&client->dev, "%s: User Submap 0x%x=%02x [ret=%d]\n", 
			__func__, user_regs[idx], val, ret);
	}

}

/***
 * Check if either IRQ work or requested state change is pending AND
 *  return values that caused waking
 */
static int __inline hw_work_pending (struct crl_adv7281 *sensor_data, int* req,
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


static int adv7281_hw_task(void * client_vp)
{
	struct i2c_client *client = client_vp;
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;

	unsigned long wait_time = msecs_to_jiffies(1000); 
	int target_state      = ADV7281_HW_STATE_READY;  /* initial target state */
	int last_lock_status  = -1;

	// Local 'copies' of request& state variables
	int req_state;
	int irq_wake;
	int timeout;
	int hwstate;
	int suspend;
	int lock_status;
	int misc;

	// initialize hw at thread startup
	adv7281_configure_hw(client, ADV7281_HW_STATE_UNKNOWN, target_state);

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
			set_hwstate(sensor_data, ADV7281_HW_STATE_SUSPENDED);

			/* Notify caller that this thread is about to suspend here (until
			 * resume is called and "should_suspend" flag clears */
			sensor_data->suspended = 1;

			wake_up_interruptible(&sensor_data->usr_wait);

			/* Wait until should_supsend clears */
			wait_event_interruptible(sensor_data->hw_wait, 
				(sensor_data->should_suspend == 0));

			sensor_data->suspended = 0;
		}

		/* Handle "misc" requests... i.e. those that didn't fit into a
		 * state change request or IRQ processing */
		if (misc == MISC_REQ_REGISTER_DUMP){
			adv7281_dump_regs(client, misc);
		} else if ( misc == MISC_REQ_INPUT_SELECT ){
			/* Try to set register "inline", on failure reset */
			if ( 0 != adv7281_i2c_write(client, ADV7281_REG_INSEL,
				sensor_data->input_select)){
				adv7281_sw_reset(client);
			}
		}

		hwstate = get_hwstate(sensor_data);

		/* Check first for a new state-change request */
		if (req_state && (req_state != target_state)) {
			target_state = req_state;
		}

		/* Any time target state != current state, try to change to the
		 * target state. */
		if (hwstate != target_state){
			if ( 0 != adv7281_configure_hw(client, hwstate, target_state)) {
				dev_warn(&client->dev, 
					"Failed to change ADV7281 to target state: resetting!!!\n");
				adv7281_sw_reset(client);
			}
		}

		/*  Handle periodic work for a given state */
		adv7821_periodic_service(client);

		/* Handle IRQ wakeup request */
		if (irq_wake){
			/* Do IRQ handling on every wakeup since this does basic lock
			 * status check AND clears the HW registers that trigger
			 * interrupt processing. */
			adv7281_handle_irq(client);
		}


		if ( req_state ||  misc || suspend ) {
			dev_dbg(&client->dev, "%s: wake up usr_wait\n", __func__);
			wake_up_interruptible(&sensor_data->usr_wait);
		}

		/* Report lock status */
		lock_status = get_user_locked_status(sensor_data);
		if ( lock_status != last_lock_status ){
			sysfs_notify(&client->dev.kobj, NULL, "lock");
			last_lock_status = lock_status;
		}
	}
	return 0;
}

/* Write a single register */
static int adv7281_i2c_write(struct i2c_client *client, u16 reg, u8 val)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);

	return crlmodule_write_reg(sensor, CRL_I2C_ADDRESS_NO_OVERRIDE,
				   reg, CRL_REG_LEN_08BIT, 0xFF, val);
}

/* Read a single register */
static int adv7281_i2c_read(struct i2c_client *client, u16 reg, u32 * val)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_register_read_rep read_reg;

	read_reg.address = reg;
	read_reg.len = CRL_REG_LEN_08BIT;
	read_reg.dev_i2c_addr = CRL_I2C_ADDRESS_NO_OVERRIDE;
	return crlmodule_read_reg(sensor, read_reg, val);
}

static int adv7281_insel_valid(int val)
{
	/* for now, only support modes used on FCA HW */
	return (val == INSEL_DIFF_AIN12 || val == INSEL_DIFF_AIN34);
}

static ssize_t adv7281_input_select_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	int insel = -1;

	if( !mutex_lock_interruptible(&sensor->mutex) ){
		insel = sensor_data->input_select;
		mutex_unlock(&sensor->mutex);
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", insel);
}

static ssize_t adv7281_input_select_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	ssize_t res = count;
	int insel = -1;

	if (kstrtoint(buf, 0, &insel) || !adv7281_insel_valid(insel)) {
		dev_err(dev, "%s: invalid input_select: %d\n",
			__func__, insel);
		return -ENXIO;
	}

	if( !mutex_lock_interruptible(&sensor->mutex)){
		if (insel != sensor_data->input_select) {
			sensor_data->input_select = insel;
			if(!mutex_lock_interruptible(&sensor_data->hw_mutex)){
				unsigned long wait_time = msecs_to_jiffies(1000); 

				/* Set misc request flag */
				set_miscreq(sensor_data, MISC_REQ_INPUT_SELECT);
				wake_up_interruptible(&sensor_data->hw_wait);

				/* Wait for request completion */
				wait_event_interruptible_timeout(sensor_data->usr_wait, 
					get_miscreq(sensor_data) == MISC_REQ_NONE, 
					wait_time);

				/* Unlock... */
				mutex_unlock(&sensor_data->hw_mutex);
			}
		}
		mutex_unlock(&sensor->mutex);
	}

	if (res)
		res = -ENXIO;
	return count;
}

static DEVICE_ATTR(input_select, S_IWUSR | S_IWGRP | S_IRUGO,
		   adv7281_input_select_show, adv7281_input_select_store);


static ssize_t adv7281_dump_regs_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;

	static unsigned long wait_time = 0;

	/* Calc wait time for handshake w/ HW thread.  Should not ever wait
	 * this long... but provides an "exit" point just incase something goes
	 * horribly wrong in the hw thread. */
	if (!wait_time){
		wait_time = msecs_to_jiffies(2000);
	}

	if(!mutex_lock_interruptible(&sensor_data->hw_mutex)){
		// Set misc request flag
		set_miscreq(sensor_data, MISC_REQ_REGISTER_DUMP);

		// Wait for request completion
		wait_event_interruptible_timeout(sensor_data->usr_wait, 
			get_miscreq(sensor_data) == MISC_REQ_NONE, 
			wait_time);

		// Unlock...
		mutex_unlock(&sensor_data->hw_mutex);
	}

	return count;
}

static DEVICE_ATTR(dump_regs, S_IWUSR | S_IWGRP, 0, adv7281_dump_regs_store);

static ssize_t adv7281_lock_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *subdev =
	    i2c_get_clientdata(to_i2c_client(dev));
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	ssize_t ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d", get_user_locked_status(sensor_data));
	return ret;
}

static DEVICE_ATTR(lock, S_IRUGO, adv7281_lock_show, NULL);

static struct attribute *adv7281_attributes[] = {
	&dev_attr_input_select.attr,
	&dev_attr_lock.attr,
	&dev_attr_dump_regs.attr,
	NULL
};

static const struct attribute_group adv7281_attr_group = {
	.attrs = adv7281_attributes,
};

int adv7281_init(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data;	/* sensor specific data */
	int rval;

	sensor_data = devm_kzalloc(&client->dev,
				   sizeof(*sensor_data), GFP_KERNEL);

	if (!sensor_data)
		return -ENOMEM;
	spin_lock_init(&sensor_data->slock);
	mutex_init(&sensor_data->hw_mutex);
	sensor_data->input_select = INSEL_DEFAULT;
	sensor_data->lock_sts = 0;
	sensor_data->client = client;
	sensor->sensor_specific_data = sensor_data;

	rval = sysfs_create_group(&client->dev.kobj, &adv7281_attr_group);
	if (rval) {
		dev_err(&client->dev, "sysfs_create_group failed: %d\n",
			rval);
		return rval;
	} else {
		dev_dbg(&client->dev, "sysfs_create_group success!\n");
	}

	init_waitqueue_head(&sensor_data->hw_wait);
	init_waitqueue_head(&sensor_data->usr_wait);
	sensor_data->hw_task = kthread_run(adv7281_hw_task, client, "adv7281-hwtask");
	if (IS_ERR(sensor_data->hw_task)){
		sensor_data->hw_task = NULL;
		dev_err(&client->dev, "Failed to create hw_task!!!\n");
		return -ENOMEM;
	}

	return 0;
}


int adv7281_cleanup(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;

	/* This can be NULL if crlmodule_registered call failed before
	 * sensor_init call.  */
	if (!sensor_data)
		return 0;

	kthread_stop(sensor_data->hw_task);
	sensor_data->hw_task = NULL;
	sysfs_remove_group(&client->dev.kobj, &adv7281_attr_group);
	return 0;
}


/***
 * Function used to request a new hardware state from hardware thread + optionally
 * wait for response/completion from userspace.
 */
int adv7281_request_hw_state(struct i2c_client *client, int state, int block)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
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

int adv7281_streamon_callback(struct i2c_client *client)
{
	int ret;
	dev_dbg(&client->dev, "enter %s\n", __func__);
	ret =  adv7281_request_hw_state(client, ADV7281_HW_STATE_STREAMING, 1);
	if (ret != 0){
		dev_warn(&client->dev, "Failed or timed out starting streaming! (reset hw to ready state)\n");
		adv7281_request_hw_state(client, ADV7281_HW_STATE_READY, 0);
	}
	dev_dbg(&client->dev, "exit %s\n", __func__);
	return ret;
}

int adv7281_streamoff_callback(struct i2c_client *client)
{
	int ret;
	dev_dbg(&client->dev, "enter %s\n", __func__);
	ret = adv7281_request_hw_state(client, ADV7281_HW_STATE_READY, 1);
	dev_dbg(&client->dev, "exit %s\n", __func__);
	return ret;
}

int adv7281_sensor_status(struct i2c_client *client, int *status)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	*status = get_user_locked_status(sensor_data);
	return 0;
}

/* Read the global status 1 register, update lock status. Also post event
 * notifications (if applicable) here.
 *
 * This is not made very clear in the ADV728x datasheet, but this register can
 * be accessed from any of the "main" submaps (that is, all but the CSI
 * submap) */
static int adv7281_update_lock_status(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	uint32_t status1;
	int locked;
	int ret;


	ret = adv7281_i2c_read(client, ADV7281_REG_STATUS1, &status1);
	if (ret != 0) {
		dev_err(&client->dev, "%s: status read failed!\n",
			__func__);
		return ret;
	}
	locked = !!(status1 & STATUS1_IN_LOCK);
	dev_dbg(&client->dev, "%s: locked=%d status1=%02x\n", __func__, locked, status1);
	if (sensor_data->lock_sts != locked) {
		unsigned long flags;
		spin_lock_irqsave(&sensor_data->slock, flags);
		sensor_data->lock_sts = locked;
		spin_unlock_irqrestore(&sensor_data->slock, flags);
	}
	return 0;
}


irqreturn_t adv7281_threaded_irq_fn(int irq, void *sensor_struct)
{
	struct crl_sensor *sensor = sensor_struct;
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;
	unsigned long flags;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (!sensor_data)
		goto error_exit;
	
	/* Just wake the main 'hwtask' on IRQ events.  TODO: investigate moving
	 * this to a "normal" irq handler instead of threaded one... */
	spin_lock_irqsave(&sensor_data->slock, flags);
	sensor_data->irq_wake = 1;
	spin_unlock_irqrestore(&sensor_data->slock, flags);
	wake_up_interruptible(&sensor_data->hw_wait);

error_exit:
	return IRQ_HANDLED;
}


int adv7281_suspend_callback(struct i2c_client* client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;

	/* Simple handshake with HW thread to block it until resume callback */
	sensor_data->should_suspend = 1;
	wake_up_interruptible(&sensor_data->hw_wait);
	wait_event_interruptible(sensor_data->usr_wait, (sensor_data->suspended));

	return 0;
}

int adv7281_resume_callback(struct i2c_client* client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	struct crl_adv7281 *sensor_data = sensor->sensor_specific_data;

	/* Simple handshake to unblock HW thread unblock it */
	sensor_data->should_suspend = 0;
	wake_up_interruptible(&sensor_data->hw_wait);
	wait_event_interruptible(sensor_data->usr_wait, (!sensor_data->suspended));
	return 0;
}
