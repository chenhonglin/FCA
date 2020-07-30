/*
 *  Driver for Mobis Touchscreens
 *
 *  Copyright (c) 2014 Red Hat Inc.
 *  2016 - 2017 Mobis Technology.
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <asm/unaligned.h>
#include <linux/device.h>

extern unsigned int event_path_global;
static unsigned int native_x_pos;
static unsigned int native_y_pos;
static unsigned int native_z_pos;
extern void  update_native_touch_coordinates(unsigned int native_x_pos, unsigned int native_y_pos, unsigned int native_z_pos);
extern char *get_displaytype(void);

static bool debug_enable = false;
/* Skip the crc check till the new displays support this */
static bool skip_crc_check = true;

struct mobis_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int abs_x_max;
	int abs_y_max;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
	int irq;
	u16 ser_touch_notification_gpio;
	bool irq_enabled;
	bool event_path;
	bool display_type_detected;
	u8 display_type;
	u8 touch_type;
	u8 version_major;
	u8 version_minor;
	u8 version_patch;
};

#define MOBIS_INT_TRIGGER		1
#define MSLEEP(ms)             usleep_range(ms*1000, ms*1000)

/* Version Data macros */
#define VERSION_DATA_MSG_ID 5
#define VERSION_DATA_SIZE 10
#define VERSION_DATA_ERR_OFFSET 0
#define VERSION_DATA_MAJOR_OFFSET 1
#define VERSION_DATA_MINOR_OFFSET 2
#define VERSION_DATA_PATCH_OFFSET 3
#define VERSION_DATA_DCSD_TYPE_OFFSET 4
#define VERSION_DATA_NUM_TOUCH_OFFSET 5
#define VERSION_DATA_MSG_RETRY_CNT 6
#define VERSION_DATA_MSG_RETRY_DELAY_MS 200

/* Touch Data Read Macros */
/* 2 Multi Touch-10 Bit */
#define MTOUCH2_10BIT_MSG_ID 2
#define MTOUCH2_10BIT_MSG_SIZE 19
#define MTOUCH2_10BIT_MSG_TOUCH_IDX 3
#define MTOUCH2_10BIT_MSG_TOUCH_NEXT_OFFSET 7
#define MTOUCH2_10BIT_MSG_READ_INDX_LMT 17
#define MTOUCH2_10BIT_MSG_TOUCH_PRESENT 0x01
/* 2 Multi Touch-11 Bit */
#define MTOUCH2_11BIT_MSG_ID 6
#define MTOUCH2_11BIT_MSG_SIZE 19
#define MTOUCH2_11BIT_MSG_TOUCH_IDX 3
#define MTOUCH2_11BIT_MSG_TOUCH_NEXT_OFFSET 7
#define MTOUCH2_11BIT_MSG_READ_INDX_LMT 17
#define MTOUCH2_11BIT_MSG_TOUCH_PRESENT 0x01
/* 5 Multi Touch-11 Bit */
#define MTOUCH5_11BIT_MSG_ID 7
#define MTOUCH5_11BIT_MSG_SIZE 25
#define MTOUCH5_11BIT_MSG_TOUCH_IDX 3
#define MTOUCH5_11BIT_MSG_TOUCH_NEXT_OFFSET 4
#define MTOUCH5_11BIT_MSG_READ_INDX_LMT 23
#define MTOUCH5_11BIT_MSG_TOUCH_PRESENT 0x80
/* 5 Multi Touch-11 Bit with velocity */
#define MTOUCH5_11BIT_VEL_MSG_ID 7
#define MTOUCH5_11BIT_VEL_MSG_SIZE 40
#define MTOUCH5_11BIT_VEL_MSG_TOUCH_IDX 3
#define MTOUCH5_11BIT_VEL_MSG_TOUCH_NEXT_OFFSET 7
#define MTOUCH5_11BIT_VEL_MSG_READ_INDX_LMT 38
#define MTOUCH5_11BIT_VEL_MSG_TOUCH_PRESENT 0x01
/* "INT Type" Message (Message ID: 0x10) */
#define INT_TYPE_MSG_ID 0x10
#define INT_TYPE_MSG_SIZE 3
#define INT_TYPE_MSG_IDX 0
#define INT_TYPE_TOUCH 0x01
#define INT_TYPE_BUTTON 0x02
#define INT_TYPE_KNOB 0x03
/* "Button control" Message (Message ID: 0x20) */
#define BUTTON_CTRL_MSG_ID 0x20
#define BUTTON_CTRL_MSG_SIZE 6
/* "Knob control" Message (Message ID: 0x21) */
#define KNOB_CTRL_MSG_ID 0x21
#define KNOB_CTRL_MSG_SIZE 6

/*
* This flasg to know whether touch notification through
* SoC GPIO[14] TOUCH_IRQ <= GPIO_0 LVDS Serializer or
* SoC GPIO[15] PRI_SERDES_IRQ <= INTB LVDS Serializer
*/
#define GPIOD_OFFSET	(434)
#define SER_GPIO_0_TOUCH_SOC_GPIO	(GPIOD_OFFSET+14)
#define SER_INTB_TOUCH_SOC_GPIO	(GPIOD_OFFSET+15)

#define RETRY_TIME_MS			25
#define I2C_ADDR_SER_PORT0		(0x0C)
#define SER_REG_ISR			(0xC7)

/* Display Size Resolutions */
/* Mobis, Alpine */
#define DCSD_8_4_1024_768 "8.4_1024x768"
/* Alpine, Mobis, Conti */
#define DCSD_10_1_1920_1200 "10.1_1920x1200"
/* Conti */
#define DCSD_12_0_1920_1200 "12.0_1920x1200"
/* MagMor */
#define DCSD_10_25_1920_720 "10.25_1920x720"
#define DCSD_12_3_1920_720 "12.3_1920x720"
/* Mostly below 3 display resolutions R1 may not support */
/* TODO: 12.1 resolution need to be confirmed, Mostly R1 may not support this display */
/* Uconnect 12.1 touchscreen system for the Dodge Charger Pursuit police vehicle */
#define DCSD_12_1_1024_768 "12.1_1024x768"
#define DCSD_12_0_1280_800 "12.0_1280x800"
/* TODO: 7.0 resolution need to be confirmed, Mostly R1 may not support this display */
#define DCSD_7_0_800_480 "7.0_800x480"
#define DEFAULT_DISPLAY_TYPE "mobis"

/* Display Resolutions */
#define X_RES_1024 1024
#define Y_RES_768 768
#define X_RES_1920 1920
#define Y_RES_1200 1200
#define Y_RES_720 720
#define X_RES_1280 1280
#define Y_RES_800 800
#define X_RES_800 800
#define Y_RES_480 480

typedef enum {
	DCSD84_NoMTouch = 0,
	DCSD121_CEDviaEthernet,
	DCSD84_wMTouch,
	DCSD120_wMTouch,
	DCSD8_4_Landscape5MT,
	DCSD10_1_Landscape5MT,
	DCSD10_1_Landscape5MT_wCtrls,
	DCSD12_Landscape5MT_wCtrls,
	DCSD7Landscape5MT_wCtrls_NoCAN,
	DCSD7Landscape5MT,
	DCSD10_25Landscape5MT,
	DCSD12_3Landscape5MT,
	DCSD_Display_Cnt
}DCSD_DISPLAY_TYPE;

typedef enum {
	DCSD_SINGLETOUCH_10_BIT = 0,
	DCSD_MULTITOUCH_TWO_10_BIT,
	DCSD_MULTITOUCH_FIVE_10_BIT,
	DCSD_MULTITOUCH_TWO_11_BIT,
	DCSD_MULTITOUCH_FIVE_11_BIT,
	DCSD_MULTITOUCH_FIVE_11_BIT_VELOCITY,
	DCSD_MULTITOUCH_FIVE_11_BIT_BUTTONS_KNOBS,
}DCSD_TOUCH_TYPE;

typedef enum {
	DCSD_SINGLE_TOUCH_POINT = 1,
	DCSD_TWO_TOUCH_POINT = 2,
	DCSD_FIVE_TOUCH_POINT = 5,
}DCSD_NUM_TOUCH_POINTS;

typedef struct {
	char display_name[64];
	char display_size_res[32];
	u16 touch_type;
	u16 num_touch_points;
	u16 x_res;
	u16 y_res;
} DCSD_DISPLAYS_INFO;

/* Below table prepared as per DCSD Acronyms Table (ex. CFTS020), if data is not clear used the mobis default configuration */
static const DCSD_DISPLAYS_INFO dcsd_displays_info[DCSD_Display_Cnt] ={
	/* Case1: DCSD is I2C Master. 8.4 inch Non-multitouch (single 10-bit touch coordinate).Used on RU17. */
	{ "DCSD84_NoMTouch", DCSD_8_4_1024_768, DCSD_SINGLETOUCH_10_BIT, DCSD_SINGLE_TOUCH_POINT, X_RES_1024, Y_RES_768 },

	/*
	 * Case1: DCSD is I2C Master. 12.1 inch Non-multitouch (single 10-bit touch coordinate).
	 * Used on LD 16.5 Police Pursuit. Allows Consumer Electronics Device (ex.  laptop) to be connected via Ethernet.
	 */
	/* TODO: 12.1 resolution need to be confirmed, Mostly R1 may not support this display */
	{ "DCSD121_CEDviaEthernet", DCSD_12_1_1024_768, DCSD_SINGLETOUCH_10_BIT, DCSD_SINGLE_TOUCH_POINT, X_RES_1024, Y_RES_768 },

	/* Case2: DCSD is I2C Slave. 8.4 inch Multitouch (two 10-bit touch coordinates).Used on RU18.*/
	/* 8.4 Mobis Display 1024x768 */
	{ "DCSD84_wMTouch", DCSD_8_4_1024_768, DCSD_MULTITOUCH_TWO_10_BIT, DCSD_TWO_TOUCH_POINT, X_RES_1024, Y_RES_768 },

	/* Case2: DCSD is I2C Slave. 12.0 inch Multitouch (two 11-bit touch coordinates).Used on DT19. 1280x800.*/
	{ "DCSD120_wMTouch", DCSD_12_0_1280_800, DCSD_MULTITOUCH_TWO_11_BIT, DCSD_TWO_TOUCH_POINT, X_RES_1280, Y_RES_800 },

	/* Case2: DCSD is I2C Slave. 8.4 inch 1024x768 Landscape 5 Multitouch 11-bit Used on WL */
	{ "DCSD8.4_Landscape5MT", DCSD_8_4_1024_768, DCSD_MULTITOUCH_FIVE_11_BIT, DCSD_FIVE_TOUCH_POINT, X_RES_1024, Y_RES_768 },

	/* Case2: DCSD is I2C Slave. 10.1 inch 1920x1200 Landscape 5 Multitouch  11-bit touch coordinates Used on WL,MP,RU,WD */
	{ "DCSD10.1_Landscape5MT", DCSD_10_1_1920_1200, DCSD_MULTITOUCH_FIVE_11_BIT, DCSD_FIVE_TOUCH_POINT, X_RES_1920, Y_RES_1200 },

	/*
	* Case2: DCSD is I2C Slave. 10.1 inch 1920x1200 Landscape 5 Multitouch  11-bit touch coordinates  With Controls (Buttons/Knobs) sent over CAN
	* Used on WS.
	*/
	{ "DCSD10.1_Landscape5MT_wCtrls", DCSD_10_1_1920_1200, DCSD_MULTITOUCH_FIVE_11_BIT, DCSD_FIVE_TOUCH_POINT, X_RES_1920, Y_RES_1200 },

	/*
	 * Case2: DCSD is I2C Slave. 12.0 inch 1920x1200 Landscape 5 Multitouch  11-bit touch coordinates  With Controls (Buttons/Knobs) sent over CAN
	 * Used on WS.
	 */
	{ "DCSD12_Landscape5MT_wCtrls", DCSD_12_0_1920_1200, DCSD_MULTITOUCH_FIVE_11_BIT, DCSD_FIVE_TOUCH_POINT, X_RES_1920, Y_RES_1200 },

	/* Case3: DCSD is I2C Slave. 7.0 inch 5 Multitouch (11-bit touch coordinates). With Controls (Buttons/Knobs) but No CAN,
	 * Data sent over LVDS
	 */
	 /* TODO: 7.0 resolution need to be confirmed, Mostly R1 may not support this display */
	{ "DCSD7Landscape5MT_wCtrls_NoCAN", DCSD_7_0_800_480, DCSD_MULTITOUCH_FIVE_11_BIT_BUTTONS_KNOBS, DCSD_FIVE_TOUCH_POINT, X_RES_800, Y_RES_480 },

	/* Case2: DCSD is I2C Slave. 7.0 inch 5 Multitouch (two 11-bit touch coordinates). Used on 500 BEV */
	/* TODO: 7.0 resolution need to be confirmed, Mostly R1 may not support this display */
	{ "DCSD7Landscape5MT", DCSD_7_0_800_480, DCSD_MULTITOUCH_FIVE_11_BIT, DCSD_FIVE_TOUCH_POINT, X_RES_800, Y_RES_480 },

	/* Case2: DCSD is I2C Slave. 10.25 inch 5 Multitouch (two 11-bit touch coordinates) */
	/* TODO: Magneti Mareli 10.25in display is following with velocity DCSD_and_HU_LVDS_Backchannel_Protocol_v2.8.docx, revisit on this once they change this part as per v2.8 */
	{ "DCSD10.25Landscape5MT", DCSD_10_25_1920_720, DCSD_MULTITOUCH_FIVE_11_BIT_VELOCITY, DCSD_FIVE_TOUCH_POINT, X_RES_1920, Y_RES_720 },

	/* Case2: DCSD is I2C Slave. 12.3 inch 5 Multitouch (two 11-bit touch coordinates). */
	{ "DCSD12.3Landscape5MT", DCSD_12_3_1920_720, DCSD_MULTITOUCH_FIVE_11_BIT, DCSD_FIVE_TOUCH_POINT, X_RES_1920, Y_RES_720 },
};

static const unsigned long mobis_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};

static int generic_i2c_read_reg(struct i2c_client *client, u8 device_addr, u8 reg, u8 val_len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[1];
	int ret;
	bool retry = false;

	/* Write register */
	buf[0] = reg;
	xfer[0].addr = device_addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = device_addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = val_len;
	xfer[1].buf = val;

	retry_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(RETRY_TIME_MS);
			retry = true;
			goto retry_read;
		} else {
			dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
					__func__, ret);
			return -EIO;
		}
	}

	return 0;
}

/**
 * mobis_ser_isr_read - clear the SER ISR status through read
 *
 * @ts: our mobis_ts_data pointer
 *
 * Called when the IRQ is triggered. to clear the recieved interrupt
 */
static void mobis_ser_isr_read(struct mobis_ts_data *ts)
{
	int ret;
	struct device *dev = &ts->client->dev;
	u8 isr_status = 0;

	ret = generic_i2c_read_reg(ts->client, I2C_ADDR_SER_PORT0, SER_REG_ISR, 1, &isr_status);
	if (ret) {
		dev_err(dev, "Failed to read SER_REG_ISR \n");
	}
}

static u16 calc_crc16(const u8 *data, int size)
{
#define INITIAL_CRC_REMAINDER 0xFFFF
#define POLYNOMIAL_VAL 0x1021

	int i, j;
	u16 crc = INITIAL_CRC_REMAINDER;

	for (i = 0; i < size; i++) {
		crc ^= ((u16)data[i] << 8);
		for (j = 8; j > 0; j--)
			if (crc & 0x8000)
				crc = ( (crc << 1) ^ POLYNOMIAL_VAL );
			else
				crc <<= 1;
	}

	return crc;
}

static int check_crc(u8 *buf, int buflen)
{
	u16 cal_crc;
	u16 recv_crc;

	cal_crc = calc_crc16(buf, buflen - 2);

	/* buflen-2=CRC-16 HI, buflen-1=CRC-16 LO */
	recv_crc = ( (buf[buflen - 2]<<8) | buf[buflen - 1] );

	if (cal_crc != recv_crc) {
		if(printk_ratelimit()) {
			printk(KERN_ERR "mobis_i2c_read: CRC error cal_crc[0x%x] != recv_crc[0x%x]\n", cal_crc, recv_crc);
		}
		return -EPERM;
	}

	return 0;
}

/**
 * mobis_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int mobis_i2c_read(struct i2c_client *client,
				u8 reg, u8 *buf, int len)
{
	int ret;

	// Select the message to read
	ret = i2c_master_send(client, &reg, 1);

	if (1 != ret)
	{
		return (-EIO);
	}

	// Read back the message data
	ret = i2c_master_recv(client, buf, len);

	if (ret != len)
	{
		return (-EIO);
	}

	if( (debug_enable == true) || (reg == VERSION_DATA_MSG_ID) ) {
		#define MAX_LOG_BUFF_SIZE 512
		#define ONE_BYTE_PRINT_SIZE 5
		unsigned char debugStr[MAX_LOG_BUFF_SIZE] = {0};
		int size;
		int i = 0;
		size = sprintf(debugStr, "Req Msg ID:%d, Req Read Length:%d, Read Data:", reg, len);
		while ((i < len) && (size < (MAX_LOG_BUFF_SIZE-ONE_BYTE_PRINT_SIZE)))
		{
			size += sprintf(&debugStr[size], " 0x%x", buf[i++]);
		}
		printk(KERN_INFO "mobis_i2c_read: %s\n", debugStr);
	}

	ret  = check_crc(buf, len);
	/* TODO: Temporary fix till Alpine and Magneti Mareli display touch works or we get the final touch FW */
	if ( (ret != 0) && (skip_crc_check != true) ) {
		return -EBADMSG;
	}

	return 0;
}

static void mobis_ts_report_touch(struct mobis_ts_data *ts, int input_x, int input_y, int id)
{
	int input_w = 1;

	if (event_path_global == 0 ) {
		input_mt_slot(ts->input_dev, id);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
	} else {
		native_x_pos = input_x;
		native_y_pos = input_y;
		update_native_touch_coordinates(native_x_pos, native_y_pos, native_z_pos);
		native_z_pos = 0; /* Not supporting in Mobis */
	}
}

static int mobis_ts_proc_2mtouch_10bit(struct mobis_ts_data *ts, int *touch_pressed)
{
	u8 data[MTOUCH2_10BIT_MSG_SIZE] = {0};
	int error = 0;
	int slot_id=0;
	int i=0;

	error = mobis_i2c_read(ts->client, MTOUCH2_10BIT_MSG_ID, data, MTOUCH2_10BIT_MSG_SIZE);
	if (error < 0) {
		if(printk_ratelimit()) {
			dev_err(&ts->client->dev, "mobis_ts_proc_2mtouch_10bit, mobis_i2c_read error: %d\n", error);
		}
		return error;
	}

	/* Sending slot id along with touch data to separate 1st finger touch with another finger touch */
	for (i = MTOUCH2_10BIT_MSG_TOUCH_IDX, slot_id = 0; i < MTOUCH2_10BIT_MSG_READ_INDX_LMT; i += MTOUCH2_10BIT_MSG_TOUCH_NEXT_OFFSET, slot_id++) {
		if (data[i] & MTOUCH2_10BIT_MSG_TOUCH_PRESENT) {
			/* Byte0:Bit 4:7 = X coordinates HI ( 4bit ), Byte1:Bit 0:5 = X coordinates LO ( 6bit ), Total 10bit */
			int input_x = (((u16)data[i] & 0xf0) << 2) | (data[i+1] & 0x3f);
			/* Byte1:Bit 6:7 = Y coordinates HI ( 2bit ), Byte2:Bit 0:7 = Y coordinates LO ( 8bit ), Total 10bit */
			int input_y = (((u16)data[i+1] & 0xc0) << 2) | data[i+2];

			mobis_ts_report_touch(ts, input_x, input_y, slot_id);
			*touch_pressed = 1;
		}
	}

	return error;
}

static int mobis_ts_proc_2mtouch_11bit(struct mobis_ts_data *ts, int *touch_pressed)
{
	u8 data[MTOUCH2_11BIT_MSG_SIZE] = {0};
	int error = 0;
	int slot_id=0;
	int i=0;

	error = mobis_i2c_read(ts->client, MTOUCH2_11BIT_MSG_ID, data, MTOUCH2_11BIT_MSG_SIZE);
	if (error < 0) {
		if(printk_ratelimit()) {
			dev_err(&ts->client->dev, "mobis_ts_proc_2mtouch_11bit, mobis_i2c_read error: %d\n", error);
		}
		return error;
	}

	/* Sending slot id along with touch data to separate 1st finger touch with another finger touch */
	for (i = MTOUCH2_11BIT_MSG_TOUCH_IDX, slot_id = 0; i < MTOUCH2_11BIT_MSG_READ_INDX_LMT; i += MTOUCH2_11BIT_MSG_TOUCH_NEXT_OFFSET, slot_id++) {
		if (data[i] & MTOUCH2_11BIT_MSG_TOUCH_PRESENT) {
			/* Byte0:Bit 2:7 = X coordinates HI ( 6bit), Byte1:Bit 0:4 = X coordinates LO ( 5bit ), Total 11bit */
			int input_x = (((u16)data[i] & 0xFC) << 3) | (data[i+1] & 0x1F);
			/* Byte1:Bit 5:7 = Y coordinates HI ( 3bit ), Byte2:Bit 0:7 = Y coordinates LO ( 8bit ), Total 11bit */
			int input_y = (((u16)data[i+1] & 0xE0) << 3) | data[i+2];

			mobis_ts_report_touch(ts, input_x, input_y, slot_id);
			*touch_pressed = 1;
		}
	}

	return error;
}

static int mobis_ts_proc_5mtouch_11bit(struct mobis_ts_data *ts, int *touch_pressed)
{
	u8 data[MTOUCH5_11BIT_MSG_SIZE] = {0};
	int error = 0;
	int slot_id=0;
	int i=0;

	error = mobis_i2c_read(ts->client, MTOUCH5_11BIT_MSG_ID, data, MTOUCH5_11BIT_MSG_SIZE);
	if (error < 0) {
		if(printk_ratelimit()) {
			dev_err(&ts->client->dev, "mobis_ts_proc_5mtouch_11bit, mobis_i2c_read error: %d\n", error);
		}
		return error;
	}

	/* Sending slot id along with touch data to separate 1st finger touch with another finger touch */
	for (i = MTOUCH5_11BIT_MSG_TOUCH_IDX, slot_id = 0; i < MTOUCH5_11BIT_MSG_READ_INDX_LMT; i += MTOUCH5_11BIT_MSG_TOUCH_NEXT_OFFSET, slot_id++) {
		if (data[i+1] & MTOUCH5_11BIT_MSG_TOUCH_PRESENT) {
			/* Byte0:Bit 0:7 = X coordinates LO (8 bit), Byte1:Bit 0:2 = X coordinates HI (3 bit), Total 11bit */
			int input_x = (((u16)data[i+1] & 0x07) << 8) | data[i];
			/* Byte2:Bit 0:7 = Y coordinates LO ( 8bit ), Byte3:Bit 0:2 = Y coordinates HI (3 bit), Total 11bit */
			int input_y = (((u16)data[i+3] & 0x07) << 8) | data[i+2];

			mobis_ts_report_touch(ts, input_x, input_y, slot_id);
			*touch_pressed = 1;
		}
	}

	return error;
}

static int mobis_ts_proc_5mtouch_11bit_velocity(struct mobis_ts_data *ts, int *touch_pressed)
{
	u8 data[MTOUCH5_11BIT_VEL_MSG_SIZE] = {0};
	int error = 0;
	int slot_id=0;
	int i=0;

	error = mobis_i2c_read(ts->client, MTOUCH5_11BIT_VEL_MSG_ID, data, MTOUCH5_11BIT_VEL_MSG_SIZE);
	if (error < 0) {
		if(printk_ratelimit()) {
			dev_err(&ts->client->dev, "mobis_ts_proc_5mtouch_11bit_velocity, mobis_i2c_read error: %d\n", error);
		}
		return error;
	}

	/* Sending slot id along with touch data to separate 1st finger touch with another finger touch */
	for (i = MTOUCH5_11BIT_VEL_MSG_TOUCH_IDX, slot_id = 0; i < MTOUCH5_11BIT_VEL_MSG_READ_INDX_LMT; i += MTOUCH5_11BIT_VEL_MSG_TOUCH_NEXT_OFFSET, slot_id++) {
		if (data[i] & MTOUCH5_11BIT_VEL_MSG_TOUCH_PRESENT) {
			/* Byte0:Bit 2:7 = X coordinates HI ( 6bit), Byte1:Bit 0:4 = X coordinates LO ( 5bit ), Total 11bit */
			int input_x = (((u16)data[i] & 0xFC) << 3) | (data[i+1] & 0x1F);
			/* Byte1:Bit 5:7 = Y coordinates HI ( 3bit ), Byte2:Bit 0:7 = Y coordinates LO ( 8bit ), Total 11bit */
			int input_y = (((u16)data[i+1] & 0xE0) << 3) | data[i+2];

			mobis_ts_report_touch(ts, input_x, input_y, slot_id);
			*touch_pressed = 1;
		}
	}

	return error;
}

static int mobis_ts_proc_button(struct mobis_ts_data *ts)
{
	u8 data[BUTTON_CTRL_MSG_SIZE] = {0};
	int error = 0;

	error = mobis_i2c_read(ts->client, BUTTON_CTRL_MSG_ID, data, BUTTON_CTRL_MSG_SIZE);
	if (error < 0) {
		if(printk_ratelimit()) {
			dev_err(&ts->client->dev, "mobis_ts_proc_button, mobis_i2c_read error: %d\n", error);
		}
		return error;
	}

	/* TODO: Button signal posting to input sub system is pending, we will handle it once we have clarity on supporting this display type */
	if(printk_ratelimit()) {
		dev_warn(&ts->client->dev, "mobis_ts_proc_button, is NOT SUPPORTED CURRENTLY\n");
	}

	return error;
}

static int mobis_ts_proc_knob(struct mobis_ts_data *ts)
{
	u8 data[KNOB_CTRL_MSG_SIZE] = {0};
	int error = 0;

	error = mobis_i2c_read(ts->client, KNOB_CTRL_MSG_ID, data, KNOB_CTRL_MSG_SIZE);
	if (error < 0) {
		if(printk_ratelimit()) {
			dev_err(&ts->client->dev, "mobis_ts_proc_knob, mobis_i2c_read error: %d\n", error);
		}
		return error;
	}

	/* TODO: Knob signal posting to input sub system is pending, we will handle it once we have clarity on supporting this display type */
	if(printk_ratelimit()) {
		dev_warn(&ts->client->dev, "mobis_ts_proc_knob, is NOT SUPPORTED CURRENTLY\n");
	}

	return error;
}

static int mobis_ts_proc_5mtouch_11bit_button_knob(struct mobis_ts_data *ts, int *touch_pressed)
{
	u8 data[INT_TYPE_MSG_SIZE] = {0};
	int error = 0;

	error = mobis_i2c_read(ts->client, INT_TYPE_MSG_ID, data, INT_TYPE_MSG_SIZE);
	if (error < 0) {
		if(printk_ratelimit()) {
			dev_err(&ts->client->dev, "mobis_ts_proc_5mtouch_11bit_button_knob, mobis_i2c_read error: %d\n", error);
		}
		return error;
	}

	switch(data[INT_TYPE_MSG_IDX]) {
		case INT_TYPE_TOUCH:
			error = mobis_ts_proc_5mtouch_11bit(ts, touch_pressed);
			break;
		case INT_TYPE_BUTTON:
			error = mobis_ts_proc_button(ts);
			break;
		case INT_TYPE_KNOB:
			error = mobis_ts_proc_knob(ts);
			break;
		default:
			break;
	}

	return error;
}

/**
 * mobis_process_events - Process incoming events
 *
 * @ts: our mobis_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void mobis_process_events(struct mobis_ts_data *ts)
{
	int error=-1;
	int touch_pressed = 0;

	switch(ts->touch_type) {
		case DCSD_MULTITOUCH_TWO_10_BIT:
			error = mobis_ts_proc_2mtouch_10bit(ts, &touch_pressed);
			break;
		case DCSD_MULTITOUCH_TWO_11_BIT:
			error = mobis_ts_proc_2mtouch_11bit(ts, &touch_pressed);
			break;
		case DCSD_MULTITOUCH_FIVE_11_BIT:
			error = mobis_ts_proc_5mtouch_11bit(ts, &touch_pressed);
			break;
		case DCSD_MULTITOUCH_FIVE_11_BIT_VELOCITY:
			error = mobis_ts_proc_5mtouch_11bit_velocity(ts, &touch_pressed);
			break;
		case DCSD_MULTITOUCH_FIVE_11_BIT_BUTTONS_KNOBS:
			error = mobis_ts_proc_5mtouch_11bit_button_knob(ts, &touch_pressed);
			break;
		case DCSD_SINGLETOUCH_10_BIT:
		case DCSD_MULTITOUCH_FIVE_10_BIT:
		default:
			if(printk_ratelimit()) {
				dev_err(&ts->client->dev, "mobis_process_events, touch_type: %d processing is NOT SUPPORTED\n", ts->touch_type);
			}
			break;
	}

	if (error < 0)
		return;

	if (event_path_global && (0 == touch_pressed)) {
		update_native_touch_coordinates(0,0,0);
	}

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

/**
 * mobis_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t mobis_ts_irq_handler(int irq, void *dev_id)
{
	struct mobis_ts_data *ts = dev_id;

	mobis_process_events(ts);

	if(ts->ser_touch_notification_gpio == SER_INTB_TOUCH_SOC_GPIO) {
		/* Read SER ISR REG to clear the interrupt */
		mobis_ser_isr_read(ts);
	}

	return IRQ_HANDLED;
}

/**
 * mobis_read_config - Read the embedded configuration of the panel
 *
 * @ts: our mobis_ts_data pointer
 *
 * Must be called during probe
 */
static void mobis_read_config(struct mobis_ts_data *ts)
{
	u8 version_data[10] = {0};
	u8 error_status = 0;
	u8 version_major = 0;
	u8 version_minor = 0;
	u8 version_patch = 0;
	u8 dcsd_type = 0;
	u8 num_touch_points = 0;
	int error = 0;
	int retry_cnt = 0;

	// Read the "version data" message, we need retry for this msg as this data will be available only after 1.2sec of touch controller boot
	while(retry_cnt < VERSION_DATA_MSG_RETRY_CNT) {
		error = mobis_i2c_read(ts->client, VERSION_DATA_MSG_ID, version_data, VERSION_DATA_SIZE);
		if(error < 0) {
			printk(KERN_WARNING "mobis_i2c_read failed error=%d, retry_cnt=%d\n", error, retry_cnt);
			MSLEEP(VERSION_DATA_MSG_RETRY_DELAY_MS);
		} else {
			break;
		}
		retry_cnt++;
	}

	error_status = version_data[VERSION_DATA_ERR_OFFSET];
	dcsd_type = version_data[VERSION_DATA_DCSD_TYPE_OFFSET];
	if ( (error < 0) || error_status || (dcsd_type >= DCSD_Display_Cnt) ) {
		/* Load default Mobis display touch */
		ts->display_type = DCSD84_wMTouch;
		ts->display_type_detected = false;
		ts->ser_touch_notification_gpio = SER_INTB_TOUCH_SOC_GPIO;
		printk(KERN_ERR "mobis_read_config Version Data Read failed, error: %d, error_status=0x%x, dcsd_type=%d, loading default Mobis display\n",
						error, error_status, dcsd_type);
	} else {

		ts->display_type = dcsd_type;
		ts->display_type_detected = true;
		version_major = version_data[VERSION_DATA_MAJOR_OFFSET];
		version_minor = version_data[VERSION_DATA_MINOR_OFFSET];
		version_patch = version_data[VERSION_DATA_PATCH_OFFSET];
		num_touch_points = version_data[VERSION_DATA_NUM_TOUCH_OFFSET];
		if(dcsd_displays_info[ts->display_type].num_touch_points != num_touch_points) {
			printk(KERN_WARNING "mobis_i2c_read Received wrong num_touch_points = %d instead of %d for dcsd_type=%d\n",
						num_touch_points, dcsd_displays_info[ts->display_type].num_touch_points, dcsd_type);
		}
		/* Mobis 2018-2020 uses INT-B others uses GPIO-0 */
		if( (version_data[VERSION_DATA_MAJOR_OFFSET] <= 20) && (ts->display_type == DCSD84_wMTouch) ) {
			ts->ser_touch_notification_gpio = SER_INTB_TOUCH_SOC_GPIO;
		} else {
			ts->ser_touch_notification_gpio = SER_GPIO_0_TOUCH_SOC_GPIO;
		}

		printk(KERN_INFO "mobis_read_config loading driver with version=%d.%d.%d, display_name=%s, touch_type=%d, num_touch_points=%d, x_res=%d, y_res=%d and ser_touch_notification_gpio =%d\n",
			version_major, version_minor, version_patch, dcsd_displays_info[ts->display_type].display_name,
			dcsd_displays_info[ts->display_type].touch_type, dcsd_displays_info[ts->display_type].num_touch_points,
			dcsd_displays_info[ts->display_type].x_res, dcsd_displays_info[ts->display_type].y_res,
			ts->ser_touch_notification_gpio);
	}

	/* TODO: Temporary fix till Alpine and Magneti Mareli display touch works or we get the final touch FW */
	{
		const char disp_2k10in[] = "10.1_1920x1200";
		const char disp_mm10in[] = "10.25_1920x720";
		char *display_type = get_displaytype();
		if ( ( strcmp(display_type, disp_2k10in )==0 ) &&
			( ( (ts->display_type != DCSD10_1_Landscape5MT) &&
			    (ts->display_type != DCSD10_1_Landscape5MT_wCtrls) ) ||
			  (ts->ser_touch_notification_gpio != SER_GPIO_0_TOUCH_SOC_GPIO) ) ) {

			ts->display_type = DCSD10_1_Landscape5MT;
			ts->ser_touch_notification_gpio = SER_GPIO_0_TOUCH_SOC_GPIO;
			printk(KERN_WARNING "mobis_read_config:Dynamic detection is not detecting Alpine 10.1 inch display, so forcing the touch configuration to Alpine\n");

		} else if ( ( strcmp(display_type, disp_mm10in )==0 ) &&
				( (ts->display_type != DCSD10_25Landscape5MT) ||
				(ts->ser_touch_notification_gpio != SER_GPIO_0_TOUCH_SOC_GPIO) ) ) {

			ts->display_type = DCSD10_25Landscape5MT;
			ts->ser_touch_notification_gpio = SER_GPIO_0_TOUCH_SOC_GPIO;
			printk(KERN_WARNING "mobis_read_config:Dynamic detection is not detecting Magneti Mareli 10.25in display, so forcing the touch configuration to Magneti Mareli\n");

		} else {
			skip_crc_check = false;
		}
	}

	ts->touch_type = dcsd_displays_info[ts->display_type].touch_type;
	ts->max_touch_num = dcsd_displays_info[ts->display_type].num_touch_points;
	ts->abs_x_max = dcsd_displays_info[ts->display_type].x_res;
	ts->abs_y_max = dcsd_displays_info[ts->display_type].y_res;
	ts->int_trigger_type = MOBIS_INT_TRIGGER;
	ts->version_major=version_major;
	ts->version_minor=version_minor;
	ts->version_patch=version_patch;

}


static ssize_t event_path_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mobis_ts_data *ts = i2c_get_clientdata(client);
    struct input_dev *input_dev = ts->input_dev;
    ssize_t ret;

    mutex_lock(&input_dev->mutex);
    if(ts->event_path)
        ret = snprintf(buf, 100 ,"Native Application path ENABLED\n");
    else
        ret = snprintf(buf, 30 ,"Android path ENABLED\n");
    mutex_unlock(&input_dev->mutex);

    return ret;
}

static ssize_t event_path_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;
	unsigned long value;
	int retval;

	retval = kstrtoul(buf, 10, &value);
	if(retval < 0)
		return retval;

	mutex_lock(&input_dev->mutex);
	switch(value){
		case 0:
			ts->event_path = false;
			event_path_global = 0;
			break;

		case 1:
			ts->event_path = true;
			event_path_global = 1;
			break;

		default:
			return -EINVAL;
	}
	mutex_unlock(&input_dev->mutex);

	return count;
}

static ssize_t drv_irq_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mobis_ts_data *ts = i2c_get_clientdata(client);
    struct input_dev *input_dev = ts->input_dev;
    ssize_t ret;

    mutex_lock(&input_dev->mutex);
    if(ts->irq_enabled)
	ret = snprintf(buf, 30 ,"Driver interrupt is ENABLED\n");
    else
	ret = snprintf(buf, 30 ,"Driver interrupt is DISABLED\n");
    mutex_unlock(&input_dev->mutex);

    return ret;
}

/*Enable/Disable IRQ via sysfs*/
static ssize_t drv_irq_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mobis_ts_data *ts = i2c_get_clientdata(client);
    struct input_dev *input_dev = ts->input_dev;
    unsigned long drv_irq_value;
    int retval;

    retval = kstrtoul(buf,10,&drv_irq_value);
    if(retval < 0)
       return retval;

    mutex_lock(&input_dev->mutex);
    switch(drv_irq_value){
    case 0:
         if (ts->irq_enabled){
             ts->irq_enabled = false;
             disable_irq(ts->irq);
         }
         break;

    case 1:
         if(ts->irq_enabled == false){
             ts->irq_enabled = true;
             enable_irq(ts->irq);
         }
         break;

    default:
         return -EINVAL;
    }
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t touch_pos_x_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;
    ssize_t ret;

    mutex_lock(&input_dev->mutex);
    ret = snprintf(buf, 30,
       "%d\n", native_x_pos);
    mutex_unlock(&input_dev->mutex);

    return ret;
}

static ssize_t touch_pos_y_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;
    ssize_t ret;

    mutex_lock(&input_dev->mutex);
    ret = snprintf(buf, 30,
       "%d\n", native_y_pos);
    mutex_unlock(&input_dev->mutex);

    return ret;
}

static ssize_t display_type_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;
	ssize_t ret;

	mutex_lock(&input_dev->mutex);
	ret = snprintf( buf, PAGE_SIZE, "%d\n", (ts->display_type_detected ? ts->display_type : 0xFF) );
	mutex_unlock(&input_dev->mutex);

	return ret;
}

static ssize_t display_size_res_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;
	ssize_t ret;
	char *display_size_res = DEFAULT_DISPLAY_TYPE;

	mutex_lock(&input_dev->mutex);
	if( (ts->display_type_detected) && (ts->display_type < DCSD_Display_Cnt) &&
		/* If detected display is mobis then return display as default one only */
		( !( (ts->display_type == DCSD84_wMTouch) && (ts->ser_touch_notification_gpio == SER_INTB_TOUCH_SOC_GPIO) ) ) ) {
		display_size_res = dcsd_displays_info[ts->display_type].display_size_res;
	}
	ret = snprintf( buf, PAGE_SIZE, "%s\n", display_size_res);
	mutex_unlock(&input_dev->mutex);

	return ret;
}

static ssize_t debug_enable_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;
	ssize_t ret;

	mutex_lock(&input_dev->mutex);
	ret = snprintf( buf, PAGE_SIZE, "%d\n", debug_enable );
	mutex_unlock(&input_dev->mutex);

	return ret;
}

static ssize_t debug_enable_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;
	u8 value;
	int retval;

	retval = kstrtou8(buf, 0, &value);
	if(retval < 0)
		return retval;

	mutex_lock(&input_dev->mutex);
	switch(value){
		case 0:
			debug_enable = false;
			retval= 1;
			break;
		case 1:
			debug_enable = true;
			retval= 1;
			break;
		default:
			retval= -EINVAL;
			break;
	}
	mutex_unlock(&input_dev->mutex);

	return retval;
}

static ssize_t touch_version_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;
	ssize_t ret;

	mutex_lock(&input_dev->mutex);
	ret = snprintf( buf, PAGE_SIZE, "%d.%d.%d\n", ts->version_major, ts->version_minor, ts->version_patch);
	mutex_unlock(&input_dev->mutex);

	return ret;
}

static struct device_attribute attributes[] = {
	__ATTR(drv_irq,S_IRUSR | S_IWUSR,drv_irq_show,drv_irq_store),
	__ATTR(event_path, S_IRUSR | S_IWUSR, event_path_show, event_path_store),
	__ATTR(touch_pos_x, S_IRUGO, touch_pos_x_show, NULL),
	__ATTR(touch_pos_y, S_IRUGO, touch_pos_y_show, NULL),
	__ATTR(display_type, S_IRUGO, display_type_show, NULL),
	__ATTR(display_size_res, S_IRUGO, display_size_res_show, NULL),
	__ATTR(touch_debug_enable, S_IRUGO | S_IWUSR, debug_enable_show, debug_enable_store),
	__ATTR(touch_version, S_IRUGO, touch_version_show, NULL),

};


static int add_sysfs_interfaces(struct device *dev)
{
     int i;
     for(i=0; i < ARRAY_SIZE(attributes);i++)
         if(device_create_file(dev,attributes+i))
             goto undo;
     return 0;

     undo:
         for(i--;i>=0;i--)
	    device_remove_file(dev,attributes + i);
         dev_err(dev, "%s: failed to create sysfs interface\n",__func__);
         return -ENODEV;
}


/**
 * mobis_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our mobis_ts_data pointer
 *
 * Must be called during probe
 */
static int mobis_request_input_dev(struct mobis_ts_data *ts)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) |
				  BIT_MASK(EV_KEY) |
				  BIT_MASK(EV_ABS);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
				ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
				ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_mt_init_slots(ts->input_dev, ts->max_touch_num,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);

	ts->input_dev->name = "Mobis Resistive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor =  0x0416;
	ts->input_dev->id.product = 0x1001;
	ts->input_dev->id.version = 10427;

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev, "Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

static int mobis_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mobis_ts_data *ts;
	unsigned long irq_flags;
	int error;

        printk("mobis ts probe called*****\n");

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;

 /* Default Android path */
	ts->event_path = false;
	/* Do not initialize the event_path_global variable
	 * if touch driver loaded after early evs writing '1'
	 * to event_path_global, then native touch would not
	 * work
	 * event_path_global = 0;
	 */

	i2c_set_clientdata(client, ts);


	mobis_read_config(ts);

	error = mobis_request_input_dev(ts);
	if (error)
		return error;

	irq_flags = mobis_irq_flags[ts->int_trigger_type] | IRQF_ONESHOT;

	client->irq = gpiod_to_irq(gpio_to_desc(ts->ser_touch_notification_gpio));
	ts->irq = client->irq;
	printk("mobis ts probe bfore request irq*****\n");

	error = request_threaded_irq(client->irq, NULL, mobis_ts_irq_handler,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     client->name, ts);
	if (error) {
		dev_err(&client->dev, "request IRQ failed: %d\n", error);
		return error;
	}
        ts->irq_enabled = true;
        printk("mobis touch adding sysfs interface for enable/diable IRQ\n");
        error = add_sysfs_interfaces(&client->dev);
        if(error){
                dev_err(&client->dev,"%s: Error,fail sysfs init\n",__func__);
                return error;
        }


	return 0;
}

static int mobis_remove(struct i2c_client *client)
{
	struct mobis_ts_data *ts = i2c_get_clientdata(client);

//	sysfs_remove_group(&client->dev.kobj, &mobis_attr_group);
        disable_irq(ts->irq);
	free_irq(ts->irq, ts);
//	mobis_free_input_device(data);
//	mobis_free_object_table(data);
	kfree(ts);

	return 0;
}


static int __maybe_unused mobis_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;

        printk("mobis_suspend called***\n");
	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	disable_irq(ts->irq);

#ifdef CONFIG_ACPI
        if (ACPI_COMPANION(dev)) {
                if (acpi_device_set_power(ACPI_COMPANION(dev), ACPI_STATE_D3_COLD))
                        dev_err(dev, "ACPI dstate transition to D3cold failed");
        }
#endif

	mutex_unlock(&input_dev->mutex);
        printk("mobis_suspend end***\n");

	return 0;
}

static int __maybe_unused mobis_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mobis_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;

        printk("mobis_resume called***\n");
	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

#ifdef CONFIG_ACPI
        if (ACPI_COMPANION(dev)) {
                if (acpi_device_set_power(ACPI_COMPANION(dev), ACPI_STATE_D0))
                        dev_err(dev, "ACPI dstate tansition to D0 failed");
        }
#endif

        enable_irq(ts->irq);

	mutex_unlock(&input_dev->mutex);
        printk("mobis_resume end ***\n");

	return 0;
}


static SIMPLE_DEV_PM_OPS(mobis_pm_ops, mobis_suspend, mobis_resume);

static const struct i2c_device_id mobis_ts_id[] = {
        { "mobis_ts", 0 },
	{ }
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id mobis_acpi_match[] = {
	{ "mobis_ts", 0 },

	{ }
};
MODULE_DEVICE_TABLE(acpi, mobis_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id mobis_of_match[] = {
	{ .compatible = "mobis_ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, mobis_of_match);
#endif

static struct i2c_driver mobis_ts_driver = {
	.probe = mobis_ts_probe,
	.remove = mobis_remove,
	.id_table = mobis_ts_id,
	.driver = {
	.name = "mobis_ts",
	.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(mobis_acpi_match),
		.of_match_table = of_match_ptr(mobis_of_match),
		.pm	= &mobis_pm_ops,
	},
};


static int __init at_mxt_modinit(void)
{
        int ret = 0;

        ret = i2c_add_driver(&mobis_ts_driver);
	if (ret)
		printk(KERN_ERR "Failed to register mobis_ts driver(%d)\n", ret);

        return ret;
}

module_init(at_mxt_modinit);

static void __exit at_mxt_exit(void)
{
        i2c_del_driver(&mobis_ts_driver);
}
module_exit(at_mxt_exit);

MODULE_AUTHOR("Scott MacKay   <scott.macKay@harman.com>");
MODULE_AUTHOR("Parinaz Sayyah <parinaz.sayyah@harman.com>");
MODULE_DESCRIPTION("Mobis touchscreen driver");
MODULE_LICENSE("GPL v2");
