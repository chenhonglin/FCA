/*
 *  nmtp_plugin.h
 *  Copyright (C) 2019 Harman International Ltd,
 *  Author: Akshay Shankarmurthy <Akshay.Shankarmurthy@harman.com>
 *  Created on: 01-Jan-2019
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2.0 as published by
 * the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _NMTP_PLUGIN_H
#define _NMTP_PLUGIN_H

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "nmtp: "fmt
#define DEBUG

#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>
#include <generated/compile.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/jiffies.h>
#include <linux/spi/nmtp_dev.h>

#define NMTP_FRAME_SIZE 256
#define MRD_FRAME_SIZE		256
#define WRITE_FIFO_SIZE		7168
#define BUF_SIZE		3840 // 15 * FRAME SIZE

#define NMTP_UNACKED_RESET_IN_PROGRESS 		0
#define NMTP_ACKED_RESET_IN_PROGRESS   		1
#define NMTP_NORMAL_WAIT_FOR_ACK	   		2
#define NMTP_NORMAL_OPERATION		   		3
#define NMTP_FORCED_RESET_PENDING	   		4
#define NMTP_REQUEST_RESET_PENDING	   		5
#define NMTP_UNACKED_RESUME_RESET_PENDING	6
#define NMTP_ACKED_RESUME_RESET_PENDING	    7
#define NMTP_NORMAL_RESUME_RESET_PENDING	8
#define NMTP_SPIDEV_MODE			9
#define NMTP_SUSPEND_STATE			10
#define NMTP_RESUME_STATE			11

#define WRITE_BUF_LIMIT		7680
/* Max one payload size is 1280 */

#define DYNAMIC_MINORS		32
#define DEVTYPE			"nmtp_spi"
#define DELAY_MS(x)		(x * 1000000)

#define RD_WR_INPROCESS		0x1
#define RD_WR_CLEAR		0x0

#define DEFAULT_PRIORITY_POLL_DEALY		10
#define MID_PRIORITY_POLL_DEALY			5
#define HIGH_PRIORITY_POLL_DEALY		2
#define EXT_HIGH_PRIORITY_POLL_DEALY		1


#define NMTP_DEVNODES	0x9

#define NMTP_IOCTL_MAGIC		0xCC

#define NMTP_START_CAPTURE _IO(NMTP_IOCTL_MAGIC, 0)
#define NMTP_STOP_CAPTURE  _IO(NMTP_IOCTL_MAGIC, 1)
#define NMTP_POLL_DURATION _IOW(NMTP_IOCTL_MAGIC, 2, unsigned long)
#define NMTP_MAX_SPEED_HZ  _IOW(NMTP_IOCTL_MAGIC, 3, unsigned long)

#define FRAME_SYNC_FIRST_BYTE			0xB6
#define FRAME_SYNC_SECOND_BYTE			0x49
#define FRAME_SYNC_IGNORE_LENGTH_BIT_SET	0xC3
#define FRAME_SYNC_LENGTH_BIT_SET		0x83
#define FRAME_SEGMENTORDER			0x60 /* MF is zero. Segment order is 11  */
#define FRAME_MF_SEGMENTORDER			0xE0 /* MF is one. Segment order is 11 */
#define FRAME_COUNTER_ORDER			0x2

#define RESYNC_FAILURE				0xFF

#define nmtp_warn(fmt, args ...) pr_warn("%s: "fmt, DEVTYPE, ## args)
#define nmtp_err(fmt, args ...)  pr_err("%s: "fmt, DEVTYPE, ## args)
#define nmtp_dbg(fmt, args ...)  pr_debug("%s: "fmt, DEVTYPE, ## args)


/* ProtocolIds */
#define EMPTY			0x0
#define	DEVICE_CONTROL		0x1
#define STANDARD_AUDIO		0x2
#define FM_RADIO		0x3
#define AM_RADIO		0x4
#define WX_RADIO		0x5
#define DAB_RADIO		0x6
#define DRM_RADIO		0x7
#define HD_RADIO		0x8
#define SIMULCAST		0xB

/* FIFO NUMBER for protocols */
#define DEVICE_CONTROL_FIFO		0x0
#define STANDARD_AUDIO_FIFO		0x1
#define FM_RADIO_FIFO			0x2
#define AM_RADIO_FIFO			0x3
#define WX_RADIO_FIFO			0x4
#define DAB_RADIO_FIFO			0x5
#define DRM_RADIO_FIFO			0x6
#define HD_RADIO_FIFO			0x7
#define SIMULCAST_FIFO			0x8

/* protocolID write limit */
#define DEVICE_CONTROL_LIMIT		1
#define STD_AUDIO_LIMIT			10
#define AR_LIMIT			10
#define DAB_LIMIT			3
#define DRM_LIMIT			3
#define HD_LIMIT			1
#define SIMULCAST_LIMIT			2

#define MAX_PRI_VAL			0x4

#define MAX_SYNC_COUNT			20
#define SYNC_DELAY			2

#define SWAP_BYTE32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))

struct nmtp_state {
	bool mode;
};

struct nmtp_data {
	dev_t				reset_devt;
	struct hrtimer			hrtimer;
	ktime_t				ktime;
	struct nmtp_state		*type;
	struct spi_device		*spi;
	struct spi_transfer		spi_txrx;
	struct spi_message		spi_msg;
	struct kfifo			fifo[NMTP_DEVNODES];
	struct kfifo			write_fifo;
	struct miscdevice		nmtp_misc[NMTP_DEVNODES + 2];
	char				name[25];
	u8				user_write;
	u8				*tx_buf;
	u8				*dummy_tx_frame;
	u8				*rx_buf;
	u8				write_count[NMTP_DEVNODES];
	u8				tx_frame_counter;
	u8				priority_arr[4];
	u8				priority_value;
	u8				capture_count;
	u32				speed_hz;
	u32				txlen;
	u8				*spidev_txbuf;
	u8				*spidev_rxbuf;
	spinlock_t			state_lock;
	wait_queue_head_t		data_wait[NMTP_DEVNODES];
	int				poll_count[NMTP_DEVNODES];
	struct completion		timer_exp;
	struct task_struct		*thread;
	struct delayed_work		resume_work, bwRequest_work;
	struct mutex			sync_lock, txrx_lock;
	struct list_head		device_entry;

	// Extra information for reset node processing
	struct mutex reset_lock;
	wait_queue_head_t reset_poll_wait;
	int reset_num_acks;
	int reset_num_reads;
	struct cdev reset_node_cdev;
	u8 reset_status;
	struct hrtimer 			reply_timer;

};

/* nmtp segment to User Space */
struct nmtp_segment {
	u8	sync[3];
	u8	priority;
	u32	length;
};

struct nmtp_intState {
	u8 reset_status;
	u8 previous_state; /* Hold Previous status of device*/
};

struct nmtp_payload_buf {
	u8 protocolId;
	u8 priority;
	u32 length;
};

/* FIFO lengths*/
#define FIFO_LOW_SEG 3072 	/* Memory is 3KiB  1024*3*/
#define FIFO_MID_SEG 10240	/* Memory is 10KiB 1024*10*/
#define FIFO_HIGH_SEG 13312	/* Memory is 13KiB 1024*13 */


int nmtp_add_dev(struct spi_device *spi, struct nmtp_data *nmtp);
int nmtp_spi_txrx(struct nmtp_data *nmtp_dev, bool user_write);
void nmtp_del_dev(struct nmtp_data *nmtp);
void crc_fill(u8 *data);
int nmtp_sync(struct nmtp_data *nmtp);
int nmtp_reset_init(struct nmtp_data *nmtp);
int nmtp_chip_reset(struct nmtp_data *nmtp, u8 status);
int nmtp_chip_primary_set_status(struct nmtp_data *nmtp, u8 status);
int nmtp_chip_secondary_set_status(struct nmtp_data *nmtp, u8 status);
int nmtp_sync(struct nmtp_data *nmtp);
void nmtp_reset_exit(struct nmtp_data *nmtp);
int nmtp_reset_ipc_op(struct nmtp_data *nmtp);
int nmtp_reset_ipc_query(struct nmtp_data *nmtp);
int nmtp_ipc_open(void);
int nmtp_ipc_close(void);
int nmtp_full_bandwidth_request(struct nmtp_data *nmtp);
void wait_for_apps_confirm(struct nmtp_data *nmtp);
int nmtp_secondary_reset_ipc_op(struct nmtp_data *nmtp);
int nmtp_secondary_query(struct nmtp_data *nmtp);
void nmtp_previous_state_update(u8 mode, u8 update);
void nmtp_current_state_update(u8 mode, u8 update);
struct nmtp_intState *get_nmtp_state(u8 mode);

struct nmtp_data *get_nmtp_details(u8 mode);

void update_nmtp_details(struct nmtp_data *data, u8 mode);

int fifo_read(struct nmtp_data *nmtp_dev, char __user *buf, size_t count, u8 fifoId);

// Start of added entries
#define RESET_CHIPS 1
#define WAIT_FOR_CHIPS_READY 2

int nmtp_reset_handler_start(void);
int nmtp_chip_reset_handler_initialize(void);
void nmtp_chip_reset_handler_exit(void);
void change_reset_state(int action);
void nmtp_reset_suspend(struct nmtp_data *nmtp);
void trigger_reset(void);

int nmtp_ipc_initialize(void);
int ipc_channel_init(void);
int nmtp_reset_ipc_query(struct nmtp_data *nmtp);
int nmtp_secondary_reset_ipc_op(struct nmtp_data *nmtp);
int nmtp_full_bandwidth_request(struct nmtp_data *nmtp);
void nmtp_ipc_exit(void);

// Put here for now
int ipc_channel_open(int ch_num);
int ipc_channel_close(int chnum);
int ipc_channel_read_timeout(int chnum, char *buf, size_t count, int timeout);
int ipc_channel_write(int chnum, const char *buf, size_t count);



#endif
