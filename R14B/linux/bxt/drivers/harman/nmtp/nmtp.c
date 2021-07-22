/*
 *  nmtp.c
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

#include "nmtp_plugin.h"
#include "nmtp_crc.h"
#define CREATE_TRACE_POINTS
#include <trace/events/nmtp_spi.h>

#define MAX_TUNERCHIPS                2

#define SINGLE_SEGMENT			0x3
#define INTERMEDIATE_SEGMENT		0x0
#define LAST_SEGMENT			0X1
#define FIRST_SEGMENT			0x2


static unsigned int nmtp_fifo_array[NMTP_DEVNODES]={
        [DEVICE_CONTROL_FIFO]        = FIFO_LOW_SEG,
        [STANDARD_AUDIO_FIFO]        = FIFO_LOW_SEG,
        [FM_RADIO_FIFO]              = FIFO_LOW_SEG,
        [AM_RADIO_FIFO]              = FIFO_LOW_SEG,
        [WX_RADIO_FIFO]               = FIFO_HIGH_SEG,
        [DAB_RADIO_FIFO]             = FIFO_HIGH_SEG,
        [DRM_RADIO_FIFO]             = FIFO_MID_SEG,
        [HD_RADIO_FIFO]              = FIFO_HIGH_SEG,
        [SIMULCAST_FIFO]             = FIFO_MID_SEG,
};



/* Global array for storing status **/
static struct nmtp_intState  nmtp_status[MAX_TUNERCHIPS];

/*Global array for storing nmtp structure */
static struct nmtp_data *nmtp_details[MAX_TUNERCHIPS] = {NULL, NULL};


void nmtp_previous_state_update(u8 mode, u8 update)
{
	if (mode < MAX_TUNERCHIPS) {
		spin_lock(&nmtp_details[mode]->state_lock);
		nmtp_status[mode].previous_state = update;
		spin_unlock(&nmtp_details[mode]->state_lock);
	}

}

void nmtp_current_state_update(u8 mode, u8 update)
{
	if (mode < MAX_TUNERCHIPS) {
		spin_lock(&nmtp_details[mode]->state_lock);
		nmtp_status[mode].reset_status = update;
		spin_unlock(&nmtp_details[mode]->state_lock);
	}
}

void update_nmtp_details(struct nmtp_data *data, u8 mode)
{
	if (mode < MAX_TUNERCHIPS)
		nmtp_details[mode] = data;

}
struct nmtp_data *get_nmtp_details(u8 mode)
{
	struct nmtp_data *ret = NULL;

	if (mode < MAX_TUNERCHIPS)
		ret = nmtp_details[mode];
	return ret;
}

void inc_poll_count(struct nmtp_data *nmtp, u32 fifoId)
{
	spin_lock(&nmtp->state_lock);
	nmtp->poll_count[fifoId]++;
	spin_unlock(&nmtp->state_lock);
}

void dec_poll_count(struct nmtp_data *nmtp, u32 fifoId)
{
	if (nmtp->poll_count[fifoId] > 0) {
		spin_lock(&nmtp->state_lock);
		nmtp->poll_count[fifoId]--;
		spin_unlock(&nmtp->state_lock);
	}
}

static void nmtp_fill_kfifo(struct nmtp_data *nmtp_dev)
{
	int payload_length, res, segment_length, fifo_length;
	u8 *rxbuf = nmtp_dev->rx_buf;
	u8 fifo_buf[MRD_FRAME_SIZE];
	u8 *datalink;
	u32 fifoId;
	u8 Notification_Command;
	u8 morefollowing, segment_order, frame_done, status;
	struct nmtp_segment app_segment;

	status = payload_length = res = frame_done = 0;

	/* BIT(6) is ignore bit */
	if ((rxbuf[3] & BIT(6)) == 0) {
		if (((rxbuf[4] & BIT(7)) == 0x0) && ((rxbuf[4] & 0x1F) == 0x0))
			return;
	} else
		return;

	status = crc_check(rxbuf);
	if (status) {
		dev_dbg_ratelimited(&nmtp_dev->spi->dev,
		"[Hnmtp] [%s] [%d] Error CRC check failed %d. Dropping Frame\n",
				__func__, __LINE__, rxbuf[2]);
		return;
	}

	datalink = &rxbuf[4];
	segment_length = 0;
	app_segment.sync[0] = 0xAB;
	app_segment.sync[1] = 0xCD;
	app_segment.sync[2] = 0xEF;
	app_segment.priority = 0x0;

	/* size less than 256 bytes;for safer side allocating 256 bytes */

	memset(fifo_buf, 0, MRD_FRAME_SIZE);
	do {
		datalink += segment_length;
		morefollowing = (datalink[0] & 0x80) >> 7;
		fifoId = (datalink[0] & 0x1F);

		if ((fifoId <  DEVICE_CONTROL) || (fifoId > SIMULCAST)) {
			dev_dbg_ratelimited(&nmtp_dev->spi->dev,
				"[Hnmtp] [%s] [%d] invalid protocolid %x exiting from loop\n",
				__func__, __LINE__, fifoId + 1);
			break;
		}
		/* ProtocolId and fifoId difference are handled here */
		if (fifoId == SIMULCAST)
			fifoId = SIMULCAST_FIFO;
		else
			fifoId -= 0x1;

		segment_order = (datalink[0] & 0x60) >> 5;
		Notification_Command = datalink[3] & 0x80;
		/* First Segment = 0x1 & Last Segment = 0x1: It's a single segment */
		switch (segment_order) {
			case SINGLE_SEGMENT:
				app_segment.priority = (datalink[1] >> 0x5) & 0x3;
				payload_length = (datalink[1] << 8) | datalink[2];
				/* extracting the 12bits with & 0xFFF */
				payload_length &= 0xFFF;
				app_segment.length = SWAP_BYTE32(payload_length);
				memcpy(fifo_buf, &app_segment, sizeof(app_segment));
				/* Payload data starts from datalink[3] */
				memcpy(fifo_buf + sizeof(app_segment),
							&datalink[3], payload_length);
				fifo_length = sizeof(app_segment) + payload_length;
				/* app_segment.length is payload length + \
			 	* 0x3 (first 3 bytes)
			 	*/
				segment_length = payload_length + 0x3;
				frame_done = 1;
				break;

				/* First Segment = 0x1 & Last Segment = 0x0:
		 		 * first segment of multiple segment
		 		 */
			case FIRST_SEGMENT:
				/* only for this segment payload starts from 7th byte */
				Notification_Command = datalink[7] & 0x80;
				app_segment.priority = datalink[1] & (0x3 << 0x4);
				payload_length  = datalink[3] << 24 |
					datalink[4] << 16 | datalink[5] << 8 |
					datalink[6];
				app_segment.length = SWAP_BYTE32(payload_length);
				segment_length = (datalink[1] << 8) | datalink[2];
				segment_length &= 0xFFF;
				memcpy(fifo_buf, &app_segment, sizeof(app_segment));
				memcpy(fifo_buf + sizeof(app_segment),
					&datalink[7], segment_length);
				/* segment_length have payload length + first 3 bytes */
				fifo_length = segment_length + sizeof(app_segment);
				segment_length += 0x3;
				break;


			case LAST_SEGMENT:
				segment_length = (datalink[1] << 8 | datalink[2]) & 0xFFF;
				memcpy(fifo_buf, &datalink[3], segment_length);
				fifo_length = segment_length;
				segment_length += 0x3;
				frame_done = 1;
				break;

			case INTERMEDIATE_SEGMENT:
			default:
				segment_length = (datalink[1] << 8 | datalink[2]) & 0xFFF;
				memcpy(fifo_buf, &datalink[3], segment_length);
				fifo_length = segment_length;
				segment_length += 0x3;
				break;
		}

		if (kfifo_avail(&nmtp_dev->fifo[fifoId]) < fifo_length) {
			dev_dbg_ratelimited(&nmtp_dev->spi->dev,
			"[Hnmtp] [%s] [%d] Error RXFIFO full, Packet dropped protoId %d\n",
			__func__, __LINE__, fifoId + 1);
			/* if kfifo is full, the packet is dropped */
		} else {
			res = kfifo_in(&nmtp_dev->fifo[fifoId], fifo_buf, fifo_length);
			if (frame_done) {
				frame_done = 0;
				inc_poll_count(nmtp_dev, fifoId);
				wake_up_interruptible(&nmtp_dev->data_wait[fifoId]);
			trace_nmtp_fill_kfifo(fifoId, nmtp_dev->name, morefollowing,
							segment_order, fifo_length);
			}
			if (nmtp_dev->write_count[fifoId] > 0) {
				if (!Notification_Command)
					nmtp_dev->write_count[fifoId]--;
			}
		}
	} while (morefollowing == 1);
}

int nmtp_sync(struct nmtp_data *nmtp)
{
	int ret, count, status;
	u8 txbuf[MRD_FRAME_SIZE], rxbuf[MRD_FRAME_SIZE];
	int pre_seq, post_seq, i;
	/* used for sequence check */
	struct spi_transfer xfers = {
			.tx_buf = txbuf,
			.rx_buf = rxbuf,
			.len	= MRD_FRAME_SIZE,
		};

	memset(txbuf, 0, MRD_FRAME_SIZE);
	memset(rxbuf, 0, MRD_FRAME_SIZE);
	pre_seq = post_seq = ret = count = 0;

	mutex_lock(&nmtp->sync_lock);

	for (count = 0; count < MAX_SYNC_COUNT; count++) {
		ret = spi_sync_transfer(nmtp->spi, &xfers, 1);
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] NMTP sync - Iteration:[%d] length:[%u] First two Bytes:[%x][%x]  -%s %d\n",
						       __func__, __LINE__, count, xfers.len, rxbuf[0], rxbuf[1], nmtp->name, __LINE__);

		if (ret) {
			nmtp_err("Failed to sync nmtp chip: %d\n", __LINE__);
			status = -EIO;
			goto sync_err;
		}
		if ((rxbuf[0] == FRAME_SYNC_FIRST_BYTE) &&
			(rxbuf[1] == FRAME_SYNC_SECOND_BYTE)) {
			dev_dbg(&nmtp->spi->dev,
				"[Hnmtp] [%s][%d] NMTP sync DONE Iteration [%d] length [%u]\n"
				"First two Bytes [%x] [%x] %s %d\n", __func__, __LINE__, count,
				xfers.len, rxbuf[0], rxbuf[1], nmtp->name, __LINE__);
			xfers.len = MRD_FRAME_SIZE;
			status = crc_check(rxbuf);
			if (!status)
				break;

		dev_err(&nmtp->spi->dev,
			"[Hnmtp] [%s] [%d] count %d CRC check failed\n",
					__func__, __LINE__, count);

		}
		/* case b6 is shifted to one byte forward*/
		else if ((rxbuf[0] == FRAME_SYNC_SECOND_BYTE)) {
			if (pre_seq == 0) {
				xfers.len = MRD_FRAME_SIZE-1;
				++pre_seq;
			} else
				xfers.len = MRD_FRAME_SIZE;
		}
		/* this is case where B6 49 is in middle of buffer*/
		else {
			if (post_seq == 0) {
				for (i = 0; i < MRD_FRAME_SIZE-1; i++) {
					if ((rxbuf[i] == FRAME_SYNC_FIRST_BYTE)
						&& (rxbuf[i+1] == FRAME_SYNC_SECOND_BYTE)) {
						xfers.len = i;
						++post_seq;
						break;
					}

				}
			} else
				xfers.len = MRD_FRAME_SIZE;
		}
		msleep(SYNC_DELAY);
	}

	if (count >= MAX_SYNC_COUNT) {
		dev_err(&nmtp->spi->dev,
			"[Hnmtp] [%s] [%d] Failure in NMTP sync %x %x %s %d\n",
			__func__, __LINE__, rxbuf[0], rxbuf[1],
			nmtp->name, __LINE__);

		status = -EIO;
		goto sync_err;
	}

	/* FrameSync: 0xB649 */
	txbuf[0] = FRAME_SYNC_FIRST_BYTE;
	txbuf[1] = FRAME_SYNC_SECOND_BYTE;
	xfers.len = MRD_FRAME_SIZE;
	/* FrameCounter */
	nmtp->tx_frame_counter = 0;

	/* Synced and length bit set */
	txbuf[3] = FRAME_SYNC_LENGTH_BIT_SET;

	for (count = 0; count < MAX_SYNC_COUNT; count++) {
		crc_fill(txbuf);
		ret = spi_sync_transfer(nmtp->spi, &xfers, 1);

		dev_dbg(&nmtp->spi->dev, "[Hnmtp] [%s] [%d] Txcount %x RX count %x\n"
				"length %u RX frames %x %x SyncByte %x\n", __func__, __LINE__,
				txbuf[2], rxbuf[2], xfers.len, rxbuf[0], rxbuf[1], rxbuf[3]);

		if (ret) {
			dev_err(&nmtp->spi->dev, "[Hnmtp] [%s] [%d] SPI Error\n"
				"Failed to get valid nmtp frame\n", __func__, __LINE__);
			status = -EIO;
			goto sync_err;
		}

		if (((rxbuf[3] & BIT(7)) == BIT(7)) && (rxbuf[2] > 0) &&
			(rxbuf[0] == FRAME_SYNC_FIRST_BYTE) &&
			(rxbuf[1] == FRAME_SYNC_SECOND_BYTE)) {
			status = crc_check(rxbuf);
			if (!status)
				break;

			dev_err(&nmtp->spi->dev,
				"[Hnmtp] [%s] [%d] count %d CRC check failed\n",
					__func__, __LINE__, count);
		}

		txbuf[2] = nmtp->tx_frame_counter++;
		msleep(SYNC_DELAY);
	}
	if (count >= MAX_SYNC_COUNT) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] [%s] [%d] Not received valid frame\n"
			"even after 20 iterations", __func__, __LINE__);
		status = -EIO;
		goto sync_err;
	}
	dev_dbg(&nmtp->spi->dev,
		"[Hnmtp] [%s] [%d] Synchronization is done Received valid frame\n"
		"First bit %x Second bit %x Sync bit %lu\n",
							 __func__, __LINE__, rxbuf[0], rxbuf[1], (rxbuf[3] & BIT(7)));
	status = 0;
sync_err:
	mutex_unlock(&nmtp->sync_lock);

	return status;
}

static int payload_append_write(struct nmtp_data *nmtp_dev)
{
	struct nmtp_payload_buf write_buf;
	u32 copied, length, count, write_buf_length;
	u8 *payload;

	payload =  nmtp_dev->tx_buf;

	memset(payload, 0, MRD_FRAME_SIZE);

	payload[0] = FRAME_SYNC_FIRST_BYTE;
	payload[1] = FRAME_SYNC_SECOND_BYTE;
	payload[3] = FRAME_SYNC_LENGTH_BIT_SET;

	count = 4;
	length = 0;
	write_buf_length = sizeof(write_buf);

	while (kfifo_len(&nmtp_dev->write_fifo) > write_buf_length) {
		copied = kfifo_out(&nmtp_dev->write_fifo, &write_buf,
						write_buf_length);
		if (copied == write_buf_length) {
			payload[count]     = write_buf.protocolId;
			payload[count + 1] = 0x0;
			payload[count + 2] = write_buf.length & 0xFF;
			if ((kfifo_len(&nmtp_dev->write_fifo)) >=
						write_buf.length)	{
				copied = kfifo_out(&nmtp_dev->write_fifo,
					payload + count + 3, write_buf.length);
				if (copied == write_buf.length)	{

					length = count +  write_buf.length + 3;

					if ((write_buf.priority < MAX_PRI_VAL) &&
					(nmtp_dev->priority_arr[write_buf.priority] > 0))
						nmtp_dev->priority_arr[write_buf.priority]--;
					if ((kfifo_peek_len(&nmtp_dev->write_fifo) == 0) ||
						((length + kfifo_peek_len(&nmtp_dev->write_fifo)) >= 250))	{
						payload[count] |= FRAME_SEGMENTORDER;
						break;
					}
					payload[count] |= FRAME_MF_SEGMENTORDER;
					count = length;
				} else{
					dev_err(&nmtp_dev->spi->dev,
						"[Hnmtp] [%s] [%d] Copied Buffer doesnt match\n"
						"requested buff\n req = %d buf copied = %d",
						__func__, __LINE__, write_buf.length, copied);
					break;
				}
			} else {
				dev_err(&nmtp_dev->spi->dev,
					"[Hnmtp] [%s] [%d] Requested Kfifo Buffer not present\n"
					"buffer requested = %d buffer available = %d",
					__func__, __LINE__, write_buf.length,
					kfifo_len(&nmtp_dev->write_fifo));
				break;
			}
		}

	}
	return length;
}

int nmtp_spi_txrx(struct nmtp_data *nmtp_dev, bool user_write)
{
	int status;
	u8 *txbuf, *rxbuf;

	if (!nmtp_dev)
		return -EINVAL;

	if (kfifo_is_empty(&nmtp_dev->write_fifo)) {
		txbuf = nmtp_dev->dummy_tx_frame;
	} else {
		status = payload_append_write(nmtp_dev);
		if (status <= 4)
			txbuf = nmtp_dev->dummy_tx_frame;
		else
			txbuf = nmtp_dev->tx_buf;
	}
	rxbuf = nmtp_dev->rx_buf;
	txbuf[2] = nmtp_dev->tx_frame_counter++;
	crc_fill(txbuf);

	nmtp_dev->spi_txrx.tx_buf = txbuf;
	nmtp_dev->spi_txrx.rx_buf = rxbuf;
	nmtp_dev->spi_txrx.len =  MRD_FRAME_SIZE;

	spi_message_init(&nmtp_dev->spi_msg);
	spi_message_add_tail(&nmtp_dev->spi_txrx, &nmtp_dev->spi_msg);
	status = spi_sync(nmtp_dev->spi, &nmtp_dev->spi_msg);

	if (status) {
		dev_dbg(&nmtp_dev->spi->dev,
		"[Hnmtp][%s][%d] Error spi_sync : spi read and write failed [%d]\n",
		__func__, __LINE__, status);
		goto ret;
	}
	if ((rxbuf[0] != FRAME_SYNC_FIRST_BYTE) || (rxbuf[1] !=
		FRAME_SYNC_SECOND_BYTE) || ((rxbuf[3] & BIT(7)) != BIT(7))) {
		dev_dbg(&nmtp_dev->spi->dev,
		"[Hnmtp] [%s] [%d] NMTP chip needs re-synchronization mode %d :\n",
		__func__, __LINE__, nmtp_dev->type->mode);
		dev_dbg(&nmtp_dev->spi->dev,
		"[Hnmtp] [%s] [%d] First & Second byte %x %x Sync Bit = %x\n",
						__func__, __LINE__, rxbuf[0], rxbuf[1], (rxbuf[3] & (1 << 7)));
		status = nmtp_sync(nmtp_dev);
		if (status) {
			trigger_reset();
		}
	} else
		nmtp_fill_kfifo(nmtp_dev);

ret:

	return status;
}

static enum hrtimer_restart nmtp_timer_poll(struct hrtimer *hrtimer)
{
	struct nmtp_data *nmtp_dev = container_of(hrtimer,
				struct nmtp_data, hrtimer);

	complete(&nmtp_dev->timer_exp);

	if (nmtp_dev->priority_arr[0] > 0) {
		nmtp_dev->priority_arr[0]--;
		nmtp_dev->ktime = ktime_set(0,
			DELAY_MS(EXT_HIGH_PRIORITY_POLL_DEALY));
	} else if (nmtp_dev->priority_arr[1] > 0) {
		nmtp_dev->ktime = ktime_set(0,
			DELAY_MS(HIGH_PRIORITY_POLL_DEALY));
			nmtp_dev->priority_value = 0x1;
	} else if (nmtp_dev->priority_arr[2] > 0) {
		nmtp_dev->ktime = ktime_set(0,
			DELAY_MS(MID_PRIORITY_POLL_DEALY));
		nmtp_dev->priority_value = 0x2;
	} else if (nmtp_dev->priority_arr[3] > 0) {
		nmtp_dev->ktime = ktime_set(0,
			DELAY_MS(DEFAULT_PRIORITY_POLL_DEALY));
		nmtp_dev->priority_value = 0x3;
	} else {
		nmtp_dev->ktime = ktime_set(0,
			DELAY_MS(DEFAULT_PRIORITY_POLL_DEALY));
		nmtp_dev->priority_value = 0x3;
	}

	hrtimer_forward(hrtimer, hrtimer_cb_get_time(hrtimer), nmtp_dev->ktime);

	return HRTIMER_NORESTART;
}

static int payload_complete_status(struct nmtp_data *nmtp_dev, u8 fifoId)
{
	int ret;
	u32 length;
	struct nmtp_segment segment_len;

	ret = kfifo_out_peek(&nmtp_dev->fifo[fifoId], &segment_len, sizeof(segment_len));
	if (ret <= 0) {
		dev_dbg_ratelimited(&nmtp_dev->spi->dev,
					"[Hnmtp] Error in kfifo out peek : %s %d\n",
							__func__, __LINE__);
		return -EAGAIN;
	}

	if ((segment_len.sync[0] != 0xAB) && (segment_len.sync[1] != 0xCD) &&
			(segment_len.sync[2] != 0xEF)) {
		dev_dbg_ratelimited(&nmtp_dev->spi->dev, "[Hnmtp] Error : Not in Sync %s %d\n",
							__func__, __LINE__);
		return -EFAULT;
	}

	length = segment_len.length;
	/* Temporary fix: need to get fix from Tuner */
	length = (SWAP_BYTE32(length) & 0xFFF) + sizeof(segment_len);


	if (length <= kfifo_len(&nmtp_dev->fifo[fifoId]))
		ret = length;
	else
		ret = -ENOMSG;

	return ret;
}

int fifo_read(struct nmtp_data *nmtp_dev, char __user *buf, size_t count, u8 fifoId)
{
	ssize_t status;
	u32 copied;
	int payload_count;

	status = copied = 0;

	payload_count = payload_complete_status(nmtp_dev, fifoId);
	if (payload_count  <= 0) {
		return payload_count;
	}
	if (payload_count > count) {
		dev_dbg_ratelimited(&nmtp_dev->spi->dev,
		"[Hnmtp][%s][%d] insufficient memory from App count %d paylod count %d\n",
						__func__, __LINE__, count, payload_count);
		return -ENOMEM;
	}
	dec_poll_count(nmtp_dev, fifoId);

	status = kfifo_to_user(&nmtp_dev->fifo[fifoId], buf, payload_count, &copied);
	if (status) {
		dev_err(&nmtp_dev->spi->dev, "[Hnmtp]%d kfifo read failed for fifoId: %d\n",
							__LINE__, fifoId);
		copied = status;
	}
	return copied;
}

static int nmtp_queue_thread(void *d)
{
	struct nmtp_data *nmtp = d;
	int ret, status;

	init_completion(&nmtp->timer_exp);
	allow_signal(SIGTERM);

	while (1) {
		if (kthread_should_stop()) {
			dev_dbg(&nmtp->spi->dev,
			"[Hnmtp] -- [%s] [%d] Thread %s exiting\n",
								__func__, __LINE__, __func__);
			break;
		}
		ret = wait_for_completion_interruptible(&nmtp->timer_exp);
		if (!ret) {
			u8 reset_status;

			mutex_lock(&nmtp->reset_lock);
			reset_status = nmtp->reset_status;
			mutex_unlock(&nmtp->reset_lock);
			if ((reset_status == NMTP_NORMAL_OPERATION)
			 || (reset_status == NMTP_REQUEST_RESET_PENDING)) {
				if (nmtp->capture_count == 0) {
					dev_err(&nmtp->spi->dev, "[Hnmtp] [%s] [%d]\n",
									__func__, __LINE__);
				} else {
					mutex_lock(&nmtp->txrx_lock);
					status = nmtp_spi_txrx(nmtp, false);
					if (status) {
						dev_err(&nmtp->spi->dev, " [Hnmtp] -- [%s] [%d] Error: nmtp error\n",
										__func__, __LINE__);
					}
					mutex_unlock(&nmtp->txrx_lock);
					/* no need to restart in case of resync failure */
					if (status != RESYNC_FAILURE) {
						hrtimer_restart(&nmtp->hrtimer);
					}
				}
			} else
				hrtimer_restart(&nmtp->hrtimer);

		}
	}
	return 0;
}


static void  nmtp_add_kthreads(struct nmtp_data *nmtp)
{

	if (nmtp->type->mode == 0)
		nmtp->thread = kthread_run(nmtp_queue_thread, nmtp,
								 "nmtp_master");
	else
		nmtp->thread = kthread_run(nmtp_queue_thread, nmtp,
								"nmtp_slave");
}

static int nmtp_alloc_res(struct nmtp_data *nmtp)
{
	int status, i;


	for (i = 0, status = 0; i < NMTP_DEVNODES; i++) {
		status = kfifo_alloc(&nmtp->fifo[i], nmtp_fifo_array[i], GFP_KERNEL);
		if (status) {
			dev_err(&nmtp->spi->dev,
				"[Hnmtp] [%s] [%d] kfifo alloc failed\n",
				__func__, __LINE__);
			i--;
			goto err_write_alloc;
		}
		nmtp->poll_count[i] = 0;
	}

	status  = kfifo_alloc(&nmtp->write_fifo, WRITE_FIFO_SIZE, GFP_KERNEL);

	if (status) {
		dev_err(&nmtp->spi->dev,
		"[Hnmtp] [%s] [%d] kfifo alloc failed\n", __func__, __LINE__);
		goto err_write_alloc;
	}
	nmtp->tx_buf = kzalloc(MRD_FRAME_SIZE, GFP_KERNEL);
	if (!nmtp->tx_buf)
		goto err_txbuf_alloc;

	nmtp->rx_buf = kzalloc(MRD_FRAME_SIZE, GFP_KERNEL);
	if (!nmtp->rx_buf)
		goto err_rxbuf_alloc;
	nmtp->dummy_tx_frame = kzalloc(MRD_FRAME_SIZE, GFP_KERNEL);
	if (!nmtp->dummy_tx_frame)
		goto err_dummy_buf_alloc;

	nmtp->dummy_tx_frame[0] = FRAME_SYNC_FIRST_BYTE;
	nmtp->dummy_tx_frame[1] = FRAME_SYNC_SECOND_BYTE;
	nmtp->tx_frame_counter = 0;
	nmtp->dummy_tx_frame[3] = FRAME_SYNC_IGNORE_LENGTH_BIT_SET;

	return status;

err_dummy_buf_alloc:
	kfree(nmtp->rx_buf);
err_rxbuf_alloc:
	kfree(nmtp->tx_buf);
err_txbuf_alloc:
	kfree(&nmtp->write_fifo);
err_write_alloc:
	while (i >= 0) {
		kfree(&nmtp->fifo[i]);
		i--;
	}

	return -ENOMEM;
}

static const struct of_device_id nmtp_of_match[] = {
	{ .compatible = "nmtp_spi_master", .data = NULL,},
	{ .compatible = "nmtp_spi_slave",  .data = NULL,},
	{}
};
MODULE_DEVICE_TABLE(of, nmtp_of_match);

static int nmtp_spi_probe(struct spi_device *spi)
{
	int status, i;
	u8 mode = 0;
	struct nmtp_data *nmtp_dev;
	const struct of_device_id *of_id =
			of_match_device(nmtp_of_match, &spi->dev);

	nmtp_dev = kzalloc(sizeof(*nmtp_dev), GFP_KERNEL);
	if (nmtp_dev == NULL)
		return -ENOMEM;

	/* Initialize driver data */
	nmtp_dev->spi = spi;
	mutex_init(&nmtp_dev->sync_lock);
	mutex_init(&nmtp_dev->txrx_lock);
	spin_lock_init(&nmtp_dev->state_lock);

	for (i = 0; i < NMTP_DEVNODES; i++)
		init_waitqueue_head(&nmtp_dev->data_wait[i]);

	nmtp_dev->speed_hz = spi->max_speed_hz;

	if (of_id)
		nmtp_dev->type = (struct nmtp_state *) of_id->data;
	else
		nmtp_dev->type = dev_get_platdata(&spi->dev);

	if (!nmtp_dev->type) {
		dev_err(&nmtp_dev->spi->dev,
		"[Hnmtp] [%s] [%d] Failed to get pdata\n",
								__func__, __LINE__);
		return -EINVAL;
	}
	mode = nmtp_dev->type->mode;
	status = nmtp_alloc_res(nmtp_dev);
	if (status)
		goto err_dev;
	status = nmtp_add_dev(spi, nmtp_dev);
	if (status)
		goto err_dev;

	nmtp_add_kthreads(nmtp_dev);

	/* Setting it to default value */
	for (i = 0; i < 4; i++)
		nmtp_dev->priority_arr[i] = 0;
	nmtp_dev->priority_value = 0x3;
	nmtp_dev->capture_count = 0;

	hrtimer_init(&nmtp_dev->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	nmtp_dev->ktime = ktime_set(0, DELAY_MS(DEFAULT_PRIORITY_POLL_DEALY));
	nmtp_dev->hrtimer.function =  &nmtp_timer_poll;
	nmtp_dev->user_write = false;
	spi_set_drvdata(spi, nmtp_dev);

	status = nmtp_reset_init(nmtp_dev);
	update_nmtp_details(nmtp_dev, nmtp_dev->type->mode);

	nmtp_dev->reset_status = NMTP_ACKED_RESET_IN_PROGRESS;
	nmtp_dev->reset_num_acks = 0;

	/* mode = 0 is master & mode = 1 is slave */
	if (nmtp_dev->type->mode == 0) {
		dev_dbg(&nmtp_dev->spi->dev, "[Hnmtp] Resume Primary tuner [%s] [%d]\n", __func__, __LINE__);

		change_reset_state(WAIT_FOR_CHIPS_READY);
	}

	return 0;
	//return status;

err_dev:
	mutex_destroy(&nmtp_dev->sync_lock);
	mutex_destroy(&nmtp_dev->txrx_lock);
	update_nmtp_details(NULL, nmtp_dev->type->mode);
	kfree(nmtp_dev);
	return status;
}

static void nmtp_dealloc_res(struct nmtp_data *nmtp)
{
	int i;

	for (i = 0; i < NMTP_DEVNODES; i++)
		kfifo_free(&nmtp->fifo[i]);
	kfifo_free(&nmtp->write_fifo);

	kfree(nmtp->tx_buf);
	kfree(nmtp->rx_buf);
	kfree(nmtp->dummy_tx_frame);
}

static int nmtp_spi_remove(struct spi_device *dev)
{
	struct nmtp_data *nmtp = spi_get_drvdata(dev);

	hrtimer_cancel(&nmtp->hrtimer);
	send_sig(SIGTERM, nmtp->thread, 1);
	kthread_stop(nmtp->thread);
	mutex_destroy(&nmtp->sync_lock);
	mutex_destroy(&nmtp->txrx_lock);
	nmtp_reset_exit(nmtp);
	nmtp_del_dev(nmtp);
	nmtp_dealloc_res(nmtp);
	kfree(nmtp);
	return 0;
}


static int  nmtp_suspend(struct device *dev)
{
	struct nmtp_data *nmtp;
	struct spi_device *spi_dev;

	spi_dev = container_of(dev, struct spi_device, dev);
	nmtp = spi_get_drvdata(spi_dev);

	nmtp_reset_suspend(nmtp);
	return 0;
}

static int nmtp_resume(struct device *dev)
{
	struct spi_device *spi_dev;
	struct nmtp_data *nmtp;

	spi_dev = container_of(dev, struct spi_device, dev);
	nmtp = spi_get_drvdata(spi_dev);

	nmtp->reset_status = NMTP_RESUME_STATE;
	wake_up_interruptible(&nmtp->reset_poll_wait);

	/* mode = 0 is master & mode = 1 is slave */
	if (nmtp->type->mode == 0) {
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] Resume Primary tuner [%s] [%d] \n", __func__, __LINE__);
		change_reset_state(WAIT_FOR_CHIPS_READY);
	}

	return 0;
}

static struct dev_pm_ops nmtp_tuner_dev_pm_ops = {
	.suspend = nmtp_suspend,
	.resume = nmtp_resume,
};

static struct spi_driver nmtp_spi_driver = {
	.driver			= {
		.name		= "nmtp_spi",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(nmtp_of_match),
		.pm		= &nmtp_tuner_dev_pm_ops,
	},
	.probe			= nmtp_spi_probe,
	.remove			= nmtp_spi_remove,
};

static int __init nmtp_init(void)
{
	return spi_register_driver(&nmtp_spi_driver);
}
late_initcall(nmtp_init);


static void __exit nmtp_spi_exit(void)
{
	spi_unregister_driver(&nmtp_spi_driver);
}
module_exit(nmtp_spi_exit);

MODULE_AUTHOR("Akshay Shankarmurthy <Akshya.Shankarmurthy@harman.com>");
MODULE_AUTHOR("Vivek Pernamitta <Vivek.Pernamitta@harman.com>");
MODULE_AUTHOR("Josmon Paul <Josmon.Paul@harman.com>");
MODULE_DESCRIPTION("Driver for NMTP with SPI interface");
MODULE_LICENSE("GPL");
