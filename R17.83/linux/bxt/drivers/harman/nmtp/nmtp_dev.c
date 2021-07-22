/*
 *  nmtp_dev.c
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

#define CREATE_TRACE_POINTS
#define SPI_PLAINMODE_DEVSIZE  4096

static DEFINE_MUTEX(payload_write_lock);

static ssize_t device_control_read(struct file *f, char __user *buf,
				size_t count, loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int ret = 0;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[1]);

	if (!nmtp_dev)
		return -ECANCELED;

	ret = fifo_read(nmtp_dev, buf, count, DEVICE_CONTROL_FIFO);

	dev_dbg(&nmtp_dev->spi->dev,
	"[Hnmtp] [%s] [%d] %s copied %d UpNotification\n",
					__func__, __LINE__, nmtp_dev->name, ret);
	return ret;
}

static int payload_extract(const char __user *buf, size_t count,
			struct nmtp_data *nmtp_dev, u8 protocolId, u8 fifoId)
{
	u8 length;
	u8 user_data[count], priority;
	u8 *txbuf;
	u8 payload[NMTP_FRAME_SIZE];
	u8 mode;
	struct nmtp_segment write_us_data;
	int total_length, status;
	struct nmtp_payload_buf write_buf;

	mode = nmtp_dev->type->mode;

	mutex_lock(&nmtp_dev->reset_lock);
	if ((nmtp_dev->reset_status == NMTP_UNACKED_RESUME_RESET_PENDING)
	 || (nmtp_dev->reset_status == NMTP_ACKED_RESUME_RESET_PENDING)
	 || (nmtp_dev->reset_status == NMTP_NORMAL_RESUME_RESET_PENDING)
	 || (nmtp_dev->reset_status == NMTP_SPIDEV_MODE)) {
		mutex_unlock(&nmtp_dev->reset_lock);
		return 0;
	}

	if ((nmtp_dev->reset_status != NMTP_NORMAL_OPERATION)
	 && (nmtp_dev->reset_status != NMTP_REQUEST_RESET_PENDING)) {
		dev_err(&nmtp_dev->spi->dev, "[hnmtp] -- [%s] [%d] Failure in write: status %d: fido id %d: nmtp is not in Normal operation\n",
				__func__, __LINE__, nmtp_dev->reset_status, fifoId);
		mutex_unlock(&nmtp_dev->reset_lock);
		return 0;
	}
	mutex_unlock(&nmtp_dev->reset_lock);
	if (nmtp_dev->capture_count == 0) {
		dev_err(&nmtp_dev->spi->dev, "[hnmtp] [%s] [%d]  PID : %d\n",
					__func__, __LINE__, task_pid_nr(current));
		return -EAGAIN;
	}

	status = copy_from_user(user_data, buf, count);
	if (status)
		return -ENOMEM;

	priority = total_length = 0;
	mutex_lock(&payload_write_lock);

	while ((total_length+sizeof(write_us_data)) < count) {
		txbuf = user_data + total_length;

		memcpy(&write_us_data, txbuf, sizeof(write_us_data));

		if ((write_us_data.sync[0] != 0xAB)
			&& (write_us_data.sync[1] != 0xCD)
			&& (write_us_data.sync[2] != 0xEF)) {
			dev_err(&nmtp_dev->spi->dev,
				"[hnmtp] [%s] [%d] lost from application %x %x %x\n",
				__func__, __LINE__, write_us_data.sync[0],
				write_us_data.sync[1], write_us_data.sync[2]);
			return -EINVAL;
		}

		length = SWAP_BYTE32(write_us_data.length) & 0xFF;
		total_length += (length + sizeof(write_us_data));
		write_buf.protocolId = protocolId;
		write_buf.length = length;
		write_buf.priority = write_us_data.priority & 0xFF;

		(write_buf.priority < MAX_PRI_VAL) ?
		nmtp_dev->priority_arr[write_buf.priority]++ : nmtp_dev->priority_arr[3]++;
		if (nmtp_dev->priority_value > write_buf.priority) {
			priority = 1;
			nmtp_dev->priority_value  = write_buf.priority;
		}
		memcpy(payload, &write_buf, sizeof(write_buf));
		if (total_length + length <= count) {
			dev_err(&nmtp_dev->spi->dev,
				"[hnmtp] [%s] [%d] payload Not present for packet\n"
				"protocolID : %02x length Requested : %d",
				__func__, __LINE__, protocolId, length);
			return -EINVAL;
		}
		memcpy(payload + sizeof(write_buf),
				txbuf + sizeof(write_us_data), length);
		kfifo_in(&nmtp_dev->write_fifo, payload,
				length + sizeof(write_buf));
		if (write_us_data.priority == 0) {
			priority = 1;
			nmtp_dev->priority_arr[0] += 4;
		}

		nmtp_dev->write_count[fifoId]++;
	}
	if (priority) {
		hrtimer_cancel(&nmtp_dev->hrtimer);
		if (nmtp_dev->priority_value == 0x3)
			nmtp_dev->ktime = ktime_set(0,
						DELAY_MS(DEFAULT_PRIORITY_POLL_DEALY));
		else if (nmtp_dev->priority_value == 0x2)
			nmtp_dev->ktime = ktime_set(0,
						DELAY_MS(MID_PRIORITY_POLL_DEALY));
		else if (nmtp_dev->priority_value == 0x1)
			nmtp_dev->ktime = ktime_set(0,
						DELAY_MS(HIGH_PRIORITY_POLL_DEALY));

		complete(&nmtp_dev->timer_exp);

	}
	mutex_unlock(&payload_write_lock);

	return 0;
}

static ssize_t device_control_write(struct file *f, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[1]);

	if (nmtp_dev == NULL)
		return -ESHUTDOWN;
	if (count > WRITE_BUF_LIMIT)
		return -EINVAL;
	if (kfifo_avail(&nmtp_dev->write_fifo) < count)
		return -EAGAIN;
	dev_dbg(&nmtp_dev->spi->dev,
		"[hnmtp] [%s] [%d] %s write count %d\n", __func__, __LINE__,
		nmtp_dev->name, nmtp_dev->write_count[DEVICE_CONTROL_FIFO]);

	if (nmtp_dev->write_count[DEVICE_CONTROL_FIFO] >= DEVICE_CONTROL_LIMIT) {
		dev_dbg(&nmtp_dev->spi->dev,
			"[hnmtp] [%s] [%d] %s Write limit failed count\n",
						__func__, __LINE__,  nmtp_dev->name);
		return -EAGAIN;
	}

	status = payload_extract(buf, count, nmtp_dev, DEVICE_CONTROL,
								DEVICE_CONTROL_FIFO);

	return (status) ? status : count;
}

static int device_control_open(struct inode *inode, struct file *f)
{
	return 0;
}

static unsigned int device_control_poll(struct file *f,
			struct poll_table_struct *wait)
{
	int mask = 0;
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[1]);

	poll_wait(f, &nmtp_dev->data_wait[DEVICE_CONTROL_FIFO], wait);

	if (nmtp_dev->poll_count[DEVICE_CONTROL_FIFO] > 0)
		mask =  POLLIN | POLLRDNORM;

	return mask;
}

static int std_audio_open(struct inode *inode, struct file *f)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[2]);

	dev_dbg(&nmtp_dev->spi->dev, "[Hnmtp] [%s] [%d]\n",
		__func__, __LINE__);

	return 0;
}

static ssize_t std_audio_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[2]);
	if (!nmtp_dev)
		return -ECANCELED;

	dev_dbg_ratelimited(&nmtp_dev->spi->dev, "[Hnmtp] [%s] [%d]\n",
		__func__, __LINE__);

	return fifo_read(nmtp_dev, buf, count, STANDARD_AUDIO_FIFO);

}

static unsigned int std_audio_poll(struct file *f,
			struct poll_table_struct *wait)
{
	int mask = 0;
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[2]);

	poll_wait(f, &nmtp_dev->data_wait[STANDARD_AUDIO_FIFO], wait);

	if (nmtp_dev->poll_count[STANDARD_AUDIO_FIFO] > 0)
		mask =  POLLIN | POLLRDNORM;

	return mask;
}

static ssize_t std_audio_write(struct file *f, const char __user *buf,
			size_t count, loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[2]);

	if (nmtp_dev == NULL) {
		dev_dbg(&nmtp_dev->spi->dev, "[Hnmtp] -- [%s] [%d]\n",
						__func__, __LINE__);
		return -ESHUTDOWN;
	}
	if (count > WRITE_BUF_LIMIT) {
		dev_dbg(&nmtp_dev->spi->dev, "[Hnmtp] -- [%s] [%d]\n",
						__func__, __LINE__);
		return -EINVAL;
	}

	if (kfifo_avail(&nmtp_dev->write_fifo) < count) {
		dev_dbg(&nmtp_dev->spi->dev, "[Hnmtp] -- [%s] [%d]\n",
						__func__, __LINE__);

		return -EAGAIN;
	}

	if (nmtp_dev->write_count[STANDARD_AUDIO_FIFO] >= STD_AUDIO_LIMIT) {
	dev_dbg(&nmtp_dev->spi->dev, "[Hnmtp] -- [%s] [%d]\n",
						__func__, __LINE__);

		return -EAGAIN;
	}

	status = payload_extract(buf, count, nmtp_dev,
		STANDARD_AUDIO, STANDARD_AUDIO_FIFO);
	dev_dbg_ratelimited(&nmtp_dev->spi->dev, "[Hnmtp] [%s] [%d] status %d\n",
						__func__, __LINE__, status);

	return (status) ? status : count;
}

static int fm_open(struct inode *inode, struct file *f)
{
	return 0;
}

static ssize_t fm_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[3]);

	if (!nmtp_dev)
		return -ECANCELED;

	return fifo_read(nmtp_dev, buf, count, FM_RADIO_FIFO);

}

static unsigned int fm_poll(struct file *f, struct poll_table_struct *wait)
{
	int mask = 0;
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[3]);

	poll_wait(f, &nmtp_dev->data_wait[FM_RADIO_FIFO], wait);

	if (nmtp_dev->poll_count[FM_RADIO_FIFO] > 0)
		mask =  POLLIN | POLLRDNORM;

	return mask;
}

static ssize_t fm_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[3]);

	if (nmtp_dev == NULL)
		return -ESHUTDOWN;
	if (count > WRITE_BUF_LIMIT)
		return -EINVAL;

	if (kfifo_avail(&nmtp_dev->write_fifo) < count)
		return -EAGAIN;
	if ((nmtp_dev->write_count[AM_RADIO_FIFO] +
			nmtp_dev->write_count[WX_RADIO_FIFO] +
			nmtp_dev->write_count[FM_RADIO_FIFO]) >= AR_LIMIT)
		return -EAGAIN;

	status = payload_extract(buf, count, nmtp_dev, FM_RADIO, FM_RADIO_FIFO);

	return (status) ? status : count;
}

static int am_open(struct inode *inode, struct file *f)
{
	return 0;
}
static ssize_t am_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[4]);
	if (!nmtp_dev)
		return -ECANCELED;

	return fifo_read(nmtp_dev, buf, count, AM_RADIO_FIFO);

}

static unsigned int am_poll(struct file *f, struct poll_table_struct *wait)
{
	int mask = 0;
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[4]);

	poll_wait(f, &nmtp_dev->data_wait[AM_RADIO_FIFO], wait);

	 if (nmtp_dev->poll_count[AM_RADIO_FIFO] > 0)
		mask =  POLLIN | POLLRDNORM;

	return mask;
}

static ssize_t am_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[4]);

	if (nmtp_dev == NULL)
		return -ESHUTDOWN;

	if (count > WRITE_BUF_LIMIT)
		return -EINVAL;

	if (kfifo_avail(&nmtp_dev->write_fifo) < count)
		return -EAGAIN;

	if ((nmtp_dev->write_count[AM_RADIO_FIFO] +
			nmtp_dev->write_count[WX_RADIO_FIFO] +
			nmtp_dev->write_count[FM_RADIO_FIFO]) >= AR_LIMIT)
		return -EAGAIN;

	status = payload_extract(buf, count, nmtp_dev, AM_RADIO, AM_RADIO_FIFO);

	return (status) ? status : count;
}

static int wx_open(struct inode *inode, struct file *f)
{
	return 0;
}
static ssize_t wx_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[5]);
	if (!nmtp_dev)
		return -ECANCELED;

	return fifo_read(nmtp_dev, buf, count, WX_RADIO_FIFO);

}

static unsigned int wx_poll(struct file *f, struct poll_table_struct *wait)
{
	int mask = 0;
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[5]);

	poll_wait(f, &nmtp_dev->data_wait[WX_RADIO_FIFO], wait);

	if (nmtp_dev->poll_count[WX_RADIO_FIFO] > 0)
		mask =  POLLIN | POLLRDNORM;

	return mask;
}

static ssize_t wx_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[5]);

	if (nmtp_dev == NULL)
		return -ESHUTDOWN;

	if (count > WRITE_BUF_LIMIT)
		return -EINVAL;

	if (kfifo_avail(&nmtp_dev->write_fifo) < count)
		return -EAGAIN;

	if ((nmtp_dev->write_count[AM_RADIO_FIFO] +
			nmtp_dev->write_count[WX_RADIO_FIFO] +
			nmtp_dev->write_count[FM_RADIO_FIFO]) >= AR_LIMIT)
		return -EAGAIN;

	status = payload_extract(buf, count, nmtp_dev, WX_RADIO, WX_RADIO_FIFO);

	return (status) ? status : count;
}

static int dab_open(struct inode *inode, struct file *f)
{
	return 0;
}
static ssize_t dab_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[6]);
	if (!nmtp_dev)
		return -ECANCELED;

	return fifo_read(nmtp_dev, buf, count, DAB_RADIO_FIFO);
}


static unsigned int dab_poll(struct file *f, struct poll_table_struct *wait)
{
	int mask = 0;
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[6]);

	poll_wait(f, &nmtp_dev->data_wait[DAB_RADIO_FIFO], wait);

	if (nmtp_dev->poll_count[DAB_RADIO_FIFO] > 0)
		mask =  POLLIN | POLLRDNORM;


	return mask;
}

static ssize_t dab_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[6]);

	if (nmtp_dev == NULL)
		return -ESHUTDOWN;
	if (count > WRITE_BUF_LIMIT)
		return -EINVAL;

	if (kfifo_avail(&nmtp_dev->write_fifo) < count)
		return -EAGAIN;

	if (nmtp_dev->write_count[DAB_RADIO_FIFO] >=  DAB_LIMIT)
		return -EAGAIN;

	status = payload_extract(buf, count, nmtp_dev,
			DAB_RADIO, DAB_RADIO_FIFO);

	return (status) ? status : count;
}

static ssize_t drm_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[7]);
	if (!nmtp_dev)
		return -ECANCELED;

	return fifo_read(nmtp_dev, buf, count, DRM_RADIO_FIFO);
}
static int drm_open(struct inode *inode, struct file *f)
{
	return 0;
}

static unsigned int drm_poll(struct file *f, struct poll_table_struct *wait)
{
	int mask = 0;
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[7]);

	poll_wait(f, &nmtp_dev->data_wait[DRM_RADIO_FIFO], wait);

	if (nmtp_dev->poll_count[DRM_RADIO_FIFO] > 0)
		mask =  POLLIN | POLLRDNORM;

	return mask;
}

static ssize_t drm_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[7]);

	if (nmtp_dev == NULL)
		return -ESHUTDOWN;
	if (count > WRITE_BUF_LIMIT)
		return -EINVAL;

	if (kfifo_avail(&nmtp_dev->write_fifo) < count)
		return -EAGAIN;

	if (nmtp_dev->write_count[DRM_RADIO_FIFO] >=  DRM_LIMIT)
		return -EAGAIN;

	status = payload_extract(buf, count, nmtp_dev,
			DRM_RADIO, DRM_RADIO_FIFO);

	return (status) ? status : count;
}

static ssize_t hd_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[8]);
	if (!nmtp_dev)
		return -ECANCELED;

	return fifo_read(nmtp_dev, buf, count, HD_RADIO_FIFO);

}

static int hd_open(struct inode *inode, struct file *f)
{
	return 0;
}

static unsigned int hd_poll(struct file *f, struct poll_table_struct *wait)
{
	int mask = 0;
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[8]);

	poll_wait(f, &nmtp_dev->data_wait[HD_RADIO_FIFO], wait);

	if (nmtp_dev->poll_count[HD_RADIO_FIFO] > 0)
		mask =  POLLIN | POLLRDNORM;


	return mask;
}

static ssize_t hd_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[8]);

	if (nmtp_dev == NULL)
		return -ESHUTDOWN;
	if (count > WRITE_BUF_LIMIT)
		return -EINVAL;

	if (kfifo_avail(&nmtp_dev->write_fifo) < count)
		return -EAGAIN;

	if (nmtp_dev->write_count[HD_RADIO_FIFO] >=  HD_LIMIT)
		return -EAGAIN;

	status = payload_extract(buf, count, nmtp_dev, HD_RADIO, HD_RADIO_FIFO);

	return (status) ? status : count;
}

static int simulcast_open(struct inode *inode, struct file *f)
{
	return 0;
}

static ssize_t simulcast_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[9]);
	if (!nmtp_dev)
		return -ECANCELED;

	return fifo_read(nmtp_dev, buf, count, SIMULCAST_FIFO);
}

static unsigned int simulcast_poll(struct file *f,
				struct poll_table_struct *wait)
{
	int mask = 0;
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[9]);

	poll_wait(f, &nmtp_dev->data_wait[SIMULCAST_FIFO], wait);

	if (nmtp_dev->poll_count[SIMULCAST_FIFO] > 0)
		mask =  POLLIN | POLLRDNORM;

	return mask;
}

static ssize_t simulcast_write(struct file *f, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[9]);

	if (nmtp_dev == NULL)
		return -ESHUTDOWN;
	if (count > WRITE_BUF_LIMIT)
		return -EINVAL;

	if (kfifo_avail(&nmtp_dev->write_fifo) < count)
		return -EAGAIN;

	if (nmtp_dev->write_count[SIMULCAST_FIFO] >= SIMULCAST_LIMIT)
		return -EAGAIN;

	status = payload_extract(buf, count, nmtp_dev,
			SIMULCAST, SIMULCAST_FIFO);

	return (status) ? status : count;
}

static long nmtp_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status = 0;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[0]);

	if (nmtp_dev == NULL)
		return -ESHUTDOWN;

	if (_IOC_TYPE(cmd) != NMTP_IOCTL_MAGIC) {

		dev_dbg(&nmtp_dev->spi->dev,
			"[hnmtp] [%s] [%d] nmtp ioctl error : invalid cmd %u\n",
							__func__, __LINE__, cmd);
		return -EINVAL;
	}

	switch (cmd) {
	case NMTP_START_CAPTURE:
#if 0
		/* Capture ioctl can be called from tunerapp & AudioApp.
		* Both can closed independently. Capture_count is to
		* keep track of it before stop capture*/
		nmtp_dev->txrx_inprocess = RD_WR_CLEAR;
		mode = nmtp_dev->type->mode;
		mutex_lock(&nmtp_dev->reset_lock);
		reset_status = nmtp_dev->reset_status;
		mutex_unlock(&nmtp_dev->reset_lock);
		if (reset_status == NMTP_NORMAL_OPERATION) {
// or NMTP_NORMAL_WAIT_FOR_ACK?
			if (nmtp_dev->capture_count ==  0) {
				dev_err(&nmtp_dev->spi->dev,
					"[hnmtp] [%s] [%d] inside IOCTL\n",
					__func__, __LINE__);
				hrtimer_start(&nmtp_dev->hrtimer,
				nmtp_dev->ktime, HRTIMER_MODE_ABS);
				nmtp_dev->capture_count++;
			}
		} else
		status = -EAGAIN;
#endif
		break;
	case NMTP_STOP_CAPTURE:
			dev_err(&nmtp_dev->spi->dev,
				"[hnmtp] [%s] [%d] NMTP_STOP_CAPTURE\n",
				__func__, __LINE__);
			break;
	case NMTP_MAX_SPEED_HZ:
		dev_dbg(&nmtp_dev->spi->dev, "[Hnmtp] -- [%s] [%d] arg %lu\n",
							__func__, __LINE__, arg);
#if 0
			if (arg <= 15000000) {
			dev_dbg(&nmtp_dev->spi->dev, "[Hnmtp] [%s] [%d] %lu\n",
							__func__, __LINE__, arg);
				nmtp_dev->speed_hz = nmtp_dev->spi->max_speed_hz
									= arg;
				spi_setup(nmtp_dev->spi);
			}
#endif
			break;
	default:
			nmtp_err("nmtp ioctl error: %d\n", cmd);
			break;
	}
	return status;
}

static long nmtp_compat_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	return nmtp_ioctl(filp, cmd, arg);
}


static int nmtp_open_ioctl(struct inode *i, struct file *f)
{

	return 0;
}

static int nmtp_release_ioctl(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int device_control_release(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int std_audio_release(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int fm_release(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int am_release(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int wx_release(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int dab_release(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int drm_release(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int hd_release(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int simulcast_release(struct inode *i, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static int spidev_open(struct  inode *inode, struct file *f)
{
	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_dev;
	int status;

	nmtp_dev = container_of(m, struct nmtp_data, nmtp_misc[10]);

	dev_dbg(&nmtp_dev->spi->dev,
		"[Hnmtp] [%s] [%d] device file path is : %s and chipName is %s\n",
					__func__, __LINE__, f->f_path.dentry->d_iname, nmtp_dev->name);

	if (!nmtp_dev)
		return -ENOMEM;

	nmtp_dev->spidev_txbuf = kmalloc(SPI_PLAINMODE_DEVSIZE, GFP_KERNEL);
	if (!nmtp_dev->spidev_txbuf) {
		status = -ENOMEM;
		dev_err(&nmtp_dev->spi->dev,
			"[Hnmtp] [%s] [%d] Failed to allocate SPI TX memory\n",
					__func__, __LINE__);
		goto err_txbuf;
	}

	nmtp_dev->spidev_rxbuf = kmalloc(SPI_PLAINMODE_DEVSIZE, GFP_KERNEL);
	if (!nmtp_dev->spidev_rxbuf) {
		dev_err(&nmtp_dev->spi->dev,
			"[Hnmtp] [%s] [%d] Failed to allocate SPI RX memory\n",
					__func__, __LINE__);
		goto err_alloc_rx_buf;
	}

	return 0;
err_alloc_rx_buf:
	kfree(nmtp_dev->spidev_txbuf);
	nmtp_dev->spidev_txbuf = NULL;

err_txbuf:
	return -ENOMEM;
}

static ssize_t spidev_write(struct file *f, const char __user *buf,
		size_t count, loff_t *offset)
{
	int    status = 0;
	struct nmtp_data *nmtp_dev;
	struct miscdevice   *d = f->private_data;
	struct spi_transfer t;
	struct spi_message m;

	nmtp_dev = container_of(d, struct nmtp_data, nmtp_misc[10]);

	if (count > SPI_PLAINMODE_DEVSIZE)
		return -EMSGSIZE;

	status = copy_from_user(nmtp_dev->spidev_txbuf, buf, count);
	if (status == 0) {

		t.tx_buf         = nmtp_dev->spidev_txbuf;
		t.len            = count;


		mutex_lock(&nmtp_dev->txrx_lock);
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		status = spi_sync(nmtp_dev->spi, &m);
		mutex_unlock(&nmtp_dev->txrx_lock);

		if (!status)
			status = m.actual_length;
	} else
		status = 0;
	dev_err(&nmtp_dev->spi->dev,
		"[Hnmtp] [%s] [%d] nmtp status = %d ChipName %s devicePath = %s\n",
		__func__, __LINE__, status, nmtp_dev->name,
		f->f_path.dentry->d_iname);
	return status;
}

static int mrddev_message(struct nmtp_data *nmtp,
                struct nmtp_spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
        struct spi_message      msg;
        struct spi_transfer     *k_xfers;
        struct spi_transfer     *k_tmp;
        struct nmtp_spi_ioc_transfer *u_tmp;
        unsigned                n, total, tx_total, rx_total;
        u8                      *tx_buf, *rx_buf;
        int                     status = -EFAULT;

        spi_message_init(&msg);
        k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
        if (k_xfers == NULL)
                return -ENOMEM;

        /* Construct spi_message, copying any tx data to bounce buffer.
         * We walk the array of user-provided transfers, using each one
         * to initialize a kernel version of the same transfer.
         */
        tx_buf = nmtp->spidev_txbuf;
        rx_buf = nmtp->spidev_rxbuf;
        total = 0;
        tx_total = 0;
        rx_total = 0;
        for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
                        n;
                        n--, k_tmp++, u_tmp++) {
                k_tmp->len = u_tmp->len;

                total += k_tmp->len;
                /* Since the function returns the total length of transfers
                 * on success, restrict the total to positive int values to
                 * avoid the return value looking like an error.  Also check
                 * each transfer length to avoid arithmetic overflow.
                 */
                if (total > INT_MAX || k_tmp->len > INT_MAX) {
                        status = -EMSGSIZE;
                        goto done;
                }
                if (u_tmp->rx_buf) {
                        /* this transfer needs space in RX bounce buffer */
                        rx_total += k_tmp->len;
                        if (rx_total > SPI_PLAINMODE_DEVSIZE) {
                                status = -EMSGSIZE;
                                goto done;
                        }
                        k_tmp->rx_buf = rx_buf;
                        if (!access_ok(VERIFY_WRITE, (u8 __user *)
                                                (uintptr_t) u_tmp->rx_buf,
                                                u_tmp->len))
                                goto done;
                        rx_buf += k_tmp->len;
                }
                if (u_tmp->tx_buf) {
                        /* this transfer needs space in TX bounce buffer */
                        tx_total += k_tmp->len;
                        if (tx_total > SPI_PLAINMODE_DEVSIZE ) {
                                status = -EMSGSIZE;
                                goto done;
                        }
                        k_tmp->tx_buf = tx_buf;
                        if (copy_from_user(tx_buf, (const u8 __user *)
                                                (uintptr_t) u_tmp->tx_buf,
                                        u_tmp->len))
                                goto done;
                        tx_buf += k_tmp->len;
                }

                k_tmp->cs_change = !!u_tmp->cs_change;
                k_tmp->tx_nbits = u_tmp->tx_nbits;
                k_tmp->rx_nbits = u_tmp->rx_nbits;
                k_tmp->bits_per_word = u_tmp->bits_per_word;
                k_tmp->delay_usecs = u_tmp->delay_usecs;
                k_tmp->speed_hz = u_tmp->speed_hz;
                if (!k_tmp->speed_hz)
                        k_tmp->speed_hz = nmtp->speed_hz;
#ifdef VERBOSE
                dev_dbg(&nmtp->spi->dev,
                        "[Hmrd] -- [%s] [%d]  xfer len %u %s%s%s%dbits %u usec %uHz\n",
			__func__, __line__,
                        u_tmp->len,
                        u_tmp->rx_buf ? "rx " : "",
                        u_tmp->tx_buf ? "tx " : "",
                        u_tmp->cs_change ? "cs " : "",
                        u_tmp->bits_per_word ? : nmtp->spi->bits_per_word,
                        u_tmp->delay_usecs,
                        u_tmp->speed_hz ? : nmtp->spi->max_speed_hz);
#endif
                spi_message_add_tail(k_tmp, &msg);
        }
        status = spi_sync(nmtp->spi, &msg);
        if (status < 0)
                goto done;

        /* copy any rx data out of bounce buffer */
        rx_buf = nmtp->spidev_rxbuf;
        for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
                if (u_tmp->rx_buf) {
                        if (__copy_to_user((u8 __user *)
                                        (uintptr_t) u_tmp->rx_buf, rx_buf,
                                        u_tmp->len)) {
                                status = -EFAULT;
                                goto done;
                        }
                        rx_buf += u_tmp->len;
                }
        }
        status = total;

done:
        kfree(k_xfers);
        return status;
}



static struct nmtp_spi_ioc_transfer *
spidev_get_ioc_message(unsigned int cmd, struct nmtp_spi_ioc_transfer __user *u_ioc,
                unsigned *n_ioc)
{
        struct nmtp_spi_ioc_transfer *ioc;
        u32     tmp;

        /* Check type, command number and direction */
        if (_IOC_TYPE(cmd) != NMTP_IOCTL_MAGIC
                        || _IOC_NR(cmd) != _IOC_NR(NMTP_SPI_IOC_MESSAGE(0))
                        || _IOC_DIR(cmd) != _IOC_WRITE)
                return ERR_PTR(-ENOTTY);

        tmp = _IOC_SIZE(cmd);
        if ((tmp % sizeof(struct nmtp_spi_ioc_transfer)) != 0)
                return ERR_PTR(-EINVAL);
        *n_ioc = tmp / sizeof(struct nmtp_spi_ioc_transfer);
        if (*n_ioc == 0)
                return NULL;

        /* copy into scratch area */
        ioc = kmalloc(tmp, GFP_KERNEL);
        if (!ioc)
                return ERR_PTR(-ENOMEM);
        if (__copy_from_user(ioc, u_ioc, tmp)) {
                kfree(ioc);
                return ERR_PTR(-EFAULT);
        }
        return ioc;
}


static long spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
        int                     err = 0;
        int                     retval = 0;
        struct nmtp_data *nmtp_dev;
        struct miscdevice   *d = filp->private_data;
//        struct spidev_data      *spidev;
        struct spi_device       *spi;
        u32                     tmp;
        unsigned                n_ioc;
        struct nmtp_spi_ioc_transfer *ioc;
	struct nmtp_data *nmtp_secondary;

        nmtp_dev = container_of(d, struct nmtp_data, nmtp_misc[10]);

        /* Check type and command number */
        if (_IOC_TYPE(cmd) != NMTP_IOCTL_MAGIC)
                return -ENOTTY;

        /* Check access direction once here; don't repeat below.
         * IOC_DIR is from the user perspective, while access_ok is
         * from the kernel perspective; so they look reversed.
         */
        if (_IOC_DIR(cmd) & _IOC_READ)
                err = !access_ok(VERIFY_WRITE,
                                (void __user *)arg, _IOC_SIZE(cmd));
        if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
                err = !access_ok(VERIFY_READ,
                                (void __user *)arg, _IOC_SIZE(cmd));
        if (err)
                return -EFAULT;

	spi = nmtp_dev->spi;

        if (spi == NULL)
                return -ESHUTDOWN;

        switch (cmd) {
	case NMTP_SPI_RAW_MODE_START:
		nmtp_current_state_update(0, NMTP_SPIDEV_MODE);
		wake_up_interruptible(&nmtp_dev->reset_poll_wait);
		hrtimer_cancel(&nmtp_dev->hrtimer);
		nmtp_chip_reset_handler_exit();
		nmtp_ipc_exit();
		if (get_nmtp_details(1) != NULL) {
			nmtp_secondary = get_nmtp_details(1);
			nmtp_current_state_update(1, NMTP_SPIDEV_MODE);
			wake_up_interruptible(&nmtp_secondary->reset_poll_wait);
			hrtimer_cancel(&get_nmtp_details(1)->hrtimer);
		}
		break;
	case NMTP_SPI_RAW_MODE_STOP:
		retval = ipc_channel_init();
		if (retval < 0) {
			dev_dbg(&nmtp_dev->spi->dev, 
			"[Hnmtp][%s][%d] ipc channel init failed %d\n",
								__func__, __LINE__, retval);
			break;
		}
		retval = nmtp_reset_handler_start();
		if (retval < 0) {
			dev_dbg(&nmtp_dev->spi->dev, 
			"[Hnmtp][%s][%d] ipc channel init failed %d\n",
								__func__, __LINE__, retval);
			break;
		}

		nmtp_current_state_update(0, NMTP_NORMAL_OPERATION);
		hrtimer_start(&nmtp_dev->hrtimer,
				nmtp_dev->ktime, HRTIMER_MODE_ABS);
		if (get_nmtp_details(1) != NULL) {
			nmtp_current_state_update(1, NMTP_NORMAL_OPERATION);
			hrtimer_start(&get_nmtp_details(1)->hrtimer,
						nmtp_dev->ktime, HRTIMER_MODE_ABS);
		}
		break;
        /* read requests */
        case NMTP_SPI_IOC_RD_MODE:
                retval = __put_user(spi->mode & NMTP_SPI_MODE_MASK,
                                        (__u8 __user *)arg);
                break;
        case NMTP_SPI_IOC_RD_MODE32:
                retval = __put_user(spi->mode & NMTP_SPI_MODE_MASK,
                                        (__u32 __user *)arg);
                break;
        case NMTP_SPI_IOC_RD_LSB_FIRST:
                retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
                                        (__u8 __user *)arg);
                break;
        case NMTP_SPI_IOC_RD_BITS_PER_WORD:
                retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
                break;
        case NMTP_SPI_IOC_RD_MAX_SPEED_HZ:
                retval = __put_user(nmtp_dev->speed_hz, (__u32 __user *)arg);
                break;

	/* write requests */
        case NMTP_SPI_IOC_WR_MODE:
        case NMTP_SPI_IOC_WR_MODE32:
                if (cmd == NMTP_SPI_IOC_WR_MODE)
                        retval = __get_user(tmp, (u8 __user *)arg);
                else
                        retval = __get_user(tmp, (u32 __user *)arg);
                if (retval == 0) {
                        u32     save = spi->mode;

                        if (tmp & ~NMTP_SPI_MODE_MASK) {
                                retval = -EINVAL;
                                break;
                        }

                        tmp |= spi->mode & ~NMTP_SPI_MODE_MASK;
                        spi->mode = (u16)tmp;
                        retval = spi_setup(spi);
                        if (retval < 0)
                                spi->mode = save;
                        else
                                dev_dbg(&nmtp_dev->spi->dev,
				"[Hnmtp] -- [%s] [%d] spi mode %x\n",__func__,__LINE__, tmp);
                }
                break;

        case NMTP_SPI_IOC_WR_BITS_PER_WORD:
                retval = __get_user(tmp, (__u8 __user *)arg);
                if (retval == 0) {
                        u8      save = spi->bits_per_word;

                        spi->bits_per_word = tmp;
                        retval = spi_setup(spi);
                        if (retval < 0)
                                spi->bits_per_word = save;
                        else
                                dev_dbg(&nmtp_dev->spi->dev,
			"[Hmrd] -- [%s] [%d]  %d bits per word\n",__func__,__LINE__, tmp);
                }
                break;

        case NMTP_SPI_IOC_WR_MAX_SPEED_HZ:
                retval = __get_user(tmp, (__u32 __user *)arg);
                if (retval == 0) {
                        u32     save = spi->max_speed_hz;

                        spi->max_speed_hz = tmp;
                        retval = spi_setup(spi);
                        if (retval >= 0){
				nmtp_dev->speed_hz = tmp;
			}
                        else{
                                dev_dbg(&spi->dev, "[Hmrd] -- [%s] [%d] %d Hz (max)\n",__func__, __LINE__, tmp);
				spi->max_speed_hz = save;
				nmtp_dev->speed_hz = save;
			}
                }
                break;

	default:
                /* segmented and/or full-duplex I/O request */
                /* Check message and copy into scratch area */
		ioc = spidev_get_ioc_message(cmd,
			(struct nmtp_spi_ioc_transfer __user *)arg, &n_ioc);
                if (IS_ERR(ioc)) {
                        retval = PTR_ERR(ioc);
                        break;
                }
                if (!ioc)
                        break;  /* n_ioc is also 0 */

                /* translate to spi_message, execute */
                retval = mrddev_message(nmtp_dev, ioc, n_ioc);
                kfree(ioc);
                break;
	}
	return retval;

}

static ssize_t spidev_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	int status = 0;
	struct nmtp_data *nmtp_spidev;
	struct miscdevice   *d = f->private_data;
	struct spi_transfer t;
	struct spi_message m;

	nmtp_spidev = container_of(d, struct nmtp_data, nmtp_misc[10]);

	if (count > SPI_PLAINMODE_DEVSIZE)
		return -EMSGSIZE;

	t.rx_buf         = nmtp_spidev->spidev_rxbuf;
	t.len            = count;


	mutex_lock(&nmtp_spidev->txrx_lock);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(nmtp_spidev->spi, &m);
	mutex_unlock(&nmtp_spidev->txrx_lock);

	if (status == 0)
		status = copy_to_user(buf, nmtp_spidev->spidev_rxbuf,
				m.actual_length);

	dev_dbg(&nmtp_spidev->spi->dev,
		"[Hnmtp] [%s] [%d]  nmtp Status = %d ChipName = %s DevicePath = %s\n",
		 __func__, __LINE__, status, nmtp_spidev->name,
		 f->f_path.dentry->d_iname);

	return status ? -EFAULT : (m.actual_length);
}

static int spidev_release(struct inode *i, struct file *f)
{

	struct miscdevice *m = f->private_data;
	struct nmtp_data *nmtp_spidev;

	nmtp_spidev = container_of(m, struct nmtp_data, nmtp_misc[10]);
	dev_dbg(&nmtp_spidev->spi->dev,
		"[Hnmtp] [%s] [%d]  device file path is : %s and chipName is %s\n",
		__func__, __LINE__, f->f_path.dentry->d_iname,
		nmtp_spidev->name);

	if (!nmtp_spidev)
		return -ENOMEM;

	if (nmtp_spidev->spidev_txbuf) {
		kfree(nmtp_spidev->spidev_txbuf);
		nmtp_spidev->spidev_txbuf = NULL;
	}

	if (nmtp_spidev->spidev_rxbuf) {
		kfree(nmtp_spidev->spidev_rxbuf);
		nmtp_spidev->spidev_rxbuf = NULL;
	}
	return 0;
}

static long spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return spidev_ioctl(filp, cmd, arg);
}


static const struct file_operations write_ioctl = {
	.owner		= THIS_MODULE,
	.open		= nmtp_open_ioctl,
	.unlocked_ioctl = nmtp_ioctl,
	.compat_ioctl	= nmtp_compat_ioctl,
	.release	= nmtp_release_ioctl,
};

static const struct file_operations device_control_fops = {
	.owner		= THIS_MODULE,
	.open		= device_control_open,
	.read		= device_control_read,
	.write		= device_control_write,
	.poll		= device_control_poll,
	.release	= device_control_release,
};

static const struct file_operations std_audio_fops = {
	.owner		= THIS_MODULE,
	.open		= std_audio_open,
	.read		= std_audio_read,
	.write		= std_audio_write,
	.poll		= std_audio_poll,
	.release	= std_audio_release,
};

static const struct file_operations fm_fops = {
	.owner		= THIS_MODULE,
	.open		= fm_open,
	.read		= fm_read,
	.write		= fm_write,
	.poll		= fm_poll,
	.release	= fm_release,
};

static const struct file_operations am_fops = {
	.owner		= THIS_MODULE,
	.open		= am_open,
	.read		= am_read,
	.write		= am_write,
	.poll		= am_poll,
	.release	= am_release,
};

static const struct file_operations wx_fops = {
	.owner		= THIS_MODULE,
	.open		= wx_open,
	.read		= wx_read,
	.write		= wx_write,
	.poll		= wx_poll,
	.release	= wx_release,
};

static const struct file_operations dab_fops = {
	.owner		= THIS_MODULE,
	.open		= dab_open,
	.read		= dab_read,
	.poll		= dab_poll,
	.write		= dab_write,
	.release	= dab_release,
};

static const struct file_operations drm_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.read		= drm_read,
	.poll		= drm_poll,
	.write		= drm_write,
	.release	= drm_release,
};

static const struct file_operations hd_fops = {
	.owner		= THIS_MODULE,
	.open		= hd_open,
	.read		= hd_read,
	.poll		= hd_poll,
	.write		= hd_write,
	.release	= hd_release,
};

static const struct file_operations simulcast_fops = {
	.owner		= THIS_MODULE,
	.open		= simulcast_open,
	.read		= simulcast_read,
	.poll		= simulcast_poll,
	.write		= simulcast_write,
	.release	= simulcast_release,
};

static const struct file_operations spidev_fops = {
	.owner          = THIS_MODULE,
	.open           = spidev_open,
	.read           = spidev_read,
	.write          = spidev_write,
        .unlocked_ioctl = spidev_ioctl,
	.compat_ioctl	= spidev_compat_ioctl,
	.release        = spidev_release,
};

static struct {
	char name[30];
	char *post_name;
	const struct file_operations *fops;
} devlist[] = {
	{"", "write_ioctl", &write_ioctl},
	{"", "device_control", &device_control_fops},
	{"", "audio", &std_audio_fops},
	{"", "fm",	&fm_fops},
	{"", "am",	&am_fops},
	{"", "wx",	&wx_fops},
	{"", "dab", &dab_fops},
	{"", "drm", &drm_fops},
	{"", "hd", &hd_fops},
	{"", "simulcast", &simulcast_fops},
	{"", "spidev", &spidev_fops}
};

int nmtp_add_dev(struct spi_device *spi, struct nmtp_data *nmtp)
{
	int i, status;
	struct miscdevice *misc;

	if (nmtp->type->mode == 0)
		strcpy(nmtp->name, "nmtp_master");
	else
		strcpy(nmtp->name, "nmtp_slave");

	dev_dbg(&nmtp->spi->dev, "[Hnmtp] [%s] [%d] misc register addition nmtp\n",
			__func__, __LINE__);
	for (i = 0, status = 0; i < NMTP_DEVNODES + 2; i++) {
		misc = &nmtp->nmtp_misc[i];
		misc->minor = MISC_DYNAMIC_MINOR;
		snprintf(devlist[i].name, 30, "%s_%s", nmtp->name, devlist[i].post_name);
		misc->name = devlist[i].name;
		misc->fops = devlist[i].fops;
		status = misc_register(misc);
		if (status) {
			dev_dbg(&nmtp->spi->dev,
				"[Hnmtp] [%s] [%d] Couldnt register misc device %s\n",
						__func__, __LINE__, devlist[i].name);
			break;
		}
	}
	return status;
}

void nmtp_del_dev(struct nmtp_data *nmtp)
{
	int i;

	for (i = 0; i < NMTP_DEVNODES + 2; i++)
		misc_deregister(&nmtp->nmtp_misc[i]);
}
