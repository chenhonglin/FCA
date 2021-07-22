/*
 *  nmtp_fca_reset.c
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

#define IPC_RESET_NMTP                      "/dev/ipc12"
#define IPC_RESET_NMTP_CHANNEL              12
#define IPC_INTERFACE_VERSION                  0x01

#define IPC_RESET_REQ_MSG                      0xD2
#define IPC_RESET_REQ_PAY_RST_REQ              0x00
#define IPC_RESET_REQ_PAY_RST_CONF             0x01

#define IPC_PRIMARY_TUNER_ID                   0x01
#define IPC_SECONDRY_TUNER_ID                  0x02
#define IPC_BOOTMODE_CONFIG                    0xD1
#define IPC_BOOTMODE_ROM		       0x00
#define IPC_BOOTMODE_HOST                      0x01
#define IPC_BOOTMODE_FLASH                     0x02

//Response from VIP
#define IPC_BOOTMODE_READY                     0x00
#define IPC_BOOTMODE_NOTVALID                  0x01
#define IPC_BOOTMODE_BUSY		       0x02
#define TUNER_ID_NOT_VALID		       0x03

#define IPC_RESET_REQ_ACK_MSG                  0xD3
#define IPC_RESET_REQ_ACK_PAY_UNDEF            0x00
#define IPC_RESET_REQ_ACK_PAY_DSP_INIT         0x01
#define IPC_RESET_REQ_ACK_PAY_DSP_READY        0x02
#define IPC_RESET_REQ_ACK_PAY_DSP_PENDING      0x03
#define IPC_RESET_REQ_ERROR                    0x04

#define IPC_RESET_QUERY_MSG                    0xD4
#define IPC_RESET_QUERY_ACK_MSG                0xD4
#define IPC_RESET_QUERY_ACK_BUSY               0x01
#define IPC_RESET_QUERY_ACK_READY              0x00

#define IPC_REQUEST_SPI_BANDWIDTH              0xD5
#define IPC_REQUEST_SPI_NOTREADY               0x00
#define IPC_REQUEST_SPI_READY		       0x01

#define GPIO_SECONDARY_TUNER			461
/** IPC API ********************************/
extern int ipc_channel_open(int ch_num);
extern int ipc_channel_close(int chnum);
extern int ipc_channel_read(int chnum, char *buf, size_t count, int flags);
extern int ipc_channel_read_timeout(int chnum, char *buf, size_t count, int timeout);
extern int ipc_channel_write(int chnum, const char *buf, size_t count);

static int nmtp_reset_major;
struct class *nmtp_reset_class;
static DECLARE_BITMAP(minors, DYNAMIC_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(nmtp_reset_lock);
static DECLARE_WAIT_QUEUE_HEAD(reset_poll_wait);
static DECLARE_WAIT_QUEUE_HEAD(reset_write_wait);
static DECLARE_WAIT_QUEUE_HEAD(reset_write_wait_secondary);
static DECLARE_WAIT_QUEUE_HEAD(reset_poll_wait_secondary);

static int ioc_fd =  -1;
static int gpio_stat = -1;
static bool resume_primary;

/* file_open*/
static int __nmtp_ipc_open(void)
{

	ioc_fd = ipc_channel_open(IPC_RESET_NMTP_CHANNEL);
	if (ioc_fd < 0)
	{
		printk(KERN_ERR" [Hnmtp] -- [%s] [%d] [%s]  File Open error ioc_fd = [%d]\n",
					 __func__, __LINE__, IPC_RESET_NMTP, ioc_fd);
		ioc_fd = -1;
		return -EIO;
	} else
		printk(KERN_ERR" [Hnmtp] [%s] File Open success\n", IPC_RESET_NMTP);

	return (ioc_fd >= 0)?0 :  -EIO;

}


int nmtp_ipc_open(void)
{
	static int count;
	int status = 0;

	if (count == 0) {
		status = __nmtp_ipc_open();
		if (status == 0) {
			++status;
			count++;
		} else
			return status;
	}
	return status;
}

static int __nmtp_ipc_close(void)
{
	int close_ret;

	close_ret = ipc_channel_close(IPC_RESET_NMTP_CHANNEL);
	if (close_ret != 0) {
		printk(KERN_ERR" [Hnmtp]  [%s] [%d]  [%s] CLOSE FAIL = %d\n",
					 __func__, __LINE__, IPC_RESET_NMTP, close_ret);
	} else
		printk(KERN_ERR" [Hnmtp] [%s] File close success\n", IPC_RESET_NMTP);

	return close_ret;
}


static int nmtp_ipc_close(void)
{
	return __nmtp_ipc_close();
}

/* Reset nmtp chip via IPC-IOC for reset. We are not handling the chimes error scenarios, error state returned*/
static int __nmtp_reset_ipc_op(struct nmtp_data *nmtp)
{
	int  ret = 0, i;
	u8  iocMsg[10];
	int status = 0, queryStat = 0;

	//this make sure read buffer is flushed out before reading actual
	while (ret != -EAGAIN)
		ret = ipc_channel_read(IPC_RESET_NMTP_CHANNEL, iocMsg, 10, O_NONBLOCK);


	iocMsg[0] = IPC_RESET_REQ_MSG;
    iocMsg[1] = IPC_INTERFACE_VERSION;
	iocMsg[2] = IPC_RESET_REQ_PAY_RST_REQ;
	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] REQUEST RESET  --\n", __func__, __LINE__);
	ret = ipc_channel_write(IPC_RESET_NMTP_CHANNEL, iocMsg, 3);
	if (ret == 3)
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d]  REQUEST RESET DONE\n", __func__, __LINE__);

	else{
		dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s] --FAILED-- WRITE <Request reset>  MSG_ID=[%x]INF_ID=[%x] MSG_PAYLOAD [%x]\n",
			__func__, __LINE__, IPC_RESET_NMTP,
			IPC_RESET_REQ_MSG, IPC_INTERFACE_VERSION, IPC_RESET_REQ_PAY_RST_REQ);
		return -1;
	}

	//read it
	for (i = 0; i < 50 ; i++) {
		ret = ipc_channel_read_timeout(IPC_RESET_NMTP_CHANNEL, iocMsg, 10, 100);

		if ((ret > 3) && (iocMsg[0] == IPC_RESET_REQ_ACK_MSG) &&
				((iocMsg[4] == IPC_RESET_REQ_ACK_PAY_DSP_PENDING) ||
					(iocMsg[4] == IPC_RESET_REQ_ERROR) || (iocMsg[4] == IPC_RESET_REQ_ACK_PAY_DSP_READY))) {
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s] IPC REQUEST:DATA READ  MSG_ID=[%x] INF_ID=[%x]HU_VARIANT =[%x] HU_TUNER_SAMPLE = [%x] TUNER_STATE = [%x]\n",
						__func__, __LINE__, IPC_RESET_NMTP, iocMsg[0], iocMsg[1], iocMsg[2],
							iocMsg[3], iocMsg[4]);

			status = 1;

		break;
		} else
			msleep(100);
	}

	if (status != 1) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s] Request Reset Error exiting\n",
			__func__, __LINE__, IPC_RESET_NMTP);
		return -1;
	}

	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] CONFIRM REQUEST\n", __func__, __LINE__);

	/* write to IPC Interface
	   0xD2 0x00 0x01
	 */
	iocMsg[0] = IPC_RESET_REQ_MSG;
	iocMsg[1] = IPC_INTERFACE_VERSION;
	iocMsg[2] = IPC_RESET_REQ_PAY_RST_CONF;

	ret = ipc_channel_write(IPC_RESET_NMTP_CHANNEL, iocMsg, 3);
	if (ret == 3) {
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] REQUEST RESET CONFIRM DONE\n", __func__, __LINE__);
	} else{
		dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s] --FAILED-- WRITE Request reset confirmMSG_ID=[%x] INF_ID=[%x] MSG_PAYLOAD =[%x]\n",
			__func__, __LINE__,
				IPC_RESET_NMTP, IPC_RESET_REQ_MSG, IPC_INTERFACE_VERSION
					 , IPC_RESET_REQ_PAY_RST_CONF);
		return -1;
	}

	msleep(500);

	/* Read Max 50 times for 1 write*/
	for (i = 0; i < 50; i++) {
		ret = ipc_channel_read_timeout(IPC_RESET_NMTP_CHANNEL, iocMsg, 10, 100);

		if ((ret > 3) && (iocMsg[0] == IPC_RESET_REQ_ACK_MSG)) {

			if (iocMsg[4] == IPC_RESET_REQ_ACK_PAY_DSP_READY) {
				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s] IPC RESET CONFIRM:DATA READ  MSG_ID=[%x]INF_ID = [%x]  TUNER_STATE =[%x] iteration = %d\n",
						__func__, __LINE__,
							 IPC_RESET_NMTP, iocMsg[0], iocMsg[1], iocMsg[4], i);
				status = 1;

				break;
			} else if (iocMsg[4] == IPC_RESET_REQ_ERROR) {
				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s] IPC RESET CONFIRM:DATA READ  MSG_ID=[%x]INF_ID = [%x]  TUNER_STATE =[%x] iteration = %d\n",
						__func__, __LINE__,
							IPC_RESET_NMTP, iocMsg[0], iocMsg[1], iocMsg[4], i);
				return -1;  //No calrification on error handling from VIP.
			} else{
				msleep(100);
			}
		} else
			msleep(100);
	}



	/* Query the IPC channel .This should be checked if previous state is ready.*/
	for (i = 0 ; (i < 50) && (status == 1); i++) {
		 queryStat = nmtp_reset_ipc_query(nmtp);
		if (queryStat == 1)
			break;
		else
			msleep(100);
	}
	dev_dbg(&nmtp->spi->dev, "[Hnmtp] --[%s] [%d] [%s] QUERY DONE--queryResult = %d ",
			__func__, __LINE__, IPC_RESET_NMTP, queryStat);

	return (status & queryStat);

}

int nmtp_reset_ipc_op(struct nmtp_data *nmtp)
{
	int status;

	if (ioc_fd < 0) {
		status =  nmtp_ipc_open();
		if (status < 0) {
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] FILE NOT OPEN RESET FAILED\n",
								 __func__, __LINE__);
			return -EIO;
		}
	}
	return __nmtp_reset_ipc_op(nmtp);
}

/* Query nmtp chip resetIPC-IOC for reset*/
static int __nmtp_reset_ipc_query(struct nmtp_data *nmtp)
{
	int ret = 0;
	int status = 0;
    int j;
    u8  iocQueryMsg[3];

		//this make sure read buffer is flushed out before reading actual
	while (ret != -EAGAIN)
		ret = ipc_channel_read(IPC_RESET_NMTP_CHANNEL, iocQueryMsg, 3, O_NONBLOCK);

    iocQueryMsg[0] = IPC_RESET_QUERY_MSG;
    iocQueryMsg[1] = IPC_INTERFACE_VERSION;

	ret = ipc_channel_write(IPC_RESET_NMTP_CHANNEL, iocQueryMsg, 2);
	if (ret != 2) {
		 dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s]--FAILED--BOOT QUERY  MSG_ID=[%x] INF_ID=[%x]\n",
				__func__, __LINE__, IPC_RESET_NMTP, iocQueryMsg[0], iocQueryMsg[1]);
		return -1;

	} else{
		 dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d]  BOOT QUERY Write done ret = %d\n",
				__func__, __LINE__, ret);
	}

     //	msleep(100);
	for (j = 0; j < 5; j++) {

		ret = ipc_channel_read_timeout(IPC_RESET_NMTP_CHANNEL, iocQueryMsg, 3, 100);

		//double confirming to avoid any junk value
		if ((ret == 3) && (iocQueryMsg[0] == IPC_RESET_QUERY_ACK_MSG) && (iocQueryMsg[2] == IPC_RESET_QUERY_ACK_READY)) {

			 dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s] BOOT QUERY READ  MSG_ID=[%x] INF_ID=[%x]BOOT_STATUS = [%x]\n",
				__func__, __LINE__, IPC_RESET_NMTP, iocQueryMsg[0],
					iocQueryMsg[1], iocQueryMsg[2]);

			status = 1;
			break;
		} else
			msleep(30);
	}
	return status;
}

int nmtp_reset_ipc_query(struct nmtp_data *nmtp)
{
	/* Check file is open else retry to open*/
	int status;

	if (ioc_fd < 0) {
		status =  nmtp_ipc_open();
		if (status < 0) {
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] FILE NOT -- OPEN BOOT STATUS FAIL\n", __func__, __LINE__);
			return -EIO;
		}
	}
	return __nmtp_reset_ipc_query(nmtp);
}

static int nmtp_ipc_reset(struct nmtp_data *nmtp)
{
	int status;

	status = nmtp_reset_ipc_op(nmtp);

	return (status == 1) ? 0 : 1;
}

int nmtp_full_bandwidth_request(struct nmtp_data *nmtp)
{

	int status = -1;
	int ret  = -1;
	u8  iocBWMsg[3];

	dev_dbg(&nmtp->spi->dev, "[Hnmtp] BW Request -- [%s] [%d]\n", __func__, __LINE__);
	if (ioc_fd < 0) {
		status = nmtp_ipc_open();
		if (status < 0) {

			dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] File Not Open Bandwidth Request\n",
						__func__, __LINE__);
			return -EIO;
		}
	}

		//this make sure read buffer is flushed out before reading actual
	while (ret != -EAGAIN)
		ret = ipc_channel_read(IPC_RESET_NMTP_CHANNEL, iocBWMsg, 3, O_NONBLOCK);

	iocBWMsg[0] = IPC_REQUEST_SPI_BANDWIDTH;
	iocBWMsg[1] = IPC_INTERFACE_VERSION;

	ret = ipc_channel_write(IPC_RESET_NMTP_CHANNEL, iocBWMsg, 2);

	if (ret != 2) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s]-FAILED BANDWIDTH REQUEST QUERY MSG_ID=[%x]\n",
							 __func__, __LINE__, IPC_RESET_NMTP, IPC_REQUEST_SPI_BANDWIDTH);

		return -1;
	}
	ret = ipc_channel_read_timeout(IPC_RESET_NMTP_CHANNEL, iocBWMsg, 3, 100);

	if ((ret == 3) && (iocBWMsg[0] == IPC_REQUEST_SPI_BANDWIDTH)) {
		status = (int)iocBWMsg[2];
	}
	return status;   //  0 on ready to handover  and 1 on vip busy

}

int nmtp_secondary_reset_ipc_op(struct nmtp_data *nmtp)
{
	int status = -1;
	int ret    = -1;
	u8  iocSecMsg[4];

	if (ioc_fd < 0) {

		status = nmtp_ipc_open();
		if (status < 0) {
			dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] File Not Open Bandwidth Request\n",
							 __func__, __LINE__);
			return -EIO;
			}
		}

			//this make sure read buffer is flushed out before reading actual
	while (ret != -EAGAIN)
		ret = ipc_channel_read(IPC_RESET_NMTP_CHANNEL, iocSecMsg, 4, O_NONBLOCK);

	iocSecMsg[0] = IPC_BOOTMODE_CONFIG;
	iocSecMsg[1] = IPC_INTERFACE_VERSION;
	iocSecMsg[2] = IPC_SECONDRY_TUNER_ID;
	iocSecMsg[3] = IPC_BOOTMODE_FLASH;
	ret = ipc_channel_write(IPC_RESET_NMTP_CHANNEL, iocSecMsg, 4);
	if (ret != 4) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s]-FAILED WRITE BOOTMODE CONFIG SECONDARY:MSG_ID=[%x]\n",
					__func__, __LINE__, IPC_RESET_NMTP, IPC_BOOTMODE_CONFIG);
		return -1;
		}
	ret = ipc_channel_read_timeout(IPC_RESET_NMTP_CHANNEL, iocSecMsg, 4, 100);
	if ((ret == 4) && (iocSecMsg[3] == IPC_BOOTMODE_READY) && (iocSecMsg[2] == IPC_SECONDRY_TUNER_ID)
 && (iocSecMsg[1] == IPC_INTERFACE_VERSION) && (iocSecMsg[0] == IPC_BOOTMODE_CONFIG)) {

		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] [%s]-Success BOOTMODE CONFIG SECONDARY:MSG_ID=[%x] BootMode = [%x]\n",
							__func__, __LINE__, IPC_RESET_NMTP, iocSecMsg[0], iocSecMsg[3]);
		status = 0;
		}
	return status;   //0 on sucess
}

int nmtp_secondary_query(struct nmtp_data *nmtp)
{
	//gpio implementation
	int i, ret;

	if (gpio_stat == -1) {
		gpio_stat = gpio_request(GPIO_SECONDARY_TUNER, "GPIO_27");
		if (gpio_stat != 0) {
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] GPIO request error %d", __func__, __LINE__, gpio_stat);
			gpio_stat = -1;
			return gpio_stat;
		}

		gpio_stat = gpio_direction_input(GPIO_SECONDARY_TUNER);

		if (gpio_stat != 0) {
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] GPIO input direction error %d", __func__, __LINE__, gpio_stat);
			gpio_stat = -1;
			return gpio_stat;
		}

	}
	ret = -1;
	for (i = 0 ; i < 5 ; i++)
	{
		ret = gpio_get_value(GPIO_SECONDARY_TUNER);   //0 on success
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] GPIO Secondary status = %d and iteration = %d",
					__func__, __LINE__, ret, i);
		if (ret == 0)
			break;
		else
			msleep(50);
	}
	return ret;
}

static int nmtp_reset_secondary(struct nmtp_data *nmtp)
{
	int status, count, i;
	u8 mode  = nmtp->type->mode;

	status = count = 0;
	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Secondary\n", __func__, __LINE__);
	nmtp_current_state_update(mode, MRD_RESET_IN_PROGRESS);
	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Secondary--  nmtp state = MRD_RESET_IN_PROGRESS\n", __func__, __LINE__);
	wake_up_interruptible(&reset_poll_wait_secondary);
	for (i = 0; i < NMTP_DEVNODES; i++)
		kfifo_reset(&nmtp->fifo[i]);

	for (i = 0; i < NMTP_DEVNODES; i++)
		nmtp->write_count[i] = 0;

	kfifo_reset(&nmtp->write_fifo);
	for (i = 0; i < 0x5; i++) {
	/*IPC function to reset nmtp chip */
		status = nmtp_secondary_reset_ipc_op(nmtp);
		if (status) {
			dev_err(&nmtp->spi->dev, "[Hnmtp] --[%s] [%d] Secondary-- tuner reset failed\n", __func__, __LINE__);
			continue;
		}
		//secondary sync nedes to be done
		status = nmtp_secondary_query(nmtp);
		if (status == 0) {
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Secondary Reset-- Boot Query Succeded-[%d] Proceeding Syncing\n",
								__func__, __LINE__, status);
			status = nmtp_sync(nmtp);
			if (!status) {
				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Secondary Reset-- Re-Sync is achieved\n",
								 __func__, __LINE__);
					     //nmtp_dbg("tuner chip Re-Sync is achieved\n");
				printk(KERN_ERR "[Hnmtp] Secondary tuner chip Re-Sync is achieved\n");
				break;
			} else
				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Secondary-- tuner is not able Sync evenafter Chip-reset retyring\n",
					__func__, __LINE__);
		}
	}
	return status;  //0 success
}

static int nmtp_reset_primary(struct nmtp_data *nmtp)
{
	int status, count, i;
	u8 mode  = nmtp->type->mode;

	status = count = 0;

	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Primary\n", __func__, __LINE__);

    // two tuner is present and reset for primary
	nmtp_current_state_update(mode, MRD_RESET_IN_PROGRESS);
	wake_up_interruptible(&reset_poll_wait);
	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d]  nmtp state = MRD_RESET_IN_PROGRESS\n", __func__, __LINE__);

	for (i = 0; i < NMTP_DEVNODES; i++)
		kfifo_reset(&nmtp->fifo[i]);

	for (i = 0; i < NMTP_DEVNODES; i++)
		nmtp->write_count[i] = 0;

	kfifo_reset(&nmtp->write_fifo);

	if (get_nmtp_details(1) != NULL) {
		nmtp_current_state_update(1, MRD_RESET_IN_PROGRESS);
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Sec nmtp state = MRD_RESET_IN_PROGRESS\n",
								 __func__, __LINE__);
		wake_up_interruptible(&reset_poll_wait_secondary);

		for (i = 0; i < NMTP_DEVNODES; i++)
			kfifo_reset(&(get_nmtp_details(1)->fifo[i]));

		for (i = 0; i < NMTP_DEVNODES; i++)
			get_nmtp_details(1)->write_count[i] = 0;

		kfifo_reset(&(get_nmtp_details(1)->write_fifo));
	}

	for (i = 0; i < 0x5; i++) {
	/*IPC function to reset nmtp chip */
		status = nmtp_ipc_reset(nmtp);
		if (status) {
			dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] tuner primary reset failed\n",
							 __func__, __LINE__);
			continue;
		}
		//secondary sync nedes to be done
		status = nmtp_reset_ipc_query(nmtp);
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] tuner Primary reset boot query [%d]\n",
							__func__, __LINE__, status);
		if (status != 1) {
			status = 1 ;  //since return of reset query is 0 on faliure
			continue;
		}

		status = nmtp_sync(nmtp);
		if (!status) {
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] tuner Primary Reset chip Re-Sync is achieved\n",
							__func__, __LINE__);
			if (get_nmtp_details(1) != NULL) {
				status = nmtp_secondary_query(get_nmtp_details(1));
				 dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] tuner Primary Reset: Secondary Boot Query staus--[%d]\n",
								 __func__, __LINE__, status);
				if (status == 0) {
					status = nmtp_sync(get_nmtp_details(1));
					if (status != 0) {
						 dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] tuner Primary Reset: Secondary sync failedretrying --[%d]\n",
								__func__, __LINE__, status);
						continue;
					} else
						dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] tuner Primary Reset: Secondary Sync Succeded\n",
						    __func__, __LINE__);

				} else
					continue;
			}

		} else {

			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d]  tuner Primary Reset :Primary sync failed\n",
						__func__, __LINE__);

		}
		if (status == 0)
			break;
	}

	return status;
}

int nmtp_chip_primary_set_status(struct nmtp_data *nmtp, u8 status)
{
	int ret, i;
	u8 mode = nmtp->type->mode;

	nmtp_current_state_update(mode, status);
	dev_dbg(&nmtp->spi->dev, "[Hnmtp] --  [%s] [%d]   updated nmtp state = %d\n", __func__, __LINE__, status);
	/* wakeup poll_wait */

	switch (status) {
		case MRD_NORMAL_OPERATION:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp state = MRD_NORMAL_OPERATION\n", __func__, __LINE__);
			ret = 0;
			break;

		case  MRD_SUSPEND:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp state = MRD_SUSPEND\n", __func__, __LINE__);
			hrtimer_cancel(&nmtp->hrtimer);
			for (i = 0; i < NMTP_DEVNODES; i++)
				kfifo_reset(&nmtp->fifo[i]);

			for (i = 0; i < NMTP_DEVNODES; i++)
				nmtp->write_count[i] = 0;
			ret = 0;
			break;

		case MRD_RESUME:
		case MRD_RESUME_DONE:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp state = MRD_RESUME/_DONE\n", __func__, __LINE__);
			/* sending poll event */
			wake_up_interruptible(&reset_poll_wait);  // no need to wake up secondary
			ret = 0;
			break;
		default:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp state = Default -%d-\n", __func__, __LINE__, status);
			/* sending poll event */
			wake_up_interruptible(&reset_poll_wait);
			hrtimer_cancel(&nmtp->hrtimer);
			/** secondary tuner is present **/
			//mode change is required

			mutex_lock(&nmtp_reset_lock);
			//put under mutex to safeguard calling secondary and primary reset togather from User space
			if (get_nmtp_details(1) != NULL) {
				nmtp_current_state_update(1, MRD_RESET_START_APP);
				wake_up_interruptible(&reset_poll_wait_secondary);
				hrtimer_cancel(&(get_nmtp_details(1)->hrtimer));

			/*end*/

			/* Timeout is 1sec, count increment is from Tunerapp and AudioApp */

				ret = wait_event_interruptible_timeout(reset_write_wait,
					((get_nmtp_state(mode)->reset_status == MRD_RESET_CONFIRM_APP) && (get_nmtp_state(1)->reset_status == MRD_RESET_CONFIRM_APP)), 1 * HZ);
			} else {
				ret = wait_event_interruptible_timeout(reset_write_wait, get_nmtp_state(mode)->reset_status == MRD_RESET_CONFIRM_APP, 1 * HZ);
			}

			if (ret == 0) {

				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Timeout! not recieved confirmaion from App for reset\n", __func__, __LINE__);
			}
			//this will make sure secondary also resetted . We need to check boot query of secondary also
			ret = nmtp_reset_primary(nmtp);
			if (ret == 0) {

				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp reset success\n", __func__, __LINE__);
				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp state = MRD_NORMAL -%d-\n",
								__func__, __LINE__, get_nmtp_state(mode)->reset_status);
				hrtimer_start(&nmtp->hrtimer,
							nmtp->ktime, HRTIMER_MODE_ABS);
				if (get_nmtp_details(1) != NULL) {
					hrtimer_start(&get_nmtp_details(1)->hrtimer,
						get_nmtp_details(1)->ktime, HRTIMER_MODE_ABS);
				}
			}

			else {
				/* Forcefully starting the hrtimer */

				msleep(1000);

				hrtimer_start(&nmtp->hrtimer,
						nmtp->ktime, HRTIMER_MODE_ABS);
				if (get_nmtp_details(1) != NULL)
					hrtimer_start(&get_nmtp_details(1)->hrtimer,
						get_nmtp_details(1)->ktime, HRTIMER_MODE_ABS);

				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] tuner Sending notification in failure case %d --\n",
					__func__, __LINE__, get_nmtp_state(mode)->reset_status);

				//ret = -EIO;
			}
			/* Have to revisit this code. Have observed in the DV2 HW,nmtp chip
			 * reset fails, but nmtp sync works. Added this code to
			 * continue the flow. Later stage, nmtp chip reset should work
			 * reliably.
			 */
			mutex_unlock(&nmtp_reset_lock);
			nmtp_current_state_update(mode, MRD_NORMAL_OPERATION);

			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] tuner Sending notification %d --\n",
							__func__, __LINE__, get_nmtp_state(mode)->reset_status);

			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp state = MRD_NORMAL\n",
							 __func__, __LINE__);

			wake_up_interruptible(&reset_poll_wait);
			if (get_nmtp_details(1) != NULL) {
				nmtp_current_state_update(1, MRD_NORMAL_OPERATION);
				wake_up_interruptible(&reset_poll_wait_secondary);
			}
			break;
		}

	return ret;
}

int nmtp_chip_secondary_set_status(struct nmtp_data *nmtp, u8 status)
{
	u8 mode = nmtp->type->mode;
	int i, ret;

	nmtp_current_state_update(mode, status);
	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Secondary mode = %d\n", __func__, __LINE__, mode);

	switch (status) {
		case MRD_NORMAL_OPERATION:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Secondary--  nmtp state = MRD_NORMAL_OPERATION\n", __func__, __LINE__);
			ret = 0;
			break;

		case  MRD_SUSPEND:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Secondary--  nmtp state = MRD_SUSPEND\n", __func__, __LINE__);
			hrtimer_cancel(&nmtp->hrtimer);
			for (i = 0; i < NMTP_DEVNODES; i++)
				kfifo_reset(&nmtp->fifo[i]);

			for (i = 0; i < NMTP_DEVNODES; i++)
				nmtp->write_count[i] = 0;
			ret = 0;
			break;

		case MRD_RESUME:
		case MRD_RESUME_DONE:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Secondary--  nmtp state = MRD_RESUME/_DONE\n", __func__, __LINE__);
			/* sending poll event */
			wake_up_interruptible(&reset_poll_wait_secondary);
			ret = 0;
			break;


		default:

			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp Secondary state = Default -%d-\n", __func__, __LINE__, status);
			/* sending poll event */
			wake_up_interruptible(&reset_poll_wait_secondary);
			hrtimer_cancel(&nmtp->hrtimer);

			mutex_lock(&nmtp_reset_lock);

			/* Timeout is 1sec, count increment is from Tunerapp and AudioApp */

			ret = wait_event_interruptible_timeout(reset_write_wait,
					(get_nmtp_state(mode)->reset_status == MRD_RESET_CONFIRM_APP), 1 * HZ);

			if (ret == 0) {

				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] Timeout! not recieved confirmaion from App for reset\n", __func__, __LINE__);
			}

			ret = nmtp_reset_secondary(nmtp);
			if (ret == 0) {

				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d]  nmtp secondary reset success\n", __func__, __LINE__);

				nmtp_current_state_update(mode, MRD_NORMAL_OPERATION);

				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp state = MRD_NORMAL\n",
						 __func__, __LINE__);
				wake_up_interruptible(&reset_poll_wait_secondary);

			} else {
				/* Forcefully starting the hrtimer */

				msleep(1000);
				nmtp_current_state_update(mode, MRD_NORMAL_OPERATION);
				wake_up_interruptible(&reset_poll_wait_secondary);
				dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] tuner Sending notification in failure case %d --\n",
												__func__, __LINE__, get_nmtp_state(mode)->reset_status);


				//ret = -EIO;
			}

			hrtimer_start(&nmtp->hrtimer,
					nmtp->ktime, HRTIMER_MODE_ABS);
			mutex_unlock(&nmtp_reset_lock);
			break;
		}

	return ret;
}

void wait_for_apps_confirm(struct nmtp_data *nmtp)
{
	long int status = 0;
	int i;
	u8 mode = nmtp->type->mode;

	if (mode == 0) {
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] primary\n", __func__, __LINE__);

		//we need to make sure boot query is succeeded
		resume_primary = 1;
		for (i = 0 ; i < 10 && status != 1 ; i++)
			status = nmtp_reset_ipc_query(nmtp);

		if (get_nmtp_details(1) != NULL) {
			msleep(100);
			for (i = 0 ; i < 10 && status != 0 ; i++)
				status = nmtp_secondary_query(get_nmtp_details(1));
		}

		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] primary tuner resume boot query [%li] ", __func__, __LINE__, status);

		nmtp_chip_primary_set_status(nmtp, MRD_RESUME);

		status = wait_event_interruptible_timeout(reset_write_wait,
				(get_nmtp_state(mode)->reset_status == MRD_RESUME_CONFIRM_APP), msecs_to_jiffies(2 * 1000));
		if (status == 0)
			dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp: [Primary] Timeout not received confirmation from Tunerapp:\n",
								__func__, __LINE__);
		for (i = 0; i < NMTP_DEVNODES; i++)
			kfifo_reset(&nmtp->fifo[i]);

		for (i = 0; i < NMTP_DEVNODES; i++)
			nmtp->write_count[i] = 0;
		kfifo_reset(&nmtp->write_fifo);

		status = nmtp_sync(nmtp);
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] sync status primary %li\n",
								 __func__, __LINE__, status);

		// Retry on error (single tuner only)
		if ((status != 0) && (get_nmtp_details(1) == NULL)) {
			dev_dbg(&nmtp->spi->dev, "[Hnmtp]-- tuner sync failed - reset and retry\n");

			// Reset the chip
			status = nmtp_ipc_reset(nmtp);
			if (status) {
				dev_dbg(&nmtp->spi->dev, "[Hnmtp]-- tuner primary reset failed --[%s] [%d]\n", __func__, __LINE__);
			}

			// Wait for reset to complete
			status = nmtp_reset_ipc_query(nmtp);
			dev_dbg(&nmtp->spi->dev, "[Hnmtp]-- tuner Primary reset boot query [%d]--[%s] [%d]\n", status, __func__, __LINE__);

			// Attempt to re-sync
			status = nmtp_sync(nmtp);
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] sync status primary (after reset) %d\n",
									__func__, __LINE__, status);
		}

		hrtimer_start(&nmtp->hrtimer,
					nmtp->ktime, HRTIMER_MODE_ABS);
		//nmtp_chip_primary_set_status(nmtp, MRD_RESUME_DONE);


		if (get_nmtp_details(1) != NULL) {
			mode = 1;
			nmtp = get_nmtp_details(1);
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] secondary\n", __func__, __LINE__);

			nmtp_chip_secondary_set_status(nmtp, MRD_RESUME);

//			status = wait_event_interruptible_timeout(reset_write_wait,
//					(get_nmtp_state(mode)->reset_status == MRD_RESUME_CONFIRM_APP), 15 * HZ);

//			if (status == 0)
//				printk(KERN_ERR "nmtp: [Secondary] Timeout not received confirmation from Tunerapp: %s %d\n",
//						__func__, __LINE__);
			for (i = 0; i < NMTP_DEVNODES; i++)
				kfifo_reset(&nmtp->fifo[i]);

			for (i = 0; i < NMTP_DEVNODES; i++)
				nmtp->write_count[i] = 0;

			kfifo_reset(&nmtp->write_fifo);

			status = nmtp_sync(nmtp);

			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] sync status primary %li\n",
									__func__, __LINE__, status);
			hrtimer_start(&nmtp->hrtimer,
											nmtp->ktime, HRTIMER_MODE_ABS);
			nmtp_current_state_update(mode, MRD_NORMAL_OPERATION);
			wake_up_interruptible(&reset_poll_wait_secondary);
		}
		nmtp_current_state_update(0, MRD_NORMAL_OPERATION);
		wake_up_interruptible(&reset_poll_wait);

	}
}


static ssize_t nmtp_reset_read(struct file *f, char __user *buf, size_t count,
		loff_t *offset)
{
	struct nmtp_data *nmtp = f->private_data;
	u8 read_data;
	u8 mode = nmtp->type->mode;
	ssize_t status;
	int reset_status = get_nmtp_state(mode)->reset_status;

	dev_dbg_ratelimited(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] READ   add=%p  status %d PID [%d]\n",
					__func__, __LINE__, f, get_nmtp_state(mode)->reset_status, task_pid_nr(current));

	switch (reset_status) {

		case MRD_RESET_IN_PROGRESS:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] MRD_RESET_IN_PROGRESS\n", __func__, __LINE__);
			read_data = 0x0;
			break;

		case MRD_NORMAL_OPERATION:
			dev_dbg_ratelimited(&nmtp->spi->dev,
			"[Hnmtp] -- [%s] [%d] MRD_NORMAL_OPERATION\n", __func__, __LINE__);
			read_data = 0x1;
			break;

		case MRD_RESET_START:
		case MRD_RESET_START_APP:
		case MRD_RESET_CONFIRM_APP:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] MRD_RESET_START/START_APP/CONFIRM_APP\n", __func__, __LINE__);
			read_data = 0x2;
			break;

		case MRD_SUSPEND:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] MRD_SUSPEND\n", __func__, __LINE__);
			read_data = 0x3;
			break;

		case MRD_RESUME:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] MRD_RESUME\n", __func__, __LINE__);
			read_data = 0x4;
			break;

		case MRD_RESUME_DONE:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] MRD_RESUME_DONE\n", __func__, __LINE__);
			read_data = 0x1;
			nmtp_current_state_update(mode, MRD_NORMAL_OPERATION);
			break;
		default:
			dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] default\n", __func__, __LINE__);
			read_data = 0xF;
			break;
	}

	status = copy_to_user(buf, &read_data, 0x1);
	//trace_printk("-- %s %d %d --\n", __func__, __LINE__, (status ? status : 0x1));

	//return status ? status : 0x1;
	return 0x1;
}

static unsigned int nmtp_reset_poll(struct file *f, struct poll_table_struct *wait)
{
	int mask = 0;
	u8 state;
	struct nmtp_data *nmtp = f->private_data;
	u8 mode = nmtp->type->mode;

	//dev_dbg(&nmtp->spi->dev,"[Hnmtp] poll status [%s] [%d]\n", __func__, __LINE__);
	if (mode == 0)
		poll_wait(f, &reset_poll_wait, wait);
	else if (mode == 1)
		poll_wait(f, &reset_poll_wait_secondary, wait);

	if (get_nmtp_state(mode)->previous_state != get_nmtp_state(mode)->reset_status) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] --  [%s] [%d] change is nmtp state is detected\n", __func__, __LINE__);
		state = get_nmtp_state(mode)->reset_status;
		nmtp_previous_state_update(mode, state);
		mask = POLLIN | POLLRDNORM;
	}

	return mask;
}

static ssize_t nmtp_reset_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	struct nmtp_data *nmtp = f->private_data;
	u8 mode = nmtp->type->mode;
	ssize_t status;
	u8 data;

	static int confirm_count_tuners[2];
	int confirm_count = confirm_count_tuners[mode];

	status = copy_from_user(&data, buf, 0x1);

	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] WRITE  add=%p  status %d data = %d PID [%d]\n", __func__, __LINE__,
					 f,  get_nmtp_state(mode)->reset_status, data, task_pid_nr(current));
	if (status) {
		count = status;
		goto err;
	}

	/* ready for reset */
	if (data == 0x1) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp chip state = %d confirm count = %d--\n",
				__func__, __LINE__, get_nmtp_state(mode)->reset_status, confirm_count);
		if ((get_nmtp_state(mode)->reset_status == MRD_RESET_START) ||
			(get_nmtp_state(mode)->reset_status == MRD_RESET_START_APP)) {
			if (confirm_count == 0)
				confirm_count = 1;
			else if (confirm_count == 1) {
				get_nmtp_state(mode)->reset_status  = MRD_RESET_CONFIRM_APP;
				confirm_count = 0;
				dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] nmtp chip state = %d confirm count = %d--\n",
					__func__, __LINE__, get_nmtp_state(mode)->reset_status, confirm_count);
				wake_up_interruptible(&reset_write_wait);
			}
		} else {
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s][%d] nmtp chip state = %d confirm count = %d--\n",
				__func__, __LINE__, get_nmtp_state(mode)->reset_status, confirm_count);
		dev_dbg(&nmtp->spi->dev,
			"[Hnmtp] -- [%s][%d] Recieved confirm  write in a wrong nmtp state : [%d] PID:[%d]\n",
			__func__, __LINE__, get_nmtp_state(mode)->reset_status, task_pid_nr(current));

		count = 1;//-EIO;
		}
	}
	/* request for reset */
	else if (data == 0x0) {

		if (get_nmtp_state(mode)->reset_status != MRD_NORMAL_OPERATION) {
			nmtp_err("tuner chip is not ready for reset\n");
			count = -EBUSY;
			goto err;
		}
		if (nmtp->type->mode == 0) {
			confirm_count = 1;
			confirm_count_tuners[mode] = confirm_count;
			nmtp_chip_primary_set_status(nmtp, MRD_RESET_START_APP);
		} else if (nmtp->type->mode == 1) {
			confirm_count = 1;
			confirm_count_tuners[mode] = confirm_count;
			nmtp_chip_secondary_set_status(nmtp, MRD_RESET_START_APP);
		}
		//nmtp_chip_reset(nmtp, MRD_RESET_START_APP);
	} else if (data == 0x3) {
		if (get_nmtp_state(mode)->reset_status ==  MRD_RESUME) {
		nmtp_current_state_update(mode, MRD_RESUME_CONFIRM_APP);
		wake_up_interruptible(&reset_write_wait);
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s][%d] nmtp received confirmation from TunerApp\n",
					__func__, __LINE__);
		} else
			dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s][%d] Confirmation is not required %d\n",
					__func__, __LINE__, get_nmtp_state(mode)->reset_status);
	}

	else {
		count = -EIO;
	}
	confirm_count_tuners[mode] = confirm_count;
err:
	return count;
}

static int nmtp_reset_open(struct inode *i, struct file *f)
{
	struct nmtp_data *nmtp;
	int status = 0;

	mutex_lock(&nmtp_reset_lock);
	list_for_each_entry(nmtp, &device_list, device_entry) {
		if (nmtp->reset_devt == i->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status) {
		pr_err("[Hnmtp] -- nothing for minor %d\n", iminor(i));
		goto err_find_dev;
	}
	f->private_data = nmtp;
	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s][%d] open add=%p\n", __func__, __LINE__, f);
	//end

err_find_dev:
	mutex_unlock(&nmtp_reset_lock);
	return status;
}

static int nmtp_reset_release(struct inode *i, struct file *f)
{
	struct nmtp_data *nmtp = f->private_data;

	dev_dbg(&nmtp->spi->dev, "[Hnmtp] -- [%s] [%d] close add=%p\n", __func__, __LINE__, f);
	f->private_data = NULL;

	return 0;
}

static const struct file_operations nmtp_reset_fops = {
	.owner		= THIS_MODULE,
	.open		= nmtp_reset_open,
	.read		= nmtp_reset_read,
	.write		= nmtp_reset_write,
	.poll		= nmtp_reset_poll,
	.release	= nmtp_reset_release,
};


int nmtp_reset_init(struct nmtp_data *nmtp)
{
	unsigned long minor;
	struct device *dev;
	int status;

	//nmtp->reset_status = MRD_NORMAL_OPERATION;
	if (nmtp_reset_major == 0) {
		nmtp_reset_major = register_chrdev(0, "nmtp_reset", &nmtp_reset_fops);
		if (nmtp_reset_major < 0) {
			dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s][%d] MRD RESET Device Registration failed\n",
							__func__, __LINE__);
			return nmtp_reset_major;
		}

		nmtp_reset_class = class_create(THIS_MODULE, "nmtp_reset");
		if (nmtp_reset_class == NULL) {
			status = -EPERM;
			goto class_fail;
		}
	}

	minor = find_first_zero_bit(minors, DYNAMIC_MINORS);
	if (minor >= DYNAMIC_MINORS) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s][%d] no minor number available!\n",
							__func__, __LINE__);

		status = -ENODEV;
		goto end;
	}
	nmtp->reset_devt = MKDEV(nmtp_reset_major, minor);

	if (nmtp->type->mode == 0) {
		dev = device_create(nmtp_reset_class, &nmtp->spi->dev, nmtp->reset_devt,
				nmtp, "nmtp_master_reset");
	} else {
		dev = device_create(nmtp_reset_class, &nmtp->spi->dev, nmtp->reset_devt,
				nmtp, "nmtp_slave_reset");
	}
	status = PTR_ERR_OR_ZERO(dev);
	if (status) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] -- [%s][%d] Could not create device file for minor %lu\n",
				__func__, __LINE__, minor);

		goto end;
	}
	set_bit(minor, minors);
	list_add(&nmtp->device_entry, &device_list);

end:
class_fail:

	return 0;
}

void nmtp_reset_exit(struct nmtp_data *nmtp)
{
	static int a;

	device_destroy(nmtp_reset_class, nmtp->reset_devt);
	clear_bit(MINOR(nmtp->reset_devt), minors);
	if (a == 0x1) {
		list_del(&nmtp->device_entry);
		class_destroy(nmtp_reset_class);
		unregister_chrdev(nmtp_reset_major, "nmtp_reset");
	}
	a++;
}



