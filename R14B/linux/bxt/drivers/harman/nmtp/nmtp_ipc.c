/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include "linux/backing-dev.h"

#include <linux/wait.h> // Required for the wait queues
#include <linux/sched.h> // Required for task states (TASK_INTERRUPTIBLE etc )

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

#include "nmtp_plugin.h"

static wait_queue_head_t ioc_queue;
static DEFINE_MUTEX(ioc_lock);
static struct task_struct *iocReaderThread = NULL;

static volatile unsigned char ack_status;
static volatile unsigned char queury_status;
static volatile unsigned char secondary_boot_status;
static volatile unsigned char bandwidth_status;
static volatile unsigned char vip_busy;

#define CREATE_TRACE_POINTS

#define IPC_RESET_NMTP_CHANNEL                 12
#define IPC_INTERFACE_VERSION                  0x01

#define IPC_RESET_REQ_MSG                      0xD2
#define IPC_RESET_REQ_PAY_RST_REQ              0x00
#define IPC_RESET_REQ_PAY_RST_CONF             0x01

#define IPC_PRIMARY_TUNER_ID                   0x01
#define IPC_SECONDRY_TUNER_ID                  0x02

#define IPC_BOOTMODE_CONFIG                    0xD1
#define IPC_BOOTMODE_ROM					   0x00
#define IPC_BOOTMODE_HOST                      0x01
#define IPC_BOOTMODE_FLASH                     0x02

//Response from VIP
#define IPC_BOOTMODE_READY                     0x00
#define IPC_BOOTMODE_NOTVALID                  0x01
#define IPC_BOOTMODE_BUSY					   0x02
#define TUNER_ID_NOT_VALID					   0x03

#define IPC_RESET_REQ_STATUS_MSG               0xD2
#define IPC_RESET_REQ_STATUS_VIP_BUSY          0x02

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
#define IPC_REQUEST_SPI_READY		           0x01

static int send_message(unsigned char messageNumber, int payloadBytes,
						unsigned char byte1, unsigned char byte2)
{
	struct {
		unsigned char messageNumber;
		unsigned char interfaceVersion;
		unsigned char payload[2];
	} iocTxMessage;
	int ret;
	int message_size = 2+payloadBytes;

	if ((payloadBytes < 0) || (payloadBytes > 2))
		return -1;

	// Send a status request to the IOC
	iocTxMessage.messageNumber    = messageNumber;
	iocTxMessage.interfaceVersion = IPC_INTERFACE_VERSION;

	if (payloadBytes == 1) {
		iocTxMessage.payload[0] = byte1;
	} else {
		iocTxMessage.payload[0] = byte1;
		iocTxMessage.payload[1] = byte2;
	}

	mutex_lock(&ioc_lock);
	ret = ipc_channel_write(IPC_RESET_NMTP_CHANNEL, (char *)&iocTxMessage, message_size);
	mutex_unlock(&ioc_lock);

	return ret;
}

// Reset nmtp via IPC-IOC for reset.
int nmtp_reset_ipc_op(struct nmtp_data *nmtp)
{
	int status = 0;
	int timeout;
	long rc;

	mutex_lock(&ioc_lock);
	ack_status = 255;
	vip_busy = 0;
	mutex_unlock(&ioc_lock);

	send_message(IPC_RESET_REQ_MSG, 1, IPC_RESET_REQ_PAY_RST_REQ, 0);

	// Wait for up to 2 seconds for a pending message
	timeout = msecs_to_jiffies(2000);
	rc = wait_event_interruptible_timeout(ioc_queue, (ack_status != 255), timeout);

	mutex_lock(&ioc_lock);
	if (ack_status == IPC_RESET_REQ_ACK_PAY_DSP_PENDING)
		status = 1;
	mutex_unlock(&ioc_lock);
	
	if (status != 1) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] Request Reset status failed.  rc = %ld, ack_status = %d [Hnmtp] [%s]  [%d] !!!\n",
			rc, ack_status,
			__func__, __LINE__);
		return -1;
	}

	mutex_lock(&ioc_lock);
	ack_status = 255;
	mutex_unlock(&ioc_lock);
	send_message(IPC_RESET_REQ_MSG, 1, IPC_RESET_REQ_PAY_RST_CONF, 0);

	// Wait for up to 2 seconds for a pending message
	timeout = msecs_to_jiffies(2000);

	/* Read Max 50 times for 1 write*/
	status = 0;
	while (status == 0) {
		wait_event_interruptible_timeout(ioc_queue, (ack_status != 255), timeout);

		mutex_lock(&ioc_lock);

		if (ack_status == IPC_RESET_REQ_ACK_PAY_DSP_READY)
			status = 1;

		else if (ack_status == IPC_RESET_REQ_ERROR)
			status = -1;

		ack_status = 255;
	    mutex_unlock(&ioc_lock);
	}

	if (status != 1) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] Request Reset Error exiting [Hnmtp] [%s]  [%d] !!!\n",
			__func__, __LINE__);
		return -1;
	}

	/* Query the IPC channel .This should be checked if previous state is ready.*/
	status = nmtp_reset_ipc_query(nmtp);

	return status;

}

int nmtp_full_bandwidth_request(struct nmtp_data *nmtp)
{

	int status = -1;
	int timeout;

	mutex_lock(&ioc_lock);
	bandwidth_status = 255;
	mutex_unlock(&ioc_lock);

	send_message(IPC_REQUEST_SPI_BANDWIDTH, 1, 0, 0);

	// Wait for up to 2 seconds for a pending message
	timeout = msecs_to_jiffies(2000);
	wait_event_interruptible_timeout(ioc_queue, (bandwidth_status != 255), timeout);

	mutex_lock(&ioc_lock);
	status = bandwidth_status;
	mutex_unlock(&ioc_lock);

	return status;   //  0 on ready to handover  and 1 on vip busy
}

int nmtp_secondary_reset_ipc_op(struct nmtp_data *nmtp)
{
	int status = -1;
	int timeout;

	mutex_lock(&ioc_lock);
	secondary_boot_status = 255;
	mutex_unlock(&ioc_lock);

	send_message(IPC_BOOTMODE_CONFIG, 2, IPC_SECONDRY_TUNER_ID, IPC_BOOTMODE_FLASH);

	// Wait for up to 2 seconds for a pending message
	timeout = msecs_to_jiffies(2000);
	wait_event_interruptible_timeout(ioc_queue, (secondary_boot_status != 255), timeout);

	mutex_lock(&ioc_lock);
	if (secondary_boot_status == IPC_BOOTMODE_READY)
		status = 0;

	mutex_unlock(&ioc_lock);
	return status;   //0 on sucess
}


/* Query nmtp resetIPC-IOC for reset*/
int nmtp_reset_ipc_query(struct nmtp_data *nmtp)
{
	int timeout;
	int status = 0;
	long rc;

	// Initialize queury status
	mutex_lock(&ioc_lock);
	queury_status = 255;
	mutex_unlock(&ioc_lock);

	// Ask for the reset status
	send_message(IPC_RESET_QUERY_MSG, 1, 0, 0);

	// Wait for up to 200 msec for a pending message
//	timeout = msecs_to_jiffies(200);
	// Change the timeout to 500 msec in case a retry is required
	timeout = msecs_to_jiffies(500);
	rc = wait_event_interruptible_timeout(ioc_queue, (queury_status != 255), timeout);

	mutex_lock(&ioc_lock);
	if (queury_status == IPC_RESET_QUERY_ACK_READY)
		status = 1;
	mutex_unlock(&ioc_lock);
	
	if (status == 0) {
		dev_err(&nmtp->spi->dev, "[Hnmtp] Request Reset status failed.  rc = %ld, queury_status = %d [Hnmtp] [%s]  [%d] !!!\n",
			rc, queury_status,
			__func__, __LINE__);
	}

	return status;
}

//
// Thread that is used to continuously read messages from the VIP
//
static int IOCDataReader(void *data)
{
	struct {
		unsigned char messageNumber;
		unsigned char interfaceVersion;
		unsigned char payload[16];
	} iocRxMessage;
	int ret;

	// Send a status request to the IOC
	ret = send_message(IPC_RESET_QUERY_MSG, 1, 0, 0);
	if (ret <= 0)
		printk(KERN_ERR "[Hnmtp] Failed to send status request to IOC\n");

	while (!kthread_should_stop()) {
		// Read a message from the VIP
		ret = ipc_channel_read_timeout(IPC_RESET_NMTP_CHANNEL, (char *)&iocRxMessage, sizeof(iocRxMessage), 1000);
		if (kthread_should_stop())
			break;

		if (ret > 0) {
			switch (iocRxMessage.messageNumber) {
			case IPC_RESET_REQ_ACK_MSG:
				mutex_lock(&ioc_lock);
				ack_status = iocRxMessage.payload[2];
				mutex_unlock(&ioc_lock);

				//wake up all the tasks waiting
				wake_up_interruptible(&ioc_queue);
				break;

			case IPC_RESET_REQ_STATUS_MSG:
				mutex_lock(&ioc_lock);
				if (iocRxMessage.payload[0] == IPC_RESET_REQ_STATUS_VIP_BUSY)
					vip_busy = 1;
				else
					vip_busy = 0;
				mutex_unlock(&ioc_lock);

				//wake up all the tasks waiting
				wake_up_interruptible(&ioc_queue);
				break;

			case IPC_RESET_QUERY_ACK_MSG:
				mutex_lock(&ioc_lock);
				queury_status = iocRxMessage.payload[0];
				mutex_unlock(&ioc_lock);

				//wake up all the tasks waiting
				wake_up_interruptible(&ioc_queue);
				break;

			case IPC_BOOTMODE_CONFIG:
				mutex_lock(&ioc_lock);
				if (iocRxMessage.payload[0] == IPC_SECONDRY_TUNER_ID) {
					secondary_boot_status = iocRxMessage.payload[1];
				}
				mutex_unlock(&ioc_lock);

				//wake up all the tasks waiting
				wake_up_interruptible(&ioc_queue);
				break;

			case IPC_REQUEST_SPI_BANDWIDTH:
				mutex_lock(&ioc_lock);
				bandwidth_status = iocRxMessage.payload[0];
				mutex_unlock(&ioc_lock);

				//wake up all the tasks waiting
				wake_up_interruptible(&ioc_queue);
				break;

			default:
				// Log an error?
				break;
			}
		} else if (ret != -EAGAIN) {
			// Sleep for a while on an error so that we don't end up in a tight loop
			msleep(100);
		}

		if (kthread_should_stop())
			break;
	}

	ret = ipc_channel_close(IPC_RESET_NMTP_CHANNEL);
	if (ret < 0) {
		printk(KERN_ERR "[hnmtp] %s %d ipc channel close failed %d\n",
				__func__, __LINE__, ret);
	}

	return 0;
}

int ipc_channel_init(void)
{
	int ret = 0;
	int ioc_fd;

	ioc_fd = ipc_channel_open(IPC_RESET_NMTP_CHANNEL);
	if (ioc_fd < 0) {
		printk(KERN_ERR "[Hnmtp] Failed to open ipc channel\n");
		ret = -1;
	} else {
		if (iocReaderThread == NULL) {
		iocReaderThread = kthread_run(IOCDataReader, 0, "IOCDataReader");
		if (!iocReaderThread) {
				printk(KERN_ERR
					"[Hnmtp] Failed to create IOC reader thread\n");
			ret = -1;
		}
    }
	}
	return ret;
}

int nmtp_ipc_initialize(void)
{
	int ret = 0;

	mutex_init(&ioc_lock);
	init_waitqueue_head(&ioc_queue);

	ret = ipc_channel_init();	

	return ret;
}

void nmtp_ipc_exit(void)
{
	if (iocReaderThread) {
	kthread_stop(iocReaderThread);
		iocReaderThread = NULL;
	}
}
