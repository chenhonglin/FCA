/*
 * Author: Shrisha Krishna <Shrisha.Krishna@harman.com>
 *
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
#include "ioc-ipc.h"
#include "ioc-hal.h"

// Protects reading and writing over IPC
static DEFINE_MUTEX(rw_lock);

static ipc_plugin_t root;
static ipc_plugin_t root_ipc[NUMBER_OF_CHANNELS];
static tx_message_t tx_message;
static rx_message_t rx_message;
static ipc_queue_t  queue[NUMBER_OF_CHANNELS];
static ipc_message_t channel_info[NUMBER_OF_CHANNELS];

/* Global variable for the first device number */
static dev_t ipc_chdev;
/* Global variable for the device class */
static struct class *dev_class;

struct ioc_dev {
	struct device *dev;
	struct spi_device *spidev;
};

/** my_work_t instance **/
static recv_work_t *work;

static int first_message;  /*flag to check nack for first interrupt */
static unsigned long timeout;

/** Declare the workqueue struct **/
static struct workqueue_struct *ewq;

//#define DEBUG_FH

#ifdef DEBUG_FH
#define DEBUG_CHANNEL NUMBER_OF_CHANNELS
#define DEBUG_CHANNEL_NAME "ipcdebug"
struct cdev         debug_cdev;
#endif

#define RC_ACK_PIN 456

void dumpBuffer(unsigned char* pBuffer, unsigned int len, const char *func, int lineNum)
{
        int  wIndex = 0;
        unsigned char   buffer[512];

        //pthread_mutex_lock(&debugMutexHandle);
        debug("\n--> (Func: %s, Line:%d Len=%d) : ", func, lineNum, len);
        for(wIndex = 0 ; wIndex < len; wIndex += 8 )
        {
                sprintf(buffer, "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                        (unsigned int)pBuffer[wIndex], (unsigned int)pBuffer[wIndex+1], (unsigned int)pBuffer[wIndex+2], (unsigned int)pBuffer[wIndex+3],
                        (unsigned int)pBuffer[wIndex+4], (unsigned int)pBuffer[wIndex+5], (unsigned int)pBuffer[wIndex+6], (unsigned int)pBuffer[wIndex+ 7]
                        );

               debug("--> %s",  buffer);
        }

        //pthread_mutex_unlock(&debugMutexHandle);
}
EXPORT_SYMBOL(dumpBuffer);

/**
 * @brief This function gives the length of message
 * @retval No of bytes
 */
static int get_flow_control_msg_length(ipc_message_t *message)
{
	int i, bytes = 0;
	int count;
//	debug(" %s: %d - %s()\n", __FILE__, __LINE__, __func__);

	/* 1st byte(channel Number) + 2nd byte(IPC version) +
	   3rd byte(No of IPC channels) */
	bytes = FLOWCONTROL_MSG_OFFSET;
	count = NUMBER_OF_CHANNELS;
	for (i = 0; i < count; i++)
		bytes++;

	debug(" bytes : <%d>\n", bytes);

	return bytes;
}

/**
 * @brief This function fills the tx header buffer
 * @retval No of bytes of header buffer
 */
static void fill_tx_header(ipc_message_t *msg)
{
//	debug(" %s: %d - %s()", __FILE__, __LINE__, __func__);

	tx_message.header[0] = msg->command;
	if (msg->channel_no == IPC_FLOWCONTROL_CHANNEL)
		tx_message.header[1] = get_flow_control_msg_length(msg);
	else
		tx_message.header[1] = DATA_CHANNEL_OFFSET + msg->buffer_len;

	debug(" message.header[0] : <%x> message.header[1]: <%x>\n",
			tx_message.header[0], tx_message.header[1]);
}

/**
 * @brief This function fills the tx data buffer
 * @retval No of bytes filled
 */
static int fill_tx_data(ipc_message_t *msg_info)
{
	int i, ret = 0;

//	debug(" %s: %d - %s()", __FILE__, __LINE__, __func__);

		switch (msg_info->channel_no) {
		case IPC_FLOWCONTROL_CHANNEL:
			tx_message.data[0] = msg_info->channel_no;
			//debug(" tx_message.data[0]: <%x>\n",	tx_message.data[0]);
			tx_message.data[1] = msg_info->ipc_version;
			//debug(" tx_message.data[1]: <%x>\n",	tx_message.data[1]);
			tx_message.data[2] = msg_info->no_of_channels;
			//debug(" tx_message.data[2]: <%x>\n",	tx_message.data[2]);
			for (i = 0; i < msg_info->no_of_channels; i++) {
				if (channel_info[i].no_of_clients > 0) {
					tx_message.data[
						FLOWCONTROL_MSG_OFFSET+i] =
						channel_info[i].no_of_clients |
						(channel_info[i].rxflowctrlstate
						 << 7);
				} else {
					tx_message.data[3+i] = 0x01;
				}

			}
			ret = FLOWCONTROL_MSG_OFFSET + msg_info->no_of_channels;
			break;
                default:
                        if (msg_info->channel_no >= CHANNEL2_MINOR && msg_info->channel_no < NUMBER_OF_CHANNELS)
                        {
	                        tx_message.data[0] = msg_info->channel_no;
        	                for (i = 0; i < msg_info->buffer_len; i++)
                	                tx_message.data[i+1] = msg_info->buffer[i];
                        	ret = DATA_CHANNEL_OFFSET + msg_info->buffer_len;
			}
			else {
				debug(" Wrong channel : <%x>\n", msg_info->channel_no);
				ret = FAILURE;
			}
			break;
	}
	debug("IPC : ret = <%d>\n", ret);
	if(ret > 0)
		dumpBuffer(tx_message.data, ret, __func__, __LINE__);
	return ret;
}

/**
 * @brief This function completes the transmit sequence
 * @retval None
 */
static int transmit_sequence(ipc_plugin_t *ipc_handler, ipc_message_t *msg)
{
	int size_data;
	int ret_val;
//	debug(" %s: %d - %s()", __FILE__, __LINE__, __func__);
	/* fill tx header */
	fill_tx_header(msg);
	/* transmit the header to SPI */
	ret_val = transmit(ipc_handler, tx_message.header, HEADER_SIZE);

	if (ret_val < 0)
		return FAILURE;

	if (!wait_for_completion_timeout(&root.ack_done, timeout)) {
		error("header ack timeout\n");
		return FAILURE;
	}
	/* fill tx data */
	size_data = fill_tx_data(msg);
	/* transmit the data to SPI */
	if (size_data) {
		ret_val = transmit(ipc_handler, tx_message.data, size_data);
		if (ret_val < 0)
			return FAILURE;
	} else {
		return FAILURE;
	}

	if (!wait_for_completion_timeout(&root.ack_done, timeout)) {
		error("Data ack timeout\n");
		return FAILURE;
	}
	return SUCCESS;

}

// SWM - Never called?  Is this needed?
#if 0
static void flowctrlcheckstate(int chnum)
{
	ipc_message_t   *flowctrl_msg;
	int result;
//	debug(" %s: %d - %s()\n", __FILE__, __LINE__, __func__);


	if (IPC_XON == channel_info[chnum].rxflowctrlstate) {
		debug("IPC  %d - %s()\n",  __LINE__, __func__);
		if (IPC_XOFF_THRESHOLD <= msgqueue_used(&channel_info[chnum])) {
//		if (IPC_XOFF_THRESHOLD >= channel_info[chnum].nfreebuffers) {
			flowctrl_msg = msgqueue_alloc(&queue[chnum].q_ch_tx,
					sizeof(ipc_message_t));
			if (flowctrl_msg) {
				flowctrl_msg->channel_no =
					IPC_FLOWCONTROL_CHANNEL;
				flowctrl_msg->no_of_channels =
					NUMBER_OF_CHANNELS;
				flowctrl_msg->no_of_clients =
					channel_info[chnum].no_of_clients;
				flowctrl_msg->command = WR_OP;
				flowctrl_msg->ipc_version = IPC_VERSION;
				channel_info[chnum].rxflowctrlstate =
					IPC_XOFF;

				// Just free the buffer and "complete" even though nothing enqueued
				// msgqueue_free(&queue[chnum].q_ch_tx, flowctrl_msg);
				// channel_info[chnum].nfreebuffers = QUEUE_SIZE;
				result = msgqueue_enqueue(&queue[chnum].q_ch_tx,
						(void *)flowctrl_msg, NULL);
                if (result < 0) {
                    error("Error in XOFF msgqueue_enqueue() %d\n", result);
                    return;
                }
				complete(&root.msg_ready);
			}
		}
	} else {
		if (IPC_XON_THRESHOLD >= msgqueue_used(&channel_info[chnum])) {
//		if (IPC_XON_THRESHOLD <= channel_info[chnum].nfreebuffers) {
			debug("IPC %d - %s()\n", __LINE__, __func__);
			flowctrl_msg = msgqueue_alloc(&queue[chnum].q_ch_tx,
					sizeof(ipc_message_t));
			if (flowctrl_msg) {
				flowctrl_msg->channel_no =
					IPC_FLOWCONTROL_CHANNEL;
				flowctrl_msg->no_of_channels =
					NUMBER_OF_CHANNELS;
				flowctrl_msg->no_of_clients =
					channel_info[chnum].no_of_clients;
				flowctrl_msg->command = WR_OP;
				flowctrl_msg->ipc_version = IPC_VERSION;
				channel_info[chnum].rxflowctrlstate = IPC_XON;
				root.channelno = chnum;
				/* Push the message to channel transmit queue */
				result = msgqueue_enqueue(&queue[chnum].q_ch_tx,
						(void *)flowctrl_msg, NULL);
                                if (result < 0) {
                                    error("Error in XON msgqueue_enqueue() %d\n", result);
                                    return;
                                }
				complete(&root.msg_ready);
			}

		}
	}

}
#endif
/**
 * @brief This function parses the received message and pushing into the
 * respective queue
 * @retval None
 */
static void process_recvd_msg(void)
{
	int i, result;
	ipc_parse_message_t *parsed_msg;
	int channelnum;
//	debug(" %s: %d - %s()", __FILE__, __LINE__, __func__);

	channelnum = rx_message.payload[0];

	debug(" channelnum is %d, freebuffers is %d\n", channelnum,
			channel_info[channelnum].nfreebuffers);

		switch (channelnum) {
		case IPC_FLOWCONTROL_CHANNEL:
			parsed_msg = msgqueue_alloc(&queue[channelnum].q_ch_rx,
					sizeof(ipc_parse_message_t));
			if (parsed_msg)
			{
				parsed_msg->buffer_size = rx_message.header[1];

				debug(" in IPC_FLOWCONTROL_CHANNEL\n");
				/* Coping 1st byte */
				parsed_msg->channel_no = rx_message.payload[0];
				/* Coping 2nd byte */
				parsed_msg->ipc_version = rx_message.payload[1];
				/* Coping 3rd byte */
				parsed_msg->no_of_channels = rx_message.payload[2];
				/* Coping the remaining bytes */
				for (i = 0; i < parsed_msg->no_of_channels; i++) {
// SWM - This is just plain goofy.  Why continuously overwrite no_of_clients?
					parsed_msg->no_of_clients =
						rx_message.payload[
						FLOWCONTROL_MSG_OFFSET+i]
						& 0x7F;
					parsed_msg->state = rx_message.payload[3+i]
						>> 7;
				}
// SWM - why free it?
				msgqueue_free(&queue[channelnum].q_ch_rx, parsed_msg);
			}
			else
			{
				error("Could not allocate buffer for queue entry\n");
			}
			break;

		default:
			if (channelnum >= CHANNEL2_MINOR && channelnum < NUMBER_OF_CHANNELS)
			{
//				info(" Recvd Msg for CH- %d \n", channelnum);
				parsed_msg = msgqueue_alloc(&queue[channelnum].q_ch_rx,
						sizeof(ipc_parse_message_t));
				if (parsed_msg)
				{
					parsed_msg->buffer_size = rx_message.header[1];

					parsed_msg->channel_no = rx_message.payload[0];

					/* Coping remaining bytes */
					for (i = 0; i < (parsed_msg->buffer_size-1); i++) {
						parsed_msg->buffer[i] = rx_message.payload[1+i];
					}
					/* check for rx flowcontrol state*/
					// SWM - This was commented out in latest code from Sagar.  Is this correct?
					//flowctrlcheckstate(parsed_msg->channel_no);

					/* TBD: Wait untill free buffers are available */
					result = msgqueue_enqueue(&queue[channelnum].q_ch_rx,
							(void *)parsed_msg, NULL);
					if (result != EOK) {
						error("Error in msgqueue_enqueue() %d\n",
								result);
                                    break;
					}
					else
					{
						channel_info[channelnum].rx_count++;
					}

					complete(&root_ipc[channelnum].read_msg);
					wake_up_interruptible(&root_ipc[channelnum].readable);
				}
				else
				{
					error("Could not allocate buffer for queue entry\n");
				}
			}
			else
				error(" invalid channel\n");

			break;

	}
}

/**
 * @brief This function completes the receive sequence
 * @retval None
 */
static int receive_sequence(ipc_plugin_t *ipc_handler, ipc_message_t *msg)
{
	int i , ret_val = 0;
	int bytes_to_read;

//	debug(" %s: %d - %s()", __FILE__, __LINE__, __func__);

	/* 1)      Master sends 2 byte header with Read command */
	/* fill tx header */
	fill_tx_header(msg);
	/* transmit the header to SPI */
	ret_val = transmit(ipc_handler, tx_message.header, HEADER_SIZE);
	if (ret_val < 0)
		return FAILURE;

	/* 2)      Slave sends ACK pin low to High */
	debug(" waiting for ack\n");

	if (!wait_for_completion_timeout(&root.ack_done, timeout)) {
		debug("ack timeout\n");
		return FAILURE;
	}
	debug(" caught signal for header\n");

	/* 3)      Slave sends 2 byte header */
	memset(&rx_message, 0, sizeof(rx_message_t));
	ret_val = receive(ipc_handler, rx_message.header, HEADER_SIZE);

	if (ret_val < 0)
		return FAILURE;

	debug(" Recevied header from V850 ...\n");
	for (i = 0; i < HEADER_SIZE; i++)
		debug(" %x\n", rx_message.header[i]);

	/* 4)      Slave sends ACK pin low to High */
	debug(" waiting for ack\n");

	if (!wait_for_completion_timeout(&root.ack_done, timeout)) {
		debug("ack timeout\n");
		return FAILURE;
	}
	debug("IPC : caught signal for header\n");

	/* Calculating no of bytes to read from V850 */
	bytes_to_read = rx_message.header[1];

	if((bytes_to_read <= 0) || (bytes_to_read > 255)){
	      debug(">>>>!!!!!!!!!!error Reading...%d bytes\n", bytes_to_read);
		  return SUCCESS;
	}

	/* 5)      Slave sends n byte payload */
	debug(" Reading...%d bytes\n", bytes_to_read);
	ret_val = receive(ipc_handler, rx_message.payload, bytes_to_read);

	if (ret_val < 0)
		return FAILURE;

	debug(" Recevied payload from V850 ...\n");
	for (i = 0; i < bytes_to_read; i++)
		debug(" %x\n", rx_message.payload[i]);

	/* 6)      Slave sends ACK pin low to High */
	debug(" waiting for ack\n");
	if (!wait_for_completion_timeout(&root.ack_done, timeout)) {
		debug("ack timeout\n");
		return FAILURE;
	}
	debug(" caught signal for header\n");

	/* Processing the received message */
	process_recvd_msg();

	return SUCCESS;
}

/**
 * @brief This function sends the flow control message to V850
 * for initialization
 * @retval None
 */
static int intialization_phase(void)
{
	ipc_message_t *flowctrl_msg;
	ipc_message_t *recv_msg;

//	debug(" %s: %d - %s()\n", __FILE__, __LINE__, __func__);

	timeout = msecs_to_jiffies(IOC_ACK_TIMEOUT);
	debug("timeout value for ack is %lu in intialization_phase\n", timeout);

	/* Allocating memory for structures */
	flowctrl_msg = kzalloc(sizeof(ipc_message_t), GFP_KERNEL);
	recv_msg =  kzalloc(sizeof(ipc_message_t), GFP_KERNEL);

	/* sending */
	if (flowctrl_msg) {
		debug(" sending flowcontrol\n");
		flowctrl_msg->buffer_len = 0;
		flowctrl_msg->channel_no = IPC_FLOWCONTROL_CHANNEL;
		flowctrl_msg->command = WR_OP;
		flowctrl_msg->ipc_version = IPC_VERSION;
		flowctrl_msg->no_of_channels = NUMBER_OF_CHANNELS;
		flowctrl_msg->length_dma = DMA_LENGTH;
		channel_info[IPC_FLOWCONTROL_CHANNEL].rxflowctrlstate = IPC_XON;
		channel_info[IPC_FLOWCONTROL_CHANNEL].no_of_clients = 1;
		if (transmit_sequence(&root, flowctrl_msg) < 0)
			goto failure;
		debug(" Transmission completed\n");
	} else
		goto failure;

	/* receving */
	if (recv_msg) {
		debug(" recv_msg waiting for NACK");
		if (!wait_for_completion_timeout(&root.irq_recv, timeout)) {
			debug("time out while recv_msg waiting for NACK");
			return FAILURE;
		}
		first_message = 1;
		recv_msg->command = RD_OP;
		receive_sequence(&root, recv_msg);
		debug(" Handshake Receiving completed\n");
		kfree(flowctrl_msg);
		flowctrl_msg = NULL;
		kfree(recv_msg);
		recv_msg = NULL;
		goto success;
	} else
		goto failure;

failure:
	return FAILURE;
success:
	return SUCCESS;
}

/* Work queue handler */
static void recv_work_handler(struct work_struct *pwork)
{
	ipc_message_t *recv_msg;
//	debug(" %s: %d - %s()\n", __FILE__, __LINE__, __func__);
	recv_msg =  kzalloc(sizeof(ipc_message_t), GFP_KERNEL);
	memset(recv_msg, 0 , sizeof(ipc_message_t));
	if (recv_msg) {
		debug(" in work handler for receving\n");
		recv_msg->command = RD_OP;
		mutex_lock(&rw_lock);
		receive_sequence(&root, recv_msg);
		kfree(recv_msg);
		recv_msg = NULL;
		mutex_unlock(&rw_lock);
	}
}

/**
 * @brief plugin_send_thread
 * @retval 0
 */
static int plugin_send_thread(void *arg)
{
	int result , ret_val = 0;
	ipc_plugin_t *root = (ipc_plugin_t *)arg;
	ipc_message_t *msg;
	int i = 0;
//	debug(" %s: %d - %s()\n", __FILE__, __LINE__, __func__);

	if (root)
		debug(" root is valid\n");

	debug("devname is %s\n", dev_name(root->dev));

	allow_signal(SIGINT);
	while (!kthread_should_stop()) {
		ret_val = wait_for_completion_interruptible(&root->msg_ready);
		if (ret_val == -ERESTARTSYS) {
			debug("interrupted\n");
			return -EINTR;
		}
		for (i = 0; i < NUMBER_OF_CHANNELS; i++) {
			while (msgqueue_used(&queue[i].q_ch_tx) > 0) {
				/* Pop the message into channel transmit queue*/
				result = msgqueue_dequeue(&queue[i].q_ch_tx,
						(void **)&msg, NULL);
				if((result == EOK) && msg) {
					mutex_lock(&rw_lock);
					ret_val = transmit_sequence(root, msg);
					mutex_unlock(&rw_lock);

					// Need to free the message even if transmit failed
					msgqueue_free(&queue[i].q_ch_tx, msg);
					if (ret_val < 0) {
						error("Transmit_sequence failed\n");
						//goto failure;
					}
				}
				else {
					break;
				}

			}
		}
	}
failure:
	if (!IS_ERR_OR_NULL(root->send_thread_id)) {
		send_sig(SIGINT, root->send_thread_id, 1);
		kthread_stop(root->send_thread_id);
	}
	return FAILURE;
}

int channel_open(int ch_num)
{
	ipc_message_t   *received_msg;
	int result;
	debug(" %s: %d - %s()\n", __FILE__, __LINE__, __func__);

	//if (ch_num != IPC_SYSTEM_CHANNEL)
	{
		if (channel_info[ch_num].channel_in_use == CHANNEL_OPEN) {
			error("channel %d is already opened\n", ch_num);
			return -EPERM;
		}
	}

	if (1 == channel_info[ch_num].debug_on)
	{
		char debugStr[256];
		sprintf(debugStr, "ioc-ipc: Channel %d opened", ch_num);
		pr_info("[ IOC-IPC ] [INF] %s\n", debugStr);
	}

	received_msg = msgqueue_alloc(&queue[ch_num].q_ch_tx,
			sizeof(ipc_message_t));
	if (received_msg) {
		received_msg->channel_no = IPC_FLOWCONTROL_CHANNEL;
		received_msg->no_of_channels = NUMBER_OF_CHANNELS;
		received_msg->command = WR_OP;
		received_msg->ipc_version = IPC_VERSION;
		debug(" chnum is %d\n", ch_num);
		channel_info[ch_num].rxflowctrlstate = IPC_XON;
		channel_info[ch_num].no_of_clients = 1;
		channel_info[ch_num].channel_in_use = CHANNEL_OPEN;
		/* Push the message into channel01 transmit queue */;
		root.channelno = ch_num;
		result = msgqueue_enqueue(&queue[ch_num].q_ch_tx,
				(void *)received_msg, NULL);
		if (result < 0) {
			error("Error in open msgqueue_enqueue() %d\n", result);
			return result;
		}
		complete(&root.msg_ready);
	} else {
		return FAILURE;
	}
	return SUCCESS;
}
EXPORT_SYMBOL_GPL(channel_open);

static int channel_write(int chnum, const char __user *buf, size_t count)
{
	int result;
	ipc_message_t   *received_msg;

//	debug("%s: %d - %s()\n", __FILE__, __LINE__, __func__);

	received_msg = msgqueue_alloc(&queue[chnum].q_ch_tx,
			sizeof(ipc_message_t));
	if (received_msg) {
		debug("CH-%d: TX_msg size %d\n", chnum, (int)count);
		received_msg->buffer_len = count;
		received_msg->channel_no = chnum;
		received_msg->command = WR_OP;
		received_msg->ipc_version = IPC_VERSION;
		received_msg->length_dma = DMA_LENGTH;
		received_msg->state = 1;

		if (copy_from_user(received_msg->buffer, buf, count) != 0)
			return FAILURE;

		if (1 == channel_info[chnum].debug_on)
		{
			char debugStr[256];
			int size;
			int i = 0;
			size = sprintf(debugStr, "ioc-ipc: Write %d bytes to channel %d:", (int)count, chnum);
			while ((i < count) && (size < 250))
			{
				size += sprintf(&debugStr[size], " 0x%x", received_msg->buffer[i++]);
			}
			pr_info("[ IOC-IPC ] [INF] %s\n", debugStr);
		}

		root.channelno = chnum;
		/* Push the message into channel transmit queue */
		result = msgqueue_enqueue(&queue[chnum].q_ch_tx,
				(void *)received_msg, NULL);
		if (result < 0) {
			error("Error in write msgqueue_enqueue() %d\n", result);
			return result;
		}
		else
		{
			channel_info[chnum].tx_count++;
		}

		complete(&root.msg_ready);
	} else {
		return FAILURE;
	}

	return count;

}

static int channel_read(struct file *filp, int chnum, char __user *buf)
{
	int result, i, ret_val;
	ipc_parse_message_t *msg;
	debug("%s: %d - %s()\n", __FILE__, __LINE__, __func__);

	if (msgqueue_used(&queue[chnum].q_ch_rx) < 1) {
		debug("There is no msges\n");
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
	}

	ret_val = wait_for_completion_interruptible(&root_ipc[chnum].read_msg);
	if (ret_val == -ERESTARTSYS) {
		error("interrupted in read\n");
		return -EINTR;
	}

	/* Pop the message from channel receive queue */
	result = msgqueue_dequeue(&queue[chnum].q_ch_rx,
			(void **)&msg, NULL);
	if (result < 0) {
		error("Error msgdequeue %d\n", result);
            return result;
	}

	for (i = 0; i < msg->buffer_size; i++) {
		debug("pushing buffer msg->buffer[%d] <%x>\n",
				i, msg->buffer[i]);
	}

	if (1 == channel_info[chnum].debug_on)
	{
		char debugStr[256];
		int size;
		int i = 0;
		size = sprintf(debugStr, "ioc-ipc: Read %d bytes from channel %d:",  msg->buffer_size-1, chnum);
		while ((i < (msg->buffer_size-1)) && (size < 250))
		{
			size += sprintf(&debugStr[size], " 0x%x", msg->buffer[i++]);
		}
		pr_info("[ IOC-IPC ] [INF] %s\n", debugStr);
	}

	debug(">>pushing buffer msg->buffer_size : %d\n", msg->buffer_size);
	/* Transfering buffer to user space */
	if(msg->buffer_size>0){
		result = copy_to_user(buf, msg->buffer, msg->buffer_size - 1);
		if (result) {
			error("copy_to_user() could not copy %d bytes.\n",
					result);
			return -EFAULT;
		}
	}
	result = msg->buffer_size - 1;
	msgqueue_free(&queue[chnum].q_ch_rx, msg);
	return result;
}

static int channel_poll(struct file *file, int chnum, struct poll_table_struct *pt)
{
	unsigned int mask = 0;

	/* always call poll_wait, even if there is no reason to wait.*/
	poll_wait(file, &root_ipc[chnum].readable, pt);

	if (msgqueue_used(&queue[chnum].q_ch_rx) > 0) {
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

int channel_close(int chnum)
{
	ipc_message_t   *flowctrl_msg;
	int result;
	debug("%s: %d - %s()\n", __FILE__, __LINE__, __func__);

	if (1 == channel_info[chnum].debug_on)
	{
		char debugStr[256];
		sprintf(debugStr, "ioc-ipc: Channel %d closed", chnum);
		pr_info("[ IOC-IPC ] [INF] %s\n", debugStr);
	}

	flowctrl_msg = msgqueue_alloc(&queue[chnum].q_ch_tx,
			sizeof(ipc_message_t));
	if (flowctrl_msg) {
		flowctrl_msg->channel_no = IPC_FLOWCONTROL_CHANNEL;
		flowctrl_msg->no_of_channels = NUMBER_OF_CHANNELS;
		flowctrl_msg->command = WR_OP;
		flowctrl_msg->ipc_version = IPC_VERSION;
		debug("chnum is %d\n", chnum);
		channel_info[chnum].rxflowctrlstate = IPC_XOFF;
		channel_info[chnum].no_of_clients = 0;
		channel_info[chnum].channel_in_use = CHANNEL_CLOSE;
		/* Push the message into channel01 transmit queue */;
		root.channelno = chnum;
		result = msgqueue_enqueue(&queue[chnum].q_ch_tx,
				(void *)flowctrl_msg,	NULL);
		if (result < 0) {
			error("Error in close msgqueue_enqueue() %d\n", result);
			return result;
		}
		complete(&root.msg_ready);
	} else {
		return FAILURE;
	}
	return SUCCESS;
}
EXPORT_SYMBOL_GPL(channel_close);

static ssize_t channeln_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	int read_bytes = 0;
	unsigned int ch_num;

	if(filp->private_data) {
		ch_num = *((unsigned int *)filp->private_data);

		debug("%d - %s() - ch_num = %d\n", __LINE__, __func__, ch_num);
		read_bytes = channel_read(filp, ch_num, buf);
	}
	else
		return -EINVAL;

	return read_bytes;
}

static ssize_t channeln_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	int len = 0;
	unsigned int ch_num;

	if(filp->private_data) {
		ch_num = *((unsigned int *)filp->private_data);

		debug("%d - %s() - ch_num = %d\n", __LINE__, __func__, ch_num);
		len = channel_write(ch_num, buf, count);
	}
	else
		return -EINVAL;

	return len;
}

static unsigned int channeln_poll(struct file *filp, struct poll_table_struct *pt)
{
	unsigned int mask = 0;
	unsigned int ch_num;

	if(filp->private_data) {
		ch_num = *((unsigned int *)filp->private_data);

		debug("%d - %s() - ch_num = %d\n", __LINE__, __func__, ch_num);
		mask = channel_poll(filp, ch_num, pt);
	}
	else
	{
		mask = POLLERR;
	}

	return mask;
}

static int channeln_close(struct inode *inode, struct file *filp)
{
	unsigned int ch_num;

	if(filp->private_data) {
		ch_num = *((unsigned int *)filp->private_data);

		debug("%d - %s() - ch_num = %d\n", __LINE__, __func__, ch_num);
		if (channel_close(ch_num))
			return SUCCESS;
		else
			return FAILURE;
	}
	else
		return -EINVAL;
}

#ifdef DEBUG_FH
static ssize_t debug_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	int read_bytes = 0;
	int result;
	char debugStr[2048];
	int size;
	int i;

    size = sprintf(debugStr, "Channel 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28\nState  ");
    for (i = CHANNEL2_MINOR; i <= CHANNEL_DEV_MAX; ++i)
    {
        size += sprintf(&debugStr[size], "  %c", (channel_info[i].channel_in_use ? 'O' : 'C'));
	}

	size += sprintf(&debugStr[size], "\n\nChannel Stats\n");

	for (i = CHANNEL2_MINOR; i <= CHANNEL_DEV_MAX; ++i)
	{
		size += sprintf(&debugStr[size], "c%02d r%08d rq%08d t%08d tq%08d \n", i,
			channel_info[i].rx_count, msgqueue_used(&queue[i].q_ch_rx), channel_info[i].tx_count,
			msgqueue_used(&queue[i].q_ch_tx));
	}

	read_bytes = min(count, (strlen(debugStr) - (int)*f_pos));


	/* Transfering buffer to user space */
	if (read_bytes > 0)
	{
		result = copy_to_user(buf, &debugStr[*f_pos], read_bytes);

		if (result) {
			error("copy_to_user() could not copy %d bytes.\n",
					result);
			return -EFAULT;
		}

		*f_pos += read_bytes;
	}
	else
	{
		read_bytes = -1;
	}

	return read_bytes;
}

static int toHexError = 0;
static char toHex(char hexValue)
{
    if ((hexValue >= '0') && hexValue <= '9')
    {
        return (hexValue - '0');
    }
    if ((hexValue >= 'a') && hexValue <= 'f')
    {
        return (10 + (hexValue - 'a'));
    }
    if ((hexValue >= 'A') && hexValue <= 'F')
    {
        return (10 + (hexValue - 'A'));
    }
	toHexError = 1;
    return 0;
}


static ssize_t debug_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	char debug_buffer[256];
	char simulate_buffer[256];
	int passed_len = min(count, sizeof(debug_buffer)-1);
	int channelNum;
	ipc_parse_message_t *parsed_msg = NULL;
	int result = EOK;
    int ret_len = -1;

	if (copy_from_user(debug_buffer, buf, passed_len) == 0)
	{
		// Trim off any trailing newline character
		if ( '\n' == debug_buffer[passed_len-1])
		{
			--passed_len;
		}
		debug_buffer[passed_len] = 0;

		if (1 == sscanf(debug_buffer, "enableTrace(%d)", &channelNum))
		{
			if ((channelNum >= CHANNEL2_MINOR) && (channelNum <= CHANNEL_DEV_MAX))
			{
				pr_info("[ IOC-IPC ] [INF] Turning on tracing for channel %d", channelNum);
				channel_info[channelNum].debug_on = 1;
                ret_len = passed_len;
			}
			else
			{
				pr_info("[ IOC-IPC ] [INF] Bad channel number (%d) passed to enableTrace command", channelNum);
			}
		}
		else if (1 == sscanf(debug_buffer, "disableTrace(%d)", &channelNum))
		{
			if ((channelNum >= CHANNEL2_MINOR) && (channelNum <= CHANNEL_DEV_MAX))
			{
				pr_info("[ IOC-IPC ] [INF] Turning off tracing for channel %d", channelNum);
				channel_info[channelNum].debug_on = 0;
                ret_len = passed_len;
			}
			else
			{
				pr_info("[ IOC-IPC ] [INF] Bad channel number (%d) passed to disableTrace command", channelNum);
			}
		}
		else if (2 == sscanf(debug_buffer, "simulate(%d) %s", &channelNum, simulate_buffer))
		{
			if ((channelNum >= CHANNEL2_MINOR) && (channelNum <= CHANNEL_DEV_MAX))
			{
                // Create a message queue entry for the new message
				parsed_msg = msgqueue_alloc(&queue[channelNum].q_ch_rx,
						sizeof(ipc_parse_message_t));
				if (parsed_msg)
				{
                    int buffer_size = 0;

                    // Convert the passed ASCII to hex and put it into the message buffer
                    int final_size = strlen(simulate_buffer)/2;
					toHexError = 0;
                    for (buffer_size = 0; buffer_size < final_size; ++buffer_size)
                    {
                        parsed_msg->buffer[buffer_size] = (toHex(simulate_buffer[2*buffer_size]) << 4) + toHex(simulate_buffer[2*buffer_size+1]);
                    }

					if (toHexError || ((final_size * 2) != strlen(simulate_buffer)))
					{
						pr_info("[ IOC-IPC ] [INF] Simulate channel %d - error in passed string %s ", channelNum, simulate_buffer);
					}
					else
					{
						parsed_msg->buffer_size = buffer_size+1;
						parsed_msg->channel_no = channelNum;

						result = msgqueue_enqueue(&queue[channelNum].q_ch_rx,
								(void *)parsed_msg, NULL);
						if (result != EOK)
						{
							pr_info("[ IOC-IPC ] [INF] Simulate channel %d - error enqueuing string %s ", channelNum, simulate_buffer);
						}
						else
						{
							complete(&root_ipc[channelNum].read_msg);

							// Wake up any pending reader thread
							wake_up_interruptible(&root_ipc[channelNum].readable);

							pr_info("[ IOC-IPC ] [INF] Simulating channel %d and passing string %s ", channelNum, simulate_buffer);
						}
					}

                    ret_len = passed_len;
				}
				else
				{
					pr_info("[ IOC-IPC ] [INF] Could not allocate memory to simulate channel %d and pass string %s ", channelNum, simulate_buffer);
				}
			}
			else
			{
				pr_info("[ IOC-IPC ] [INF] Bad channel number (%d) passed to simulate command", channelNum);
			}
		}
		else
		{
			pr_info("[ IOC-IPC ] [INF] Unknown debugging command %s", debug_buffer);
		}
	}

	return ret_len;
}

static unsigned int debug_poll(struct file *filp, struct poll_table_struct *pt)
{
	return 	(POLLIN | POLLRDNORM);
}

static int debug_close(struct inode *inode, struct file *filp)
{
	return SUCCESS;
}

static const struct file_operations fopsdebug = {
	.owner        = THIS_MODULE,
	.read         = debug_read,
	.write        = debug_write,
	.poll			= debug_poll,
	.release		= debug_close,
};

static int debug_open(struct inode *inode, struct file *filp)
{
	unsigned int    	minor = iminor(inode);

	debug("Opened the debug interface\n");

	/* and use filp->private_data to point to the device data */
	if (DEBUG_CHANNEL == minor)
	{
		filp->f_op = &fopsdebug;
		filp->f_pos = 0;
	}
	else
	{
		error( "%s: Invalid IPC debug channel/minor %u ",  __FUNCTION__, minor);
		return -EINVAL;
	}

	return SUCCESS;
}

static const struct file_operations debug_fops = {
	.open = debug_open, /* just a selector for the real open */
};

#endif

static const struct file_operations fopschanneln = {
	.owner        = THIS_MODULE,
	.read         = channeln_read,
	.write        = channeln_write,
	.poll			= channeln_poll,
	.release		= channeln_close,
};

static struct {
	unsigned int minor;
	char *name;
	const struct file_operations *fops;
} devlist[] = { /* list of minor devices */
	{ 2, "ipc2", &fopschanneln},
	{ 3, "ipc3", &fopschanneln},
	{ 4, "ipc4", &fopschanneln},
	{ 5, "ipc5", &fopschanneln},
	{ 6, "ipc6", &fopschanneln},
	{ 7, "ipc7", &fopschanneln},
	{ 8, "ipc8", &fopschanneln},
	{ 9, "ipc9", &fopschanneln},
	{10, "ipc10", &fopschanneln},
	{11, "ipc11", &fopschanneln},
	{12, "ipc12", &fopschanneln},
	{13, "ipc13", &fopschanneln},
	{14, "ipc14", &fopschanneln},
	{15, "ipc15", &fopschanneln},
	{16, "ipc16", &fopschanneln},
	{17, "ipc17", &fopschanneln},
	{18, "ipc18", &fopschanneln},
	{19, "ipc19", &fopschanneln},
	{20, "ipc20", &fopschanneln},
	{21, "ipc21", &fopschanneln},
	{22, "ipc22", &fopschanneln},
	{23, "ipc23", &fopschanneln},
	{24, "ipc24", &fopschanneln},
	{25, "ipc25", &fopschanneln},
	{26, "ipc26", &fopschanneln},
	{27, "ipc27", &fopschanneln},
	{28, "ipc28", &fopschanneln},
	{29, "ipc29", &fopschanneln},
};


static int device_open(struct inode *inode, struct file *filp)
{
	unsigned int    	minor = iminor(inode);

	debug("Opened the device device_open minor = %d\n", minor);

	/* and use filp->private_data to point to the device data */
	if (minor >= CHANNEL2_MINOR && minor < NUMBER_OF_CHANNELS)
	{
		filp->private_data = &(devlist[minor-CHANNEL2_MINOR].minor);
		filp->f_op = &fopschanneln;
	}
	else
	{
		error( "%s: Invalid IPC channel/minor %u ",  __FUNCTION__, minor);
		return -EINVAL;
	}

	if (channel_open(minor) < 0)
		return -EPERM;

	return SUCCESS;
}


static const struct file_operations channel_fops = {
	.open = device_open, /* just a selector for the real open */
};

static const struct spi_device_id ipc_ids[] = {
	{"ioc-ipc"},
	{ },
};

static irqreturn_t ipc_ack_interrupt(int irq, void *dev_id)
{
	debug(" %d - %s()*******************************",
			__LINE__, __func__);
	complete(&root.ack_done);
	debug("ack done signal\n");
	return IRQ_HANDLED;
}

static irqreturn_t ipc_nack_interrupt(int irq, void *dev_id)
{
	debug(" %d - %s()&&&&&&&&&&&&&&&&&&&&&&&&&&",
			 __LINE__, __func__);
	if (first_message) {
		debug("Push!! work into the ewq--Queue Work..\n");
		queue_work(ewq, &(work->recv_work));
	}
	complete(&root.irq_recv);
	debug("interuupt recvd\n");
	return IRQ_HANDLED;
}

static int register_channels(void)
{
	int init_result , i = 0;
	dev_t devt;

	init_result = alloc_chrdev_region(&ipc_chdev, 2, CHANNEL_DEV_MAX,
			"ioc-ipc");
	if (init_result < 0)
		debug("IPC %s:alloc failed\n", __FILE__);

	if (register_chrdev(MAJOR(ipc_chdev), "ioc-ipc-ch", &channel_fops)) {
		error("unable to get major %d for ipc channel devs\n",
				MAJOR(ipc_chdev));
	} else {
		debug("got major %d\n", MAJOR(ipc_chdev));
	}
	dev_class = class_create(THIS_MODULE, "ioc-ipc");
	for (i = 0; i < CHANNEL_DEV_MAX; i++) {
		if (i >= CHANNEL2_MINOR && i < NUMBER_OF_CHANNELS)
		{
			cdev_init(&root_ipc[i].cdev, &channel_fops);

			devt = MKDEV(MAJOR(ipc_chdev), i);
			init_result = cdev_add(&root_ipc[i].cdev, devt, 1);
			if (init_result < 0) {
				error("Error %d adding cdev for channel %u\n", init_result, i);
				return -1;
			}
			device_create(dev_class, NULL, MKDEV(MAJOR(ipc_chdev),
				devlist[i-CHANNEL2_MINOR].minor), NULL, devlist[i-CHANNEL2_MINOR].name);
		}
	}

#ifdef DEBUG_FH
	cdev_init(&debug_cdev, &debug_fops);
	devt = MKDEV(MAJOR(ipc_chdev), DEBUG_CHANNEL);
	init_result = cdev_add(&debug_cdev, devt, 1);
	if (init_result < 0) {
		error("Error %d adding cdev for debug channel\n", init_result);
		return -1;
	}
	device_create(dev_class, NULL, devt, NULL, DEBUG_CHANNEL_NAME);
#endif

	return SUCCESS;
}

static int spi_ipc_probe(struct spi_device *dev)
{
	struct ioc_dev *item;
	int ret , rc_ack, rc_nack, i = 0;
	//struct device_node *np = dev->dev.of_node;
	int result;
	int status;
	unsigned char dummy[] = { DUMMY_DATA , DUMMY_DATA1};

	info(" %s: %d - %s()\n", __FILE__, __LINE__, __func__);

	// Determine if the VIP is set up to use
	//  single-spi or dual-spi IPC by checking
	//  the initial value of the "ack" pin
	if ((!gpio_is_valid(RC_ACK_PIN))
	 || (gpio_request(RC_ACK_PIN, "ipc_ack_pin"))
	 || (gpio_direction_input(RC_ACK_PIN))) {
		error("Setting up GPIO pin for ipc type failed\n");
		return -ENODEV;
	}

	// Check if line is low
	if (0 == gpio_get_value(RC_ACK_PIN))
	{
		// Check again after a couple of microseconds to make sure
		udelay(2);
		if (0 == gpio_get_value(RC_ACK_PIN))
		{
			// Line is low - It's dual spi IPC
			gpio_free(RC_ACK_PIN);
			return -ENODEV;
		}
	}

	item = kzalloc(sizeof(struct ioc_dev), GFP_KERNEL);
	if (!item) {
		dev_err(&dev->dev,
			"%s: unable to kzalloc for ioc_dev\n", __func__);
		ret = -ENOMEM;
	}

	memset(&channel_info, 0, sizeof(ipc_message_t));
	memset(&root, 0 , sizeof(ipc_plugin_t));

	for (i = 0; i < NUMBER_OF_CHANNELS; i++)
		memset(&root_ipc, 0, sizeof(ipc_plugin_t));

	item->spidev = dev;
	item->dev = &dev->dev;
	root.dev = item->dev;
	root.spidev = dev;
	dev_set_drvdata(&dev->dev, item);
	dev->bits_per_word = 8;
	dev->max_speed_hz = 5000000;


	debug(" %s: spi->mode = 0x%x \n", __func__,dev->mode);
        dev->mode = SPI_MODE_1;
        //dev->mode = SPI_MODE_2; /* ONLY MODE_! is working ...*/
	debug(" %s:New spi->mode = 0x%x \n", __func__, dev->mode);

	ret = spi_setup(dev);
	if (ret < 0)
		error("spi_setup failed\n");

	init_completion(&root.msg_ready);
	init_completion(&root.ack_done);
	init_completion(&root.irq_recv);
	init_completion(&root.read_msg);

	init_waitqueue_head(&root.readable);
	init_waitqueue_head(&root.writeable);

	for (i = 0; i < NUMBER_OF_CHANNELS; i++)
		init_completion(&root_ipc[i].read_msg);

	for (i = 0; i < NUMBER_OF_CHANNELS; i++)
		init_waitqueue_head(&root_ipc[i].readable);

	mutex_init(&rw_lock);

	debug(" spi registered, item=0x%p\n", (void *)item);

	if (register_channels() < 0)
		goto register_fail;

	timeout = msecs_to_jiffies(IOC_ACK_TIMEOUT);
	debug("timeout value for ack is %lu\n", timeout);

	debug("resgistering gpios...\n");

	//rc_ack = of_get_named_gpio(np, "ack_gpios", 0);

	rc_ack = 456;
#if 0 // This is now done when determining IPC type
	if (!gpio_is_valid(rc_ack))
		return -ENODEV;
	status = gpio_request(rc_ack, "ipc_ack_pin");
	if (status) {
		info("Claiming gpio pin %d failed!\n", rc_ack);
		return FAILURE;
	}
	gpio_direction_input(rc_ack);
#endif
	//rc_nack = of_get_named_gpio(np, "req_gpios", 0);
	rc_nack = 463;
	if (!gpio_is_valid(rc_nack))
		return -ENODEV;
	status = gpio_request(rc_nack, "ipc_req_pin");
	if (status) {
		info("Claiming gpio pin %d failed!\n", rc_nack);
		return FAILURE;
	}
	gpio_direction_input(rc_nack);

	ret = request_irq(gpio_to_irq(rc_ack), ipc_ack_interrupt,
			IRQ_TYPE_EDGE_FALLING,
			dev_name(&dev->dev), root.dev);
	if (ret)
		info("request_irq error\n");

	ret = request_irq(gpio_to_irq(rc_nack), ipc_nack_interrupt,
			IRQ_TYPE_EDGE_FALLING,
			dev_name(&dev->dev), root.dev);
	if (ret)
		error("request_irq nack error\n");

	debug("IPC %s..create - plugin_send_thread", __func__);
	root.send_thread_id = kthread_create(plugin_send_thread,
			&root, "plugin_send_thread");
	if (IS_ERR(root.send_thread_id))
		error("IPC %s:Can't create sendthread\n", __func__);
	else {
		debug("waking up thread\n");
		wake_up_process(root.send_thread_id);
	}

	/* Initialize the rx and tx queues for channel 4,5,7 */
	for (i = 0; i < NUMBER_OF_CHANNELS; i++) {
		result = msgqueue_init(&queue[i].q_ch_rx, "channel_rx");
		debug("channel_rx: <%p>, result : <%d>, %d\n",
				&queue[i].q_ch_rx, result, i);
		result = msgqueue_init(&queue[i].q_ch_tx, "channel_tx");
		debug("ch01_tx: <%p>, result : <%d>, %d\n",
				&queue[i].q_ch_tx, result, i);

		channel_info[i].rxflowctrlstate = IPC_XOFF;
	}


	work = kzalloc(sizeof(recv_work_t), GFP_KERNEL);

	/** Init the work struct with the work handler **/
	INIT_WORK(&(work->recv_work), recv_work_handler);

	if (!ewq) {
		info(" ewq..Single worker Thread created\n");
		ewq = create_workqueue("eWorkqueue");
	}

	debug(" wiritng dummy data %lu\n", sizeof(dummy));
	spi_write(dev, dummy, sizeof(dummy));
	debug(" written dummy byte---------------\n");

	if (!wait_for_completion_timeout(&root.ack_done, timeout)) {
		error("dummy data ack timeout\n");
		return FAILURE;
	}

	if (intialization_phase() < 0) {
		error("Error in intialization_phase()\n");
//workaround patch for soft reboot kernel panic
	//	goto handshake_fail;
	}

	info("probing done\n");

	return SUCCESS;

register_fail:
	kfree(item);
	item = NULL;
	kfree(work);
	work = NULL;
	unregister_chrdev(MAJOR(ipc_chdev), "ioc-ipc");
	unregister_chrdev_region(ipc_chdev, CHANNEL_DEV_MAX);

//handshake_fail:
	first_message = 0;
	for (i = 0; i < NUMBER_OF_CHANNELS; i++) {
		msgqueue_destroy(&queue[i].q_ch_rx);
		msgqueue_destroy(&queue[i].q_ch_tx);

		channel_info[i].rxflowctrlstate = IPC_XOFF;
	}
	kfree(item);
	item = NULL;
	kfree(work);
	work = NULL;
	if (!IS_ERR_OR_NULL(root.send_thread_id)) {
		send_sig(SIGINT, root.send_thread_id, 1);
		kthread_stop(root.send_thread_id);
	}
	class_destroy(dev_class);
	unregister_chrdev(MAJOR(ipc_chdev), "ioc-ipc-ch");
	unregister_chrdev_region(ipc_chdev, CHANNEL_DEV_MAX);
	return FAILURE;


}

static int spi_ipc_remove(struct spi_device *spi)
{
	int i;
	first_message = 0;
	for (i = 0; i < NUMBER_OF_CHANNELS; i++) {
		msgqueue_destroy(&queue[i].q_ch_rx);
		msgqueue_destroy(&queue[i].q_ch_tx);

		channel_info[i].rxflowctrlstate = IPC_XOFF;
	}
	kfree(work);
	work = NULL;
	if (!IS_ERR_OR_NULL(root.send_thread_id)) {
		send_sig(SIGINT, root.send_thread_id, 1);
		kthread_stop(root.send_thread_id);
	}
	class_destroy(dev_class);
	unregister_chrdev(MAJOR(ipc_chdev), "/ipc/ch");
	unregister_chrdev_region(ipc_chdev, CHANNEL_DEV_MAX);
	debug("Device unregistered\n");
	return SUCCESS;
}

static void flush_ipc_queue(void)
{
    int result = 0;
    ipc_message_t *msg;
    int i = 0;

    for (i = 0; i < NUMBER_OF_CHANNELS; i++) {
        while (msgqueue_used(&queue[i].q_ch_tx) > 0) {
            /* Pop the message from channel transmit queue*/
            result = msgqueue_dequeue(&queue[i].q_ch_tx,
                    (void **)&msg, NULL);
            if((result == EOK) && msg) {
                // Need to free the message
                msgqueue_free(&queue[i].q_ch_tx, msg);
            }
            else {
                error("flush_ipc_queue: transmit msgqueue_dequeue failed, result[ch_id=%d]=%d\n", i, result);
                break;
            }
        }
    }

    for (i = 0; i < NUMBER_OF_CHANNELS; i++) {
        while (msgqueue_used(&queue[i].q_ch_rx) > 0) {
            /* Pop the message from channel receive queue*/
            result = msgqueue_dequeue(&queue[i].q_ch_rx,
                    (void **)&msg, NULL);
            if((result == EOK) && msg) {
                // Need to free the message
                msgqueue_free(&queue[i].q_ch_rx, msg);
            }
            else {
                error("flush_ipc_queue: receive msgqueue_dequeue failed, result[ch_id=%d]=%d\n", i, result);
                break;
            }
        }
    }
}

/**
 * @brief This function checks if IPC is ready for runtime suspend
 * @retval 0 if going for runtime suspend, -EBUSY otherwise
 */
int spi_ipc_suspend(struct device *dev)
{
        printk(KERN_INFO "in spi ipc dynamic suspend, pm event\n");
        return(0);
}

/**
 * @brief resume callback for IPC
 * @retval 0
 */
int spi_ipc_resume(struct device *dev)
{
        unsigned char dummy[] = { DUMMY_DATA , DUMMY_DATA1};

        printk(KERN_INFO "in spi ipc resume callback\n");

        flush_ipc_queue();

        debug(" writng dummy data %lu\n", sizeof(dummy));
        spi_write(root.spidev, dummy, sizeof(dummy));
        debug(" written dummy byte---------------\n");

        timeout = msecs_to_jiffies(IOC_ACK_TIMEOUT);
        if (!wait_for_completion_timeout(&root.ack_done, timeout)) {
                error("dummy data ack timeout\n");
                return FAILURE;
        }

	first_message = 0;
        if (intialization_phase() < 0) {
                error("Error in intialization_phase()\n");
                //goto handshake_fail;
                return FAILURE;
        }
        return(0);
}
static const struct of_device_id spi_ipc_dt_ids[] = {
	{ .compatible = "ioc-ipc"},
	{ /* sentinel */ }
};


static const struct dev_pm_ops spi_ipc_pm_ops = {
	.suspend = spi_ipc_suspend,
	.resume =  spi_ipc_resume,
};

static struct spi_driver spi_ipc_driver = {
	.driver = {
		.name = "ioc-ipc",
		.bus  =  &spi_bus_type,
		.owner = THIS_MODULE,
//		.of_match_table = spi_ipc_dt_ids,
                .pm = &spi_ipc_pm_ops,
	},
	.id_table = ipc_ids,
	.probe = spi_ipc_probe,
	.remove = spi_ipc_remove,
};

static int __init ipc_init(void)
{
	int ret;

	info("IPC module installing\n");

	ret = spi_register_driver(&spi_ipc_driver);
	if (ret)
		pr_err("%s: problem at spi_register_driver\n", __func__);

	return ret;
}

static void __exit ipc_exit(void)
{
	debug("IPC module exiting\n");
	spi_unregister_driver(&spi_ipc_driver);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ioc-ipc");
MODULE_AUTHOR("Shrisha Krishna");
MODULE_VERSION("0.1");

module_init(ipc_init);
module_exit(ipc_exit);
