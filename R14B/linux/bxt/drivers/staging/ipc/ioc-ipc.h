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

#ifndef _IOCIPC_H
#define _IOCIPC_H

#include <linux/cdev.h>
#include "msgqueue.h"

#define IPC_MSG_LEN	255		/* Length of IPC messages */
#define IPC_VERSION	0x10		/* Currently IPC Version */
#define HEADER_SIZE	2		/* Header size */
#define RECEIVED_PAYLOAD_SIZE	255	/* Received payload size */
#define WR_OP		0xA2	/* Write Operation */
#define RD_OP		0xA1	/* Read Operation */
#define DMA_LENGTH	0x10	/* V850 DMA length */

#define IPC_FLOWCONTROL_CHANNEL	        0	/* Flow Control Channel No. */
#define IPC_ALIVE_CHANNEL		1      /*Used for Watchdog Heart Beat*/
#define IPC_RESERVED_CHANNEL0		2
#define IPC_RESERVED_CHANNEL1		3
#define IPC_SYSTEM_CHANNEL		4      /*Used for Reset,*/
#define IPC_NAVI_CHANNEL		5      /*Used for NAVI Messages*/
#define IPC_RESERVED_CHANNEL2		6      /*unused */
#define IPC_LOOPBACK_CHANNEL		7      /*Used for Look Back Channel*/
#define IPC_WATCHDOG_CHANNEL		8

#define MSG_WATCHDOG_IPCFRAME_STANDARD  11
#define MSG_WATCHDOG_IPCFRAME_REGISTER  22
#define MSG_WATCHDOG_IPCFRAME_RESET     33

#define FLOWCONTROL_MSG_OFFSET		3
#define DATA_CHANNEL_OFFSET		1

/* Max IPC channels = */
#define CHANNEL_DEV_MAX 		28
#define CHANNEL2_MINOR 			2
#define NUMBER_OF_CHANNELS		(CHANNEL2_MINOR + CHANNEL_DEV_MAX)

#define EOK 0
#define IPC_XON				1
#define IPC_XOFF			0
#define IPC_XON_THRESHOLD		16
#define IPC_XOFF_THRESHOLD		18
#define QUEUE_SIZE			20
#define IOC_ACK_TIMEOUT			100
#define IOC_IPC_RESET			0x11
#define CHANNEL_OPEN			1
#define CHANNEL_CLOSE			0

#define DUMMY_DATA      0x02
#define DUMMY_DATA1     0x01

#define SUCCESS 0
#define FAILURE -1

#define __malloc(size, allocpoint) kmalloc(size, GFP_KERNEL)
#define __free(ptr) kfree(ptr)

/* logging */
#define debug(fmt, arg...)                                          \
	pr_debug("[ IPC :%-20.20s: %-1.1d ] [DBG] " fmt "\n", \
	__func__, __LINE__, ##arg)
#define error(fmt, arg...)                                          \
	pr_err("[ IPC :%-20.20s: %-1.1d ] [ERR] " fmt "\n",   \
	__func__, __LINE__, ##arg)
#define info(fmt, arg...)                                           \
	pr_info("[ IPC :%-20.20s: %-1.1d ] [INF] " fmt "\n",  \
	__func__, __LINE__, ##arg)

/* check if we shd mk this to inline func, multiple use in same func will thrw
 * err */
static inline void *__calloc(unsigned int n, size_t size, unsigned int
			allocpoint)
{
	void *ptr = NULL;
	ptr = kmalloc(size*n, GFP_KERNEL);
	if (ptr != NULL)
		memset(ptr, 0, n*size);

	return ptr;
}

/* @brief Message Format */
typedef struct {
	uint8_t	command;	/* Command: master
				   read/write READ- 0xA1, WRITE - 0xA2 */
	uint8_t	length_dma;	/* Length to configure V850 DMA */
	uint8_t	channel_no;	/*	Channel Number */
	uint8_t	buffer[IPC_MSG_LEN];	/*	Application Payload,
						it should be <= 255 */
	uint8_t	ipc_version;	/*	IPC Version, Currently it is 0x10*/
	uint8_t	no_of_channels;	/*	Number of IPC channels */
	uint8_t	no_of_clients;	/*	Number of Clients per channel */
	uint8_t	state;	/*	Tx XON/XOFF state 0 = XOFF, 1 = XON */
	uint8_t	buffer_len;	/*	Length of the data */
	uint8_t	rxflowctrlstate;
	uint8_t nfreebuffers;
	uint8_t channel_in_use; /*to avoid opening same channel twice*/
	uint8_t msg_queue_count;
	uint8_t poll_flag;
	uint8_t debug_on;
	uint32_t rx_count;
	uint32_t tx_count;
} ipc_message_t;

/* @brief Message to be transmit */
typedef struct {
	unsigned char	header[HEADER_SIZE];	/* Header */
	unsigned char	data[IPC_MSG_LEN];	/* Data */
} tx_message_t;

/* @brief Message to be receive */
typedef struct {
	unsigned char	header[HEADER_SIZE];	/* Header */
	unsigned char	payload[RECEIVED_PAYLOAD_SIZE];	/* Data */
} rx_message_t;

/* @brief This structure is used to fill
the received message from V850 after parsing.
*/
typedef struct {
	uint8_t	command;	/*Command - master read/write
				READ - 0xA1, WRITE - 0xA2 */
	uint8_t	channel_no;	/*	Channel Number */
	unsigned char	buffer[IPC_MSG_LEN];	/* Application Payload,
						   it should be <= 255 */
	uint8_t	ipc_version;	/* IPC Version, Currently it is 0x10*/
	uint8_t	no_of_channels;	/*Number of IPC channels */
	uint8_t	no_of_clients;	/* Number of Clients per channel */
	uint8_t	state;	/*Tx XON/XOFF state 0 = XOFF, 1 = XON */
	int	buffer_size;	/*Length of the data */
} ipc_parse_message_t;

/* @brief Message queues for rx and tx */
typedef struct {
	msgqueue_t	q_ch_rx;	/* Receive queue for channels */
	msgqueue_t	q_ch_tx;	/* Transmit queue for cannels */
} ipc_queue_t;

/* @brief Main data management structure */
typedef struct ipc_plugin_s {
	struct task_struct	*send_thread_id;	/*ID of send thread */
	struct device		*dev;
	struct spi_device	*spidev;
	struct completion	msg_ready;	/* to notify sender thread */
	struct completion	ack_done;
	struct completion	irq_recv;
	struct completion	read_msg;
	unsigned int		channelno;
	wait_queue_head_t 	readable, writeable;

	struct cdev         cdev;			/**< cdev structure */
} ipc_plugin_t;

typedef struct {
	struct work_struct recv_work;
	int x;
} recv_work_t;

/* Format of the HEARTBEAT, DEREGISTER, PAUSE from the Watchdog to the V850 */
typedef struct _watchdog_ipcframe_standard {
	uint8_t messagetype;   /* one of the WATCHDOG2V850 defines above */
} watchdog_ipcframe_standard;

/* Format of the REGISTER Message from the Watchdog to the V850 */
typedef struct _watchdog_ipcframe_register {
	uint8_t     messagetype;           /* messagetype */
	uint8_t     interfaceversion;     /* WATCHDOG2V850_INTERFACE_VERSION */
	uint16_t    padding;		/* Needed ofr alignment*/
	uint32_t    heartbeatinterval;    /* heartbeat interval in msec */
} watchdog_ipcframe_register;

/* Format of the RESET Message from the Watchdog to the V850 */
typedef struct _watchdog_ipcframe_reset {
	uint8_t     messagetype;
	uint8_t     interfaceversion;     /* WATCHDOG2V850_INTERFACE_VERSION */
	uint8_t     rootcause;         /* reason of the reset */
	uint8_t     additionalinfo;    /* bitcoded value - see defines above */
	uint32_t    pid;               /* if client fails, its PID */
	uint32_t    tid;               /* if client fails, its TID */
	uint32_t    timestamp;     /* timestamp (monotonic clock) (in secs) */
	uint8_t    probename[16];  /* name of failing client/plugin */
} watchdog_ipcframe_reset;

typedef struct ipc_watchdog_s {
	watchdog_ipcframe_standard s_msg;
	watchdog_ipcframe_register r_msg;
	watchdog_ipcframe_reset reset_msg;
	uint8_t wbuf_len;
} ipc_watchdog_t;

/* Functions used by IPC and watchdog*/
int channel_open(int channel_number);
int channel_close(int channel_number);
int system_channel_write(char *buf, size_t size);
int system_channel_close(void);
int read_navi_data(unsigned char *buf);
int close_navi_channel(void);

#endif
