/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	ipc.h
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * This header file defines all the data structures used to
 * handle the IPC and the attributes for the channel devices
 */

#ifndef IPC_H
#define IPC_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include <linux/cdev.h>
#include <linux/interrupt.h>

#include "queue.h"
#include "ipcProtocol.h"

#define MAX_SOCK_NAME_SIZE		128
#define MAX_STRING_SIZE			128

#define IPC_TEST_MODE          	0x1


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * STRUCTURES
 *****************************************************************************/

#pragma pack(push, 1)
typedef struct SockIPCMsg
{
	UInt16 		pktLen;
	UInt8		pktBuf[IPC_BUFFER_SIZE];
}SockIPCMsg_t;
#pragma pack(pop)

typedef struct
{
	UInt32 maxTxMsgSize;
	UInt32 maxRxMsgSize;
	UInt32 txXoffCnt;
	UInt32 txXonCnt;
	UInt32 txMsgCnt;
	UInt32 txLastMsgCnt;
	UInt32 txNBytes;
	UInt32 rxXoffCnt;
	UInt32 rxXonCnt;
	UInt32 rxMsgCnt;
	UInt32 rxLastMsgCnt;
	UInt32 rxNBytes;
}TChannelStatistics;

/**
 * Device attributes structure.
 */
typedef struct TDeviceAttr_s
{
	UInt8                   channelID;       	/* channel identifier */
	UInt8                   priority;       	/* channel priority */

	UInt8                	clients;       		/* Number of clients opened this channel */
	UInt8                	prevClients;       		/* Number of clients opened this channel */
	UInt8                   rxFlowCtrlState; 	/* flow control state of the rx section */
	UInt8                   txFlowCtrlState; 	/* flow control state of the tx section */

	struct mutex 		txFlowCtrlMutex;
	struct mutex 		rxFlowCtrlMutex;

	UInt32			flags;
	struct mutex		accessMutex;		/* locks the access to all device attributes */

	TQueueHandle            rxQueue;         	/* rx message queue handle */
	TQueueHandle            txQueue;         	/* tx message queue handle */

	wait_queue_head_t 	rxWq;
	wait_queue_head_t 	txWq;
	wait_queue_head_t 	pollWq;

	Int32			rc_mreq;

	struct cdev           	cdev;
	spinlock_t		ch_lock;
	UInt8			ch_open_count;

	TChannelStatistics	statistics;
}TDeviceAttr;
#define IOFUNC_ATTR_T struct TDeviceAttr_s

typedef struct IPCAttr_s
{
	TDeviceAttr		*deviceAttr;	/*<* Array of IPC channel devices, the last is reserved as error channel */
	/* ToDo: When mltiple ipc_i2c, push this to TDeviceAttr, since, it will depend on the channel */
	struct i2c_client	*client;
	dev_t			ipc_chdev;
	struct class 		*dev_class;
	UInt32			uNoChannels;

}IPCAttr_t;


#endif /* IPC_H */
