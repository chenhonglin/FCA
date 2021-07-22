/*****************************************************************************
 * Project     		Harman Car Multimedia System
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	ipc_handler.c
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * IPC protocol handler implementation.
 * This module contains the protocol implementation on the IPC layer.
 */


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "types.h"
#include "ipc.h"
#include "hdlc.h"
#include "log.h"
#include "ipcProtocol.h"
#include "ipcDev.h"
#include "flowControl.h"
#include "handler.h"
#include "hdlc.h"
#include "statistics.h"

/*****************************************************************************
 * STRUCTURES
 *****************************************************************************/
typedef struct TPriorityEntryS
{
	TDeviceAttr*            pDevice;
	struct TPriorityEntryS* pNext;
}TPriorityEntry;

typedef struct TPriorityLevelS
{
	TPriorityEntry*         pCurrent;
	TPriorityEntry*         pFirst;
	struct TPriorityLevelS* pNext;
}TPriorityLevel;

/*****************************************************************************
 * VARIABLES
 *****************************************************************************/
static UInt32 		nPendingTxMsgs = 0;
static UInt32 		nPendingFlowCtrlMsgs = 0;
static TPriorityLevel* 	pPriorityRootLevel = NULL;
static DEFINE_SPINLOCK(txMutexHandle);

//#define FLOW_CONTROL

/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/
static Int32  	handlerGetTransmitBuffer(UInt8 *bufPtr, UInt32 *dataLen);
#if	0
static Int32  	handlerGetTransmitBuffer1(UInt8 *bufPtr, UInt32 *dataLen);
#endif
static void  	handlerFreePriorityLists(void);

/*****************************************************************************
 * FUNCTION DEFINITIONS
 *****************************************************************************/
/**
 * This function signals the transmission thread that new messages
 * are available.
 *
 * @param  none
 * @return none
 */
void handlerTriggerTransmission(UInt8 channel)
{
	TDeviceAttr* pDeviceAttr;

	if (IPC_FLOWCTRL_CHANNEL == channel) {
#ifdef	FLOW_CONTROL
		spin_lock(&txMutexHandle);
		if (0 == nPendingFlowCtrlMsgs) {
			nPendingFlowCtrlMsgs = 1;
			spin_unlock(&txMutexHandle);

			/* Signal the HDLC Tx thread */
			setEventDataAvailable();
		}
		else
			spin_unlock(&txMutexHandle);

		log_info("Trigger event for flowcontrol message: %d",nPendingFlowCtrlMsgs);
		//mdelay(50);
#endif
	}
	else {
		pDeviceAttr = getDeviceAttrPtr(channel);
		spin_lock(&txMutexHandle);
		nPendingTxMsgs++;
		spin_unlock(&txMutexHandle);
#ifdef	FLOW_CONTROL
		/* HDLC API to signal the TX thread */
		if(IPC_XON == flowCtrlGetTxState(pDeviceAttr)) {
			log_info("Trigger event for Tx message, XON: nPendingTxMsgs = %d",nPendingTxMsgs);
#else
			log_info("Trigger event for Tx message, nPendingTxMsgs = %d",nPendingTxMsgs);
#endif
			/* Signal the HDLC Tx thread */
			setEventDataAvailable();
#ifdef	FLOW_CONTROL
		}
		else {
			log_info("Trigger event for Tx message XOFF: nPendingTxMsgs = %d",nPendingTxMsgs);
		}
#endif
	}
	return;
}

/**
 * This function signals the transmission thread that new messages
 * are available.
 *
 * @param  none
 * @return none
 */
UInt8 dataAvailable(void)
{
	TPriorityLevel* pLevel = NULL;
	UInt8 		uMsgSent = FALSE;

	spin_lock(&txMutexHandle);
#ifdef	FLOW_CONTROL
	if (nPendingFlowCtrlMsgs > 0) {
		spin_unlock(&txMutexHandle);
		uMsgSent = TRUE;
	} else if(nPendingTxMsgs > 0){

		spin_unlock(&txMutexHandle);
		pLevel = pPriorityRootLevel;
		while ((NULL != pLevel) && (FALSE == uMsgSent))
		{
			TPriorityEntry* pEntry = pLevel->pCurrent;
			do {
				TDeviceAttr* pDeviceAttr = pEntry->pDevice;
				TWriteMsg* pWriteMsg;

				/* Check if data is available in the queue & the flowcontrol state is XON */
				pWriteMsg = (TWriteMsg*)queueGetReadPtr(pDeviceAttr->txQueue);
				if (NULL != pWriteMsg) {
					if (IPC_XON == flowCtrlGetTxState(pDeviceAttr) && pDeviceAttr->channelID != IPC_WATCHDOG_CHANNEL) {
						uMsgSent = TRUE;		/* store that we've sent a message */
						pLevel->pCurrent = pEntry;	/* set current entry pointer to this entry */
						break;
					}
				}

				pEntry = pEntry->pNext;
			}while (pLevel->pCurrent != pEntry);

			pLevel = pLevel->pNext;
		}
	}
	else {
		spin_unlock (&txMutexHandle);
	}
#else
	if(nPendingTxMsgs > 0){
		uMsgSent = TRUE;
	}
	spin_unlock (&txMutexHandle);
#endif

	return uMsgSent;
}

/**
 * This function provides the highest priority data to the TX thread
 *
 * @param  bufPtr  - Buffer pointer to fill the data
 * @param  dataLen - Length of data filled in the buffer
 * @return SUCCESS/FAILURE
 */
Int32 getIpcTxBuffer(UInt8 *bufPtr, UInt32 *dataLen)
{
	TDeviceAttr* pFctrlDeviceAttr;
	Int32 iRet = ERROR;

	log_info( "%s(): nPendingTxMsgs = %d nPendingFlowCtrlMsgs = %d", __FUNCTION__, nPendingTxMsgs, nPendingFlowCtrlMsgs);

	spin_lock (&txMutexHandle);
	if((nPendingFlowCtrlMsgs > 0))
	{
		nPendingFlowCtrlMsgs = 0;
		spin_unlock (&txMutexHandle);
		log_info("i%s(): Received Tx Event for Flow Control Message ", __FUNCTION__);
		iRet = flowCtrlWriteMessage(bufPtr, dataLen);
		pFctrlDeviceAttr = getDeviceAttrPtr(IPC_FLOWCTRL_CHANNEL);
		STATISTICS_INC(pFctrlDeviceAttr->statistics.txMsgCnt);
	}
	else if((nPendingTxMsgs > 0)) {
		spin_unlock (&txMutexHandle);
		log_info( "%s(): Received Tx Event ***", __FUNCTION__);
		iRet = handlerGetTransmitBuffer(bufPtr, dataLen);
	}
	else {
		spin_unlock (&txMutexHandle);
		log_debug("No Pending IPC messages Return ERROR");
	}
	return iRet;
}

/**
 * This function provides the highest priority user messages
 * to the TX thread
 *
 * @param  pBuffer  - Buffer pointer to fill the data
 * @param  pDataLen - Length pointer to fill the data Length
 * @return SUCCESS/FAILURE
 */
static Int32 handlerGetTransmitBuffer(UInt8 *pBuffer, UInt32 *pDataLen)
{
	TPriorityLevel* pLevel;
	UInt8 		uMsgSent = FALSE;
	Int32		iRet=EOK;

	pLevel = pPriorityRootLevel;

	while ((NULL != pLevel) && (FALSE == uMsgSent)) {

		TPriorityEntry* pEntry = pLevel->pCurrent;
		do {
			TDeviceAttr* pDeviceAttr = pEntry->pDevice;
			TWriteMsg* pWriteMsg;

			pWriteMsg = (TWriteMsg*)queueGetReadPtr(pDeviceAttr->txQueue);
			if (NULL != pWriteMsg) {
				/* get read pointer of tx queue and transmit the message
				 * to the other CPU */
				if (IPC_XON == flowCtrlGetTxState(pDeviceAttr) && pDeviceAttr->channelID != IPC_WATCHDOG_CHANNEL) {
					log_info( "%s(): Send Msg for CH = %d, Msglen = %d", __FUNCTION__, pDeviceAttr->channelID, pWriteMsg->frLen);

					memcpy(pBuffer, (UInt8*)pWriteMsg, (IPC_HEADER_SIZE + pWriteMsg->frLen));
					*pDataLen =  IPC_HEADER_SIZE + pWriteMsg->frLen;

					//STATISTICS_SETMAX(channel[pWriteMsg->msg.channel].maxTxMsgSize, pWriteMsg->msg.frLen);
					STATISTICS_ADD(pDeviceAttr->statistics.txNBytes, pWriteMsg->frLen);

					uMsgSent = TRUE;                   /* store that we've sent a message */
					pLevel->pCurrent = pEntry->pNext; /* set current entry pointer to this entry */

					/* Remove the message from the queue and notify any waiting thread */
					spin_lock(&txMutexHandle);
					nPendingTxMsgs--;               /* decrement counter for pending tx messages */
					spin_unlock(&txMutexHandle);
					log_info( "nPendingTxMsgs = %d ",nPendingTxMsgs);

					queueRemoveData(pDeviceAttr->txQueue);
					wake_up_interruptible(&pDeviceAttr->txWq);

					/* If queue is empty and no clients are open for the channel, trigger flowcontrol message */
					if((pDeviceAttr->clients == 0) && queueIsEmpty(pDeviceAttr->txQueue))
						handlerTriggerTransmission(IPC_FLOWCTRL_CHANNEL);

					STATISTICS_INC(pDeviceAttr->statistics.txMsgCnt);
					break;
				}
				else {
					log_info("Channel %d is in XOFF State: flowCtrlGetTxState(pDeviceAttr) = %d ", pDeviceAttr->channelID, flowCtrlGetTxState(pDeviceAttr));
				}
			}
			pEntry = pEntry->pNext;
		}while (pLevel->pCurrent != pEntry);

		pLevel = pLevel->pNext;
	}

	if(uMsgSent == FALSE)
		iRet = ERROR;

	return iRet;
}

/**
 * This function process the IPC message from the HDLC layer
 *
 * @param  bufPtr  - Pointer to the data
 * @param  dataLen - Length of data filled in the buffer
 * @return no
 */
Int32 handlerProcessReceiveEvent(UInt8 *bufPtr, UInt32 dataLen)
{
	Int32		iRet = EOK;
	ipc_msghdr_t 	*pIpcHeader = NULL;
	TDeviceAttr	*pDevice = NULL;

	if(!bufPtr) {
		log_error("Invalid Input Parameter. Null Pointer");
		return ERROR;
	}

	if(dataLen > (IPC_DATA_SIZE + IPC_HEADER_SIZE)) {
		log_error("Invalid Data Length '%d', Ignore the Packet", dataLen);
		return ERROR;
	}

	/* read header data */
	pIpcHeader = (ipc_msghdr_t *)bufPtr;

	if(pIpcHeader->frLen > IPC_DATA_SIZE) {
		log_error("Normal packet: Invalid IPC frame Length %d, Ignore the packet",pIpcHeader->frLen);
		return ERROR;
	}

	/* Validate the Length from HDLC/IPC layer */
	if((pIpcHeader->frLen + IPC_HEADER_SIZE) != dataLen){
		log_warning("Packet length mismatch: IPC layer - '%d' Hdlc layer - '%d', Ignore the Packet", pIpcHeader->frLen, dataLen);
		log_notice("dev-ipc: Msg [RX, ch: %02u, len: %03u]: Invalid length !!!", pIpcHeader->channel, pIpcHeader->frLen);
		//return ERROR;
	}

	switch (pIpcHeader->channel) {
		/* flow control message received */
		case IPC_FLOWCTRL_CHANNEL:
			pDevice = getDeviceAttrPtr(pIpcHeader->channel);
			log_info("Flowctrl Msg [RX, ch: %02u, len: %03u]:", IPC_FLOWCTRL_CHANNEL, pIpcHeader->frLen);
			iRet = flowCtrlReadMessage(&bufPtr[IPC_HEADER_SIZE], (dataLen - IPC_HEADER_SIZE) );
			STATISTICS_INC(pDevice->statistics.rxMsgCnt);
			log_info("iret = %d",iRet);
			if(iRet > 0)
				return EOK;
			else if(iRet < 0)
				return ERROR;
			break;

			/* watchdog message received */
		case IPC_WATCHDOG_CHANNEL:
			//STATISTICS_INC(channel[IPC_WATCHDOG_CHANNEL].rxMsgCnt);
			log_info("Msg [RX, ch: %02u, len: %03u]:", IPC_WATCHDOG_CHANNEL, 0);
			break;

			/* normal channel message received */
		default:
			/* get the device pointer for the given channel */
			pDevice = getDeviceAttrPtr(pIpcHeader->channel);
			if (NULL != pDevice) {

				UInt32 continueRead = 0;
				TReadMsg* pReadMsg;

				/* try to read data as long as a free queue entry is available */
				for(;;) {

					pReadMsg = (TReadMsg*)queueGetWritePtr(pDevice->rxQueue);
					if (NULL != pReadMsg) {
						pReadMsg->channel = pIpcHeader->channel;
						pReadMsg->frame_type = pIpcHeader->frame_type;
						pReadMsg->frLen = pIpcHeader->frLen;

						if (pReadMsg->frLen > IPC_DATA_SIZE)
							memcpy(&pReadMsg->data[0], &bufPtr[4], IPC_DATA_SIZE);
						else
							memcpy(&pReadMsg->data[0], &bufPtr[4], pReadMsg->frLen);

						STATISTICS_INC(pDevice->statistics.rxMsgCnt);
						//STATISTICS_SETMAX(channel[pReadMsg->msg.channel].maxRxMsgSize, pReadMsg->msg.frLen);
						STATISTICS_ADD(pDevice->statistics.rxNBytes, pReadMsg->frLen);

						queueAddData(pDevice->rxQueue);        /* add data to queue */

						/* Notify RX thread blocked on RXQ Data Availability */
						wake_up_interruptible(&pDevice->rxWq);
						continueRead = 0;
					}
					else {
						continueRead++;
						log_notice( "%s(): NO Free Space in RXQ.. Wait for some time...", __func__);
					}
					/* check if it is necessary to send an XOFF command */
					flowCtrlCheckRxState(pIpcHeader->channel);
					/* if the queue is full, it is necessary to continue
					 * the read process until a free queue entry is available
					 * (this should never occur) */
					if (0 == continueRead)
						break;
					else if(continueRead >= 3) {
						log_warning( "%s(): RXQ-%d is full, Ignore the message", __func__, pDevice->channelID);
						break;
					}
				}
			}
			else {
				log_warning("dev-ipc: Message for unknown channel-%d received", pIpcHeader->channel);
				return ERROR;
			}
			break;
	}
	return iRet;
}


/**
 * This function sorts the all transmit channels by their priority.
 *
 * To do this, a structure is filled with the
 * channel number and the appropriate priority. This structure elements
 * are given to the quick sort algorithm which sorts them from the lowest
 * up to the highest priority.
 *
 * @param  none
 * @return 0 on success, ERROR on failure
 */
Int32 handlerCreatePriorityLists(IPCAttr_t *pIpcRoot)
{
	TDeviceAttr** 	sortedDevices;
	TDeviceAttr* 	pDeviceAttr;
	UInt32 		i, j, maximal;
	UInt8 		priority = 0;
	TPriorityLevel* pLevel = NULL;
	TPriorityEntry* pEntry = NULL;
	UInt32 		nSortedDevices = 0;
	UInt8 		ch_count = pIpcRoot->uNoChannels;

	sortedDevices = kmalloc(ch_count * sizeof(TDeviceAttr*), GFP_KERNEL);
	if (NULL != sortedDevices)
	{
		/* sort channels */
		for (i = 0; i < (ch_count + IPC_FREE_CHANNEL); i++) {
			pDeviceAttr = getDeviceAttribute(i);
			if (NULL != pDeviceAttr) {
				if((pDeviceAttr->channelID == IPC_FLOWCTRL_CHANNEL) || (pDeviceAttr->channelID == IPC_WATCHDOG_CHANNEL))
					continue;
				sortedDevices[nSortedDevices] = pDeviceAttr;
				nSortedDevices++;
			}
		}
		log_info( "%s(): nSortedDevices = %d", __FUNCTION__, nSortedDevices);

		if (1 < nSortedDevices) {
			for (i = 0; i < (nSortedDevices); i++) {
				maximal = i;
				for (j = i+1; j < nSortedDevices; j++) {
					if (sortedDevices[j]->priority < sortedDevices[maximal]->priority)
						maximal = j;
				}
				pDeviceAttr = sortedDevices[maximal];
				sortedDevices[maximal] = sortedDevices[i];
				sortedDevices[i] = pDeviceAttr;
			}
		}

		log_info("handlerCreatePriorityLists ");
		/* create linked list */
		for (i = 0; i < nSortedDevices ; i++) {
			pEntry = (TPriorityEntry*) kmalloc(sizeof(TPriorityEntry), GFP_KERNEL);
			if (NULL != pEntry) {
				pEntry->pDevice = sortedDevices[i];
				pEntry->pNext = NULL;
				log_info( "SortedDevices = %d ch = %d",i,sortedDevices[i]->channelID);
				log_info( "old priority = %d current priortiy = %d",priority,sortedDevices[i]->priority);

				if ((NULL == pLevel) || (sortedDevices[i]->priority != priority)) {
					TPriorityLevel* pNewLevel = (TPriorityLevel*)kmalloc(sizeof(TPriorityLevel), GFP_KERNEL);
					if (NULL != pNewLevel) {
						priority = sortedDevices[i]->priority;
						if (NULL == pLevel)
							pLevel = pPriorityRootLevel = pNewLevel;
						else {
							pLevel->pCurrent->pNext = pLevel->pFirst;
							pLevel->pCurrent = pLevel->pFirst;
							pLevel->pNext = pNewLevel;
							pLevel = pNewLevel;
						}
						pLevel->pFirst = pLevel->pCurrent = pEntry;
						pLevel->pNext = NULL;
					}
					else {
						log_error("Can't allocate priority level");
						kfree(pEntry);
						handlerFreePriorityLists();
						pPriorityRootLevel = NULL;
						return -ENOMEM;
					}
				}
				else {
					pLevel->pCurrent->pNext = pEntry;
					pLevel->pCurrent = pEntry;
				}
			}
			else {
				log_error("Can't allocate priority list entry");
				handlerFreePriorityLists();
				pPriorityRootLevel = NULL;
				return -ENOMEM;
			}
		}
		if (NULL != pLevel)
		{
			pLevel->pCurrent->pNext = pLevel->pFirst;
			pLevel->pCurrent = pLevel->pFirst;
		}
	}
	log_info("handlerCreatePriorityLists Done");
	return EOK;
}


/**
 * This function frees all priority list entries.
 *
 * @param  none
 * @return none
 */
static void handlerFreePriorityLists(void)
{
	TPriorityLevel* pLevel = pPriorityRootLevel;

	/* free linked lists */
	while (NULL != pLevel)
	{
		TPriorityLevel* pLevelToFree = pLevel;
		TPriorityEntry* pEntry = pLevel->pCurrent;
		TPriorityEntry* pEntryToFree = NULL;

		do
		{
			if(NULL == pEntry)
				break;

			pEntryToFree = pEntry;
			pEntry = pEntry->pNext;
			kfree(pEntryToFree);
		}
		while (pLevel->pFirst != pEntry);

		pLevel = pLevel->pNext;
		kfree(pLevelToFree);
	}
}


/**
 * This function cleans all handler resources.
 *
 * @param  none
 * @return none
 */
void handlerCleanup(void)
{
	handlerFreePriorityLists();          /* free all priority list entries */
	return;
}
