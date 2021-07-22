/*****************************************************************************
 * Project     		Harman Car Multimedia System
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	flowControl.c
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * This module contains the flow control implementation for the IPC layer.
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>

#include "types.h"
#include "log.h"
#include "ipcProtocol.h"
#include "ipcDev.h"
#include "flowControl.h"
#include "handler.h"
#include "hdlc.h"
#include "statistics.h"
#include "ipc.h"
#include "options.h"
/*****************************************************************************
 * VARIABLES
 ****************************************************************************/
static UInt8	sflowCtrlIsEnabled;
static UInt32	zTxFlowCtrlState;
static UInt32	zRxFlowCtrlState;
static UInt32	zChannelMask;

extern UInt32 	gNoChannels;

DEFINE_MUTEX(gFlowCtrlMutex);
/*****************************************************************************
 * FUNCTION DEFINITIONS
 ****************************************************************************/

/**
 * This function updates the Rx flowcontrol mask for the given channel.
 *
 * @param  uState  - XON/XOFF
 * @param  channel - channel ID
 *
 * @return none
 */
static void flowCtrlUpdateRxFlowControlMask(UInt8 uState, UInt8 uChannel)
{
	mutex_lock(&gFlowCtrlMutex);
	if(uState)
	{
		zRxFlowCtrlState |= (0x1 << uChannel);
	}
	else
	{
		zRxFlowCtrlState &= ~(0x1 << uChannel);
	}
	mutex_unlock(&gFlowCtrlMutex);
	return;
}

/**
 * This function updates the Tx flowcontrol mask for the given channel.
 *
 * @param  uState  - XON/XOFF
 * @param  channel - channel ID
 *
 * @return none
 */
static void flowCtrlUpdateTxFlowControlMask(UInt8 uState, UInt8 uChannel)
{
	if(uState)
	{
		zTxFlowCtrlState |= (0x1 << uChannel);
	}
	else
	{
		zTxFlowCtrlState &= ~(0x1 << uChannel);
	}
	return;
}

/**
 * This function updates the Tx flowcontrol mask for all channels.
 *
 * @param  qFcMask - Flow control mask
 * @return none
 */
static void flowCtrlMaskUpdate(UInt32 qFcMask)
{
	TDeviceAttr* 	pDeviceAttr = NULL;
	UInt8 uChannel, updateState;

	for(uChannel = 2; uChannel < (gNoChannels+IPC_FREE_CHANNEL); uChannel++)
	{
		pDeviceAttr = getDeviceAttrPtr(uChannel);
		if(pDeviceAttr)
		{
			mutex_lock(&pDeviceAttr->txFlowCtrlMutex);
			if(TRUE == sflowCtrlIsEnabled)
			{
				updateState = (qFcMask & (1 << uChannel)) ? IPC_XON:IPC_XOFF;
				if (pDeviceAttr->txFlowCtrlState != updateState)
				{
					if (IPC_XON == updateState)
					{
						log_notice( "Sframe: XON for CH-%d recvd", uChannel);
					}
					else
					{
						log_notice( "Sframe: XOFF for CH-%d recvd", uChannel);
					}

					pDeviceAttr->txFlowCtrlState = updateState;
				}
				mutex_unlock(&pDeviceAttr->txFlowCtrlMutex);
			}
			else
			{
				mutex_unlock(&pDeviceAttr->txFlowCtrlMutex);
				break;
			}
		}
	}

	return;
}


/**
 * This function updates the Tx flowcontrol mask for all channels.
 *
 * @param  qMaskUpdate - Flow control mask
 * @return none
 */
void flowCtrlUpdateTxFlowControlStatus(UInt32 qMaskUpdate)
{
	UInt32	qNewMask;

	qNewMask = qMaskUpdate & zChannelMask;
	if(zTxFlowCtrlState != qNewMask)
	{
		log_notice( "FlowCtrl: Tx Flow Control Status Update: 0x%x -> 0x%x", zTxFlowCtrlState, qNewMask);
		zTxFlowCtrlState = qNewMask;
		flowCtrlMaskUpdate(qNewMask);
	}

	return;
}


/**
 * This function gets the Rx flowcontrol mask.
 *
 * @param  none
 * @return Rx flow control state of all channels
 */
UInt32 flowCtrlGetRxFlowControlMask(void)
{
	return zRxFlowCtrlState;
}


/**
 * This function enables the flow control mechanism.
 *
 * @param  none
 * @return none
 */
void flowCtrlEnable(void)
{
	TDeviceAttr* 	pDeviceAttr;
	UInt8		uChannelID;

	for (uChannelID = 2; uChannelID < (gNoChannels+IPC_FREE_CHANNEL); uChannelID++)
	{
		pDeviceAttr = getDeviceAttribute(uChannelID);
		if (NULL != pDeviceAttr)
		{
			pDeviceAttr->rxFlowCtrlState = IPC_XON;
#ifdef	FLOWCONTROL_OFF
			pDeviceAttr->txFlowCtrlState = IPC_XOFF;
#else
			pDeviceAttr->txFlowCtrlState = IPC_XON;
#endif
		}

		zChannelMask = zChannelMask | (1 << uChannelID);
	}

	zRxFlowCtrlState = 0xFFFFFFFF;
#ifdef	FLOWCONTROL_OFF
	zTxFlowCtrlState = 0x0;
#else
	zTxFlowCtrlState = 0xFFFFFFFF;
#endif

	pDeviceAttr = getDeviceAttribute(IPC_FLOWCTRL_CHANNEL);
	if (NULL != pDeviceAttr)
	{
		pDeviceAttr->rxFlowCtrlState = IPC_XON;
		pDeviceAttr->txFlowCtrlState = IPC_XON;
	}

	sflowCtrlIsEnabled = TRUE;
	return;
}


/**
 * This function disables the flow control mechanism.
 *
 * @param  none
 *
 * @return none
 */
void flowCtrlDisable(void)
{
	TDeviceAttr* 	pDeviceAttr;
	UInt8		uChannelID;

	sflowCtrlIsEnabled = FALSE;

	for (uChannelID = 0; uChannelID < (gNoChannels+IPC_FREE_CHANNEL); uChannelID++)
	{
		pDeviceAttr = getDeviceAttribute(uChannelID);
		if (NULL != pDeviceAttr)
		{
			mutex_lock(&pDeviceAttr->txFlowCtrlMutex);
			if (IPC_FLOWCTRL_CHANNEL == uChannelID)
			{
				pDeviceAttr->txFlowCtrlState = IPC_XON;
			}
			else
			{
				pDeviceAttr->txFlowCtrlState = IPC_XOFF;
			}
			mutex_unlock(&pDeviceAttr->txFlowCtrlMutex);

			mutex_lock(&pDeviceAttr->rxFlowCtrlMutex);
			if (IPC_FLOWCTRL_CHANNEL == uChannelID)
			{
				pDeviceAttr->rxFlowCtrlState = IPC_XOFF;
			}
			else
			{
				pDeviceAttr->rxFlowCtrlState = IPC_XON;
			}
			mutex_unlock(&pDeviceAttr->rxFlowCtrlMutex);
		}
	}

	mutex_lock(&gFlowCtrlMutex);
	zRxFlowCtrlState = 0xFFFFFFFF;
	zTxFlowCtrlState = 0x0;
	mutex_unlock(&gFlowCtrlMutex);
	return;
}


/**
 * This function returns the current tx state.
 *
 * @param  pDeviceAttr - device attribute pointer
 * @return tx flow control state
 */
UInt32 flowCtrlGetTxState(TDeviceAttr* pDeviceAttr)
{
	UInt32 state = IPC_XOFF;

	if (NULL != pDeviceAttr)
	{
		mutex_lock(&pDeviceAttr->txFlowCtrlMutex);
		state = pDeviceAttr->txFlowCtrlState;
		mutex_unlock(&pDeviceAttr->txFlowCtrlMutex);
	}

	return state;
}


/**
 * This function checks the rx state of the given channel.
 *
 * @param  channel - channel ID
 * @return none
 */
void flowCtrlCheckRxState(UInt8 channel)
{
	TDeviceAttr* 	pDeviceAttr;
	UInt8 		triggerTransmission = FALSE;

	/* get the device pointer for the channel */
	pDeviceAttr = getDeviceAttrPtr(channel);
	if (NULL != pDeviceAttr)
	{
		mutex_lock(&pDeviceAttr->rxFlowCtrlMutex);
		if ((TRUE == sflowCtrlIsEnabled) && (IPC_FREE_CHANNEL <= channel))
		{
			if (IPC_XON == pDeviceAttr->rxFlowCtrlState)
			{
				if (IPC_XOFF_THRESHOLD >= queueGetFreeElements(pDeviceAttr->rxQueue))
				{
					STATISTICS_INC(pDeviceAttr->statistics.rxXoffCnt);
					pDeviceAttr->rxFlowCtrlState = IPC_XOFF;
					triggerTransmission = TRUE;
					log_notice("CH-%d: goes to RX-XOFF state", channel);
				}
			}
			else
			{
				if (IPC_XON_THRESHOLD <= queueGetFreeElements(pDeviceAttr->rxQueue))
				{
					STATISTICS_INC(pDeviceAttr->statistics.rxXonCnt);
					pDeviceAttr->rxFlowCtrlState = IPC_XON;
					triggerTransmission = TRUE;
					log_notice("CH-%d: goes to RX-XON state", channel);
				}
			}
		}
		mutex_unlock(&pDeviceAttr->rxFlowCtrlMutex);

		if (TRUE == triggerTransmission)
		{
			log_info("%s: Send Flow Control Message", __FUNCTION__);
			handlerTriggerTransmission(IPC_FLOWCTRL_CHANNEL);
			flowCtrlUpdateRxFlowControlMask(pDeviceAttr->rxFlowCtrlState, channel);
		}
	}
	return;
}


/**
 * This function reads and parses the flowcontrol message for all channels.
 *
 * @param  bufPtr  - pointer to data
 * @param  dataLen - length of flowcontrol message
 *
 * @return 0 on success, -1 on failure
 */
Int32 flowCtrlReadMessage(UInt8 *bufPtr, UInt32 dataLen)
{
	UInt8 		uVersion, uNChannels;
	Int8		iRet = EOK;
	UInt8 		uSetEvent=FALSE;
	TDeviceAttr	*pDeviceAttr;
	UInt8 		*pFlowCtrlMsg;
	UInt8           uChannelID,uFlowCtrlState,uIdx;

	if(!bufPtr)
	{
		log_error("flowCtrlReadMsg: Invalid input param");
		return -1;
	}

	uVersion = bufPtr[0];
	uNChannels = bufPtr[1];

	/* check version */
	if ((IPC_PROTOCOL_VERSION_1 & IPC_PROTOCOL_VERSION_MASK) == (uVersion & IPC_PROTOCOL_VERSION_MASK))
	{
		pFlowCtrlMsg = &bufPtr[IPC_FLOWCTRL_HDRLEN];

		/* Validate the Length Of message */
		if((uNChannels * 4 + IPC_FLOWCTRL_HDRLEN) != (Int32)dataLen) {
			log_error( "Invalid FlowCtrl message Length %u received (nChannels = %d)", dataLen, uNChannels);
			return -1;
		}

		for (uIdx = 0; (uIdx < uNChannels); uIdx++) {
			log_debug("pFlowCtrlMsg 0 -3 : 0x%x  0x%x  0x%x  0x%x", pFlowCtrlMsg[0], pFlowCtrlMsg[1],  pFlowCtrlMsg[2],  pFlowCtrlMsg[3]);
			uChannelID = pFlowCtrlMsg[0];
                        uFlowCtrlState = pFlowCtrlMsg[2];

			if ((IPC_XON == uFlowCtrlState) || (IPC_XOFF == uFlowCtrlState))
			{
				switch (uChannelID)
				{
					/* if a XON or XOFF message was received for channel 0
					 * all channels will be enabled or disabled. Additionally
					 * the watchdog mechanism will be enabled or disabled */
					case IPC_FLOWCTRL_CHANNEL:
						if (IPC_XON == uFlowCtrlState)
						{
							if (FALSE == ipcServerIsEnabled())
							{
								log_notice( "Call ipcServerEnable(): Flow Control Channel XON received");
								/* Enable IPC server */
								ipcServerEnable();
							}
						}
						else
						{
							log_notice( "Call ipcServerDisable(): Flow Control Channel XOFF received");
							/* Disable IPC server */
							ipcServerDisable();
						}
						break;

					case IPC_WATCHDOG_CHANNEL:
						break;

					/* do only change flow control state for channels
					 * greater than 1 */
					default:
						pDeviceAttr = getDeviceAttrPtr(uChannelID);
						if (NULL != pDeviceAttr)
						{
							mutex_lock(&pDeviceAttr->txFlowCtrlMutex);
							if (TRUE == sflowCtrlIsEnabled)
							{
								if (pDeviceAttr->txFlowCtrlState != uFlowCtrlState)
								{
									if (IPC_XON == pFlowCtrlMsg[2])
									{
										log_notice( "XON for CH-%d recvd", uChannelID);
										uSetEvent = TRUE;
									}
									else
									{
										log_notice( "XOFF for CH-%d recvd", uChannelID);
									}

									pDeviceAttr->txFlowCtrlState = uFlowCtrlState;
									flowCtrlUpdateTxFlowControlMask(uFlowCtrlState, uChannelID);
								}
							}
							mutex_unlock(&pDeviceAttr->txFlowCtrlMutex);
						}
						else
						{
							log_warning( "Invalid channel '%d' specified in FlowCtrl message ", uChannelID);
						}
						break;
				}
			}

			pFlowCtrlMsg += 4;
		}

		if(TRUE == uSetEvent) {
			setEventDataAvailable();
		}
	}
	else
	{
		log_error( "Flowctrl Version conflict (required: %02X, received: %02X", IPC_PROTOCOL_VERSION_1, uVersion);
		return -1;
	}
	return iRet;
}


/**
 * This function reads and parses the flowcontrol message for all channels.
 *
 * @param  bufPtr  - pointer to data
 * @param  dataLen - pointer to store the length of flowcontrol message
 *
 * @return 0 on success, -1 on failure
 */
Int8 flowCtrlWriteMessage(UInt8 *bufPtr, UInt32 *dataLen)
{
	TDeviceAttr* 	pDeviceAttr = NULL;
	ipc_msghdr_t	*ipcMsgHdr = NULL;
	UInt8 		uChannel, uIndex;

	if(!bufPtr  || !dataLen)
	{
		log_error("flowCtrlWriteMsg: Invalid input param");
		return -1;
	}

	ipcMsgHdr = (ipc_msghdr_t *) bufPtr;

	ipcMsgHdr->channel 	= IPC_FLOWCTRL_CHANNEL;
	ipcMsgHdr->frame_type 	= 0;
	ipcMsgHdr->frLen	= IPC_FLOWCTRL_HDRLEN;

	bufPtr[IPC_HEADER_SIZE] = IPC_PROTOCOL_VERSION_1;
	bufPtr[IPC_HEADER_SIZE+1] = gNoChannels + IPC_FREE_CHANNEL;

	uIndex = IPC_HEADER_SIZE + IPC_FLOWCTRL_HDRLEN;
#ifdef	FLOW_CONTROL
	for (uChannel = 0; uChannel < (gNoChannels+IPC_FREE_CHANNEL); uChannel++)
	{
		bufPtr[uIndex] = (UInt8)uChannel;

		pDeviceAttr = getDeviceAttribute(uChannel);
		if (NULL != pDeviceAttr)
		{
			mutex_lock(&pDeviceAttr->accessMutex);
			bufPtr[uIndex + 1] = pDeviceAttr->clients;
			if((pDeviceAttr->prevClients == 1) && (pDeviceAttr->clients == 0))
			{
				if(!queueIsEmpty(pDeviceAttr->txQueue))
				{
					bufPtr[uIndex + 1] = pDeviceAttr->prevClients;
				}
			}

			bufPtr[uIndex + 2] = pDeviceAttr->rxFlowCtrlState;
			pDeviceAttr->prevClients = bufPtr[uIndex + 1];
			mutex_unlock(&pDeviceAttr->accessMutex);
		}
		else
		{
			bufPtr[uIndex + 1] = 0;
			bufPtr[uIndex + 2] = IPC_XOFF;
		}
		bufPtr[uIndex + 3] = 0;

		uIndex += 4;
		ipcMsgHdr->frLen += 4;
	}
#endif
	*dataLen = uIndex;
	return EOK;
}
