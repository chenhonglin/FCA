/*****************************************************************************
 * Project              Harman Car Multimedia System
 *
 * c) copyright         2015
 * Company              Harman International Industries, Incorporated
 *                      All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file                ipc_handler.c
 * @author              Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup             Kernel and Drivers
 *
 * IPC protocol handler implementation.
 * This module contains the protocol implementation on the IPC layer.
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>

#include "types.h"
#include "log.h"
#include "ipcProtocol.h"
#include "flowControl.h"
#include "ipcDev.h"
#include "statistics.h"
#include "ipc.h"

/*****************************************************************************
 * FUNCTION DEFINITIONS
 *****************************************************************************/

/**
 * Print statistic values.
 *
 * @return none
 */
#ifdef	IPC_STATS
void statisticsPrint(IPCAttr_t *pIpcRoot)
#else
void statisticsPrint(struct seq_file *m, IPCAttr_t *pIpcRoot)
#endif
{
	TDeviceAttr* pDeviceAttr;
	UInt32 i;

	if(pIpcRoot == NULL)
		return;

	for (i = 0; i < (pIpcRoot->uNoChannels + IPC_FREE_CHANNEL); i++) {

		pDeviceAttr = &pIpcRoot->deviceAttr[i];

		if (IPC_FLOWCTRL_CHANNEL == pDeviceAttr->channelID) {
#ifdef	IPC_STATS
			log_notice(
#else
			seq_printf(m,
#endif
					"FC(TX - %s RX - %s), Msg cnt: rx %u/tx %u ",

					((pIpcRoot->deviceAttr[i].txFlowCtrlState == IPC_XOFF) ? "XOFF" : "XON") ,
					((pIpcRoot->deviceAttr[i].rxFlowCtrlState == IPC_XOFF) ? "XOFF" : "XON") ,
					pDeviceAttr->statistics.rxMsgCnt, pDeviceAttr->statistics.txMsgCnt);

		} else if (IPC_WATCHDOG_CHANNEL == pDeviceAttr->channelID) {

		} else {
#ifdef	IPC_STATS
			log_notice(
#else
			seq_printf(m,
#endif
					"CH-%02u:(TX:%s RX:%s): TX %u|%u|%u:%u / RX %u|%u|%u:%u",
					pDeviceAttr->channelID,
					((pIpcRoot->deviceAttr[i].txFlowCtrlState == IPC_XOFF) ? "XOFF" : "XON") ,
					((pIpcRoot->deviceAttr[i].rxFlowCtrlState == IPC_XOFF) ? "XOFF" : "XON"),
				pDeviceAttr->statistics.txMsgCnt, queueGetNumElements(pDeviceAttr->txQueue), pDeviceAttr->statistics.maxTxMsgSize, pDeviceAttr->statistics.txNBytes,
				pDeviceAttr->statistics.rxMsgCnt, queueGetNumElements(pDeviceAttr->rxQueue), pDeviceAttr->statistics.maxRxMsgSize, pDeviceAttr->statistics.rxNBytes);
		}
	}
}
