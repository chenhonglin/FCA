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
 * Flow control implementation.
 */

#ifndef FLOW_CONTROL_H
#define FLOW_CONTROL_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "types.h"
#include "ipc.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/

#define IPC_XON                	1
#define IPC_XOFF               	0

#define IPC_XON_THRESHOLD      	4
#define IPC_XOFF_THRESHOLD     	2

#define IPC_FLOWCTRL_HDRLEN     2


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

void   	flowCtrlEnable(void);
void   	flowCtrlDisable(void);
UInt32 	flowCtrlGetTxState(TDeviceAttr* pDeviceAttr);
void   	flowCtrlCheckRxState(UInt8 channel);
Int32   flowCtrlReadMessage(UInt8 *bufPtr, UInt32 dataLen);
Int8   	flowCtrlWriteMessage(UInt8 *bufPtr, UInt32 *dataLen);
void    flowCtrlUpdateTxFlowControlStatus(UInt32 qMaskUpdate);
UInt32  flowCtrlGetRxFlowControlMask(void);


#endif /* FLOW_CONTROL_H */
