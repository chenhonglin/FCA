/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	handler.h
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * This header file contains the declarations for all the
 * API's provided by the IPC protocol layer
 */

#ifndef HANDLER_H
#define HANDLER_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

Int32 handlerCreatePriorityLists(IPCAttr_t *pIpcRoot);
void  handlerCleanup(void);
void  handlerEnable(void);
void  handlerDisable(void);
void  handlerTriggerTransmission(UInt8 channel);

Int32 getIpcTxBuffer(UInt8 *bufPtr, UInt32 *dataLen);
UInt8 dataAvailable(void);
Int32 handlerProcessReceiveEvent(UInt8 *bufPtr, UInt32 dataLen);
Int32 handlerProcessReceiveEvent1(UInt8 *bufPtr, UInt32 dataLen);

#endif /* HANDLER_H */
