/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	ipcSockServer.h
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * This header file contains the declarations for all the
 * API's provided by the User interface/Socket layer
 */

#ifndef IPCSOCKSERVER_H
#define IPCSOCKSERVER_H

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "types.h"
#include "queue.h"
#include "ipc.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

Int32        ipcDeviceInit(IPCAttr_t *ipcRoot);
Int32        ipcMsgQueueInit( IPCAttr_t *ipcRoot );
void         ipcMsgQueueCleanup( IPCAttr_t *ipcRoot );
void         ipcDeviceCleanup( IPCAttr_t *ipcRoot );

void 	     ipcServerCleanup( IPCAttr_t *ipcRoot );
void         ipcServerEnable(void);
void         ipcServerDisable(void);
UInt8        ipcServerIsEnabled(void);

TDeviceAttr* getDeviceAttrPtr(UInt32 device);
TDeviceAttr* getDeviceAttribute(UInt32 device);

UInt8        ipcServerRxIsEnabled( void );
void         ipcServerRxEnable( void );
void         ipcServerRxDisable( void );


#endif /* IPCSOCKSERVER_H */
