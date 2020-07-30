/*****************************************************************
* Project Harman Car Multimedia System
* (c) copyright 2019
* Company Harman/Becker Automotive Systems GmbH
* All rights reserved
* Secrecy Level STRICTLY CONFIDENTIAL
****************************************************************/
/***************************************************************
* @file fs3ipc.h
* @ingroup fs3ipc Component
* @author David Rogala
* Module header file
****************************************************************/

#ifndef AUTOSAR_APPL_SOURCE_FS3IPC_FS3IPC_H_
#define AUTOSAR_APPL_SOURCE_FS3IPC_FS3IPC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fs3ipc_types.h"

fs3ipc_StatusType fs3ipc_Init (fs3ipc_handleType handle);
fs3ipc_StatusType fs3ipc_DeInit (void);
fs3ipc_StatusType fs3ipc_WriteMessage (fs3ipc_clientHandleType,
	fs3ipc_cDataPtr data,
	fs3ipc_length length);

fs3ipc_StatusType fs3ipc_ReadMessage (fs3ipc_clientHandleType,
	fs3ipc_dataPtr data,
	fs3ipc_length *length);

fs3ipc_StatusType fs3ipc_OpenChannel(fs3ipc_clientHandleType clientHandle);
fs3ipc_StatusType fs3ipc_CloseChannel(fs3ipc_clientHandleType clientHandle);
fs3ipc_StatusType fs3ipc_GetChannelAvailable(fs3ipc_clientHandleType clientHandle);
fs3ipc_StatusType fs3ipc_GetMessagePending(fs3ipc_clientHandleType clientHandle, fs3ipc_u32 *result);
fs3ipc_StatusType fs3ipc_WaitForMessage (fs3ipc_clientHandleType clientHandle);

fs3ipc_StatusType fs3ipc_WaitForMessageTimeout (fs3ipc_clientHandleType clientHandle, fs3ipc_u32 timeoutInMsec );
void fs3ipc_MainFunctionHandler(fs3ipc_handleType handle);

#ifdef __cplusplus
}
#endif

#endif /* AUTOSAR_APPL_SOURCE_FS3IPC_FS3IPC_H_ */
