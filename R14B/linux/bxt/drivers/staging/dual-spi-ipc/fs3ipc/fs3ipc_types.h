/*****************************************************************
* Project Harman Car Multimedia System
* (c) copyright 2019
* Company Harman/Becker Automotive Systems GmbH
* All rights reserved
* Secrecy Level STRICTLY CONFIDENTIAL
****************************************************************/
/***************************************************************
* @file fs3ipc_types.h
* @ingroup fs3ipc Component
* @author David Rogala
* fs3ipc_type header file.
* This file used for defining the types of
****************************************************************/

#ifndef FS3IPC_TYPES_H_
#define FS3IPC_TYPES_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef uint8_t fs3ipc_handleType;
typedef uint32_t fs3ipc_clientHandleType;
typedef uint8_t *fs3ipc_dataPtr;
typedef uint16_t *fs3ipc_dataPtr16;
typedef const uint8_t *fs3ipc_cDataPtr;
typedef uint32_t fs3ipc_length;

#define FS3IPC_FALSE (0)
#define FS3IPC_TRUE (1)
#define FS3IPC_NULL ((void *)0)

#ifndef fs3ipc_StatusType
#define fs3ipc_StatusType fs3ipc_u32
#endif
/**
 * No Error condition
 */
#ifndef fs3ipc_StatusType_OK
#define fs3ipc_StatusType_OK (0)
#endif
/**
 * Error configuration error condition
 */
#ifndef fs3ipc_StatusType_ErrorGen
#define fs3ipc_StatusType_ErrorGen (1)
#endif

/**
 * Configuration error condition
 */
#ifndef fs3ipc_StatusType_ErrorCfg
#define fs3ipc_StatusType_ErrorCfg (2)
#endif

/**
 * Timeout error with a partial transfer condition
 */
#ifndef fs3ipc_StatusType_ErrorXferTimeout
#define fs3ipc_StatusType_ErrorXferTimeout (3)
#endif

/**
 * Invalid handle condition
 */
#ifndef fs3ipc_StatusType_ErrorHandle
#define fs3ipc_StatusType_ErrorHandle (4)
#endif

/**
 * message doesn't fit in current buffer error condition
 */
#ifndef fs3ipc_StatusType_BufferFull
#define fs3ipc_StatusType_BufferFull (5)
#endif

/**
 * Invalid frame size detected
 */
#ifndef fs3ipc_StatusType_Invalid_Frame_Size
#define fs3ipc_StatusType_Invalid_Frame_Size (6)
#endif

/**
 *
 */
#ifndef fs3ipc_StatusType_ErrorLostMessage
#define fs3ipc_StatusType_ErrorLostMessage (7)
#endif

/**
 * Buffer empty error detected
 */
#ifndef fs3ipc_StatusType_BufferEmpty
#define fs3ipc_StatusType_BufferEmpty (8)
#endif

/**
 *Channel not available, the Flowcontrol message has not
 *been enable the Channel
 */
#ifndef fs3ipc_StatusType_ChannelNotAvailable
#define fs3ipc_StatusType_ChannelNotAvailable (9)
#endif

/**
 * Invalid length detected
 */
#ifndef fs3ipc_StatusType_MessageSize
#define fs3ipc_StatusType_MessageSize (10)
#endif

/**
 * Transmission Window is full, normally the transmission window is full
 * when there is no received any ACK message
 */
#ifndef fs3ipc_StatusType_Tx_WindowFull
#define fs3ipc_StatusType_Tx_WindowFull (11)
#endif

/**
 *The client already enabled. only one client allowed
 */
#ifndef fs3ipc_StatusType_ErrorClientState
#define fs3ipc_StatusType_ErrorClientState (12)
#endif

/**
 * Invalid CRC detected
 */
#ifndef fs3ipc_StatusType_Rx_InvalidCRC
#define fs3ipc_StatusType_Rx_InvalidCRC (13)
#endif

/**
 * Invalid initialization detected
 */
#ifndef fs3ipc_StatusType_NOINIT
#define fs3ipc_StatusType_NOINIT (14)
#endif

/**
 * Invalid initialization detected
 */
#ifndef fs3ipc_StatusType_ErrorInterrupted
#define fs3ipc_StatusType_ErrorInterrupted (15)
#endif

	typedef uint32_t fs3ipc_u32;
	typedef uint16_t fs3ipc_u16;
	typedef uint8_t fs3ipc_u8;
	typedef int8_t fs3ipc_s8;
	typedef int32_t fs3ipc_s32;
	typedef int16_t fs3ipc_s16;

/**
 * UNUSED fs3ipc_fp_GetBuffer function pointer
 */
	typedef fs3ipc_StatusType (*fs3ipc_fp_GetBuffer)(fs3ipc_handleType,
		fs3ipc_dataPtr *,
		fs3ipc_length *);

/**
 * UNUSED fs3ipc_fp_EncodeMessage function pointer
 */
	typedef fs3ipc_StatusType (*fs3ipc_fp_EncodeMessage)(fs3ipc_handleType,
		fs3ipc_dataPtr,
		fs3ipc_length *);

/**
 * UNUSED fs3ipc_fp_DecodeMessage function pointer
 */
typedef fs3ipc_StatusType (*fs3ipc_fp_DecodeMessage)(fs3ipc_handleType,
	fs3ipc_cDataPtr,
	fs3ipc_length);

/**
 * Function pointer called when flowcontrol message is received
 */
typedef void (*fs3ipc_fp_flowControlCb)(fs3ipc_u8, fs3ipc_u8);

#ifdef __cplusplus
}
#endif

#endif /* AUTOSAR_APPL_SOURCE_FS3IPC_FS3IPC_TYPES_H_ */
