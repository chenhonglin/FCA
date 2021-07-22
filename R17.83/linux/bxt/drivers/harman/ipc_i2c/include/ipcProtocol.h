/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	ipcProtocol.h
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * This header file defines the data structures for the IPC layer protocol handling
 */

#ifndef IPCPROTOCOL_H
#define IPCPROTOCOL_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/


/*****************************************************************************
 * DEFINES
 *****************************************************************************/

#define IPC_PROTOCOL_VERSION_1  (UInt8)0x10 /* protocol version 1 */

#define IPC_MAX_CHANNELS        32
#define IPC_FREE_CHANNEL        2
#define CHANNEL2_MINOR          2

#define IPC_FLOWCTRL_CHANNEL    0
#define IPC_WATCHDOG_CHANNEL    1

#define IPC_DATA_SIZE           21
#define IPC_HEADER_SIZE         4
#define IPC_BUFFER_SIZE         (IPC_HEADER_SIZE + IPC_DATA_SIZE)
#define MAX_IPC_PKT_LEN         IPC_BUFFER_SIZE

#define IPC_PROTOCOL_VERSION_MASK                       \
                                (UInt8)0xF0

#define IPC_TX_QUEUE_SIZE       20       /* number of tx queue elements */
#define IPC_RX_QUEUE_SIZE       20       /* number of rx queue elements */

#define IPC_GET_MAJOR_PROTOCOL_VERSION(version)         \
                                (UInt8)((version >> 4) & 0x0F)
#define IPC_GET_MINOR_PROTOCOL_VERSION(version)         \
                                (UInt8)(version & 0x0F)

/* --------------------------------------------------------------------- */
/*
 * Field Name(Number of Bits)
 ___________________________________________________
| Channel (8) |Res/FrType(8) | 	Data Length  (16)   |
|_____________|______________|______________________|

The maximum length of one IPC Message can be 512B in length. This is only
limited by the Tx Transfer size configuration for this channel.
 */

#pragma pack(push, 1)
typedef struct
{
	UInt8                   channel;        /**< IPC channel no */
	UInt8                   frame_type;     /**< frame type */
	UInt16                  frLen;          /**< sequence count and message length */
} ipc_msghdr_t;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct
{
	UInt8                   channel;        /**< IPC channel no */
	UInt8                   frame_type;     /**< frame type */
	UInt16                  frLen;          /**< sequence count and message length */
	UInt8                   data[IPC_DATA_SIZE];
}TIpcMsg;
#pragma pack(pop)

typedef TIpcMsg                 TWriteMsg;
typedef TIpcMsg                 TReadMsg;


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/


#endif /* IPCPROTOCOL_H */
