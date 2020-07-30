/******************************************************************
 *  Project          Harman Car Multimedia System
 *  (c) copyright    2018
 *  Company          Harman International Industries, Incorporated
 *                   All rights reserved
 *  Secrecy Level    STRICTLY CONFIDENTIAL
 ******************************************************************/
/**
 *  @file            fs3ipc_app.h
 *  @ingroup         fs3ipc Component
 *  @author          David Rogala
 *  @brief
 */

#ifndef FS3IPC_APP_H_
#define FS3IPC_APP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "fs3ipc_cfg.h"
#include "fs3ipc_flowControl.h"

/**
 * Defines the invalid parameter for Application layer
 */
#define FS3IPC_APP_HANDLE_INVALID (255)
/**
 * Configuration parameter to determine the total number of channels
 */
#define FS3IPC_APP_MAX_NUM_LOGICAL_CHANNEL (32)

/**
 * Defines the FlowControl channel number
  */
#define FS3IPC_APP_FLOWCONTROL_CHAN (1)

#if FS3IPC_APP_STATS_ENABLED == 1

/**
 * structure dedicated for collecting statics values
 */
typedef struct fs3ipc_app_StatsT
{
	fs3ipc_u32 rxWrappedMsgCnt; /**< Total Wrapped Receiver messages */
	fs3ipc_u32 txWrappedMsgCnt; /**< Total Wrapped Transmitted messages*/

	fs3ipc_u32 rxWrappedMsgErrorCnt;			/**< Total Wrapped with error messages*/
	fs3ipc_u32 rxMsgLengthErrorCnt;			/**< Total length error counter */
	fs3ipc_u32 rxUnhandledChannelErrorCnt; /**< Total invalid channel counter */
	fs3ipc_u32 rxGenErrorCnt;					/**< Total dropped error due to configuration error*/
	fs3ipc_u32 rxBufferOverflow;				/**< Total buffer receiver overflow error detected*/
	fs3ipc_u32 txGenErrorCnt;					/**< Total buffer transmitted errors */
} fs3ipc_app_StatsType;
#endif

/**
 * This structure collects the client information
 */
typedef struct Tagfs3ipc_app_LTClientCfg
{
	fs3ipc_u8 clientHandle;				/**< the client ID information >**/
	fs3ipc_os_EventType *osCfg;		/**< OS configuration to handle events/flags >**/
	fs3ipc_os_EventMaskType rxEvent; /**< The expected events related to this channel >**/
} fs3ipc_app_LTClientCfgType;

/*
 * This structure collects the Application Channel information
 */
typedef struct Tagfs3ipc_app_LTChannelCfgType
{
	fs3ipc_u8 *TX_Buffer;		/**< Pointer to access the Tx buffer >**/
	fs3ipc_u8 *RX_Buffer;		/**< Pointer to access the Rx buffer >**/
	fs3ipc_u8 priority;			/**< the corresponding priority for this channel>**/
	fs3ipc_u8 channel;			/**< The client identifier  >**/
	fs3ipc_u16 TX_Buffer_Size; /**< Configuration value to determine the buffer size >**/
	fs3ipc_u16 RX_Buffer_Size; /**< Configuration value to determine the buffer size>**/

	fs3ipc_app_queueType *TX_Q;		 /**< OS resource dedicated for  transmission  >**/
	fs3ipc_os_ResourceType *TX_QLock; /**< OS lock for transmission >**/
	fs3ipc_app_queueType *RX_Q;		 /**< OS resource dedicated for  reception >**/
	fs3ipc_os_ResourceType *RX_QLock; /**< OS lock for reception >**/

	fs3ipc_app_LTClientCfgType clientCfg; /**< client - flow control related >**/
	fs3ipc_u8 *loggingEnabled;
} fs3ipc_app_LTChannelCfgType;

/*
 *
 */
typedef struct Tagfs3ipc_app_clientChannelLookup
{
	/**<The client handle >**/
	fs3ipc_u8 handle;
	/**<The channel to look up>**/
	fs3ipc_u8 channel;
	fs3ipc_u8 initialized;
} fs3ipc_app_clientChannelLookupType;

#pragma pack(1)
/**
 * the fs3ipc_app_messageHdr provides the application header, it determines
 * the length and channel.
 */
typedef struct
{
	/**< the channel Length associated to this channel  >**/
	fs3ipc_u16 length : 11;
	/**< Related Channel ID >**/
	fs3ipc_u16 channel : 5;
} fs3ipc_app_messageHdr;

/**
 * structure dedicated to configure the message header and the buffer data
 */
typedef struct
{
	/**< Application header, provides length and channel parameters >**/
	fs3ipc_app_messageHdr hdr;
	/**<  buffer dedicated to send data >**/
	fs3ipc_u8 data[FS3_IPC_MAX_MSG_SIZE];
} fs3ipc_app_messageType;
#pragma pack()

/**
 * handles the application message information for transmitting  and receiving
 * data
 */
typedef struct
{
	/**< Message header and data information >**/
	fs3ipc_app_messageType message;
	/**< provides the address position >**/
	fs3ipc_u16 position;
} fs3ipc_app_messageBufferType;

/**
 * Structure dedicated to configure all the application layer parameters
 * this structure shall be initialized at the fs3ipc_cfg.h
 */
typedef struct
{
	/**<all channel/client configuration  */
	const fs3ipc_app_LTChannelCfgType *ltChannelCfg[FS3_IPC_APP_MAX_NUMBER_CHANNELS];
	/**< this parameter counts the channel operation*/
	fs3ipc_u8 channelCount;

	/**< channel priority configuration parameters */
	fs3ipc_app_priorityNodeType *orderedChannelListHead;
	fs3ipc_app_priorityNodeType *orderedChannelNodes;

	/**< */
	fs3ipc_os_EventType *phys_txEventOsCfg;

	/**< bitmask associated with client in order to transmit data*/
	fs3ipc_os_EventMaskType phys_txEvent;

	/**<flowcontrol configuration parameters >*/
	const fs3ipc_flowCtrlLTCfgType *flowControlCfg;

	/**<reception buffer configuration message */
	fs3ipc_app_messageBufferType *rxMsg;

	/**< Transmission buffer configuration */
	fs3ipc_app_messageBufferType *txMsg;

	fs3ipc_length *rxMsgBufferOverflowIndex;
#if FS3IPC_APP_STATS_ENABLED == 1
	fs3ipc_app_StatsType *stats;
#endif

} fs3ipc_app_LTconfig;

fs3ipc_StatusType fs3ipc_app_Encoder(
	fs3ipc_handleType handle,
	fs3ipc_dataPtr data,
	fs3ipc_length *ptrLength);

fs3ipc_StatusType fs3ipc_app_Decoder(
	fs3ipc_handleType handle,
	fs3ipc_cDataPtr data,
	fs3ipc_length length);

fs3ipc_StatusType fs3ipc_app_HandleExtNodeReset(
	fs3ipc_handleType handle);

#if FS3IPC_APP_STATS_ENABLED == 1
const fs3ipc_app_StatsType *fs3ipc_app_GetStats(
	fs3ipc_handleType Hndl);

fs3ipc_StatusType fs3ipc_app_ClearStats(
	fs3ipc_handleType Hndl);

const fs3ipc_flowCtrlChanPBCfgType *fs3ipc_app_GetChanStats(
	fs3ipc_handleType Hndl,
	fs3ipc_u8 channel);

fs3ipc_StatusType fs3ipc_app_ClearChanStats(
	fs3ipc_handleType Hndl,
	fs3ipc_u8 channel);
#endif

fs3ipc_StatusType fs3ipc_app_SetLogging(
	fs3ipc_handleType Hndl,
	fs3ipc_u8 channel,
	fs3ipc_u8 enabled);
#ifdef __cplusplus
}
#endif

#endif /* FS3IPC_APP_H_ */
