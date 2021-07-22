/*****************************************************************
* Project Harman Car Multimedia System
* (c) copyright 2019
* Company Harman/Becker Automotive Systems GmbH
* All rights reserved
* Secrecy Level STRICTLY CONFIDENTIAL
****************************************************************/
/***************************************************************
* @file fs3ipc_hdlc.h
* @ingroup fs3ipc Component
* @author David Rogala
* Data linker layer header
****************************************************************/
#ifndef FS3IPC_HDLC_H_
#define FS3IPC_HDLC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fs3ipc_cfg.h"

/*----------------------------------------------------------------*/
/*----------------------------------------------------------------*/
/***************************************
 * defines for Configuration
***************************************/
#define HDLC_DATA_MAX_LEN (FS3IPC_TRANS_SIZE_MAX)
#define HDLC_DATA_MIN_LEN (FS3IPC_TRANS_SIZE_MAX)

/* hdlc*/
#define FS3IPC_SPI_IPC_NO_DATA_REMOVED (0)

#ifndef WINDOW_SIZE
#define WINDOW_SIZE 5							/**< Window Size        >**/
#endif

#define DEFAULT_WINDOW_SIZE (WINDOW_SIZE) /**< Window Size        >**/
#define INVALID_SEQ (0x7)						/**< HDLC Header Length >**/

/***************************************
 * Defines related to length
 ***************************************/
#define HDLC_FRAME_HEADER_LEN (2) /**< HDLC Header Length >**/
#define HDLC_CONTROL_LEN (1)
#define HDLC_ADDRESS_LEN (1)
// HDLC_CONTROL_LEN+ HDLC_ADDRESS_LEN
#define HDLC_CTR_ADDR_LEN (2)
// window size + buffer size
#define HDLC_UFRAME_PAYLOAD_LEN (3)
// CRC Length
#define CRC_SIZE (2)
//HDLC_FRAME_HEADER_LEN+HDLC_CONTROL_LEN+ HDLC_ADDRESS_LEN+CRC_SIZE
#define HDLC_TOTAL_FRAME (6)
//HDLC_FRAME_HEADER_LEN + HDLC_CONTROL_LEN +HDLC_ADDRESS_LEN
#define HDLC_TOTAL_HEADER (4)


/* DRogala - length and index definitions */
#define HDLC_MSG_SIZE_LEN (2)
#define HDLC_HDR_ADDR_LEN (1)
#define HDLC_HDR_CTRL_LEN (1)
#define HDLC_CRC_LEN      (2)

#define HDLC_MSG_SIZE_IDX (0)
#define HDLC_HDR_ADDR_IDX (HDLC_MSG_SIZE_IDX + HDLC_MSG_SIZE_LEN)
#define HDLC_HDR_CTRL_IDX (HDLC_HDR_ADDR_IDX + HDLC_HDR_ADDR_LEN)
#define HDLC_DATA_IDX     (HDLC_HDR_CTRL_IDX + HDLC_HDR_CTRL_LEN)
/***************************************
 * defines for Position
 * HDLC FORMAT
 * POS0*** POS1***POS2***POS3***POS4***POS5***POS6*** POS7***POS8***
 * Len *** len ***Addr***CTRL***Chan***RES ***Len *** Len ***Data***
 ***************************************/
#define HDLC_ADDR_INDEX (2)
#define HDLC_CTRL_INDEX (3)
#define HDLC_CHANNEL_INDEX (4)

/************************************
 * defines for offset
 ************************************/
#define HDLC_FRAME_HEADER_OFFSET (2)
#define HDLC_FRAME_DATA_OFFSET (4)

/**********************************
 * Mask defines
 *********************************/
#define HDLC_FRAME_TYPE_MASK (3)

/**************************************
 * Timer configuration
 * TODO set the proper values when the
 * timers are enables
 *************************************/
 /* flags options*/
#define FS3IPC_HDLC_FLAG_IFRAME_TIMEOUT_ENABLED (1u << 1)
#define FS3IPC_HDLC_FLAG_IFRAME_TIMEOUT_DISABLED (0u << 1)

#define FS3IPC_HDLC_FLAG_UFRAME_TIMEOUT_ENABLED (1u << 1)
#define FS3IPC_HDLC_FLAG_UFRAME_TIMEOUT_DISABLED (0u << 1)

/***************************************
 * defines used for calculate position
 */
#define FRAME_DATA_INDEX (HDLC_FRAME_HEADER_LEN + HDLC_CONTROL_LEN + \
								  HDLC_ADDRESS_LEN)
#define FRAME_MIN_LENGTH_SUPPORTED (HDLC_CTR_ADDR_LEN + CRC_SIZE)

/**************************************
 * Events used for HDLC handling
 *************************************/

#define EVENT_ALL (EVENT_TX_MSG|EVENT_RX_RST_UFRAME|EVENT_RX_ACK_UFRAME|\
		EVENT_TX_TIMEOUT|EVENT_RX_OUT_OF_SEQ_FRAME|EVENT_RX_RR_POLL_FRAME|\
		EVENT_RX_REJ_FRAME|EVENT_RX_RR_FINAL_FRAME|EVENT_RX_IFRAME|\
		EVENT_EXT_WAKEUP|EVENT_SUSPEND|EVENT_RESUME|EVENT_TX_WAKEUP)

/* Frame types */
#define I_FRAME (0x0)
#define S_FRAME (0x1)
#define U_FRAME (0x3)

/* Types of S-frame */
#define RR_FRAME (0x00)
#define REJ_FRAME (0x01)
#define RNR_FRAME (0x10)
#define SREJ_FRAME (0x11)

/* Types of U-frame */
#define URST_FRAME (19)
#define UACK_FRAME (12) /*UA - Unnumbered acknowledgement*/

#define POLL_COMMAND (0x1)
#define POLL_RESPONSE (0x0)

#define POLL_REQUEST (0x1)
#define FRAME_REQUEST (0x0)

/* FLAGS */
/* This flag informs the hdlc module to pad the message so that it is a consistent length. 
   This is necessary for certain physical layers such as for fixed transfer length SPI. */
#define FS3IPC_HDLC_FLAG_TX_FULL_FRAME (1u<<0)

typedef enum
{
	DIAG_MSG_ID_IFRAME_TIMEOUT,
	DIAG_MSG_ID_UFRAME_TIMEOUT
} HDLCMsgID_e;

/** Exposed API's **/

typedef struct
{
	/*OS*/
	fs3ipc_os_EventType *OS_Task;
	fs3ipc_os_EventMaskType event_task;
} fs3ipc_hdlc_OSCfgType;

typedef struct
{
	fs3ipc_u32 RxIframeCnt;
	fs3ipc_u32 TxIframeCnt;
	fs3ipc_u32 RxSFrameCnt;
	fs3ipc_u32 TxSFrameCnt;

	fs3ipc_u32 RxInvalidFrameCnt;
	fs3ipc_u32 TxRetransmit;
	fs3ipc_u32 TxAckTimeout;
	fs3ipc_u32 RxCrcErrorCnt;
	fs3ipc_u32 RxSequenceErrorCnt;
	fs3ipc_u32 RxResetReqCnt;
	fs3ipc_u32 RxPollReqCnt;
} fs3ipc_hdlc_StatsType;

typedef struct
{
	/*TIMER */
	/* configuration flags*/
	fs3ipc_u32 hdlcConfigFlags;
	fs3ipc_os_TimeoutType *wIfTimer;
	fs3ipc_os_TimeoutType *wUfTimer;
	fs3ipc_u16 wIfTxTimeout;
	fs3ipc_u16 wUfTxTimeout;
	fs3ipc_u8 uWindowSiz;
	fs3ipc_u16 uBufferSiz;

	fs3ipc_hdlc_OSCfgType hdlcOSCfg;

#if FS3IPC_HDLC_STATS_ENABLED == 1
	fs3ipc_hdlc_StatsType *stats;
#endif

	fs3ipc_os_ResourceType *lock;
	fs3ipc_u32 flags;
} fs3ipc_hdlc_AppConfigType;

extern fs3ipc_StatusType fs3ipc_hdlc_GetRxBuffer_Cb(
	fs3ipc_handleType handle,
	fs3ipc_dataPtr *dataPtr,
	fs3ipc_length *buffer_length);

/*
 *
 */
extern fs3ipc_StatusType fs3ipc_hdlc_GetTxBuffer_Cb(fs3ipc_handleType handle,
																	 fs3ipc_dataPtr *dataPtr,
																	 fs3ipc_length *length);

extern fs3ipc_StatusType fs3ipc_hdlc_Encode_Cb(fs3ipc_handleType handle,
															  fs3ipc_dataPtr dataPtr,
															  fs3ipc_length *length);

extern fs3ipc_StatusType fs3ipc_hdlc_Decode_Cb(fs3ipc_handleType handle,
															  fs3ipc_cDataPtr dataPtr,
															  fs3ipc_length length);

extern void FS3IPC_Process_Iframe_Timeout(fs3ipc_handleType handle);

extern void FS3IPC_Process_Uframe_Timeout(fs3ipc_handleType handle);

extern fs3ipc_s16 FS3IPC_hdlcInitialize(fs3ipc_u8 uLldId);

#if FS3IPC_HDLC_STATS_ENABLED == 1
const fs3ipc_hdlc_StatsType *fs3ipc_hdlc_GetStats(fs3ipc_handleType Hndl);

fs3ipc_StatusType fs3ipc_hdlc_ClearStats(fs3ipc_handleType Hndl);
#endif

extern void fs3ipc_hdlc_suspend(fs3ipc_handleType handle);
extern void fs3ipc_hdlc_resume(fs3ipc_handleType handle);
extern void fs3ipc_TxThread(fs3ipc_handleType handle);

/*******************************************************************
 * Unit test define space
 */
/*
 * only enable DATA_LAYER_TEST for calling unit test
 */
//#define DATA_LAYER_TEST
//extern void Spi_RunUnitTest(void);
/*----------------------------------------------------------------*/
/*----------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* FS3IPC_HDLC_H_ */
