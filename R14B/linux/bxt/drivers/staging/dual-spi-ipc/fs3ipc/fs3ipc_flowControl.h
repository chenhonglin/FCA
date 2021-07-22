
/*****************************************************************
* Project Harman Car Multimedia System
* (c) copyright 2019
* Company Harman/Becker Automotive Systems GmbH
* All rights reserved
* Secrecy Level STRICTLY CONFIDENTIAL
****************************************************************/
/***************************************************************
* @file fs3ipc_flowControl.h
* @ingroup fs3ipc Component
* @author David Rogala
* Physical Layer header file
****************************************************************/

#ifndef FS3IPC_FLOWCONTROL_H_
#define FS3IPC_FLOWCONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "fs3ipc_cfg.h"

/***************************************
 * defines for Configuration
***************************************/
/** if the image supports position independant code add code
* offset to the linker time  function pointer before executing
*/
#if FS3IPC_PIC_EN == 1
#define GET_RX_FC_CB(cfg, ch)\
    ((fs3ipc_fp_flowControlCb)(((fs3ipc_u8*)(cfg)->ltStChanConfig[(ch)].rxCallback) + fs3ipc_getCodeOffset()))
#else
#define GET_RX_FC_CB(cfg, ch)\
    ((cfg)->ltStChanConfig[(ch)].rxCallback)
#endif

/******************************************************************************
 * structures, macros and constants
 *****************************************************************************/
/**
 * fc channel dynamic configuration data
 * */
typedef struct
{
	fs3ipc_u8 txEnabled;
	fs3ipc_u8 rxOverflow;
	fs3ipc_u8 rxEnabled;
	fs3ipc_u16 maxTxPerFrame;
	fs3ipc_u16 byteCountTxFrame;
#if FS3IPC_APP_STATS_ENABLED == 1
	fs3ipc_u16 XoffCnt;
	fs3ipc_u32 rxCnt;
	fs3ipc_u32 txCnt;
#endif
} fs3ipc_flowCtrlChanPBCfgType;

/**
 * fc instance dynamic configuration data
 * */
typedef struct
{
	fs3ipc_u8 pendingRxReply;
	fs3ipc_u8 pendingTxRequest;
	fs3ipc_u8 loggingEnabled;
} fs3ipc_flowCtrlPBCfgType;

/**
 * fc instance constant channel configuration data
 * */
typedef struct
{
	fs3ipc_fp_flowControlCb rxCallback;
	fs3ipc_u16 maxRxPerFrame;
} fs3ipc_flowCtrlChanLTCfgType;

/**
 * fc instance constant instance configuration data
 * */
typedef struct
{
	fs3ipc_u8 channelCount;
	fs3ipc_flowCtrlPBCfgType *pbStConfig;
	fs3ipc_flowCtrlChanPBCfgType *pbStChanConfig;
	const fs3ipc_flowCtrlChanLTCfgType *ltStChanConfig;
	fs3ipc_os_ResourceType *resLock;
} fs3ipc_flowCtrlLTCfgType;

/******************************************************************************
 * declarations
 *****************************************************************************/

fs3ipc_u8 fs3ipc_flowControl_MsgPending(
	const fs3ipc_flowCtrlLTCfgType *appCfg);

fs3ipc_u32 fs3ipc_flowControl_MsgSize(
	fs3ipc_u8 channelCount);

fs3ipc_StatusType fs3ipc_flowControl_Decode(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_cDataPtr data,
	fs3ipc_length length);

fs3ipc_StatusType fs3ipc_flowControl_Encode(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_dataPtr data,
	fs3ipc_length *length);

fs3ipc_StatusType fs3ipc_flowControl_GetTxEnabled(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel);

fs3ipc_StatusType fs3ipc_flowControl_ResetByteCountTxFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg);
fs3ipc_StatusType fs3ipc_flowControl_GetFreeByteCountTxFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel,
	fs3ipc_length *output);

fs3ipc_StatusType fs3ipc_flowControl_AddByteCountTxFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel,
	fs3ipc_length byteCount);

fs3ipc_StatusType fs3ipc_flowControl_SetOverflow(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel,
	fs3ipc_u8 value);
fs3ipc_u32 fs3ipc_flowControl_GetMaxTxPerFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel);

fs3ipc_u32 fs3ipc_flowControl_GetMaxRxPerFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel);

fs3ipc_StatusType fs3ipc_flowControl_EnableRxClient(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel,
	fs3ipc_u8 enabled);

fs3ipc_StatusType fs3ipc_flowcontrol_Init(
	const fs3ipc_flowCtrlLTCfgType *appCfg);

fs3ipc_StatusType fs3ipc_flowcontrol_HandleNodeReset(
	const fs3ipc_flowCtrlLTCfgType *appCfg);

#if FS3IPC_APP_STATS_ENABLED == 1

fs3ipc_StatusType fs3ipc_flowControl_IncRxMsgCnt(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel);

fs3ipc_StatusType fs3ipc_flowControl_IncTxMsgCnt(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel);

const fs3ipc_flowCtrlChanPBCfgType *fs3ipc_flowcontrol_GetStats(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel);

fs3ipc_StatusType fs3ipc_flowcontrol_ClearStats(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel);

#else

#define fs3ipc_flowControl_IncRxMsgCnt(a, b)
#define fs3ipc_flowControl_IncTxMsgCnt(a, b)

#endif

fs3ipc_StatusType fs3ipc_flowcontrol_SetLogging(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 enabled);

#ifdef __cplusplus
}
#endif

#endif /* FS3IPC_FLOWCONTROL_H_ */
