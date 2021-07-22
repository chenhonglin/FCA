/******************************************************************
 *  Project          Harman Car Multimedia System
 *  (c) copyright    2018
 *  Company          Harman International Industries, Incorporated
 *                   All rights reserved
 *  Secrecy Level    STRICTLY CONFIDENTIAL
 ******************************************************************/
/**
 *  @file            fs3ipc_flowControl.c
 *  @ingroup         fs3ipc Component
 *  @author          David Rogala
 *  @brief
 */

#include "fs3ipc_cfg.h"
#include "fs3ipc_flowControl.h"

#ifdef LOG_MODULE
#undef LOG_MODULE
#endif
#define LOG_MODULE "fs3ipc_flowControl"

/******************************************************************************
 * structures, macros and constants
 *****************************************************************************/

#pragma pack(1)
/** Header structure used for parsing/encoding flow control frame*/
typedef struct
{
	fs3ipc_u8 protocolVersion;
	fs3ipc_u8 msgId;
} flowControlMsgHeader_t;

/** Expected protocol version*/
#define PROTOCOL_VERSION_ID_EXP (0x20)
/** Request message type*/
#define MSG_ID_CHANNEL_STATUS_REQUEST (0x01)
/** Reply message type*/
#define MSG_ID_CHANNEL_STATUS_REPLY (0x02)
/** Flow control header size */
#define CHANNEL_STATUS_HDR_SIZE (3)

/** channel status field structure used for parsing/encoding flow control frame*/
typedef struct
{
	fs3ipc_u16 maximumBytesPerFrame : 15;
	fs3ipc_u16 channelEnabled : 1;
} flowControlMsgChannelStatusField_t;

/** message Header structure used for parsing/encoding flow control frame*/
typedef struct
{
	flowControlMsgHeader_t header;
	fs3ipc_u8 channelCount;
	flowControlMsgChannelStatusField_t channelStatusArray[32];
} flowControlMsgChannelStatus_t;
#pragma pack()

/******************************************************************************
 * Public Functions
 *****************************************************************************/

/**
 * @details This function initializes flow control
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 *
 * @returns                   - status:
 *                              fs3ipc_StatusType_OK = channel available
 *                              fs3ipc_StatusType_ErrorHandle = general error
 */
fs3ipc_StatusType fs3ipc_flowcontrol_Init(
	const fs3ipc_flowCtrlLTCfgType *appCfg)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;

	if (appCfg)
	{
		fs3ipc_flowCtrlChanPBCfgType *chanCfg;
		fs3ipc_flowCtrlPBCfgType *cfg = appCfg->pbStConfig;
		fs3ipc_u8 channel = appCfg->channelCount;

		fs3ipc_os_GetResource(appCfg->resLock);
		fs3ipc_memset(cfg, 0, sizeof(fs3ipc_flowCtrlPBCfgType));

		for (channel = 0; channel < appCfg->channelCount; channel++)
		{
			chanCfg = &appCfg->pbStChanConfig[channel];
			fs3ipc_memset(chanCfg, 0, sizeof(fs3ipc_flowCtrlChanPBCfgType));
		}

		/* flow control is always enabled*/
		chanCfg = &appCfg->pbStChanConfig[FS3IPC_APP_FLOWCONTROL_CHAN];
		chanCfg->rxEnabled = FS3IPC_TRUE;

		/* Set initial flow control tx pending*/
		cfg->pendingTxRequest = FS3IPC_TRUE;
		cfg->loggingEnabled = FS3IPC_FALSE;

		fs3ipc_os_ReleaseResource(appCfg->resLock);
	}
	else
	{
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}

	return stat;
}

/**
 * @details This function returns if a flow control message is pending transmit
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 *
 * @returns                   - 1 = true; 0 = false
 */
fs3ipc_u8 fs3ipc_flowControl_MsgPending(const fs3ipc_flowCtrlLTCfgType *appCfg)
{
	fs3ipc_u8 ret = FS3IPC_FALSE;

	if (appCfg != FS3IPC_NULL)
	{
		fs3ipc_flowCtrlPBCfgType *appPbCfg = appCfg->pbStConfig;

		if ((appPbCfg != FS3IPC_NULL) &&
			 ((appPbCfg->pendingRxReply > 0) || (appPbCfg->pendingTxRequest > 0)))
		{
			ret = FS3IPC_TRUE;
		}
		else
		{
			/* nothing to do*/
		}
	}
	else
	{
		/* nothing to do*/
		LOG_ERROR("Invalid ccb ptr");
	}
	return ret;
}

/**
 * @details This function returns the size of a flow control message
 *
 *
 * @param[in]  channelCount   - channel count
 *
 * @returns                   - size of flow control message
 */
fs3ipc_u32 fs3ipc_flowControl_MsgSize(fs3ipc_u8 channelCount)
{
	return CHANNEL_STATUS_HDR_SIZE + (sizeof(flowControlMsgChannelStatusField_t)
		* channelCount);
}

/**
 * @details This function decodes a flow control message
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  data           - input buffer
 * @param[in]  length         - buffer size
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = channel available
 *                              fs3ipc_StatusType_ChannelNotAvailable = channel
 *                                 not available
 *                              fs3ipc_StatusType_ErrorGen = general error
 */
fs3ipc_StatusType fs3ipc_flowControl_Decode(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_cDataPtr data,
	fs3ipc_length length)
{
	/* this should be called in a critical section*/

	fs3ipc_StatusType status = fs3ipc_StatusType_OK;

	if (appCfg == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}
	else
	{
		uint8_t channelCount = appCfg->channelCount;

		const flowControlMsgHeader_t *hdr =
			(const flowControlMsgHeader_t *)&data[0];

		LOG_DATA(appCfg->pbStConfig->loggingEnabled, &data[0], length,
			LOG_DATA_MAX_BYTES, "IPC Rx C%02d L%03d",
			FS3IPC_APP_FLOWCONTROL_CHAN, length);
		if (length < sizeof(flowControlMsgHeader_t))
		{
			LOG_ERROR("buffer to small:%d", length);
		}
		else if (hdr->protocolVersion != PROTOCOL_VERSION_ID_EXP)
		{
			LOG_ERROR("Wrong protocol version:%02X", hdr->protocolVersion);
		}
		else
		{
			switch (hdr->msgId)
			{
			/* update */
			case MSG_ID_CHANNEL_STATUS_REQUEST:
				/* deliberate fall-through. no break*/

			case MSG_ID_CHANNEL_STATUS_REPLY:
			{
				const flowControlMsgChannelStatus_t *channelStatusMsg =
					 (const flowControlMsgChannelStatus_t *)hdr;

				if (length < CHANNEL_STATUS_HDR_SIZE)
				{
					status = fs3ipc_StatusType_ErrorGen;
					LOG_ERROR("buffer to small:%d", length);
				}
				else if (channelStatusMsg->channelCount >
					FS3_IPC_APP_MAX_NUMBER_CHANNELS)
				{
					status = fs3ipc_StatusType_ErrorGen;
					LOG_ERROR("Invalid channel count:%d",
						channelStatusMsg->channelCount);
				}
				else
				{
#if FS3IPC_APP_STATS_ENABLED == 1
					appCfg->pbStChanConfig[FS3IPC_APP_FLOWCONTROL_CHAN].rxCnt++;
#endif

					/* validate size of message*/
					if (length < (CHANNEL_STATUS_HDR_SIZE +
						(channelStatusMsg->channelCount *
						sizeof(flowControlMsgChannelStatusField_t))))
					{
						status = fs3ipc_StatusType_ErrorGen;
						LOG_ERROR("buffer to small:%d", length);
					}
					else
					{
						fs3ipc_u8 i = 0;

						/* clamp channel*/
						channelCount = (channelCount <
											 channelStatusMsg->channelCount)
												 ? channelCount
												 : channelStatusMsg->channelCount;

						for (i = 0; i < channelStatusMsg->channelCount; i++)
						{
							fs3ipc_flowCtrlChanPBCfgType *chanPbConfig =
								 &appCfg->pbStChanConfig[i];
							fs3ipc_u8 channelStatus;
							fs3ipc_u8 newChannelStatus;

							fs3ipc_os_GetResource(appCfg->resLock);

							channelStatus = chanPbConfig->txEnabled;
							newChannelStatus =
								channelStatusMsg->channelStatusArray[i].channelEnabled;
							chanPbConfig->txEnabled = newChannelStatus;
							chanPbConfig->maxTxPerFrame =
								channelStatusMsg->channelStatusArray[i].
									maximumBytesPerFrame;
							fs3ipc_os_ReleaseResource(appCfg->resLock);

							/* call channel callback*/
							if (channelStatus != newChannelStatus)
							{

#if FS3IPC_FC_LOG_EXT_CLIENT_STATUS == 1
								if (newChannelStatus)
								{
									LOG_NOTICE("Channel %d Available", i);
								}
								else
								{
									LOG_NOTICE("Channel %d Unavailable", i);
								}
#endif //FS3IPC_FC_LOG_EXT_CLIENT_STATUS == 1

								if (appCfg->ltStChanConfig[i].rxCallback != FS3IPC_NULL)
								{
									const fs3ipc_fp_flowControlCb rxCbFn = GET_RX_FC_CB(appCfg, i);
									rxCbFn(newChannelStatus, channelStatus);
								}
							}
						}
					}
				}
				break;
			}
			default:
				LOG_ERROR("Invalid MSG ID:%02Xh", hdr->msgId);
				status = fs3ipc_StatusType_ErrorGen;
				break;
			}
		}
	}

	return status;
}

/**
 * @details This function encodes a flow control message
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[out] data           - buffer
 * @param[in/out] length      - input: buffer size; output: encoded size
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = channel available
 *                              fs3ipc_StatusType_ChannelNotAvailable = channel
 *                                 not available
 *                              fs3ipc_StatusType_ErrorGen = general error
 */
fs3ipc_StatusType fs3ipc_flowControl_Encode(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_dataPtr data,
	fs3ipc_length *length)
{
	fs3ipc_StatusType status = fs3ipc_StatusType_OK;

	if (appCfg == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}
	else if (data == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null data ptr arg");
	}
	else if (length == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null length arg ptr");
	}
	else
	{
		fs3ipc_flowCtrlPBCfgType *pbCfg = appCfg->pbStConfig;
		flowControlMsgChannelStatus_t *flowCtrlMsg =
			(flowControlMsgChannelStatus_t *)data;
		fs3ipc_u8 pendingRxReply = pbCfg->pendingRxReply;
		fs3ipc_u8 pendingTxRequest = pbCfg->pendingTxRequest;
		fs3ipc_u32 msgSize = CHANNEL_STATUS_HDR_SIZE +
			(sizeof(flowControlMsgChannelStatusField_t) * appCfg->channelCount);

		if (*length < msgSize)
		{
			status = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("buffer to small:%d", *length);
			*length = 0;
		}

		else if (pendingRxReply || pendingTxRequest)
		{
			fs3ipc_u32 chan;
			pbCfg->pendingRxReply = FS3IPC_FALSE;
			pbCfg->pendingTxRequest = FS3IPC_FALSE;

#if FS3IPC_APP_STATS_ENABLED == 1
			appCfg->pbStChanConfig[FS3IPC_APP_FLOWCONTROL_CHAN].txCnt++;
#endif

			flowCtrlMsg->header.protocolVersion = PROTOCOL_VERSION_ID_EXP;

			/* pending reply takes priority. This will minimize the amount of
			 *flow control traffic
			 */
			if (pendingRxReply != FS3IPC_FALSE)
			{
				flowCtrlMsg->header.msgId = MSG_ID_CHANNEL_STATUS_REPLY;
			}
			else if (pendingTxRequest != FS3IPC_FALSE)
			{
				flowCtrlMsg->header.msgId = MSG_ID_CHANNEL_STATUS_REQUEST;
			}
			else
			{
			}

			fs3ipc_os_GetResource(appCfg->resLock);

			flowCtrlMsg->channelCount = appCfg->channelCount;

			for (chan = 0; chan < appCfg->channelCount; chan++)
			{
				if (chan != FS3IPC_APP_FLOWCONTROL_CHAN)
				{
					fs3ipc_flowCtrlChanPBCfgType *chanPBCfg =
						&appCfg->pbStChanConfig[chan];
					flowCtrlMsg->channelStatusArray[chan].channelEnabled =
						((chanPBCfg->rxEnabled == FS3IPC_TRUE) &&
						(chanPBCfg->rxOverflow == FS3IPC_FALSE));

					flowCtrlMsg->channelStatusArray[chan].maximumBytesPerFrame =
						appCfg->ltStChanConfig[chan].maxRxPerFrame;
				}
				else
				{
					flowCtrlMsg->channelStatusArray[chan].channelEnabled =
						FS3IPC_TRUE;
					flowCtrlMsg->channelStatusArray[chan].maximumBytesPerFrame =
						FS3_IPC_MAX_MSG_SIZE;
				}
			}

			fs3ipc_os_ReleaseResource(appCfg->resLock);

			LOG_DATA(pbCfg->loggingEnabled, &data[0], msgSize,
				LOG_DATA_MAX_BYTES, "IPC Tx C%02d L%03d",
				FS3IPC_APP_FLOWCONTROL_CHAN, msgSize);
			*length = msgSize;
		}
		else
		{
			/* nothing to do*/
			*length = 0;
		}
	}

	return status;
}

/**
 * @details This function returns if a channel is available for transmit
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = channel available
 *                              fs3ipc_StatusType_ChannelNotAvailable: channel
 *                                 not available
 */
fs3ipc_StatusType fs3ipc_flowControl_GetTxEnabled(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel)
{
	fs3ipc_StatusType status = fs3ipc_StatusType_OK;

	if (appCfg && channel < appCfg->channelCount)
	{

		if (appCfg->pbStChanConfig[channel].txEnabled == FS3IPC_FALSE)
		{
			status = fs3ipc_StatusType_ChannelNotAvailable;
		}
		else
		{
			/* do nothing. Channel is enabled*/
		}
	}
	else
	{
		status = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}

	return status;
}

/**
 * @details This function gets the resets the number of bytes transmitted for
 * all logical channels
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorHandle = invalid handle
 */
fs3ipc_StatusType fs3ipc_flowControl_ResetByteCountTxFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg)
{
	fs3ipc_StatusType status = fs3ipc_StatusType_OK;
	fs3ipc_u8 i = 0;

	if (appCfg)
	{
		/* no need to syncrhonize. byteCountTxFrame is only set by a single
		thread */
		for (i = 0; i < appCfg->channelCount; i++)
		{
			appCfg->pbStChanConfig[i].byteCountTxFrame = 0;
		}
	}
	else
	{
		status = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}

	return status;
}

/**
 * @details This function gets the number of bytes transmitted for a given
 * logical channel, per frame
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 * @param[in]  output         - number of bytes transmitted
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorGen: general error
 *                              fs3ipc_StatusType_ErrorHandle = invalid handle
 */
fs3ipc_StatusType fs3ipc_flowControl_GetFreeByteCountTxFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel,
	fs3ipc_length *output)
{
	fs3ipc_StatusType status = fs3ipc_StatusType_OK;

	if (appCfg == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}
	else if (channel >= appCfg->channelCount)
	{
		status = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Invalid channel number:%d", channel);
	}
	else if (output == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null output ptr arg");
	}
	else
	{
		fs3ipc_flowCtrlChanPBCfgType *pbCfg = &appCfg->pbStChanConfig[channel];

		if (pbCfg->maxTxPerFrame > pbCfg->byteCountTxFrame)
		{
			*output = pbCfg->maxTxPerFrame - pbCfg->byteCountTxFrame;
		}
		else
		{
			*output = 0;
		}
	}

	return status;
}

/**
 * @details This function increments the number of bytes which have been
 * transmitted to a given client per frame.
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 * @param[in]  byte           - number of bytes transmitted
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorGen: general error
 *                              fs3ipc_StatusType_ErrorHandle = invalid handle
 */
fs3ipc_StatusType fs3ipc_flowControl_AddByteCountTxFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel,
	fs3ipc_length byteCount)
{
	fs3ipc_StatusType status = fs3ipc_StatusType_OK;

	if (appCfg == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}
	else if (channel >= appCfg->channelCount)
	{
		status = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Invalid channel number:%d", channel);
	}
	else
	{
		fs3ipc_flowCtrlChanPBCfgType *pbCfg = &appCfg->pbStChanConfig[channel];

		pbCfg->byteCountTxFrame += byteCount;
	}

	return status;
}

/**
 * @details This function sets a queue overflow state. This queues a flowcontrol
 *  message informing the external client not to send any additional message for
 *  a given client.
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 * @param[in]  value          - 0 = overflow inactive; 1 = overflow active
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorGen: general error
 *                              fs3ipc_StatusType_ErrorHandle = invalid handle
 */
fs3ipc_StatusType fs3ipc_flowControl_SetOverflow(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel,
	fs3ipc_u8 value)
{
	fs3ipc_StatusType status = fs3ipc_StatusType_OK;

	if (appCfg == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}
	else if (channel >= appCfg->channelCount)
	{
		status = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Invalid channel number:%d", channel);
	}
	else
	{
		fs3ipc_u8 currentStat;

		fs3ipc_os_GetResource(appCfg->resLock);
		currentStat = appCfg->pbStChanConfig[channel].rxOverflow;
		value = (value == FS3IPC_FALSE ? FS3IPC_FALSE : FS3IPC_TRUE);

		if (value != currentStat)
		{
			appCfg->pbStChanConfig[channel].rxOverflow = value;
			appCfg->pbStConfig->pendingTxRequest = FS3IPC_TRUE;

#if FS3IPC_APP_STATS_ENABLED == 1
			if (value == FS3IPC_TRUE)
			{
				appCfg->pbStChanConfig[channel].XoffCnt++;
			}
#endif
		}
		fs3ipc_os_ReleaseResource(appCfg->resLock);
	}

	return status;
}

/**
 * @details This function gets the maximum number of bytes that can be
 *  transmitted per frame for a given client, for the external node.
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 *
 * @returns                   - number of bytes. 0 = not available/error.
 */
fs3ipc_u32 fs3ipc_flowControl_GetMaxTxPerFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel)
{
	fs3ipc_u32 size = 0;

	if (appCfg == FS3IPC_NULL)
	{
		LOG_ERROR("Invalid ccb ptr");
	}
	else if (channel >= appCfg->channelCount)
	{
		LOG_ERROR("Invalid channel number:%d", channel);
	}
	else
	{
		size = appCfg->pbStChanConfig[channel].maxTxPerFrame;
	}

	return size;
}

/**
 * @details This function gets the maximum number of bytes that can be
 *  received per frame for a given client, for the internal node.
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 *
 * @returns                   - number of bytes. 0 = not available/error.
 */
fs3ipc_u32 fs3ipc_flowControl_GetMaxRxPerFrame(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel)
{
	fs3ipc_u32 size = 0;

	if (appCfg == FS3IPC_NULL)
	{
		LOG_ERROR("Invalid ccb ptr");
	}
	else if (channel >= appCfg->channelCount)
	{
		LOG_ERROR("Invalid channel number:%d", channel);
	}
	else
	{
		size = appCfg->ltStChanConfig[channel].maxRxPerFrame;
	}

	return size;
}

/**
 * @details This function schedules a flow control message to inform the
 * external client that a given channel is ready or not to receive.
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 * @param[in]  enabled        - 0 = disabled; 1 = enabled
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorGen: general error
 *                              fs3ipc_StatusType_ErrorClientState: client is
 *                                 already in requested state.
 *                              fs3ipc_StatusType_ErrorHandle = invalid handle
 */
fs3ipc_StatusType fs3ipc_flowControl_EnableRxClient(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel,
	fs3ipc_u8 enabled)
{
	fs3ipc_StatusType status = fs3ipc_StatusType_OK;

	if (appCfg == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}
	else if (channel >= appCfg->channelCount)
	{
		status = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Invalid channel number:%d", channel);
	}
	else
	{
		fs3ipc_u8 currentStat;

		fs3ipc_os_GetResource(appCfg->resLock);
		currentStat = appCfg->pbStChanConfig[channel].rxEnabled;
		enabled = enabled == FS3IPC_FALSE ? FS3IPC_FALSE : FS3IPC_TRUE;

		if (enabled != currentStat)
		{
			appCfg->pbStConfig->pendingTxRequest = FS3IPC_TRUE;

			appCfg->pbStChanConfig[channel].rxEnabled = enabled;
		}
		else
		{
			if (currentStat)
			{
				LOG_ERROR("Channel %d already open", channel);
			}
			else
			{
				LOG_ERROR("Channel %d not open", channel);
			}

			/* client already enabled. only one client allowed*/
			status = fs3ipc_StatusType_ErrorClientState;
		}

		fs3ipc_os_ReleaseResource(appCfg->resLock);
	}

	return status;
}

/**
 * @details This function clears data related to an external node, and should
 *  called upon reset.
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorHandle = operation not
 *                                 successful
 */
fs3ipc_StatusType fs3ipc_flowcontrol_HandleNodeReset(
	 const fs3ipc_flowCtrlLTCfgType *appCfg)
{
	fs3ipc_StatusType status = fs3ipc_StatusType_OK;

	if (appCfg == FS3IPC_NULL)
	{
		status = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid ccb ptr");
	}

	if (status == fs3ipc_StatusType_OK)
	{
		fs3ipc_flowCtrlPBCfgType *pbcfg = appCfg->pbStConfig;
		fs3ipc_u8 cnt = appCfg->channelCount;

		while (cnt-- > 0)
		{
			fs3ipc_flowCtrlChanPBCfgType *chcfg = &appCfg->pbStChanConfig[cnt];

			if (chcfg != FS3IPC_NULL)
			{
				fs3ipc_u8 myTxEnabled;

				fs3ipc_os_GetResource(appCfg->resLock);
				myTxEnabled = chcfg->txEnabled;

				if (myTxEnabled)
				{
					chcfg->XoffCnt++;
				}
				chcfg->txEnabled = 0;
				chcfg->maxTxPerFrame = 0;
				fs3ipc_os_ReleaseResource(appCfg->resLock);

				if (myTxEnabled != 0 && (appCfg->ltStChanConfig[cnt].rxCallback))
				{
					const fs3ipc_fp_flowControlCb rxCbFn = GET_RX_FC_CB(appCfg, cnt);
					rxCbFn(FS3IPC_FALSE, FS3IPC_TRUE);
				}
			}
		}

		pbcfg->pendingTxRequest = FS3IPC_TRUE;
	}
	return status;
}

/**
 * @details This function increments the number of messages received stats
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorHandle = operation not
 *                                 successful
 */
#if FS3IPC_APP_STATS_ENABLED == 1
fs3ipc_StatusType fs3ipc_flowControl_IncRxMsgCnt(const fs3ipc_flowCtrlLTCfgType *appCfg,
																 fs3ipc_u8 channel)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_ErrorGen;
	if (appCfg && channel < appCfg->channelCount)
	{
		fs3ipc_flowCtrlChanPBCfgType *chanCfg = &appCfg->pbStChanConfig[channel];
		chanCfg->rxCnt++;
		ret = fs3ipc_StatusType_OK;
	}

	return ret;
}

/**
 * @details This function increments the number of messages transmitted stats
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorHandle = operation not
 *                                 successful
 */
fs3ipc_StatusType fs3ipc_flowControl_IncTxMsgCnt(const fs3ipc_flowCtrlLTCfgType *appCfg,
																 fs3ipc_u8 channel)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_ErrorGen;
	if (appCfg && channel < appCfg->channelCount)
	{
		fs3ipc_flowCtrlChanPBCfgType *chanCfg = &appCfg->pbStChanConfig[channel];
		chanCfg->txCnt++;
		ret = fs3ipc_StatusType_OK;
	}

	return ret;
}

/**
 * @details This function returns flow control statistics
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 *
 * @returns                   - if error, NULL is returned; else, pointer to
 *                              structured statistics data
 */
const fs3ipc_flowCtrlChanPBCfgType *fs3ipc_flowcontrol_GetStats(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel)
{
	fs3ipc_flowCtrlChanPBCfgType *ret = FS3IPC_NULL;
	if (appCfg && channel < appCfg->channelCount)
	{
		ret = &appCfg->pbStChanConfig[channel];
	}
	return ret;
}

/**
 * @details This function resets flow control statistics buffer
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  channel        - logical channel number
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorHandle = operation not
 *                                 successful
 */
fs3ipc_StatusType fs3ipc_flowcontrol_ClearStats(
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 channel)
{

	fs3ipc_StatusType ret = fs3ipc_StatusType_ErrorGen;
	if (appCfg && channel < appCfg->channelCount)
	{
		fs3ipc_flowCtrlChanPBCfgType *stat = &appCfg->pbStChanConfig[channel];

		stat->rxCnt = 0;
		stat->txCnt = 0;
		stat->XoffCnt = 0;

		ret = fs3ipc_StatusType_OK;
	}
	return ret;
}
#endif

/**
 * @details This function enables message logging for the flow control channel
 *
 *
 * @param[in]  appCfg         - pointer to flow control config block
 * @param[in]  enabled        - 0 = disable logging; 1 = enable logging
 *
 * @returns                   - Status:
 *                              fs3ipc_StatusType_OK = operation successful
 *                              fs3ipc_StatusType_ErrorHandle = operation not
 *                                 successful
 */
fs3ipc_StatusType fs3ipc_flowcontrol_SetLogging (
	const fs3ipc_flowCtrlLTCfgType *appCfg,
	fs3ipc_u8 enabled)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;
	if (!appCfg)
	{
		ret = fs3ipc_StatusType_ErrorHandle;
	}
	if (ret == fs3ipc_StatusType_OK)
	{
		if (enabled)
		{
			appCfg->pbStConfig->loggingEnabled = FS3IPC_TRUE;
		}
		else
		{
			appCfg->pbStConfig->loggingEnabled = FS3IPC_TRUE;
		}
	}
	return ret;
}