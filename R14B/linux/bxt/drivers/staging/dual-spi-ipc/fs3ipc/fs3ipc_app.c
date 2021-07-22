/******************************************************************
 *  Project          Harman Car Multimedia System
 *  (c) copyright    2018
 *  Company          Harman International Industries, Incorporated
 *                   All rights reserved
 *  Secrecy Level    STRICTLY CONFIDENTIAL
 ******************************************************************/
/**
 *  @file            fs3ipc_app.c
 *  @ingroup         fs3ipc Component
 *  @author          David Rogala
 *  @brief
 */

#include "fs3ipc_cfg.h"
#include "fs3ipc_app.h"

#ifdef LOG_MODULE
#undef LOG_MODULE
#endif
#define LOG_MODULE "fs3ipc_app"

/* increment statistics macro*/
#if FS3IPC_APP_STATS_ENABLED == 1
#define INC_STATS(var) (var)++
#else
#define INC_STATS(var)
#endif

/******************************************************************************
 * External Variables
 *****************************************************************************/
extern const fs3ipc_app_LTconfig fs3ipc_app_appConfigs[];
/* logical channel lookup. Should be size FS3IPC_APP_MAX_NUM_LOGICAL_CHANNEL
 * (32)*/
extern fs3ipc_app_clientChannelLookupType fs3_ipc_clientChannelLookupTable[];

/******************************************************************************
 * Private Functions Prototypes
 *****************************************************************************/
static fs3ipc_u32 handleEncodeResumeMessage(fs3ipc_dataPtr out,
	fs3ipc_s32 outLength, fs3ipc_app_messageBufferType *partialMessage,
	const fs3ipc_flowCtrlLTCfgType *fcCfg);

static fs3ipc_u32 handleEncodeFlowControlMessage(fs3ipc_dataPtr out,
	fs3ipc_s32 outLength, fs3ipc_app_messageBufferType *partialMessage,
	const fs3ipc_app_LTconfig *cfg);

static fs3ipc_u32 handleEncodeNextMessage(fs3ipc_dataPtr out,
	fs3ipc_s32 outLength, fs3ipc_app_messageBufferType *partialMessage,
	const fs3ipc_app_LTChannelCfgType *channelCfg,
	const fs3ipc_app_LTconfig *cfg);

static fs3ipc_u32 handleDecodeResumeMessage(fs3ipc_cDataPtr in,
	fs3ipc_length outLength, const fs3ipc_app_LTconfig *cfg,
	fs3ipc_StatusType *stat);

static fs3ipc_u32 handleDecodeNewMessage(fs3ipc_cDataPtr in,
	fs3ipc_length outLength, const fs3ipc_app_LTconfig *cfg,
	fs3ipc_StatusType *stat);

/******************************************************************************
 * Private Functions
 *****************************************************************************/

/**
 * @details This function resumes/encodes general information to send
 *
 *
 * @param[out] out            - Pointer used to copy the transmitted data
 * @param[in]  outLength      - The total bytes to transmit
 * @param[out] partialMessage - The partial message to decode
 * @param[in]  fcCfg          - Application configuration parameter
 *
 * @returns                   - the total bytes copied/encoded
 */
static fs3ipc_u32 handleEncodeResumeMessage(fs3ipc_dataPtr out,
	fs3ipc_s32 outLength, fs3ipc_app_messageBufferType *partialMessage,
	const fs3ipc_flowCtrlLTCfgType *fcCfg)
{
	fs3ipc_app_messageHdr *resumeMsgHdr = (fs3ipc_app_messageHdr *)out;
	fs3ipc_length bytesCoppied;

	/* skip the resume message header*/
	if (partialMessage->position > 0)
	{
		fs3ipc_length dataSize = partialMessage->message.hdr.length -
			partialMessage->position;
		fs3ipc_dataPtr inData =
			&partialMessage->message.data[partialMessage->position];
		fs3ipc_u8 msgOverflow;

		LOG_DEBUG("Partial Msg %d", partialMessage->position);

		/* skip past the resume message header and update remaining buffer
		 * length*/
		out += sizeof(fs3ipc_app_messageHdr);
		outLength -= sizeof(fs3ipc_app_messageHdr);

		/* does message fit in the amount of remaining buffer?*/
		msgOverflow = (dataSize > outLength);

		if (msgOverflow)
		{
			LOG_DEBUG("Msg wrap");
			bytesCoppied = outLength;
		}
		else //!msgOverflow
		{
			bytesCoppied = dataSize;
		}

		/* write the resume header */
		resumeMsgHdr->length = dataSize;
		resumeMsgHdr->channel = partialMessage->message.hdr.channel;

		fs3ipc_memcpy(&out[0], &inData[0], bytesCoppied);

		if (msgOverflow)
		{
			partialMessage->position += outLength;
		}
		else //(index < length)
		{
			fs3ipc_app_messageType *msg = &partialMessage->message;

			/* if the message is not a flow control message, inform the flow
			 * control module of the number of bytes transmitted*/
			if (msg->hdr.channel != FS3IPC_APP_FLOWCONTROL_CHAN)
			{
				(void)fs3ipc_flowControl_AddByteCountTxFrame(fcCfg,
					msg->hdr.channel, msg->hdr.length + FS3IPC_APP_QUEUE_HDR_SIZE);
			}

			/* message has been completed. clear partial message struct*/
			partialMessage->position = 0;
			msg->hdr.channel = 0;
			msg->hdr.length = 0;
		}
		bytesCoppied += sizeof(fs3ipc_app_messageHdr);
	}
	else
	{
		LOG_DEBUG("No Partial Msg");
		resumeMsgHdr->length = 0;
		resumeMsgHdr->channel = 0;
		bytesCoppied = sizeof(fs3ipc_app_messageHdr);
	}

	LOG_DEBUG("%d bytes consumed", bytesCoppied);

	return bytesCoppied;
}

/**
 * @brief  This function resumes/encodes flowcontrol message
 *
 * @param[out] out            - Pointer used to copy the transmitted data
 * @param[in]  outLength      - The total bytes to transmit
 * @param[out] partialMessage - The partial message to decode
 * @param[in]  Cfg            - Application configuration parameter
 *
 * @returns                   - the total bytes copied/encoded
 */
static fs3ipc_u32 handleEncodeFlowControlMessage(fs3ipc_dataPtr out,
	fs3ipc_s32 outLength, fs3ipc_app_messageBufferType *partialMessage,
	const fs3ipc_app_LTconfig *cfg)
{
	fs3ipc_length fcLength;
	fs3ipc_StatusType fcStat;
	fs3ipc_app_messageType *outPtr;
	fs3ipc_length bytesTransferred = 0;

	if (fs3ipc_flowControl_MsgPending(cfg->flowControlCfg) == FS3IPC_TRUE)
	{
		fs3ipc_u8 msgOverflow =
			(fs3ipc_flowControl_MsgSize(cfg->flowControlCfg->channelCount) +
			sizeof(fs3ipc_app_messageHdr) > outLength);

		/* see which buffer should be used*/
		if (!msgOverflow)
		{
			outPtr = (fs3ipc_app_messageType *)out;
			fcLength = outLength - sizeof(fs3ipc_app_messageHdr);
		}
		else
		{
			outPtr = &partialMessage->message;
			fcLength = FS3_IPC_MAX_MSG_SIZE;
		}

		fcStat = fs3ipc_flowControl_Encode(cfg->flowControlCfg,
													  &outPtr->data[0],
													  &fcLength);

		if (fcStat == fs3ipc_StatusType_OK && fcLength != 0)
		{
			outPtr->hdr.length = fcLength;
			outPtr->hdr.channel = FS3IPC_APP_FLOWCONTROL_CHAN;

			if (msgOverflow)
			{
				INC_STATS(cfg->stats->txWrappedMsgCnt);
				fs3ipc_memcpy(&out[0], (void *)outPtr, outLength);
				partialMessage->position += outLength -
					sizeof(fs3ipc_app_messageHdr);
				bytesTransferred = outLength;
			}
			else
			{
				bytesTransferred = fcLength + sizeof(fs3ipc_app_messageHdr);
			}
		}
		else
		{
			LOG_ERROR("fc encode:%d ", fcStat);
		}
	}
	else
	{
	}

	return bytesTransferred;
}

/**
 * @brief  this function encodes the next message
 *
 * @param[out] out            - Pointer used to copy the transmitted data
 * @param[out] outLength      - The total bytes to transmit
 * @param[out] partialMessage - The partial message to decode
 * @param[in]  fcCfg          - Application configuration parameter
 *
 * @returns       the total bytes copied/encoded
 *
 * @details    load news message from message queue
 */

static fs3ipc_u32 handleEncodeNextMessage(fs3ipc_dataPtr out,
	fs3ipc_s32 outLength, fs3ipc_app_messageBufferType *partialMessage,
	const fs3ipc_app_LTChannelCfgType *channelCfg,
	const fs3ipc_app_LTconfig *cfg)
{
	fs3ipc_length bytesCoppied = 0;
	fs3ipc_s32 qStat;
	fs3ipc_u32 outSize;
	fs3ipc_length freeByteTx = 0;
	fs3ipc_length maxTxMsg;
	fs3ipc_app_messageType *outPtr;

	/* make sure channel config is valid and and channel is enabled*/
	if ((channelCfg) && (fs3ipc_StatusType_OK ==
		fs3ipc_flowControl_GetTxEnabled(cfg->flowControlCfg,
		channelCfg->channel)))
	{
		fs3ipc_os_GetResource(channelCfg->TX_QLock);
		qStat = fs3ipc_app_QueueFrontSize(channelCfg->TX_Q);
		fs3ipc_os_ReleaseResource(channelCfg->TX_QLock);

		(void)fs3ipc_flowControl_GetFreeByteCountTxFrame(cfg->flowControlCfg,
			channelCfg->channel,
			&freeByteTx);

		maxTxMsg = fs3ipc_flowControl_GetMaxTxPerFrame(cfg->flowControlCfg,
			channelCfg->channel);
		LOG_DEBUG("Channel is enabled:%d", channelCfg->channel);

		if (qStat == fs3ipc_app_qStat_empty)
		{
			LOG_DEBUG("Tx Q empty channel:%d ", channelCfg->channel);
		}
		else if (maxTxMsg < 0)
		{
			INC_STATS(cfg->stats->txGenErrorCnt);
			LOG_ERROR("fs3ipc_flowControl_GetMaxTxPerFrame:%d chan:%02d",
				maxTxMsg, channelCfg->channel);
		}
		else if (qStat <= 0)
		{
			/* buffer empty or there is some other error*/
			INC_STATS(cfg->stats->txGenErrorCnt);
			LOG_ERROR("vlqueue_fsize error:%d chan:%02d",
				qStat, channelCfg->channel);
		}
		else if ((qStat + FS3IPC_APP_QUEUE_HDR_SIZE) > maxTxMsg)
		{
			/* message is larger than maximum size. This should never happen if it
			does, the application is misbehaving and
			FS3IPC_APP_ALLOW_MSG_Q_WO_FC_CHECK == 1.
			*/
			INC_STATS(cfg->stats->txGenErrorCnt);
			LOG_ERROR("CRITICAL! Chan %d blocked; msg too large %d - %d expected",
				channelCfg->channel, qStat, maxTxMsg);

			/* pop message to unused tx partial message buffer to free the buffer*/
			fs3ipc_app_QueuePop (channelCfg->TX_Q,
				&partialMessage->message.data[0], FS3_IPC_MAX_MSG_SIZE);
			partialMessage->position = 0;
		}
		else if ((qStat + FS3IPC_APP_QUEUE_HDR_SIZE) > freeByteTx)
		{
			/* Too many bytes have been transmitted for
			 * the given channel and frame */
			LOG_DEBUG("Tx Channel:%d throttled. overflow:%d ",
				channelCfg->channel,
				qStat - freeByteTx);
		}
		else /* message is valid size*/
		{
			fs3ipc_u8 msgOverflow = (qStat + sizeof(fs3ipc_app_messageHdr) >
				(outLength));

			LOG_DEBUG("Valid Msg Size:%d ", qStat);

			/* see which buffer should be used*/
			if (!msgOverflow)
			{
				LOG_DEBUG("msg complete");
				outPtr = (fs3ipc_app_messageType *)(&out[0]);
				outSize = outLength - sizeof(fs3ipc_app_messageHdr);
			}
			else //msgOverflow == TRUE
			{
				LOG_DEBUG("msg wrapped");
				outPtr = &partialMessage->message;
				outSize = FS3_IPC_MAX_MSG_SIZE;
			}

			fs3ipc_os_GetResource(channelCfg->TX_QLock);
			qStat = fs3ipc_app_QueuePop(channelCfg->TX_Q,
				&outPtr->data[0], outSize);
			fs3ipc_os_ReleaseResource(channelCfg->TX_QLock);

			if (qStat > fs3ipc_app_qStat_OK)
			{
				/* message transmitted, increment stats*/
				fs3ipc_flowControl_IncTxMsgCnt(cfg->flowControlCfg,
					channelCfg->channel);

				LOG_DEBUG("vlqueue_pop OK");

				outPtr->hdr.channel = channelCfg->channel;
				outPtr->hdr.length = qStat;

				if (msgOverflow)
				{
					INC_STATS(cfg->stats->txWrappedMsgCnt);
					fs3ipc_memcpy(&out[0], (void *)outPtr, outLength);
					partialMessage->position += outLength -
						sizeof(fs3ipc_app_messageHdr);
					bytesCoppied = outLength;
				}
				else
				{
					//fs3ipc_app_messageType* msg = &partialMessage->message;

					/* if the message is not a flow control message, inform the flow
					 * control module of the number of bytes transmitted*/
					if (outPtr->hdr.channel != FS3IPC_APP_FLOWCONTROL_CHAN)
					{
						LOG_DEBUG("FC Msg");
						(void)fs3ipc_flowControl_AddByteCountTxFrame(
							cfg->flowControlCfg, outPtr->hdr.channel,
							qStat + FS3IPC_APP_QUEUE_HDR_SIZE);
					}

					bytesCoppied = qStat + sizeof(fs3ipc_app_messageHdr);
				}
			}
			else if (qStat == fs3ipc_app_qStat_empty)
			{
				/* Queue is empty. continue on to next message*/
				LOG_DEBUG("Q Emtpy chan:%d", channelCfg->channel);
			}

			/* message does not fit into buffer*/
			else
			{
				/* error - VLQ call failed. This should never happen*/
				INC_STATS(cfg->stats->txGenErrorCnt);
				LOG_ERROR("vlqueue_pop:%d", qStat);
			}
		}
	}
	else
	{
		LOG_DEBUG("Channel not enabled:%d", channelCfg->channel);
	}

	LOG_DEBUG("Bytes copied :%d", bytesCoppied);

	return bytesCoppied;
}

/**
 * @brief this function resumes the message
 *
 * @param[in]  in         - buffer to decode the data
 * @param[in]  outLength  - remaining bytes to process
 * @param[in]  cfg        - provides the Application Configuration Parameter
 * @param[out] stat       - returns the result of the function call
 *
 * @returns    the number of bytes processed
 *
 * @details this function verifies the correct configuration parameters, it
 *           resumes the decoding operation when message is correct, the message
 *           is written to the queue
 */

static fs3ipc_u32 handleDecodeResumeMessage(fs3ipc_cDataPtr in,
	fs3ipc_length outLength, const fs3ipc_app_LTconfig *cfg,
	fs3ipc_StatusType *stat)
{
	fs3ipc_StatusType myStatus = fs3ipc_StatusType_OK;
	fs3ipc_app_messageBufferType *partialMessage = cfg->rxMsg;
	const fs3ipc_length rxResumeMsgSize =
		((const fs3ipc_app_messageHdr *)in)->length;
	fs3ipc_s32 storedMsgRemSize = partialMessage->message.hdr.length -
		partialMessage->position;
	fs3ipc_length bytesCoppied;

	bytesCoppied = ((const fs3ipc_app_messageHdr *)in)->length +
		sizeof(fs3ipc_app_messageHdr);
	if (bytesCoppied > outLength)
	{
		bytesCoppied = outLength;
	}

	/* no partial message encoded in current frame */
	if (rxResumeMsgSize == 0)
	{
		/* nothing to do, exit*/
		if (storedMsgRemSize != 0)
		{
			/* error - partial message discarded. erase message*/
			INC_STATS(cfg->stats->rxWrappedMsgErrorCnt);

			LOG_ERROR("Partial Msg Discarded. Not in Rx %d",
				storedMsgRemSize);

			fs3ipc_memset(partialMessage, 0,
				sizeof(fs3ipc_app_messageBufferType));
		}
	}

	/* stored message size does not match incoming partial message*/
	else if (storedMsgRemSize != rxResumeMsgSize)
	{
		/* error - stored message does not match incoming partial message
		 * discarded. erase message*/
		INC_STATS(cfg->stats->rxWrappedMsgErrorCnt);

		LOG_ERROR("Partial Msg Discarded. Size mismatch %d, %d",
			storedMsgRemSize, rxResumeMsgSize);

		fs3ipc_memset(partialMessage, 0, sizeof(fs3ipc_app_messageBufferType));
	}

	/* message appears to be good*/
	else
	{
		fs3ipc_dataPtr storedMsgData =
			&partialMessage->message.data[partialMessage->position];

		/* update in pointer to point to beginning of message*/
		in += sizeof(fs3ipc_app_messageHdr);
		outLength -= sizeof(fs3ipc_app_messageHdr);

		if (storedMsgRemSize <= outLength)
		{
			fs3ipc_u8 channel = partialMessage->message.hdr.channel;
			fs3ipc_length msgLength = partialMessage->message.hdr.length;
			const fs3ipc_app_LTChannelCfgType *ltChanCfg =
				cfg->ltChannelCfg[channel];

			/* copy data */
			fs3ipc_memcpy(storedMsgData, in, rxResumeMsgSize);

			if (channel == FS3IPC_APP_FLOWCONTROL_CHAN)
			{
				/* message fits in frame buffer. Add message to application Q*/
				(void)fs3ipc_flowControl_Decode(cfg->flowControlCfg,
					&partialMessage->message.data[0],
					partialMessage->message.hdr.length);
				/* wake up hdlc thread for any new messages to be sent*/
				(void)fs3ipc_os_SetEvent(cfg->phys_txEventOsCfg,
						cfg->phys_txEvent);
			}
			else if ((channel < cfg->channelCount) && (ltChanCfg))
			{
				fs3ipc_s32 qstat;
				LOG_DATA(*ltChanCfg->loggingEnabled,
					&partialMessage->message.data[0], msgLength,
					LOG_DATA_MAX_BYTES, "IPC Rx C%02d L%03d", channel, msgLength);

				fs3ipc_os_GetResource(ltChanCfg->RX_QLock);
				/* message fits in frame buffer. Add message to application Q*/
				qstat = fs3ipc_app_QueuePush(ltChanCfg->RX_Q,
					&partialMessage->message.data[0], msgLength);
				fs3ipc_os_ReleaseResource(ltChanCfg->RX_QLock);

				if (qstat == fs3ipc_app_qStat_OK)
				{
					fs3ipc_flowControl_IncRxMsgCnt(cfg->flowControlCfg, channel);

					fs3ipc_os_GetResource(ltChanCfg->RX_QLock);
					qstat = fs3ipc_app_QueueFreeSpace(ltChanCfg->RX_Q);
					fs3ipc_os_ReleaseResource(ltChanCfg->RX_QLock);

					if ((qstat > fs3ipc_app_qStat_OK) &&
						(qstat < (fs3ipc_flowControl_GetMaxRxPerFrame(
							cfg->flowControlCfg, channel) * 2)))
					{
						(void)fs3ipc_flowControl_SetOverflow(cfg->flowControlCfg,
							channel, FS3IPC_TRUE);

						if (fs3ipc_flowControl_MsgPending(cfg->flowControlCfg) !=
							FS3IPC_FALSE)
						{
							(void)fs3ipc_os_SetEvent(cfg->phys_txEventOsCfg,
								cfg->phys_txEvent);
							LOG_DEBUG("Pre-emptive off: event channel:%d", channel);
						}
					}

					/* set receive event */
					if (fs3ipc_StatusType_OK !=
						fs3ipc_os_SetEvent(ltChanCfg->clientCfg.osCfg,
						ltChanCfg->clientCfg.rxEvent))
					{
						LOG_ERROR("cannot set rx event");
					}

					fs3ipc_memset(partialMessage, 0,
						sizeof(fs3ipc_app_messageBufferType));
				}
				else if (qstat == fs3ipc_app_qStat_full)
				{
					/* message doesn't fit in current buffer*/
					myStatus = fs3ipc_StatusType_BufferFull;
					INC_STATS(cfg->stats->rxBufferOverflow);
					LOG_ERROR("Q Full. Dropped Message chan:%d len:%d", channel,
						msgLength);

					(void)fs3ipc_flowControl_SetOverflow(cfg->flowControlCfg,
						channel, FS3IPC_TRUE);

					if (fs3ipc_flowControl_MsgPending(cfg->flowControlCfg) !=
						FS3IPC_FALSE)
					{
						(void)fs3ipc_os_SetEvent(cfg->phys_txEventOsCfg,
							cfg->phys_txEvent);
					}
				}
				else
				{
					INC_STATS(cfg->stats->rxWrappedMsgErrorCnt);
					LOG_ERROR("Dropped Message chan:%d len:%d stat:%d", channel,
						msgLength, qstat);

					fs3ipc_memset(partialMessage, 0,
						sizeof(fs3ipc_app_messageBufferType));
				}
			}
			else
			{
				/* error - unhandled channel*/
				INC_STATS(cfg->stats->rxUnhandledChannelErrorCnt);
				LOG_ERROR("Partial Msg Discarded. Unhandled channel:%d",
					channel);

				fs3ipc_memset(partialMessage, 0,
					sizeof(fs3ipc_app_messageBufferType));
			}
		}
		else
		{
			INC_STATS(cfg->stats->rxWrappedMsgErrorCnt);
			LOG_DEBUG("Partial Msg Wrapped OF:%d",
				storedMsgRemSize - outLength);

			fs3ipc_memcpy(storedMsgData, in, outLength);
			/* complete message has not been received. update partial rx message
			 * buffer*/
			partialMessage->position += outLength;
		}
	}

	*stat = myStatus;

	return bytesCoppied;
}

/**
 * @brief this function decodes the new messages
 *
 * @param[in]   in        -  buffer to decode the data
 * @param[in]   outLength  - remaining bytes to process
 * @param[in]   cfg       - provides the Application Configuration Parameter
 * @param[out]  stat      - returns the result of the function call
 *
 * @returns     the number of bytes processed
 *
 * @details   This function validates the proper condition to decode new
 *            messages, it retrieves the Application layer configuration
 *            parameters.
 *
 */

static fs3ipc_u32 handleDecodeNewMessage(fs3ipc_cDataPtr in,
	fs3ipc_length outLength, const fs3ipc_app_LTconfig *cfg,
	fs3ipc_StatusType *stat)
{
	fs3ipc_StatusType myStatus = fs3ipc_StatusType_OK;
	fs3ipc_app_messageBufferType *partialMessage = cfg->rxMsg;
	fs3ipc_u8 msgChannel = ((const fs3ipc_app_messageHdr *)&in[0])->channel;
	fs3ipc_length msgLength = ((const fs3ipc_app_messageHdr *)&in[0])->length;
	fs3ipc_length bytesCoppied = msgLength + sizeof(fs3ipc_app_messageHdr);
	const fs3ipc_app_LTChannelCfgType *ltChanCfg = cfg->ltChannelCfg[msgChannel];

	/* return number of bytes for next message */
	if (bytesCoppied > outLength)
	{
		bytesCoppied = outLength;
	}

	if (msgLength == 0 || msgLength > FS3_IPC_MAX_MSG_SIZE)
	{
		/* nothing to send */
		INC_STATS(cfg->stats->rxMsgLengthErrorCnt);
		LOG_ERROR("Invalid Length Message %d", msgLength);
	}
	else if (msgChannel > cfg->channelCount)
	{
		/* error - invalid channel*/
		INC_STATS(cfg->stats->rxUnhandledChannelErrorCnt);
		LOG_ERROR("Invalid Channel:%d", msgChannel);
	}
	else //msgHdr->length != 0
	{
		if (msgChannel == FS3IPC_APP_FLOWCONTROL_CHAN)
		{
			/* flow control message - requires special handling*/
			if (msgLength + sizeof(fs3ipc_app_messageHdr) > outLength)
			{
				/* message does not fit in frame buffer. copy the partial message
				 * into the rx buffer*/
				INC_STATS(cfg->stats->rxWrappedMsgCnt);
				fs3ipc_memcpy(&partialMessage->message, in, outLength);
				partialMessage->position = outLength -
					sizeof(fs3ipc_app_messageHdr);
			}
			else
			{
				/* message fits in frame buffer. Add message to application Q*/
				(void)fs3ipc_flowControl_Decode(cfg->flowControlCfg,
					&((fs3ipc_app_messageType *)in)->data[0], msgLength);
				/* wake up hdlc thread for any new messages to be sent*/
				(void)fs3ipc_os_SetEvent(cfg->phys_txEventOsCfg,
						cfg->phys_txEvent);
			}
		}
		else if (ltChanCfg)
		{
			fs3ipc_app_queueType *rxQ = ltChanCfg->RX_Q;
			fs3ipc_s32 qstat;
			LOG_DATA(*ltChanCfg->loggingEnabled,
				&((fs3ipc_app_messageType *)in)->data[0], msgLength,
				LOG_DATA_MAX_BYTES, "IPC Rx C%02d L%03d", ltChanCfg->channel,
				msgLength);

			/* message fits in application Q*/
			if (msgLength + sizeof(fs3ipc_app_messageHdr) > outLength)
			{
				/* message does not fit in frame buffer. copy the partial message
				 * into the rx buffer*/
				INC_STATS(cfg->stats->rxWrappedMsgCnt);
				fs3ipc_memcpy(&partialMessage->message, in, outLength);
				partialMessage->position = outLength -
					sizeof(fs3ipc_app_messageHdr);
			}
			else
			{
				fs3ipc_os_GetResource(ltChanCfg->RX_QLock);
				/* message fits in frame buffer. Add message to application Q*/
				qstat = fs3ipc_app_QueuePush(rxQ,
					&((fs3ipc_app_messageType *)in)->data[0], msgLength);
				fs3ipc_os_ReleaseResource(ltChanCfg->RX_QLock);

				if (qstat == fs3ipc_app_qStat_OK)
				{
					fs3ipc_flowControl_IncRxMsgCnt(cfg->flowControlCfg, msgChannel);

					fs3ipc_os_GetResource(ltChanCfg->RX_QLock);
					qstat = fs3ipc_app_QueueFreeSpace(rxQ);
					fs3ipc_os_ReleaseResource(ltChanCfg->RX_QLock);

					if ((qstat > fs3ipc_app_qStat_OK) &&
						(qstat < (fs3ipc_flowControl_GetMaxRxPerFrame(
						cfg->flowControlCfg, msgChannel) * 2)))
					{
						(void)fs3ipc_flowControl_SetOverflow(cfg->flowControlCfg,
							msgChannel, FS3IPC_TRUE);

						if (fs3ipc_flowControl_MsgPending(cfg->flowControlCfg) !=
							 FS3IPC_FALSE)
						{
							(void)fs3ipc_os_SetEvent(cfg->phys_txEventOsCfg,
								cfg->phys_txEvent);
							LOG_DEBUG("Pre-emptive off: event channel:%d", msgChannel);
						}
					}

					/* set receive event */
					if (fs3ipc_StatusType_OK !=
						fs3ipc_os_SetEvent(ltChanCfg->clientCfg.osCfg,
						ltChanCfg->clientCfg.rxEvent))
					{
						LOG_ERROR("cannot set rx event");
					}
				}
				else if (qstat == fs3ipc_app_qStat_full)
				{
					/* message doesn't fit in current buffer*/
					myStatus = fs3ipc_StatusType_BufferFull;
					INC_STATS(cfg->stats->rxBufferOverflow);
					LOG_ERROR("Q Full. Dropped Message chan:%d len:%d", msgChannel,
						msgLength);

					(void)fs3ipc_flowControl_SetOverflow(cfg->flowControlCfg,
						msgChannel, FS3IPC_TRUE);

					if (fs3ipc_flowControl_MsgPending(cfg->flowControlCfg) !=
						FS3IPC_FALSE)
					{
						(void)fs3ipc_os_SetEvent(cfg->phys_txEventOsCfg,
							cfg->phys_txEvent);
					}
				}
				else
				{
					INC_STATS(cfg->stats->rxGenErrorCnt);
					LOG_ERROR("Dropped Message chan:%d len:%d stat:%d", msgChannel,
						msgLength, qstat);
				}
			}
		}
		else
		{
			INC_STATS(cfg->stats->rxUnhandledChannelErrorCnt);
			/* warning - invalid channel!. Continue parsing*/
			LOG_ERROR("Invalid channel chan:%d", msgChannel);
		}
	}

	*stat = myStatus;

	return bytesCoppied;
}

/**
 * @brief this funtion changes the channel status to enable/disable
 *
 * @param[out] clientHandle  - the client/channel to modify
 * @param[out] enable        - Enable/Disable
 *
 * @returns     -      fs3ipc_StatusType_OK else error condition
 *
 * @details This function sets/changes the channel stat value demanded by
 *          enable parameter
 */
static fs3ipc_StatusType fs3ipc_SetChannelState(
	fs3ipc_clientHandleType clientHandle, fs3ipc_u8 enable)
{
	fs3ipc_StatusType ret;
	const fs3ipc_app_LTconfig *appCfg;
	fs3ipc_app_clientChannelLookupType *clientLookup;

	if (clientHandle >= fs3ipc_clientHandleCount_All) {
		LOG_ERROR("Invalid client handle:%d", clientHandle);
		return fs3ipc_StatusType_ErrorHandle;
	}

	clientLookup = &fs3_ipc_clientChannelLookupTable[clientHandle];

	if (clientLookup->initialized != FS3IPC_TRUE) {
		LOG_ERROR("Client handle not initialized:%d", clientHandle);
		return fs3ipc_StatusType_ErrorGen;
	}

	appCfg = &fs3ipc_app_appConfigs[clientLookup->handle];

	ret = fs3ipc_flowControl_EnableRxClient(appCfg->flowControlCfg,
		clientLookup->channel, enable);
	if (ret == fs3ipc_StatusType_OK) {
		(void)fs3ipc_os_SetEvent(appCfg->phys_txEventOsCfg,
			appCfg->phys_txEvent);
	}
	return ret;
}

/*******************************************************************************
 *                            Public Functions
 ******************************************************************************/

/**
 * @brief initialization function
 *
 *
 * @returns  fs3ipc_StatusType_OK else error
 *
 * @details This function initializes each client/channel based on the
 *           configuration prestablished by fs3ipc_cfg.c
 */

fs3ipc_StatusType fs3ipc_Init(fs3ipc_handleType handle)
{
	fs3ipc_s32 chan;
	const fs3ipc_app_LTconfig *cfg;

	if (handle >= FS3IPC_NUM_OF_INSTANCES) {
		return fs3ipc_StatusType_ErrorHandle;
	}

	cfg = &fs3ipc_app_appConfigs[handle];

	(void)fs3ipc_flowcontrol_Init(cfg->flowControlCfg);

	/* init ccb */
	for (chan = 0; chan < cfg->channelCount; chan++)
	{
		const fs3ipc_app_LTChannelCfgType *chanCfg = cfg->ltChannelCfg[chan];

		if (chanCfg)
		{
			fs3ipc_app_priorityNodeType *node = &cfg->orderedChannelNodes[chan];
			fs3ipc_clientHandleType client;
			fs3ipc_s32 qstat;

			/* add entry to client->channel lookup table*/
			client = chanCfg->clientCfg.clientHandle;
			if (client < fs3ipc_clientHandleCount_All)
			{
				if (fs3_ipc_clientChannelLookupTable[client].initialized ==
					FS3IPC_TRUE) {
					/* client handle already registered. print error*/
					LOG_ERROR("handle already registered:%d", client);
					return fs3ipc_StatusType_ErrorCfg;
				} else {
					fs3_ipc_clientChannelLookupTable[client].handle = handle;
					fs3_ipc_clientChannelLookupTable[client].channel = chan;
					fs3_ipc_clientChannelLookupTable[client].initialized = FS3IPC_TRUE;
				}
			}
			else
			{
				/* invalid client handle */
				LOG_ERROR("client handle invalid:%d", handle);
			}

			fs3ipc_os_GetResource(chanCfg->RX_QLock);
			qstat = fs3ipc_app_QueueInit(chanCfg->RX_Q, chanCfg->RX_Buffer,
				chanCfg->RX_Buffer_Size);
			fs3ipc_os_ReleaseResource(chanCfg->RX_QLock);

			if (qstat != fs3ipc_app_qStat_OK)
			{
				LOG_ERROR("VLQ Rx Init Error:%d chan:%d", handle, chan);
			}

			fs3ipc_os_GetResource(chanCfg->TX_QLock);
			qstat = fs3ipc_app_QueueInit(chanCfg->TX_Q, chanCfg->TX_Buffer,
				chanCfg->TX_Buffer_Size);
			fs3ipc_os_ReleaseResource(chanCfg->TX_QLock);

			if (qstat != fs3ipc_app_qStat_OK)
			{
				LOG_ERROR("VLQ Tx Init Error:%d chan:%d", handle, chan);
			}

			/* initialize Tx priority list*/
			fs3ipc_app_PriorityNodeInit(node, (void *)chanCfg,
				chanCfg->priority);
			fs3ipc_app_PriorityNodeInsert(cfg->orderedChannelListHead, node);
			*chanCfg->loggingEnabled = FS3IPC_FALSE;
		}
		else
		{
			/* no channel*/
		}
	}

	return fs3ipc_StatusType_OK;
}

/**
 * @brief this function de initializes the application layer
 *
 * @param[in]  void          - none
 *
 * @returns                 - fs3ipc_StatusType_OK else error condition
 *
 * @details this function is not implemented yet
 *
 * @warning This function is not implemented yet
 */
fs3ipc_StatusType fs3ipc_DeInit(void)
{
	/* todo*/
	return fs3ipc_StatusType_OK;
}

/**
 * @brief this function is called by the client to write SPI data
 *
 * @param[in] clientHandle   - the client that request to write data
 * @param[in] data           - The data buffer to write the data
 * @param[in] length         - The total bytes to write
 *
 * @returns                 - fs3ipc_StatusType_OK else error condition
 *
 * @details  this function is specialized to send the information provided by
 *           data parameter
 */
fs3ipc_StatusType fs3ipc_WriteMessage(fs3ipc_clientHandleType clientHandle,
												  fs3ipc_cDataPtr data,
												  fs3ipc_length length)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;
	fs3ipc_app_clientChannelLookupType *clientLookup;
	const fs3ipc_app_LTconfig *appCfg;
	const fs3ipc_app_LTChannelCfgType *chanCfg;
	fs3ipc_s32 qstat;

	if (clientHandle >= fs3ipc_clientHandleCount_All)
	{
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid client handle:%d", clientHandle);
	}
	else if (data == FS3IPC_NULL)
	{
		stat = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null Data");
	}
	else if (length == 0 || length > FS3_IPC_MAX_MSG_SIZE)
	{
		stat = fs3ipc_StatusType_MessageSize;
		LOG_ERROR("Invalid Msg Size:%d", length);
	}

	if (stat == fs3ipc_StatusType_OK)
	{
		clientLookup = &fs3_ipc_clientChannelLookupTable[clientHandle];
		appCfg = &fs3ipc_app_appConfigs[clientLookup->handle];
		chanCfg = appCfg->ltChannelCfg[clientLookup->channel];

		if (clientLookup->initialized != FS3IPC_TRUE)
		{
			stat = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("Uninitialized client handle:%d", clientLookup->handle);
		}
	}

#if FS3IPC_APP_ALLOW_MSG_Q_WO_FC_CHECK != 1
	if (stat == fs3ipc_StatusType_OK)
	{

		if (fs3ipc_flowControl_GetTxEnabled(appCfg->flowControlCfg,
			clientLookup->channel) != fs3ipc_StatusType_OK)
		{
			stat = fs3ipc_StatusType_ChannelNotAvailable;
			LOG_WARNING("Channel %d unavailable", clientLookup->channel);
		}
		else if (length > fs3ipc_flowControl_GetMaxTxPerFrame(
			appCfg->flowControlCfg, clientLookup->channel))
		{

			stat = fs3ipc_StatusType_MessageSize;
			LOG_ERROR("Invalid Msg Size:%d", length);
		}
	}
#endif

	if (stat == fs3ipc_StatusType_OK)
	{
		fs3ipc_os_GetResource(chanCfg->TX_QLock);
		qstat = fs3ipc_app_QueuePush(chanCfg->TX_Q, data, length);
		fs3ipc_os_ReleaseResource(chanCfg->TX_QLock);

		if (qstat == fs3ipc_app_qStat_full)
		{
			stat = fs3ipc_StatusType_BufferFull;
			LOG_WARNING("Tx Q channel %d full", clientLookup->channel);
		}
		else if (qstat != fs3ipc_app_qStat_OK)
		{
			stat = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("Tx Q channel %d error %d:",
						 clientLookup->channel, qstat);
		}
		else
		{
			LOG_DATA(*chanCfg->loggingEnabled,
				&data[0], length,
				LOG_DATA_MAX_BYTES, "IPC: Tx C%02d L%03d", chanCfg->channel,
				length);
			(void)fs3ipc_os_SetEvent(appCfg->phys_txEventOsCfg,
				appCfg->phys_txEvent);
		}
	}

	return stat;
}

/**
 * @brief this function is called by the client to read SPI information
 *
 * @param[in] clientHandle    - the client that reads the data
 * @param[in] data            - the data buffer to read
 * @param[out] length         - the total bytes to read
 *
 * @returns   - fs3ipc_StatusType_OK else error condition
 *
 * @details this function is called by the client when it receives data.
 *
 */
fs3ipc_StatusType fs3ipc_ReadMessage(fs3ipc_clientHandleType clientHandle,
	fs3ipc_dataPtr data, fs3ipc_length *length)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;
	fs3ipc_app_clientChannelLookupType *clientLookup;

	if (clientHandle >= fs3ipc_clientHandleCount_All)
	{
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid client handle:%d", clientHandle);
	}
	else if (data == FS3IPC_NULL)
	{
		stat = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null Data");
	}
	else if ((length == FS3IPC_NULL) || (*length == 0))
	{
		stat = fs3ipc_StatusType_MessageSize;
		LOG_ERROR("Invalid length arg");
	}

	if (stat == fs3ipc_StatusType_OK) {

		clientLookup = &fs3_ipc_clientChannelLookupTable[clientHandle];

		if (clientLookup->initialized != FS3IPC_TRUE) {
			stat = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("uninitialized instance handle:%d", clientLookup->handle);
		}
	}

	if (stat == fs3ipc_StatusType_OK) {
		const fs3ipc_app_LTconfig *appCfg =
			&fs3ipc_app_appConfigs[clientLookup->handle];
		fs3ipc_s32 qstat;
		const fs3ipc_app_LTChannelCfgType *chanCfg =
			appCfg->ltChannelCfg[clientLookup->channel];

		if (chanCfg)
		{
			fs3ipc_os_GetResource(chanCfg->RX_QLock);
			qstat = fs3ipc_app_QueuePop(chanCfg->RX_Q, data, *length);
			fs3ipc_os_ReleaseResource(chanCfg->RX_QLock);

			if (qstat == fs3ipc_app_qStat_empty) {
				stat = fs3ipc_StatusType_BufferEmpty;
				(void)fs3ipc_flowControl_SetOverflow(
					appCfg->flowControlCfg, clientLookup->channel,
					FS3IPC_FALSE);
				if (fs3ipc_flowControl_MsgPending(appCfg->flowControlCfg) !=
					FS3IPC_FALSE) {
					(void)fs3ipc_os_SetEvent(appCfg->phys_txEventOsCfg,
							appCfg->phys_txEvent);
				}
				LOG_DEBUG("Rx Q channel %d Empty", clientLookup->channel);
			}
			else if (qstat < fs3ipc_app_qStat_OK) {
				stat = fs3ipc_StatusType_ErrorGen;
				LOG_ERROR("Rx Q channel %d error: %d, sz %d",
					clientLookup->channel, qstat, *length);
			}
			else {
				/* clear the overflow flag*/
				/* update length field*/
				*length = qstat;
			}
		}
	}

	return stat;
}

/**
 * @brief this function will block the operation until the message is received
 *
 * @param[in] clientHandle      - Client that waits for any message
 *
 * @returns                     fs3ipc_StatusType_OK else error
 *
 * @details this function blocks the task operation until the message is received
 */
fs3ipc_StatusType fs3ipc_WaitForMessage(fs3ipc_clientHandleType clientHandle)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;

	if (clientHandle >= fs3ipc_clientHandleCount_All)
	{
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid client handle:%d", clientHandle);
	}
	else
	{
		fs3ipc_app_clientChannelLookupType *clientLookup =
			&fs3_ipc_clientChannelLookupTable[clientHandle];

		if (clientLookup->initialized != FS3IPC_TRUE)
		{
			stat = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("Invalid instance handle:%d", clientLookup->handle);
		}
		else
		{
			const fs3ipc_app_LTClientCfgType *ccfg =
				&fs3ipc_app_appConfigs[clientLookup->handle].ltChannelCfg[
				clientLookup->channel]->clientCfg;
			fs3ipc_os_EventMaskType mask = 0;

			if (((stat = fs3ipc_os_WaitEvent(ccfg->osCfg, ccfg->rxEvent)) ==
				fs3ipc_StatusType_OK))
			{
				fs3ipc_os_GetEvent(ccfg->osCfg, &mask);
				fs3ipc_os_ClearEvent(ccfg->osCfg, mask & ccfg->rxEvent);
			}
			else
			{
				/* interrupted*/
			}
		}
	}

	return stat;
}

/**
 * @brief this function will block the operation until the message is received or timeout occurs
 *
 * @param[in] clientHandle      - Client that waits for any message
 *
 * @returns                     fs3ipc_StatusType_OK else error
 *
 * @details this function blocks the task operation until the message is received
 */
fs3ipc_StatusType fs3ipc_WaitForMessageTimeout (fs3ipc_clientHandleType clientHandle, fs3ipc_u32 timeoutInMsec)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;

	if (clientHandle >= fs3ipc_clientHandleCount_All)
	{
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid client handle:%d", clientHandle);
	}
	else
	{
		fs3ipc_app_clientChannelLookupType* clientLookup =
				&fs3_ipc_clientChannelLookupTable [clientHandle];

		if (clientLookup->initialized != FS3IPC_TRUE)
		{
			stat = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("Invalid instance handle:%d", clientLookup->handle);
		}
		else
		{
			const fs3ipc_app_LTClientCfgType* ccfg =
				&fs3ipc_app_appConfigs[clientLookup->handle].ltChannelCfg[clientLookup->channel]->clientCfg;
			fs3ipc_os_EventMaskType mask = 0;

			if (((stat = fs3ipc_os_WaitEventTimeout(ccfg->osCfg, ccfg->rxEvent, timeoutInMsec)) == fs3ipc_StatusType_OK))
			{
				fs3ipc_os_GetEvent(ccfg->osCfg, &mask);
				fs3ipc_os_ClearEvent(ccfg->osCfg, mask & ccfg->rxEvent);
			}
			else
			{
				/* interrupted*/
			}
		}
	}

	return stat;
}

/**
 * @brief This function opens the channel
 *
 * @param[in] clientHandle  - The client to be opened
 *
 * @returns                 - fs3ipc_StatusType_OK else error condition
 *
 * @details This function is specialized to open the channel requested by
 *          clientHandle
 */
fs3ipc_StatusType fs3ipc_OpenChannel(fs3ipc_clientHandleType clientHandle)
{
	return fs3ipc_SetChannelState(clientHandle, FS3IPC_TRUE);
}

/**
 * @brief This function closes the channel requested by clientHandle
 *
 * @param[in] clientHandle  - The client to be closed
 *
 * @returns                 - fs3ipc_StatusType_OK else error condition
 *
 * @details This function is specialized to close the channel requested by
 *          clientHandle
 */
fs3ipc_StatusType fs3ipc_CloseChannel(fs3ipc_clientHandleType clientHandle)
{
	return fs3ipc_SetChannelState(clientHandle, FS3IPC_FALSE);
}

/**
 * @brief                   - Gets the channel status
 *
 * @param[in] clientHandle  - The client requesting the channel status
 *
 * @returns                 - fs3ipc_StatusType_OK else
 *                            fs3ipc_StatusType_ChannelNotAvailable
 *
 * @details                 - this function validates the configuration
 *                            parameters, it returns if the client has been
 *                            enabled by flowcontrol message
 */
fs3ipc_StatusType fs3ipc_GetChannelAvailable(
	fs3ipc_clientHandleType clientHandle)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;

	if (clientHandle >= fs3ipc_clientHandleCount_All)
	{
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid client handle:%d", clientHandle);
	}
	else
	{
		fs3ipc_app_clientChannelLookupType *clientLookup =
			&fs3_ipc_clientChannelLookupTable[clientHandle];

		if (clientLookup->initialized != FS3IPC_TRUE)
		{
			stat = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("Invalid instance handle:%d", clientLookup->handle);
		}
		else
		{
			const fs3ipc_app_LTconfig *appCfg =
				&fs3ipc_app_appConfigs[clientLookup->handle];

			if (clientLookup->channel >= appCfg->channelCount)
			{
				stat = fs3ipc_StatusType_ErrorGen;
				LOG_ERROR("invalid channel %d cfg", clientLookup->channel);
			}
			else
			{
				stat = fs3ipc_flowControl_GetTxEnabled(appCfg->flowControlCfg,
					clientLookup->channel);
			}
		}
	}

	return stat;
}

/**
 * @brief                   - Gets the number of messages queued requested by
 *                            client
 *
 * @param[in] clientHandle - this is the client requesting the count of messages
 * @param[out] result      - number of messages queued
 *
 * @returns                - fs3ipc_StatusType_OK else error condition
 *
 * @details                - this function validates the configuration parameter
 *                           it returns  the number of messages queued requested
 *                           by the client
 *
 */
fs3ipc_StatusType fs3ipc_GetMessagePending(fs3ipc_clientHandleType clientHandle,
	fs3ipc_u32 *result)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;

	if (clientHandle >= fs3ipc_clientHandleCount_All)
	{
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("Invalid client handle:%d", clientHandle);
	}
	else if (result == FS3IPC_NULL)
	{
		stat = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null Data");
	}
	else
	{

		fs3ipc_app_clientChannelLookupType *clientLookup =
			&fs3_ipc_clientChannelLookupTable[clientHandle];

		if (clientLookup->initialized != FS3IPC_TRUE)
		{
			stat = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("Invalid instance handle:%d", clientLookup->handle);
		}
		else
		{
			const fs3ipc_app_LTconfig *appCfg =
				&fs3ipc_app_appConfigs[clientLookup->handle];

			fs3ipc_s32 qstat;

			const fs3ipc_app_LTChannelCfgType *chanCfg =
				appCfg->ltChannelCfg[clientLookup->channel];

			if (chanCfg)
			{
				fs3ipc_os_GetResource(chanCfg->RX_QLock);
				qstat = fs3ipc_app_QueueMsgCnt(chanCfg->RX_Q);
				fs3ipc_os_ReleaseResource(chanCfg->RX_QLock);

				if (qstat >= 0)
				{
					*result = qstat;
				}
				else
				{
					stat = fs3ipc_StatusType_ErrorGen;
					LOG_ERROR("error reading chan %d rx q msg count: %d",
						clientLookup->channel, qstat);
				}
			}
		}
	}

	return stat;
}

/**
 * @brief               - this function resets the queue data
 *
 * @param[in] handle    - Driver context index
 *
 * @returns             - returns the number of bytes encoded
 *
 * @details             - this function flushes the information contained
 *                        into the queues. this function normally is called with
 *                        error handling process.
 */
fs3ipc_StatusType fs3ipc_app_HandleExtNodeReset(fs3ipc_handleType handle)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;

	if (handle >= FS3IPC_APP_NUM_OF_INSTANCES)
	{
		/* This shouldn't happen. Config issue*/
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("invalid handle %d cfg", handle);
	}

	if (stat == fs3ipc_StatusType_OK)
	{
		const fs3ipc_app_LTconfig *appCfg = &fs3ipc_app_appConfigs[handle];
		fs3ipc_u8 chanIndex = appCfg->channelCount;

		fs3ipc_flowcontrol_HandleNodeReset(appCfg->flowControlCfg);
		/* reset queues*/
		while (chanIndex-- > 0)
		{
			const fs3ipc_app_LTChannelCfgType *chanCfg =
				appCfg->ltChannelCfg[chanIndex];

			if (chanCfg)
			{
				fs3ipc_os_GetResource(chanCfg->RX_QLock);
				fs3ipc_app_QueueClear(chanCfg->RX_Q);
				fs3ipc_os_ReleaseResource(chanCfg->RX_QLock);

				fs3ipc_os_GetResource(chanCfg->TX_QLock);
				fs3ipc_app_QueueClear(chanCfg->TX_Q);
				fs3ipc_os_ReleaseResource(chanCfg->TX_QLock);
			}
		}

		/* reset flow control state*/
		memset(appCfg->rxMsg, 0, sizeof(*appCfg->rxMsg));
		memset(appCfg->txMsg, 0, sizeof(*appCfg->txMsg));
		*appCfg->rxMsgBufferOverflowIndex = 0;
	}
	return stat;
}

/**
 * @brief This function encodes the received data buffer
 *
 * @param[in] handle    - Driver context index
 * @param[out] data     - the received data for encoding
 * @param[out] length   - number of bytes to encode
 *
 *
 * @returns             - returns the number of bytes encoded
 *
 * @details   Validates the configuration parameters, encodes the partial
 *            message, it provides the connection to  load the new message from
 *            the queues
 */
fs3ipc_StatusType fs3ipc_app_Encoder(fs3ipc_handleType handle,
												 fs3ipc_dataPtr data,
												 fs3ipc_length *ptrLength)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;
	fs3ipc_length index = 0;

	if (handle >= FS3IPC_APP_NUM_OF_INSTANCES)
	{
		/* This shouldn't happen. Config issue*/
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("invalid handle %d cfg", handle);
	}
	else if (ptrLength == FS3IPC_NULL)
	{
		stat = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("null length ptr arg");
	}
	else if (data == FS3IPC_NULL)
	{
		stat = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("null data ptr arg");
	}
	else if (*ptrLength <= (sizeof(fs3ipc_app_messageHdr) * 2))
	{
		stat = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("buffer too small to encode message: %d", *ptrLength);
	}
	else
	{
	}

	if (stat == fs3ipc_StatusType_OK)
	{
		const fs3ipc_app_LTconfig *cfg = &fs3ipc_app_appConfigs[handle];
		fs3ipc_app_priorityNodeType *currentChannel = cfg->orderedChannelListHead;
		fs3ipc_length length = *ptrLength;
		fs3ipc_s32 remBytes = length;
		fs3ipc_app_messageBufferType *partialMessage = cfg->txMsg;

		/* reset flow control, logical channel byte counter*/
		(void)fs3ipc_flowControl_ResetByteCountTxFrame(cfg->flowControlCfg);

		/* step 1.
		 * Finish encoding partial message
		 * */
		index += handleEncodeResumeMessage(&data[index], remBytes,
		partialMessage, cfg->flowControlCfg);

		/* update number of remaining bytes*/
		remBytes = length - index;

		/* step 2.
		 * Finish handle flow control
		 * */
		if (remBytes > sizeof(fs3ipc_app_messageHdr))
		{
			index += handleEncodeFlowControlMessage(&data[index], remBytes,
				partialMessage, cfg);
		}
		else
		{
		}
		/* update number of remaining bytes*/
		remBytes = length - index;

		/* get first channel */
		fs3ipc_app_PriorityNodeNext(&currentChannel);

		/* 3 - load new message from message queues*/
		while ((currentChannel) &&
			(remBytes > sizeof(fs3ipc_app_messageHdr)))
		{
			fs3ipc_s32 bytesTransferred = 0;

			bytesTransferred = handleEncodeNextMessage(&data[index],
				remBytes, partialMessage,
				(fs3ipc_app_LTChannelCfgType *)currentChannel->data, cfg);
			if (bytesTransferred == 0)
			{
				/* get the next channel */
				fs3ipc_app_PriorityNodeNext(&currentChannel);
			}
			else
			{
			}

			/* update number of remaining bytes*/
			index += bytesTransferred;
			remBytes = length - index;
		}

		/* if the index is equal to or less than the size of the resumed
		 * message header, it means that there is nothing to send.*/
		if (index <= sizeof(fs3ipc_app_messageHdr))
		{
			*ptrLength = 0;
		}
		else
		{
			/* update length*/
			*ptrLength = index;
		}
	}

	return stat;
}

/**
 * @brief function dedicated to decode the received data
 *
 * @param[in] handle    - Driver context index
 * @param[in] data      - the received data for decoding
 * @param[in] length    - length of the buffer
 *
 * @returns             - fs3ipc_StatusType_OK else error condition
 *
 * @details  this function decodes the received buffer data, it verifies
 *           configuration parameters, it resumes the received message or
 *           treat the message as new.
 */
fs3ipc_StatusType fs3ipc_app_Decoder(fs3ipc_handleType handle,
	fs3ipc_cDataPtr data,
	fs3ipc_length length)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;

	if (handle >= FS3IPC_APP_NUM_OF_INSTANCES)
	{
		/* This shouldn't happen. Config issue*/
		stat = fs3ipc_StatusType_ErrorHandle;
		LOG_ERROR("invalid handle %d cfg", handle);
	}
	else if (data == FS3IPC_NULL || length <= (sizeof(fs3ipc_app_messageHdr)))
	{
		stat = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("invalid data buffer addr:%p len:%d", data, length);
	}

	if (stat == fs3ipc_StatusType_OK)
	{
		const fs3ipc_app_LTconfig *appCfg = &fs3ipc_app_appConfigs[handle];
		fs3ipc_length index = 0;
		fs3ipc_s32 remBytes = length - index;

		if (*appCfg->rxMsgBufferOverflowIndex == 0 || length <=
			*appCfg->rxMsgBufferOverflowIndex)
		{
			index += handleDecodeResumeMessage(&data[index], remBytes, appCfg,
				&stat);
			remBytes = length - index;
			*appCfg->rxMsgBufferOverflowIndex = 0;
		}

		/* recovering from a buffer overflow. Skip wrapped message and resume from
		 * previous index so that messages are not queued twice. Per HDLC, the
		 * transmitter will retransmitted the same frame.*/
		else //*appCfg->rxMsgBufferOverflowIndex != 0
		{
			index = *appCfg->rxMsgBufferOverflowIndex;
			remBytes = length - index;
			*appCfg->rxMsgBufferOverflowIndex = 0;
		}

		while (stat == fs3ipc_StatusType_OK &&
			remBytes > sizeof(fs3ipc_app_messageHdr))
		{
			fs3ipc_s32 cnt;

			cnt = handleDecodeNewMessage(&data[index], remBytes, appCfg, &stat);
			if (stat == fs3ipc_StatusType_BufferFull)
			{
				*appCfg->rxMsgBufferOverflowIndex = index;
			}
			else
			{
				index += cnt;
			}

			remBytes = length - index;
		}
	}

	return stat;
}

#if FS3IPC_APP_STATS_ENABLED == 1
/**
 * @details this function gets the Application layer statistics
 *
 * @param[in] Hndl   - Driver context index
 *
 * @returns          - returns the current status counter
 *
 */
const fs3ipc_app_StatsType *fs3ipc_app_GetStats(fs3ipc_handleType Hndl)
{
	fs3ipc_app_StatsType *ret = FS3IPC_NULL;

	if (Hndl < FS3IPC_NUM_OF_INSTANCES)
	{
		const fs3ipc_app_LTconfig *cfg = &fs3ipc_app_appConfigs[Hndl];
		ret = cfg->stats;
	}

	return ret;
}

/**
 * @details this function clears the Application layer statistics
 *
 * @param[in] Hndl         - Driver context index
 *
 * @returns                 - fs3ipc_StatusType_OK else error condition
 *
 */
fs3ipc_StatusType fs3ipc_app_ClearStats(fs3ipc_handleType Hndl)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_ErrorHandle;

	if (Hndl < FS3IPC_NUM_OF_INSTANCES)
	{
		const fs3ipc_app_LTconfig *cfg = &fs3ipc_app_appConfigs[Hndl];

		if (cfg->stats)
		{
			fs3ipc_memset(cfg->stats, 0, sizeof(fs3ipc_app_StatsType));
			stat = fs3ipc_StatusType_OK;
		}
	}
	return stat;
}

/**
 * @brief this function returns the channel statistics
 *
 * @param[in] Hndl         - Driver context index
 * @param[in] channel      - the channel to request the information
 *
 * @returns                 - returns the pointer to access the structure
 *                            defined for collecting the statics
 *
 * @details this function checks for the correct number of instances, if the
 *          Arguments are correct, the get operation is executed

 *
 */
const fs3ipc_flowCtrlChanPBCfgType *
fs3ipc_app_GetChanStats(fs3ipc_handleType Hndl, fs3ipc_u8 channel)
{
	const fs3ipc_flowCtrlChanPBCfgType *ret = NULL;

	if ((Hndl < FS3IPC_NUM_OF_INSTANCES) &&
		(channel < FS3_IPC_APP_MAX_NUMBER_CHANNELS))
	{
		ret = fs3ipc_flowcontrol_GetStats(
			fs3ipc_app_appConfigs[Hndl].flowControlCfg, channel);
	}

	return ret;
}

/**
 * @brief this function empties the channel statistics
 *
 * @param[in] Hndl         - Driver context index
 * @param[in] channel      - the channel to clear the information
 *
 * @returns                 - fs3ipc_StatusType_OK else error condition
 *
 * @details this function checks for the correct number of instances, if the
 *          Arguments are correct, the clear operation is executed
 */
fs3ipc_StatusType fs3ipc_app_ClearChanStats(fs3ipc_handleType Hndl,
														  fs3ipc_u8 channel)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_ErrorHandle;

	if ((Hndl < FS3IPC_NUM_OF_INSTANCES) &&
		(channel < FS3_IPC_APP_MAX_NUMBER_CHANNELS))
	{
		stat = fs3ipc_flowcontrol_ClearStats(
			fs3ipc_app_appConfigs[Hndl].flowControlCfg, channel);
	}
	return stat;
}
#endif

/**
 * @brief this function enables message logging for a channel
 *
 * @param[in] Hndl         - Driver context index
 * @param[in] channel      - the channel to clear the information
 * @param[in] enabled      - 0 = logging disabled; 1 = logging enabled
 *
 * @returns                 - fs3ipc_StatusType_OK else error condition
 *
 * @details this function enables message logging for a channel.
 */
fs3ipc_StatusType fs3ipc_app_SetLogging(fs3ipc_handleType Hndl,
													 fs3ipc_u8 channel,
													 fs3ipc_u8 enabled)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;
	const fs3ipc_app_LTChannelCfgType *chanCfg;
	if ((Hndl >= FS3IPC_NUM_OF_INSTANCES) ||
		(channel >= FS3_IPC_APP_MAX_NUMBER_CHANNELS))
	{
		ret = fs3ipc_StatusType_ErrorHandle;
	}
	if (ret == fs3ipc_StatusType_OK)
	{
		enabled = enabled ? FS3IPC_TRUE : FS3IPC_FALSE;
		if (channel == FS3IPC_APP_FLOWCONTROL_CHAN)
		{
			ret = fs3ipc_flowcontrol_SetLogging(
				 fs3ipc_app_appConfigs[Hndl].flowControlCfg, enabled);
		}
		else
		{
			chanCfg = fs3ipc_app_appConfigs[Hndl].ltChannelCfg[channel];
			if (chanCfg == FS3IPC_NULL)
			{
				ret = fs3ipc_StatusType_ErrorGen;
			}
			else
			{
				*chanCfg->loggingEnabled = enabled;
			}
		}
	}
	return ret;
}
