/*****************************************************************************
 *  Project          Harman Car Multimedia System
 *
 *  (c) copyright    2015
 *  Company          Harman International Industries, Incorporated
 *                   All rights reserved
 *  Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 *  @file      		hdlc.c
 *  @ingroup       	Kernel and Drivers
 *  @author             Bhagat Singh <bhagat.singh@harman.com>
 *                      Prasad Lavande <prasad.lavande@harman.com>
 *  @brief              HDLC implementation with Go-Back-N arq
 *
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include "hdlc.h"
#include "types.h"
#include "ipc.h"
#include "ipcDev.h"
#include "flowControl.h"
#include "handler.h"
#include "log.h"
#include "options.h"

//HDLC global structure
tHdlc gHdlc;
UInt8 uWaitForFinal=FALSE;

Int8 ipcExitFlag=FALSE;
UInt8 uStrFlag=FALSE;

extern IPCAttr_t* ipcRoot;
static Int8 hdlcInitDone = FALSE;
static UInt32 qFlowCtrlStatus;
static UInt32 qPrevFlowCtrlStatus;


/**
 * Print HDLC statistics
 *
 * @param  none
 * @return none
 */
#ifdef	IPC_STATS
void HdlcStatisticsPrint(void)
{
	log_notice("FC     |%12u|%12u|",gHdlc.hdlcStats.tx_count,gHdlc.hdlcStats.rx_count);
	log_notice("RTX/OOS|%12u|%12u|",gHdlc.hdlcStats.tx_retransmit_count,gHdlc.hdlcStats.rx_oos_count);
	log_notice("ACK    |%12u|%12u|",gHdlc.hdlcStats.tx_ack_count,gHdlc.hdlcStats.rx_ack_count);
	log_notice("REJ    |%12u|%12u|",gHdlc.hdlcStats.tx_rej_count,gHdlc.hdlcStats.rx_rej_count);
	log_notice("POLL   |%12u|%12u|",gHdlc.hdlcStats.tx_poll_count,gHdlc.hdlcStats.rx_poll_count);
	log_notice("FINAL  |%12u|%12u|",gHdlc.hdlcStats.tx_final_count,gHdlc.hdlcStats.rx_final_count);
	log_notice("TExp   |%12u|",gHdlc.hdlcStats.tx_timeout_count);
	log_notice("CRC    |%12u|",gHdlc.hdlcStats.crc_fail_count);
	log_notice("PIACK  |%12u|",gHdlc.hdlcStats.tx_piggy_ack_count);
	log_notice("DUP    |%12u|",gHdlc.hdlcStats.dup_pkt_count);
	return;
}
#else
void HdlcStatisticsPrint(struct seq_file *m)
{
	seq_printf(m, "FC     |%12u|%12u|",gHdlc.hdlcStats.tx_count,gHdlc.hdlcStats.rx_count);
	seq_printf(m, "RTX/OOS|%12u|%12u|",gHdlc.hdlcStats.tx_retransmit_count,gHdlc.hdlcStats.rx_oos_count);
	seq_printf(m, "ACK    |%12u|%12u|",gHdlc.hdlcStats.tx_ack_count,gHdlc.hdlcStats.rx_ack_count);
	seq_printf(m, "REJ    |%12u|%12u|",gHdlc.hdlcStats.tx_rej_count,gHdlc.hdlcStats.rx_rej_count);
	seq_printf(m, "POLL   |%12u|%12u|",gHdlc.hdlcStats.tx_poll_count,gHdlc.hdlcStats.rx_poll_count);
	seq_printf(m, "FINAL  |%12u|%12u|",gHdlc.hdlcStats.tx_final_count,gHdlc.hdlcStats.rx_final_count);
	seq_printf(m, "TExp   |%12u|",gHdlc.hdlcStats.tx_timeout_count);
	seq_printf(m, "CRC    |%12u|",gHdlc.hdlcStats.crc_fail_count);
	seq_printf(m, "PIACK  |%12u|",gHdlc.hdlcStats.tx_piggy_ack_count);
	seq_printf(m, "DUP    |%12u|",gHdlc.hdlcStats.dup_pkt_count);
	return;
}
#endif

/**
 * Informs that data is available to process, called from IPC tx thread
 *
 * @param  none
 * @return none
 */
void setEventDataAvailable(void)
{
	spin_lock(&gHdlc.eventLock);
	SET_EVENT(gHdlc.eventFlag, EVENT_TX_WAKEUP);
	spin_unlock(&gHdlc.eventLock);

	wake_up_interruptible(&gHdlc.wq);
}

static void clearWakeUpEvent(void)
{
	spin_lock(&gHdlc.eventLock);

	if(TEST_EVENT(gHdlc.eventFlag, EVENT_TX_WAKEUP))
		CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_WAKEUP);

	spin_unlock(&gHdlc.eventLock);
	return;
}

#ifndef	RX_COMM_THREAD
static void checkSlaveReadyEvent(void)
{
	UInt8 *uEncBuff = (UInt8 *)NULL;
	/* write the data */
	halWrite(uEncBuff, HDLC_FRAME_MAXLEN);
	return;
}
#endif

/**
 * Checks for any pending Uframe requests
 *
 * @param  none
 * @return none
 */
static void checkPendingUframeRequest(void)
{
	UInt16 	wEventFlag,wFrameLen;
	UInt8 	uEncBuff[HDLC_FRAME_MAXLEN];
	UInt32 syncDone = FALSE;

	memset(uEncBuff, 0, sizeof(wFrameLen));
	wFrameLen = sizeof(gHdlc.uFrame) + CRC_SIZE;
	memcpy(uEncBuff, &wFrameLen, sizeof(wFrameLen));

	spin_lock(&gHdlc.eventLock);
	wEventFlag = gHdlc.eventFlag;
	spin_unlock(&gHdlc.eventLock);

	/// Indicates that we have received a Uframe reset, we need to send ack using uframe ack
	if(TEST_EVENT(wEventFlag, EVENT_TX_UFRAME_ACK))
	{
		log_notice("Event Tx Uframe ACK: Sending Uframe ACK");

		/// clear the event
		spin_lock(&gHdlc.eventLock);
		CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_UFRAME_ACK);
		spin_unlock(&gHdlc.eventLock);

		/// create, encode and transmit the UACK frame
		hdlcCreateUAckFrame((tUframe*)&uEncBuff[HDLC_FRAME_HEADER_OFFSET], gHdlc.goBackN.window_size);
		hdlcEncodeFrame(&uEncBuff[HDLC_FRAME_HEADER_OFFSET], sizeof(gHdlc.uFrame));
		halWrite(uEncBuff, HDLC_FRAME_MAXLEN);
	}

	/// Indicates that we have to send a U-frame reset
	if(TEST_EVENT(wEventFlag, EVENT_TX_UFRAME_RST))
	{
		log_info("Event Tx Uframe Reset");

		/// clear the event
		spin_lock(&gHdlc.eventLock);
		CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_UFRAME_RST);
		syncDone = gHdlc.syncDone;
		spin_unlock(&gHdlc.eventLock);

		if(syncDone == FALSE) {
			log_info("Sending Uframe RESET\n");
			/// create, encode and transmit the URST frame
			hdlcCreateURstFrame((tUframe*)&uEncBuff[HDLC_FRAME_HEADER_OFFSET], gHdlc.goBackN.window_size);
			hdlcEncodeFrame(&uEncBuff[HDLC_FRAME_HEADER_OFFSET], sizeof(gHdlc.uFrame));
			resetUframeTimer();
			halWrite(uEncBuff, HDLC_FRAME_MAXLEN);
		}
		else {
			log_notice("Sync Done Already....Not Sending Uframe RESET");
		}
	}
	return;
}

/**
 * Checks for any pending Rx request(ACK,OOS,POLL) for the IOC
 *
 * @param  none
 * @return none
 */
#ifdef	PENDING_RX_OLD
static void checkPendingRxRequest(UInt16 wEventFlag)
#else
static void checkPendingRxRequest(void)
#endif
{
	UInt8	uRxSeq;
	UInt8 	uEncBuff[HDLC_FRAME_MAXLEN];
	UInt16 	wFrameLen;

#ifndef	PENDING_RX_OLD
	UInt16 wEventFlag;
#endif
	memset(uEncBuff, 0, sizeof(wFrameLen));
	wFrameLen = sizeof(gHdlc.sFrame) + CRC_SIZE;
	memcpy(uEncBuff, &wFrameLen, sizeof(wFrameLen));
#ifndef	PENDING_RX_OLD
	spin_lock(&gHdlc.eventLock);
	wEventFlag = gHdlc.eventFlag;
	spin_unlock(&gHdlc.eventLock);
#endif

	/** indicates that we have received an out-of-seq frame, need to send a REJ frame with the expected sequence number
	 * The REJ frame is sent only once when we receive a out-of-seq frame, the next REJ frame is sent only after we receive
	 * a valid I-frame */
	if(TEST_EVENT(wEventFlag, EVENT_TX_REJ_FRAME))
	{
		log_info("Event Tx REJ frame: Sending REJ Frame");

		/// clear the event
		if(gHdlc.goBackN.rejAlreadySent == TRUE)
		{
			spin_lock(&gHdlc.eventLock);
			CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_REJ_FRAME);
			spin_unlock(&gHdlc.eventLock);

			uRxSeq = gHdlc.goBackN.rx_seq;

			/// create, encode and transmit REJ frame
			hdlcCreateREJFrame(uRxSeq, &uEncBuff[HDLC_FRAME_HEADER_OFFSET]);
			hdlcEncodeFrame(&uEncBuff[HDLC_FRAME_HEADER_OFFSET], sizeof(gHdlc.sFrame));
			halWrite(uEncBuff, HDLC_FRAME_MAXLEN);
			HDLC_STATS_INC(tx_rej_count);
		}
	}

	/// Indicates that we have received a Poll frame, need to inform our status by sending a Final frame
	if(TEST_EVENT(wEventFlag, EVENT_TX_FINAL_FRAME))
	{
		log_info("Event Tx Final Frame: Sending Final Frame");

		/// clear the event
		spin_lock(&gHdlc.eventLock);
		CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_FINAL_FRAME);
		spin_unlock(&gHdlc.eventLock);

		uRxSeq = gHdlc.goBackN.rx_seq;

		/// create, encode and transmit the Poll frame
		hdlcCreateRRFinalFrame(uRxSeq, &uEncBuff[HDLC_FRAME_HEADER_OFFSET]);
		hdlcEncodeFrame(&uEncBuff[HDLC_FRAME_HEADER_OFFSET], sizeof(gHdlc.sFrame));
		halWrite(uEncBuff, HDLC_FRAME_MAXLEN);
		HDLC_STATS_INC(tx_final_count);
	}

	/** Indicates that a valid Iframe has been received and Tx window is not open..so need to acknowledge by
	 * sending a RR ack frame */
	if(TEST_EVENT(wEventFlag, EVENT_TX_ACK_FRAME))
	{
		log_info("Event Tx ACK frarme: Sending Ack Frame");

		/// clear the event
		spin_lock(&gHdlc.eventLock);
		CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_ACK_FRAME);
		spin_unlock(&gHdlc.eventLock);

		uRxSeq = gHdlc.goBackN.rx_seq;
		qFlowCtrlStatus = flowCtrlGetRxFlowControlMask();

		/// create, encode and transmit RR Ack frame
		hdlcCreateRRAckFrame(uRxSeq, &uEncBuff[HDLC_FRAME_HEADER_OFFSET], qFlowCtrlStatus);
		hdlcEncodeFrame(&uEncBuff[HDLC_FRAME_HEADER_OFFSET], sizeof(gHdlc.sFrame));
		halWrite(uEncBuff, HDLC_FRAME_MAXLEN);
		HDLC_STATS_INC(tx_ack_count);
	}
	return;
}

/**
 * Check for any Timeout Event
 *
 * @param  none
 * @return none
 */
static UInt8 checkTimeoutEvent(UInt16 wEventStatus)
{
	UInt16 wFrameLen;
	UInt8  uRxSeq;
	UInt8 uEncBuff[HDLC_FRAME_MAXLEN];
	UInt8 uRet=FALSE;

	memset(uEncBuff, 0, sizeof(wFrameLen));
	wFrameLen = sizeof(gHdlc.sFrame) + CRC_SIZE;
	memcpy(uEncBuff, &wFrameLen, sizeof(wFrameLen));

	/** Indicates that we have not received any ack from the remote end, need to send a Poll frame to find the status of the
	 * receiver, this flag is updated from the timeout handler */
	if(TEST_EVENT(wEventStatus, EVENT_TX_TIMEOUT))
	{
		/// clear the event
		spin_lock(&gHdlc.eventLock);
		CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_TIMEOUT);
		spin_unlock(&gHdlc.eventLock);

		if(gHdlc.goBackN.ack_exp != gHdlc.goBackN.tx_seq)
		{
			//log_info("timerHandler: Waiting for Final");
			uWaitForFinal = TRUE;

			log_info("Event Tx Timeout: Sending poll frame");

			uRxSeq = INVALID_SEQ; /**< 7F as the rx_seq is not valid in a poll frame */

			/// create, encode and transmit the Poll frame
			hdlcCreateRRPollFrame(uRxSeq, &uEncBuff[HDLC_FRAME_HEADER_OFFSET]);
			hdlcEncodeFrame(&uEncBuff[HDLC_FRAME_HEADER_OFFSET], sizeof(gHdlc.sFrame));
			halWrite(uEncBuff, HDLC_FRAME_MAXLEN);
			HDLC_STATS_INC(tx_poll_count);

			if(uWaitForFinal == TRUE)
			{
				resetTimer();
				uRet = TRUE;
			}
		}
		else
		{
			uWaitForFinal = FALSE;
			log_info("All frames have been acked..continuing");
		}
	}
	return uRet;
}


/**
 * Check for any pending retransmission event
 *
 * @param  none
 * @return none
 */
static void checkRetransmissionEvent(UInt16 wEventStatus)
{
	UInt8 uRxSeq,uTxSeq;
	UInt8 uFrameIndex,uLastFrameIndex,uFirstFrameIndex;
	UInt8 *pData;
	UInt16 wFrameLen;

	/** Indicates that a reject/final frame has been received(as we have sent an out-of-seq frame), need to retransmit
	 * all frames starting from N(R) (in the REJ frame received) upto tx_seq-1 */
	if(TEST_EVENT(wEventStatus, EVENT_RETRANSMIT_REJ_FRAME) || TEST_EVENT(wEventStatus, EVENT_RETRANSMIT_FINAL_FRAME))
	{
		log_info("Event retransmit REJ/Final frame: Retransmission");

		/// clear the REJ frame event
		spin_lock(&gHdlc.eventLock);
		CLEAR_EVENT(gHdlc.eventFlag, EVENT_RETRANSMIT_REJ_FRAME);
		spin_unlock(&gHdlc.eventLock);

		/// find the index of the last frame to be transmitted
		uFirstFrameIndex = gHdlc.goBackN.ack_exp;
		uLastFrameIndex = gHdlc.goBackN.tx_seq;
		if(uLastFrameIndex < uFirstFrameIndex)
			uLastFrameIndex = uLastFrameIndex + gHdlc.goBackN.window_size;

		log_info("REJ/Final frame retransmission: window[%d %d]",uFirstFrameIndex,uLastFrameIndex);
		if((uLastFrameIndex - uFirstFrameIndex) > (gHdlc.goBackN.window_size-1))
			log_warning("HDLC: sequence error : window[%d %d]", uFirstFrameIndex,uLastFrameIndex);

		/// start retransmission of the frames in the retransmission queue
		for(uFrameIndex = uFirstFrameIndex; uFrameIndex < uLastFrameIndex; uFrameIndex++)
		{
			uRxSeq = INVALID_SEQ; /**< 7F */
			uTxSeq = uFrameIndex % gHdlc.goBackN.window_size;
			pData = gHdlc.rtxQueue[uTxSeq].data;

			log_info("Retransmitting frame: seq no = %d, rx_seq = %d",uTxSeq, uRxSeq);
			hdlcCreateIframe(pData + HDLC_FRAME_HEADER_OFFSET, uTxSeq, uRxSeq);
			hdlcEncodeFrame(pData + HDLC_FRAME_HEADER_OFFSET, gHdlc.rtxQueue[uTxSeq].datalen + HDLC_FRAME_HEADER_LEN);
			wFrameLen = gHdlc.rtxQueue[uTxSeq].datalen + HDLC_FRAME_HEADER_LEN + CRC_SIZE;
			memcpy(pData, &wFrameLen, sizeof(wFrameLen));

			halWrite(pData, HDLC_FRAME_MAXLEN);
			HDLC_STATS_INC(tx_retransmit_count);
			/*ToDo: Window check with lock reqd*/
			//			/// reset the timer
			//			spin_lock(&gHdlc.eventLock);
			//			if(gHdlc.goBackN.ack_exp != gHdlc.goBackN.tx_seq)
			resetTimer();
			//			spin_unlock(&gHdlc.eventLock);

			/*ToDo: Redq?*/
			/// check for any pending request for the other end
			checkPendingRxRequest();
		}

		/// clear the final frame event
		spin_lock(&gHdlc.eventLock);
		CLEAR_EVENT(gHdlc.eventFlag, EVENT_RETRANSMIT_FINAL_FRAME);
		spin_unlock(&gHdlc.eventLock);
	}
	return;
}

/**
 * Check for any Tx event
 *
 * @param  none
 * @return none
 */
static void checkTxEvent(UInt16 wEventStatus)
{
	UInt8 uRxSeq,uTxSeq;
	UInt16 wFrameLen;
	UInt8 *pData;

	/// indicates that data is available and window is open
	if(TEST_EVENT(wEventStatus, EVENT_TX_WINDOW_OPEN))
	{
		/// get the index of the frame to be transmitted
		uTxSeq = gHdlc.goBackN.tx_seq;
		pData = gHdlc.rtxQueue[uTxSeq].data;

		/// fill the IPC data and its length which will be transmitted
		if(!getIpcTxBuffer(pData + HDLC_FRAME_DATA_OFFSET, &gHdlc.rtxQueue[uTxSeq].datalen))
		{
			if(gHdlc.rtxQueue[uTxSeq].datalen <= IPC_BUFFER_SIZE)
			{
				log_info("Event Tx window open");
				spin_lock(&gHdlc.eventLock);
				/// clear the window open event
				CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_WINDOW_OPEN);

				uRxSeq = INVALID_SEQ; //7F

				if(TEST_EVENT(gHdlc.eventFlag, EVENT_TX_ACK_FRAME))
				{
					/// get the flow control status
					qFlowCtrlStatus = flowCtrlGetRxFlowControlMask();
					if(qPrevFlowCtrlStatus == qFlowCtrlStatus)
					{
						uRxSeq = gHdlc.goBackN.rx_seq;
						CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_ACK_FRAME);
						HDLC_STATS_INC(tx_piggy_ack_count);
					}
				}
				spin_unlock(&gHdlc.eventLock);

				log_info("frame transmit seq no = %d..rx_seq = %x..pktlen=%d",uTxSeq, uRxSeq, gHdlc.rtxQueue[uTxSeq].datalen);

				/// create, encode and transmit the Iframe
				hdlcCreateIframe(pData + HDLC_FRAME_HEADER_OFFSET, uTxSeq, uRxSeq);

				hdlcEncodeFrame(pData + HDLC_FRAME_HEADER_OFFSET, gHdlc.rtxQueue[uTxSeq].datalen + HDLC_FRAME_HEADER_LEN);
				wFrameLen = gHdlc.rtxQueue[uTxSeq].datalen + HDLC_FRAME_HEADER_LEN + CRC_SIZE;
				memcpy(pData, &wFrameLen, sizeof(wFrameLen));
				//dumpBuffer(pData, wFrameLen+2, __func__,__LINE__);
				IncSeqNo(gHdlc.goBackN.tx_seq, gHdlc.goBackN.window_size);
				log_info("going to write");
				halWrite(pData, HDLC_FRAME_MAXLEN);
				HDLC_STATS_INC(tx_count);
				/*ToDo: Window check with lock reqd*/
				//				/// reset the timer
				//				spin_lock(&gHdlc.eventLock);
				//				if(gHdlc.goBackN.ack_exp != gHdlc.goBackN.tx_seq)
				resetTimer();
				//				spin_unlock(&gHdlc.eventLock);
			}
			else
			{
				log_warning("getIpcTxBuffer: Invalid data length from IPC, len=%d",gHdlc.rtxQueue[uTxSeq].datalen);
			}
		}
		else
		{
			log_warning("getIpcTxBuffer failed");
		}
	}
	return;
}

static UInt16 getEventFlag(void)
{
	UInt16 wEventFlag;
	UInt8 txEnable;
	UInt8 window;

	/// check if window is open and data is available, if yes process else block
	spin_lock(&gHdlc.eventLock);
	txEnable = gHdlc.txEnable;
	window = windowOpen(&gHdlc.goBackN);
	spin_unlock(&gHdlc.eventLock);
	if(txEnable && window && dataAvailable()) {
		log_info("Tx window open and data available\n");
		spin_lock(&gHdlc.eventLock);
		SET_EVENT(gHdlc.eventFlag, EVENT_TX_WINDOW_OPEN);
		spin_unlock(&gHdlc.eventLock);
	}
	else {
		log_info("Tx window not open or data not available\n");
	}

	spin_lock(&gHdlc.eventLock);
	if(uWaitForFinal == TRUE) {
		wEventFlag = gHdlc.eventFlag & ~((1 << EVENT_TX_WINDOW_OPEN) | (1 << EVENT_RETRANSMIT_REJ_FRAME) | (1 << EVENT_RETRANSMIT_FINAL_FRAME));
	}
	else {
		wEventFlag = gHdlc.eventFlag;
	}
	spin_unlock(&gHdlc.eventLock);
	log_info("event flag = %d, %d",wEventFlag, gHdlc.eventFlag);

	return wEventFlag;
}

/**
 * Tx communication thread which handles transmit functions
 *
 * Receives the data from the IPC Tx queue, forms a Hdlc frame and transmits
 * over the serial port, also responds to frames received from the Rx thread
 * by sending the appropriate S-frame (REJ,RR Ack/Poll/final)
 *
 * @param  none
 * @param  none
 */
static int Tx_Comm_Thread(void* arg)
{
	UInt16 wEventFlag;
	UInt8 uTxEnable=FALSE;

	/// Set the Uframe Reset event to send Uframe RST to the remote end
	SET_EVENT(gHdlc.eventFlag, EVENT_TX_UFRAME_RST);

	while(TRUE)
	{
		if(kthread_should_stop()) {
			log_notice("Thread %s: exiting", __func__);
			break;
		}
#ifndef	RX_COMM_THREAD
		wait_event_interruptible(gHdlc.wq, ((0 != (wEventFlag = getEventFlag())) || (gHdlc.uSlaveReady == TRUE)));
		log_info("Start event processing..event flag = %d sready flag = %d",wEventFlag, gHdlc.uSlaveReady);
#else
		wait_event_interruptible( gHdlc.wq, (0 != (wEventFlag = getEventFlag())) );
		log_info("Start event processing..event flag = %d, %d",wEventFlag, gHdlc.eventFlag);
#endif
		if(wEventFlag)
		{
			/// check for any wakeup event
			clearWakeUpEvent();

			uTxEnable = gHdlc.txEnable;

			/// check for any pending U-frame request to be sent
			checkPendingUframeRequest();
			if(uTxEnable)
			{
				// check for any timeout event for packets sent
				if(FALSE == checkTimeoutEvent(wEventFlag))
				{
					/// check for any retransmission event
					checkRetransmissionEvent(wEventFlag);

					/// check for any transmission event
					checkTxEvent(wEventFlag);
				}

				/// check for any pending request for the other end
#ifdef	PENDING_RX_OLD
				checkPendingRxRequest(wEventFlag);
#else
				checkPendingRxRequest();
#endif


				/// update the prevFlowCtrlStatus
				qPrevFlowCtrlStatus = qFlowCtrlStatus;
			}
			else
			{
				log_info("Tx not enabled");
			}
		}
#ifndef	RX_COMM_THREAD
		/// check for any data request from slave
		if(gHdlc.uSlaveReady == TRUE)
		{
			log_info("Event Slave Ready");
			checkSlaveReadyEvent();
		}
#endif
		if(ipcExitFlag) {
			break;
		}
	}
	return EOK;
}

/**
 * Process the U-frame
 *
 * @param  rx_frame	pointer to received frame
 * @return none
 */
static void processUframe(UInt8* rx_frame)
{
	tUframe *pUframe = (tUframe *) rx_frame;
	UInt8 uFrameType;

	/// determine the frame type (URST/UACK)
	uFrameType = (pUframe->type_12 | (pUframe->type_345 << 2));
	switch (uFrameType) {
		case URST_FRAME:
			log_notice("RESET frame received..reset the sequence nos and clear the retrans queue");

			if(TRUE == uStrFlag)
			{
				uStrFlag = FALSE;
				/// reset the tx,rx & ack_exp seq nos and signal the tx thread to send U-frame ack
				gHdlc.goBackN.ack_exp = 0;
				gHdlc.goBackN.tx_seq = 0;
				gHdlc.goBackN.rx_seq = 0;
				gHdlc.goBackN.rejAlreadySent = 0;
				if((pUframe->payload == WINDOW_SIZE_4) || (pUframe->payload == DEFAULT_WINDOW_SIZE))
				{
					log_notice("URST_FRAME: change window size to %d", pUframe->payload);
					gHdlc.goBackN.window_size = pUframe->payload;
				}
				else
				{
					log_warning("URST_FRAME: Invalid payload"
						" (%d) recvd in U-frame, continuing"
						" with window size %d", pUframe->payload,
						gHdlc.goBackN.window_size);
				}

			}

			spin_lock(&gHdlc.eventLock);
			SET_EVENT(gHdlc.eventFlag, EVENT_TX_UFRAME_ACK);
			wake_up_interruptible(&gHdlc.wq);
			spin_unlock(&gHdlc.eventLock);
			break;

		case UACK_FRAME:
			log_notice("U-frame ACK received");
			stopUframeTimer();

			spin_lock(&gHdlc.eventLock);
			if(gHdlc.syncDone == FALSE) {
				/// reset the tx,rx & ack_exp seq nos and enable transmission
				gHdlc.goBackN.ack_exp = 0;
				gHdlc.goBackN.tx_seq = 0;
				gHdlc.goBackN.rx_seq = 0;
				gHdlc.goBackN.rejAlreadySent = 0;

				if((pUframe->payload == WINDOW_SIZE_4) || (pUframe->payload == DEFAULT_WINDOW_SIZE))
				{
					gHdlc.goBackN.window_size = pUframe->payload;
					spin_unlock(&gHdlc.eventLock);
					log_notice("UACK_FRAME: change window size to %d",pUframe->payload);
				}
				else
				{
					spin_unlock(&gHdlc.eventLock);
					log_warning("UACK_FRAME: Invalid payload"
						" (%d) recvd in U-frame, continuing"
						" with window size %d", pUframe->payload,
						gHdlc.goBackN.window_size);
				}

				spin_lock(&gHdlc.eventLock);
				gHdlc.txEnable = TRUE;
				gHdlc.syncDone = TRUE;
				spin_unlock(&gHdlc.eventLock);
				setEventDataAvailable();
			}
			else {
				spin_unlock(&gHdlc.eventLock);
			}
			break;

		default:
			log_warning("Invalid U-frame type received: %d %d %d ",uFrameType, pUframe->type_12, pUframe->type_345);
			break;
	}
	return;
}

/**
 * Process the S-frame
 *
 * @param  rx_frame	pointer to received frame
 * @return none
 */
static void processSframe(UInt8* rx_frame, Int32 dataLen)
{
	tSframe *pSframe = (tSframe *) rx_frame;
	UInt8 uFrameType;

	/// determine the frame type (REJ/RR/RNR/SREJ)
	uFrameType = pSframe->ss;
	switch (uFrameType) {
		case REJ_FRAME:
			log_info("REJ frame recvd: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pSframe->nr);
			HDLC_STATS_INC(rx_rej_count);
			/// check if Nr falls within the window
			if(validNr(&gHdlc.goBackN, pSframe->nr))
			{
				/// valid Nr, stop the timer and update the acknowledge window
				stopTimer();
				gHdlc.goBackN.ack_exp = pSframe->nr;
				if(gHdlc.goBackN.ack_exp == gHdlc.goBackN.tx_seq)
				{
					log_info("REJ Frame: All frames acked");
				}
				else
				{
					///reset the timer & set the event to retransmit the frames as they have not been acknowledged
					spin_lock(&gHdlc.eventLock);
					SET_EVENT(gHdlc.eventFlag, EVENT_RETRANSMIT_REJ_FRAME);
					spin_unlock(&gHdlc.eventLock);
					wake_up_interruptible(&gHdlc.wq);
				}
			}
			else
			{
				log_warning("REJ Frame: Invalid Nr, out-of-window: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pSframe->nr);
			}
			break;

		case RR_FRAME:
			if(pSframe->pf == FALSE) /**< P/F bit is clear, so it is a normal ack frame from the receiver */
			{
				log_info("RR Ack Frame recvd: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pSframe->nr);
				HDLC_STATS_INC(rx_ack_count);
#ifdef	FLOW_CONTROL
				//update the flow control status
				if(dataLen > 0)
				{
					flowCtrlUpdateTxFlowControlStatus(pSframe->qFlowCtrlMask);
				}
#endif
				///check if Nr falls within the window
				if(validNr(&gHdlc.goBackN, pSframe->nr))
				{
					if(pSframe->nr == gHdlc.goBackN.ack_exp)
					{
						log_warning("RR Ack Frame: Invalid Nr, Already acked: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pSframe->nr);
					}
					else
					{
						gHdlc.goBackN.ack_exp = pSframe->nr;
						/// valid ack, reset the timer and update the acknowledge window
						if(gHdlc.goBackN.ack_exp == gHdlc.goBackN.tx_seq)
						{
							/// stop the timer as all frames have been acknowledged
							stopTimer();
							log_info("RR Ack Frame: stop timer");
						}
						else
						{
							/// reset the timer as there are frames yet to be acknowledged
							resetTimer();
						}

						/** As we have got a valid ack, the window is now open. signal the tx thread so that if it has
						 * further frames to send,it can start transmitting */
						setEventDataAvailable();
					}
				}
				else
				{
					log_warning("RR Ack Frame: Invalid Nr, out-of-window: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pSframe->nr);

				}
			}
			else	/**< P/F bit is set, so it is either a poll frame or a final frame based on the command/response bit */
			{
				if(pSframe->addr_cr == POLL_RESPONSE)	/**< the command bit is 0, so it is a final frame */
				{
					log_info("RR Final Frame recvd: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pSframe->nr);
					HDLC_STATS_INC(rx_final_count);

					///check if Nr falls within the window
					if(uWaitForFinal == TRUE)
					{
						log_info("Final frame received");
						uWaitForFinal = FALSE;
						if(validNr(&gHdlc.goBackN, pSframe->nr))
						{
							/// valid Nr, stop the timer and update the acknowledge window
							stopTimer();
							gHdlc.goBackN.ack_exp = pSframe->nr;
							if(gHdlc.goBackN.ack_exp == gHdlc.goBackN.tx_seq)
							{
								log_info("RR Final Frame: All frames acked");
							}
							else
							{
								///reset the timer & set the event to retransmit the frames as they have not been acknowledged
								spin_lock(&gHdlc.eventLock);
								SET_EVENT(gHdlc.eventFlag, EVENT_RETRANSMIT_FINAL_FRAME);
								spin_unlock(&gHdlc.eventLock);
								wake_up_interruptible(&gHdlc.wq);
							}
						}
						else
						{
							log_warning("RR Final Frame: Invalid Nr, out-of-window: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pSframe->nr);
						}
					}
				}
				else if(pSframe->addr_cr == POLL_COMMAND)	/**< the command bit is 1, so it is a poll frame */
				{
					log_info("RR Poll Frame recvd");
					HDLC_STATS_INC(rx_poll_count);

					/// poll frame received, need to send our status to the remote end
					spin_lock(&gHdlc.eventLock);
					SET_EVENT(gHdlc.eventFlag, EVENT_TX_FINAL_FRAME);
					spin_unlock(&gHdlc.eventLock);
					wake_up_interruptible(&gHdlc.wq);
				}
			}
			break;

		default: /**< Invalid frame received (RNR or SREJ frame not used in our implementation) */
			log_warning("Invalid frame type received");
			break;
	}
	return;
}

/**
 * Process the I-frame
 *
 * @param  rx_frame	pointer to received frame
 * @param  len		size fo data
 * @return 1 if it a valid Iframe, 0 otherwise
 */
static UInt8 processIframe(UInt8* pRxFrame, Int32 len)
{
	tIframe *pIframe = (tIframe *) pRxFrame;
	UInt8 uRetVal=TRUE;
	static UInt8 prevSeq=0;
	UInt8	uSignalTxThread=FALSE;

	/// validate the ns of the received frame to check if we are expecting the same sequence no
	if(pIframe->ns != gHdlc.goBackN.rx_seq) /** < out-of-sequence frame */
	{
		log_info("OOS Frame received: exp=%d actual=%d",gHdlc.goBackN.rx_seq, pIframe->ns);
		HDLC_STATS_INC(rx_oos_count);

		if(prevSeq != gHdlc.goBackN.rx_seq)
		{
			if(pIframe->ns == prevSeq)
			{
				log_warning("Duplicate packet received: rx_seq=%d ns=%d",gHdlc.goBackN.rx_seq, pIframe->ns);
				HDLC_STATS_INC(dup_pkt_count);
				uRetVal = FALSE;
				/* ToDo: Check for efficiency */
				return uRetVal;
			}
		}

		/// out-of-sequence frame received, need to send a REJ frame with the expected frame no.
		if(gHdlc.goBackN.rejAlreadySent == FALSE)
		{
			gHdlc.goBackN.rejAlreadySent = TRUE;
			spin_lock(&gHdlc.eventLock);
			SET_EVENT(gHdlc.eventFlag, EVENT_TX_REJ_FRAME);
			spin_unlock(&gHdlc.eventLock);
			uSignalTxThread = TRUE;
		}

		/** verify the nr of the received frame to check for acknowledgement of the frames which we have sent.
		 *  the ack should be : ack_exp <= ack < tx_seq */
		if((pIframe->nr != INVALID_SEQ) && (gHdlc.goBackN.ack_exp != gHdlc.goBackN.tx_seq)) //window is not open (nothing to send, so not expecting ack)
		{
			log_info("OOS Frame: Piggy ack: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pIframe->nr);
			/// check if ack falls within the window
			if(validNr(&gHdlc.goBackN, pIframe->nr))
			{
				if(pIframe->nr == gHdlc.goBackN.ack_exp)
				{
					log_warning("OOS Frame: Piggy Ack: Invalid Nr, already acked: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pIframe->nr);
				}
				else
				{
					log_info("OOS frame: Valid piggy ack..nr=%d",pIframe->nr);

					/// valid ack, reset/stop the timer and update the acknowledge window
					gHdlc.goBackN.ack_exp = pIframe->nr;

					if(gHdlc.goBackN.ack_exp == gHdlc.goBackN.tx_seq)
					{
						/// stop the timer as all frames have been successfully acknowledged
						stopTimer();
						log_info("OOS Frame: Piggy Ack: Stop timer");
					}
					else
					{
						/// reset the timer as there are frames yet to be acknowledged
						resetTimer();
					}
					/** As we have got a valid ack, the window is now open, signal the tx thread so that if it has further frames to send,
					 * it can start transmitting */
					uSignalTxThread = TRUE;
				}
			}
			else
			{
				log_warning("OOS Frame: Piggy Ack: Invalid Nr, out of window: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pIframe->nr);
			}
		}
		if(uSignalTxThread == TRUE)
		{
			setEventDataAvailable();
		}
		uRetVal = FALSE;
	}
	else	/**< valid Iframe received, process the Iframe and call the IPC receive handler */
	{
		log_info("Valid Iframe received: exp=%d actual=%d",gHdlc.goBackN.rx_seq,pIframe->ns);
		HDLC_STATS_INC(rx_count);

		/// valid I-frame received, increment the receive seq no
		prevSeq = gHdlc.goBackN.rx_seq;
		IncSeqNo(gHdlc.goBackN.rx_seq, gHdlc.goBackN.window_size);
		gHdlc.goBackN.rejAlreadySent = FALSE;

		/// set the Rx_Iframe event and signal the tx thread to send ack fro the frame received
		spin_lock(&gHdlc.eventLock);
		SET_EVENT(gHdlc.eventFlag, EVENT_TX_ACK_FRAME);
		CLEAR_EVENT(gHdlc.eventFlag, EVENT_TX_REJ_FRAME);
		spin_unlock(&gHdlc.eventLock);
		//wake_up_interruptible(&gHdlc.wq);

		/** verify the nr of the received frame to check for acknowledgement of the frames which we have sent.
		 *  the ack should be : ack_exp <= ack < tx_seq */
		if((pIframe->nr != INVALID_SEQ) && (gHdlc.goBackN.ack_exp != gHdlc.goBackN.tx_seq)) /**< window is not open (nothing to send, so not expecting ack) */
		{
			log_info("I-Frame: Piggy ack: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pIframe->nr);
			/// check if ack falls within the window
			if(validNr(&gHdlc.goBackN, pIframe->nr))
			{
				if(pIframe->nr == gHdlc.goBackN.ack_exp)
				{
					log_warning("I-frame: Piggy Ack: Invalid Nr,already acked: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pIframe->nr);
				}
				else
				{
					log_info("I-frame: Valid piggy ack: nr=%d",pIframe->nr);

					/// valid ack, reset/stop the timer and update the acknowledge window
					gHdlc.goBackN.ack_exp = pIframe->nr;

					if(gHdlc.goBackN.ack_exp == gHdlc.goBackN.tx_seq)
					{
						/// stop the timer as all frames have been successfully acknowledged
						stopTimer();
						log_info("I-frame: Piggy Ack: Stop timer");
					}
					else
					{
						/// reset the timer as there are frames yet to be acknowledged
						resetTimer();
					}
				}
			}
			else
			{
				log_warning("I-frame: Piggy Ack: Invalid Nr,out of window: exp ack window[%d %d] nr=%d",gHdlc.goBackN.ack_exp,gHdlc.goBackN.tx_seq,pIframe->nr);
			}

		}
		setEventDataAvailable();
	}
	return uRetVal;
}


/**
 * Rx communication thread which handles receive functionality
 *
 * Receives the data from the serial layer, decodes the frame, verifies the checksum, determines the frame
 * type and processes the frame in a loop.
 *
 * @param  none
 * @param  none
 */
#ifdef	RX_COMM_THREAD
static int Rx_Comm_Thread(void* arg)
{
	Int32 iRet=0;
	struct i2c_msg msg;
	UInt8 uRxBuff[HDLC_FRAME_MAXLEN];
	UInt8* pData;
	UInt8 uFrameType;
	UInt16 wFrameLen;
	UInt32 uDataLen;

	allow_signal(SIGTERM);
	while(TRUE)
	{
		if(kthread_should_stop()) {
			log_notice("Thread %s: exiting", __func__);
			break;
		}
		/// check if slave wants to transmit, if yes process else block
		log_info("Waiting for slave-ready completion");
		iRet = wait_for_completion_interruptible(&gHdlc.slaveReady);
		if(!iRet) {
			log_info("ipc_i2c: Slave requested read.");
			memset(uRxBuff, 0, sizeof(uRxBuff));
			msg.addr = ipcRoot->client->addr;
			msg.flags = I2C_M_RD; /* Read */
			msg.len = HDLC_FRAME_MAXLEN;
			msg.buf = uRxBuff;
			log_info("ipc_i2c: Master performing I2C Read......1en=%d", HDLC_FRAME_MAXLEN);
			if( (iRet = i2c_transfer(ipcRoot->client->adapter, &msg, 1)) < 0 ) {
				log_error("ipc_i2c: i2c_transfer(read) failed error=%d", iRet);
			}
			else {
				log_info("ipc_i2c: I2C Read is complete");
				dumpBuffer(uRxBuff, 32, __FUNCTION__, __LINE__);
				if(checkValidFrame(uRxBuff, &wFrameLen) == TRUE)
				{
					log_info("Valid frame received, decoded frame size = %d",wFrameLen);
					//dumpBuffer(&uRxBuff[FRAME_LEN], wFrameLen, __FUNCTION__, __LINE__);
					pData = &uRxBuff[FRAME_LEN];

					/// Verify the CRC, if failed drop the frame, else process the frame */
					if(CheckCRC(pData, wFrameLen) == 0)
					{
						log_info("CRC check failed, frame dropped");
						HDLC_STATS_INC(crc_fail_count);
						//dumpBuffer(uRxBuff, uFrameLen, __FUNCTION__, __LINE__);
					}
					else
					{
						log_info("CRC check passed");
						uDataLen = wFrameLen - (CRC_SIZE + HDLC_FRAME_HEADER_LEN);

						///Determine Frame type
						uFrameType = pData[HDLC_CONTROL_OFFSET] & HDLC_FRAME_TYPE_MASK;;
						if(uFrameType == S_FRAME)
						{
							processSframe(pData, uDataLen);
						}
						else if(uFrameType == U_FRAME)
						{
							processUframe(pData);
						}
						else
						{
							if(processIframe(pData, uDataLen))
							{
								/// valid Iframe received signal the IPC Receive handler
								handlerProcessReceiveEvent(pData + HDLC_FRAME_HEADER_LEN, uDataLen);
							}
						}
					}//CRC
				}//Valid
				else
				{
					log_warning("Invalid frame received, decoded frame size = %d",wFrameLen);
					//dumpBuffer(uRxBuff, 32, __FUNCTION__, __LINE__);
				}
			}//i2c_transfer
		} else {
			log_notice("%s thread interrupted(%d)....exiting", __func__, iRet);
			break;
		}//wait_for_completion_interruptible
	}//while
	return EOK;
}
#else//RX_COMM_THREAD
void processRxPacket(UInt8* pData, UInt16 wFrameLen)
{
	UInt8* uRxBuff;
	UInt8 uFrameType;
	UInt32 uDataLen;

	/// Verify the CRC, if failed drop the frame, else process the frame */
	if(CheckCRC(pData, wFrameLen) == 0)
	{
		log_info("CRC check failed, frame dropped");
		HDLC_STATS_INC(crc_fail_count);
		//dumpBuffer(uRxBuff, uFrameLen, __FUNCTION__, __LINE__);
	}
	else
	{
		uRxBuff = pData;
		log_info("CRC check passed");
		uDataLen = wFrameLen - (CRC_SIZE + HDLC_FRAME_HEADER_LEN);

		///Determine Frame type
		uFrameType = uRxBuff[HDLC_CONTROL_OFFSET] & HDLC_FRAME_TYPE_MASK;;
		if(uFrameType == S_FRAME)
		{
			processSframe(uRxBuff, uDataLen);
		}
		else if(uFrameType == U_FRAME)
		{
			processUframe(uRxBuff);
		}
		else
		{
			if(processIframe(uRxBuff, uDataLen))
			{
				/// valid Iframe received signal the IPC Receive handler
				handlerProcessReceiveEvent(uRxBuff + HDLC_FRAME_HEADER_LEN, uDataLen);
			}
		}
	}
	return;
}
#endif


/**
 * This function initializes the HDLC layer
 *
 * @param  comDevice : com{Port to open
 *         mode      : Normal or loopbak mode
 *         syncFlag  : Not used
 *         comSpeed  : Baud rate
 * @return none
 */
Int32 hdlcInit(UInt8* comDevice, UInt8 mode, UInt8 syncFlag, UInt32 comSpeed)
{
	/// Initialize the global Hdlc structure
	memset(&gHdlc, 0, sizeof(gHdlc));

	gHdlc.syncDone = FALSE;

	/// disable transmissio, set the default window size to 8
	gHdlc.txEnable = FALSE;
	gHdlc.goBackN.window_size = DEFAULT_WINDOW_SIZE;

	/// Initialize the mutex
	spin_lock_init(&gHdlc.eventLock);
	init_waitqueue_head(&gHdlc.wq);
	init_completion(&gHdlc.slaveReady);

	/// Initialize the timer handling
	if(timerInit(IFRAME_TIMER_PERIOD_MSECS, UFRAME_TIMER_PERIOD_MSECS) != EOK)
	{
		log_error("Timer Init failed");
		return ERROR;
	}

	/// Create the Tx Comm thread
	gHdlc.tx_task = kthread_run(Tx_Comm_Thread, NULL, "TxCommThread");
	if (IS_ERR(gHdlc.tx_task)) {
		log_error("Tx Comm Thread creation failed: %ld", PTR_ERR(gHdlc.timer_task));
		return ERROR;
	}
	else {
		log_info("Tx Comm Thread Created successfully");
	}
#ifdef	RX_COMM_THREAD
	gHdlc.rx_task = kthread_run(Rx_Comm_Thread, NULL, "RxCommThread");
	if (IS_ERR(gHdlc.rx_task)) {
		log_error("Rx Comm Thread creation failed: %ld", PTR_ERR(gHdlc.timer_task));
		return ERROR;
	}
	else {
		log_info("Rx Comm Thread Created successfully");
	}
#endif
	hdlcInitDone = TRUE;
	return EOK;
}


/**
 * This function cleans all HDLC resources.
 *
 * @param  none
 * @return none
 */
void hdlcCleanup(void)
{
	int ret = 0;
	/* Cleanup the TxCommThread */
	if(gHdlc.tx_task) {
		spin_lock(&gHdlc.eventLock);
		SET_EVENT(gHdlc.eventFlag, EVENT_TX_WAKEUP);
		spin_unlock(&gHdlc.eventLock);
		kthread_stop(gHdlc.tx_task);
	}
#ifdef	RX_COMM_THREAD
	/* Cleanup the RxCommThread */
	if(gHdlc.rx_task) {
		if((ret = send_sig(SIGTERM, gHdlc.rx_task, 1)) != 0)
			log_warning("%s: send_sig failed(%d)", __func__, ret);
		kthread_stop(gHdlc.rx_task);
	}
#endif

	/* Cleanup the timer handling */
	timerCleanup();

	hdlcInitDone = FALSE;
//	memset(&gHdlc, 0, sizeof(gHdlc));

	return;
}
