/*****************************************************************************
 *  Project          Harman Car Multimedia System
 *
 *  (c) copyright    2015
 *  Company          Harman International Industries, Incorporated
 *                   All rights reserved
 *  Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 *  @file      		hdlc_helper.c
 *  @ingroup    	Kernel and Drivers
 *  @author           	Bhagat
 *  @brief              Helper functions for Hdlc
 *
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include "hdlc.h"
#include "types.h"
#include "log.h"

/*****************************************************************************
 * FUNCTION DEFINITIONS
 *****************************************************************************/
/**
 * Form a Hdlc frame with header, data, CRC and trialer with byte stuffing
 *
 * @param  pData        pointer to input data
 * @param  iLen          size of the data
 * @param  pOutBuff     pointer to the encoded data (hdlc frame)
 * @return length of the encoded frame
 */
Int32 hdlcEncodeFrame (UInt8* pData, Int32 iLen)
{
	UInt16  wCrc;

	/// Calculate CRC on the original data including header
	wCrc = CalculateCRC(pData, iLen);
	pData[iLen+1] = (UInt8)(wCrc & 0x00ff);
	pData[iLen] = (UInt8)(wCrc >> 8);

	return EOK;
}

/**
 * Decode a Hdlc frame by removing byte stuffing, header and trailer
 *
 * @param  pData        pointer to input data
 * @param  uSize        size of the data
 * return  TRUE if frame is valid otherwise FALSE.
 */
Int32 checkValidFrame(UInt8 *pData, UInt16* uPktLen)
{
	UInt16 	uLen;
	Int32 	iRet=FALSE;

	uLen = *((UInt16*)&pData[0]);
	if((uLen < HDLC_FRAME_MINLEN) || (uLen > (HDLC_FRAME_MAXLEN - HDLC_FRAME_HEADER_OFFSET))) {
		log_warning("Invalid header/header, ignore the packet: %d",uLen);
	} else {
		*uPktLen = uLen;
		iRet = TRUE;
	}

	return iRet;
}

/**
 * Check if window is open for data to be sent
 *
 * @param  uAckExp 	seq no of the ack expected
 * @param  uTxSeq	seq no of the next frame to be transmitted
 * @return 1 if window is open, 0 otherwise
 */
UInt8 windowOpen(tGoBackN* goBackN)
{
	UInt8 uOccupiedWindow, uResult, uAckExp, uTxSeq, window_size;

	uAckExp = goBackN->ack_exp;
	uTxSeq = goBackN->tx_seq;
	window_size = goBackN->window_size;

	uOccupiedWindow = ((uTxSeq >= uAckExp) ? (uTxSeq - uAckExp) : (uTxSeq + window_size - uAckExp));
	log_info("occupied window = %d",uOccupiedWindow);
	uResult = uOccupiedWindow < (window_size-1);
	return uResult;
}

/**
 * Check if Nr (rcv seq no) is valid
 *
 * @param  uAckExp 	seq no of the ack expected
 * @param  uNr	 	the received seq no
 * @param  uTxSeq	seq no of the next frame to be transmitted
 * @return 1 if the Nr is valid, 0 otherwise
 */
UInt8 validNr(tGoBackN* goBackN, UInt8 uNr)
{
	UInt8 uRet=FALSE;
	UInt8 uAckExp, uTxSeq, window_size;

	uAckExp = goBackN->ack_exp;
	uTxSeq = goBackN->tx_seq;
	window_size = goBackN->window_size;

	if(uNr < window_size)
	{
		/// calculate if ack_exp<=nr<=tx_seq circularly
		if(((uAckExp <= uNr) && (uNr <= uTxSeq)) || ((uTxSeq < uAckExp) && (uAckExp <= uNr)) || ((uNr <= uTxSeq) && (uTxSeq < uAckExp)))
		{
			uRet = TRUE;
		}
	}
	return uRet;
}

/**
 * Create a Hdlc I-frame (fill the header)
 *
 * @param  pData 	pointer to data
 * @param  uSndSeq	seq no of the next frame to be transmitted
 * @param  uRcvSeq	seq no of the frames which we have received
 * @return none
 */
void hdlcCreateIframe(UInt8* pData, UInt8 uSndSeq, UInt8 uRcvSeq)
{
	tIframe *pIframe = (tIframe *) pData;
	memset(pIframe, 0, HDLC_FRAME_HEADER_LEN);
	pIframe->ns = uSndSeq;
	pIframe->nr = uRcvSeq;
	pIframe->pf = FALSE;
	pIframe->hdlc_0 = I_FRAME;
	return;
}

/**
 * Create a Hdlc RR Ack frame (S-frame)
 *
 * @param  uRcvSeq	the expected seq no of the next frame
 * @param  pSframe	pointer to S-frame
 * @return none
 */
void hdlcCreateRRAckFrame(UInt8 uRcvSeq, UInt8* pData, UInt32 qMaskUpdate)
{
	tSframe *pSframe = (tSframe *) pData;

	memset(pSframe, 0, sizeof(*pSframe));
	pSframe->nr = uRcvSeq;
	pSframe->pf = FALSE;
	pSframe->hdlc_01 = S_FRAME;
	pSframe->ss = RR_FRAME;
	//pSframe->qFlowCtrlMask = qMaskUpdate;
	return;
}

/**
 * Create a Hdlc REJ S-frame
 *
 * @param  uRcvSeq	the expected seq no of the next frame
 * @param  pSframe	pointer to S-frame
 * @return none
 */
void hdlcCreateREJFrame(UInt8 uRcvSeq, UInt8* pData)
{
	tSframe *pSframe = (tSframe *) pData;

	memset(pSframe, 0, sizeof(*pSframe));
	pSframe->nr = uRcvSeq;
	pSframe->pf = FALSE;
	pSframe->hdlc_01 = S_FRAME;
	pSframe->ss = REJ_FRAME;
	return;
}

/**
 * Create a Hdlc RR Poll frame (S-frame)
 *
 * @param  uRcvSeq	the expected seq no of the next frame
 * @param  pSframe	pointer to S-frame
 * @return none
 */
void hdlcCreateRRPollFrame(UInt8 uRcvSeq, UInt8* pData)
{
	tSframe *pSframe = (tSframe *) pData;

	memset(pSframe, 0, sizeof(*pSframe));
	pSframe->addr_cr = POLL_COMMAND;
	pSframe->nr = uRcvSeq;
	pSframe->pf = TRUE;
	pSframe->hdlc_01 = S_FRAME;
	pSframe->ss = RR_FRAME;
	return;
}

/**
 * Create a Hdlc RR Final frame (S-frame)
 *
 * @param  uRcvSeq	the expected seq no of the next frame
 * @param  pSframe	pointer to S-frame
 * @return none
 */
void hdlcCreateRRFinalFrame(UInt8 uRcvSeq, UInt8* pData)
{
	tSframe *pSframe = (tSframe *) pData;

	memset(pSframe, 0, sizeof(*pSframe));
	pSframe->addr_cr = POLL_RESPONSE;
	pSframe->nr = uRcvSeq;
	pSframe->pf = TRUE;
	pSframe->hdlc_01 = S_FRAME;
	pSframe->ss = RR_FRAME;
	return;
}

/**
 * Create a Hdlc ACK U-frame
 *
 * @param  pUframe	pointer to U-frame
 * @return none
 */
void hdlcCreateUAckFrame(tUframe* pUframe, UInt8 windowSize)
{
	memset(pUframe, 0, sizeof(*pUframe));
	pUframe->type_12 = 0b00;
	pUframe->type_345 = 0b11;
	pUframe->hdlc_11 = U_FRAME;
	pUframe->payload = windowSize;
	return;
}

/**
 * Create a Hdlc RST U-frame
 *
 * @param  pUframe	pointer to U-frame
 * @return none
 */
void hdlcCreateURstFrame(tUframe* pUframe, UInt8 windowSize)
{
	memset(pUframe, 0, sizeof(*pUframe));
	pUframe->type_12 = 0b11;
	pUframe->type_345 = 0b100;
	pUframe->hdlc_11 = U_FRAME;
	pUframe->payload = windowSize;
	return;
}
