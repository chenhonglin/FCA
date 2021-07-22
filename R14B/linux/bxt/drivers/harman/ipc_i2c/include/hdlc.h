/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	hdlc.h
 * @author        	Bhagat Singh
 * @ingroup       	Kernel and Drivers
 *
 * This header file is used to handle HDLC functionalities
 */

#ifndef HDLC_H
#define HDLC_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include <linux/mutex.h>
#include <linux/sched.h>
#ifdef	IPC_STATS
#include <linux/seq_file.h>
#endif
#include <linux/i2c.h>


#include "types.h"
#include "log.h"
#include "ipcProtocol.h"
#include "hal_serial.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/

#define	RX_COMM_THREAD
#define WINDOW_SIZE_4			5
#define WINDOW_SIZE_3			3
#define WINDOW_SIZE_2			2
#define DEFAULT_WINDOW_SIZE		WINDOW_SIZE_3
#define IncSeqNo(i,window_size)		{(i) = ((i)+1)%(window_size);}

#define IFRAME_TIMER_PERIOD_SECS	0
#define IFRAME_TIMER_PERIOD_MSECS	100
#define UFRAME_TIMER_PERIOD_SECS	0
#define UFRAME_TIMER_PERIOD_MSECS	500

#define FRAME_LEN			2
#define CRC_SIZE			2
#define HDLC_FRAME_HEADER_LEN		3
#define HDLC_FRAME_HEADER_OFFSET	FRAME_LEN
#define HDLC_FRAME_DATA_OFFSET		HDLC_FRAME_HEADER_OFFSET + HDLC_FRAME_HEADER_LEN

#define HDLC_CONTROL_OFFSET		1
#define HDLC_FRAME_TYPE_MASK		3
#define INVALID_SEQ			0x7f

#define HDLC_FRAME_ALIGN		16	/**< Hdlc frames are aligned to 16 bytes before transmit */
#define ALIGN_16			16	/**< Hdlc frames are aligned to 16 bytes before transmit */
#define ALIGN_32			32	/**< Hdlc Poll frame requires 32 byte alignment */

#define RTXQ_BUFLEN			(IPC_BUFFER_SIZE + HDLC_FRAME_HEADER_LEN + CRC_SIZE) + HDLC_FRAME_HEADER_OFFSET
#define HDLC_FRAME_MAXLEN		(IPC_BUFFER_SIZE + HDLC_FRAME_HEADER_LEN + CRC_SIZE) + HDLC_FRAME_HEADER_OFFSET		/**< max length of Hdlc frame without bytestuffing (header+crc+data) */
#define HDLC_FRAME_MINLEN		(HDLC_FRAME_HEADER_LEN + CRC_SIZE)					/**< min length of Hdlc frame */


//Event flags
#define EVENT_TX_WINDOW_OPEN		0	/**< indicates that the tx window is open and we can send more data */
#define EVENT_TX_TIMEOUT		1 	/**< indicates that a timeout has happened waiting for acknowledgement, need to send a poll frame */
#define EVENT_TX_ACK_FRAME		2	/**< indicates that a valid frame has been received and it has to be acknowledged, need to send a RR ack frame */
#define EVENT_TX_REJ_FRAME		3	/**< indicates that we have received an out-of-seq frame, need to send a REJ frame */
#define EVENT_TX_FINAL_FRAME		4	/**< indicates that we have received a POLL frame, need to send our status by sending a final frame */
#define EVENT_RETRANSMIT_REJ_FRAME	5	/**< indicates that we have received a REJ frame, and we need to retransmit the failed frames */
#define EVENT_RETRANSMIT_FINAL_FRAME	6	/**< indicates that we have received a Final frame and we need to retransmit the failed frames */
#define EVENT_TX_UFRAME_RST		7 	/**< indicates that we have to send a Uframe RST */
#define EVENT_TX_UFRAME_ACK		8	/**< indicates that we have received a Uframe RST and we need to reply with a Uframe ACK */
#define EVENT_TX_WAKEUP			9	/**< indicates that we have received a Uframe RST and we need to reply with a Uframe ACK */
#define EVENT_SLAVE_READY		10	/**< indicates that we have received a Uframe RST and we need to reply with a Uframe ACK */

///Frame types
#define I_FRAME		0x0
#define S_FRAME		0x1
#define U_FRAME		0x3

//Types of S-frame
#define RR_FRAME	0x00
#define REJ_FRAME	0x01
#define RNR_FRAME	0x10
#define SREJ_FRAME	0x11

//Types of U-frame
#define URST_FRAME	19
#define UACK_FRAME	12

#define POLL_COMMAND	0x1
#define POLL_RESPONSE	0x0

#define TEST_EVENT(var,pos)     (var & (1 << pos))
#define SET_EVENT(var,pos)      (var |= (1 << pos))
#define CLEAR_EVENT(var,pos)    (var &= ~(1 << pos))

#define IPC_TEST_MODE   0x1
#define IPC_RESYNC_FLAG 0x1


/*****************************************************************************
 * STRUCTURES
 *****************************************************************************/

typedef struct
{
	UInt8 data[RTXQ_BUFLEN];  	/**< retransmission queue buffer */
	UInt32 datalen;			/**< size of the data */
}tRtxQueue;

typedef struct __attribute__((packed)) {
	UInt8 addr_sap:6;		/**< hdlc station address, not used for point-point */
	UInt8 addr_cr:1;		/**< command/response bit used to identify a poll/final frame */
	UInt8 addr_ext:1;		/**< not used */
	UInt8 hdlc_0:1;			/**< used to identify an I-frame */
	UInt16 ns:7;			/**< transmit seq no */
	UInt16 pf:1;			/**< pf bit used in poll/final frame */
	UInt16 nr:7;			/**< receive seq no */
}tIframe;

typedef struct __attribute__((packed)) {
	UInt8 addr_ext:6;		/**< hdlc station address, not used for point-point */
	UInt8 addr_cr:1;		/**< command/response bit used to identify a poll/final frame */
	UInt8 addr_sap:1;		/**< not used */
	UInt16 hdlc_01:2;		/**< used to identify a S-frame */
	UInt16 ss:2;			/**< used to identify the type of s-frame */
	UInt16 na:4;			/**< not used */
	UInt16 pf:1;			/**< pf bit used in poll/final frame */
	UInt16 nr:7;			/**< receive seq no */
	UInt32 qFlowCtrlMask;
}tSframe;

typedef struct __attribute__((packed)) {
	UInt8 addr;			/**< hdlc station address, not used for point-point */
	UInt8 hdlc_11:2;		/**< used to identify a U-frame */
	UInt8 type_12:2;		/**< used to identify the type of U-frame */
	UInt8 pf:1;			/**< pf bit used in poll/final frame */
	UInt8 type_345:3;		/**< used to identify the type of U-frame */
	UInt8 payload;			/**< payload for Uframe */
}tUframe;

typedef struct {
	UInt8 tx_seq;			/**< sequence number of next frame to be sent */
	UInt8 ack_exp;			/**< sequence number of the first unacknowledged frame (ack expected) */
	UInt8 rx_seq;			/**< Sequence number of next frame expected to be received (frame expected)*/
	UInt8 rejAlreadySent;		/**< 0 => Can send Nak. 1 => Can't send Nak */
	UInt8 window_size;		/**< window size of hdlc protocol */
}tGoBackN;

typedef struct {
	UInt32 tx_count;			/**< no of packets transmitted */
	UInt32 tx_retransmit_count;		/**< no of packets retransmitted */
	UInt32 tx_timeout_count;		/**< timer expiry count */
	UInt32 rx_count;			/**< no of packets received */
	UInt32 rx_oos_count;			/**< no of oos packets received */
	UInt32 crc_fail_count;			/**< crc failure count */
	UInt32 tx_ack_count;			/**< no of ACK frames transmitted */
	UInt32 tx_rej_count;			/**< no of REJ frames transmitted */
	UInt32 tx_final_count;			/**< no of FINAL frames transmitted */
	UInt32 tx_poll_count;			/**< no of POLL frames transmitted */
	UInt32 rx_ack_count;			/**< no of ACK frames received */
	UInt32 rx_rej_count;			/**< no of REJ frames received */
	UInt32 rx_final_count;			/**< no of FINAL frames received */
	UInt32 rx_poll_count;			/**< no of POLL frames received */
	UInt32 tx_piggy_ack_count;		/**< no of piggy acks sent */
	UInt32 dup_pkt_count;			/**< no of duplicate packets recvd */
}tHdlcStats;


typedef struct {
	tRtxQueue	rtxQueue[DEFAULT_WINDOW_SIZE];
	tIframe		iFrame;
	tSframe		sFrame;
	tUframe		uFrame;
	tGoBackN	goBackN;
	tHdlcStats	hdlcStats;
	struct mutex	mutex;		/**< mutex for handling synchronization between tx and rx comm thread */
	spinlock_t	eventLock;
	wait_queue_head_t wq;
	UInt16	 	eventFlag;	/**< describes the events that are to be processed */
	UInt32	 	syncDone;	/**< indicates that Soc and Ioc have finished synchronization */
	UInt8		txEnable;	/**< indicates that we can transmit packets */

	struct completion slaveReady;
	struct task_struct *tx_task;
#ifdef	RX_COMM_THREAD
	struct task_struct *rx_task;
#else
	UInt8		uSlaveReady;	/**< indicates that we can transmit packets */
#endif
	struct task_struct *timer_task;
}tHdlc;


/*****************************************************************************
 * VARIABLES
 *****************************************************************************/

extern tHdlc gHdlc;
#define HDLC_STATS_INC(member)	{gHdlc.hdlcStats.member ++;}


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

Int32 timerInit(int delay_in_ms, int uframe_delay_in_ms);
void timerCleanup(void);
void stopTimer(void);
void resetTimer(void);
void stopUframeTimer(void);
void resetUframeTimer(void);

UInt8 windowOpen(tGoBackN* goBackN);
UInt8 validNr(tGoBackN* goBackN, UInt8 Nr);
void hdlcCreateIframe(UInt8* pData, UInt8 snd_seq, UInt8 rcv_seq);
void hdlcCreateREJFrame(UInt8 rcv_seq, UInt8* pSframe);
void hdlcCreateRRPollFrame(UInt8 rcv_seq, UInt8* pSframe);
void hdlcCreateRRFinalFrame(UInt8 rcv_seq, UInt8* pSframe);
void hdlcCreateRRAckFrame(UInt8 rcv_seq, UInt8* pSframe, UInt32 qMaskUpdate);
void hdlcCreateURstFrame(tUframe* pUframe, UInt8 windowSize);
void hdlcCreateUAckFrame(tUframe* pUframe, UInt8 windowSize);

Int8 CheckCRC(const UInt8 *pData, Int32 datalen );
UInt16 CalculateCRC(const UInt8 *pBuf, Int32 iBufLen);

Int32 checkValidFrame(UInt8* pRxFrame, UInt16* len);
Int32 hdlcEncodeFrame (UInt8* pData, Int32 len);
void processRxPacket(UInt8* pData, UInt16 wFrameLen);
void hdlcDecodeFrame(UInt8* pRxFrame, UInt16* len);
UInt8 hdlcDecodeFrameType(UInt8* pRxFrame);

void setEventDataAvailable(void);
Int32 hdlcInit(UInt8*, UInt8, UInt8, UInt32);
void  hdlcCleanup(void);
#ifdef	IPC_STATS
void HdlcStatisticsPrint(void);
#else
void HdlcStatisticsPrint(struct seq_file *m);
#endif


#endif /* HDLC_H */
