/*
 * fs3ipc_cfg.h
 *
 *  Created on: Oct 22, 2018
 *      Author: DRogala
 */

#ifndef FS3IPC_CFG_H_
#define FS3IPC_CFG_H_

#include <linux/types.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/time.h>

#include "vlqueue.h"
#include "orrlist.h"
#include "fs3ipc_types.h"
#include "log.h"
#include "ipc.h"

extern int snprintf (char * buf, size_t size, const char * fmt, ...);
#define DbgPrint(a,b,c)

/*****************************************************************************
 * General configuration
 ****************************************************************************/
#define FS3IPC_NUM_OF_INSTANCES (1)
#define FS3IPC_INSTANCE_0 (0)
#define FS3_IPC_APP_GEN_NUM_PRIORITIES (10)
#define FS3_IPC_APP_MAX_NUMBER_CHANNELS (32)
#define FS3IPC_TRANS_SIZE_MAX (512)
#define LOG_DATA_MAX_BYTES (512)

#define fs3ipc_memset(dest,val,len)    memset(dest,val,len)
#define fs3ipc_memcpy(dest,src,len)    memcpy(dest,src,len)
#define fs3ipc_memcmp(dest,src,len)    memcmp(dest,src,len)

/* OS/Environment abstraction configuration*/
#define FS3IPC_OS_AUTOSAR (0)
#define FS3IPC_OS_POSIX (0)
#define FS3IPC_OS_UCOSII (0)
#define FS3IPC_OS_LINUX_KERNEL (1)

#define FS3IPC_PIC_EN (0)
#define FS3IPC_DATA_LAYER_ENABLE_TEST_SECTION (0)
#define FS3IPC_FC_LOG_EXT_CLIENT_STATUS (1)


/* events flags*/
#define EVENT_TX_TIMEOUT (1u << 0)
	/**< indicates that a valid frame has been received and it has to be
	 * acknowledged, need to send a RR ack frame */
#define EVENT_RX_IFRAME (1u << 1)
	/**< indicates that we have received an out-of-seq frame, need to send a
	 * REJ frame */
#define EVENT_RX_OUT_OF_SEQ_FRAME (1u << 2)
	/**< indicates that we have received a REJ frame and we need to retransmit
	 * the failed frames */
#define EVENT_RX_REJ_FRAME (1u << 3)
	/**< indicates that we have received a POLL frame, need to send our status
	 * by sending a final frame */
#define EVENT_RX_RR_POLL_FRAME (1u << 4)
	/**< indicates that we have received a Final frame and we need to retransmit
	 *  the failed frames */
#define EVENT_RX_RR_FINAL_FRAME (1u << 5)
	/**< indicates that we have received U FRAME ACK message, we need to respond
	 * ACK*/
#define EVENT_RX_ACK_UFRAME (1u << 6)
	/**< indicates that we have received U FRAME RST message, we need to respond
	 * ACK*/
#define EVENT_RX_RST_UFRAME (1u << 7)
#define EVENT_TX_MSG (1u << 8)
#define EVENT_TX_WAKEUP (1u << 9)
#define EVENT_EXT_WAKEUP (1u << 10)
#define EVENT_SUSPEND (1u << 11)
#define EVENT_RESUME (1u << 12)
#define EVENT_PHYS_TO (1u << 13)
#define EVENT_PHYS_XFER_COMP_0 (1u << 14)
#define EVENT_PHYS_XFER_COMP_1 (1u << 15)

/*****************************************************************************
 * os integration module configuration
 ****************************************************************************/
#define fs3ipc_os_getSystemTime(out) {\
	struct timespec ts = current_kernel_time();\
	*(out) = (fs3ipc_u32)((ts.tv_sec * 1000) + (ts.tv_nsec/1000000));\
}


/*****************************************************************************
 * Physical Layer
 ****************************************************************************/
#define FS3IPC_PHYS_NUM_OF_INSTANCES (FS3IPC_NUM_OF_INSTANCES)
#define FS3IPC_PHYS_STATS_ENABLED (1)

/* external type linkage*/
typedef int fs3ipc_phys_SpiSlaveHandleType;
/* external function linkage*/
#define fs3ipc_phys_ext_Encode(handle, buf, len)\
      fs3ipc_hdlc_Encode_Cb(handle, buf, len)
#define fs3ipc_phys_ext_Decode(handle, buf, len)\
      fs3ipc_hdlc_Decode_Cb(handle, buf, len)
#define fs3ipc_phys_ext_SpiAbort(handle)\
      ipc_SpiAbort(handle)
#define fs3ipc_phys_ext_PulseGPIO(pin)\
      ipc_PulseGPIO(pin)
#define fs3ipc_phys_ext_SetupSpiXfer(handle, in, out, size) \
      ipc_SetupSpiXfer(handle, in, out, size)

#define SPI_HANDLE_0 (0)
#define SPI_HANDLE_1 (1)

#define FS3IPC_PHYS_I0_SPI_PORTS_COUNT (2)

/*****************************************************************************
 * Data-link layer configuration - HDLC
 ****************************************************************************/
/* global */
#define FS3IPC_HDLC_NUM_OF_INSTANCES (FS3IPC_NUM_OF_INSTANCES)
#define FS3IPC_HDLC_STATS_ENABLED (1)
/* external function linkage*/
#define fs3ipc_hdlc_ext_Decode(handle, buf, len)\
		fs3ipc_app_Decoder(handle, buf, len)

#if FS3IPC_DATA_LAYER_ENABLE_TEST_SECTION == 1
#define fs3ipc_hdlc_ext_Encode(handle, buf, len)\
		fs3ipc_test_app_Encoder(handle, buf, len)
#else
#define fs3ipc_hdlc_ext_Encode(handle, buf, len)\
		fs3ipc_app_Encoder(handle, buf, len)
#endif

#define fs3ipc_hdlc_ext_transmit(uLldId, dataPtr, length)\
	fs3ipc_phys_Transfer(uLldId, dataPtr, length)

#define fs3ipc_hdlc_ext_CalculateCRC(buf, len)\
		CalculateCRC(buf, len)

#define fs3ipc_hdlc_ext_HandleExtNodeReset(handle) \
		fs3ipc_app_HandleExtNodeReset(handle)

#define fs3ipc_hdlc_ext_suspendComplete(handle) \
		ipc_suspend_complete(handle)

/*Only in case if the lock conditions are enabled*/
/**
 * The Define disables the interrupts
 */
/**< Disable interrupts */
#define HDLC_EVENT_LOCK() //SuspendAllInterrupts()
/**
 * The Define enables the interrupts
 */
/**< Return Interrupts to previous state */
#define HDLC_EVENT_UNLOCK() //ResumeAllInterrupts()

/* Instance 0 for hdlc*/
#define FS3IPC_HDLC_I0_HANDLE (0)

/* Instance 0 for loopback*/
#define FS3IPC_LOOPBACK_I0_HANDLE (0)

#define WINDOW_SIZE (5)
/*****************************************************************************
 * Application layer configuration
 ****************************************************************************/
/* global */
#define FS3IPC_APP_NUM_OF_INSTANCES (FS3IPC_NUM_OF_INSTANCES)
#define FS3IPC_APP_WRAPPED_MESSAGE_ENABLED (1)
#define FS3_IPC_MAX_MSG_SIZE (255)
#define FS3IPC_APP_STATS_ENABLED (1)
#define FS3IPC_APP_ALLOW_MSG_Q_WO_FC_CHECK (1)

/* external type linkage*/
typedef VLQueue_ccb_t fs3ipc_app_queueType;
typedef orrlist_node_t fs3ipc_app_priorityNodeType;
#define fs3ipc_app_qStat_OK VLQUEUE_OK                 /* must be 0*/
#define fs3ipc_app_qStat_full VLQUEUE_ERR_QUEUE_FULL   /* must be negative*/
#define fs3ipc_app_qStat_empty VLQUEUE_ERR_QUEUE_EMTPY /* must be negative*/

/* external function linkage*/
#define fs3ipc_app_QueueInit(context, buf, len)\
		vlqueue_init(context, buf, len)
#define fs3ipc_app_QueuePop(context, buf, len)\
		vlqueue_pop(context, buf, len)
#define fs3ipc_app_QueuePush(context, buf, len)\
		vlqueue_push(context, buf, len)
#define fs3ipc_app_QueueFrontSize(context)         vlqueue_fsize(context)
#define fs3ipc_app_QueueMsgCnt(context)            vlqueue_size(context)
#define fs3ipc_app_QueueFreeSpace(context)         vlqueue_bfree(context)
#define fs3ipc_app_QueueClear(context)             vlqueue_empty(context)
#define FS3IPC_APP_QUEUE_HDR_SIZE (2)

#define fs3ipc_app_PriorityNodeInit(node,cfg,prio)\
		orrlist_init_node(node,cfg,prio)
#define fs3ipc_app_PriorityNodeInsert(head,node)   orrlist_insert(head,node)
#define fs3ipc_app_PriorityNodeNext(node)          orrlist_next(node)

/* Instance 0*/
#define FS3IPC_APP_I0_HANDLE (0)

/* Section defined for testing operation configuration*/

/* Enables/disables the loopback mode
 * 1  : the module will copy the RX buffer to TX buffer
 * 0 : the module will operate normally
 */
#define FS3IPC_LOOPBACK_ENABLED      (0)

typedef enum
{
	fs3ipc_clientHandle_0_OnOff      = 1,
	fs3ipc_clientHandle_0_RTC,
	fs3ipc_clientHandle_0_NAV,
	fs3ipc_clientHandle_0_NAD,
	fs3ipc_clientHandle_0_HW,
	fs3ipc_clientHandle_0_DIAG,
	fs3ipc_clientHandle_0_VI_0,
	fs3ipc_clientHandle_0_VI_1,
	fs3ipc_clientHandle_0_Channel10,
	fs3ipc_clientHandle_0_Channel11,
	fs3ipc_clientHandle_0_Channel12,
	fs3ipc_clientHandle_0_Channel13,
	fs3ipc_clientHandle_0_Channel14,
	fs3ipc_clientHandle_0_Channel15,
	fs3ipc_clientHandle_0_Channel16,
	fs3ipc_clientHandle_0_Channel17,
	fs3ipc_clientHandle_0_Channel18,
	fs3ipc_clientHandle_0_Channel19,
	fs3ipc_clientHandle_0_Channel20,
	fs3ipc_clientHandle_0_Channel21,
	fs3ipc_clientHandle_0_Channel22,
	fs3ipc_clientHandle_0_Channel23,
	fs3ipc_clientHandle_0_Channel24,
	fs3ipc_clientHandle_0_Channel25,
	fs3ipc_clientHandle_0_Channel26,
	fs3ipc_clientHandle_0_Channel27,
	fs3ipc_clientHandle_0_Channel28,
	fs3ipc_clientHandle_0_Channel29,
	fs3ipc_clientHandle_0_Channel30,
	fs3ipc_clientHandle_0_Channel31,
	fs3ipc_clientHandleCount_All,
}fs3ipc_clientHandle_t;

#define fs3ipc_clientHandleCount_0 (fs3ipc_clientHandle_0_Channel25 - \
		fs3ipc_clientHandle_0_OnOff + 1)


/*****************************************************************************
 * Debugging
 *****************************************************************************/
/* print every trasnfer - requires a lot of overhead*/
#define FS3IPC_PRINT_EACH_TRANSFER 0

/* Supported logging levels*/
#define FS3IPC_LOG_LVL_EMERG     (0)
#define FS3IPC_LOG_LVL_ALERT     (1)
#define FS3IPC_LOG_LVL_CRIT      (2)
#define FS3IPC_LOG_LVL_ERROR     (3)
#define FS3IPC_LOG_LVL_WARNING   (4)
#define FS3IPC_LOG_LVL_NOTICE    (5)
#define FS3IPC_LOG_LVL_INFO      (6)
#define FS3IPC_LOG_LVL_DEBUG     (7)

/*Enables disables the debug print operation
 * 1 : enables the debug operation
 * 0 : disables the debug operation
 */
#define FS3IPC_OS_FS3IPC_DBG_PRINT   (1)

/* Controls which print statements are compiled.
 * Larger number = more verbose
 */
#define FS3IPC_LOG_LVL (FS3IPC_LOG_LVL_NOTICE)

/* helper macros*/
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

/*TODO move me to specific place to print debug*/

extern fs3ipc_u8 fs3ipc_gLogLevel;

#if FS3IPC_OS_FS3IPC_DBG_PRINT == 1
	#define LOG_EMERG(str, ...) log_error(str, ##__VA_ARGS__)
	#define LOG_ALERT(str, ...) log_error(str, ##__VA_ARGS__)
	#define LOG_CRIT(str, ...) log_error(str, ##__VA_ARGS__)
	#define LOG_ERROR(str, ...) log_error(str, ##__VA_ARGS__)

	#define LOG_WARNING(str, ...) log_warning(str, ##__VA_ARGS__)
	#define LOG_NOTICE(str, ...) log_notice(str, ##__VA_ARGS__)
	#define LOG_INFO(str, ...) log_info(str, ##__VA_ARGS__)
	#define LOG_DEBUG(str, ...) log_debug(str, ##__VA_ARGS__)

	#define LOG_DATA(enabled, data, length, maxlength, str, ...) \
		if (enabled) {\
			char sdata[(maxlength * 3) + 5]; \
			printData(sdata, sizeof(sdata), data, length, maxlength); \
			pr_notice(str ": %s\n", ##__VA_ARGS__, sdata);\
		}

	#define LOG_DATA_ERROR(enabled, data, length, maxlength, str, ...) \
		if (enabled) {\
			char sdata[(maxlength * 3) + 5]; \
			printData(sdata, sizeof(sdata), data, length, maxlength); \
			pr_err(str ": %s\n", ##__VA_ARGS__, sdata);\
		}
#else //FS3IPC_OS_FS3IPC_DBG_PRINT != 1

#define LOG_ERROR(str, ...)
#define LOG_WARNING(str, ...)
#define LOG_DEBUG(str, ...)
#define LOG_DATA(enabled, data, length, maxlength, str, ...)
#endif

/*****************************************************************************
 * External functions
 *****************************************************************************/

void app_0_CustomInit(void);
/*****************************************************************************
 * Supported module includes
 *****************************************************************************/
/*
 * all module/layer includes. These are necessary since it is not known by
 * each layer module what modules it will communicate with. This is specified
 * by this configuration file. These modules must be imported at the end of
 * the files since they may be dependant on configurations defined above.
 */
#include "../fs3ipc_os.h" /* os first*/
#include "../fs3ipc_app.h"
#include "../fs3ipc_phys.h"
#include "../fs3ipc_hdlc.h"


#endif /* FS3IPC_CFG_H_ */
