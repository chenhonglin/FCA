/*****************************************************************************
 * Project              Harman Car Multimedia System
 *
 * c) copyright         2019
 * Company              Harman International Industries, Incorporated
 *                      All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file                ipc_debug.c
 * @author              David Rogala
 *
 * This module contains handlers for the /dev/ipcdebug interface
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fcntl.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/types.h>

#include "log.h"
#include "fs3ipc_cfg.h"
#include "fs3ipc.h"



#if DEBUG_FH

/*****************************************************************************
 * Externs
 *****************************************************************************/
extern const fs3ipc_app_LTconfig fs3ipc_app_appConfigs[];


/*****************************************************************************
 * Private functions
 *****************************************************************************/
static int toHexError = 0;
static char toHex(char hexValue)
{
	 if ((hexValue >= '0') && hexValue <= '9')
	 {
		  return (hexValue - '0');
	 }
	 if ((hexValue >= 'a') && hexValue <= 'f')
	 {
		  return (10 + (hexValue - 'a'));
	 }
	 if ((hexValue >= 'A') && hexValue <= 'F')
	 {
		  return (10 + (hexValue - 'A'));
	 }
	toHexError = 1;
	 return -1;
}

static ssize_t debug_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	const fs3ipc_app_StatsType *appStats = fs3ipc_app_GetStats(FS3IPC_INSTANCE_0);
	const fs3ipc_hdlc_StatsType *hdlcStats = fs3ipc_hdlc_GetStats(FS3IPC_INSTANCE_0);

	int channel = 0;
	int size;
	int i = 0;
	static char debugStr[16384] = {0};
	int read_bytes = 0;

	size = strlen(debugStr);

	if(size <= 0)
	{
		size = 0;

		i += snprintf(&debugStr[i], sizeof(debugStr) - i, "FS3IPC Debug:\n");

		if ((i >= 0) && (hdlcStats))
		{
			int size = snprintf(&debugStr[i], sizeof(debugStr) - i,
				"\nFS3IPC HDLC:\n"
				"  IC     |%12u|%12u|\n"
				"  SC     |%12u|%12u|\n"
				"  TR    |%12u|\n"
				"  RSeq  |%12u|\n"
				"  RInv  |%12u|\n"
				"  RPol  |%12u|\n"
				"  RRst  |%12u|\n"
				"  CRC    |%12u|\n"
				"  TExp   |%12u|\n",

				hdlcStats->TxIframeCnt, hdlcStats->RxIframeCnt,
				hdlcStats->TxSFrameCnt, hdlcStats->RxSFrameCnt,
				hdlcStats->TxRetransmit, hdlcStats->RxSequenceErrorCnt,
				hdlcStats->RxInvalidFrameCnt, hdlcStats->RxPollReqCnt,
				hdlcStats->RxResetReqCnt, hdlcStats->RxCrcErrorCnt,
				hdlcStats->TxAckTimeout);

			i = (size >= 0) ? (size + i) : (i);
		}

		if ((i >= 0) && (appStats))
		{
			int size = snprintf(&debugStr[i], sizeof(debugStr) - i,
				"\nFS3IPC APP:\n"
				"  WRAPPED MSG CNT   |%12u|%12u|%12u|\n"
				"  RXLERR            |%12u|\n"
				"  RXCHERR           |%12u|\n"
				"  RXGENERR          |%12u|%12u|\n"
				"  BUFOFERR          |%12u|\n\n",
				appStats->rxWrappedMsgCnt, appStats->txWrappedMsgCnt, appStats->rxWrappedMsgErrorCnt,
				appStats->rxMsgLengthErrorCnt,
				appStats->rxUnhandledChannelErrorCnt,
				appStats->rxGenErrorCnt, appStats->txGenErrorCnt,
				appStats->rxBufferOverflow);
			i = (size >= 0) ? (size + i) : (i);
		}

		if (i >= 0)
		{
			int size = snprintf(&debugStr[i], sizeof(debugStr) - i,
				"<Channel> |      <RXST>|     <RXCNT>|   <RXBUFOF>|   <RXOFCNT>|\n"
				"          |      <TXST>|     <TXCNT>| <TXMAXCNTF>|\n\n");
			i = (size >= 0) ? (size + i) : (i);
		}

		while( i >= 0 && channel < 32)
		{
			const fs3ipc_flowCtrlChanPBCfgType *chanStats = fs3ipc_app_GetChanStats(0, channel);

			if(chanStats)
			{
				int size = snprintf(&debugStr[i], sizeof(debugStr) - i,
					"Channel %02d|%12s|%12u|%12s|%12u|\n"
					"          |%12s|%12u|%12u|\n\n",
					channel, chanStats->rxEnabled ? "XON" : "XOFF",
					chanStats->rxCnt, chanStats->rxOverflow ? "TRUE" : "FALSE",
					chanStats->XoffCnt, chanStats->txEnabled ? "XON" : "XOFF",
					chanStats->txCnt, chanStats->maxTxPerFrame);
				i = (size >= 0) ? (size + i) : (i);
			}
			else
			{
				int size = snprintf(&debugStr[i], sizeof(debugStr) - i,
					"Channel %02d: N/A\n", channel);
				i = (size >= 0) ? (size + i) : (i);
				/* code */
			}

			channel++;
		}
		size = strlen(debugStr);
	}

	if (size > 0 && *f_pos <= size)
	{
		read_bytes = min(count, (strlen(debugStr) - (int)*f_pos));

		if (read_bytes > 0)
		{
			if (copy_to_user(buf, &debugStr[*f_pos], read_bytes)) {
				log_error("copy_to_user() could not copy");
				return -EFAULT;
			}

			*f_pos += read_bytes;

			if (*f_pos >= size)
			{
				/* all of stats are read. clear the buffer to trigger reforming
					the stats message*/
				debugStr[0] = '\0';
			}
		}
		else
		{
			read_bytes = 0;
		}

	}
	else
	{
		read_bytes = -1;
	}


	return read_bytes;
}

static ssize_t debug_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	int32_t ret = count;
	fs3ipc_StatusType ipcStat = fs3ipc_StatusType_OK;
	char debug_buffer[768];

	int channelNum;
	int log_level = 0;


	/* check inputs*/
	if (!filp || !buf) {
		log_error("Invalid args %p %p", filp, buf);
		ret = -EINVAL;
	}
	else if (count > sizeof (debug_buffer)) {
		log_error("buf too large %ld. Max = %ld", count, sizeof (debug_buffer));
		ret = -E2BIG;
	}

	if (ret >= 0) {

		if (copy_from_user(debug_buffer, buf, count) != 0) {
			log_error("copy_from_user failure");
			ret = -EFAULT;
		}
	}

	if (ret >= 0) {

		if (1 == sscanf(debug_buffer, "enableTrace(%d)", &channelNum)) {

			ipcStat = fs3ipc_app_SetLogging(0, channelNum, FS3IPC_TRUE);

			if (ipcStat != fs3ipc_StatusType_OK) {
				log_error("Error %d enabling chan %d logging", ipcStat, channelNum);
				ret = -EFAULT;
			}
			else {
				log_info("enableTrace: channel %d", channelNum);
			}

		}
		else if (1 == sscanf(debug_buffer, "disableTrace(%d)", &channelNum)) {

			ipcStat = fs3ipc_app_SetLogging(0, channelNum, FS3IPC_FALSE);

			if (ipcStat != fs3ipc_StatusType_OK) {
				log_error("Error %d enabling chan %d logging", ipcStat, channelNum);
				ret = -EFAULT;
			}
			else {
				log_info("enableTrace: channel %d", channelNum);
			}

		}
		else if (1 == sscanf(debug_buffer, "verbose(%d)", &log_level)) {
			gIpcLogLevel = log_level;
			log_info("verbose: %d", log_level);
		}
		else if (0 == strncmp(debug_buffer, "enableRawTrace()", sizeof("enableRawTrace()")-1)) {
			fs3ipc_phys_setLogging(FS3IPC_INSTANCE_0, FS3IPC_TRUE);
			log_info("enableRawTrace");
		}
		else if (0 == strncmp(debug_buffer, "disableRawTrace()", sizeof("disableRawTrace()")-1)) {
			fs3ipc_phys_setLogging(FS3IPC_INSTANCE_0, FS3IPC_FALSE);
			log_info("disableRawTrace");
		}
		else if (1 == sscanf(debug_buffer, "simulateRx(%d) ", &channelNum))
		{
			const fs3ipc_app_LTChannelCfgType* chanCfg = NULL;
			const fs3ipc_app_LTconfig* appCfg = &fs3ipc_app_appConfigs[0];
			fs3ipc_u8 parsed_msg[FS3_IPC_MAX_MSG_SIZE];
			int i=0;

			memset(&parsed_msg[0], 0, sizeof(parsed_msg));

			/* find channel */
			for (i=0; i < appCfg->channelCount; i++)
			{
				if (appCfg->ltChannelCfg[i] &&
					appCfg->ltChannelCfg[i]->channel == channelNum)
				{
					log_info ("SimulateRx: found channel %d ind %d", channelNum,
						i);
					chanCfg = appCfg->ltChannelCfg[i];
					break;
				}
			}

			/* was the channel identified*/
			if (!chanCfg)
			{
				log_error("SimulateRx: Invalid channel: %d", channelNum);
				ret = -EFAULT;
			}
			else
			{
				/* parse message*/
				int buffer_size = 0;

				i = 0;
				/* find ')'*/
				while (i < count)
				{
					if (debug_buffer[i] == ')')
					{
						i++;
						break;
					}
					i++;
				}

				// Convert the passed ASCII to hex and put it into the message
				// buffer

				while (i < count-1)
				{
					char c1,c2 = 0;
					c1 = toHex(debug_buffer[i]);
					c2 = toHex(debug_buffer[i+1]);

					if (c1 != -1 && c2 != -1)
					{
						parsed_msg[buffer_size++] = (c1 << 4) + c2;
						i+=2; /* read 2 chars. skip 1*/
					}
					else
					{
						i++;
					}
				}

				if (buffer_size == 0)
				{
					ret = -EFAULT;
					log_error("SimulateRx: channel %d - error in passed string %s ",
						channelNum, debug_buffer);
				}
				else
				{
					int qstat = 0;
					/* post message to Rx queue */
					fs3ipc_os_GetResource(chanCfg->RX_QLock);
					qstat = fs3ipc_app_QueuePush(chanCfg->RX_Q, &parsed_msg[0],
						buffer_size);
					fs3ipc_os_ReleaseResource(chanCfg->RX_QLock);

					if (fs3ipc_app_qStat_OK > qstat)
					{
						log_error("SimulateRx: Cannot queue chan %d message; err %d",
							channelNum, qstat);
						ret = -EFAULT;
					}

					if (ret >= EOK)
					{
						/* wakeup any reader threads */
						fs3ipc_os_SetEvent(chanCfg->clientCfg.osCfg,
							chanCfg->clientCfg.rxEvent);

						LOG_DATA (1,
							&(parsed_msg[0]), buffer_size,
							FS3_IPC_MAX_MSG_SIZE, "Simulate IPC: Rx C%02d L%03d", channelNum,
							buffer_size);
					}
				}
			}
		}
		else if (1 == sscanf(debug_buffer, "simulateTx(%d) ", &channelNum))
		{
			const fs3ipc_app_LTChannelCfgType* chanCfg = NULL;
			const fs3ipc_app_LTconfig* appCfg = &fs3ipc_app_appConfigs[0];
			fs3ipc_u8 parsed_msg[FS3_IPC_MAX_MSG_SIZE];
			int i=0;

			memset(&parsed_msg[0], 0, sizeof(parsed_msg));

			/* find channel */
			for (i=0; i < appCfg->channelCount; i++)
			{
				if (appCfg->ltChannelCfg[i] &&
					appCfg->ltChannelCfg[i]->channel == channelNum)
				{
					log_info ("SimulateTx: found channel %d ind %d", channelNum,
						i);
					chanCfg = appCfg->ltChannelCfg[i];
					break;
				}
			}

			/* was the channel identified*/
			if (!chanCfg)
			{
				log_error("SimulateTx: Invalid channel: %d", channelNum);
				ret = -EFAULT;
			}
			else
			{
				/* parse message*/
				int buffer_size = 0;

				i = 0;
				/* find ')'*/
				while (i < count)
				{
					if (debug_buffer[i] == ')')
					{
						i++;
						break;
					}
					i++;
				}

				// Convert the passed ASCII to hex and put it into the message
				// buffer
				while (i < count-1)
				{
					char c1,c2 = 0;
					c1 = toHex(debug_buffer[i]);
					c2 = toHex(debug_buffer[i+1]);

					if (c1 != -1 && c2 != -1)
					{
						parsed_msg[buffer_size++] = (c1 << 4) + c2;
						i+=2; /* read 2 chars. skip 1*/
					}
					else
					{
						i++;
					}
				}

				if (buffer_size == 0)
				{
					ret = -EFAULT;
					log_error("SimulateTx: channel %d - error in passed string %s ",
						channelNum, debug_buffer);
				}
				else
				{
					int qstat = 0;
					/* post message to Rx queue */
					fs3ipc_os_GetResource(chanCfg->TX_QLock);
					qstat = fs3ipc_app_QueuePush(chanCfg->TX_Q, &parsed_msg[0],
						buffer_size);
					fs3ipc_os_ReleaseResource(chanCfg->TX_QLock);

					if (fs3ipc_app_qStat_OK > qstat)
					{
						log_error("SimulateTx: Cannot queue chan %d message; err %d",
							channelNum, qstat);
						ret = -EFAULT;
					}

					if (ret >= EOK)
					{
						/* wakeup any reader threads */
						fs3ipc_os_SetEvent(appCfg->phys_txEventOsCfg,
							appCfg->phys_txEvent);
						LOG_DATA (1,
							&(parsed_msg[0]), buffer_size,
							FS3_IPC_MAX_MSG_SIZE, "Simulate IPC: Tx C%02d L%03d", channelNum,
							buffer_size);
					}
				}
			}
		}
		else
		{
			log_error("Invalid debug command");
			ret = -EINVAL;
		}
	}

	return ret;
}

static unsigned int debug_poll(struct file *filp, struct poll_table_struct *pt)
{
	return 	(POLLIN | POLLRDNORM);
}


static int debug_close(struct inode *inode, struct file *filp)
{
	return EOK;
}

static const struct file_operations fopsdebug = {
	.owner        = THIS_MODULE,
	.read         = debug_read,
	.write        = debug_write,
	.poll         = debug_poll,
	.release      = debug_close,
};


static int debug_open(struct inode *inode, struct file *filp)
{
	unsigned int    	minor = iminor(inode);

	log_debug("Opened the debug interface\n");

	/* and use filp->private_data to point to the device data */
	if (DEBUG_CHANNEL == minor)
	{
		filp->f_op = &fopsdebug;
		filp->f_pos = 0;
	}
	else
	{
		log_error( "%s: Invalid IPC debug channel/minor %u ",  __FUNCTION__, minor);
		return -EINVAL;
	}

	return 0;
}

const struct file_operations debug_fops = {
	.open = debug_open, /* just a selector for the real open */
};

#endif //DEBUG_FH
