/*****************************************************************************
 * Project              Harman Car Multimedia System
 *
 * c) copyright         2015
 * Company              Harman International Industries, Incorporated
 *                      All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file                ipc.c
 * @author              Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup             Kernel and Drivers
 *
 * This module contains the common thread-safe queue implementation.
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
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

#include "types.h"
#include "hdlc.h"
#include "log.h"
#include "queue.h"
#include "ipc.h"
#include "handler.h"
#include "flowControl.h"
#include "options.h"
#include "ipcProtocol.h"


/*****************************************************************************
 * VARIABLES
 *****************************************************************************/
static TDeviceAttr* pDevAttr_g = NULL;
static UInt8 uIsServEnabled = FALSE;
UInt32 gNoChannels=0;


/*****************************************************************************
 * FUNCTION DEFINITIONS
 *****************************************************************************/

/**
 * This function frees all resources.
 *
 * @return success
 */
void ipcMsgQueueCleanup(IPCAttr_t *ipcRoot )
{
	UInt32 uIdx=0;

	for (uIdx = 0; uIdx < (ipcRoot->uNoChannels + IPC_FREE_CHANNEL); uIdx++)
	{
		log_notice( "%s: For Node %u:-", __func__, uIdx);
		if(NULL != pDevAttr_g[uIdx].rxQueue) {
			queueDestroy(&(pDevAttr_g[uIdx].rxQueue));
			log_notice( "rxQ cleanedup: %p", pDevAttr_g[uIdx].rxQueue);
		}
		if(NULL != pDevAttr_g[uIdx].txQueue) {
			queueDestroy(&(pDevAttr_g[uIdx].txQueue));
			log_notice( "txQ cleanedup: %p", pDevAttr_g[uIdx].txQueue);
		}
	}

	/* destroy the mutex*/
	mutex_destroy(&pDevAttr_g[uIdx].accessMutex);

	if(NULL != pDevAttr_g)
		pDevAttr_g = NULL;
	return;
}


/**
 * This function enables the driver.
 *
 * @return none
 */
void ipcServerEnable( void )
{
	UInt8	uIndex;

	for (uIndex = 2; uIndex < (gNoChannels+IPC_FREE_CHANNEL); uIndex++)
	{
		queueClear((&pDevAttr_g[uIndex])->txQueue);
		queueClear((&pDevAttr_g[uIndex])->rxQueue);
	}

	/* ToDo: clear Tx nPending message count
	 * clearPendingTxMsgs();
	 */

	/* Enable flowCtrl */
	flowCtrlEnable();

	/* enable the IPC server */
	uIsServEnabled = TRUE;

	/* Trigger transmission of flowcontrol message */
	handlerTriggerTransmission(IPC_FLOWCTRL_CHANNEL);
	log_notice( "IPC Communication switched on");
}


/**
 * This function disables the driver.
 *
 * @return none
 */
void ipcServerDisable( void )
{
	flowCtrlDisable();
	uIsServEnabled = FALSE;
	log_info( "IPC Communication switched off");
}


/**
 * This function checks if the driver is enabled.
 *
 * @return TRUE or FALSE
 */
UInt8 ipcServerIsEnabled( void )
{
   return uIsServEnabled;
}

/**
 * This function returns a pointer to the given device
 * attribute structure.
 *
 * @param  device - device number (have to be in the range of 0 - 127)
 * @return device attribute pointer
 */
TDeviceAttr* getDeviceAttrPtr(UInt8 channel)
{
        UInt8 uIdx;

        for(uIdx=0; uIdx < (gNoChannels + IPC_FREE_CHANNEL); uIdx++)
        {
                if(channel == pDevAttr_g[uIdx].channelID)
                {
                        return &pDevAttr_g[uIdx];
                }
        }

        return NULL;
}

/**
 * This function returns a pointer to the given device
 * attribute structure.
 *
 * @param  device - device number (have to be in the range of 0 - 31)
 * @return device attribute pointer
 */
TDeviceAttr* getDeviceAttribute(UInt8 device)
{
	if ((pDevAttr_g == NULL) || (device >= IPC_MAX_CHANNELS))
	{
		return NULL;
	}

	return &pDevAttr_g[device];
}

static ssize_t channeln_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int            	ret = EOK;
	TDeviceAttr	*pDeviceAttr = NULL;
	TReadMsg* 	pReadMsg;

	pDeviceAttr = filp->private_data;
	if (NULL != pDeviceAttr) {
		log_info("Read for channel %d called", pDeviceAttr->channelID);
		/* check if the client has opened the file with read/rw permission */
		ret = (((filp->f_flags & O_ACCMODE) == O_RDWR) || ((filp->f_flags & O_ACCMODE) == O_RDONLY)) ? EOK : -EINVAL;
		if(ret == EOK) {
			/* if flowcontrol/watchdog channel return true */
			if ((IPC_FLOWCTRL_CHANNEL == pDeviceAttr->channelID) || (IPC_WATCHDOG_CHANNEL == pDeviceAttr->channelID)) {
				ret = EOK;
			}
			else if (pDeviceAttr->channelID >= IPC_WATCHDOG_CHANNEL && pDeviceAttr->channelID <= IPC_MAX_CHANNELS) {
				/* Check the RX queue status */
				if((pReadMsg=(TReadMsg*)queueGetReadPtr(pDeviceAttr->rxQueue)) == NULL) {
					/* check for non blocking mode */
					if (filp->f_flags & O_NONBLOCK) {
						ret = -EAGAIN;
						goto OUT;
					}
					else {
						log_info("waiting on channel_read");
						ret = wait_event_interruptible(pDeviceAttr->rxWq, (NULL != (pReadMsg=(TReadMsg*)queueGetReadPtr(pDeviceAttr->rxQueue))));
						if(ret){
							log_warning("channeln_read: wait_event_interruptible interrupted for channel-%d",pDeviceAttr->channelID);
							goto OUT;
						}
					}
				}
				ret = pReadMsg->frLen;
				log_info("DATA for CH-%d len = %d", pDeviceAttr->channelID, ret);

				/* return message to client */
				if (pReadMsg->frLen <= count) {
					if (copy_to_user (buf, (void*)pReadMsg->data, pReadMsg->frLen)) {
						log_error("channeln_read: copy_to_user error for channel-%d",pDeviceAttr->channelID);
						ret = -EFAULT;
						goto OUT;
					}
				}
				queueRemoveData(pDeviceAttr->rxQueue);  /* remove data from the queue */
				flowCtrlCheckRxState(pDeviceAttr->channelID);
			} else {
				ret = -EIO;
				log_error("%s: Invalid channelID(%u):", __func__,
							pDeviceAttr->channelID);
			}
		} else {
			log_error("%s: Invalid permissions(%ld):", __func__,
							filp->f_flags);
		}
	} else {
		ret = -EIO;
		log_error("%s: Invalid pDeviceAttr(%p):", __func__, pDeviceAttr);
	}

OUT:
	log_info("%s: ret %d", __func__, ret);
	return ret;
}

static ssize_t channeln_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	Int32 		ret = EOK;
	TDeviceAttr	*pDeviceAttr = NULL;
	TWriteMsg* 	pWriteMsg;

	pDeviceAttr = filp->private_data;
	if (NULL != pDeviceAttr) {
		if ((IPC_DATA_SIZE >= count)&& buf) {
			log_info("Write for channel %d called", pDeviceAttr->channelID);
			if (pDeviceAttr->channelID >= IPC_WATCHDOG_CHANNEL && pDeviceAttr->channelID <= IPC_MAX_CHANNELS) {
				/* check if the client has opened the file with write/rw permission */
				ret = (((filp->f_flags & O_ACCMODE) == O_RDWR) || ((filp->f_flags & O_ACCMODE) == O_WRONLY   ))?EOK:-EINVAL;
				if (ret == EOK) {
					ret = wait_event_interruptible(pDeviceAttr->txWq, (NULL != (pWriteMsg = (TWriteMsg*)queueGetWritePtr(pDeviceAttr->txQueue))));
					if(ret) {
						log_error("channeln_write: wait_event_interruptible error for channel-%d",pDeviceAttr->channelID);
						goto OUT;
					}

					/* store the client data to the write pointer */
					pWriteMsg->frLen = count;           /* store message length */
					pWriteMsg->frame_type = 0;           	/* reserved */
					pWriteMsg->channel = pDeviceAttr->channelID; /* store channel */

					/* read client data and store them */
					if((ret = copy_from_user(pWriteMsg->data, buf, count))) {
						log_error("channeln_write: copy_from_user error for channel-%d",pDeviceAttr->channelID);
						ret = -EFAULT;
						goto OUT;
					} else {
						log_info("Add data to TXQ for channel-%d", pDeviceAttr->channelID);
						/* add data to queue, increment counter for pending TX messages and tell the
						 * transmission thread that new messages are available */
						queueAddData(pDeviceAttr->txQueue);
						handlerTriggerTransmission(pDeviceAttr->channelID);
						ret = count;
					}
				} else {
					log_error("%s: Invalid permissions(%ld):", __func__,
							filp->f_flags);
				}
			} else {
				ret = -EIO;
				log_error("%s: Invalid channelID(%u):", __func__,
							pDeviceAttr->channelID);
			}
		} else {
			ret = -EINVAL;
			log_error("%s: Invalid Arguments for channel-%d, buf = "
					"%p, count = %zd", __func__,
					pDeviceAttr->channelID, buf, count);
		}
	} else {
		ret = -EIO;
		log_error("%s: Invalid pDeviceAttr(%p):", __func__, pDeviceAttr);
	}

OUT:
	return ret;
}

static unsigned int channeln_poll(struct file *filp, struct poll_table_struct *pt)
{
	unsigned int 	mask = 0;
	TDeviceAttr	*pDeviceAttr = NULL;
	return 0;
	pDeviceAttr = filp->private_data;

	if (NULL != pDeviceAttr) {

		poll_wait(filp, &pDeviceAttr->pollWq, pt);

		if (FALSE == queueIsEmpty(pDeviceAttr->rxQueue)) {
			mask |= POLLIN | POLLRDNORM;
		}
	}
	return mask;
}

static int channeln_close(struct inode *inode, struct file *filp)
{
	TDeviceAttr	*pDeviceAttr = NULL;
	UInt32		minor = iminor(inode);

	log_info("Close for device minor = %d called", minor);
	pDeviceAttr = filp->private_data;
	if (NULL != pDeviceAttr)
	{
		mutex_lock(&pDeviceAttr->accessMutex);
		if (pDeviceAttr->clients) {
			pDeviceAttr->clients--; /* release the device */
			mutex_unlock(&pDeviceAttr->accessMutex);
		}

		/* Trigger the flowcontrol message if the tx queue is empty */
		if(queueIsEmpty(pDeviceAttr->txQueue))
			handlerTriggerTransmission(IPC_FLOWCTRL_CHANNEL);

		log_notice("%s: IPC channel %u closed for minor %d", __func__,
				pDeviceAttr->channelID, minor);
	}
	else {
		log_warning("%s: For minor %d, device attribute is invalid.",
				__func__, pDeviceAttr->channelID);
	}
	return 0;
}

static const struct file_operations fopschanneln = {
	.owner        = THIS_MODULE,
	.read         = channeln_read,
	.write        = channeln_write,
	.poll         = channeln_poll,
	.release      = channeln_close,
};

static const struct {
        unsigned int minor;
        char *name;
        const struct file_operations *fops;
} devlist[] = { /* list of minor devices */
#ifndef	VTP
        {2,  "ipc-ivi2", &fopschanneln},
        {3,  "ipc-ivi3", &fopschanneln},
        {4,  "ipc-ivi4", &fopschanneln},
        {5,  "ipc-ivi5", &fopschanneln},
        {6,  "ipc-ivi6", &fopschanneln},
        {7,  "ipc-ivi7", &fopschanneln},
#else
	{2,  "ipc-vtp2", &fopschanneln},
        {3,  "ipc-vtp3", &fopschanneln},
        {4,  "ipc-vtp4", &fopschanneln},
        {5,  "ipc-vtp5", &fopschanneln},
        {6,  "ipc-vtp6", &fopschanneln},
        {7,  "ipc-vtp7", &fopschanneln},
        {8,  "ipc-vtp8", &fopschanneln},
        {9,  "ipc-vtp9", &fopschanneln},
#endif
};

static int channeln_open(struct inode *inode, struct file *filp)
{
	Int32 		result=0;
	TDeviceAttr	*pDeviceAttr = NULL;
	UInt32		minor = iminor(inode);

	log_info("Open for device minor = %d called", minor);

	/* and use filp->private_data to point to the device data */
	if (minor >= CHANNEL2_MINOR && minor < IPC_MAX_CHANNELS)
	{
		pDeviceAttr = container_of(inode->i_cdev, TDeviceAttr, cdev);
		filp->private_data = pDeviceAttr;
		filp->f_op = &fopschanneln;
	}
	else
	{
		log_error( "%s: Invalid IPC channel/minor %u ",  __FUNCTION__, minor);
		return -EINVAL;
	}

	pDeviceAttr = container_of(inode->i_cdev, TDeviceAttr, cdev);

	mutex_lock(&pDeviceAttr->accessMutex);
	if (pDeviceAttr->clients) {
		log_warning( "%s: IPC channel/minor %u already opened",  __FUNCTION__, minor);
		mutex_unlock(&pDeviceAttr->accessMutex);
		return -EBUSY; /* already open */
	}
	pDeviceAttr->clients++;
	mutex_unlock(&pDeviceAttr->accessMutex);

	log_notice("%s: IPC channel %u opened for minor %d",  __func__,
			pDeviceAttr->channelID, minor);
	handlerTriggerTransmission(IPC_FLOWCTRL_CHANNEL);
	return result;
}

static const struct file_operations channel_fops = {
	.owner = THIS_MODULE,
	.open = channeln_open, /* just a selector for the real open */
};

void ipcDeviceCleanup(IPCAttr_t *ipcRoot )
{
	UInt32 uIdx=0;

	for(uIdx=0; uIdx < ipcRoot->uNoChannels; uIdx++)
	{
		device_destroy(ipcRoot->dev_class, MKDEV(MAJOR(ipcRoot->ipc_chdev), devlist[uIdx].minor));
		cdev_del(&ipcRoot->deviceAttr[uIdx + IPC_FREE_CHANNEL].cdev);
	}
	class_destroy(ipcRoot->dev_class);
	unregister_chrdev_region(ipcRoot->ipc_chdev, IPC_MAX_CHANNELS - IPC_FREE_CHANNEL);
	if (ipcRoot->deviceAttr != NULL) {
		kfree(ipcRoot->deviceAttr);
	}

	return;
}


/**
 * This function creates a resource manager.
 *
 * @return 0 == failure | 1 == success
 */
Int32 ipcDeviceInit(IPCAttr_t *ipcRoot)
{
	Int32 result = EOK,i=0;
	dev_t devt;
	TDeviceAttr* pDeviceAttr=NULL;
	UInt8	uDevIdx=0;

	result = alloc_chrdev_region(&ipcRoot->ipc_chdev, 2, IPC_MAX_CHANNELS, IPC_STR);
	if (result != 0) {
		log_error("ipcDeviceInit: alloc_chrdev_region failed: %d", result);
		return ERROR;
	}

	ipcRoot->dev_class = class_create(THIS_MODULE, IPC_STR);
	ipcRoot->uNoChannels = sizeof(devlist)/sizeof(devlist[0]);
	gNoChannels = ipcRoot->uNoChannels;

        ipcRoot->deviceAttr = kzalloc((ipcRoot->uNoChannels + IPC_FREE_CHANNEL) * sizeof(TDeviceAttr), GFP_KERNEL);
	if (!ipcRoot->deviceAttr) {
		log_error("ipcDeviceInit: Memory allocation failed");
		return -ENOMEM;
	}

	pDeviceAttr = &ipcRoot->deviceAttr[0];

	for (i = 0; i < ipcRoot->uNoChannels; i++) {

		/* create device nodes for all channels */
		if (devlist[i].minor < IPC_MAX_CHANNELS)
		{
			uDevIdx = i + IPC_FREE_CHANNEL;
			cdev_init(&ipcRoot->deviceAttr[uDevIdx].cdev, &channel_fops);

			devt = MKDEV(MAJOR(ipcRoot->ipc_chdev), devlist[i].minor);
			result = cdev_add(&ipcRoot->deviceAttr[uDevIdx].cdev, devt, 1);
			if (result < 0) {
				log_error("Error %d adding cdev for channel %u", result, i);
				return ERROR;
			}

			device_create(ipcRoot->dev_class, NULL, devt, NULL, devlist[i].name);
			pDeviceAttr[uDevIdx].channelID = devlist[i].minor;
		}
		else
		{
			log_error("ipcDeviceInit: Invalid channel specified");
			return ERROR;
		}
	}

	/* Assign the pDevAttr_g pointer */
	pDevAttr_g = &ipcRoot->deviceAttr[0];

	return EOK;
}

/**
 * This function initializes the resource manager.
 *
 * @return 0 == failure | 1 == success
 */
Int32 ipcMsgQueueInit( IPCAttr_t *ipcRoot )
{
	Int32 		status = EOK;
	TDeviceAttr* 	pDeviceAttr = NULL;
	UInt8 		idx;

	/* Create the queue for the configured channels */
	pDeviceAttr = &ipcRoot->deviceAttr[0];
	if (NULL != pDeviceAttr)
	{
		for (idx = 0; idx < (ipcRoot->uNoChannels + IPC_FREE_CHANNEL); idx++)
		{
			if (IPC_FLOWCTRL_CHANNEL == idx)
			{
				log_debug("Flow control channel");
				pDeviceAttr[idx].rxQueue = NULL;
				pDeviceAttr[idx].txQueue = NULL;
				pDeviceAttr[idx].channelID = IPC_FLOWCTRL_CHANNEL;
			}
			else if (IPC_WATCHDOG_CHANNEL == idx)
			{
				log_debug("Watchdog channel");
				pDeviceAttr[idx].rxQueue = NULL;
				pDeviceAttr[idx].txQueue = NULL;
				pDeviceAttr[idx].channelID = IPC_WATCHDOG_CHANNEL;
			}
			else
			{
				pDeviceAttr[idx].rxQueue = queueCreate(sizeof(TReadMsg), IPC_RX_QUEUE_SIZE);
				if(pDeviceAttr[idx].rxQueue == NULL) {
					log_error("%s: rx Queue Create failed for channel-%d", __func__, pDeviceAttr->channelID);
					status = ERROR;
					break;
				}
				pDeviceAttr[idx].txQueue = queueCreate(sizeof(TWriteMsg), IPC_TX_QUEUE_SIZE);
				if(pDeviceAttr[idx].txQueue == NULL) {
					log_error("%s: tx Queue Create failed for channel-%d", __func__, pDeviceAttr->channelID);
					status = ERROR;
					break;
				}

				init_waitqueue_head(&pDeviceAttr[idx].txWq);
				init_waitqueue_head(&pDeviceAttr[idx].rxWq);
			}
			pDeviceAttr[idx].clients = 0;
			pDeviceAttr[idx].prevClients = 0;

			mutex_init(&pDeviceAttr[idx].accessMutex);
			mutex_init(&pDeviceAttr[idx].txFlowCtrlMutex);
			mutex_init(&pDeviceAttr[idx].rxFlowCtrlMutex);
		}
	}

	return status;
}
