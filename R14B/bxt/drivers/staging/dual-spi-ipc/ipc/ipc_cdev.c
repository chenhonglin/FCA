/*****************************************************************************
 * Project              Harman Car Multimedia System
 *
 * c) copyright         2019
 * Company              Harman International Industries, Incorporated
 *                      All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file                ipc_cdev.c
 * @author              David Rogala <David Rogala@harman.com>

 * This module provides a user space interface to the fs3ipc driver using
 * character devices.
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
#include <linux/slab.h>

#include "log.h"
#include "fs3ipc_cfg.h"
#include "fs3ipc.h"

/*****************************************************************************
 * VARIABLES
 *****************************************************************************/
extern const fs3ipc_app_LTconfig fs3ipc_app_appConfigs[];

/* static function declarations*/
static ssize_t channeln_read(struct file *filp, char __user *buf, size_t count,
	loff_t *f_pos);
static ssize_t channeln_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos);
static unsigned int channeln_poll(struct file *filp,
	struct poll_table_struct *pt);
static int channeln_close(struct inode *inode, struct file *filp);


static TDeviceAttr* pDevAttr_g = NULL;

static uint8_t uIsServEnabled = FALSE;
uint32_t gNoChannels=0;

#ifdef DEBUG_FH
static struct cdev         debug_cdev;
#endif

static const struct file_operations fopschanneln = {
	.owner        = THIS_MODULE,
	.read         = channeln_read,
	.write        = channeln_write,
	.poll         = channeln_poll,
	.release      = channeln_close,
};

const static uint8_t channelClientHandleLookup [IPC_MAX_CHANNELS] = {
	INVALID_CHANNEL_ID,
	INVALID_CHANNEL_ID,
	fs3ipc_clientHandle_0_OnOff,
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
	fs3ipc_clientHandle_0_Channel31
};

static const struct {
	unsigned int minor;
	char *name;
	const struct file_operations *fops;
} devlist[] = { /* list of minor devices */
	{2,  "ipc2", &fopschanneln},
	{3,  "ipc3", &fopschanneln},
	{4,  "ipc4", &fopschanneln},
	{5,  "ipc5", &fopschanneln},
	{6,  "ipc6", &fopschanneln},
	{7,  "ipc7", &fopschanneln},
	{8,  "ipc8", &fopschanneln},
	{9,  "ipc9", &fopschanneln},
	{10, "ipc10", &fopschanneln},
	{11, "ipc11", &fopschanneln},
	{12, "ipc12", &fopschanneln},
	{13, "ipc13", &fopschanneln},
	{14, "ipc14", &fopschanneln},
	{15, "ipc15", &fopschanneln},
	{16, "ipc16", &fopschanneln},
	{17, "ipc17", &fopschanneln},
	{18, "ipc18", &fopschanneln},
	{19, "ipc19", &fopschanneln},
	{20, "ipc20", &fopschanneln},
	{21, "ipc21", &fopschanneln},
	{22, "ipc22", &fopschanneln},
	{23, "ipc23", &fopschanneln},
	{24, "ipc24", &fopschanneln},
	{25, "ipc25", &fopschanneln},
	{26, "ipc26", &fopschanneln},
	{27, "ipc27", &fopschanneln},
	{28, "ipc28", &fopschanneln},
	{29, "ipc29", &fopschanneln},
	{30, "ipc30", &fopschanneln},
	{31, "ipc31", &fopschanneln},
};

/*****************************************************************************
 * FUNCTION DEFINITIONS
 *****************************************************************************/
static int32_t ipc_cdev_init(IPCAttr_t *ipcRoot);


/**
 * This function enables the driver.
 *
 * @return none
 */
void ipcServerEnable( IPCAttr_t *ipcRoot )
{
	int i;
	dev_t devt;

	/* enable the IPC server */
	uIsServEnabled = TRUE;

	/* create sysfs node for each ipc channel and debug cdev, if enabled.
	 * This must be done after setting uIsServEnabled, otherwise there is a
	 * potential race condition where a user space client may perform a cdev
	 * call before uIsServEnabled is set to 1, causing an error to be
	 * returned*/
	for (i = 0; i < ipcRoot->uNoChannels; i++) {
		/* create device nodes for all channels */
		if (devlist[i].minor < IPC_MAX_CHANNELS) {
			devt = MKDEV(MAJOR(ipcRoot->ipc_chdev),
				devlist[i].minor);
			device_create(ipcRoot->dev_class, NULL, devt, NULL,
				devlist[i].name);
		}
	}

#ifdef DEBUG_FH
	devt = MKDEV(MAJOR(ipcRoot->ipc_chdev), DEBUG_CHANNEL);
	device_create(ipcRoot->dev_class, NULL, devt, NULL, DEBUG_CHANNEL_NAME);
#endif


	log_notice( "IPC Communication switched on");
}


/**
 * This function disables the driver.
 *
 * @return none
 */
void ipcServerDisable( void )
{
	uIsServEnabled = FALSE;
	log_info( "Communication switched off");
}


/**
 * This function checks if the driver is enabled.
 *
 * @return TRUE or FALSE
 */
uint8_t ipcServerIsEnabled( void )
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
TDeviceAttr* getDeviceAttrPtr(uint8_t channel)
{
	uint8_t uIdx;

	for(uIdx=0; uIdx < (gNoChannels + IPC_FREE_CHANNEL); uIdx++) {
		if(channel == pDevAttr_g[uIdx].channelID) {
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
TDeviceAttr* getDeviceAttribute(uint8_t device)
{
	if ((pDevAttr_g == NULL) || (device >= IPC_MAX_CHANNELS)) {
		return NULL;
	}

	return &pDevAttr_g[device];
}

int ipc_channel_read(int chnum, char *buf, size_t count, int flags)
{
	int ret = EOK;
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;
	char localBuffer[FS3_IPC_MAX_MSG_SIZE];
	fs3ipc_length msgLen = FS3_IPC_MAX_MSG_SIZE;

	int clhndl;

	if (!uIsServEnabled) {
		return -ENODEV;
	}

	clhndl = channelClientHandleLookup[chnum];

	if (clhndl == INVALID_CHANNEL_ID) {
		log_error( "%s: Invalid IPC channel %u ",  __FUNCTION__, chnum);
		return -EINVAL;
	}

	/* clamp msgLen to the lesser of the two */
//	fs3ipc_length msgLen = (FS3_IPC_MAX_MSG_SIZE < count) ?
//		FS3_IPC_MAX_MSG_SIZE : count;


	while (fs3ipc_StatusType_BufferEmpty ==
			(stat = fs3ipc_ReadMessage (clhndl, localBuffer,
			&msgLen))) {
		if (flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		} else {
			if (fs3ipc_StatusType_OK !=
				(fs3ipc_WaitForMessage (clhndl))) {
				log_error("%s: chan %d: "
					"fs3ipc_WaitForMessage error",
					__FUNCTION__, chnum);
				ret = -EAGAIN;
				break;
			}
		}
	}

	/* DRR todo: check for error related to output buffer size. It
	   is expected that the output buffer is large enough to consume
	    the entire message. This can be specified in user space*/

	if((stat != fs3ipc_StatusType_OK) &&
			(stat != fs3ipc_StatusType_BufferEmpty)) {
		log_error("%s: chan %d: fs3ipc_ReadMessage error %d ",
			__FUNCTION__, chnum, stat);
		ret = -EFAULT;
	} else if(ret == -EAGAIN) {
		/* do nothing. Polling mode and the queue is empty*/
	} else {
		/* success*/
		ret = msgLen;
	}

	if (ret > 0) {
		if (msgLen > count) {
			//printk(KERN_ERR "Expected %d bytes, received %d "
			//	"bytes\n", count, msgLen);
			//printk(KERN_ERR "Bytes are 0x%x, 0x%x, 0x%x, 0x%x, "
			//	"0x%x\n", localBuffer[0], localBuffer[1],
			//	localBuffer[2], localBuffer[3], localBuffer[4]);
			ret = count;
		}
		memcpy(buf, localBuffer, ret);
	}

	//printk(KERN_INFO "%s: ret %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(ipc_channel_read);

int ipc_channel_read_timeout(int chnum, char *buf, size_t count,
		int timeoutInMsec)
{
	int ret = EOK;
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;
	char localBuffer[FS3_IPC_MAX_MSG_SIZE];
	int clhndl;
	fs3ipc_length msgLen;

	if (!uIsServEnabled) {
		return -ENODEV;
	}

	clhndl = channelClientHandleLookup[chnum];
	if (clhndl == INVALID_CHANNEL_ID) {
		log_error( "%s: Invalid IPC channel %u ",  __FUNCTION__, chnum);
		return -EINVAL;
	}

	/* clamp msgLen to the lesser of the two */
//	fs3ipc_length msgLen = (FS3_IPC_MAX_MSG_SIZE < count) ?
//		FS3_IPC_MAX_MSG_SIZE : count;
	msgLen = FS3_IPC_MAX_MSG_SIZE;

	while (fs3ipc_StatusType_BufferEmpty ==
			(stat = fs3ipc_ReadMessage (clhndl, localBuffer,
			&msgLen))) {
		if (fs3ipc_StatusType_OK !=
				(fs3ipc_WaitForMessageTimeout (clhndl,
				timeoutInMsec))) {
//			log_error("%s: chan %d: fs3ipc_WaitForMessage "
//				"error",
//				__FUNCTION__, chnum);
			ret = -EAGAIN;
			break;
		}
	}

	/* DRR todo: check for error related to output buffer size.
	   It is expected that the output buffer is large enough to
	   consume the entire message. This can be specified in user
	   space*/

	if((stat != fs3ipc_StatusType_OK) &&
			(stat != fs3ipc_StatusType_BufferEmpty)) {
		log_error("%s: chan %d: fs3ipc_ReadMessage error %d ",
			__FUNCTION__, chnum, stat);
		ret = -EFAULT;
	} else if(ret == -EAGAIN) {
		/* do nothing. Polling mode and the queue is empty*/
	} else {
		/* success*/
		ret = msgLen;
	}

	if (ret > 0) {
		if (msgLen > count) {
			printk(KERN_ERR "Expected %ld bytes, received %d "
				"bytes\n", count, msgLen);
			printk(KERN_ERR "Bytes are 0x%x, 0x%x, 0x%x, 0x%x, "
				"0x%x\n", localBuffer[0], localBuffer[1],
				localBuffer[2], localBuffer[3], localBuffer[4]);
			ret = count;
		}
		memcpy(buf, localBuffer, ret);
	}

	//printk(KERN_INFO "%s: ret %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(ipc_channel_read_timeout);

static ssize_t channeln_read(struct file *filp, char __user *buf, size_t count,
	loff_t *f_pos)
{
	int            	ret = EOK;
	TDeviceAttr	*pDeviceAttr = NULL;
	//fs3ipc_StatusType stat = fs3ipc_StatusType_OK;
	uint8_t msg[FS3_IPC_MAX_MSG_SIZE];

	pDeviceAttr = filp->private_data;
	if (NULL == pDeviceAttr) {
		log_error("null pDeviceAttr");
		ret = -EFAULT;
	} else if (NULL == buf) {
		log_error("null buffer");
		ret = -EFAULT;
	} else if (((filp->f_flags & O_ACCMODE) != O_RDWR) &&
		((filp->f_flags & O_ACCMODE) != O_RDONLY)) {
		log_error("%s: chan %d: invalid flags %08x ", __FUNCTION__,
			pDeviceAttr->channelID, filp->f_flags);
		ret = -EINVAL;
	}

	if (ret == EOK) {
		ret = ipc_channel_read(pDeviceAttr->channelID, msg, count,
			filp->f_flags);
	}

	if ( ret > 0) {
		if (copy_to_user (buf, (void*)msg, ret)) {
			log_error("%s: chan %d: copy_to_user error ",
				__FUNCTION__, pDeviceAttr->channelID);
			ret = -EFAULT;
		}
	}

	//printk(KERN_INFO "%s: ret %d\n", __func__, ret);
	return ret;
}


int ipc_channel_write(int chnum, const char *buf, size_t count)
{
	int32_t 		ret = EOK;
	fs3ipc_StatusType stat;
	int clhndl;

	if (!uIsServEnabled) {
		return -ENODEV;
	}

	clhndl = channelClientHandleLookup[chnum];
	if (clhndl == INVALID_CHANNEL_ID) {
		log_error( "%s: Invalid IPC channel %u ",  __FUNCTION__, chnum);
		return -EINVAL;
	}

	if ((!count) || (count > FS3_IPC_MAX_MSG_SIZE)) {
		log_error("%s: chan %d: msg size%ld ",
			__FUNCTION__, chnum, count);
		ret = -EFAULT;
	} else if (fs3ipc_StatusType_OK !=
		(stat = fs3ipc_WriteMessage (clhndl, buf, count))) {
		log_warning("%s: chan %d: fs3ipc_WriteMessage error %d ",
			__FUNCTION__, chnum, stat);
		ret = -EFAULT;
	} else {
		ret = count;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(ipc_channel_write);

static ssize_t channeln_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	int32_t 		ret = EOK;
	TDeviceAttr	*pDeviceAttr = NULL;
	//TWriteMsg* 	pWriteMsg;
	uint8_t msg[FS3_IPC_MAX_MSG_SIZE];
	//fs3ipc_StatusType stat;

	pDeviceAttr = filp->private_data;

	if((!pDeviceAttr) || (!buf)) {
		/* invalid args*/
		ret = -EFAULT;
	} else if (((filp->f_flags & O_ACCMODE) != O_RDWR) &&
		((filp->f_flags & O_ACCMODE) != O_WRONLY)) {
		log_error("%s: chan %d: invalid flags %x ",
			__FUNCTION__, pDeviceAttr->channelID, filp->f_flags);
		ret = -EFAULT;
	} else if ((!count) || (count > FS3_IPC_MAX_MSG_SIZE)) {
		log_error("%s: chan %d: msg size%ld ",
			__FUNCTION__, pDeviceAttr->channelID, count);
		ret = -EFAULT;
	} else if (copy_from_user (msg, (void*)buf, count)) {
		log_error("channeln_write: copy_from_user error for "
			"channel-%d\n",pDeviceAttr->channelID);
		ret = -EFAULT;
	} else {
		ret = ipc_channel_write(pDeviceAttr->channelID, msg, count);
	}

	return ret;
}

static unsigned int channeln_poll(struct file *filp,
	struct poll_table_struct *pt)
{
	unsigned int mask = 0;
	TDeviceAttr	*pDeviceAttr = filp->private_data;
	fs3ipc_StatusType stat;
	fs3ipc_u32 result = 0;

	if (NULL == pDeviceAttr) {
		log_error("%s: chan %d: NULL pDeviceAttr ", __FUNCTION__,
			pDeviceAttr->channelID);
		mask = POLLERR;
	} else if (pDeviceAttr->clientRxWq == NULL) {
		log_error("%s: chan %d: NULL clientRxWq ", __FUNCTION__,
			pDeviceAttr->channelID);
		mask = POLLERR;
	}

	if (mask != POLLERR) {
		poll_wait (filp, pDeviceAttr->clientRxWq, pt);

		if (fs3ipc_StatusType_OK ==
			(stat = fs3ipc_GetMessagePending (
				pDeviceAttr->clientHandle, &result)))
		{
			if (result > 0)
			{
				mask = POLLIN | POLLRDNORM | POLLOUT |
					POLLWRNORM;
			} else {
				/* DRR: Write is always asynchronous*/
				mask = POLLOUT | POLLWRNORM;
			}
		} else {
			log_error("%s: chan %d: fs3ipc_GetMessagePending %d ",
				__FUNCTION__, pDeviceAttr->channelID, stat);
			mask = POLLERR;
		}
	}

	return mask;
}

int ipc_channel_close(int chnum)
{
	uint8_t		clhndl;
	fs3ipc_StatusType stat;

	if (!uIsServEnabled) {
		return -ENODEV;
	}

	clhndl = channelClientHandleLookup[chnum];
	if (clhndl == INVALID_CHANNEL_ID) {
		log_error( "%s: Invalid IPC channel %u ",  __FUNCTION__, chnum);
		return -EINVAL;
	}

	stat = fs3ipc_CloseChannel (clhndl);
	if (stat != fs3ipc_StatusType_OK) {
		log_error( "%s: fs3ipc_CloseChannel error %d - channel %u ",
			__FUNCTION__, stat, chnum);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ipc_channel_close);

static int channeln_close(struct inode *inode, struct file *filp)
{
	TDeviceAttr	*pDeviceAttr = NULL;
	uint32_t		minor = iminor(inode);
	int rc = 0;

	pDeviceAttr = filp->private_data;
	if (NULL != pDeviceAttr) {
		log_info("IPC channel %u closed for minor %d",
			pDeviceAttr->channelID, minor);

		rc = ipc_channel_close(minor);
	}
	return rc;
}

int ipc_channel_open(int ch_num)
{
	uint8_t		clhndl;
	fs3ipc_StatusType stat;

	if (!uIsServEnabled) {
		return -ENODEV;
	}

	clhndl = channelClientHandleLookup[ch_num];
	if (clhndl == INVALID_CHANNEL_ID) {
		log_error( "%s: Invalid IPC channel/minor %u ",  __FUNCTION__,
			ch_num);
		return -EINVAL;
	}

	stat = fs3ipc_OpenChannel (clhndl);
	if (stat == fs3ipc_StatusType_ErrorClientState) {
		return -EBUSY; /* already open */
	} else if (stat != fs3ipc_StatusType_OK) {
		log_error("%s: fs3ipc_EnableChannel error %d - channel %d "
			, __FUNCTION__, stat, ch_num);
		return -EINVAL;
	}
	return 0;
}

EXPORT_SYMBOL_GPL(ipc_channel_open);

static int device_open(struct inode *inode, struct file *filp)
{
	int32_t 		result=0;
	TDeviceAttr	*pDeviceAttr = NULL;
	uint32_t		minor = iminor(inode);

	/* and use filp->private_data to point to the device data */
	if (minor >= CHANNEL2_MINOR && minor < IPC_MAX_CHANNELS) {
		pDeviceAttr = container_of(inode->i_cdev, TDeviceAttr, cdev);
		filp->private_data = pDeviceAttr;
		filp->f_op = &fopschanneln;

	} else {
		log_error( "%s: Invalid IPC channel/minor %u ",  __FUNCTION__,
			minor);
		return -EINVAL;
	}

	result = ipc_channel_open(minor);

	if (result == 0) {
		log_info("%s: IPC channel %u opened for minor %d",
			__FUNCTION__, pDeviceAttr->channelID, minor);
	}
	return result;
}

static const struct file_operations channel_fops = {
	.owner = THIS_MODULE,
	.open = device_open, /* just a selector for the real open */
};

void ipcDeviceCleanup(IPCAttr_t *ipcRoot )
{
	uint32_t uIdx=0;

	for(uIdx=0; uIdx < ipcRoot->uNoChannels; uIdx++) {
		device_destroy(ipcRoot->dev_class,
			MKDEV(MAJOR(ipcRoot->ipc_chdev), devlist[uIdx].minor));
		cdev_del(&ipcRoot->deviceAttr[uIdx + IPC_FREE_CHANNEL].cdev);
	}
	class_destroy(ipcRoot->dev_class);
	unregister_chrdev_region(ipcRoot->ipc_chdev,
		IPC_MAX_CHANNELS - IPC_FREE_CHANNEL);
	return;
}


/**
 * This function creates a resource manager.
 *
 * @return 0 == failure | 1 == success
 */
int32_t ipcDeviceInit(IPCAttr_t *ipcRoot)
{
	int32_t result = EOK,i=0;
	dev_t devt;
	TDeviceAttr* pDeviceAttr;
	uint8_t	uDevIdx=0;


	result = alloc_chrdev_region(&ipcRoot->ipc_chdev, 2, IPC_MAX_CHANNELS,
		"ioc-ipc");
	if (result < 0) {
		log_error("ipcDeviceInit: alloc_chrdev_region failed");
		return -1;
	}

	ipcRoot->dev_class = class_create(THIS_MODULE, "ioc-ipc");
	ipcRoot->uNoChannels = sizeof(devlist)/sizeof(devlist[0]);
	gNoChannels = ipcRoot->uNoChannels;

	ipcRoot->deviceAttr = kzalloc((ipcRoot->uNoChannels + IPC_FREE_CHANNEL)
		* sizeof(TDeviceAttr), GFP_KERNEL);
	if (!ipcRoot->deviceAttr) {
		log_error("ipcDeviceInit: Memory allocation failed\n");
		return -ENOMEM;
	}

	pDeviceAttr = &ipcRoot->deviceAttr[0];

	for (i = 0; i < ipcRoot->uNoChannels; i++) {
		/* create device nodes for all channels */
		if (devlist[i].minor < IPC_MAX_CHANNELS) {
			uDevIdx = i + IPC_FREE_CHANNEL;
			cdev_init(&ipcRoot->deviceAttr[uDevIdx].cdev,
				&channel_fops);

			devt = MKDEV(MAJOR(ipcRoot->ipc_chdev),
				devlist[i].minor);
			result = cdev_add(&ipcRoot->deviceAttr[uDevIdx].cdev,
				devt, 1);
			if (result < 0) {
				log_error("Error %d adding cdev for channel "
					"%u\n", result, i);
				return -1;
			}

			pDeviceAttr[uDevIdx].channelID = devlist[i].minor;
		} else {
			log_error("ipcDeviceInit: Invalid channel specified\n");
			return -1;
		}
	}

#ifdef DEBUG_FH
	cdev_init(&debug_cdev, &debug_fops);
	devt = MKDEV(MAJOR(ipcRoot->ipc_chdev), DEBUG_CHANNEL);
	result = cdev_add(&debug_cdev, devt, 1);
	if (result < 0) {
		log_error("Error %d adding cdev for debug channel\n", result);
		return -1;
	}

#endif

	/* Assign the pDevAttr_g pointer */
	pDevAttr_g = &ipcRoot->deviceAttr[0];

	return ipc_cdev_init(ipcRoot);
}

/**
 * This function initializes the resource manager.
 *
 * @return 0 == failure | 1 == success
 */
static int32_t ipc_cdev_init( IPCAttr_t *ipcRoot )
{
	int32_t           status = EOK;
	TDeviceAttr*    pDeviceAttr = NULL;
	uint8_t           idx;

	/* Create the queue for the configured channels */
	pDeviceAttr = &ipcRoot->deviceAttr[0];
	if (NULL != pDeviceAttr) {
		for (idx = 0; idx < (ipcRoot->uNoChannels + IPC_FREE_CHANNEL);
			idx++) {

			const fs3ipc_app_LTconfig* appCfg =
				&fs3ipc_app_appConfigs[0];

			if (idx < appCfg->channelCount) {
				const fs3ipc_app_LTChannelCfgType* chanCfg =
						appCfg->ltChannelCfg[idx];
				if (chanCfg) {
					pDeviceAttr[idx].clientRxWq =
						&chanCfg->clientCfg.osCfg->wq;
					pDeviceAttr[idx].clientHandle =
						channelClientHandleLookup[idx];
				} else {
					pDeviceAttr[idx].clientRxWq = NULL;
					pDeviceAttr[idx].clientHandle =
						INVALID_CHANNEL_ID;
				}

			} else {
				pDeviceAttr[idx].clientRxWq = NULL;
				pDeviceAttr[idx].clientHandle =
					INVALID_CHANNEL_ID;
			}

			if(!pDeviceAttr[idx].clientRxWq) {
				log_error("ipcMsgQueueInit: chan %d: NULL "
					"clientRxWq", idx);
			}
			if (pDeviceAttr[idx].clientHandle ==
				INVALID_CHANNEL_ID) {
				log_error("ipcMsgQueueInit: chan %d: "
					"clientHandle=%d", idx,
					pDeviceAttr[idx].clientHandle);
			}
		}
	}

	return status;
}
