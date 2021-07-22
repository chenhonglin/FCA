/*****************************************************************************
 * Project              Harman Car Multimedia System
 *
 * c) copyright         2019
 * Company              Harman International Industries, Incorporated
 *                      All rights reserved
 * Secrecy Level    	STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file                ipc.c
 * @author              Bhagat Singh <bhagat.singh@harman.com>
 *                      Prasad Lavande <prasad.lavande@harman.com>
 * @ingroup             Kernel and Drivers
 *
 * This module contains the IPC-I2C implementation between SoC-DU.
 */


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/slab.h>
#ifndef	IPC_STATS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif
#include <linux/i2c.h>

#include "ipc.h"
#include "types.h"
#include "hdlc.h"
#include "log.h"
#include "options.h"
#include "handler.h"
#include "hdlc.h"
#include "flowControl.h"
#include "ipcDev.h"
#include "statistics.h"


#define	IPC_VERSION		"0.2"

/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

#ifndef	IPC_STATS
static int ipc_proc_open(struct inode *inode, struct  file *file);
void createProcEntry(void);
#endif


/*****************************************************************************
 * VARIABLES
 *****************************************************************************/

IPCAttr_t* ipcRoot=NULL;
static int IpcInitialized = FALSE;
#ifndef	IPC_STATS
static const struct file_operations ipc_proc_fops = {
	.owner = THIS_MODULE,
	.open = ipc_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif


/*****************************************************************************
 * FUNCTION DEFINITIONS
 *****************************************************************************/

#ifndef	IPC_STATS
static int ipc_proc_show(struct seq_file *m, void *v) {
	statisticsPrint(m, ipcRoot);
	HdlcStatisticsPrint(m);
	return 0;
}

static int ipc_proc_open(struct inode *inode, struct  file *file) {
	return single_open(file, ipc_proc_show, NULL);
}

void createProcEntry(void)
{
	proc_create(IPC_STR"-stats", 0, NULL, &ipc_proc_fops);
}
#endif

static int ipc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	log_notice("ipc_i2c_probe called");
	log_notice("ipc_i2c_probe: Supported functionalities by the adapter : 0x%x", i2c_get_functionality(client->adapter));
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		log_error("ipc_i2c_probe: I2C_FUNC_I2C functionality not present for given adapter");
		/*	ToDo: check for failure
		return -EIO;
		*/
	}
	if(ipcRoot == NULL) {
		ipcRoot = kzalloc(sizeof(IPCAttr_t), GFP_KERNEL);
		if (!ipcRoot) {
			log_error("ipc_i2c_probe: Memory allocation failed");
			return -ENOMEM;
		}
	}

	ipcRoot->client = client;

	if(EOK != halInit(client))
	{
		log_error("ipc_i2c_probe: hal init failed");
		goto ipcfail0;
	}

/*	ToDo: When multiple ipc_i2cs, use following:
        if(dev->master->bus_num == 2) {
                ipcRoot->spidev1 = dev;
                log_info("bus no = 2");
                if(EOK != halInit(dev))
                {
                        log_error("ipc_i2c_probe: hal init failed");
                        goto ipcfail0;
                }
        }
        else {
                log_info("Invalid bus number");
        }

*/
        if(IpcInitialized == TRUE) {
                log_info("IPC already initialized");
                return EOK;
        }

	/* initialize resource manager and stay in it's dispatch loop */
	if (EOK != ipcDeviceInit(ipcRoot))
	{
		log_error("ipc_i2c: ipcDeviceInit failed");
		goto ipcfail1;
	}
	log_info("ipc_i2c_probe: ipcDeviceInit done");

	log_info("ipc_i2c_probe: HAL initialisation done");

	/* initialize message queue for all channels */
	if (EOK != ipcMsgQueueInit(ipcRoot))
	{
		log_error("ipc_i2c_probe: MsgQueue Initialization failed");
		goto ipcfail2;
	}
	log_info("ipc_i2c_probe: ipcMsgQueueInit done");

	/* start handler threads, set up irq and signal handlers */
	if (EOK != handlerCreatePriorityLists(ipcRoot))
	{
		log_error("ipc_i2c_probe: handlerInit failed");
		goto ipcfail3;
	}
	log_info("ipc_i2c_probe: handlerInit done");

	/* Initialize the HDLC & HAL layer */
	if (EOK != hdlcInit("ipc_i2c", 1, 1, 1))
	{
		log_error("ipc_i2c_probe: hdlcInit failed");
		goto ipcfail4;
	}
	log_info("ipc_i2c_probe: hdlcInit done");

#ifndef	IPC_STATS
	createProcEntry();
#endif

	/* Enable the ipc server */
	ipcServerEnable();
	log_notice("ipc_i2c_probe: IPC communication switched on");
        IpcInitialized = TRUE;
	return EOK;

ipcfail4:
	handlerCleanup();
ipcfail3:
	ipcMsgQueueCleanup(ipcRoot);
ipcfail2:
	ipcDeviceCleanup(ipcRoot);
ipcfail1:
	halCleanup();
ipcfail0:
	if(ipcRoot)
		kfree(ipcRoot);
	ipcRoot = NULL;

	return	ERROR;
}

static int ipc_i2c_remove(struct i2c_client *client)
{
	/* cleanup IPC */
	log_notice("cleanup_ipc called");

	/* Disable the ipc server */
	ipcServerDisable();

	halCleanup();
	log_notice("Hal cleanup done");

	hdlcCleanup();
	log_notice("Hdlc cleanup done");

	handlerCleanup();
	log_notice("Handler cleanup done");

	ipcMsgQueueCleanup(ipcRoot);
	log_notice("Ipc msgqueue cleanup done");

	ipcDeviceCleanup(ipcRoot);
	log_notice("Ipcdevice cleanup done");

#ifndef	IPC_STATS
	remove_proc_entry("ipc-stats", NULL);
#endif

	if(ipcRoot)
		kfree(ipcRoot);

        IpcInitialized = FALSE;
	ipcRoot = NULL;

	return 0;
}

static const struct i2c_device_id ipc_ids[] = {
#ifndef	VTP
	{"ipc_i2c", 0},
#else
	{"ipc_i2c_vtp", 0},
#endif
	{ },
};

MODULE_DEVICE_TABLE(i2c, ipc_ids);

static const struct of_device_id ipc_i2c_dt_ids[] = {
#ifndef	VTP
	{ .compatible = "ipc_i2c"},
#else
	{ .compatible = "ipc_i2c_vtp"},
#endif
	{ /* sentinel */ }
};

static struct i2c_driver ipc_i2c_driver = {
	.driver = {
#ifndef	VTP
		.name = "ipc_i2c",
#else
		.name = "ipc_i2c_vtp",
#endif
		.owner = THIS_MODULE,
//		.of_match_table = ipc_i2c_dt_ids,
	},
	.id_table = ipc_ids,
	.probe = ipc_i2c_probe,
	.remove = ipc_i2c_remove,
};

static int __init ipc_init(void)
{
	Int32 ret=EOK;

	log_notice("ipc_init: IPC module initialising ver-%s", IPC_VERSION);

	/* register the i2c driver */
	ret = i2c_add_driver(&ipc_i2c_driver);
	if (ret)
		log_error("ipc_init: i2c driver registration failed: %d", ret);
	else
		log_notice("ipc_init: i2c driver registration successful");
	return ret;
}

static void __exit ipc_exit(void)
{
	log_info("ipc_exit: IPC module exiting");

	/* unregister the phy driver */
	i2c_del_driver(&ipc_i2c_driver);
	return;
}

module_init(ipc_init);
module_exit(ipc_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION(IPC_VERSION);
MODULE_DESCRIPTION("Harman IPC-I2C driver");
MODULE_AUTHOR("Prasad Lavande<prasad.lavande@harman.com>");
