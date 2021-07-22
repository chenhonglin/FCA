/*****************************************************************************
 * Project              Harman Car Multimedia System
 *
 * c) copyright         2019
 * Company              Harman International Industries, Incorporated
 *                      All rights reserved
 * Secrecy Level    	   STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file                ipc.c
 * @author              David Rogala <david.rogala@harman.com>
 * @ingroup             Kernel and Drivers
 *
 * This module contains the SPI IPC implementation between SoC-IoC.
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <uapi/asm-generic/ioctls.h>

#include "fs3ipc.h"
#include "fs3ipc_phys.h"
#include "fs3ipc_os.h"
#include "fs3ipc_hdlc.h"
#include "log.h"




/*****************************************************************************
 * CONSTANTS
 *****************************************************************************/
#define FS3IPC_DRV_VERSION "0.3.0"
#define FS3IPC_DRV_NAME "ioc-dual-ipc"
#define FS3IPC_TIMER_POLL_RATE (50)

#define IPC_BUS_NUM_0      (2)
#define IPC_BUS_NUM_1      (3)
#define PM_BUS_0_MASK      (1u << IPC_BUS_NUM_0)
#define PM_BUS_1_MASK      (1u << IPC_BUS_NUM_1)
#define PM_BUS_ALL         (PM_BUS_0_MASK|PM_BUS_1_MASK)
#define RC_ACK_PIN         (456)

#define PROBE_ST_ERROR     (-1)
#define PROBE_ST_GPIO      (0)
#define PROBE_ST_SPI       (1)
#define PROBE_ST_INIT_MOD  (2)
#define PROBE_ST_COMP      (3)

#define PROBE_SPI_ERROR    (-1)
#define PROBE_SPI_NOINIT   (0)
#define PROBE_SPI_INIT     (1)

#define PROBE_GPIO_ERROR         (-1)
#define PROBE_GPIO_NOT_SUPPORTED (0)
#define PROBE_GPIO_SUPPORTED     (1)

extern void app_0_CustomInit(void);

/*****************************************************************************
 * VARIABLES
 *****************************************************************************/
struct fs3ipc_spi_port fs3ipc_spi_ports[] = {
	{
		.spi_bus_hndl = 2,
		.fs3ipc_hndl = FS3IPC_INSTANCE_0,
		.fs3ipc_port_hndl = 0,
		.init = FALSE,
	},
	{
		.spi_bus_hndl = 3,
		.fs3ipc_hndl = FS3IPC_INSTANCE_0,
		.fs3ipc_port_hndl = 1,
		.init = FALSE,
	},
};
const int fs3ipc_spi_port_count = (sizeof(fs3ipc_spi_ports) /
		sizeof(struct fs3ipc_spi_port));

struct fs3ipc_drv g_fs3ipc_drv_instances[] = {
	{
		.spi_ports = &fs3ipc_spi_ports[0],
		.spi_port_count = 2,
		.fs3ipc_hndl = FS3IPC_INSTANCE_0,
		.ipc_state = PROBE_ST_GPIO,
		.mrq_pin = 452,
		.srq_pin = 463,
		.spi_mode = SPI_MODE_1,
		.spi_bits_per_word = 8,
		.max_speed_hz = 5000000,
	},
};
const int fs3ipc_drv_count = sizeof(g_fs3ipc_drv_instances) /
		sizeof(struct fs3ipc_drv);


/*****************************************************************************
 * FUNCTION DEFINITIONS
 *****************************************************************************/
inline static struct fs3ipc_spi_port *lookup_fs3ipc_to_spi (uint8_t handle){
	if (handle < fs3ipc_spi_port_count) {
		return &fs3ipc_spi_ports[handle];
	} else {
		return NULL;
	}
}

inline static struct fs3ipc_drv *lookup_fs3ipc_to_ipc (uint8_t handle){
	if (handle < fs3ipc_drv_count) {
		return &g_fs3ipc_drv_instances[handle];
	} else {
		return NULL;
	}
}

static void spi_complete_cb(void *data) {
	struct fs3ipc_spi_port* spi_port = (struct fs3ipc_spi_port*)data;
	fs3ipc_phys_ProcessSpiCompletion(spi_port->fs3ipc_hndl,
			spi_port->fs3ipc_port_hndl);
}

irqreturn_t ipc_srq_interrupt(int irq, void *dev_id)
{
	struct fs3ipc_drv* ipc = (struct fs3ipc_drv*)dev_id;
	fs3ipc_phys_ProcessExtReqEvent(ipc->fs3ipc_hndl);
	return IRQ_HANDLED;
}

int ipc_SetupSpiXfer(uint8_t spi_handle, uint8_t *in, const uint8_t *out,
		uint32_t size)
{
	struct fs3ipc_spi_port* spi_port = lookup_fs3ipc_to_spi(spi_handle);

	if (spi_port) {
		spi_port->transfer.tx_buf = out;
		spi_port->transfer.rx_buf = in;
		spi_port->transfer.len = size;

		memset(in, 0, size);
		spi_message_init(&spi_port->message);
		spi_port->message.complete = spi_complete_cb;
		spi_port->message.context = spi_port;
		spi_message_add_tail(&spi_port->transfer, &spi_port->message);

		if (spi_async (spi_port->spi_dev, &spi_port->message)) {
			log_error ("spi_async port %d returned error",
					spi_port->spi_bus_hndl);
			return fs3ipc_StatusType_ErrorGen;
		}

		return fs3ipc_StatusType_OK;
	}
	return fs3ipc_StatusType_ErrorHandle;
}

int ipc_SpiAbort(uint8_t spi_handle)
{
	/* nothing to do. There is no API to terminate a transaction*/
	return fs3ipc_StatusType_OK;
}

int ipc_PulseGPIO(uint8_t handle)
{
	struct fs3ipc_drv* ipc = lookup_fs3ipc_to_ipc(handle);

	if (ipc) {
		int pin = ipc->mrq_pin;

		gpio_set_value(pin, 0);
		udelay(1);
		gpio_set_value(pin, 1);
		return fs3ipc_StatusType_OK;
	}
	return fs3ipc_StatusType_ErrorHandle;
}

void ipc_suspend_complete(uint8_t handle)
{
	struct fs3ipc_drv* ipc = lookup_fs3ipc_to_ipc(handle);
	ipc->suspend = 1;
	wake_up_interruptible(&ipc->suspend_q);
}



int ipc_gpio_init(struct fs3ipc_drv* drv)
{
	int status;
	if (!gpio_is_valid(drv->mrq_pin)) {
		return -ENODEV;
	}

	status = gpio_request(drv->mrq_pin, "ipc_mreq_pin");
	if (status) {
		log_error("Claiming gpio pin %d failed!\n", drv->mrq_pin);
		return -EINVAL;
	}
	gpio_direction_output(drv->mrq_pin, 1);
	gpio_set_value(drv->mrq_pin, 1);

	if (!gpio_is_valid(drv->srq_pin)) {
		return -ENODEV;
	}

	status = gpio_request(drv->srq_pin, "ipc_sreq_pin");
	if (status) {
		log_error("Claiming gpio pin %d failed!\n", drv->srq_pin);
		return -EINVAL;
	}
	gpio_direction_input(drv->srq_pin);

	status = request_irq(gpio_to_irq(drv->srq_pin), ipc_srq_interrupt,
			IRQ_TYPE_EDGE_FALLING, "spi-ipc", drv);
	if (status) {
		log_error("request_irq error\n");
	}

	return status;
}


static int detect_gpio_cfg(int pin)
{
	int ret = PROBE_GPIO_SUPPORTED;

	if (!(gpio_is_valid (pin)) || (gpio_request (pin, "ipc_ack_pin"))) {
		log_error ("Setting up GPIO pin for ipc type failed");
		ret = PROBE_GPIO_ERROR;

	} else {
		if (gpio_direction_input (pin)) {
			log_error("Setting up GPIO pin direction for ipc type failed");
			ret = PROBE_GPIO_ERROR;

		} else if (1 == gpio_get_value(pin)) {
			// Check again after a couple of microseconds to make sure
			udelay(2);
			if (1 == gpio_get_value(pin)) {
				// Line is high - It's single spi vip
				ret = PROBE_GPIO_NOT_SUPPORTED;
			}
		}

		gpio_free(RC_ACK_PIN);
	}

	return ret;
}

static int tx_thread(void *arg)
{
	struct fs3ipc_drv* ipc = (struct fs3ipc_drv*)arg;
	log_info("Enter %s", __func__);
	/* no need to loop. The following call does not return*/
	fs3ipc_TxThread(ipc->fs3ipc_hndl);
	return 0;
}

static int timer_thread(void *arg)
{
	struct fs3ipc_drv* ipc = (struct fs3ipc_drv*)arg;
	log_info("Enter %s", __func__);

	/* Loop forever*/
	for (;;) {
		msleep(FS3IPC_TIMER_POLL_RATE);
		fs3ipc_os_PeriodicTimerHandler(ipc->fs3ipc_hndl);
	}
	return 0;
}

static int fs3ipc_complete_init(struct fs3ipc_drv *ipc) {

	app_0_CustomInit();
	if (fs3ipc_phys_Init(ipc->fs3ipc_hndl) !=
			fs3ipc_StatusType_OK) {
		log_error("fs3ipc_phys_uart_Init failure");
		goto probeError1;
	}
	if (FS3IPC_hdlcInitialize(ipc->fs3ipc_hndl) != FS3IPC_TRUE) {
		log_error("FS3IPC_hdlcInitialize failure");
		goto probeError1;
	}
	log_error("fs3ipc_Init");
	if (fs3ipc_Init(ipc->fs3ipc_hndl) != fs3ipc_StatusType_OK) {
		log_error("fs3ipc_Init failure");
		goto probeError1;
	}
	if (ipcDeviceInit(&ipc->ipcRoot) != EOK){
		log_error("dev-ipc: ipcDeviceInit failed");
		goto probeError1;
	}

	log_info("fs3ipc layers initialized" );

	if (ipc_gpio_init(ipc) != EOK) {
		log_error("failure to initialize GPIO");
		goto probeError2;
	}

	ipc->tx_task = kthread_run(tx_thread, (void*)ipc, "fs3ipcTx");
	if (ipc->tx_task) {
		log_info("fs3ipcTx thread created" );
	} else {
		log_error("fs3ipcTx thread creation failed");
		goto probeError2;
	}

	ipc->timer_task = kthread_run(timer_thread, (void*)ipc,
			"fs3ipcTimer");
	if (ipc->timer_task) {
		log_info("fs3ipcTimer thread created" );
	} else {
		log_error("fs3ipcTimer thread creation failed");
		goto probeError2;
	}



	ipcServerEnable(&ipc->ipcRoot);
	probeError1:
	probeError2:

	return 0;
}

static int spi_bus_init(struct spi_device *dev, struct fs3ipc_drv *ipc)
{
	int ret=0;
	/* initialise the SPI mode, speed and bits */
	dev->mode = ipc->spi_mode;
	dev->bits_per_word = ipc->spi_bits_per_word;
	dev->max_speed_hz = ipc->max_speed_hz;

	ret = spi_setup(dev);
	if (ret < 0) {
		log_error("spi_setup failed %d\n", ret);
		ret = -1;
	}
	return ret;
}

static int ipc_drv_lookup(struct spi_device *spi_dev,
	struct fs3ipc_drv **ipc_drv, struct fs3ipc_spi_port **spi_port)
{
	int i;
	int bus_num = spi_dev->master->bus_num;
	struct fs3ipc_spi_port *s;
	for (i=0; i < fs3ipc_spi_port_count; i++) {
		s = &fs3ipc_spi_ports[i];
		if (s->spi_bus_hndl == bus_num && s->fs3ipc_hndl < fs3ipc_drv_count) {
			if (ipc_drv) {
				*ipc_drv = &g_fs3ipc_drv_instances[s->fs3ipc_hndl];
			}
			if (spi_port) {
				*spi_port = s;
			}
			return 0;
		}
	}
	return -1;
}

static int register_spi_bus(struct spi_device *spi_dev,
	struct fs3ipc_drv *ipc_drv, struct fs3ipc_spi_port *spi_port)
{
	if (!spi_port->init && spi_port->spi_bus_hndl == spi_dev->master->bus_num) {
		spi_port->init = TRUE;
		spi_port->spi_dev = spi_dev;
		ipc_drv->spi_port_init_count++;
		spi_set_drvdata(spi_dev, ipc_drv);

		return spi_bus_init(spi_dev, ipc_drv);
	}

	return -1;
}

static int fs3ipc_probe(struct spi_device *spi_dev)
{
	int ret = 0;
	struct fs3ipc_drv *ipc = NULL;
	struct fs3ipc_spi_port *spi_cfg = NULL;
	static DEFINE_MUTEX(mylock);

	ipc_drv_lookup(spi_dev, &ipc, &spi_cfg);

	if (!ipc || !spi_cfg) {
		return -ENODEV;
	}

	mutex_lock(&mylock);

	/* If probe is in an unexpected state, return ENODEV*/
	if (!ret && ((ipc->ipc_state != PROBE_ST_GPIO) &&
			(ipc->ipc_state != PROBE_ST_SPI) &&
			(ipc->ipc_state != PROBE_ST_INIT_MOD))) {

		log_error("aborted call. state = %d", ipc->ipc_state);
		ret = -ENODEV;
	}

	if (!ret && ipc->ipc_state == PROBE_ST_GPIO) {
		int stat;
		stat = detect_gpio_cfg (RC_ACK_PIN);
		if (stat == PROBE_GPIO_SUPPORTED) {
			ipc->ipc_state = PROBE_ST_SPI;
			log_info ("gpio det - supported");
		} else if (stat == PROBE_GPIO_NOT_SUPPORTED) {
			ipc->ipc_state = PROBE_ST_COMP;
			log_error ("gpio det - not supported");
			ret = -ENODEV;
		} else {
			ipc->ipc_state = PROBE_ST_ERROR;
			log_error ("gpio det - error");
			ret = -ENODEV;
		}
	}

	/* If GPIO probe indicates fs3ipc is supported and the sequence has not been
	 * completed already, initialize the SPI ports. Note, completing this step
	 * will require multiple calls, depending on the number of SPI ports*/
	if (!ret && ipc->ipc_state == PROBE_ST_SPI) {
		if (register_spi_bus(spi_dev, ipc, spi_cfg)) {
			ret = -ENODEV;
		} else if (ipc->spi_port_init_count >= ipc->spi_port_count) {
			ipc->ipc_state = PROBE_ST_INIT_MOD;
			log_info("SPI init comp");
		}
	}

		/* Once the GPIO probe has been completed and all SPI hardware is initalized
	 * complete initialization by initializing modules and start main thread*/
	if (!ret && ipc->ipc_state == PROBE_ST_INIT_MOD) {
		if (fs3ipc_complete_init(ipc)) {
			ipc->ipc_state = PROBE_ST_ERROR;
			ret = -ENODEV;
			log_error("init modules error");
		}
		else
		{
			ipc->ipc_state = PROBE_ST_COMP;
			log_info("init codules success");
		}
	}

	if (ret || ipc->ipc_state == PROBE_ST_ERROR) {
		log_error("failure %d", ret);
	}

	mutex_unlock (&mylock);

	return ret;
}

static int fs3ipc_remove(struct spi_device *spi_dev)
{
	struct fs3ipc_drv *ipc = spi_get_drvdata(spi_dev);
	ipcDeviceCleanup(&ipc->ipcRoot);

	return 0;
}

static inline void fs3ipc_calc_pm_comp_mask (struct fs3ipc_drv *ipc_drv)
{
	int i;
	uint32_t pm_mask = 0;
	for (i=0; i < ipc_drv->spi_port_count; i++) {
		pm_mask |= (1u << ipc_drv->spi_ports[i].spi_bus_hndl);
	}
}

static void update_pm_mask(struct spi_device *dev, uint32_t *mask)
{
	uint32_t bus = dev->master->bus_num;
	*mask |= (1u << bus);
}

static int fs3ipc_suspend(struct device *dev)
{
	struct spi_device *spi_dev = to_spi_device(dev);
	struct fs3ipc_drv *ipc_drv = spi_get_drvdata(spi_dev);
	unsigned long irqctx = 0;
	uint8_t bSuspend = FALSE;
	uint32_t previous_mask = ipc_drv->suspend_mask;

	log_error ("enter");

	spin_lock_irqsave(&ipc_drv->lock, irqctx);
	update_pm_mask (spi_dev, &ipc_drv->suspend_mask);

	if ((ipc_drv->suspend_mask & ipc_drv->pm_comp_mask) ==
		ipc_drv->pm_comp_mask) {
		/**suspend for both buses has been received. Reset mask for next suspend*/
		ipc_drv->suspend_mask = 0;
	}

	if ((ipc_drv->suspend_mask & ipc_drv->pm_comp_mask) && (!previous_mask)) {
		/** fs3ipc_suspend is called once for each bus. Suspend ipc state machine
		 * call should only be called once. Call upon first suspend callback*/
		bSuspend = TRUE;
	}

	spin_unlock_irqrestore(&ipc_drv->lock, irqctx);

	if (bSuspend) {
		int waitStat;
		fs3ipc_hdlc_suspend(ipc_drv->fs3ipc_hndl);
		waitStat = wait_event_interruptible(ipc_drv->suspend_q, ipc_drv->suspend == 1);
		log_error ("suspend completed");
	}

	return(0);
}

static int fs3ipc_resume(struct device *dev)
{
	struct spi_device *spi_dev = to_spi_device(dev);
	struct fs3ipc_drv *ipc_drv = spi_get_drvdata(spi_dev);
	uint8_t bResume = FALSE;
	unsigned long irqctx = 0;

	spin_lock_irqsave(&ipc_drv->lock, irqctx);
	update_pm_mask (spi_dev, &ipc_drv->resume_mask);

	if ((ipc_drv->resume_mask & ipc_drv->pm_comp_mask) == ipc_drv->pm_comp_mask)
	{
		/** resume for both buses has been received. Reset mask for next resume*/
		ipc_drv->resume_mask = 0;

		/** fs3ipc_resume is called once for each bus. resume ipc state machine
		 * should only be called once. Call upon both callbacks being triggered*/
		bResume = TRUE;
	}

	spin_unlock_irqrestore(&ipc_drv->lock, irqctx);

	if (bResume)
	{
		ipc_drv->suspend = 0;
		fs3ipc_hdlc_resume(ipc_drv->fs3ipc_hndl);
	}

	return(0);
}

static const struct dev_pm_ops fs3ipc_pm_ops = {
	.suspend = fs3ipc_suspend,
	.resume  = fs3ipc_resume,
};

static const struct spi_device_id ipc_ids[] = {
	{"ioc-ipc"},
	{"dual-spi-ipc"},
	{ },
};

static struct spi_driver fs3ipc_driver = {
	.probe = fs3ipc_probe,
	.remove = fs3ipc_remove,
	.id_table = ipc_ids,
	.driver = {
		.name = FS3IPC_DRV_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.pm = &fs3ipc_pm_ops,
	},
};

static int __init ipc_init(void)
{
	int ret=0, i, j;
	struct fs3ipc_drv *drv;

	for (i = 0; i < fs3ipc_drv_count; i++) {
		uint32_t mask = 0;
		drv = &g_fs3ipc_drv_instances[i];

		for (j = 0; j < drv->spi_port_count; j++) {
			struct fs3ipc_spi_port *port = &drv->spi_ports[j];
			mask |= 1u << (port->spi_bus_hndl);
		}
		drv->pm_comp_mask = mask;
		drv->suspend = 0;
		init_waitqueue_head(&drv->suspend_q);
	}

	log_error("ipc_init: IPC module initialising ver-0.3");
	/* register the spi driver */
	ret = spi_register_driver(&fs3ipc_driver);
	if (ret)
		log_error("ipc_init: spi driver registration failed:");

	return ret;
}

subsys_initcall(ipc_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Harman dual SPI-IPC driver");
MODULE_VERSION(FS3IPC_DRV_VERSION);
MODULE_AUTHOR("David Rogala<david.rogala@harman.com>");
