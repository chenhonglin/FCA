/*****************************************************************************
 *  Project          Harman Car Multimedia System
 *
 *  (c) copyright    2015
 *  Company          Harman International Industries, Incorporated
 *                   All rights reserved
 *  Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 *  @file      		hal_serial.c
 *  @ingroup       	Kernel and Drivers
 *  @author             Bhagat Singh <bhagat.singh@harman.com>
 *                      Prasad Lavande <prasad.lavande@harman.com>
 *  @brief              API's for configuring, reading and writing data over serial port.
 *
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/i2c.h>

#include "hdlc.h"
#include "types.h"
#include "log.h"
#include "ipc.h"
#include "options.h"


/* IPC I2C For DU */
#define GPIO_BASE_N			434
#define GPIO_BASE_NW			357
#define GPIO_BASE_W			310
#define GPIO_BASE_SW			267

#define VTP_PIN				20      //LGA_SDCARD_CLK/GPIO172
#define IVI_PIN				25      //LGA_SDCARD_D(3)/GPIO176

#define VTP_BASE			GPIO_BASE_SW
#define IVI_BASE			GPIO_BASE_SW

#define	BJEV_VTP_DU_ADDR		0x01
#define	BJEV_IVI_DU_ADDR		0x01

#define IVI_SREQ                	"ivi_ipc_sreq"
#define VTP_SREQ                	"vtp_ipc_sreq"

#ifndef	VTP
#define	SREQ				IVI_SREQ
#else
#define	SREQ				VTP_SREQ
#endif


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

#ifndef	VTP
extern void bjev_ivi_ser_read_isr(void);
#else
extern void bjev_vtp_ser_read_isr(void);
#endif

static irqreturn_t ipc_sready_interrupt(int irq, void *dev_id);


/*****************************************************************************
 * VARIABLES
 *****************************************************************************/

#ifndef	RX_COMM_THREAD
static DEFINE_SPINLOCK(mLock);
#endif
static UInt32 uIrqNo;
extern IPCAttr_t* ipcRoot;


/***************************************************************************n
 * FUNCTION DEFINITIONS
 *****************************************************************************/

//static Int32 ipcGpioInit(IPCAttr_t *root)
static Int32 ipcGpioInit(void)
{
	Int32 ret = EOK, rc_sreq, status;
	log_info("ipcGpioInit: Registering gpios...");

#ifndef	VTP
	rc_sreq = IVI_BASE + IVI_PIN;	//gpio176_pin25
#else
	rc_sreq = VTP_BASE + VTP_PIN;	//gpio172_pin20
#endif
	if (!gpio_is_valid(rc_sreq)) {
		log_error("Sreq Gpio(%d) is not valid!", rc_sreq);
		return -ENODEV;
	}

	status = gpio_request(rc_sreq, "ipc_sreq_pin");
	if (status) {
		log_error("Claiming gpio pin %d failed!", rc_sreq);
		return -EINVAL;
	}
	gpio_direction_input(rc_sreq);
	uIrqNo = gpio_to_irq(rc_sreq);
	log_notice("Registering SREQ: gpio pin: %d, irq no.: %u", rc_sreq, uIrqNo);
#ifdef	NON_THREADED_IRQ
	ret = request_irq(uIrqNo, ipc_sready_interrupt,
			IRQ_TYPE_EDGE_FALLING,
			"ipc_i2c", "IPC");
#else
	ret = request_threaded_irq(uIrqNo, NULL, ipc_sready_interrupt,
			IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT, SREQ, IPC_STR);

#endif	//NON_THREADED_IRQ
	if (ret < 0)
		log_error("request_irq error: %d", ret);
	return ret;
}

/**
 * Initialize and configure the serial port
 *
 * @param  none
 * @return 0 on success and -1 on failure
 */
Int32 halInit(struct i2c_client *client)
{
	struct i2c_timings t;

	/* Initialize the GPIO's */
	if (EOK != ipcGpioInit())
	{
		log_error("ipc_i2c: GPIO initialisation failed");
		return ERROR;
	}
	log_info("ipc_i2c: GPIO initialisation done");

/*	u32 speed = i2c_acpi_find_bus_speed(&client->adapter->dev);
	//Returns the speed in Hz or zero
	log_notice("halInit: %u", speed);
*/
	i2c_parse_fw_timings(&client->adapter->dev, &t, true);
	log_notice("halInit: Freq: %u, SCL Rise time(ns): %u, SCL Fall time(ns): "
		"%u, SCL Int Delay(ns): %u, SDA Fall time(ns) : %u", t.bus_freq_hz,
		t.scl_rise_ns, t.scl_fall_ns, t.scl_int_delay_ns, t.sda_fall_ns);

	return EOK;
}

/**
 * Cleanup the Hal resources
 *
 * @param  none
 * @return none
 */
void halCleanup(void)
{
	Int32 rc_sreq;
#ifndef	VTP
	rc_sreq = IVI_BASE + IVI_PIN;	//gpio176_pin25
#else
	rc_sreq = VTP_BASE + VTP_PIN;	//gpio172_pin20
#endif
	free_irq(uIrqNo,
#ifdef	NON_THREADED_IRQ
			"IPC");
#else
			IPC_STR);
#endif	//NON_THREADED_IRQ
	log_info("ipc_i2c: GPIO(%s) irq freed", SREQ);
	gpio_free(rc_sreq);
}

static irqreturn_t ipc_sready_interrupt(int irq, void *dev_id)
{
	log_info("sready interrupt from IOC");
	/* Clear the interrupt at the Serializer */
#ifndef	VTP
	bjev_ivi_ser_read_isr();
#else
	bjev_vtp_ser_read_isr();
#endif
	log_info("Cleared the interrupt at the Serializer");

#ifdef	RX_COMM_THREAD
	complete(&gHdlc.slaveReady);
#else
	spin_lock(&mLock);
	gHdlc.uSlaveReady = TRUE;
	spin_unlock(&mLock);
	log_notice("wakeup");
	wake_up_interruptible(&gHdlc.wq);
#endif//RX_COMM_THREAD
	return IRQ_HANDLED;
}

#ifndef	RX_COMM_THREAD
static UInt8 uRxBuff[HDLC_FRAME_MAXLEN];
#endif//RX_COMM_THREAD

Int32 halWrite(UInt8* buff, Int32 len)
{
	Int32 iRet=0;
#ifdef	RX_COMM_THREAD
	struct i2c_msg msg;
	msg.addr = ipcRoot->client->addr;
	msg.flags = 0; /* Write */
	msg.len = len;
	msg.buf = buff;

	log_info("ipc_i2c: Master performing I2C Write......1en=%d", len);
	/* Handle for EAGAIN */
	if( (iRet = i2c_transfer(ipcRoot->client->adapter, &msg, 1)) < 0 )
	{
		log_error("ipc_i2c: i2c_transfer(write) failed error=%d", iRet);
	}
	log_info("ipc_i2c: I2C Write is complete");
	dumpBuffer(buff, 32, __FUNCTION__, __LINE__);
#else
	unsigned long flags = 0;
	struct i2c_msg msg[2];
	UInt8 index = 0;
	UInt8 op = 0;
#define	OP_WRITE	1
#define	OP_READ		2
	UInt16 wFrameLen=0;
	if(buff != (UInt8 *)NULL)
	{
		log_info("ipc_i2c: Master wants to write.");
		msg[index].addr = ipcRoot->client->addr;
		msg[index].flags = 0; /* Write */
		msg[index].len = len;
		msg[index].buf = buff;
		index++;
		op |= OP_WRITE;
	}

	spin_lock_irqsave(&mLock, flags);
	if(gHdlc.uSlaveReady == FALSE) {
		spin_unlock_irqrestore(&mLock, flags);
	} else {
		gHdlc.uSlaveReady = FALSE;
		spin_unlock_irqrestore(&mLock, flags);
		log_info("ipc_i2c: Slave requested read.");
		op |= OP_READ;
		memset(uRxBuff, 0, sizeof(uRxBuff));
		msg[index].addr = ipcRoot->client->addr;
		msg[index].flags = I2C_M_RD; /* Read */
		msg[index].len = len;
		msg[index].buf = uRxBuff;
		index++;
	}

	log_info("ipc_i2c: Master performing I2C transaction......1en=%d", len);
	if( (iRet = i2c_transfer(ipcRoot->client->adapter, msg, index)) < 0 )
	{
		log_error("ipc_i2c: i2c_transfer failed error=%d", iRet);
	}
	log_info("ipc_i2c: I2C transaction is complete");
	if(op & WRITE)
		dumpBuffer(buff, 32, __FUNCTION__, __LINE__);
	if(op & OP_READ)
		dumpBuffer(uRxBuff, 32, __FUNCTION__, __LINE__);

	if((op & OP_READ) && checkValidFrame(uRxBuff, &wFrameLen))
	{
		log_info("Valid frame received, decoded frame size = %d",wFrameLen);
		processRxPacket(&uRxBuff[FRAME_LEN], wFrameLen);
	}
#endif//RX_COMM_THREAD
	return EOK;
}
