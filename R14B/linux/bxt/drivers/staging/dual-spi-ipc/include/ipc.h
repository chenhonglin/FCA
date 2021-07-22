/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	ipc.h
 * @author        	David Rogala
 *
 * This header file defines all the data structures used to
 * handle the IPC and the attributes for the channel devices
 */

#ifndef __IPC_H
#define __IPC_H

/*---------------------------------------------------------------------
 * INCLUDES
 *--------------------------------------------------------------------*/
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fcntl.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>

#define INVALID_CHANNEL_ID (255u)
#define IPC_MAX_CHANNELS (32)
#define IPC_FREE_CHANNEL (2)
#define CHANNEL2_MINOR (2)


#ifdef DEBUG_FH
#define DEBUG_CHANNEL IPC_MAX_CHANNELS
#define DEBUG_CHANNEL_NAME "ipcdebug"
#endif

#define TRUE (1)
#define FALSE (0)
#define EOK (0)

/*---------------------------------------------------------------------
 * STRUCTURES
 *--------------------------------------------------------------------*/
/**
 * Device attributes structure.
 */
typedef struct TDeviceAttr_s
{
	uint8_t                   channelID;       	/* channel identifier */
	uint32_t			flags;
	wait_queue_head_t*   clientRxWq;
	uint8_t                clientHandle;
	struct cdev          cdev;
}TDeviceAttr;


typedef struct IPCAttr_s
{
	TDeviceAttr	*deviceAttr;	/*<* Array of IPC channel devices, the last is reserved as error channel */
	dev_t	ipc_chdev;
	struct class *dev_class;
	uint32_t			uNoChannels;

}IPCAttr_t;

struct fs3ipc_spi_port {
	struct spi_device *spi_dev;
	struct spi_message message;
	struct spi_transfer transfer;
	int spi_bus_hndl;
	int fs3ipc_hndl;
	int fs3ipc_port_hndl;
	uint8_t init;
};

struct fs3ipc_drv {
	spinlock_t lock;
	struct fs3ipc_spi_port* spi_ports;
	int32_t spi_port_count;
	int32_t spi_port_init_count;
	struct task_struct *tx_task;
	struct task_struct *timer_task;
	fs3ipc_handleType fs3ipc_hndl;
	IPCAttr_t ipcRoot;
	uint32_t suspend_mask;
	uint32_t resume_mask;
	uint32_t pm_comp_mask;
	int32_t ipc_state;

	int32_t mrq_pin;
	int32_t srq_pin;

	int spi_mode;
	int spi_bits_per_word;
	int max_speed_hz;
	wait_queue_head_t suspend_q;
	int suspend;
};

extern struct fs3ipc_spi_port fs3ipc_spi_ports[];
extern const int fs3ipc_spi_port_count;
extern struct fs3ipc_drv g_fs3ipc_drv_instances[];
extern const int fs3ipc_drv_count;

extern int ipc_SetupSpiXfer(uint8_t spi_handle, uint8_t *in, const uint8_t *out,
		uint32_t size);
extern int ipc_SpiAbort(uint8_t spi_handle);
extern int ipc_PulseGPIO(uint8_t handle);
extern void ipc_suspend_complete(uint8_t handle);

#ifdef DEBUG_FH
extern const struct file_operations debug_fops;
#endif


int32_t ipcDeviceInit(IPCAttr_t *ipcRoot);
void ipcDeviceCleanup(IPCAttr_t *ipcRoot );
TDeviceAttr* getDeviceAttribute(uint8_t device);
TDeviceAttr* getDeviceAttrPtr(uint8_t channel);
uint8_t ipcServerIsEnabled(void);
void ipcServerDisable(void);
void ipcServerEnable(IPCAttr_t *ipcRoot);


int ipc_channel_open(int ch_num);
int ipc_channel_close(int chnum);
int ipc_channel_write(int chnum, const char *buf, size_t count);
int ipc_channel_read_timeout(int chnum, char *buf, size_t count,
	int timeoutInMsec);
int ipc_channel_read(int chnum, char *buf, size_t count, int flags);
void fs3ipc_uart_transmit(void* usr_ptr,u8* data, u32 count);

#endif /* __IPC_H */

