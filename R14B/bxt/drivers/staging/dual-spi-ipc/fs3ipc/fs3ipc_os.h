/*
 * fs3ipc_os.h
 *
 *  Created on: Oct 19, 2018
 *      Author: drogala
 */
#ifndef FS3IPC_OS_H_
#define FS3IPC_OS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fs3ipc_cfg.h"

#if FS3IPC_OS_AUTOSAR == 1
#include "os.h"
#elif FS3IPC_OS_UCOSII == 1
#include "ucos_ii.h"
#elif FS3IPC_OS_LINUX_KERNEL == 1
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
//#include "../include/hdlc.h"
#elif FS3IPC_OS_POSIX_US == 1
#else
#error "invalid OS config"
#endif

/* data types which are customized per OS*/
#if FS3IPC_OS_AUTOSAR == 1

typedef TaskType fs3ipc_os_EventType;
typedef EventMaskType fs3ipc_os_EventMaskType;
typedef TickType fs3ipc_os_TickType;
typedef ResourceType fs3ipc_os_ResourceType;

#elif FS3IPC_OS_UCOSII == 1
typedef OS_FLAG_GRP *fs3ipc_os_EventType;
typedef OS_FLAGS fs3ipc_os_EventMaskType;
typedef OS_EVENT *fs3ipc_os_ResourceType;
typedef fs3ipc_u32 fs3ipc_os_TickType;
#elif FS3IPC_OS_LINUX_KERNEL == 1

typedef struct
{
	uint32_t flags;
	spinlock_t lock;
	wait_queue_head_t wq;
} fs3ipc_os_EventType;

typedef uint32_t fs3ipc_os_EventMaskType;
typedef uint32_t fs3ipc_os_TickType;
typedef struct mutex fs3ipc_os_ResourceType;

#elif FS3IPC_OS_POSIX_US == 1
#endif

/* timer implementation is common*/
#define FS3IPC_TIMER_MAX_TIMEOUT (3600000)
#define FS3IPC_TIMER_MARGIN (2000)
typedef void (*fs3ipc_TimerCallbackType)(void *);

typedef struct
{
	fs3ipc_u8 isActive;
	fs3ipc_os_TickType alarmTs;
	fs3ipc_TimerCallbackType callback;
	void *token;
} fs3ipc_os_TimeoutType;

typedef struct
{
	fs3ipc_os_TimeoutType *timers;
	fs3ipc_u16 timerCount;
} fs3ipc_timerCtxType;

/* global functions*/
fs3ipc_StatusType fs3ipc_os_WaitEvent(fs3ipc_os_EventType *event,
												fs3ipc_os_EventMaskType mask);

fs3ipc_StatusType fs3ipc_os_WaitEventTimeout(fs3ipc_os_EventType *event,
														fs3ipc_os_EventMaskType mask,
														fs3ipc_u32 timeoutInMsec);
fs3ipc_StatusType fs3ipc_os_GetEvent(fs3ipc_os_EventType *event,
												fs3ipc_os_EventMaskType *mask);

fs3ipc_StatusType fs3ipc_os_ClearEvent(fs3ipc_os_EventType *event,
													fs3ipc_os_EventMaskType mask);

fs3ipc_StatusType fs3ipc_os_PeriodicTimerHandler(fs3ipc_handleType handle);

fs3ipc_StatusType fs3ipc_os_SetEvent(fs3ipc_os_EventType *event,
												fs3ipc_os_EventMaskType mask);

fs3ipc_StatusType fs3ipc_os_ArmTimeout(fs3ipc_os_TimeoutType *timer,
													fs3ipc_os_TickType ticks);

fs3ipc_StatusType fs3ipc_os_StopTimeout(fs3ipc_os_TimeoutType *timer);

fs3ipc_StatusType fs3ipc_os_GetResource(fs3ipc_os_ResourceType *resource);

fs3ipc_StatusType fs3ipc_os_ReleaseResource(fs3ipc_os_ResourceType *resource);

#ifdef __cplusplus
}
#endif

#endif /* FS3IPC_OS_H_ */
