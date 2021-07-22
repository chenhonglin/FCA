/*****************************************************************
* Project Harman Car Multimedia System
* (c) copyright 2019
* Company Harman/Becker Automotive Systems GmbH
* All rights reserved
* Secrecy Level STRICTLY CONFIDENTIAL
****************************************************************/
/***************************************************************
* @file fs3ipc_os.c
* @ingroup fs3ipc Component
* @author David Rogala
* OS  source file.
*
****************************************************************/

#include "fs3ipc_cfg.h"
#include "fs3ipc_os.h"

#if FS3IPC_OS_AUTOSAR == 1
#include "Os.h"
#define HDLC_TIMER_CYCLIC_DISABLED (0)

#elif FS3IPC_OS_UCOSII == 1
#include "ucos_ii.h"
#endif

#ifdef LOG_MODULE
#undef LOG_MODULE
#endif
#define LOG_MODULE "fs3ipc_os"

#ifndef fs3ipc_os_getSystemTime
#error "fs3ipc_timer_getSystemTime() must be defined. This macro must get the"\
"current system time in milliseconds and return the least significant 32 bits"\
"of time. It is expected that the returned value will roll over correctly"
#endif

extern const fs3ipc_timerCtxType fs3ipc_timerCtxs[];

#if FS3IPC_OS_LINUX_KERNEL == 1
static fs3ipc_os_EventMaskType GetFlagsHelper(fs3ipc_os_EventType *event)
{
	unsigned long irqctx = 0;
	fs3ipc_os_EventMaskType ret;
	spin_lock_irqsave(&event->lock, irqctx);
	ret = event->flags;
	spin_unlock_irqrestore(&event->lock, irqctx);

	return ret;
}
#endif

/**
 * @param[in] event the group of the events in case if it applies
 * @param[in] mask Mask of the events waited for.
 * @returns fs3ipc_StatusType if E_OK No error else error condition
 * @details the state of the current task is set to WAITING, unless at least on
 *          of the given events is set.
 */
fs3ipc_StatusType fs3ipc_os_WaitEvent(fs3ipc_os_EventType *event,
												  fs3ipc_os_EventMaskType mask)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;

#if FS3IPC_OS_AUTOSAR == 1
	StatusType stat;
	stat = WaitEvent(mask);

	if (stat != E_OK)
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("CRIT! WaitEvent %d", ret)
	}

#elif FS3IPC_OS_UCOSII == 1
	fs3ipc_u8 stat = OS_NO_ERR;
	OSFlagPend(*event,
				  mask,
				  OS_FLAG_WAIT_SET_ANY,
				  0,
				  &stat);
	if (stat != OS_NO_ERR)
	{
		LOG_ERROR("CRIT! GetEvent stat = %d", stat);
		ret = fs3ipc_StatusType_ErrorGen;
	}
#elif FS3IPC_OS_LINUX_KERNEL == 1

	if (event)
	{
		fs3ipc_s32 waitStat = 0;
		if ((waitStat = wait_event_interruptible(event->wq, mask &
			GetFlagsHelper(event))) == -ERESTARTSYS)
		{
			//LOG_ERROR("wait_event_interruptible err %d", waitStat);
			/* signal received, return error*/
			ret = fs3ipc_StatusType_ErrorInterrupted;
		}
		else if (waitStat != 0)
		{
			LOG_ERROR("wait_event_interruptible err %d", waitStat);
			/* signal received, return error*/
			ret = fs3ipc_StatusType_ErrorGen;
		}
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
	}

#elif FS3IPC_OS_POSIX_US == 1
	/* todo: implement for posix user space*/
#endif

	return ret;
}
/* todo - only enable for kernel */
fs3ipc_StatusType fs3ipc_os_WaitEventTimeout(fs3ipc_os_EventType *event,
											 fs3ipc_os_EventMaskType mask,
											 fs3ipc_u32 timeoutInMsec)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;

#if FS3IPC_OS_AUTOSAR == 1
	StatusType stat;
	stat = WaitEvent (mask);

	if (stat != E_OK)
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("CRIT! WaitEvent %d", ret)
	}

#elif FS3IPC_OS_UCOSII == 1
	ret = fs3ipc_StatusType_ErrorGen;
	LOG_ERROR("CRIT! Not supported %d", ret)
#elif FS3IPC_OS_LINUX_KERNEL == 1

	if (event)
	{
		fs3ipc_s32 timeoutInJiffies = msecs_to_jiffies(timeoutInMsec);
		fs3ipc_s32 waitStat = 0;
		if ((waitStat = wait_event_interruptible_timeout(event->wq, mask & GetFlagsHelper(event), timeoutInJiffies)) == -ERESTARTSYS)
		{
			//LOG_ERROR("wait_event_interruptible err %d", waitStat);
			/* signal received, return error*/
			ret = fs3ipc_StatusType_ErrorInterrupted;
		}
		else if(waitStat == 0)
		{
			//LOG_ERROR("wait_event_interruptible err %d", waitStat);
			/* signal received, return error*/
			ret = fs3ipc_StatusType_ErrorGen;
		}
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
	}

#elif FS3IPC_OS_POSIX_US == 1
	/* todo: implement for posix user space*/
#endif

	return ret;
}

/**
 * @param[in] event - The task/flag group which shall be queried
 * @param[in] mask - Events which are set
 * @returns fs3ipc_StatusType if E_OK No error else error condition
 * @details This service returns the state of all event bits of the given task.
 */
fs3ipc_StatusType fs3ipc_os_GetEvent(fs3ipc_os_EventType *event,
												 fs3ipc_os_EventMaskType *mask)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;

#if FS3IPC_OS_AUTOSAR == 1
	StatusType stat;

	if (event)
	{
		stat = GetEvent(*event, mask);

		if (stat != E_OK)
		{
			/* todo: print error*/
			ret = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("CRIT! GetEvent %d", ret);
		}
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null Arg");
	}

#elif FS3IPC_OS_UCOSII == 1
	fs3ipc_u8 stat = OS_NO_ERR;
	*mask = OSFlagAccept(*event,
								0xffffffffu, /*check bits 32*/
								OS_FLAG_WAIT_SET_ANY,
								&stat);
	if ((stat != OS_NO_ERR) && (stat != OS_FLAG_ERR_NOT_RDY))
	{
		LOG_ERROR("CRIT! GetEvent stat = %d", stat);
		ret = fs3ipc_StatusType_ErrorGen;
	}
#elif FS3IPC_OS_LINUX_KERNEL == 1

	if (event && mask)
	{
		*mask = GetFlagsHelper(event);
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
	}

	/* todo: implement for linux kernel*/
#elif FS3IPC_OS_POSIX_US == 1
	/* todo: implement for posix user space*/
#endif
	return ret;
}

/**
 * @param[in] event the group of the events in case if it applies
 * @param[in] mask The events which shall be set
 * @returns fs3ipc_StatusType if E_OK No error else error condition
 * @details The events of the calling task are cleared according to the given
 *           event mask
 */
fs3ipc_StatusType fs3ipc_os_ClearEvent(fs3ipc_os_EventType *event,
													fs3ipc_os_EventMaskType mask)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;

#if FS3IPC_OS_AUTOSAR == 1
	StatusType stat;
	stat = ClearEvent(mask);

	if (stat != E_OK)
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("CRIT! ClearEvent %d", ret);
	}

#elif FS3IPC_OS_UCOSII == 1
	/* todo: implement for uCOS-ii*/
	fs3ipc_u8 stat = OS_NO_ERR;
	OSFlagPost(*event, mask, OS_FLAG_CLR, &stat);
	if (stat != OS_NO_ERR)
	{
		LOG_ERROR("CRIT! ClearEvent %d", stat);
		ret = fs3ipc_StatusType_ErrorGen;
	}

#elif FS3IPC_OS_LINUX_KERNEL == 1
	if (event)
	{
		unsigned long irqctx = 0;
		spin_lock_irqsave(&event->lock, irqctx);
		event->flags &= ~mask;
		spin_unlock_irqrestore(&event->lock, irqctx);
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
	}

#elif FS3IPC_OS_POSIX_US == 1
	/* todo: implement for posix user space*/
#endif
	return ret;
}

/**
 * @param[in] event The task/event group which shall be modified
 * @param[in] mask The events which shall be set
 * @returns fs3ipc_StatusType if E_OK No error else error condition
 * @details The events of the calling task are set  according to the given
 *           event mask
 */
fs3ipc_StatusType fs3ipc_os_SetEvent(fs3ipc_os_EventType *event,
												 fs3ipc_os_EventMaskType mask)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;

#if FS3IPC_OS_AUTOSAR == 1
	StatusType stat;

	if (event)
	{
		stat = SetEvent(*event, mask);

		if (stat != E_OK)
		{
			ret = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("CRIT! SetEvent %d", ret);
		}
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null Arg");
	}

#elif FS3IPC_OS_UCOSII == 1
	/* todo: implement for uCOS-ii*/
	fs3ipc_u8 stat = OS_NO_ERR;
	OSFlagPost(*event, mask, OS_FLAG_SET, &stat);
	if (stat != OS_NO_ERR)
	{
		LOG_ERROR("CRIT! SetEvent stat = %d, mask = 0x%X", stat, mask);
		ret = fs3ipc_StatusType_ErrorGen;
	}

#elif FS3IPC_OS_LINUX_KERNEL == 1
	if (event)
	{
		unsigned long irqctx = 0;
		spin_lock_irqsave(&event->lock, irqctx);
		event->flags |= mask;
		spin_unlock_irqrestore(&event->lock, irqctx);

		wake_up_interruptible(&event->wq);
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
	}

#elif FS3IPC_OS_POSIX_US == 1
	/* todo: implement for posix user space*/
#endif
	return ret;
}

/**
 * @param[in] resource The resource which shall be occupied
 * @returns fs3ipc_StatusType if E_OK No error else error condition
 * @details This API serves to enter critical sections in the code.
 *          A critical section shall always be left using
 *          fs3ipc_os_ReleaseResource(). the function will call the  suspend
 *          all interruption
 */
fs3ipc_StatusType fs3ipc_os_GetResource(fs3ipc_os_ResourceType *resource)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;

#if FS3IPC_OS_AUTOSAR == 1
	StatusType status;

	if (resource)
	{
		status = GetResource(*resource);

		if (status != E_OK)
		{
			ret = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("CRIT! GetResource %d", ret);
		}
	}
	else
	{
		/* for null pointer, fall back to disable all interrupts*/
		SuspendAllInterrupts();
	}

#elif FS3IPC_OS_UCOSII == 1
	if (resource)
	{
		fs3ipc_u8 stat = OS_NO_ERR;
		OSSemPend(*resource, 0, &stat);
		if (stat != OS_NO_ERR)
		{
			ret = fs3ipc_StatusType_ErrorGen;
		}
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("CRIT! GetResource NULL");
	}
#elif FS3IPC_OS_LINUX_KERNEL == 1
	if (resource)
	{
		mutex_lock(resource);
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("NULL mutex");
	}

#elif FS3IPC_OS_POSIX_US == 1
	/* todo: implement for posix user space*/
#endif
	return ret;
}

/**
 * @param[in] resource The resource which shall be released
 * @returns fs3ipc_StatusType if E_OK No error else error condition
 * @details This API is the counterpart of fs3ipc_os_GetResource() and serves to
 *          leave critical sections in the code, if the parameter is NULL,
 *          the function will call the enable  all interruption
 */
fs3ipc_StatusType fs3ipc_os_ReleaseResource(fs3ipc_os_ResourceType *resource)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;

#if FS3IPC_OS_AUTOSAR == 1
	StatusType status;

	if (resource)
	{
		status = ReleaseResource(*resource);

		if (status != E_OK)
		{
			ret = fs3ipc_StatusType_ErrorGen;
			LOG_ERROR("CRIT! ReleaseResource %d", ret);
		}
	}
	else
	{
		/* for null pointer, fall back to disable all interrupts*/
		ResumeAllInterrupts();
	}

#elif FS3IPC_OS_UCOSII == 1
	if (resource)
	{
		fs3ipc_u8 stat = OS_NO_ERR;
		stat = OSSemPost(*resource);
		if (stat != OS_NO_ERR)
		{
			ret = fs3ipc_StatusType_ErrorGen;
		}
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("CRIT! ReleaseResource NULL");
	}
#elif FS3IPC_OS_LINUX_KERNEL == 1
	if (resource)
	{
		mutex_unlock(resource);
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("NULL mutex");
	}

#elif FS3IPC_OS_POSIX_US == 1
	/* todo: implement for posix user space*/
#endif
	return ret;
}

/**
 * @param[in] void
 * @returns void
 * @details this function shall be clled periodically, it will determine
 *          the Timer status
 */
fs3ipc_StatusType fs3ipc_os_PeriodicTimerHandler(fs3ipc_handleType handle)
{
	//fs3ipc_u16 i = 0;

	fs3ipc_os_TickType currentTs = 0;
	const fs3ipc_timerCtxType *ctx;
	fs3ipc_os_TickType lowerCutoff;
	fs3ipc_os_TimeoutType *timer;
	fs3ipc_u16 myTimerCount;
	fs3ipc_u16 j;

	if (handle >= FS3IPC_NUM_OF_INSTANCES)
	{
		return fs3ipc_StatusType_ErrorHandle;
	}

	ctx = &fs3ipc_timerCtxs[handle];
	fs3ipc_os_getSystemTime(&currentTs);
	lowerCutoff = currentTs - (FS3IPC_TIMER_MAX_TIMEOUT +
			FS3IPC_TIMER_MARGIN);
	myTimerCount = ctx->timerCount;

	/* iterate through timers*/
	for (j = 0; j < myTimerCount; j++)
	{
		timer = &ctx->timers[j];

		/* is timer active? */
		if (timer->isActive == FS3IPC_TRUE)
		{
			fs3ipc_u8 isTimerExpired = FS3IPC_FALSE;
			fs3ipc_os_TickType myTimeout = timer->alarmTs;

			if (lowerCutoff <= currentTs)
			{
				/* early timer cutoff does not rollover */
				/* (0 <= cutoff <= alarmTs <= currentTs <= (2^32 -1)) */
				if ((lowerCutoff <= myTimeout) && (myTimeout <= currentTs))
				{
					/* timer expired*/
					isTimerExpired = FS3IPC_TRUE;
				}
				else
				{
					/* timer has not expired */
				}
			}
			else
			{
				/* early timer cutoff does rollover */
				/* (0 <= cutoff <= currentTs) OR
					* (cutoff <= currentTs <= (2^32 -1)) */
				if ((lowerCutoff <= myTimeout) || (myTimeout <= currentTs))
				{
					/* timer expired*/
					isTimerExpired = FS3IPC_TRUE;
				}
				else
				{
					/* timer has not expired */
				}
			}

			/* has the timer expired? */
			if (isTimerExpired == FS3IPC_TRUE)
			{
				fs3ipc_TimerCallbackType cb = timer->callback;

				/* timer has been consumed. Mark it inactive*/
				timer->isActive = FS3IPC_FALSE;

				if (cb != NULL)
				{
					/** if the image supports position independant code add
					 * code offset to the linker time function pointer before
					 * executing
					*/
					#if FS3IPC_PIC_EN == 1
						cb = (fs3ipc_TimerCallbackType)((void*)cb +
							fs3ipc_getCodeOffset());
					#endif //FS3IPC_PIC_EN == 1

					/* call the callback function */
					cb(timer->token);
				}
				else
				{
					/* no callback. print error?*/
				}
			}
			else
			{
				/* timer not active*/
			}
		}
		else
		{
			/* timer not active. move to next timer */
		}
	}

	return fs3ipc_StatusType_OK;
}

/**
 * @param[in] timer The Timer ID  which shall be enabled
 * @param[in] ticks The number of tick to activate timeout
 * @returns fs3ipc_StatusType if E_OK No error else error condition
 * @details this function enables the timer requested
 */
fs3ipc_StatusType fs3ipc_os_ArmTimeout(fs3ipc_os_TimeoutType *timer,
													fs3ipc_os_TickType ticks)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;

	if (timer && ticks <= FS3IPC_TIMER_MAX_TIMEOUT)
	{
		fs3ipc_os_TickType ostime = 0;
		timer->isActive = FS3IPC_FALSE;
		fs3ipc_os_getSystemTime(&ostime);
		timer->alarmTs = ostime + ticks;
		timer->isActive = FS3IPC_TRUE;
	}
	else
	{
		/* invalid params */
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null Arg");
	}
	return ret;
}

/**
 * @param[in] timer The Timer ID  which shall be enabled
 * @returns fs3ipc_StatusType if E_OK No error else error condition
 * @details this function stops the requested timer ID
 */
fs3ipc_StatusType fs3ipc_os_StopTimeout(fs3ipc_os_TimeoutType *timer)
{
	fs3ipc_StatusType ret = fs3ipc_StatusType_OK;

	if (timer)
	{
		timer->isActive = FS3IPC_FALSE;
	}
	else
	{
		ret = fs3ipc_StatusType_ErrorGen;
		LOG_ERROR("Null Arg");
	}

	return ret;
}
