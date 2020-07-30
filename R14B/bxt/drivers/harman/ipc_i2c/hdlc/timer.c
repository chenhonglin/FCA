/*****************************************************************************
 *  Project          Harman Car Multimedia System
 *
 *  (c) copyright    2015
 *  Company          Harman International Industries, Incorporated
 *                   All rights reserved
 *  Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 *  @file      	        timer.c
 *  @ingroup       	Kernel and Drivers
 *  @author             Bhagat Singh <bhagat.singh@harman.com>
 *                      Prasad Lavande <prasad.lavande@harman.com>
 *  @brief              Hdlc timeout implementation
 *
 */


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/timer.h>

#include "types.h"
#include "log.h"
#include "ipc.h"
#include "hdlc.h"
#include "statistics.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/

#define IFRAME_TIMER_EVENT	0
#define UFRAME_TIMER_EVENT	1
#define STAT_TIMER_EVENT	2

#define MS_TO_NS(msec)		((msec) * 1000 * 1000)
#define S_TO_NS(sec)		((sec) * 1000 * 1000 * 1000)
#define STAT_PERIOD_IN_SEC	5


/*****************************************************************************
 * VARIABLES
 *****************************************************************************/

static struct completion timer_exp;
static struct hrtimer hr_timer;
static struct hrtimer uframe_timer;
static struct timer_list stat_timer;
static unsigned long delay_in_ns;
static unsigned long uframe_delay_in_ns;
static UInt8 uTimerEvent = 0;
static DEFINE_SPINLOCK(timerLock);
static UInt8 timer_should_run = 0;

extern IPCAttr_t* ipcRoot;
extern UInt8 uWaitForFinal;
extern tHdlc gHdlc;


/*****************************************************************************
 * FUNCTION DEFINITIONS
 *****************************************************************************/

static Int32 TimerThread(void *unused)
{
	Int32 ret;
	UInt8 uEvent;
	unsigned long flags;

	allow_signal(SIGTERM);
	/* Run until signal received */
	while(1) {
		if(kthread_should_stop()) {
			log_notice("Thread %s: exiting", __func__);
			break;
		}

		ret = wait_for_completion_interruptible(&timer_exp);
		if(!ret) {
			spin_lock_irqsave(&timerLock, flags);
			uEvent = uTimerEvent;
			if(uEvent == 0)
				log_notice("stray timer event");

			uTimerEvent = 0;
			spin_unlock_irqrestore(&timerLock, flags);

			if(uEvent & (1 << IFRAME_TIMER_EVENT)) {
				HDLC_STATS_INC(tx_timeout_count);
				log_info("Timer expired %u times", gHdlc.hdlcStats.tx_timeout_count);

				spin_lock(&gHdlc.eventLock);
				SET_EVENT(gHdlc.eventFlag, EVENT_TX_TIMEOUT);
				spin_unlock(&gHdlc.eventLock);
				wake_up_interruptible(&gHdlc.wq);
			}

			if(uEvent & (1 << UFRAME_TIMER_EVENT)) {
				spin_lock(&gHdlc.eventLock);
				SET_EVENT(gHdlc.eventFlag, EVENT_TX_UFRAME_RST);
				spin_unlock(&gHdlc.eventLock);
				wake_up_interruptible(&gHdlc.wq);
			}
#ifdef	IPC_STATS
			if(uEvent & (1 << STAT_TIMER_EVENT)) {
				statisticsPrint(ipcRoot);
				HdlcStatisticsPrint();
			}
#endif
		} else {
			log_notice("%s thread interrupted(%d)....exiting", __func__, ret);
			break;
		}
	}

	return EOK;
}

/**
 * Handler function when the timer expires
 *
 * @param  none
 * @return none
 */
static enum hrtimer_restart timerHandler(struct hrtimer *timer)
{
	log_info("Iframe Timer handler");
	spin_lock(&timerLock);
	uTimerEvent = uTimerEvent | (1 << IFRAME_TIMER_EVENT);
	spin_unlock(&timerLock);
	complete(&timer_exp);
	return HRTIMER_NORESTART;
}

/**
 * Uframe timer Handler function when the timer expires
 *
 * @param  none
 * @return none
 */
static enum hrtimer_restart uFrameTimerHandler(struct hrtimer *timer)
{
	log_info("Uframe Timer handler");
	spin_lock(&timerLock);
	uTimerEvent = uTimerEvent | (1 << UFRAME_TIMER_EVENT);
	spin_unlock(&timerLock);
	complete(&timer_exp);
	return HRTIMER_NORESTART;
}

/**
 * Handler function when the stat timer expires
 *
 * @param  none
 * @return none
 */
static void statTimerHandler(unsigned long data)
{
	log_info("Stat Timer handler");
	spin_lock(&timerLock);
	uTimerEvent = uTimerEvent | (1 << STAT_TIMER_EVENT);
	spin_unlock(&timerLock);
	complete(&timer_exp);
	if(timer_should_run == TRUE) {
		mod_timer(&stat_timer, jiffies + STAT_PERIOD_IN_SEC * HZ);
	}
	return;
}

/**
 * Initialize and configure the timer handling
 *
 * @param  delay_in_ms iframe delay value in millisecs
 * @param  uframe_delay_in_ms uframe delay value in millisecs
 * @return none
 */
Int32 timerInit(int delay_in_ms, int uframe_delay_in_ms)
{
	init_completion(&timer_exp);
	/// Create the timer thread
	gHdlc.timer_task = kthread_run(TimerThread, NULL, "TimerThread");
	if (IS_ERR(gHdlc.timer_task)) {
		log_debug("Timer Thread creation failed: %ld", PTR_ERR(gHdlc.timer_task));
		return ERROR;
	}
	else {
		log_debug("Timer Thread Created successfully");
	}

	hrtimer_init(&hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr_timer.function = &timerHandler;
	delay_in_ns = MS_TO_NS(delay_in_ms);

	hrtimer_init(&uframe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	uframe_timer.function = &uFrameTimerHandler;
	uframe_delay_in_ns = MS_TO_NS(uframe_delay_in_ms);

	timer_should_run = TRUE;
	setup_timer(&stat_timer, statTimerHandler, 0);
	mod_timer(&stat_timer, jiffies + STAT_PERIOD_IN_SEC * HZ);

	return EOK;
}

/**
 * Cleanup the timer handling
 *
 * @return none
 */
void timerCleanup(void)
{
	int ret = 0;
	timer_should_run = FALSE;
	hrtimer_cancel(&uframe_timer);
	hrtimer_cancel(&hr_timer);
	del_timer_sync(&stat_timer);
	log_notice("Timer Cleanup Done");

	/// Cleanup the timer thread
	if(gHdlc.timer_task) {
		if((ret = send_sig(SIGTERM, gHdlc.timer_task, 1)) != 0)
			log_warning("%s: send_sig failed(%d)", __func__, ret);
		kthread_stop(gHdlc.timer_task);
	}
}

/**
 * Reset the timer
 *
 * @param  none
 * @return none
 */
void resetTimer(void)
{
	ktime_t interval;
	int ret;

	interval = ktime_set(0, delay_in_ns);
	ret = hrtimer_cancel(&hr_timer);
	if(ret)
		log_info("resetTimer: timer was still running");

	hrtimer_start(&hr_timer, interval, HRTIMER_MODE_REL);

	return;
}

/**
 * Stop the timer
 *
 * @param  none
 * @return none
 */
void stopTimer(void)
{
	int ret;

	if(uWaitForFinal == FALSE)
	{
		ret = hrtimer_cancel(&hr_timer);
		if(ret)
			log_info("stopTimer: timer was still running");
	}
	return;
}

/**
 * Reset the timer
 *
 * @param  none
 * @return none
 */
void resetUframeTimer(void)
{
	ktime_t interval;
	int ret;

	interval = ktime_set(0, uframe_delay_in_ns);
	ret = hrtimer_cancel(&uframe_timer);
	if(ret)
		log_info("resetUframeTimer: timer was still running");

	hrtimer_start(&uframe_timer, interval, HRTIMER_MODE_REL);

	return;
}

/**
 * Stop the timer
 *
 * @param  none
 * @return none
 */
void stopUframeTimer(void)
{
	int ret;

	ret = hrtimer_cancel(&uframe_timer);
	if(ret)
		log_info("stopUframeTimer: timer was still running");

	return;
}
