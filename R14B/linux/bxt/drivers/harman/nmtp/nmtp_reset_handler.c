#include "nmtp_plugin.h"

#include <linux/hrtimer.h>
#include <linux/ktime.h>

#define MS_TO_NS(msec)	((msec) * 1000 * 1000)

#define PRIMARY_TUNER_TYPE 0
#define SECONDARY_TUNER_TYPE 1

#define GPIO_SECONDARY_TUNER		 	461

static wait_queue_head_t reset_queue;
static DEFINE_MUTEX(reset_lock);
static volatile int reset_command;
static struct task_struct *resetHandlerThread = NULL;

#define CREATE_TRACE_POINTS

static void nmtp_reset_fifos(struct nmtp_data *nmtp)
{
	int i;

	for (i = 0; i < NMTP_DEVNODES; i++) {
		kfifo_reset(&nmtp->fifo[i]);
		nmtp->write_count[i] = 0;
		nmtp->poll_count[i] = 0;
	}
	kfifo_reset(&nmtp->write_fifo);
}

void nmtp_reset_suspend(struct nmtp_data *nmtp)
{
	nmtp->reset_status = NMTP_SUSPEND_STATE;
	hrtimer_cancel(&nmtp->hrtimer);
}

static int nmtp_ipc_reset(struct nmtp_data *nmtp)
{
	int status;
	status = nmtp_reset_ipc_op(nmtp);

	return (status == 1) ? 0 : 1;
}

static int request_full_bandwidth(void)
{
	struct nmtp_data *primarynmtp = get_nmtp_details(PRIMARY_TUNER_TYPE);
	int status = 0;

	if (primarynmtp) {
		int full = nmtp_full_bandwidth_request(primarynmtp);

		if (full == 0) {
			//moving to full bandwidth
			if (primarynmtp->capture_count == 0x1) {
				hrtimer_cancel(&primarynmtp->hrtimer);
			}
			mutex_lock(&primarynmtp->txrx_lock);
			primarynmtp->speed_hz = primarynmtp->spi->max_speed_hz = 10000000;
			spi_setup(primarynmtp->spi);
			mutex_unlock(&primarynmtp->txrx_lock);
			if (primarynmtp->capture_count == 0x1) {
				hrtimer_start(&primarynmtp->hrtimer,
					primarynmtp->ktime, HRTIMER_MODE_ABS);
			}
			status = 1;
		}
	}

	return status;
}

static void set_low_bandwidth(void)
{
    struct nmtp_data *primarynmtp = get_nmtp_details(PRIMARY_TUNER_TYPE);

	if (primarynmtp) {
		//moving to low bandwidth
		if (primarynmtp->capture_count == 0x1) {
			hrtimer_cancel(&primarynmtp->hrtimer);
		}
		mutex_lock(&primarynmtp->txrx_lock);
		primarynmtp->speed_hz = primarynmtp->spi->max_speed_hz = 8000000;
		spi_setup(primarynmtp->spi);
		mutex_unlock(&primarynmtp->txrx_lock);
		if (primarynmtp->capture_count == 0x1) {
			hrtimer_start(&primarynmtp->hrtimer,
				primarynmtp->ktime, HRTIMER_MODE_ABS);
		}
	}
}

int nmtp_secondary_query(struct nmtp_data *nmtp)
{
	//gpio implementation
	int i, ret;

	ret = -1;
	for (i = 0 ; i < 50; i++) {
		ret = gpio_get_value(GPIO_SECONDARY_TUNER);   //0 on success
		dev_dbg(&nmtp->spi->dev, "[Hnmtp] GPIO Secondary status = %d and iteration = %d [%s] [%d]",
				ret, i, __func__, __LINE__);
		if (ret == 0)
			break;
		else
			msleep(10);
	}

	return ret;
}



static int wait_for_chips_ready(void)
{
	int status = 0;
	struct nmtp_data *primarynmtp = get_nmtp_details(PRIMARY_TUNER_TYPE);
	struct nmtp_data *secondarynmtp = get_nmtp_details(SECONDARY_TUNER_TYPE);

	if (primarynmtp) {
		dev_dbg(&primarynmtp->spi->dev, "[Hnmtp] primary -- [%s] [%d]  \n", __func__, __LINE__);

		//we need to make sure boot query is succeeded
		status = nmtp_reset_ipc_query(primarynmtp);
		if (status == 0)
			return -1;

		if (secondarynmtp != NULL) {
			status = nmtp_secondary_query(secondarynmtp);
			if (status != 0)
				return -1;
		}

		nmtp_reset_fifos(primarynmtp);

		status = nmtp_sync(primarynmtp);
		dev_dbg(&primarynmtp->spi->dev, "[Hnmtp] -- [%s] [%d] sync status primary %d\n",
								 __func__, __LINE__, status);

		// Retry on error (single tuner only)
		if ((status != 0) && (secondarynmtp == NULL)) {
			dev_dbg(&primarynmtp->spi->dev, "[Hnmtp]-- nmtp sync failed - reset and retry\n");

			// Reset the chip
			status = nmtp_ipc_reset(primarynmtp);
			if (status) {
				dev_dbg(&primarynmtp->spi->dev, "[Hnmtp]-- nmtp primary reset failed --[%s] [%d]\n", __func__, __LINE__);
			}

			// Wait for reset to complete
			status = nmtp_reset_ipc_query(primarynmtp);
			dev_dbg(&primarynmtp->spi->dev, "[Hnmtp]-- nmtp Primary reset boot query [%d]--[%s] [%d]\n", status, __func__, __LINE__);

			// Attempt to re-sync
			status = nmtp_sync(primarynmtp);
			dev_dbg(&primarynmtp->spi->dev, "[Hnmtp] -- [%s] [%d] sync status primary (after reset) %d\n",
									__func__, __LINE__, status);
		}

		if (status != 0)
			return -1;

		if (secondarynmtp != NULL) {
			dev_dbg(&secondarynmtp->spi->dev, "[Hnmtp] secondary -- [%s] [%d]  \n", __func__, __LINE__);

			nmtp_reset_fifos(secondarynmtp);

			status = nmtp_sync(secondarynmtp);

			dev_dbg(&secondarynmtp->spi->dev, "[Hnmtp] -- [%s] [%d] sync status primary %d\n",
					__func__, __LINE__, status);

			if (status != 0)
				return -1;

			hrtimer_start(&secondarynmtp->hrtimer,
						   secondarynmtp->ktime, HRTIMER_MODE_ABS);
			secondarynmtp->capture_count = 0x1;

			mutex_lock(&secondarynmtp->reset_lock);
			secondarynmtp->reset_status = NMTP_NORMAL_OPERATION;
			mutex_unlock(&secondarynmtp->reset_lock);
			wake_up_interruptible(&secondarynmtp->reset_poll_wait);
		}

		hrtimer_start(&primarynmtp->hrtimer,
					primarynmtp->ktime, HRTIMER_MODE_ABS);
		primarynmtp->capture_count = 0x1;
		mutex_lock(&primarynmtp->reset_lock);
		primarynmtp->reset_status = NMTP_NORMAL_OPERATION;
		mutex_unlock(&primarynmtp->reset_lock);
		wake_up_interruptible(&primarynmtp->reset_poll_wait);
	}

	return 0;
}


static int mrd_reset_chips(void)
{
	int status = 1;
	struct nmtp_data *primarynmtp = get_nmtp_details(PRIMARY_TUNER_TYPE);
	struct nmtp_data *secondarynmtp = get_nmtp_details(SECONDARY_TUNER_TYPE);

	if (primarynmtp) {
		mutex_lock(&primarynmtp->reset_lock);
		primarynmtp->reset_num_acks = 0;
		primarynmtp->reset_status = NMTP_UNACKED_RESET_IN_PROGRESS;
		mutex_unlock(&primarynmtp->reset_lock);

		wake_up_interruptible(&primarynmtp->reset_poll_wait);
		dev_dbg(&primarynmtp->spi->dev, "[Hnmtp]--  nmtp state = MRD_RESET_IN_PROGRESS --[%s] [%d]  \n", __func__, __LINE__);

		if (secondarynmtp) {
			mutex_lock(&secondarynmtp->reset_lock);
			secondarynmtp->reset_num_acks = 1;
			secondarynmtp->reset_status = NMTP_UNACKED_RESET_IN_PROGRESS;
			mutex_unlock(&secondarynmtp->reset_lock);

			dev_dbg(&secondarynmtp->spi->dev, "[Hnmtp]--  Sec nmtp state = MRD_RESET_IN_PROGRESS --[%s] [%d]  \n", __func__, __LINE__);
			wake_up_interruptible(&secondarynmtp->reset_poll_wait);
		}

		status = nmtp_ipc_reset(primarynmtp);
		if (status) {
			dev_dbg(&primarynmtp->spi->dev, "[Hnmtp]-- primarynmtp primary reset failed --[%s] [%d]\n", __func__, __LINE__);
		}
	}

	return status;
}

//
// Thread that is used to continuously read messages from the VIP
//
static int ChipResetHandler(void *data)
{
	int fullBandwidth = 0;
	int new_command;
	int timeout;
	int status;

	while (!kthread_should_stop()) {
		// Wait for a message
		if (fullBandwidth)
			timeout = msecs_to_jiffies(1000);
		else
			timeout = msecs_to_jiffies(100);
		wait_event_interruptible_timeout(reset_queue, (reset_command != 0), timeout);
		new_command = reset_command;
		reset_command = 0;
		if (new_command == RESET_CHIPS) {
			// Set bandwidth back to 8Mhz
			fullBandwidth = 0;
			set_low_bandwidth();
			do {
				status = mrd_reset_chips();
				if (status != 0)
					msleep(100);
				else
					status = wait_for_chips_ready();
			} while (status != 0);
		} else if (new_command == WAIT_FOR_CHIPS_READY) {
			// Set bandwidth to 8Mhz
			fullBandwidth = 0;
			set_low_bandwidth();
			status = wait_for_chips_ready();
			if (status != 0) {
				do {
					status = mrd_reset_chips();
					if (status != 0)
						msleep(100);
					else
						status = wait_for_chips_ready();
				} while (status != 0);
			}
		}

		if (!fullBandwidth) {
			int needBandwidthRequest = 0;
			struct nmtp_data *primarynmtp = get_nmtp_details(PRIMARY_TUNER_TYPE);
			mutex_lock(&primarynmtp->reset_lock);
			if (primarynmtp->reset_status == NMTP_NORMAL_OPERATION) {
				needBandwidthRequest = 1;
			}
			mutex_unlock(&primarynmtp->reset_lock);

			if (needBandwidthRequest) {
				fullBandwidth = request_full_bandwidth();
			}
		}
	}
	return 0;
}

void change_reset_state(int action)
{
	reset_command = action;
	wake_up_interruptible(&reset_queue);
}

int nmtp_reset_handler_start(void)
{
	int ret = 0;

	if (resetHandlerThread == NULL) {
		resetHandlerThread = kthread_run(ChipResetHandler, 0, "ChipResetHandler");
		if (!resetHandlerThread) {
			ret = -1;
		}
	}

	return ret;
}

int nmtp_chip_reset_handler_initialize(void)
{
	int ret = 0;
	int gpio_stat;

	mutex_init(&reset_lock);
	init_waitqueue_head(&reset_queue);

	gpio_stat = gpio_request(GPIO_SECONDARY_TUNER, "GPIO_27");
	if (gpio_stat != 0) {
		printk(KERN_ERR "[MRD Driver] Failed to get access to secondary tuner status\n");
		return -1;
	}

	gpio_stat = gpio_direction_input(GPIO_SECONDARY_TUNER);
	if (gpio_stat != 0) {
		printk(KERN_ERR "[MRD Driver] Failed to set direction of secondary tuner status\n");
		return -1;
	}

	ret = nmtp_reset_handler_start();

	return ret;
}


void nmtp_chip_reset_handler_exit(void)
{
	if (resetHandlerThread) {
		kthread_stop(resetHandlerThread);
		resetHandlerThread = NULL;
	}
}
