#include "nmtp_plugin.h"


#include <linux/hrtimer.h>
#include <linux/ktime.h>

#define MS_TO_NS(msec)	((msec) * 1000 * 1000)

#define PRIMARY_TUNER_TYPE 0
#define SECONDARY_TUNER_TYPE 1
#define SUCCESS 0
#define FAILURE 1

typedef struct instance_data_s {
	struct nmtp_data *nmtp_data;
	int last_value;
} instance_data_t;


#define CREATE_TRACE_POINTS


static int mrd_reset_major;
struct class *mrd_reset_class;
static DECLARE_BITMAP(minors, DYNAMIC_MINORS);
static DEFINE_MUTEX(mrd_reset_lock);

static enum hrtimer_restart timerHandler(struct hrtimer *timer)
{
	struct nmtp_data *nmtp = NULL;
	if (timer == &get_nmtp_details(PRIMARY_TUNER_TYPE)->reply_timer) {
		// Primary chip
		nmtp = get_nmtp_details(PRIMARY_TUNER_TYPE);
	} else if (timer == &get_nmtp_details(SECONDARY_TUNER_TYPE)->reply_timer) {
		nmtp = get_nmtp_details(SECONDARY_TUNER_TYPE);
	}

	if (nmtp) {
		mutex_lock(&nmtp->reset_lock);
		if ((nmtp->reset_status == NMTP_FORCED_RESET_PENDING)  ||
		    (nmtp->reset_status == NMTP_REQUEST_RESET_PENDING)) {
			// reset
			if (get_nmtp_details(SECONDARY_TUNER_TYPE)) {
				mutex_lock(&get_nmtp_details(SECONDARY_TUNER_TYPE)->reset_lock);
				get_nmtp_details(SECONDARY_TUNER_TYPE)->reset_num_acks = 1;
				get_nmtp_details(SECONDARY_TUNER_TYPE)->reset_status = NMTP_UNACKED_RESET_IN_PROGRESS;
				wake_up_interruptible(&get_nmtp_details(SECONDARY_TUNER_TYPE)->reset_poll_wait);
				mutex_unlock(&get_nmtp_details(SECONDARY_TUNER_TYPE)->reset_lock);
			}
			nmtp->reset_num_acks = 0;
			nmtp->reset_status = NMTP_UNACKED_RESET_IN_PROGRESS;
			wake_up_interruptible(&nmtp->reset_poll_wait);

			change_reset_state(RESET_CHIPS);
		}
		mutex_unlock(&nmtp->reset_lock);
	}

	return HRTIMER_NORESTART;
}

void trigger_reset(void)
{
	struct nmtp_data *nmtp = get_nmtp_details(0);
	struct nmtp_data *nmtp_secondary = get_nmtp_details(1);
	if (nmtp) {
		mutex_lock(&nmtp->reset_lock);
		nmtp->reset_num_acks = 0;
		nmtp->reset_status = NMTP_FORCED_RESET_PENDING;
		mutex_unlock(&nmtp->reset_lock);
		wake_up_interruptible(&nmtp->reset_poll_wait);

		if (nmtp_secondary) {
			mutex_lock(&nmtp_secondary->reset_lock);
			nmtp_secondary->reset_num_acks = 1;
			nmtp_secondary->reset_status = NMTP_FORCED_RESET_PENDING;
			mutex_unlock(&nmtp_secondary->reset_lock);
			wake_up_interruptible(&nmtp_secondary->reset_poll_wait);
		}
		// Start a timeout timer
		hrtimer_cancel(&nmtp->reply_timer);
		hrtimer_start(&nmtp->reply_timer, ktime_set(0, MS_TO_NS(100)), HRTIMER_MODE_REL);
	}
}


static ssize_t mrd_reset_read(struct file *f, char __user *buf, size_t count,
		loff_t *offset)
{

	if (f->private_data) {
		int reset_status;
		ssize_t status;
		u8 read_data;
		instance_data_t *instance_data = (instance_data_t *)f->private_data;

		mutex_lock(&instance_data->nmtp_data->reset_lock);
		reset_status = instance_data->nmtp_data->reset_status;
		mutex_unlock(&instance_data->nmtp_data->reset_lock);

		// Temporary code so that we can print out when new status read
		if (reset_status != instance_data->last_value) {
			if (instance_data->nmtp_data->type->mode == SECONDARY_TUNER_TYPE)
				dev_dbg(&instance_data->nmtp_data->spi->dev, "[Hnmtp] secondary reset status = 0x%x [%s] [%d]  \n", reset_status, __func__, __LINE__);
			else
				dev_dbg(&instance_data->nmtp_data->spi->dev, "[Hnmtp] primary reset status = 0x%x [%s] [%d]  \n", reset_status, __func__, __LINE__);
		}

		switch (reset_status) {
		case NMTP_UNACKED_RESET_IN_PROGRESS:
		case NMTP_ACKED_RESET_IN_PROGRESS:
		case NMTP_NORMAL_WAIT_FOR_ACK:
		default:
			read_data = 0;
			break;
		case NMTP_NORMAL_OPERATION:
			read_data = 1;
			break;
		case NMTP_REQUEST_RESET_PENDING:
			read_data = 2;
			break;
		case NMTP_FORCED_RESET_PENDING:
			read_data = 2;
			break;
		case NMTP_SUSPEND_STATE:
			read_data = 3;
			break;
		case NMTP_RESUME_STATE:
			read_data = 4;
			break;
		case NMTP_SPIDEV_MODE:
			read_data = 5;
			break;
		}

		mutex_lock(&instance_data->nmtp_data->reset_lock);
		if (reset_status == NMTP_UNACKED_RESET_IN_PROGRESS) {
			if (instance_data->last_value != NMTP_UNACKED_RESET_IN_PROGRESS) {
				++instance_data->nmtp_data->reset_num_reads;
				if (instance_data->nmtp_data->reset_num_reads == 2 /*instance_data->nmtp_data->reset_num_open*/) {
					instance_data->nmtp_data->reset_status = NMTP_ACKED_RESET_IN_PROGRESS;
					instance_data->nmtp_data->reset_num_reads = 0;
					wake_up_interruptible(&instance_data->nmtp_data->reset_poll_wait);
				}
			}
		} else if (reset_status == NMTP_NORMAL_WAIT_FOR_ACK) {
			if (instance_data->last_value != NMTP_NORMAL_WAIT_FOR_ACK) {
				++instance_data->nmtp_data->reset_num_reads;
				if (instance_data->nmtp_data->reset_num_reads == 2 /*instance_data->nmtp_data->reset_num_open*/) {
					instance_data->nmtp_data->reset_status = NMTP_NORMAL_OPERATION;
					instance_data->nmtp_data->reset_num_reads = 0;
					wake_up_interruptible(&instance_data->nmtp_data->reset_poll_wait);
				}
			}
		}
		instance_data->last_value = reset_status;
		mutex_unlock(&instance_data->nmtp_data->reset_lock);

		status = copy_to_user(buf, &read_data, 0x1);
		if (status)
			return -EFAULT;

		return 0x1;
	} else {
		return -EINVAL;
	}
}

static unsigned int mrd_reset_poll(struct file *f, struct poll_table_struct *wait)
{
	int mask = 0;
	if (f->private_data) {
		instance_data_t *instance_data = (instance_data_t *)f->private_data;
		poll_wait(f, &instance_data->nmtp_data->reset_poll_wait, wait);

		mutex_lock(&instance_data->nmtp_data->reset_lock);
		if (instance_data->nmtp_data->reset_status != instance_data->last_value) {
			dev_dbg(&instance_data->nmtp_data->spi->dev, "[Hnmtp]  [%s] [%d]\n", __func__, __LINE__);
			mask = POLLIN | POLLRDNORM;
		}
		mutex_unlock(&instance_data->nmtp_data->reset_lock);

	} else {
		mask = POLLERR;
	}

	return mask;
}

static ssize_t mrd_reset_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	unsigned long delay_in_ns;
	ktime_t interval;
	ssize_t rc = count;
	static int ignore_reset_acknowledge;

	if (f->private_data) {
		instance_data_t *instance_data = (instance_data_t *)f->private_data;

		if (count == 1) {
			u8 command;
			int ready_for_reset = 0;
			int status = copy_from_user(&command, buf, 0x1);

			if (status) {
				rc = -EIO;
			} else {
				dev_dbg(&instance_data->nmtp_data->spi->dev, "[Hnmtp]  [%s] [%d]  command = %d\n", __func__, __LINE__, command);
				switch (command) {
				case 0:
					// Reset request
					mutex_lock(&instance_data->nmtp_data->reset_lock);
					if (instance_data->nmtp_data->reset_status != NMTP_NORMAL_OPERATION) {
						rc = -EBUSY;
					} else {
						// Ignore reset requests for secondary tuner
						if (instance_data->nmtp_data->type->mode == PRIMARY_TUNER_TYPE) {
							// Start a timeout timer
							delay_in_ns = MS_TO_NS(100);
							interval = ktime_set(0, delay_in_ns);
							hrtimer_cancel(&instance_data->nmtp_data->reply_timer);
							hrtimer_start(&instance_data->nmtp_data->reply_timer, interval, HRTIMER_MODE_REL);

							if (get_nmtp_details(1)) {
								mutex_lock(&get_nmtp_details(1)->reset_lock);
								get_nmtp_details(1)->reset_num_acks = 1;
								get_nmtp_details(1)->reset_status = NMTP_REQUEST_RESET_PENDING;
								wake_up_interruptible(&get_nmtp_details(1)->reset_poll_wait);
								mutex_unlock(&get_nmtp_details(1)->reset_lock);
							}

							instance_data->nmtp_data->reset_num_acks = 0;
							instance_data->nmtp_data->reset_status = NMTP_REQUEST_RESET_PENDING;
							wake_up_interruptible(&instance_data->nmtp_data->reset_poll_wait);
						}
					}
					mutex_unlock(&instance_data->nmtp_data->reset_lock);
					break;
				case 1:
					// Reset Ack
					if (ignore_reset_acknowledge)
						break;
					mutex_lock(&instance_data->nmtp_data->reset_lock);
					if ((instance_data->nmtp_data->reset_status == NMTP_REQUEST_RESET_PENDING)
					 || (instance_data->nmtp_data->reset_status == NMTP_FORCED_RESET_PENDING)) {

						++instance_data->nmtp_data->reset_num_acks;
						if (instance_data->nmtp_data->reset_num_acks == 2 /*instance_data->nmtp_data->reset_num_open*/) {
							instance_data->nmtp_data->reset_num_acks = 0;
							instance_data->nmtp_data->reset_status = NMTP_UNACKED_RESET_IN_PROGRESS;
							wake_up_interruptible(&instance_data->nmtp_data->reset_poll_wait);
							if (instance_data->nmtp_data->type->mode == PRIMARY_TUNER_TYPE) {
								ready_for_reset = 1;
							}
						}
					}
					mutex_unlock(&instance_data->nmtp_data->reset_lock);

					if (ready_for_reset) {
						// Perform the reset
						change_reset_state(RESET_CHIPS);
					} else {
						wake_up_interruptible(&instance_data->nmtp_data->reset_poll_wait);
					}

					break;
				case 2:
					// Reset delay
					mutex_lock(&instance_data->nmtp_data->reset_lock);
					if (instance_data->nmtp_data->reset_status == NMTP_REQUEST_RESET_PENDING) {
						// Restart a timeout timer
						int timer_status;
						delay_in_ns = MS_TO_NS(100);
						interval = ktime_set(0, delay_in_ns);
						timer_status = hrtimer_cancel(&instance_data->nmtp_data->reply_timer);
						if (timer_status == 1) {
							// Only restart the timer if it was active
							hrtimer_start(&instance_data->nmtp_data->reply_timer, interval, HRTIMER_MODE_REL);
						}
					}
					mutex_unlock(&instance_data->nmtp_data->reset_lock);
					break;
				/* Case 3: Removed for the longterm s2r support 
				 * Write 0x3, S2R event confirmation is not required from
				 * Tunerapp
				*/
				case 4:
					// Used to trigger a reset from the command line for testing
					trigger_reset();
					break;

				case 5:
					// Used to disable acknowlege processing for testing
					ignore_reset_acknowledge = 1;
					break;

				case 6:
					// Used to enable acknowlege processing for testing
					ignore_reset_acknowledge = 0;
					break;

				default:
					break;
				}
			}
		} else {
			rc = -EIO;
		}
	} else {
		rc = -EINVAL;
	}
	return rc;
}

static int mrd_reset_open(struct file *f, int tuner_type)
{
	int rc = SUCCESS;

	// Allocate private storage
	instance_data_t *instance_data = kzalloc(sizeof(instance_data_t), GFP_KERNEL);

	if (instance_data) {
		instance_data->nmtp_data = get_nmtp_details(tuner_type);
		dev_dbg(&instance_data->nmtp_data->spi->dev, "[Hnmtp]  [%s] [%d]  \n", __func__, __LINE__);
		instance_data->last_value = 255;
		f->private_data = instance_data;
	} else {
		rc = -ENOMEM;
	}

	return rc;
}

static int mrd_reset_master_open(struct inode *i, struct file *f)
{
	return mrd_reset_open(f, PRIMARY_TUNER_TYPE);
}

static int mrd_reset_slave_open(struct inode *i, struct file *f)
{
	return mrd_reset_open(f, SECONDARY_TUNER_TYPE);
}

static int mrd_reset_release(struct inode *i, struct file *f)
{
	if (f->private_data) {
		instance_data_t *instance_data = (instance_data_t *)f->private_data;
		dev_dbg(&instance_data->nmtp_data->spi->dev, "[Hnmtp]  [%s] [%d]  \n", __func__, __LINE__);

		// Free up the data allocated in the open command
		kfree(f->private_data);
		f->private_data = 0;
		return SUCCESS;
	} else {
		return -EINVAL;
	}
}

static const struct file_operations mrd_reset_master_fops = {
	.owner		= THIS_MODULE,
	.open		= mrd_reset_master_open,
	.read		= mrd_reset_read,
	.write		= mrd_reset_write,
	.poll		= mrd_reset_poll,
	.release	= mrd_reset_release,
};

static const struct file_operations mrd_reset_slave_fops = {
	.owner		= THIS_MODULE,
	.open		= mrd_reset_slave_open,
	.read		= mrd_reset_read,
	.write		= mrd_reset_write,
	.poll		= mrd_reset_poll,
	.release	= mrd_reset_release,
};

int nmtp_reset_init(struct nmtp_data *nmtp)
{
	unsigned long minor;
	struct device *dev;
	int status;
	static const struct file_operations mrd_reset_fops;
	dev_dbg(&nmtp->spi->dev, "[Hnmtp]  [%s] [%d]  \n", __func__, __LINE__);

	nmtp->reset_status = NMTP_NORMAL_WAIT_FOR_ACK;

	init_waitqueue_head(&nmtp->reset_poll_wait);
	mutex_init(&nmtp->reset_lock);

	hrtimer_init(&nmtp->reply_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	nmtp->reply_timer.function = &timerHandler;

	if (mrd_reset_major == 0) {
		mrd_reset_major = register_chrdev(0, "nmtp_reset", &mrd_reset_fops);
		if (mrd_reset_major < 0) {
			pr_err("MRD RESET Device Registration failed\n");
			return mrd_reset_major;
		}

		mrd_reset_class = class_create(THIS_MODULE, "nmtp_reset");
		if (IS_ERR(mrd_reset_class)) {
			unregister_chrdev(mrd_reset_major, "nmtp_reset");
			mrd_reset_major = 0;
			return FAILURE;
		}
	}

	minor = find_first_zero_bit(minors, DYNAMIC_MINORS);
	if (minor >= DYNAMIC_MINORS) {
		pr_err("no minor number available!\n");
		status = -ENODEV;
		goto end;
	}
	nmtp->reset_devt = MKDEV(mrd_reset_major, minor);

	if (nmtp->type->mode == PRIMARY_TUNER_TYPE) {
		cdev_init(&nmtp->reset_node_cdev, &mrd_reset_master_fops);
		cdev_add(&nmtp->reset_node_cdev, nmtp->reset_devt, 1);

		dev = device_create(mrd_reset_class, &nmtp->spi->dev, nmtp->reset_devt,
				nmtp, "nmtp_master_reset");
	} else {
		cdev_init(&nmtp->reset_node_cdev, &mrd_reset_slave_fops);
		cdev_add(&nmtp->reset_node_cdev, nmtp->reset_devt, 1);
		dev = device_create(mrd_reset_class, &nmtp->spi->dev, nmtp->reset_devt,
				nmtp, "nmtp_slave_reset");
	}
	status = PTR_ERR_OR_ZERO(dev);
	if (status) {
		pr_err("Could not create device file for minor %lu\n", minor);
		goto end;
	}
	set_bit(minor, minors);

	if (nmtp->type->mode == PRIMARY_TUNER_TYPE) {
		nmtp_ipc_initialize();
		nmtp_chip_reset_handler_initialize();
	}

end:
	dev_dbg(&nmtp->spi->dev, "[Hnmtp]  [%s] [%d]  \n", __func__, __LINE__);

	return 0;
}

void nmtp_reset_exit(struct nmtp_data *nmtp)
{
	static int a;
	device_destroy(mrd_reset_class, nmtp->reset_devt);
	clear_bit(MINOR(nmtp->reset_devt), minors);
	if (a == 0x1) {
		list_del(&nmtp->device_entry);
		class_destroy(mrd_reset_class);
		unregister_chrdev(mrd_reset_major, "nmtp_reset");

		nmtp_ipc_exit();
		nmtp_chip_reset_handler_exit();
	}
	a++;
}

