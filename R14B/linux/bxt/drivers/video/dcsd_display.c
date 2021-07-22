/*****************************************************************************
 * Project              Harman Car Multimedia System
 *
 * c) copyright         2019
 * Company              Harman International Industries, Incorporated
 *                      All rights reserved
 * Secrecy Level    	STRICTLY CONFIDENTIAL
 *****************************************************************************/
/*
 * drivers/video/dcsd_display.c
 * FPDLink Serializer/Deserializer for DCSD display driver
 */
#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c/dcsd_display.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/delay.h>

/* Touch device name */
#define TC_DEV_NAME_MOBIS			"dcsd_ts"
#define TC_DEV_NAME 				TC_DEV_NAME_MOBIS

/* I2C Addresses used */
#define I2C_ADDR_SER_PORT0			(0x0C)
#define I2C_ADDR_DESER				(0x2C)
#define I2C_ADDR_MOBIS_TC			(0x70)
#define I2C_ADDR_DCSD_TC			(0x12)

/* I2C Chip registers */
#define SER_REG_SLAVE_ADDR_0		(0x07)
#define SER_REG_SLAVE_ALIAS_0		(0x08)
#define SER_REG_GPIO0				(0x0D)
#define SER_REG_GENERAL_CONFIGURATION	(0x03)
#define SER_REG_ICR 					(0xC6)
#define DESER_REG_GPIO0 			(0x1D)
#define DESER_REG_GENERAL_STATUS	(0x1C)

/* I2C Chip Values / Masks */
#define SER_VALUE_GPIO0 			(0x1F)
#define SER_VALUE_LVDS_BYPASS		(0xDA)
#define SER_REG_ICR_ENABLE_RX_INT	(0x21)
#define DESER_VALUE_GPIO0			(0x23)
#define DESER_VALUE_LOCK_BIT		(0x01)

/* Display lock status values*/
#define DESER_SIGNAL_LOCK_INVALID (0xFFF)
#define DESER_SIGNAL_LOCK_OK (1)
#define DESER_SIGNAL_LOCK_NOT_OK (0)
#define DESER_ACCESS_RETRY_MS (1000)
#define DESER_SIGNAL_LOCK_RETRY_MS (250)

/* Display Detect Values */
#define DISPLAY_DETECT_INIT (0)
#define DISPLAY_DETECT_OK (1)
#define DISPLAY_DETECT_NOK (-1)
/* DISPLAY_DETECT_FAILURE_CNT = 5sec/DESER_ACCESS_RETRY_MS */
#define DISPLAY_DETECT_FAILURE_CNT (5)
/* DISPLAY_DETECT_LOCK_CNT = 5sec/DESER_SIGNAL_LOCK_RETRY_MS */
#define DISPLAY_DETECT_LOCK_CNT (20)

#define RETRY_TIME_MS			(25)

struct dcsd_display_rec {
	struct dcsd_display_platform_data *pdata;
	struct workqueue_struct 		*work_queue;
	struct delayed_work 			 work_item;
	struct i2c_board_info			 touch_device_board_info;
	struct i2c_client				*deser_client;
	struct i2c_client				*touch_client;
	int display_detect;
	int display_detect_fail_cnt;
};

#ifdef FCA_BACKLIGHT_STATE
static int prev_disp_state = -1;
extern int DisplayOn;
#endif
u8 touch_i2c_slave_addr;
EXPORT_SYMBOL(touch_i2c_slave_addr);
u8 ser_touch_i2c_slave_addr;
EXPORT_SYMBOL(ser_touch_i2c_slave_addr);

extern char *get_displaytype(void);

/*
 * dcsd_display_touch_disable is used control the touch enable disable status on dcsd_ts load time
 * We need this as dcsd_ts will be loaded onlt on DisplayOn and display signal is locked
 * during this time if any one sets touch disable we can handle it through this parameter
 */
static volatile bool dcsd_display_touch_disable = false;
bool get_dcsd_display_touch_disable_status(void)
{
	return dcsd_display_touch_disable;
}
EXPORT_SYMBOL(get_dcsd_display_touch_disable_status);

/*
 * dcsd_display_touch_resume is used to control the touch interrupt enable status on resume
 * We need to resume touch on dislay lock detected and display configurations are done
 */
static volatile bool dcsd_display_touch_resume = false;
bool allow_dcsd_display_touch_resume(void)
{
	return dcsd_display_touch_resume;
}
EXPORT_SYMBOL(allow_dcsd_display_touch_resume);

static int ser_i2c_read(struct i2c_client *client, int reg, uint8_t *val)
{
	int ret;
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err_ratelimited(&client->dev, "dcsd_display:Failed to read dev:0x%.2hhX reg:0x%.2hhX\n",
			client->addr, reg);
		return ret;
	}
	*val = ret;
	return 0;
}

static int ser_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	bool retry = false;

retry_write:
	ret=i2c_smbus_write_byte_data(client, reg, val);
	if (ret) {
		dev_err(&client->dev,
			"dcsd_display: Failed to write to dev:0x%.2hhX reg:0x%.2hhX val:0x%.2hhX, retry=%d\n",
			client->addr, reg, val, retry);
		if (!retry) {
			msleep(RETRY_TIME_MS);
			retry = true;
			goto retry_write;
		}
	}

	return ret;
}

static int is_deserializer_locked(struct i2c_client* client)
{
	uint8_t val;
	int 	err;

	/* Read the lock status of the deserializer */
	err = ser_i2c_read(client, DESER_REG_GENERAL_STATUS, &val);
	if(err < 0) {
		return err;
	}

	return ((val & DESER_VALUE_LOCK_BIT) == DESER_VALUE_LOCK_BIT ) ? DESER_SIGNAL_LOCK_OK : DESER_SIGNAL_LOCK_NOT_OK;
}

static void register_touch_controller(struct work_struct* work_arg){

	/* Dig out the delayed_work pointer */
	struct delayed_work *delayed_work =
		container_of(work_arg, struct delayed_work, work);

	/* Then the dcsd_display_rec pointer */
	struct dcsd_display_rec *data =
		container_of(delayed_work, struct dcsd_display_rec, work_item);
	int err;

	pr_info("dcsd_display: delay complete - starting touch driver\n");

	/* Configure the deserializer's GPIO0 for back channel Interrupt from Touch controller */
	err = ser_i2c_write(data->deser_client, DESER_REG_GPIO0, DESER_VALUE_GPIO0);
	if (err < 0) {
		pr_err("dcsd_display: Failed to write in de-serializer\n");
	}

	/* Announce touch_client only if we are not announced it earlier */
	if (data->touch_client == NULL) {
		/* Now time to announce that the touch controller appeared */
		data->touch_client = i2c_new_device(data->deser_client->adapter,
			&data->touch_device_board_info);
		if (data->touch_client == NULL) {
			pr_err("dcsd_display: Unable to create new touch client\n");
		} else {
			/* release use of the i2c client structure as we are finished with touch_client in this driver */
			i2c_release_client(data->touch_client);
		}
	}
	dcsd_display_touch_resume = true;
}

static void check_lock_status(struct work_struct *work_arg){

	/* Dig out the delayed_work pointer */
	struct delayed_work *delayed_work =
		container_of(work_arg, struct delayed_work, work);

	/* Then the dcsd_display_rec pointer */
	struct dcsd_display_rec *data =
		container_of(delayed_work, struct dcsd_display_rec, work_item);
	int err = DESER_SIGNAL_LOCK_INVALID;

#ifdef FCA_BACKLIGHT_STATE
	if(prev_disp_state != DisplayOn)
		pr_info("dcsd_display: check_lock_status: DisplayOn = %d\n", DisplayOn);

	if(DisplayOn == 1){
		err = is_deserializer_locked(data->deser_client);
	}
	prev_disp_state = DisplayOn;
#else
	err = is_deserializer_locked(data->deser_client);
#endif

	if (err == DESER_SIGNAL_LOCK_OK) {
		/* Start the required 230ms delay */
		pr_info("dcsd_display: LVDS_LOCK detected, starting delay\n");
		INIT_DELAYED_WORK(&data->work_item, register_touch_controller);
		queue_delayed_work(data->work_queue, &data->work_item, HZ/4);
		data->display_detect = DISPLAY_DETECT_OK;
		data->display_detect_fail_cnt = 0;
	}
	else if(err == DESER_SIGNAL_LOCK_INVALID) {
		/* Deserializer signal lock status not checked yet due to display is not ON */
		queue_delayed_work(data->work_queue, &data->work_item, msecs_to_jiffies(DESER_SIGNAL_LOCK_RETRY_MS));
	}
	else if (err == DESER_SIGNAL_LOCK_NOT_OK) {
		/* Deserializer signal lock not detected */
		if(data->display_detect_fail_cnt < DISPLAY_DETECT_LOCK_CNT) {
			data->display_detect_fail_cnt++;
			pr_info_ratelimited("dcsd_display: deserializer is not locked, cnt:%d\n", data->display_detect_fail_cnt);
		}
		if(data->display_detect_fail_cnt >= DISPLAY_DETECT_LOCK_CNT) {
			data->display_detect = DISPLAY_DETECT_NOK;
			pr_err_ratelimited("dcsd_display: deserializer is not locked, after cnt:%d\n", data->display_detect_fail_cnt);
		}
		queue_delayed_work(data->work_queue, &data->work_item, msecs_to_jiffies(DESER_SIGNAL_LOCK_RETRY_MS));
	}
	else {
		/* Deserializer not accessible or display not connected */
		if(data->display_detect_fail_cnt < DISPLAY_DETECT_FAILURE_CNT) {
			data->display_detect_fail_cnt++;
			pr_err_ratelimited("dcsd_display: unable to even talk to the deserializer, cnt:%d\n", data->display_detect_fail_cnt);
		}
		if(data->display_detect_fail_cnt >= DISPLAY_DETECT_FAILURE_CNT) {
			data->display_detect = DISPLAY_DETECT_NOK;
		}
		queue_delayed_work(data->work_queue, &data->work_item, msecs_to_jiffies(DESER_ACCESS_RETRY_MS));
	}
}

static int dcsd_display_enable(struct i2c_client *client)
{
	struct dcsd_display_rec *data = i2c_get_clientdata(client);
	int err;

	memset(&data->touch_device_board_info, 0, sizeof(data->touch_device_board_info));
	strlcpy(data->touch_device_board_info.type, TC_DEV_NAME, I2C_NAME_SIZE);
	data->touch_device_board_info.addr = touch_i2c_slave_addr;
	data->touch_device_board_info.platform_data = &data->pdata->dcsd_ts_pdata;

	do {
		/* Configures the Serializer with Touch controller I2C slave address (remote side) */
		err = ser_i2c_write(client, SER_REG_SLAVE_ADDR_0, ser_touch_i2c_slave_addr);
		if (err < 0)
			break;

		/* Configures the Serializer with Touch controller I2C slave address (local side) */
		err = ser_i2c_write(client, SER_REG_SLAVE_ALIAS_0, ser_touch_i2c_slave_addr);
		if (err < 0)
			break;

		/* Configure the serializer's GPIO0 for back channel Interrupt from Touch controller */
		err = ser_i2c_write(client, SER_REG_GPIO0, SER_VALUE_GPIO0);
		if (err < 0)
			break;

		/* Enable the communication over I2C link (to deserializer and touch controller) */
		err = ser_i2c_write(client, SER_REG_GENERAL_CONFIGURATION, SER_VALUE_LVDS_BYPASS);
		if (err < 0)
			break;

		/* Enable Interrupt on Receiver interrupt and Global Interrupt */
		err = ser_i2c_write(client, SER_REG_ICR, SER_REG_ICR_ENABLE_RX_INT);
		if (err < 0)
			break;

		/* Create a new i2c_client for the deserializer address */
		data->deser_client = i2c_new_dummy(client->adapter, I2C_ADDR_DESER);
		if (data->deser_client == NULL){
			err=-ENODEV;
			break;
		}

		/* Create work queue */
		data->work_queue = create_singlethread_workqueue("dcsd_display");
		if(data->work_queue == NULL)
		{
			dev_err(&client->dev, "Cannot create dcsd_display work_queue\n");
			err=-ENOMEM;
			break;
		}

		/* Kick off work queue item to sample LVDS lock */
		INIT_DELAYED_WORK(&data->work_item, check_lock_status);
		queue_delayed_work(data->work_queue, &data->work_item, HZ/10);

	}while(0);

	if (err<0){
		// Free everything
		return err;
	}
	else
	{
		pr_info("dcsd_display:Report success\n");
		return 0;
	}

}

static void release_all_resources(struct dcsd_display_rec* data)
{
	if (data->work_queue){
		cancel_delayed_work_sync(&data->work_item);
		drain_workqueue(data->work_queue);
		destroy_workqueue(data->work_queue);
		data->work_queue = 0;
	}

	if (data->deser_client){
		i2c_unregister_device(data->deser_client);
		data->deser_client = 0;
	}
}

static void dcsd_display_disable(struct i2c_client *client)
{
	struct dcsd_display_rec *data = i2c_get_clientdata(client);
	dcsd_display_touch_resume = false;
	release_all_resources(data);
}

static void update_touch_i2c_slave_address(void)
{
	const char disp_mobis[] = "mobis";
	char *display_type = get_displaytype();

	if ( strcmp(display_type, disp_mobis)==0 ) {
		touch_i2c_slave_addr = (u8)(I2C_ADDR_MOBIS_TC);
	} else {
		touch_i2c_slave_addr = (u8)(I2C_ADDR_DCSD_TC);
	}
	ser_touch_i2c_slave_addr = (u8)(touch_i2c_slave_addr<<1);
}

static ssize_t dcsd_display_detect_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dcsd_display_rec *data = i2c_get_clientdata(client);
	ssize_t ret;

	ret = snprintf( buf, PAGE_SIZE, "%d", data->display_detect);
	return ret;
}

static ssize_t dcsd_display_touch_disable_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	ret = snprintf( buf, PAGE_SIZE, "%d", dcsd_display_touch_disable);
	return ret;
}

static ssize_t dcsd_display_touch_disable_store(struct device *dev,
						struct device_attribute *attr,const char *buf,size_t count)
{
	unsigned long touch_disable;
	int retval;

	retval = kstrtoul(buf,10,&touch_disable);
	if(retval < 0)
		return retval;


	switch(touch_disable){
	case 0:
		if (dcsd_display_touch_disable == true) {
			dcsd_display_touch_disable = false;
		}
		break;

	case 1:
		if(dcsd_display_touch_disable == false) {
			dcsd_display_touch_disable = true;
		}
		break;

	default:
		count = -EINVAL;
		break;
	}

	return count;
}

static struct device_attribute attributes[] = {
	__ATTR(dcsd_display_detect, S_IRUGO, dcsd_display_detect_show, NULL),
	__ATTR(dcsd_display_touch_disable, S_IRUSR | S_IWUSR, dcsd_display_touch_disable_show, dcsd_display_touch_disable_store),

};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;
	for(i=0; i < ARRAY_SIZE(attributes);i++)
		if(device_create_file(dev,attributes+i))
			goto undo;
	return 0;

	undo:
		for(i--;i>=0;i--)
		device_remove_file(dev,attributes + i);
		dev_err(dev, "dcsd_display: %s: failed to create sysfs interface\n",__func__);
		return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for(i=0; i < ARRAY_SIZE(attributes);i++)
		device_remove_file(dev,attributes+i);
}

static int dcsd_display_probe(struct i2c_client *client,
							const struct i2c_device_id *id)
{
	struct dcsd_display_rec *data;
	struct dcsd_display_platform_data *pdata =
		client->dev.platform_data;
	int err;

	dev_info(&client->dev, "dcsd_display_probe start\n");
	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "dcsd_display_probe failed due to, SMBUS Byte Data not Supported\n");
		return -EIO;
	}
	if (!pdata) {
		dev_err(&client->dev, "dcsd_display_probe failed due to, no platform data?\n");
		return -EINVAL;
	}
	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev, "dcsd_display_probe failed due to ENOMEM\n");
		return -ENOMEM;
	}
	data->pdata = pdata;
	data->display_detect = DISPLAY_DETECT_INIT;

	update_touch_i2c_slave_address();

	i2c_set_clientdata(client, data);
	err = dcsd_display_enable(client);
	if (err < 0){
		dev_err(&client->dev, "dcsd_display_probe failed due to dcsd_display_enable failure, err=%d\n", err);
		release_all_resources(data);
		i2c_set_clientdata(client, NULL);
		return err;
	}
	else{
		err = add_sysfs_interfaces(&client->dev);
		if(err){
			dev_err(&client->dev,"%s: Error,fail sysfs init\n",__func__);
			return err;
		}
		dev_info(&client->dev, "dcsd_display_probe success\n");
		return 0;
	}
}

static int dcsd_display_remove(struct i2c_client *client)
{
	struct dcsd_display_rec *data = i2c_get_clientdata(client);
#ifdef FCA_BACKLIGHT_STATE
	prev_disp_state = -1;
#endif
	dev_info(&client->dev, "dcsd_display_remove\n");
	dcsd_display_disable(client);
	remove_sysfs_interfaces(&client->dev);
	devm_kfree(&client->dev, data);
	return 0;
}

static void dcsd_display_shutdown(struct i2c_client *client)
{
#ifdef FCA_BACKLIGHT_STATE
	prev_disp_state = -1;
#endif
	dev_info(&client->dev, "dcsd_display_shutdown\n");
	dcsd_display_disable(client);
	i2c_set_clientdata(client, NULL);
}

#ifdef CONFIG_PM_SLEEP
static int dcsd_display_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
#ifdef FCA_BACKLIGHT_STATE
	prev_disp_state = -1;
#endif
	dev_info(&client->dev, "dcsd_display_suspend\n");
	dcsd_display_disable(client);
	return 0;
}

static int dcsd_display_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	dev_info(&client->dev, "dcsd_display_resume\n");
	return dcsd_display_enable(client);
}
#endif

static const struct i2c_device_id dcsd_display_id[] = {
	{ "dcsd_display", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, dcsd_display_id);

static SIMPLE_DEV_PM_OPS(dcsd_display_pm_ops, dcsd_display_suspend, dcsd_display_resume);

static struct i2c_driver dcsd_display_driver = {
	.driver = {
		.name  = "dcsd_display",
		.owner = THIS_MODULE,
		.pm    = &dcsd_display_pm_ops,
	},
	.probe	  = dcsd_display_probe,
	.remove   = dcsd_display_remove,
	.shutdown = dcsd_display_shutdown,
	.id_table = dcsd_display_id,
};

module_i2c_driver(dcsd_display_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("dcsd_display FPDLink Serializer driver");
MODULE_ALIAS("i2c:dcsd_display");

