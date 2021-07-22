/*
 * drivers/video/r1_display.c
 * FPDLink Serializer/Deserializer for R1 display driver
 * R1 Display here Refers to Titan 1080p Display
 * TODO: File & functions name shall be changed to  avoid future conflicts between Titan & FCA R1
 */
#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c/r1_display.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

/* I2C Addresses used */
#define I2C_ADDR_SER_PORT0             (0x0C)
#define I2C_ADDR_DESER                 (0x2C)
#define I2C_ADDR_ATMEL_TC              (0x4A)

/* I2C Chip registers */
#define SER_REG_TRANS_PORT_SEL         (0x1E)
#define SER_REG_SLAVE_ADDR_0           (0x07)
#define SER_REG_SLAVE_ALIAS_0          (0x08)
#define SER_REG_GPIO0                  (0x0D)
#define SER_REG_GENERAL_CONFIGURATION  (0x03)
#define DESER_REG_GPIO0                (0x1D)
#define DESER_REG_GPIO7                (0x21)
#define DESER_REG_MAP_SELECT           (0x49)
#define DESER_REG_GENERAL_STATUS       (0x1C)


/* I2C Chip Values / Masks */
#define SER_VALUE_TOUCH_ADDR           (0x94) // 0x4A left shifted
#define SER_VALUE_GPIO0                (0x17)
#define SER_VALUE_LVDS_BYPASS          (0xDA)
#define DESER_VALUE_GPIO0              (0x23)
#define DESER_VALUE_GPIO7              (0x09)
#define DESER_VALUE_GPIO7_OFF          (0x01)
#define DESER_VALUE_MAP_SELECT         (0x40)
#define DESER_VALUE_LOCK_BIT           (0x01)

struct r1_display_rec {
	struct r1_display_platform_data *pdata;
	struct workqueue_struct         *work_queue;
	struct delayed_work              work_item;
	struct i2c_board_info            touch_device_board_info;
	struct i2c_client               *deser_client;
    struct i2c_client               *touch_client;
};

static void check_lock_status(struct work_struct *work_arg);


static int ser_i2c_read(struct i2c_client *client, int reg, uint8_t *val)
{
	int ret;
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read dev:0x%.2hhX reg:0x%.2hhX\n",
			client->addr, reg);
		return ret;
	}
	*val = ret;
	return 0;
}

static int ser_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret)
		dev_err(&client->dev, 
			"Failed to write to dev:0x%.2hhX reg:0x%.2hhX val:0x%.2hhX\n",
			client->addr, reg, val );
	return ret;
}

static int is_deserializer_locked(struct i2c_client* client)
{
	uint8_t val;
	int     err;

	/* Read the lock status of the deserializer */
	err = ser_i2c_read(client, DESER_REG_GENERAL_STATUS, &val);
	if(err < 0) {
		return err;
	}

	return ((val & DESER_VALUE_LOCK_BIT) == DESER_VALUE_LOCK_BIT ) ? 1 : 0;
}

static void check_lock_loss(struct work_struct *work_arg){

	/* Dig out the delayed_work pointer	*/
	struct delayed_work *delayed_work =
		container_of(work_arg, struct delayed_work, work);

	/* Then the r1_display_rec pointer */
	struct r1_display_rec *data = 
		container_of(delayed_work, struct r1_display_rec, work_item);

	int err;

	err = is_deserializer_locked(data->deser_client);
	if (err == 1)
	{
		/* video still locked, just reset timer */
		queue_delayed_work(data->work_queue, &data->work_item, HZ/10); 
	}
	else if (err == 0)
	{
		/* lost video lock, unload touch driver, turn off backlight, 
		   set queue thread to start looking for video lock */

		if (data->touch_client){
			pr_notice("r1_display: Lost Lock, remove touch client\n");
			i2c_unregister_device(data->touch_client);
			data->touch_client = NULL;	
		}
		
		if (data->deser_client){
			pr_notice("r1_display: Turn off backlight");
			ser_i2c_write(data->deser_client, DESER_REG_GPIO7, DESER_VALUE_GPIO7_OFF);
		}

		/* Kick off work queue item to sample LVDS lock */
//		INIT_DELAYED_WORK(&data->work_item, check_lock_status);
//		queue_delayed_work(data->work_queue, &data->work_item, HZ/10); 
	}
	else
	{
		pr_err("r1_display: unable to even talk to the deserializer?");
	}
}


static void register_touch_controller(struct work_struct* work_arg){

	/* Dig out the delayed_work pointer	*/
	struct delayed_work *delayed_work =
		container_of(work_arg, struct delayed_work, work);

	/* Then the r1_display_rec pointer */
	struct r1_display_rec *data = 
		container_of(delayed_work, struct r1_display_rec, work_item);

	pr_notice("r1_display: delay complete - start touch driver\n");

	/* Now time to announce that the touch controller appeared */
	data->touch_client = i2c_new_device(data->deser_client->adapter, 
		&data->touch_device_board_info);
	if (data->touch_client == NULL) {
		pr_err("r1_display: Unable to create new touch client\n");
		return;
	}
//	i2c_release_client(data->touch_client);
//	data->touch_client = 0;

	/* touch controller loaded, now start looking for LVDS_LOCK lost */
	INIT_DELAYED_WORK(&data->work_item, check_lock_loss);
	queue_delayed_work(data->work_queue, &data->work_item, HZ/10); 
}

static void check_lock_status(struct work_struct *work_arg){

	/* Dig out the delayed_work pointer	*/
	struct delayed_work *delayed_work =
		container_of(work_arg, struct delayed_work, work);

	/* Then the r1_display_rec pointer */
	struct r1_display_rec *data = 
		container_of(delayed_work, struct r1_display_rec, work_item);

	int err;

	err = is_deserializer_locked(data->deser_client);
	if (err == 1)	/* detected LVDS_LOCK*/
	{
	    /* Configure the Deserializer's GPIO7 to enable the power for the remote display */
		err = ser_i2c_write(data->deser_client, DESER_REG_GPIO7, DESER_VALUE_GPIO7);
		if (err < 0)
			goto err_exit;

//		pr_notice("r1_display: enable backlight\n");
		/* Start the required 230ms delay */
		pr_notice("r1_display: LVDS_LOCK detected, turn backlight on, start delay for TC power\n");
		INIT_DELAYED_WORK(&data->work_item, register_touch_controller);
		queue_delayed_work(data->work_queue, &data->work_item, HZ/4); 
	}
	else if (err == 0)
	{
		/* TC still not ready...  Resubmit this function */
		queue_delayed_work(data->work_queue, &data->work_item, HZ/10); 
	}
	else
	{
err_exit:
		pr_err("r1_display: unable to even talk to the deserializer?");
	}
}


static int r1_display_enable(struct i2c_client *client)
{
	struct r1_display_rec *data = i2c_get_clientdata(client);
	int err;

	pr_notice(">>>r1_display_enable call\n");

	memset(&data->touch_device_board_info, 0, sizeof(data->touch_device_board_info));
	strlcpy(data->touch_device_board_info.type, "atmel_mxt_ts", I2C_NAME_SIZE);
	data->touch_device_board_info.addr          = I2C_ADDR_ATMEL_TC;
	data->touch_device_board_info.platform_data = &data->pdata->mxt_platform_data;

	do {
	    /* Configures the Serializer with Atmel Touch controller I2C slave address (remote side) */
		err = ser_i2c_write(client, SER_REG_SLAVE_ADDR_0, SER_VALUE_TOUCH_ADDR);
		if (err < 0)
			break;

		/* Configures the Serializer with Atmel Touch controller I2C slave address (local side) */
		err = ser_i2c_write(client, SER_REG_SLAVE_ALIAS_0, SER_VALUE_TOUCH_ADDR);
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

		/* Create a new i2c_client for the deserializer address */
		data->deser_client = i2c_new_dummy(client->adapter, I2C_ADDR_DESER);
		if (data->deser_client == NULL){
			err=-ENODEV;
			break;
		}

		/* Configure the deserializer's GPIO0 for back channel Interrupt from Touch controller */
		err = ser_i2c_write(data->deser_client, DESER_REG_GPIO0, DESER_VALUE_GPIO0);
		if (err < 0)
			break;
#if 0
	    /* Configure the Deserializer's GPIO7 to enable the power for the remote display */
		err = ser_i2c_write(data->deser_client, DESER_REG_GPIO7, DESER_VALUE_GPIO7);
		if (err < 0)
			break;
#endif
		/* Change the color packing order - Fixes psychodelic colors on 1080p display */
		err = ser_i2c_write(data->deser_client, DESER_REG_MAP_SELECT, DESER_VALUE_MAP_SELECT);
		if (err < 0)
			break;

		/* Create work queue */
		data->work_queue = create_singlethread_workqueue("r1_display");

		/* Kick off work queue item to sample LVDS lock */
		INIT_DELAYED_WORK(&data->work_item, check_lock_status);
		queue_delayed_work(data->work_queue, &data->work_item, HZ/10); 

	}while(0);
	
	if (err<0){
		// Free everything
		pr_notice(">>>r1_display: enable err\n");
		return err;
	}
	else
	{
		// Free only touch client (if present)
		if(data->touch_client){
			pr_notice("r1_display: Releasing touch client\n");
			i2c_release_client(data->touch_client);
			data->touch_client = 0;
		}
		pr_notice(">>>r1_display: enable success\n");
		return 0;
	}
	
}

static void r1_display_disable(struct i2c_client *client)
{
	struct r1_display_rec *data = i2c_get_clientdata(client);

	if (data->touch_client){
		pr_notice("r1_display: removing touch client\n");
		i2c_unregister_device(data->touch_client);
		data->touch_client = NULL;	
	}
	
	if (data->deser_client){
		pr_notice("r1_display: Turning off backlight");
		ser_i2c_write(data->deser_client, DESER_REG_GPIO7, DESER_VALUE_GPIO7_OFF);

		pr_notice("r1_display: removing deser client\n");
		i2c_unregister_device(data->deser_client);
		data->deser_client = NULL;	
	}
}

static release_all_resources(struct r1_display_rec* data)
{
	if (data->work_queue){
		cancel_delayed_work_sync(&data->work_item);
                drain_workqueue(data->work_queue);
		destroy_workqueue(data->work_queue);
		data->work_queue = 0;
	}

	if (data->touch_client){
		pr_notice("r1_display: removing touch client\n");
		i2c_unregister_device(data->touch_client);
		data->touch_client = NULL;	
	}
	
	if (data->deser_client){
//		pr_notice("r1_display: Turning off backlight");
//		ser_i2c_write(data->deser_client, DESER_REG_GPIO7, DESER_VALUE_GPIO7_OFF);

		pr_notice("r1_display: removing deser client\n");
		i2c_unregister_device(data->deser_client);
		data->deser_client = NULL;	
	}
}

static int r1_display_probe(struct i2c_client *client,
					        const struct i2c_device_id *id)
{
	struct r1_display_rec *data;
	struct r1_display_platform_data *pdata =
		client->dev.platform_data;
	int err;
	pr_notice(">>> r1_display: probe call\n");
	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}
	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}
	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	data->pdata = pdata;

	i2c_set_clientdata(client, data);
	err = r1_display_enable(client);
	if (err < 0){
		release_all_resources(data);
		i2c_set_clientdata(client, NULL);
		return err;
	}
	else{
		return 0;
	}
	pr_notice(">>> r1_display: probe end\n");
}

static int r1_display_remove(struct i2c_client *client)
{
	struct r1_display_rec *data = i2c_get_clientdata(client);
//	r1_display_disable(client);
	release_all_resources(data);
//	i2c_set_clientdata(client, NULL);
	return 0;
}
static void r1_display_shutdown(struct i2c_client *client)
{
	struct r1_display_rec *data = i2c_get_clientdata(client);
//	r1_display_disable(client);
	release_all_resources(data);
//	i2c_set_clientdata(client, NULL);
//	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int r1_display_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct r1_display_rec *data = i2c_get_clientdata(client);
//	r1_display_disable(client);
	release_all_resources(data);
//	i2c_set_clientdata(client, NULL);
	return 0;
}

static int r1_display_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return r1_display_enable(client);
}
#endif

static const struct i2c_device_id r1_display_id[] = {
	{ "r1_display", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, r1_display_id);

static SIMPLE_DEV_PM_OPS(r1_display_pm_ops, r1_display_suspend, r1_display_resume);

static struct i2c_driver r1_display_driver = {
	.driver = {
		.name  = "r1_display",
		.owner = THIS_MODULE,
		.pm	   = &r1_display_pm_ops,
	},
	.probe    = r1_display_probe,
	.remove   = r1_display_remove,
	.shutdown = r1_display_shutdown,
	.id_table = r1_display_id,
};

module_i2c_driver(r1_display_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("r1_display FPDLink Serializer driver");
MODULE_ALIAS("i2c:r1_display");

