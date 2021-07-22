/*
 * drivers/video/bjev_vtp_display.c
 * FPDLink Serializer/Deserializer  for bjev vtp display driver
 */
#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c/bjev_vtp_display.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

/* I2C Addresses used */
#define I2C_ADDR_SER_PORT0             (0x1a)
#define I2C_ADDR_DESER                 (0x2c)
#define I2C_MCU_ADDR          	       (0x3a)
#define I2C_ADDR_S7883_TC              (0x20)
#define I2C_ADDR_CYAT81688_TC          (0x24)

/* I2C Chip registers */
#define SER_REG_TRANS_PORT_SEL         (0x1E)
#define SER_REG_SLAVE_ADDR_0           (0x07)
#define SER_REG_SLAVE_ALIAS_0          (0x08)
#define SER_REG_SLAVE_ADDR_1           (0x70)
#define SER_REG_SLAVE_ALIAS_1          (0x77)
#define SER_REG_GPIO0                  (0x0D)
#define SER_REG_GENERAL_CONFIGURATION  (0x03)
#define SER_REG_I2C_CONTROL            (0x17)
#define SER_REG_INT_CONTROL            (0xC6)
#define SER_REG_INT_RECEIVE            (0xC7)
#define DESER_REG_GPIO0                (0x1D)
#define DESER_REG_GPIO7                (0x21)
#define DESER_REG_MAP_SELECT           (0x49)
#define DESER_REG_GENERAL_STATUS       (0x1C)
#define DESER_SCL_HIGH_REG             (0x26)
#define DESER_SCL_LOW_REG              (0x27)
#define DESER_REG_GPIO6                (0x20)

/* I2C Chip Values / Masks */
#define SER_VALUE_TOUCH_ADDR           (0x48)
#define SER_VALUE_GPIO0                (0x17)
#define SER_VALUE_LVDS_BYPASS          (0xDA)
#define SER_VALUE_I2C_CONTROL          (0x9E)
#define SER_VALUE_INT_CONTROL          (0x21)
#define DESER_VALUE_GPIO0              (0x23)
#define DESER_VALUE_GPIO7              (0x09)
#define DESER_VALUE_GPIO7_OFF          (0x01)
//#define DESER_VALUE_MAP_SELECT         (0x60)
#define DESER_VALUE_MAP_SELECT         (0x40)
#define DESER_VALUE_LOCK_BIT           (0x01)
#define DESER_SCL_HIGH_VALUE           (0x0C)
#define DESER_SCL_LOW_VALUE            (0x1A)
#define DESER_VALUE_GPIO6              (0x90)

struct i2c_client *vtp_ser_global_client;


struct bjev_vtp_display_rec {
	struct bjev_vtp_display_platform_data *pdata;
	struct workqueue_struct         *work_queue;
	struct delayed_work              work_item;
	struct i2c_client               *deser_client;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
	struct i2c_board_info            touch1_device_board_info;
	struct i2c_client               *touch1_client;
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
	struct i2c_board_info            touch2_device_board_info;
	struct i2c_client               *touch2_client;
#endif

};

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
			client->addr, reg, val);
	return ret;
}

static int ser_i2c_writebytes(const struct i2c_client *client, const char *buf, int count)
{
	int i;
	int ret = i2c_master_send(client, buf, count);
	if (ret < 0)
		dev_err(&client->dev,
			"Failed to write to dev:0x%.2hhX %d bytes\n", client->addr, count);
	return ret;
}

extern void bjev_vtp_ser_read_isr(void)
{
	uint8_t val;
	int err;

	err = ser_i2c_read(vtp_ser_global_client, SER_REG_INT_RECEIVE, &val);
	if (err < 0)
		dev_err(&vtp_ser_global_client->dev, "INT reciever 0xC7 read fail err=%d\n", err);
	dev_dbg(&vtp_ser_global_client->dev, "INT reciever 0xC7: 0x%u\n", val);
}

EXPORT_SYMBOL_GPL(bjev_vtp_ser_read_isr);

static int is_deserializer_locked(struct i2c_client *client)
{
	uint8_t val;
	int     err;

	/* Read the lock status of the deserializer */
	err = ser_i2c_read(client, DESER_REG_GENERAL_STATUS, &val);
	if (err < 0) {
		return err;
	}

	return ((val & DESER_VALUE_LOCK_BIT) == DESER_VALUE_LOCK_BIT) ? 1 : 0;
}

static void register_mcu_touch_controller(struct work_struct *work_arg)
{
	struct delayed_work *delayed_work =
		container_of(work_arg, struct delayed_work, work);

	/* Then the bjev_vtp_display_rec pointer */
	struct bjev_vtp_display_rec *data =
		container_of(delayed_work, struct bjev_vtp_display_rec, work_item);

	pr_notice("bjev_vtp_display: delay complete - starting mcu and touch driver\n");
	printk(KERN_ERR"bjev_vtp_display: delay complete - starting mcu and touch driver\n");

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
	/* Now time to announce that the mcu and touch controller appeared */
	data->touch1_client = i2c_new_device(data->deser_client->adapter,
		&data->touch1_device_board_info);
	if (data->touch1_client == NULL) {
		pr_err("Unable to create new touch1 client\n");
		return;
	}

	i2c_release_client(data->touch1_client);
	data->touch1_client = 0;
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
	/* Now time to announce that the mcu and touch controller appeared */
	data->touch2_client = i2c_new_device(data->deser_client->adapter,
		&data->touch2_device_board_info);
	if (data->touch2_client == NULL) {
		pr_err("Unable to create new touch2 client\n");
		return;
	}

	i2c_release_client(data->touch2_client);
	data->touch2_client = 0;
#endif

}

static void check_lock_status(struct work_struct *work_arg)
{
	struct delayed_work *delayed_work =
		container_of(work_arg, struct delayed_work, work);

	/* Then the bjev_vtp_display_rec pointer */
	struct bjev_vtp_display_rec *data =
		container_of(delayed_work, struct bjev_vtp_display_rec, work_item);

	int err;

	err = is_deserializer_locked(data->deser_client);
	if (err == 1) {
		pr_notice("bjev_vtp_display: LVDS_LOCK detected, starting delay\n");
		printk(KERN_ERR"bjev_vtp_display: LVDS_LOCK detected, starting delay\n");
		INIT_DELAYED_WORK(&data->work_item, register_mcu_touch_controller);
		queue_delayed_work(data->work_queue, &data->work_item, HZ/4);
	} else if (err == 0)
		queue_delayed_work(data->work_queue, &data->work_item, HZ/10);
	else
		pr_err("unable to even talk to the deserializer?");
}

#if 0
static int setup_comm_over_backchannel(struct i2c_client *client)
{
	int err = 0;

	do {
		/* Set up slave address to talk to RL78 */
		err = ser_i2c_write(client, SER_REG_SLAVE_ADDR_1, I2C_RL78_ADDR << 1);
		if (err < 0)
			break;

		/* Set up slave alias to talk to RL78 */
		err = ser_i2c_write(client, SER_REG_SLAVE_ALIAS_1, I2C_RL78_ADDR << 1);
		if (err < 0)
			break;
	} while (0);

	return err;
}

static int setup_remote_side(struct i2c_client *client)
{
	struct i2c_client *rl78_client = NULL;
	int err = 0;

	do {
		/* Change the I2C SCL HIGH time (required to talk to RL87) */
		err = ser_i2c_write(client, DESER_SCL_HIGH_REG, DESER_SCL_HIGH_VALUE);
		if (err < 0)
			break;

		/* Change the I2C SCL LOW time (required to talk to RL87) */
		err = ser_i2c_write(client, DESER_SCL_LOW_REG, DESER_SCL_LOW_VALUE);
		if (err < 0)
			break;

		/* Create a new i2c_client for the rl78 address */
		rl78_client = i2c_new_dummy(client->adapter, I2C_RL78_ADDR);
		if (rl78_client == NULL) {
			err = -ENODEV;
			break;
		}

		/* Write the backlight on command */
		err = ser_i2c_writebytes(rl78_client, cmd_backlight_on, sizeof(cmd_backlight_on));
	} while (0);

	if (rl78_client != NULL) {
		i2c_unregister_device(rl78_client);
		rl78_client = NULL;
	}

	return err;
}
#endif

static int bjev_vtp_display_enable(struct i2c_client *client)
{
	struct bjev_vtp_display_rec *data = i2c_get_clientdata(client);
	int err;
	uint8_t val;

	dev_err(&client->dev, "bjev_vtp_display_enable: entry\n");

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
	memset(&data->touch1_device_board_info, 0, sizeof(data->touch1_device_board_info));
	strlcpy(data->touch1_device_board_info.type, "cyttsp6_i2c_adapter2", I2C_NAME_SIZE);
	data->touch1_device_board_info.addr          = I2C_ADDR_CYAT81688_TC;
	data->touch1_device_board_info.platform_data = &data->pdata->cypress_platform_data;

#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
	memset(&data->touch2_device_board_info, 0, sizeof(data->touch2_device_board_info));
	strlcpy(data->touch2_device_board_info.type, "synaptics_dsx_i2c_vtp", I2C_NAME_SIZE);
	data->touch2_device_board_info.addr          = I2C_ADDR_S7883_TC;
	data->touch2_device_board_info.platform_data = &data->pdata->synaptics_platform_data;
#endif

	do {
#if 1
		/* Configures the Serializer with Touch chip controller I2C slave address (remote side) */
		err = ser_i2c_write(client, SER_REG_SLAVE_ADDR_0, SER_VALUE_TOUCH_ADDR);
		if (err < 0)
			break;

		/* Configures the Serializer with Touch chip Touch controller I2C slave address (local side) */
		err = ser_i2c_write(client, SER_REG_SLAVE_ALIAS_0, SER_VALUE_TOUCH_ADDR);
		if (err < 0)
			break;
#if 0
		/* Configures the Serializer dual pixel mode */
		err = ser_i2c_write(client, 0x4f, 0x00);
		if (err < 0)
			break;
#endif

		/* Configure the serializer's INTB for back channel Interrupt from Touch controller */
		err = ser_i2c_write(client, SER_REG_INT_CONTROL, SER_VALUE_INT_CONTROL);
		if (err < 0)
			break;
#endif

		#if 0
	    /* Configure the serializer's GPIO0 for back channel Interrupt from Touch controller */
		err = ser_i2c_write(client, SER_REG_GPIO0, SER_VALUE_GPIO0);
		if (err < 0)
			break;

		/*Configure the RL78 I2C slave and address*/
		err = setup_comm_over_backchannel(client);
		if (err < 0)
			break;
		#endif
		/* Enable the communication over I2C link (to deserializer and touch controller) */
		err = ser_i2c_write(client, SER_REG_GENERAL_CONFIGURATION, SER_VALUE_LVDS_BYPASS);
		if (err < 0)
			break;
#if 1
		/* Enable the communication over I2C Control (to deserializer and touch controller) */
		err = ser_i2c_write(client, SER_REG_I2C_CONTROL, SER_VALUE_I2C_CONTROL);
		if (err < 0)
			break;

		/*Read from HDCP_ISR register to arm the interrupt for the first time*/
		err = ser_i2c_read(client, SER_REG_INT_RECEIVE, &val);
		if (err < 0)
			break;
		/* Set 947 gpio1/2 remote hold mode */
		err = ser_i2c_write(client, 0x0e, 0x55);
		if (err < 0)
			break;
#endif
		/* Create a new i2c_client for the deserializer address */
		data->deser_client = i2c_new_dummy(client->adapter, I2C_ADDR_DESER);
		if (data->deser_client == NULL) {
			err = -ENODEV;
			break;
		}

		#if 0
		/* Turn the far cluster on */
		err = setup_remote_side(data->deser_client);
		if (err < 0)
			break;

		/* Configure the deserializer's GPIO0 for back channel Interrupt from Touch controller */
		err = ser_i2c_write(data->deser_client, DESER_REG_GPIO0, DESER_VALUE_GPIO0);
		if (err < 0)
			break;

		/* Configure the Deserializer's GPIO7 to enable the power for the remote display */
		err = ser_i2c_write(data->deser_client, DESER_REG_GPIO7, DESER_VALUE_GPIO7);
		if (err < 0)
			break;
		#endif
#if 0
		/* Configure the Deserializer's GPIO6 to reset touch screen synaptics*/
		err = ser_i2c_write(data->deser_client, DESER_REG_GPIO6, DESER_VALUE_GPIO6);
		if (err < 0)
			break;
#endif
		val = 0;
		err = ser_i2c_read(data->deser_client, 0x00, &val);
		dev_err(&client->dev, "Read TI948 deser ID=%d\n",val);
		if (err < 0)
			break;

		/* Change the color packing order - Fixes psychodelic colors on 1080p display */
		err = ser_i2c_write(data->deser_client, DESER_REG_MAP_SELECT, DESER_VALUE_MAP_SELECT);
		if (err < 0)
			break;

		 /* Set 948 gpio1/2 input */
		 err =ser_i2c_write(data->deser_client, 0x1e, 0x33);
		 if(err < 0)
			break;
#if 1
		/* Set 948 I2C speed from 75KHZ to 400KHZ --Ian */
		err =ser_i2c_write(data->deser_client, 0x26, 0x19);
		if(err < 0)
			break;

		 err =ser_i2c_write(data->deser_client, 0x27, 0x19);
		if(err < 0)
			break;
#endif

#if 1
		/* Create work queue */
		data->work_queue = create_singlethread_workqueue("bjev_vtp_display");

		/* Kick off work queue item to sample LVDS lock */
		INIT_DELAYED_WORK(&data->work_item, check_lock_status);
		queue_delayed_work(data->work_queue, &data->work_item, HZ/10);
#endif
	} while (0);

	if (err < 0) {
		pr_err("bjev_vtp_display_enable: i2c read/write errir!\n");
		return err;
	} else {
		/*if (data->touch_client) {
			pr_notice("bjev_vtp_display:Releasing touch client\n");
			i2c_release_client(data->touch_client);
			data->touch_client = 0;
		}*/
		pr_notice("bjev_vtp_display:Report success\n");
		return 0;
	}
}

static void bjev_vtp_display_disable(struct i2c_client *client)
{
	struct bjev_vtp_display_rec *data = i2c_get_clientdata(client);

#ifdef	CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
	if (data->touch1_client) {
		pr_notice("Removing touch1 client\n");
		i2c_unregister_device(data->touch1_client);
		data->touch1_client = NULL;
	}
#endif

#ifdef	CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
	if (data->touch2_client) {
		pr_notice("Removing touch2 client\n");
		i2c_unregister_device(data->touch2_client);
		data->touch2_client = NULL;
	}
#endif

	if (data->deser_client) {
		//pr_notice("Turning off backlight");
		//ser_i2c_write(data->deser_client, DESER_REG_GPIO7, DESER_VALUE_GPIO7_OFF);

		pr_notice("Removing deser client\n");
		i2c_unregister_device(data->deser_client);
		data->deser_client = NULL;
	}
}

static void release_all_resources(struct bjev_vtp_display_rec *data)
{
	if (data->work_queue) {
		destroy_workqueue(data->work_queue);
		data->work_queue = 0;
	}

	if (data->deser_client) {
		i2c_unregister_device(data->deser_client);
		data->deser_client = 0;
	}
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
	if (data->touch1_client) {
		i2c_unregister_device(data->touch1_client);
		data->touch1_client = 0;
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
	if (data->touch2_client) {
		i2c_unregister_device(data->touch2_client);
		data->touch2_client = 0;
	}
#endif

}

static int bjev_vtp_display_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct bjev_vtp_display_rec *data;
	struct bjev_vtp_display_platform_data *pdata =
		client->dev.platform_data;
	int err;
	pr_notice("bjev_vtp_display_probe: success\n");

	vtp_ser_global_client = client;

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
	err = bjev_vtp_display_enable(client);
	if (err < 0) {
		release_all_resources(data);
		i2c_set_clientdata(client, NULL);
		return err;
	} else
		return 0;
}

static int bjev_vtp_display_remove(struct i2c_client *client)
{
	struct bjev_vtp_display_rec *data = i2c_get_clientdata(client);
	bjev_vtp_display_disable(client);
	i2c_set_clientdata(client, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bjev_vtp_display_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	bjev_vtp_display_disable(client);
	return 0;
}

static int bjev_vtp_display_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return bjev_vtp_display_enable(client);
}
#endif

static const struct i2c_device_id bjev_vtp_display_id[] = {
	{ "bjev_vtp_display", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bjev_vtp_display_id);

static SIMPLE_DEV_PM_OPS(bjev_vtp_display_pm_ops, bjev_vtp_display_suspend, bjev_vtp_display_resume);

static struct i2c_driver bjev_vtp_display_driver = {
	.driver = {
		.name  = "bjev_vtp_display",
		.owner = THIS_MODULE,
		.pm	   = &bjev_vtp_display_pm_ops,
	},
	.probe    = bjev_vtp_display_probe,
	.remove   = bjev_vtp_display_remove,
	/*.shutdown = bjev_vtp_display_remove,*/
	.id_table = bjev_vtp_display_id,
};

module_i2c_driver(bjev_vtp_display_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("bjev_vtp_display FPDLink Serializer driver");
MODULE_ALIAS("i2c:bjev_vtp_display");

