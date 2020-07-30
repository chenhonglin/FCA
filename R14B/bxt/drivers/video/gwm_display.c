/*
 * drivers/video/gwm_display.c
 * FPDLink Serializer DS90UB929/Deserializer DS90UB948 for gwm display driver
 */
#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c/gwm_display.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

/* I2C Addresses used */
#define I2C_ADDR_SER_PORT0             (0x0C)
#define I2C_ADDR_DESER                 (0x2c)
#define I2C_MCU_ADDR          	       (0x3a)
#define I2C_ADDR_S7883_TC              (0x20)

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
#define SER_VALUE_TOUCH_ADDR           (0x40)
#define SER_VALUE_GPIO0                (0x17)
#define SER_VALUE_LVDS_BYPASS          (0xDA)
#define SER_VALUE_I2C_CONTROL          (0x9E)
#define SER_VALUE_INT_CONTROL          (0x21)
#define DESER_VALUE_GPIO0              (0x23)
#define DESER_VALUE_GPIO7              (0x09)
#define DESER_VALUE_GPIO7_OFF          (0x01)
#define DESER_VALUE_MAP_SELECT         (0x60)
#define DESER_VALUE_LOCK_BIT           (0x01)
#define DESER_SCL_HIGH_VALUE           (0x0C)
#define DESER_SCL_LOW_VALUE            (0x1A)
#define DESER_VALUE_GPIO6              (0x90)

#ifdef CONFIG_GWMV3_BOARD
#define GPIO_BASE_BANK_NORTH    357  /* GWMV3 HW */
#define LVDS_OUT1_RST_Q         39   /* GWMV3 HW */
#elif CONFIG_GWM_BOARD
#define GPIO_BASE_BANK_NORTH    434  /* GWMV2 HW */
#define LVDS_OUT1_RST_Q         33   /* GWMV2 HW */
#endif

#define SER_PDB_GPIO            (GPIO_BASE_BANK_NORTH + LVDS_OUT1_RST_Q)

struct i2c_client *ser_global_client;

//static const __u8 cmd_backlight_on[] = { 0x26, 0x00, 0x42, 0x00, 0x22, 0x01};

struct gwm_display_rec {
	struct gwm_display_platform_data *pdata;
	struct workqueue_struct         *work_queue;
	struct delayed_work              work_item;
	struct i2c_board_info            touch_device_board_info;
	struct i2c_board_info            mcu_device_board_info;
	struct i2c_client               *deser_client;
	struct i2c_client               *touch_client;
	struct i2c_client               *mcu_client;
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

extern int gwm_ser_read_isr(void)
{
	uint8_t val;
	int err = 0;

	err = ser_i2c_read(ser_global_client, SER_REG_INT_RECEIVE, &val);
	if (err < 0)
		dev_err(&ser_global_client->dev, "INT reciever 0xC7 read fail err=%d\n", err);
	dev_dbg(&ser_global_client->dev, "INT reciever 0xC7: 0x%u\n", val);
	return err;
}

EXPORT_SYMBOL_GPL(gwm_ser_read_isr);

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

	/* Then the gwm_display_rec pointer */
	struct gwm_display_rec *data =
		container_of(delayed_work, struct gwm_display_rec, work_item);

	pr_notice("gwm_display: delay complete - starting mcu and touch driver\n");

	/* Now time to announce that the mcu and touch controller appeared */
	data->touch_client = i2c_new_device(data->deser_client->adapter,
		&data->touch_device_board_info);
	if (data->touch_client == NULL) {
		pr_err("Unable to create new touch client\n");
		return;
	}

	data->mcu_client = i2c_new_device(data->deser_client->adapter,
		&data->mcu_device_board_info);
	if (data->mcu_client == NULL) {
		pr_err("Unable to create new mcu client\n");
		return;
	}

	i2c_release_client(data->touch_client);
	data->touch_client = 0;

	i2c_release_client(data->mcu_client);
	data->mcu_client = 0;

}

static void check_lock_status(struct work_struct *work_arg)
{
	struct delayed_work *delayed_work =
		container_of(work_arg, struct delayed_work, work);

	/* Then the gwm_display_rec pointer */
	struct gwm_display_rec *data =
		container_of(delayed_work, struct gwm_display_rec, work_item);

	int err;

	err = is_deserializer_locked(data->deser_client);
	if (err == 1) {
		pr_notice("gwm_display: LVDS_LOCK detected, starting delay\n");
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

static int gwm_display_enable(struct i2c_client *client)
{
	struct gwm_display_rec *data = i2c_get_clientdata(client);
	int err;
	uint8_t val;
	
	memset(&data->touch_device_board_info, 0, sizeof(data->touch_device_board_info));
	strlcpy(data->touch_device_board_info.type, "synaptics_dsx_i2c", I2C_NAME_SIZE);
	data->touch_device_board_info.addr          = I2C_ADDR_S7883_TC;
	data->touch_device_board_info.platform_data = &data->pdata->synaptics_platform_data;
	
	memset(&data->mcu_device_board_info, 0, sizeof(data->mcu_device_board_info));
	strlcpy(data->mcu_device_board_info.type, "stmicro_mcu_i2c", I2C_NAME_SIZE);
	data->mcu_device_board_info.addr            = I2C_MCU_ADDR;
	data->mcu_device_board_info.platform_data = NULL;

        if (devm_gpio_request_one(&client->dev, SER_PDB_GPIO, 0,
                                "SERIALIZER PDB") != 0) {
                                dev_err(&client->dev, "unable to acquire: SERIALIZER PDB %d\n",
                                SER_PDB_GPIO);
                                return -ENODEV;
        }

	gpio_set_value(SER_PDB_GPIO, 0);
	usleep_range(5000, 10000);
	gpio_set_value(SER_PDB_GPIO, 1);
	usleep_range(5000, 10000);
	do {
	    /* Configures the Serializer with Synaptics_dsx Touch controller I2C slave address (remote side) */
		err = ser_i2c_write(client, SER_REG_SLAVE_ADDR_0, SER_VALUE_TOUCH_ADDR);
		if (err < 0)
			break;

		/* Configures the Serializer with Synaptics_dsx Touch controller I2C slave address (local side) */
		err = ser_i2c_write(client, SER_REG_SLAVE_ALIAS_0, SER_VALUE_TOUCH_ADDR);
		if (err < 0)
			break;

	    /* Configure the serializer's INTB for back channel Interrupt from Touch controller */
		err = ser_i2c_write(client, SER_REG_INT_CONTROL, SER_VALUE_INT_CONTROL);
		if (err < 0)
			break;

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

		/* Enable the communication over I2C Control (to deserializer and touch controller) */
		err = ser_i2c_write(client, SER_REG_I2C_CONTROL, SER_VALUE_I2C_CONTROL);
		if (err < 0)
			break;

		/*Read from HDCP_ISR register to arm the interrupt for the first time*/
		err = ser_i2c_read(client, SER_REG_INT_RECEIVE, &val);
		if (err < 0)
			break;

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

		/* Configure the Deserializer's GPIO6 to reset touch screen synaptics*/
		err = ser_i2c_write(data->deser_client, DESER_REG_GPIO6, DESER_VALUE_GPIO6);
		if (err < 0)
			break;

		/* Change the color packing order - Fixes psychodelic colors on 1080p display */
		err = ser_i2c_write(data->deser_client, DESER_REG_MAP_SELECT, DESER_VALUE_MAP_SELECT);
		if (err < 0)
			break;

		/* Create work queue */
		data->work_queue = create_singlethread_workqueue("gwm_display");

		/* Kick off work queue item to sample LVDS lock */
		INIT_DELAYED_WORK(&data->work_item, check_lock_status);
		queue_delayed_work(data->work_queue, &data->work_item, HZ/10);

	} while (0);

	if (err < 0) {
		pr_err("gwm_display_enable: i2c read/write errir!\n");
		return err;
	} else {
		if (data->touch_client) {
			pr_notice("gwm_display:Releasing touch client\n");
			i2c_release_client(data->touch_client);
			data->touch_client = 0;
		}

		if (data->mcu_client) {
			pr_notice("gwm_display:Releasing mcu client\n");
			i2c_release_client(data->mcu_client);
			data->mcu_client = 0;
		}

		pr_notice("gwm_display:Report success\n");
		return 0;
	}
}

static void gwm_display_disable(struct i2c_client *client)
{
	struct gwm_display_rec *data = i2c_get_clientdata(client);

	if (data->mcu_client) {
		pr_notice("Removing mcu client\n");
		i2c_unregister_device(data->mcu_client);
		data->mcu_client = NULL;
	}

	if (data->touch_client) {
		pr_notice("Removing touch client\n");
		i2c_unregister_device(data->touch_client);
		data->touch_client = NULL;
	}

	if (data->deser_client) {
		//pr_notice("Turning off backlight");
		//ser_i2c_write(data->deser_client, DESER_REG_GPIO7, DESER_VALUE_GPIO7_OFF);

		pr_notice("Removing deser client\n");
		i2c_unregister_device(data->deser_client);
		data->deser_client = NULL;
	}
}

static void release_all_resources(struct gwm_display_rec *data)
{
	if (data->work_queue) {
		destroy_workqueue(data->work_queue);
		data->work_queue = 0;
	}

	if (data->deser_client) {
		i2c_unregister_device(data->deser_client);
		data->deser_client = 0;
	}

	if (data->touch_client) {
		i2c_unregister_device(data->touch_client);
		data->touch_client = 0;
	}

	if (data->mcu_client) {
		i2c_unregister_device(data->mcu_client);
		data->mcu_client = 0;
	}

}

static int gwm_display_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct gwm_display_rec *data;
	struct gwm_display_platform_data *pdata =
		client->dev.platform_data;
	int err;
	pr_notice("gwm_display_probe: success\n");

	ser_global_client = client;

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
	err = gwm_display_enable(client);
	if (err < 0) {
		release_all_resources(data);
		i2c_set_clientdata(client, NULL);
		return err;
	} else
		return 0;
}

static int gwm_display_remove(struct i2c_client *client)
{
	struct gwm_display_rec *data = i2c_get_clientdata(client);
	gwm_display_disable(client);
	i2c_set_clientdata(client, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gwm_display_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	gwm_display_disable(client);
	return 0;
}

static int gwm_display_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return gwm_display_enable(client);
}
#endif

static const struct i2c_device_id gwm_display_id[] = {
	{ "gw_display", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, gwm_display_id);

#ifdef CONFIG_PM_SLEEP
static SIMPLE_DEV_PM_OPS(gwm_display_pm_ops, gwm_display_suspend, gwm_display_resume);
#endif

static struct i2c_driver gwm_display_driver = {
	.driver = {
		.name  = "gwm_display",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm	   = &gwm_display_pm_ops,
#endif
	},
	.probe    = gwm_display_probe,
	.remove   = gwm_display_remove,
	.id_table = gwm_display_id,
};

module_i2c_driver(gwm_display_driver);

MODULE_AUTHOR("Xingfeng Ye <Xingfeng.Ye@harman.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("gwm_display FPDLink Serializer driver");
MODULE_ALIAS("i2c:gw_display");
