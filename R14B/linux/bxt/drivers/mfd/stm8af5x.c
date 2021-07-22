/*
 * I2C driver for ST Microcontroller Stm8af5288
 *
 * Copyright (C) 2018 Harman International Ltd.
 * Xingfeng Ye <Xingfeng.Ye@harman.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mfd/stm8af5x.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c-dev.h>
#include <linux/gpio.h>

#define LDVS_OUT1_PANEL0_VDDEN 363  //357 + 6
#define LVDS_OUT1_PANEL0_BKLTEN 364 //357 + 7

static char sw_version_ascii[6];

static dev_t devt = MKDEV(I2C_MAJOR, 50);

static struct class *stm8af5x_status_class;

static struct resource bl_resources[] = {
	{ read_command_id3, read_command_id3, "read brightness", IORESOURCE_REG, },
	{ write_command_id1, write_command_id1, "write brightness", IORESOURCE_REG, },
};

static const struct mfd_cell bl_devs[] = {
	{
		.name		= "stm8af5x-bl",
		.num_resources	= ARRAY_SIZE(bl_resources),
		.resources	= &bl_resources[0],
		.id		= -1,
	},
};

/*hex transfer to ASCII char*/
unsigned char HexToChar(unsigned char bchar)
{
	if((bchar >= 0x30) && (bchar <= 0x39))
		bchar -= 0x30;
	else if((bchar >= 0x41) && (bchar <= 0x46))
		bchar -= 0x37;
	else if((bchar >= 0x61) && (bchar <= 0x66))
		bchar -= 0x57;
	else
		bchar = 0xff;
	return bchar;
}

static int lvds_out_gpio_control(struct device *dev, unsigned int gpio,
	unsigned int rw, unsigned int value)
{
	int gpio_rval;

	if ((gpio == LDVS_OUT1_PANEL0_VDDEN) && (rw == 0)) {  /* read gpio363 status */
		gpio_rval = gpio_get_value(LDVS_OUT1_PANEL0_VDDEN);
		return gpio_rval;
	}
	else if ((gpio == LVDS_OUT1_PANEL0_BKLTEN) && (rw == 1)) {
		/* write gpio364 0 or 1, 0 is sleep, 1 is wakeup */
		gpio_set_value(LVDS_OUT1_PANEL0_BKLTEN, value);
	}
	else
		dev_err(dev, "Unable to acquire gpio\n");

	return 0;
}

static inline int stm8af5x_read_device(struct i2c_client *i2c,
				      int reg, int bytes, void *dest)
{
	int ret;

	if (bytes > 1)
	{
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, bytes, dest);
		if(ret < 0)
		{
			dev_err(&i2c->dev, "read from command 0x%x error %d\n", reg, ret);
			return ret;
		}
		else {
			dev_err(&i2c->dev, "read from command 0x%x bytes 0x%d\n", reg, ret);
		}
	}
	else {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret < 0)
		{
			dev_err(&i2c->dev, "read from command 0x%x error %d\n", reg, ret);
			return ret;
		}
		else {
			dev_err(&i2c->dev, "read from command 0x%x value 0x%x\n", reg, ret);
			*(unsigned char *)dest = (unsigned char)ret;
		}
	}
	return ret;
}

static inline int stm8af5x_write_device(struct i2c_client *i2c,
				       int reg, int bytes, void *src)
{
	unsigned char buf[4];
	int ret;

	buf[0] = (unsigned char)reg;
	memcpy(&buf[1], src, bytes);

	ret = i2c_master_send(i2c, buf, bytes + 1);
	if (ret < 0)
	{
		dev_err(&i2c->dev, "write to command 0x%x error %d\n", reg, ret);
		return ret;
	}
	dev_err(&i2c->dev, "write to command 0x%x bytes %d values 0x%x%x%x\n", reg, ret, buf[1], buf[2], buf[3]);
	return ret;
}

/*read display sw firmware version*/
static int read_stm8af5x_sw_version(struct i2c_client *i2c)
{
	int ret, i;
	int read_bytes = 5;
	unsigned char sw_version[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	struct stm8af5x_chip *chip = i2c_get_clientdata(i2c);

	ret = stm8af5x_read_device(chip->client, read_command_id1, read_bytes, sw_version);
	if (ret < 0) {
		dev_err(chip->dev, "read from command 0x%x error %d\n", read_command_id1, ret);
		return ret;
	}
	else {
		for (i = 0; i < 5; i++)
		{
			dev_err(chip->dev, "read from command 0x%x value 0x%x\n", read_command_id1, sw_version[i]);
		}
		if ((sw_version[0] == 0x53) && (sw_version[1] == 0x57))
		{
			sw_version_ascii[0] = 'S';	  /*0x53 ASCII is S*/
			sw_version_ascii[1] = 'W';	  /*0x57 ASCII is W*/
			sw_version_ascii[2] = '0' + HexToChar(sw_version[2]);
			sw_version_ascii[3] = '0' + HexToChar(sw_version[3]);
			sw_version_ascii[4] = '0' + HexToChar(sw_version[4]);
			sw_version_ascii[5] = '\0';
			dev_err(chip->dev, "report stm8af5x display sw version %s\n", sw_version_ascii);
		}
	}
	return 0;
}

/*read display internal status*/
static int read_stm8af5x_internal_status(struct i2c_client *i2c, int bytenumber)
{
	int ret;
	int read_bytes = 6;
	unsigned char internal_status[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	ret = stm8af5x_read_device(i2c, read_command_id2, read_bytes, internal_status);
	if (ret < 0) {
		dev_err(&i2c->dev, "read from command 0x%x error %d\n", read_command_id2, ret);
		return ret;
	}
	else {
		switch (bytenumber) {
		case 0:
			return internal_status[0];
			break;
		case 1:
			return internal_status[1];
			break;
		case 2:
			return internal_status[2];
			break;
		case 3:
			return internal_status[3];
			break;
		case 4:
			return internal_status[4];
			break;
		case 5:
			return internal_status[5];
			break;
		default:
			return 0;
			break;
		}
	}

}

static ssize_t sw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t	status;
	struct stm8af5x_chip *chip_data;
	chip_data = dev_get_drvdata(dev);
	read_stm8af5x_sw_version(chip_data->client);

	status = sprintf(buf, "%s\n", sw_version_ascii);
	if (status < 0) {
		dev_err(dev, "show display stm8af5x mcu sw version err = %d\n", status);
	}

	return status;
}
static DEVICE_ATTR_RO(sw_version);

static ssize_t ctp_voltage_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t status;
	struct stm8af5x_chip *chip_data;
	chip_data = dev_get_drvdata(dev);
	int ret;
	ret = read_stm8af5x_internal_status(chip_data->client, 0);
	if (ret < 0) {
		dev_err(dev, "read CTP voltage err = %d\n", ret);
		return ret;
	}
	status = sprintf(buf, "%d\n", ret);
	return status;
}
static DEVICE_ATTR_RO(ctp_voltage);

static ssize_t tft_voltage_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t status;
	struct stm8af5x_chip *chip_data;
	chip_data = dev_get_drvdata(dev);
	int ret;
	ret = read_stm8af5x_internal_status(chip_data->client, 1);
	if (ret < 0) {
		dev_err(dev, "read TFT voltage err = %d\n", ret);
		return ret;
	}
	status = sprintf(buf, "%d\n", ret);
	return status;
}
static DEVICE_ATTR_RO(tft_voltage);

static ssize_t bl_voltage_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t status;
	struct stm8af5x_chip *chip_data;
	chip_data = dev_get_drvdata(dev);
	int ret;
	ret = read_stm8af5x_internal_status(chip_data->client, 2);
	if (ret < 0) {
		dev_err(dev, "read backlight voltage err = %d\n", ret);
		return ret;
	}
	status = sprintf(buf, "%d\n", ret);
	return status;
}
static DEVICE_ATTR_RO(bl_voltage);

static ssize_t lvds_lock_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t status;
	struct stm8af5x_chip *chip_data;
	chip_data = dev_get_drvdata(dev);
	int ret;
	ret = read_stm8af5x_internal_status(chip_data->client, 3);
	if (ret < 0) {
		dev_err(dev, "read LVDS lock err = %d\n", ret);
		return ret;
	}
	status = sprintf(buf, "%d\n", ret);
	return status;
}
static DEVICE_ATTR_RO(lvds_lock);

static ssize_t lcd_temperature_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t status;
	struct stm8af5x_chip *chip_data;
	chip_data = dev_get_drvdata(dev);
	int ret;
	ret = read_stm8af5x_internal_status(chip_data->client, 4);
	if (ret < 0) {
		dev_err(dev, "read lcd temperature err = %d\n", ret);
		return ret;
	}
	status = sprintf(buf, "%d\n", ret);
	return status;
}
static DEVICE_ATTR_RO(lcd_temperature);

static ssize_t battery_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t status;
	struct stm8af5x_chip *chip_data;
	chip_data = dev_get_drvdata(dev);
	int ret;
	ret = read_stm8af5x_internal_status(chip_data->client, 5);
	if (ret < 0) {
		dev_err(dev, "read battery status err = %d\n", ret);
		return ret;
	}
	status = sprintf(buf, "%d\n", ret);
	return status;
}
static DEVICE_ATTR_RO(battery_status);

static ssize_t sleep_wakeup_control_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t status;
	int ret;
	ret = gpio_get_value(LVDS_OUT1_PANEL0_BKLTEN);
	status = sprintf(buf, "%d\n", ret);
	return status;
}
static ssize_t sleep_wakeup_control_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	int ret;
	unsigned char value;

	ret = kstrtou8(buf, 0, &value);
	if (ret)
		return ret;

	ret = lvds_out_gpio_control(dev, LVDS_OUT1_PANEL0_BKLTEN, 1, value);
	if (ret) {
		dev_err(dev, "write sleep wakeup status err = %d\n", ret);
		return ret;
	}
	return size;
}
static DEVICE_ATTR_RW(sleep_wakeup_control);

static struct attribute *stm8af5x_attrs[] = {
	&dev_attr_sw_version.attr,
	&dev_attr_ctp_voltage.attr,
	&dev_attr_tft_voltage.attr,
	&dev_attr_bl_voltage.attr,
	&dev_attr_lvds_lock.attr,
	&dev_attr_lcd_temperature.attr,
	&dev_attr_battery_status.attr,
	&dev_attr_sleep_wakeup_control.attr,
	NULL,
};

static const struct attribute_group stm8af5x_group = {
	.attrs = stm8af5x_attrs,
};

static const struct attribute_group *stm8af5x_groups[] = {
	&stm8af5x_group,
	NULL
};

/*Read display brightness value*/
int read_stm8af5x_display_btness(struct i2c_client *client, int reg, int count)
{
	int ret;
	u8 btness_value[3] = {0x00, 0x00, 0x00};
	int value;
	u16 value_u16;
	unsigned int bt_value;

	if (client)
	{
		ret = stm8af5x_read_device(client, reg, count, btness_value);
		if (ret < 0) {
			dev_err(&client->dev, "read from command 0x%x error %d\n", reg, ret);
			return ret;
		}
		if (((btness_value[0] + btness_value[1]) & 0xFF) == btness_value[2])
		{
			value_u16 = btness_value[0];
			value_u16 <<= 8;
			value_u16 |= btness_value[1];
			value = 100 * value_u16;
			bt_value = (value / 65535);
			dev_err(&client->dev, "read from command 0x%x value 0x%x dec %d percent %d%\n",
				reg, value_u16, value_u16, bt_value);
			return bt_value;
		}
	}
	else {
		return -1;
	}
}
EXPORT_SYMBOL(read_stm8af5x_display_btness);

/*write display brightness value*/
int write_stm8af5x_display_btness(struct i2c_client *client, int reg,
		unsigned char data)
{
	u16 value_u16;
	int value, ret;
	value = 65535 * data;
	value_u16 = (value / 100);
	unsigned char btness[3] = {0x00, 0x00, 0x00};
	btness[0] = value_u16 >> 8;      /*MSB*/
	btness[1] = value_u16 & 0xFF;      /*LSB*/
	btness[2] = (btness[0] + btness[1]) & 0xFF;   /*checksum = last byte of (MSB + LSB)*/
	ret = stm8af5x_write_device(client, reg, 3, btness);
	if(ret < 0) {
		dev_err(&client->dev, "write to command 0x%x error %d\n", reg, ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(write_stm8af5x_display_btness);



static int stm8af5x_device_init(struct i2c_client *client)
{
	int ret;
	struct stm8af5x_chip *chip = i2c_get_clientdata(client);

	ret = read_stm8af5x_sw_version(client);
	if (ret < 0) {
		dev_err(chip->dev, "read from command 0x%x error %d\n", read_command_id1, ret);
		goto out;
	}

	ret = mfd_add_devices(chip->dev, 0, &bl_devs[0],
			      ARRAY_SIZE(bl_devs),
			      NULL, 0, NULL);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add stm8af5x subdev\n");
		goto out;
	}

	ret = devm_gpio_request(chip->dev, LDVS_OUT1_PANEL0_VDDEN, "lvds tft vdd on");
	if (ret != 0) {
		dev_err(chip->dev, "Unable to acquire gpio %d\n", LDVS_OUT1_PANEL0_VDDEN);
		goto out;
	}
	gpio_direction_input(LDVS_OUT1_PANEL0_VDDEN);

	ret = devm_gpio_request(chip->dev, LVDS_OUT1_PANEL0_BKLTEN, "lvds tft bklt en");
	if (ret != 0) {
		dev_err(chip->dev, "Unable to acquire gpio %d\n", LVDS_OUT1_PANEL0_BKLTEN);
		goto out;
	}
	gpio_direction_output(LVDS_OUT1_PANEL0_BKLTEN, 1);

	return 0;
out:
	return ret;
}

static void stm8af5x_device_exit(struct i2c_client *client)
{
	struct stm8af5x_chip *chip = i2c_get_clientdata(client);
	if (chip)
	{
		devm_gpio_free(chip->dev, LVDS_OUT1_PANEL0_BKLTEN);
		devm_gpio_free(chip->dev, LDVS_OUT1_PANEL0_VDDEN);
		mfd_remove_devices(chip->dev);
		dev_err(chip->dev, "delete stm8af5x subdev\n");
	}
}

static int stm8af5x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	static struct stm8af5x_chip *chip;
	struct device *stm8af5x_dev;

	chip = devm_kzalloc(&client->dev,
			    sizeof(struct stm8af5x_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->client = client;
	i2c_set_clientdata(client, chip);
	dev_set_drvdata(chip->dev, chip);
	mutex_init(&chip->io_lock);

	stm8af5x_status_class = class_create(THIS_MODULE, "stm8af5x_class");
	stm8af5x_dev = device_create_with_groups(stm8af5x_status_class, &client->dev,
		devt, chip, stm8af5x_groups, "stm8af5x_device");
	if (IS_ERR(stm8af5x_dev)) {
		class_destroy(stm8af5x_status_class);
		return PTR_ERR(stm8af5x_dev);
	}

	stm8af5x_device_init(client);

	return 0;
}

static int stm8af5x_remove(struct i2c_client *client)
{
	struct stm8af5x_chip *chip = i2c_get_clientdata(client);

	stm8af5x_device_exit(client);
	device_destroy(stm8af5x_status_class, devt);
	class_destroy(stm8af5x_status_class);
	dev_set_drvdata(chip->dev, NULL);
	i2c_set_clientdata(client, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int stm8af5x_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct stm8af5x_chip *chip = i2c_get_clientdata(client);
	stm8af5x_device_exit(client);
	return 0;
}

static int stm8af5x_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct stm8af5x_chip *chip = i2c_get_clientdata(client);
	stm8af5x_device_init(client);
	return 0;
}

static SIMPLE_DEV_PM_OPS(stm8af5x_pm_ops, stm8af5x_pm_suspend, stm8af5x_pm_resume);
#endif

static const struct i2c_device_id stm8af5x_id[] = {
	{ "stmicro_mcu_i2c", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, stm8af5x_id);

static struct i2c_driver stm8af5x_driver = {
	.driver = {
		.name  = "stm8af5x",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm	   = &stm8af5x_pm_ops,
#endif
	},
	.probe    = stm8af5x_probe,
	.remove   = stm8af5x_remove,
	.id_table = stm8af5x_id,
};

module_i2c_driver(stm8af5x_driver);

MODULE_AUTHOR("Xingfeng Ye <Xingfeng.Ye@harman.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("GWMV2 Display STmicro MCU control driver");
MODULE_ALIAS("i2c:stmicro_mcu_i2c");



