/*
 * drivers/video/int_dis_adv7613_serdes.c
 * FPDLink Serializer/Deserializer for primary and secondary display driver
 */
#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>

//GPIO for ADV7613 reset
#define SERDES_PWD	467
#define ADV7613_RESET_DELAY_MS	5

static int adv7613_enable_seq(struct i2c_client *client)
{
   int adv7613_init_values[][3] = {
	{0x4c, 0xF9, 0x64}, {0x4c, 0xFA, 0x6C}, {0x32, 0x74, 0x00}, {0x36, 0x00, 0x00},
	{0x36, 0x01, 0xFF}, {0x36, 0x02, 0xFF}, {0x36, 0x03, 0xFF}, {0x36, 0x04, 0xFF},
	{0x36, 0x05, 0xFF}, {0x36, 0x06, 0xFF}, {0x36, 0x07, 0x00}, {0x36, 0x08, 0x0D},
	{0x36, 0x09, 0xAF}, {0x36, 0x0A, 0x00}, {0x36, 0x0B, 0x00}, {0x36, 0x0C, 0x01},
	{0x36, 0x0D, 0x00}, {0x36, 0x0E, 0x00}, {0x36, 0x0F, 0x00}, {0x36, 0x10, 0x01},
	{0x36, 0x11, 0x00}, {0x36, 0x12, 0x01}, {0x36, 0x13, 0x03}, {0x36, 0x14, 0x80},
	{0x36, 0x15, 0x29}, {0x36, 0x16, 0x1F}, {0x36, 0x17, 0x78}, {0x36, 0x18, 0x08},
	{0x36, 0x19, 0x96}, {0x36, 0x1A, 0x05}, {0x36, 0x1B, 0x8D}, {0x36, 0x1C, 0x64},
	{0x36, 0x1D, 0x2B}, {0x36, 0x1E, 0x5C}, {0x36, 0x1F, 0x93}, {0x36, 0x20, 0x4C},
	{0x36, 0x21, 0x40}, {0x36, 0x22, 0x50}, {0x36, 0x23, 0x00}, {0x36, 0x24, 0x00},
	{0x36, 0x25, 0x00}, {0x36, 0x26, 0x01}, {0x36, 0x27, 0x01}, {0x36, 0x28, 0x01},
	{0x36, 0x29, 0x01}, {0x36, 0x2A, 0x01}, {0x36, 0x2B, 0x01}, {0x36, 0x2C, 0x01},
	{0x36, 0x2D, 0x01}, {0x36, 0x2E, 0x01}, {0x36, 0x2F, 0x01}, {0x36, 0x30, 0x01},
	{0x36, 0x31, 0x01}, {0x36, 0x32, 0x01}, {0x36, 0x33, 0x01}, {0x36, 0x34, 0x01},
	{0x36, 0x35, 0x01}, {0x36, 0x36, 0xE2}, {0x36, 0x37, 0x17}, {0x36, 0x38, 0x00},
	{0x36, 0x39, 0x21}, {0x36, 0x3A, 0x50}, {0x36, 0x3B, 0x00}, {0x36, 0x3C, 0x08},
	{0x36, 0x3D, 0x30}, {0x36, 0x3E, 0x10}, {0x36, 0x3F, 0x08}, {0x36, 0x40, 0x32},
	{0x36, 0x41, 0x00}, {0x36, 0x42, 0x01}, {0x36, 0x43, 0x01}, {0x36, 0x44, 0x00},
	{0x36, 0x45, 0x00}, {0x36, 0x46, 0x00}, {0x36, 0x47, 0x18}, {0x36, 0x48, 0x00},
	{0x36, 0x49, 0x00}, {0x36, 0x4A, 0x00}, {0x36, 0x4B, 0x10}, {0x36, 0x4C, 0x00},
	{0x36, 0x4D, 0x00}, {0x36, 0x4E, 0x00}, {0x36, 0x4F, 0x00}, {0x36, 0x50, 0x00},
	{0x36, 0x51, 0x00}, {0x36, 0x52, 0x00}, {0x36, 0x53, 0x00}, {0x36, 0x54, 0x00},
	{0x36, 0x55, 0x00}, {0x36, 0x56, 0x00}, {0x36, 0x57, 0x00}, {0x36, 0x58, 0x00},
	{0x36, 0x59, 0x00}, {0x36, 0x5A, 0x00}, {0x36, 0x5B, 0x00}, {0x36, 0x5C, 0x00},
	{0x36, 0x5D, 0xFF}, {0x36, 0x5E, 0x00}, {0x36, 0x5F, 0x31}, {0x36, 0x60, 0x32},
	{0x36, 0x61, 0x33}, {0x36, 0x62, 0x34}, {0x36, 0x63, 0x35}, {0x36, 0x64, 0x36},
	{0x36, 0x65, 0x37}, {0x36, 0x66, 0x38}, {0x36, 0x67, 0x39}, {0x36, 0x68, 0x30},
	{0x36, 0x69, 0x0A}, {0x36, 0x6A, 0x20}, {0x36, 0x6B, 0x20}, {0x36, 0x6C, 0x00},
	{0x36, 0x6D, 0x00}, {0x36, 0x6E, 0x00}, {0x36, 0x6F, 0xFC}, {0x36, 0x70, 0x00},
	{0x36, 0x71, 0x47}, {0x36, 0x72, 0x4D}, {0x36, 0x73, 0x20}, {0x36, 0x74, 0x57},
	{0x36, 0x75, 0x58}, {0x36, 0x76, 0x47}, {0x36, 0x77, 0x41}, {0x36, 0x78, 0x20},
	{0x36, 0x79, 0x38}, {0x36, 0x7A, 0x22}, {0x36, 0x7B, 0x0A}, {0x36, 0x7C, 0x20},
	{0x36, 0x7D, 0x20}, {0x36, 0x7E, 0x00}, {0x36, 0x7F, 0x53}, {0x36, 0x80, 0xFF},
	{0x36, 0x81, 0xFF}, {0x36, 0x82, 0xFF}, {0x36, 0x83, 0xFF}, {0x36, 0x84, 0xFF},
	{0x36, 0x85, 0xFF}, {0x36, 0x86, 0xFF}, {0x36, 0x87, 0xFF}, {0x36, 0x88, 0xFF},
	{0x36, 0x89, 0xFF}, {0x36, 0x8A, 0xFF}, {0x36, 0x8B, 0xFF}, {0x36, 0x8C, 0xFF},
	{0x36, 0x8D, 0xFF}, {0x36, 0x8E, 0xFF}, {0x36, 0x8F, 0xFF}, {0x36, 0x90, 0xFF},
	{0x36, 0x91, 0xFF}, {0x36, 0x92, 0xFF}, {0x36, 0x93, 0xFF}, {0x36, 0x94, 0xFF},
	{0x36, 0x95, 0xFF}, {0x36, 0x96, 0xFF}, {0x36, 0x97, 0xFF}, {0x36, 0x98, 0xFF},
	{0x36, 0x99, 0xFF}, {0x36, 0x9A, 0xFF}, {0x36, 0x9B, 0xFF}, {0x36, 0x9C, 0xFF},
	{0x36, 0x9D, 0xFF}, {0x36, 0x9E, 0xFF}, {0x36, 0x9F, 0xFF}, {0x36, 0xA0, 0xFF},
	{0x36, 0xA1, 0xFF}, {0x36, 0xA2, 0xFF}, {0x36, 0xA3, 0xFF}, {0x36, 0xA4, 0xFF},
	{0x36, 0xA5, 0xFF}, {0x36, 0xA6, 0xFF}, {0x36, 0xA7, 0xFF}, {0x36, 0xA8, 0xFF},
	{0x36, 0xA9, 0xFF}, {0x36, 0xAA, 0xFF}, {0x36, 0xAB, 0xFF}, {0x36, 0xAC, 0xFF},
	{0x36, 0xAD, 0xFF}, {0x36, 0xAE, 0xFF}, {0x36, 0xAF, 0xFF}, {0x36, 0xB0, 0xFF},
	{0x36, 0xB1, 0xFF}, {0x36, 0xB2, 0xFF}, {0x36, 0xB3, 0xFF}, {0x36, 0xB4, 0xFF},
	{0x36, 0xB5, 0xFF}, {0x36, 0xB6, 0xFF}, {0x36, 0xB7, 0xFF}, {0x36, 0xB8, 0xFF},
	{0x36, 0xB9, 0xFF}, {0x36, 0xBA, 0xFF}, {0x36, 0xBB, 0xFF}, {0x36, 0xBC, 0xFF},
	{0x36, 0xBD, 0xFF}, {0x36, 0xBE, 0xFF}, {0x36, 0xBF, 0xFF}, {0x36, 0xC0, 0xFF},
	{0x36, 0xC1, 0xFF}, {0x36, 0xC2, 0xFF}, {0x36, 0xC3, 0xFF}, {0x36, 0xC4, 0xFF},
	{0x36, 0xC5, 0xFF}, {0x36, 0xC6, 0xFF}, {0x36, 0xC7, 0xFF}, {0x36, 0xC8, 0xFF},
	{0x36, 0xC9, 0xFF}, {0x36, 0xCA, 0xFF}, {0x36, 0xCB, 0xFF}, {0x36, 0xCC, 0xFF},
	{0x36, 0xCD, 0xFF}, {0x36, 0xCE, 0xFF}, {0x36, 0xCF, 0xFF}, {0x36, 0xD0, 0xFF},
	{0x36, 0xD1, 0xFF}, {0x36, 0xD2, 0xFF}, {0x36, 0xD3, 0xFF}, {0x36, 0xD4, 0xFF},
	{0x36, 0xD5, 0xFF}, {0x36, 0xD6, 0xFF}, {0x36, 0xD7, 0xFF}, {0x36, 0xD8, 0xFF},
	{0x36, 0xD9, 0xFF}, {0x36, 0xDA, 0xFF}, {0x36, 0xDB, 0xFF}, {0x36, 0xDC, 0xFF},
	{0x36, 0xDD, 0xFF}, {0x36, 0xDE, 0xFF}, {0x36, 0xDF, 0xFF}, {0x36, 0xE0, 0xFF},
	{0x36, 0xE1, 0xFF}, {0x36, 0xE2, 0xFF}, {0x36, 0xE3, 0xFF}, {0x36, 0xE4, 0xFF},
	{0x36, 0xE5, 0xFF}, {0x36, 0xE6, 0xFF}, {0x36, 0xE7, 0xFF}, {0x36, 0xE8, 0xFF},
	{0x36, 0xE9, 0xFF}, {0x36, 0xEA, 0xFF}, {0x36, 0xEB, 0xFF}, {0x36, 0xEC, 0xFF},
	{0x36, 0xED, 0xFF}, {0x36, 0xEE, 0xFF}, {0x36, 0xEF, 0xFF}, {0x36, 0xF0, 0xFF},
	{0x36, 0xF1, 0xFF}, {0x36, 0xF2, 0xFF}, {0x36, 0xF3, 0xFF}, {0x36, 0xF4, 0xFF},
	{0x36, 0xF5, 0xFF}, {0x36, 0xF6, 0xFF}, {0x36, 0xF7, 0xFF}, {0x36, 0xF8, 0xFF},
	{0x36, 0xF9, 0xFF}, {0x36, 0xFA, 0xFF}, {0x36, 0xFB, 0xFF}, {0x36, 0xFC, 0xFF},
	{0x36, 0xFD, 0xFF}, {0x36, 0xFE, 0xFF}, {0x36, 0xFF, 0xFF}, {0x32, 0x74, 0x01},
	{0x4c, 0xF4, 0x80}, {0x4c, 0xF5, 0x7C}, {0x4c, 0xF8, 0x4C}, {0x4c, 0xFB, 0x68},
	{0x4c, 0xFD, 0x44}, {0x4c, 0xE9, 0xC0}, {0x4c, 0x00, 0x10}, {0x4c, 0x01, 0x06},
	{0x4c, 0x02, 0xF2}, {0x4c, 0x03, 0x42}, {0x4c, 0x05, 0x28}, {0x4c, 0x0C, 0x42},
	{0x4c, 0x15, 0xAE}, {0x60, 0x40, 0x0A}, {0x22, 0x6C, 0x00}, {0x22, 0xC9, 0x2D},
//Commented to disable no signal blue screen. value 0x14 written in 0xBF
//	{0x22, 0xBF, 0x16}, {0x22, 0xC0, 0x40}, {0x22, 0xC1, 0x40}, {0x22, 0xC2, 0xC0},
	{0x22, 0xBF, 0x14}, {0x22, 0xC0, 0x40}, {0x22, 0xC1, 0x40}, {0x22, 0xC2, 0xC0},
	{0x22, 0x8F, 0x42}, {0x22, 0x90, 0x5C}, {0x22, 0xA5, 0x31}, {0x22, 0xA6, 0x60},
	{0x22, 0xA7, 0x16}, {0x22, 0xAB, 0x31}, {0x22, 0xAC, 0x60}, {0x22, 0x91, 0x00},
	{0x32, 0x40, 0x81}, {0x34, 0x03, 0x98}, {0x34, 0x10, 0xA5}, {0x34, 0x1B, 0x08},
	{0x34, 0x45, 0x04}, {0x34, 0x97, 0xC0}, {0x34, 0x3D, 0x10}, {0x34, 0x3E, 0x7B},
	{0x34, 0x3F, 0x5E}, {0x34, 0x4E, 0xFE}, {0x34, 0x4F, 0x08}, {0x34, 0x57, 0xA3},
	{0x34, 0x58, 0x07}, {0x34, 0x6F, 0x08}, {0x34, 0x83, 0xFE}, {0x34, 0x85, 0x10},
	{0x34, 0x86, 0x9B}, {0x34, 0x89, 0x01}, {0x34, 0x9B, 0x03}, {0x34, 0x9C, 0x80},
	{0x34, 0x9C, 0xC0}, {0x34, 0x9C, 0x00}, {0x4c, 0x20, 0x80}, {0x34, 0x6C, 0xA3},
	{0x34, 0x48, 0x40}, {0x60, 0x43, 0x03}, {0x60, 0x45, 0x04}, {0x60, 0x46, 0x53},
	{0x60, 0x47, 0x03}, {0x60, 0x4C, 0x19}, {0x60, 0x4E, 0x24}, {0x60, 0x40, 0x08},
	};

	union i2c_smbus_data data;
	int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(adv7613_init_values); i++) {
		data.byte = adv7613_init_values[i][2];
		ret = i2c_smbus_xfer(client->adapter, adv7613_init_values[i][0], 0,
						I2C_SMBUS_WRITE, adv7613_init_values[i][1],
						I2C_SMBUS_BYTE_DATA, &data);
		if (ret != 0) {
			pr_err("%s Error in serdes initialisation: slave addr %x\n",
						__func__, adv7613_init_values[i][0]);
			return -1;
		}
	}
	return 0;
}

static void reset_adv(int gpio)
{
/*
 * Hold the RESET line low for at least 5 ms after the supplies have powered up
 * Allow a minimum additional 5 ms before carrying out the first I2C transaction
 */
	gpio_set_value(gpio, 0);
	mdelay(ADV7613_RESET_DELAY_MS);
	gpio_set_value(gpio, 1);
	mdelay(ADV7613_RESET_DELAY_MS);
}

static int adv7613_enable(struct i2c_client *client)
{
	int err;

	err = adv7613_enable_seq(client);
	if (err) {
		pr_err("%s:%d adv7613_enable_seq failed, err=%d, we will retry again\n", __func__, __LINE__, err);
		reset_adv(SERDES_PWD);
		err = adv7613_enable_seq(client);
		if (err) {
			pr_err("%s:%d retry after reset, adv7613_enable_seq failed, err=%d\n", __func__, __LINE__, err);
		}
	}

	return err;
}
static int adv7613_serdes_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err;

	printk(KERN_ERR "#### ADV7613 probe start\n");
	pr_info("#### ADV7613 probe start\n");

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s SMBUS Byte Data not Supported\n", __func__);
		return -EIO;
	}

	err = gpio_request(SERDES_PWD, "SERDES_PWD");
	if (err) {
		pr_err("%s: Failed to request GPIO err = %d\n", __func__, err);
		return err;
	}
	gpio_direction_output(SERDES_PWD, 1);
	reset_adv(SERDES_PWD);
	err = adv7613_enable(client);
	if (err) {
		pr_err("%s:%d Failed to initialize the driver\n", __func__, __LINE__);
		return -EIO;
	}
	pr_info("#### ADV7613 serdes initialization is done successfuly: %s\n", __func__);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int adv7613_serdes_suspend(struct device *dev)
{
	union i2c_smbus_data data;
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	pr_info("#### ADV suspend start\n");
	return 0;
}

static int adv7613_serdes_resume(struct device *dev)
{
	int err=-1;
	struct i2c_client *client = to_i2c_client(dev);

	pr_info("#### %s\n", __func__);
	/*
	 * S2R resume adv reset not required as ADV goes to powerdown during suspend and
	 * ADV power up during resume as part of gpio-dv1-s2r resume
	 */
	err = adv7613_enable(client);
	if (err) {
		pr_err("%s:%d Failed to initialize the driver\n", __func__, __LINE__);
		return -EIO;
	}

	return 0;
}


static SIMPLE_DEV_PM_OPS(adv7613_pm_ops, adv7613_serdes_suspend, adv7613_serdes_resume);
#endif

static int adv7613_serdes_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id adv7613_serdes_id[] = {
	{ "adv7613_serdes", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, adv7613_serdes_id);

static struct i2c_driver adv7613_serdes_driver = {
	.driver = {
		.name  = "adv7613_serdes",
		.owner = THIS_MODULE,
		.pm	   = &adv7613_pm_ops,
	},
	.probe    = adv7613_serdes_probe,
	.remove   = adv7613_serdes_remove,
	.id_table = adv7613_serdes_id,
};

static int __init fcrserdes_init(void)
{
	return i2c_add_driver(&adv7613_serdes_driver);
}
late_initcall(fcrserdes_init);

static void __exit fcrserdes_cleanup(void)
{
	i2c_del_driver(&adv7613_serdes_driver);
}
module_exit(fcrserdes_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ADV7613 FPDLink Serializer driver");
MODULE_ALIAS("i2c:adv7613_serdes");