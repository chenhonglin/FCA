/*
 * cyttsp4_i2c.c
 * Cypress TrueTouch(TM) Standard Product V4 I2C Module.
 * For use with Cypress touchscreen controllers.
 * Supported part families include:
 * GEN6 XL
 * GEN L
 *
 * Copyright (C) 2012-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include "cyttsp4_regs.h"
#include <linux/cyttsp4_platform.h>
#include <linux/i2c.h>
#include <linux/version.h>

#define CY_I2C_DATA_SIZE  (2 * 256)

/*
 * bl_i2c_client_addr comes with SOLO support. if it is 0, then it means
 * application address will be used. otherwise, bootloader address will be used.
 */
static int cyttsp4_i2c_read_block_data(struct device *dev, u16 addr,
				       int length, void *values, int max_xfer,
				       u16 bl_i2c_client_addr)
{
	struct i2c_client *client = to_i2c_client(dev);
	int trans_len;
	u16 slave_addr = client->addr;
	u8 client_addr;
	u8 addr_lo;
	struct i2c_msg msgs[2];
	int rc = -EINVAL;
	int msg_cnt = 0;

	if (bl_i2c_client_addr)
		slave_addr = bl_i2c_client_addr;

	while (length > 0) {
		client_addr = slave_addr | ((addr >> 8) & 0x1);
		addr_lo = addr & 0xFF;
		trans_len = min(length, max_xfer);

		msg_cnt = 0;
		memset(msgs, 0, sizeof(msgs));
		/* There is no address information in
		 * bootloader mode of SOLO.
		 */
		if (!bl_i2c_client_addr) {
			msgs[msg_cnt].addr = client_addr;
			msgs[msg_cnt].flags = 0;
			msgs[msg_cnt].len = 1;
			msgs[msg_cnt].buf = &addr_lo;
			msg_cnt++;
		}

		rc = i2c_transfer(client->adapter, msgs, msg_cnt);
		if (rc != msg_cnt)
			goto exit;

		msg_cnt = 0;
		msgs[msg_cnt].addr = client_addr;
		msgs[msg_cnt].flags = I2C_M_RD;
		msgs[msg_cnt].len = trans_len;
		msgs[msg_cnt].buf = values;
		msg_cnt++;

		rc = i2c_transfer(client->adapter, msgs, msg_cnt);
		if (rc != msg_cnt)
			goto exit;

		length -= trans_len;
		values += trans_len;
		addr += trans_len;
	}

exit:
	return (rc < 0) ? rc : rc != msg_cnt ? -EIO : 0;
}

/*
 * bl_i2c_client_addr comes with SOLO support. if it is 0, then it means
 * application address will be used. otherwise, bootloader address will be used.
 */
static int cyttsp4_i2c_write_block_data(struct device *dev, u16 addr,
					u8 *wr_buf, int length,
					const void *values,
	int max_xfer, u16 bl_i2c_client_addr)
{
	struct i2c_client *client = to_i2c_client(dev);
	u16 slave_addr = client->addr;
	u8 client_addr;
	u8 addr_lo;
	int trans_len;
	struct i2c_msg msg;
	int rc = -EINVAL;

	if (bl_i2c_client_addr)
		slave_addr = bl_i2c_client_addr;

	while (length > 0) {
		client_addr = slave_addr | ((addr >> 8) & 0x1);
		addr_lo = addr & 0xFF;
		trans_len = min(length, max_xfer);

		memset(&msg, 0, sizeof(msg));
		msg.addr = client_addr;
		msg.flags = 0;

		if (bl_i2c_client_addr) {
			msg.len = trans_len;
			msg.buf = &wr_buf[1];
		} else {
			msg.len = trans_len + 1;
			msg.buf = wr_buf;
		}

		wr_buf[0] = addr_lo;
		memcpy(&wr_buf[1], values, trans_len);

		/* write data */
		rc = i2c_transfer(client->adapter, &msg, 1);
		if (rc != 1)
			goto exit;

		length -= trans_len;
		values += trans_len;
		addr += trans_len;
	}

exit:
	return (rc < 0) ? rc : rc != 1 ? -EIO : 0;
}

static int cyttsp4_i2c_write(struct device *dev, u16 addr, u8 *wr_buf,
			     const void *buf, int size, int max_xfer,
			     u16 bl_i2c_client_addr)
{
	int rc;

	pm_runtime_get_noresume(dev);
	rc = cyttsp4_i2c_write_block_data(dev, addr, wr_buf, size, buf,
					  max_xfer, bl_i2c_client_addr);
	pm_runtime_put_noidle(dev);

	return rc;
}

static int cyttsp4_i2c_read(struct device *dev, u16 addr, void *buf, int size,
			    int max_xfer, u16 bl_i2c_client_addr)
{
	int rc;

	pm_runtime_get_noresume(dev);
	rc = cyttsp4_i2c_read_block_data(dev, addr, size, buf, max_xfer,
					 bl_i2c_client_addr);
	pm_runtime_put_noidle(dev);

	return rc;
}



static int i2c_read(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *val)
{
	int ret = 0;
	u8 *data = NULL;
	struct i2c_msg msg[2];
	unsigned int size = reg_len + 1u;

	if ((reg == NULL) || (val == NULL) || (reg_len <= 0u)) {
		printk("reg/val/reg_len is %02x/%02x/%d\n",
			*reg, *val, reg_len);
		return -EINVAL;
	}

	data = kzalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOSPC;

	(void)memcpy(data, reg, reg_len);
	(void)memset(msg, 0, sizeof(msg));

	msg[0].addr = (slave_addr);
	msg[0].flags = 0;
	msg[0].len = (__u16)reg_len;
	msg[0].buf = data;

	msg[1].addr = (slave_addr);
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + reg_len;

	//client->addr = (slave_addr);

	ret = i2c_transfer(client->adapter, msg, 2);

	*val = *(data + reg_len);
	kfree(data);
	data = NULL;
	printk("read dev/reg/val/ret is %02x/%02x/%02x/%d\n",
		slave_addr, *reg, *val, ret);

	return ret;
}

static int i2c_write(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *val)
{
	u8 *data = NULL;
	int ret = 0;
	struct i2c_msg msg;
	unsigned int size = reg_len + 1u;

	if ((reg == NULL) || (val == NULL) || (reg_len <= 0u)) {
		printk("reg/val/reg_len is %02x/%02x/%d\n",
			*reg, *val, reg_len);
		return -EINVAL;
	}

	data = kzalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOSPC;

	(void)memcpy(data, reg, reg_len);
	*(data + reg_len) = *val;

	(void)memset(&msg, 0, sizeof(msg));
	msg.addr = (slave_addr);
	msg.flags = 0;
	msg.len = (u16)size;
	msg.buf = data;

	//client->addr = (slave_addr);
	ret = i2c_transfer(client->adapter, &msg, 1);

	kfree(data);
	data = NULL;
	printk("write dev/reg/val/ret is %02x/%02x/%02x/%d\n",
		slave_addr, *reg, *val, ret);
	return ret;
}

static int cyttsp4_serilizer_init(struct i2c_client *i2c)
{
	int retval, max_xfer;
	u8 serilizer_reg,reg_val;
	u16 serializer_i2c_addr, deserializer_i2c_addr;

	struct cyttsp4_core_data *cd = i2c_get_clientdata(i2c);

	serializer_i2c_addr = cd->cpdata->serializer_i2c_addr;
	deserializer_i2c_addr = cd->cpdata->deserializer_i2c_addr;
	max_xfer = cd->max_xfer;

	//read serilizer device id
	serilizer_reg = 0x00;
	retval = i2c_read(i2c, serializer_i2c_addr, &serilizer_reg, 1, &reg_val);
	printk("ti947 device id:0x%x\n",reg_val);

	//Set I2C pass-through mode enable.
	serilizer_reg = 0x03;
	reg_val = 0xDA;
	retval = i2c_write(i2c, serializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	//Set Slave ID.
	serilizer_reg = 0x07;
	reg_val = 0x40;
	retval = i2c_write(i2c, serializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	//Set Slave Alias.
	serilizer_reg = 0x08;
	reg_val = 0x40;
	retval = i2c_write(i2c, serializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	//set dual pixel mode
	serilizer_reg = 0x4F;
	reg_val = 0x00;
	retval = i2c_write(i2c, serializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	//Pass through remote INTB_IN to INTB
	serilizer_reg = 0xC6;
	reg_val = 0x21;
	retval = i2c_write(i2c, serializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	/* Enable the communication over I2C Control (to deserializer and touch controller) */
	serilizer_reg = 0x17;
	reg_val = 0x9e;
	retval = i2c_write(i2c, serializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	/*Read from HDCP_ISR register to arm the interrupt for the first time*/
	serilizer_reg = 0xc7;
	retval = i2c_read(i2c, serializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	//Set 947 gpio1/2 remote hold mode
	serilizer_reg = 0x0E;
	reg_val = 0x55;
	retval = i2c_write(i2c, serializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	//read 948 de-serilizer device id
	serilizer_reg = 0x00;
	retval = i2c_read(i2c, deserializer_i2c_addr, &serilizer_reg, 1, &reg_val);
	printk(KERN_WARNING"ti948 device id:0x%x, readbackid:0x%x\n", deserializer_i2c_addr,reg_val);

	//Set 948 gpio1/2 input
	serilizer_reg = 0x1E;
	reg_val = 0x33;
	retval = i2c_write(i2c, deserializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	 /*Set 948, Change the color packing order - Fixes psychodelic colors on 1080p display (set this 949 is work --hubert,)*/
	/*serilizer_reg = 0x49;
	reg_val = 0x60;
	retval = cyttsp4_i2c_write(i2c, hw_if.board_data->deserializer_i2c_addr, &serilizer_reg, 1, &reg_val);*/

	/* Set 948 I2C speed from 75KHZ to 400KHZ --Ian */
	serilizer_reg = 0x26;
	reg_val = 0x19;
	retval = i2c_write(i2c, deserializer_i2c_addr, &serilizer_reg, 1, &reg_val);

	serilizer_reg = 0x27;
	reg_val = 0x19;
	retval = i2c_write(i2c, deserializer_i2c_addr, &serilizer_reg, 1, &reg_val);


	return retval;
}

static struct cyttsp4_bus_ops cyttsp4_i2c_bus_ops = {
	.write = cyttsp4_i2c_write,
	.read = cyttsp4_i2c_read,
};

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT
static struct of_device_id cyttsp4_i2c_of_match[] = {
	{ .compatible = "cy,cyttsp6_i2c_adapter", },
	{ .compatible = "cy,cyttsp6_i2c_adapter2", }, { }
};
MODULE_DEVICE_TABLE(of, cyttsp4_i2c_of_match);
#endif

static int cyttsp4_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *i2c_id)
{
	struct device *dev = &client->dev;
	enum cy_dev_id dev_id = i2c_id->driver_data;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT
	const struct of_device_id *match;
#endif
	int rc;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C functionality not Supported\n");
		return -EIO;
	}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(cyttsp4_i2c_of_match), dev);
	if (match) {
		rc = cyttsp4_devtree_create_and_get_pdata(dev);
		if (rc < 0)
			return rc;
		rc = cyttsp4_get_loader_platform_data(dev, dev_id);
		if (rc < 0)
			return rc;
		rc = cyttsp4_get_tch_param_size(dev, dev_id);
		if (rc < 0)
			return rc;
	}
#endif
	rc = cyttsp4_probe(&cyttsp4_i2c_bus_ops, &client->dev, client->irq,
			   dev_id);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT
	if (rc && match)
		cyttsp4_devtree_clean_pdata(dev);
#endif

	return rc;
}

static int cyttsp4_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT
	struct device *dev = &client->dev;
	const struct of_device_id *match;
#endif
	struct cyttsp4_core_data *cd = i2c_get_clientdata(client);

	cyttsp4_release(cd);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(cyttsp4_i2c_of_match), dev);
	if (match)
		cyttsp4_devtree_clean_pdata(dev);
#endif

	return 0;
}

static const struct i2c_device_id cyttsp4_i2c_id[] = {
	{ CYTTSP6_I2C_NAME, CY_DEVID_GEN6 },
	{ CYTTSP6_I2C_NAME2, CY_DEVID_GEN6_2 }, {}
};
MODULE_DEVICE_TABLE(i2c, cyttsp4_i2c_id);

static struct i2c_driver cyttsp4_i2c_driver = {
	.driver = {
		.name = CYTTSP4_I2C_NAME,
		.owner = THIS_MODULE,
		//.pm = &cyttsp4_pm_ops,
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT
		.of_match_table = cyttsp4_i2c_of_match,
#endif
	},
	.probe = cyttsp4_i2c_probe,
	.remove = cyttsp4_i2c_remove,
	.id_table = cyttsp4_i2c_id,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
module_i2c_driver(cyttsp4_i2c_driver);
#else
static int __init cyttsp4_i2c_init(void)
{
	int rc = i2c_add_driver(&cyttsp4_i2c_driver);

	pr_info("%s: Cypress TTSP I2C Driver (Built %s) rc=%d\n",
			__func__, CY_DRIVER_VERSION, rc);

	return rc;
}
module_init(cyttsp4_i2c_init);

static void __exit cyttsp4_i2c_exit(void)
{
	i2c_del_driver(&cyttsp4_i2c_driver);
}
module_exit(cyttsp4_i2c_exit);
#endif


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product I2C driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
