/*
 * Include of ST Microcontroller Stm8af5288 driver
 *
 * Copyright (C) 2018 Harman International Ltd.
 * Xingfeng Ye <Xingfeng.Ye@harman.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_STM8AF5X_H
#define __LINUX_MFD_STM8AF5X_H

#include <linux/mutex.h>

/*read command list from radio*/
#define read_command_id1    0x01  /*read display mcu sw firmware version*/
#define read_command_id2    0x02  /*read display mcu internal status*/
#define read_command_id3    0x03  /*read display mcu brightness value*/

/*write command list from radio*/
#define write_command_id1   0x04  /*write display brightness value*/



struct stm8af5x_chip {
	struct device		*dev;
	struct i2c_client   *client;
	struct mutex		io_lock;
};

extern int read_stm8af5x_display_btness(struct i2c_client *client, int reg, int count);
extern int write_stm8af5x_display_btness(struct i2c_client *client, int reg, unsigned char data);

#endif /* __LINUX_MFD_STM8AF5X_H */

