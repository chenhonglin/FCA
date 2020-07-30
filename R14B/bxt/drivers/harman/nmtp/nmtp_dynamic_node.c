/*
 *  nmtp_dynamic_node.c
 *  Copyright (C) 2019 Harman International Ltd,
 *  Author: Akshay Shankarmurthy <Akshay.Shankarmurthy@harman.com>
 *  Created on: 01-Mar-2019
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2.0 as published by
 * the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/moduleparam.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/spidev.h>

unsigned short mode;
module_param(mode, ushort, 0000);
MODULE_PARM_DESC(mode, "mode=0 for boot_mode & mode=1 for control_mode");

static struct pxa2xx_spi_chip chip_data_1 = {
	.gpio_cs = -EINVAL,
	.dma_burst_size = 1,
//        .pio_dma_threshold = 8,
};
struct spi_board_info nmtp_boot_spidev[] = {
	{
		.max_speed_hz = 1500000, /* 1.5MHz */
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.controller_data = &chip_data_1,
		.modalias = "spidev",
	},
	{
		.max_speed_hz = 1500000, /* 1.5MHz */
		.bus_num = 1,
		.chip_select = 1,
		.mode = SPI_MODE_1,
		.controller_data = &chip_data_1,
		.modalias = "spidev",
	}

};

struct spi_board_info nmtp_control_spidev[] = {
	{
		.max_speed_hz = 10000000, /* 10MHz */
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.modalias = "nmtp_spi",
		.controller_data = &chip_data_1,
//		.platform_data = &nmtp_master_mode,
	},
	{
		.max_speed_hz = 10000000, /* 10MHz */
		.bus_num = 1,
		.chip_select = 1,
		.mode = SPI_MODE_1,
		.modalias = "nmtp_spi",
		.controller_data = &chip_data_1,
//		.platform_data = &nmtp_slave_mode,
	},
};

static struct spi_device *spi_device[2];

static int __init nmtp_dynamic_init(void)
{
	int ret, i;
	struct spi_master *master;
	struct spi_board_info *board_info;

	if (mode == 0)
		board_info = nmtp_boot_spidev;
	else
		board_info = nmtp_control_spidev;

	for (i = 0; i < 2; i++) {
		master = spi_busnum_to_master(board_info[i].bus_num);
		if (!master) {
			pr_err("%s %d\n", __func__, __LINE__);
			return -ENODEV;
		}
		spi_device[i] = spi_new_device(master, &board_info[i]);
		if (!spi_device[i])
			return -ENODEV;
		ret = spi_setup(spi_device[i]);
		if (ret)
			spi_unregister_device(spi_device[i]);
	}

	return ret;
}
module_init(nmtp_dynamic_init);

static void __exit nmtp_dynamic_exit(void)
{
	int i;

	pr_debug("Exit nmtp dynamic driver mode = %d\n", mode);
	for (i = 0; i < 2; i++)
		spi_unregister_device(spi_device[i]);
}
module_exit(nmtp_dynamic_exit);

MODULE_AUTHOR("Akshay Shankarmurthy <Akshya.Shankarmurthy@harman.com>");
MODULE_DESCRIPTION("Driver for NMTP chip with SPI interface");
MODULE_LICENSE("GPL");
