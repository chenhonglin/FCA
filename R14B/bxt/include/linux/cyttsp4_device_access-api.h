/*
 * cyttsp4_device_access-api.h
 * Cypress TrueTouch(TM) Standard Product V4 Device Access API module.
 * For use with Cypress touchscreen controllers.
 * Supported parts include:
 * GEN6 XL
 * GEN6 L
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

#ifndef _LINUX_CYTTSP4_DEVICE_ACCESS_API_H
#define _LINUX_CYTTSP4_DEVICE_ACCESS_API_H

#include <linux/types.h>
#include <linux/device.h>

#define GRPNUM_OP_COMMAND	1
#define GRPNUM_TOUCH_CONFIG	6

#define OP_CMD_NULL		0
#define OP_CMD_GET_PARAMETER	2
#define OP_CMD_SET_PARAMETER	3
#define OP_CMD_GET_CONFIG_CRC	5

#define OP_PARAM_ACTIVE_DISTANCE		0x4A
#define OP_PARAM_SCAN_TYPE			0x4B
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_TEST_DEV_ACCESS_TTSP20_MODE
#define OP_PARAM_LOW_POWER_INTERVAL		0x1C
#define OP_PARAM_REFRESH_INTERVAL		0x1D
#define OP_PARAM_ACTIVE_MODE_TIMEOUT		0x1E
#define OP_PARAM_WAKEUP_SOURCE			0x1F
#else
#define OP_PARAM_LOW_POWER_INTERVAL		0x4C
#define OP_PARAM_REFRESH_INTERVAL		0x4D
#define OP_PARAM_ACTIVE_MODE_TIMEOUT		0x4E
#endif
#define OP_PARAM_ACTIVE_LOOK_FOR_TOUCH_INTERVAL 0x4F

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_TEST_DEV_ACCESS_TTSP20_MODE
#define OP_PARAM_MIN			OP_PARAM_LOW_POWER_INTERVAL
#define OP_PARAM_MAX			OP_PARAM_WAKEUP_SOURCE
#else
#define OP_PARAM_MIN			OP_PARAM_ACTIVE_DISTANCE
#define OP_PARAM_MAX			OP_PARAM_ACTIVE_LOOK_FOR_TOUCH_INTERVAL
#endif

int cyttsp4_device_access_read_command(const char *core_name,
				       int ic_grpnum, int ic_grpoffset,
				       u8 *buf, int buf_size);

int cyttsp4_device_access_write_command(const char *core_name,
					int ic_grpnum, int ic_grpoffset,
					u8 *buf, int length);

int cyttsp4_device_access_read_command_async(const char *core_name,
					     int ic_grpnum, int ic_grpoffset,
					     u8 *buf, int length,
					     void (*cont)(const char
					     *core_name, int ic_grpnum,
					     int ic_grpoffset, u8 *buf,
					     int length, int rc));

int cyttsp4_device_access_write_command_async(const char *core_name,
					      int ic_grpnum, int ic_grpoffset,
					      u8 *buf, int length,
					      void (*cont)(const char
					      *core_name, int ic_grpnum,
					      int ic_grpoffset, u8 *buf,
					      int length, int rc));
#endif /* _LINUX_CYTTSP4_DEVICE_ACCESS_API_H */