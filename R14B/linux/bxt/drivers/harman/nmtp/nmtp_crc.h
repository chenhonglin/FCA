/*
 *  nmtp_crc.h
 *  Copyright (C) 2019 Harman International Ltd,
 *  Author: Akshay Shankarmurthy <Akshay.Shankarmurthy@harman.com>
 *  Created on: 28-May-2019
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

#ifndef NMTP_CRC_H
#define NMTP_CRC_H

#include <linux/types.h>

void crc_fill(u8 *data);
int crc_check(u8 *data);


#endif
