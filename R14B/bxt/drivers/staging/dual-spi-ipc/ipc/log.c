/*****************************************************************************
 * Project              Harman Car Multimedia System
 *
 * c) copyright         2019
 * Company              Harman International Industries, Incorporated
 *                      All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file                log.c
 * @author              David Rogala
 *
 * This module contains helper logic and global variables used to configure
 * logging levels
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/types.h>
#include <linux/string.h>

#include "log.h"

/* global logging level */
int gIpcLogLevel = LOG_DEFAULT_LVL;
/* logging enabled of raw frame transfer - output*/
int gHalTxLogEnabled = 0;
/* logging enabled of raw frame transfer - input*/
int gHalRxLogEnabled = 0;

/**
 * @func 	printData
 * @desc		converts a byte array to a hex string
 * @param	(out) str: output string
 * @param	(in) slen: length of output string
 * @param	(in) data: pointer to data array
 * @param   (in) dlen: length of input data array
 * @param   (in) max:  maximum number of bytes to print
*/
void printData(char* str, int32_t slen,const uint8_t *data, int32_t dlen, int32_t max)
{
	int32_t i = 0;
	int32_t ret;

	if (slen >= 1) {
		str[0] = '\0';

		while (slen > 0 && i < max && i < dlen) {
			ret = snprintf(str, slen, "%02X ", data[i++]);

			if (ret > 0) {
				str += ret;
				slen -= ret;
			} else {
				break;
			}
		}

		if (max < dlen) {
			snprintf(str, slen, "...");
		}
	}
}
