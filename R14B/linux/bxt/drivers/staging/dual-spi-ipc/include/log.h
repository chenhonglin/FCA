/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2019
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	log.h
 * @author        	David Rogala
 *
 * This header file contains the definition for MACROS used for logging
 */

#ifndef __LOG_H
#define __LOG_H

#include <linux/kernel.h>

#define LOG_NONE_LVL  	(0)
#define LOG_ERROR_LVL 	(3)
#define LOG_WARNING_LVL (4)
#define LOG_NOTICE_LVL 	(5)
#define LOG_INFO_LVL 	(6)
#define LOG_DEBUG_LVL 	(7)

#define LOG_DEFAULT_LVL (LOG_WARNING_LVL)

/* logging */
#define log_debug(fmt, arg...)                                          \
	if (gIpcLogLevel >= LOG_DEBUG_LVL) {\
		pr_debug("[ IPC:%-20.20s: %-1.1d ] [DBG] " fmt "\n", \
		__func__, __LINE__, ##arg); \
	}
#define log_error(fmt, arg...)                                          \
	if (gIpcLogLevel >= LOG_ERROR_LVL) {\
		pr_err("[ IPC:%-20.20s: %-1.1d ] [ERR] " fmt "\n",   \
		__func__, __LINE__, ##arg);\
	}
#define log_info(fmt, arg...)                                          \
	if (gIpcLogLevel >= LOG_INFO_LVL) {\
		pr_info("[ IPC:%-20.20s: %-1.1d ] [INF] " fmt "\n",  \
		__func__, __LINE__, ##arg);\
	}
#define log_notice(fmt, arg...)                                          \
	if (gIpcLogLevel >= LOG_NOTICE_LVL) {\
		pr_notice("[ IPC:%-20.20s: %-1.1d ] [INF] " fmt "\n",  \
		__func__, __LINE__, ##arg);\
	}
#define log_warning(fmt, arg...)                                          \
	if (gIpcLogLevel >= LOG_WARNING_LVL) {\
		pr_warn("[ IPC:%-20.20s: %-1.1d ] [INF] " fmt "\n",  \
		__func__, __LINE__, ##arg);\
	}

extern int gIpcLogLevel;
void printData(char* str, int32_t slen, const uint8_t *data, int32_t dlen,
		int32_t max);

#endif /* __LOG_H */
