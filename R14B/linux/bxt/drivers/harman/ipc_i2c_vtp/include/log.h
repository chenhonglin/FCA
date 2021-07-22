/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	log.h
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * This header file contains the definition for MACROS used for logging
 */

#ifndef LOG_H
#define LOG_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include <linux/kernel.h>


/*****************************************************************************
 * DEFINES
 *****************************************************************************/

#define IPC_STATS

#define VTP

#ifndef	VTP
#define	IPC_STR         "IPC_IVI"
#else
#define	IPC_STR         "IPC_VTP"
#endif

/* logging */
#define log_debug(fmt, arg...)                                          \
	pr_debug("[ " IPC_STR ":%-20.20s: %-1.1d ] [DBG] " fmt "",      \
			__func__, __LINE__, ##arg)
#define log_error(fmt, arg...)                                          \
	pr_err("[ " IPC_STR ":%-20.20s: %-1.1d ] [ERR] " fmt "",        \
			__func__, __LINE__, ##arg)
#define log_info(fmt, arg...)                                           \
	pr_debug("[ " IPC_STR ":%-20.20s: %-1.1d ] [INF] " fmt "",      \
			__func__, __LINE__, ##arg)
#define log_notice(fmt, arg...)                                         \
	pr_warning("[ " IPC_STR ":%-20.20s: %-1.1d ] [INF] " fmt "",    \
			__func__, __LINE__, ##arg)
#define log_warning(fmt, arg...)                                        \
	pr_warning("[ " IPC_STR ":%-20.20s: %-1.1d ] [INF] " fmt "",    \
			__func__, __LINE__, ##arg)


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

void dumpBuffer(unsigned char* pBuffer, unsigned int len, const char *func, int lineNum);


#endif /* LOG_H */
