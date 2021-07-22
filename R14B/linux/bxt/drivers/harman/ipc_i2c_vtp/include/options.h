/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	options.h
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * This module provides routines to scan the command line options.
 */

#ifndef OPTIONS_H
#define OPTIONS_H

#include "ipc.h"


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

void dumpBuffer(unsigned char* pBuffer, unsigned int len, const char *func, int lineNum);
#ifdef	IPC_OPTIONS
void optInit(TOptions   *drvOptions);
TOptions* optGet(void);
void optPrint(TOptions *drvOptions);
#endif


#endif /* OPTIONS_H */
