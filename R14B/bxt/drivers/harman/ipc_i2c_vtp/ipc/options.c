/*****************************************************************************
 * Project              Harman Car Multimedia System
 *
 * c) copyright         2015
 * Company              Harman International Industries, Incorporated
 *                      All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file                ipc.c
 * @author              Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup             Kernel and Drivers
 *
 * This module contains the common thread-safe queue implementation.
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>

#include "types.h"
#include "hdlc.h"
#include "log.h"
#include "options.h"
#include "ipc.h"

/*****************************************************************************
 * STRUCTURES
 *****************************************************************************/

/*****************************************************************************
 * FUNCTION DEFINITIONS
 *****************************************************************************/

void dumpBuffer(unsigned char* pBuffer, unsigned int len, const char *func, int lineNum)
{
        int  wIndex = 0;
        unsigned char   buffer[RTXQ_BUFLEN];
	len = (len > (RTXQ_BUFLEN))? (RTXQ_BUFLEN):len;

        log_info("--> (Func: %s, Line:%d Len=%d) : ", func, lineNum, len);
        for(wIndex = 0 ; wIndex < len; wIndex += 8 )
        {
		log_info("--> 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                        (unsigned int)pBuffer[wIndex], (unsigned int)pBuffer[wIndex+1], (unsigned int)pBuffer[wIndex+2], (unsigned int)pBuffer[wIndex+3],
                        (unsigned int)pBuffer[wIndex+4], (unsigned int)pBuffer[wIndex+5], (unsigned int)pBuffer[wIndex+6], (unsigned int)pBuffer[wIndex+ 7]
                        );
        }
}

#ifdef	IPC_OPTIONS
void optInit(TOptions   *drvOptions)
{
	drvOptions->verbose = 5;
	drvOptions->numOfChannels = IPC_MAX_CHANNELS;
	drvOptions->priority = 21;
	drvOptions->instance = 0;
	drvOptions->protocolVersion = IPC_PROTOCOL_VERSION_1;
	//strcpy(drvOptions->deviceName, "ipc");

	g_pDrvOptions = drvOptions;
	return;
}

TOptions* optGet(void)
{
	return g_pDrvOptions;
}

void optPrint(TOptions *drvOptions)
{
	log_notice("dev-ipc: Versions:");

	log_notice("dev-ipc: Options:");
	log_notice("dev-ipc:   Verbose:      %u", drvOptions->verbose);
	log_notice("dev-ipc:   Priority:     %u", drvOptions->priority);
	log_notice("dev-ipc:   Num channels: %u", drvOptions->numOfChannels);
	//log_notice("dev-ipc:   Device name:  '%s'", drvOptions->deviceName);
	return;
}
#endif
