/*****************************************************************************
 * Project         HARMAN AVB Stack(2.0)
 *
 * (c) copyright   2018
 * Company         Harman International
 *                 All rights reserved
 *****************************************************************************/
/**
 * @file          logMem.h
 * @ingroup       utilities
 * @author        Nagaprasad A R
 *
 * Logger to copy logs to memory.
 */

#ifndef __LOGMEM_H
#define __LOGMEM_H


/*---------------------------------------------------------------------
 * INCLUDES
 *--------------------------------------------------------------------*/


#define TRACE_LEVEL_SHUTDOWN  0   /* Shut down the system NOW. eg: for OEM use */
#define TRACE_LEVEL_CRITICAL  1   /* Unexpected unrecoverable error. eg: hard disk error */
#define TRACE_LEVEL_ERROR     2   /* Unexpected recoverable error. eg: needed to reset a hw controller */
#define TRACE_LEVEL_WARNING   3   /* Expected error. eg: parity error on a serial port */
#define TRACE_LEVEL_NOTICE    4   /* Warnings. eg: Out of paper */
#define TRACE_LEVEL_INFO      5   /* Information. eg: Printing page 3 */
#define TRACE_LEVEL_DEBUG1    6   /* Debug messages eg: Normal detail */
#define TRACE_LEVEL_DEBUG2    7   /* Debug messages eg: Fine detail */

#define LOG_TYPE_DEFAULT           10   /* Log type: version message */
#define LOG_TYPE_VERSION           11   /* Log type: version message */
#define LOG_TYPE_CONFIG            12   /* Log type: config message */
#define LOG_TYPE_UNSUPPORTED       13   /* Log type: unsupported log message */
#define LOG_TYPE_ERRUNSUPPORTED    14   /* Log type: unsupported log error */

/* Linux userspace logging - constants */
#define SZ_LOG_MSG                  100 /* Maximum size of each log message. */
#define NO_OF_STARTUP_MSG           120 /* First N reserved locvations for startup log messages (no overwrite!) */


/* Constant strings */
#define PREFIX_ERROR        " ERROR: "
#define PREFIX_WARNING      " WARNING: "
#define PREFIX_VERSION      " VERSION: "
#define PREFIX_CONFIG       " CONFIG: "
#define PREFIX_UNSPRT       " UNSUPPORTED: "
#define PREFIX_ERRUNSPRT    " ERRORUNSUPPORTED: "

/* Linux userspace logging - structures */
typedef volatile struct _avbLogger
{
    uint8_t     isReady;
    uint8_t     wrapAroundCnt;
    char        projVariant[20];
    int32_t     rdIdx;
    int32_t     wrIdx;
    int32_t     startupMsgs;
    int32_t     maxMsgs;
    void*       maxAddr;     //Maximum address of the logger in RAM.
}t_avbLogger;

typedef volatile struct _avbLogElement
{
    uint32_t    sec;
    uint32_t    nSec;
    uint8_t     logLevel;
    uint8_t     logType;
    char        msg[SZ_LOG_MSG];
}t_avbLogElement;

#endif /* __LOGMEM_H */

