/*****************************************************************************
 * Project         HARMAN AVB Stack(2.0)
 *
 * (c) copyright   2018
 * Company         Harman International
 *                 All rights reserved
 *****************************************************************************/
/**
 * @file          trace.h
 * @ingroup       utilities
 * @author        Thorben Link, Nagaprasad A R
 *
 * Common trace module for Linux..
 */

#ifndef __TRACE_H
#define __TRACE_H


/*---------------------------------------------------------------------
 * INCLUDES
 *--------------------------------------------------------------------*/
#include "osIncludes.h"
#include "types.h"
#include "logMem.h"

/*---------------------------------------------------------------------
 * DEFINES
 *--------------------------------------------------------------------*/
#define APP_NAME    "AVB-KERNEL"
#define TRACE_CHANNEL_SLOG    0
#define TRACE_CHANNEL_PRINTF  1

#ifndef _SLOGC_SYSTEM
#define _SLOGC_SYSTEM 		20002
#endif



/* Function declaration for the Linux application logger function (should be declared before logger macros!) */
void logMsgToMemory(uint8_t traceLevel, uint8_t logType, const char* funcName, uint32_t lineNo, const char* format, ...);


/**
 * log message macro
 */


#define logMessage(handle, traceLevel, logType, lineNo, ...)                                        \
   logMsgToMemory(traceLevel, logType, __FUNCTION__, lineNo, __VA_ARGS__);                          \
   if (traceLevel <= handle->level)                                                                 \
   {                                                                                                \
       if( traceLevel <= TRACE_LEVEL_WARNING)                                                       \
       {                                                                                            \
           if(TRACE_LEVEL_ERROR == traceLevel && LOG_TYPE_DEFAULT == logType)                       \
           {                                                                                        \
               printk("%s%s%s, %d ", APP_NAME, PREFIX_ERROR, __FUNCTION__, lineNo);                  \
           }                                                                                        \
           else if(TRACE_LEVEL_WARNING == traceLevel)                                                    \
           {                                                                                        \
               printk("%s%s%s, %d ", APP_NAME, PREFIX_WARNING, __FUNCTION__, lineNo);            \
           }                                                                                        \
           else if(LOG_TYPE_VERSION == logType)                                                          \
           {                                                                                        \
               printk("%s%s", APP_NAME, PREFIX_VERSION);                                         \
           }                                                                                        \
           else if( LOG_TYPE_CONFIG == logType )                                                         \
           {                                                                                        \
               printk("%s%s",  APP_NAME, PREFIX_CONFIG);                                         \
           }                                                                                        \
           else if(LOG_TYPE_UNSUPPORTED == logType)                                                      \
           {                                                                                        \
               printk("%s%s%s, %d ", APP_NAME, PREFIX_UNSPRT, __FUNCTION__, lineNo);             \
           }                                                                                        \
           else if(LOG_TYPE_ERRUNSUPPORTED == logType)                                                   \
           {                                                                                        \
               printk("%s%s%s, %d ", APP_NAME, PREFIX_ERRUNSPRT, __FUNCTION__, lineNo);          \
           }                                                                                        \
       }                                                                                            \
       printk(__VA_ARGS__);                                                                         \
       printk("\n");                                                                                \
   }                                                                                                \




#define logError(...)               logMessage(TRACE_DEFAULT, TRACE_LEVEL_ERROR, LOG_TYPE_DEFAULT, __LINE__,  __VA_ARGS__)
#define logVersion(...)				logMessage(TRACE_DEFAULT, TRACE_LEVEL_ERROR, LOG_TYPE_VERSION, __LINE__,   __VA_ARGS__)
#define logConfig(...)				logMessage(TRACE_DEFAULT, TRACE_LEVEL_ERROR, LOG_TYPE_CONFIG, __LINE__,   __VA_ARGS__)
#define logUnsupported(...)			logMessage(TRACE_DEFAULT, TRACE_LEVEL_ERROR, LOG_TYPE_UNSUPPORTED, __LINE__,   __VA_ARGS__)
#define logErrorUnsupported(...)	logMessage(TRACE_DEFAULT, TRACE_LEVEL_ERROR, LOG_TYPE_ERRUNSUPPORTED, __LINE__,   __VA_ARGS__)

#define logWarning(...) logMessage(TRACE_DEFAULT, TRACE_LEVEL_WARNING, LOG_TYPE_DEFAULT, __LINE__, __VA_ARGS__)
#define logInfo(...) 	logMessage(TRACE_DEFAULT, TRACE_LEVEL_INFO, LOG_TYPE_DEFAULT, __LINE__, __VA_ARGS__)
#define logDebug1(...) logMessage(TRACE_DEFAULT, TRACE_LEVEL_DEBUG1, LOG_TYPE_DEFAULT, __LINE__, __VA_ARGS__)
#define logDebug2(...) logMessage(TRACE_DEFAULT, TRACE_LEVEL_DEBUG2, LOG_TYPE_DEFAULT, __LINE__, __VA_ARGS__)




/*---------------------------------------------------------------------
 * STRUCTURES
 *--------------------------------------------------------------------*/
typedef struct
{
   struct timespec slogTime;
   UInt32 level;
   UInt32 majorCode;
   UInt32 minorBase;
   UInt8  channel;
}TTrace;


/*---------------------------------------------------------------------
 * VARIABLES
 *--------------------------------------------------------------------*/
extern TTrace* TRACE_DEFAULT;


/*---------------------------------------------------------------------
 * FUNCTION PROTOTYPES
 *--------------------------------------------------------------------*/
TTrace*     traceInit(const UInt32 majorCode, const UInt32 minorBase, const UInt8 channel);
void        traceDestroy(TTrace* pTrace);
void        traceSetLevel(TTrace* pTrace, const UInt32 level);
UInt32      traceGetLevel(const TTrace* const pTrace);
void        traceSetChannel(TTrace* pTrace, const UInt8 channel);
UInt32      traceGetTime(const TTrace* const pTrace);
const char* traceGetDataString(const UInt8* const pData, const UInt16 length, char* string, const UInt32 maxChars);
const char* traceGetDataStringExt(const UInt8* const pData, const UInt16 length, char* string, const UInt32 maxChars, const UInt32 retOffset, const char* const slideInString);



#endif /* __TRACE_H */

