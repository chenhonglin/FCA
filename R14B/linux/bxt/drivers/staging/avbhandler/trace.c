/*****************************************************************************
 * Project         HARMAN AVB Stack(2.0)
 *
 * (c) copyright   2018
 * Company         Harman International
 *                 All rights reserved
 *****************************************************************************/
/**
 * @file          trace.c
 * @ingroup       utilities
 * @author        Thorben Link, Nagaprasad A R
 *
 * Common trace module for Linux.
 */


/*---------------------------------------------------------------------
 * INCLUDES
 *--------------------------------------------------------------------*/
#include "types.h"
#include "trace.h"
#include <linux/slab.h>

#ifdef AVB_ENABLE_APP_LOGS
#undef AVB_ENABLE_APP_LOGS
#endif
/* Global variables. */
t_avbLogger *pMyLogger = NULL;

/* Forward declarations. */
extern uint64_t currTs64;
void initAvbLogger(uint32_t startAddr, uint32_t size);
void exitAvbLogger(void);

/**
 * Initialize trace module.
 *
 * @param  majorCode - major code
 * @param  minorBase - minor code base
 * @param  channel   - trace channel (printf or slog)
 * @return trace instance
 */
TTrace* traceInit(const UInt32 majorCode, const UInt32 minorBase, const UInt8 channel)
{
   TTrace* pTrace;

   pTrace = (TTrace*)kmalloc(sizeof(TTrace), GFP_KERNEL);
   if (NULL != (void*)pTrace)
   {
      pTrace->level = 0;
      pTrace->majorCode = majorCode;
      pTrace->minorBase = minorBase;
      pTrace->channel = channel;
   }

#ifdef AVB_ENABLE_APP_LOGS
   /* Linux - Logger mechanism to write AVB logs into reserved RAM area for AVB */
   initAvbLogger(LOGGER_START_ADDR, LOGGGER_MEM_SIZE);
#endif

   return pTrace;
}


/**
 * Frees all trace module resources.
 *
 * @param  pTrace - trace module pointer
 * @return none
 */
void traceDestroy(TTrace* pTrace)
{
   if (NULL != pTrace)
   {
      kfree(pTrace);
   }
#ifdef AVB_ENABLE_APP_LOGS
   exitAvbLogger();
#endif
}


/**
 * Set verbose level.
 *
 * @param  pTrace - trace module pointer
 * @param  level - verbose level
 * @return 1
 */
void traceSetLevel(TTrace* pTrace, const UInt32 level)
{
   if (NULL != pTrace)
   {
      pTrace->level = level;
   }
}


/**
 * Returns verbose level.
 *
 * @param  pTrace - trace module pointer
 * @return verbose level
 */
UInt32 traceGetLevel(const TTrace* const pTrace)
{
   if (NULL != pTrace)
   {
      return pTrace->level;
   }

   return TRACE_LEVEL_SHUTDOWN;
}


/**
 * Set trace channel.
 *
 * @param  pTrace - trace module pointer
 * @param  channel - trace channel
 * @return 1
 */
void traceSetChannel(TTrace* pTrace, const UInt8 channel)
{
   if (NULL != pTrace)
   {
      pTrace->channel = channel;
   }
}

/**
 * Initialize Linux memory logger.
 *
 * @param  startAddr - Start address of RAM
 * @param  channel -   Size of RAM memory
 * @return None
 */
#ifdef AVB_ENABLE_APP_LOGS
void initAvbLogger(uint32_t startAddr, uint32_t size)
{
    void* baseAddr = NULL;
    uint32_t minSz = sizeof(t_avbLogger) + (NO_OF_STARTUP_MSG * sizeof(t_avbLogElement)) + 10 * sizeof(t_avbLogElement);

    if(size < minSz)
    {
        //We need to have at least 10 message capacity
        printk(KERN_ALERT "%s: %s Log buffer is too small! Actual Size = %d, Min required = %d\n", APP_NAME, __FUNCTION__, size, minSz);
        return;
    }

    baseAddr = myOSMapDeviceMemory( startAddr, size, 0);
    if(baseAddr == NULL){
        printk("%s: ioremap failed in %s", APP_NAME, __FUNCTION__);
        return;
    }

    pMyLogger = baseAddr;
    memset( (void*)pMyLogger, 0, sizeof(t_avbLogger) );
    /* Initialize all members */
    pMyLogger->maxAddr = baseAddr + size;
    pMyLogger->rdIdx = 0;
    pMyLogger->wrIdx = 0;
    pMyLogger->wrapAroundCnt = 0;
    pMyLogger->startupMsgs = NO_OF_STARTUP_MSG;
    pMyLogger->maxMsgs = (size - sizeof(t_avbLogger)) / sizeof(t_avbLogElement);

    if (strlen(APP_NAME) < 20)
    {
        strcpy((char *) pMyLogger->projVariant, APP_NAME);
    }
    else
    {
        strcpy((char *) pMyLogger->projVariant, "AVB");
    }
    pMyLogger->isReady = true;

    //printk("%s:  Size of logger = %d, Size of log element = %d, Max No of log messages = %d, pMyLogger = 0x%x\n", APP_NAME, sizeof(t_avbLogger), sizeof(t_avbLogElement), pMyLogger->maxMsgs, pMyLogger );
}

/**
 * Exit Linux memory logger.
 *
 * @param  None
 * @return None
 */
void exitAvbLogger(void)
{
    if(NULL != pMyLogger)
    {
        /* In case application is still running */
        pMyLogger->isReady = false;
        myOSUnMapDeviceMemory((uintptr_t) pMyLogger, 1);
        pMyLogger = NULL;
    }
}
#endif

/**
 * Function to write log messages to memory
 *
 * @param  format Variable arguments
 * @return None
 */
void logMsgToMemory(uint8_t traceLevel, uint8_t logType, const char* funcName, uint32_t lineNo, const char* format, ...)
{
#ifdef AVB_ENABLE_APP_LOGS
    if(NULL != pMyLogger)
    {
        void* msgStartAddr = (void*) ((uint32_t) pMyLogger + sizeof(t_avbLogger));
        uint32_t idx = 0;
        t_avbLogElement* nextLog = (void*) ((uint32_t)msgStartAddr + (pMyLogger->wrIdx * sizeof(t_avbLogElement)));
        va_list argList;

        if( pMyLogger->wrIdx >= pMyLogger->maxMsgs  || ((uint32_t)nextLog + sizeof(t_avbLogElement)) >= (uint32_t) pMyLogger->maxAddr )
        {
            //printk(KERN_ALERT "AVB-LOG-KERN: Logger wraparound - pMyLogger->wrIdx = %d\n", pMyLogger->wrIdx);
            //Reset the pointer and throw an error
            pMyLogger->wrIdx = pMyLogger->startupMsgs;
            nextLog = (t_avbLogElement*) ((uint32_t) msgStartAddr + ((uint32_t) pMyLogger->startupMsgs * sizeof(t_avbLogElement)));
            //Set the wrap around count to indicate possible overflow
            ++pMyLogger->wrapAroundCnt;
        }

        //Calculate the time and put it in the log msg (to corelate with gPTP time on wire).
        nextLog->sec = myOSDiv64( currTs64, ONE_SECOND_NS);
        nextLog->nSec = (uint32_t)myOSMod64( currTs64, ONE_SECOND_NS);


        //Copy trace level and type
        nextLog->logLevel = traceLevel;
        nextLog->logType = logType;

        va_start(argList, format);
        vsnprintf((char *) (nextLog->msg + idx), SZ_LOG_MSG, format, argList);
        va_end(argList);
        nextLog->msg[SZ_LOG_MSG-1] = '\0'; //Make sure you end the string even if the application sends more than SZ_LOG_MSG chars.
        pMyLogger->wrIdx++;
    }
#endif
}

