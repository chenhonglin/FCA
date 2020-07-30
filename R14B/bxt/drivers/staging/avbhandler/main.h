/*****************************************************************************
 * Project         HARMAN AVB Stack(2.0)
 *
 * (c) copyright   2018
 * Company         Harman International
 *                 All rights reserved
 *****************************************************************************/
/**
 * @file          main.h
 * @author        Siva Kumar Mellemputi
 *
 */

#ifndef __MAIN_H
#define __MAIN_H

/*---------------------------------------------------------------------
 * INCLUDES
 *--------------------------------------------------------------------*/
#include "trace.h"
#include "avbCharDev.h"

/*---------------------------------------------------------------------
 * STRUCTURES
 *--------------------------------------------------------------------*/
typedef struct _LocalUnchachedMemory
{
	int32_t handle;
	void* buffer;
	uint32_t size;
	uint64_t physBuffer;
}t_LocalUnchachedMemory;

/*---------------------------------------------------------------------
 * DEFINES
 *--------------------------------------------------------------------*/
#define DEFINE_LOCAL_UNCHACHED_MEMORY( a, s )		\
t_LocalUnchachedMemory a = { .handle=-1, .buffer=NULL, .size=s, .physBuffer=0} ;

#endif /* __MAIN_H */

