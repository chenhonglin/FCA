/*****************************************************************************
 * Project         HARMAN AVB Stack 2.0
 *
 * (c) copyright   2018
 * Company         Harman International
 *                 All rights reserved
 *                 STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          scanOptions.c
 * @author        Gerd Zimmermann
 *
 * This module provides routines to scan the command line options.
 */


/*---------------------------------------------------------------------
 * INCLUDES
 *--------------------------------------------------------------------*/
#include "osIncludes.h"

/*---------------------------------------------------------------------
 * DEFINES
 *--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
 * EXTERNAL
 *--------------------------------------------------------------------*/
extern int32_t v;
extern int32_t t;
extern int32_t size;

extern uint8_t traceChannel;
extern uint32_t verbose;
extern uint32_t avbMemSize;
/*---------------------------------------------------------------------
 * VARIABLES
 *--------------------------------------------------------------------*/


/*---------------------------------------------------------------------
 * DECLARATION
 *--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
 * IMPLEMENTATION
 *--------------------------------------------------------------------*/

/**
 * Parse options from a command line.
 *
 * @param  argc - not used, only for common function declaration.
 * @param  argv - not used, only for common function declaration.
 *
 * @return void
 */
void optScanCommandLine( int32_t argc, char* const* argv )
{
	if ( v >= 0 )
	{
		verbose = v;
	}
	if ( t >= 0 )
	{
    	traceChannel = t;
	}
	if ( size > 0 )
	{
    	avbMemSize = size;
	}
}

/*** EOF ***/
