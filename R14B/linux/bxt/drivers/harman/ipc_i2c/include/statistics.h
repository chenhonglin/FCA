/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	ipc.h
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * Statistics module.
 */

#ifndef STATISTICS_H
#define STATISTICS_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "log.h"

#ifndef	IPC_STATS
#include <linux/seq_file.h>
#endif


/*****************************************************************************
 * DEFINES
 *****************************************************************************/

#define STATISTICS_SET(member, value)      {member = value;}
#define STATISTICS_ADD(member, value)      {member += value;}
#define STATISTICS_SUB(member, value)      {member -= value;}
#define STATISTICS_INC(member)             {member ++;}
#define STATISTICS_DEC(member)             {member --;}


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

#ifdef	IPC_STATS
void statisticsPrint(IPCAttr_t *pIpcRoot);
#else
void statisticsPrint(struct seq_file *m, IPCAttr_t *pIpcRoot);
#endif


#endif /* STATISTICS_H */
