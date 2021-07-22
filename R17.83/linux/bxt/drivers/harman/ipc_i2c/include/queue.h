/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	queue.h
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * Common queue implementation.
 */

#ifndef QUEUE_H
#define QUEUE_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "types.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/

typedef void*   TQueueHandle;     /**< queue handle */


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

TQueueHandle    queueCreate(UInt32 elementSize, UInt32 size);
void            queueDestroy(TQueueHandle *handle);
UInt8           queueIsEmpty(TQueueHandle handle);
void            queueClear(TQueueHandle handle);
void*           queueGetReadPtr(TQueueHandle handle);
void            queueRemoveData(TQueueHandle handle);
void*           queueGetWritePtr(TQueueHandle handle);
void*           queueGetWritePtrExt(TQueueHandle handle, UInt32 length);
void            queueAddData(TQueueHandle handle);
UInt32          queueGetFreeElements(TQueueHandle handle);
UInt32          queueGetNumElements(TQueueHandle handle);


#endif /* QUEUE_H */
