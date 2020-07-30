/*****************************************************************************
 * Project     		Harman Car Multimedia System
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	ipc.c
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * This module contains the common thread-safe queue implementation.
 */

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "types.h"
#include "log.h"
#include "queue.h"

/*****************************************************************************
 * STRUCTURES
 *****************************************************************************/

/* Queue data structure. */
typedef struct
{
	void*  pData;  /**< data pointer */
	UInt32 length; /**< size of data */
}TQueueData;

/* Queue structure. Used as handle */
typedef struct
{
	UInt32		elementSize;	/**< size of one element of the queue */
	UInt32		size;		/**< size of the queue */
	TQueueData*	pQueueData;	/**< queue data */
	UInt32		writeIndex;	/**< pointer to current element for write operation */
	UInt32		readIndex;	/**< pointer to current element for write operation */
	UInt32		nkfreeElements;	/**< no fo free elements in the queue */
	struct mutex	accessMutex;	/**< mutex to protect the access to the queue */
}TQueue;


/*****************************************************************************
 * FUNCTION DEFINITIONS
 ****************************************************************************/

/**
 * Create a queue with the given sizes.
 *
 * @param  elementSize  - data size of one queue element
 * @param  size         - number of elements
 * @return pointer to a valid queue handle or NULL if failure
 */
TQueueHandle queueCreate(UInt32 elementSize, UInt32 size)
{
	TQueue*	pQueue;
	Int32	i = 0;
	UInt8	error = FALSE;

	pQueue = (TQueue*)kmalloc(sizeof(TQueue), GFP_KERNEL);
	if (NULL != pQueue)
	{
		pQueue->pQueueData = kmalloc((size * sizeof(TQueueData)), GFP_KERNEL);
		if (NULL != pQueue->pQueueData)
		{
			for (i = 0; i < size; i++)
			{
				pQueue->pQueueData[i].length = elementSize;
				pQueue->pQueueData[i].pData = kmalloc(elementSize, GFP_KERNEL);
				if (NULL == pQueue->pQueueData[i].pData)
				{
					error = TRUE;
					break;
				}
			}
		}
		else
		{
			error = TRUE;
		}

		if (FALSE == error)
		{
			pQueue->size = size;
			pQueue->elementSize = elementSize;
			pQueue->writeIndex = 0;
			pQueue->readIndex = pQueue->writeIndex;
			pQueue->nkfreeElements = pQueue->size;

			mutex_init(&pQueue->accessMutex);
		}
		else
		{
			if (NULL != pQueue->pQueueData)
			{
				while (--i >= 0)
				{
					if (NULL != pQueue->pQueueData[i].pData)
					{
						kfree(pQueue->pQueueData[i].pData);
					}
				}

				kfree(pQueue->pQueueData);
			}

			kfree(pQueue);
			pQueue = NULL;
		}
	}

	return pQueue;
}


/**
 * Destroy all resources allocated by the given queue handle.
 *
 * @param  handle - queue handle
 * @return none
 */
void queueDestroy(TQueueHandle *handle)
{
	UInt32 i;

	TQueue* pQueue = (TQueue*)(*handle);
	if (NULL != pQueue)
	{
		if (NULL != pQueue->pQueueData)
		{
			for (i = 0; i < pQueue->size; i++)
			{
				if (NULL != pQueue->pQueueData[i].pData)
				{
					kfree(pQueue->pQueueData[i].pData);
				}
			}

			kfree(pQueue->pQueueData);
		}

		mutex_destroy(&pQueue->accessMutex);
		kfree(pQueue);
		*handle = NULL;
	}
	return;
}


/**
 * Checks if the queue is empty or not.
 *
 * @param  handle  - queue handle
 * @return TRUE | FALSE
 */
UInt8 queueIsEmpty(TQueueHandle handle)
{
	UInt8 isEmpty = TRUE;

	if (NULL != handle)
	{
		TQueue* pQueue = (TQueue*)handle;

		mutex_lock(&pQueue->accessMutex);
		if (pQueue->size != pQueue->nkfreeElements)
		{
			isEmpty = FALSE;
		}
		mutex_unlock(&pQueue->accessMutex);
	}

	return isEmpty;
}


/**
 * Clears all used elements.
 *
 * @param  handle - queue handle
 * @return none
 */
void queueClear(TQueueHandle handle)
{
	if (NULL != handle)
	{
		TQueue* pQueue = (TQueue*)handle;

		mutex_lock(&pQueue->accessMutex);
		pQueue->writeIndex = 0;
		pQueue->readIndex = pQueue->writeIndex;
		pQueue->nkfreeElements = pQueue->size;
		mutex_unlock(&pQueue->accessMutex);
	}
}


/**
 * Returns a read pointer based on the given read handle.
 *
 * This function returns a read pointer to the current data element to
 * read. After this function returns the element is not freed. To free
 * the queue element 'queueRemoveData' has to be called.
 *
 * @param  handle     - queue handle
 * @return data pointer to the current element to read or NULL if queue is empty
 */
void* queueGetReadPtr(TQueueHandle handle)
{
	void* pElement = NULL;

	if (NULL != handle)
	{
		TQueue* pQueue = (TQueue*)handle;

		mutex_lock(&pQueue->accessMutex);

		if (pQueue->size != pQueue->nkfreeElements)
		{
			pElement = pQueue->pQueueData[pQueue->readIndex].pData;
		}

		mutex_unlock(&pQueue->accessMutex);
	}

	return pElement;
}


/**
 * Frees the current queue element.
 *
 * @param  handle - queue handle
 * @return data pointer to the current element to read
 */
void queueRemoveData(TQueueHandle handle)
{
	UInt32 nextIndex;

	if (NULL != handle)
	{
		TQueue* pQueue = (TQueue*)handle;

		mutex_lock(&pQueue->accessMutex);

		/* buffer not empty? */
		if (pQueue->size != pQueue->nkfreeElements)
		{
			/* calculate Pointer to next element */
			nextIndex = pQueue->readIndex + 1;
			if (nextIndex >= pQueue->size)
			{
				nextIndex = 0;
			}

			pQueue->readIndex = nextIndex; /* set Pointer to next element */
			pQueue->nkfreeElements++;       /* increment counter of kfree elements */
		}

		mutex_unlock(&pQueue->accessMutex);
	}
}


/**
 * Returns the data pointer to the head of the queue.
 *
 * This function returns only the data pointer. To add the data to
 * the queue, 'queueAddData' has to be called.
 *
 * @param  handle - queue handle
 * @param  length - data length
 * @return data pointer or NULL if queue is full
 */
void* queueGetWritePtr(TQueueHandle handle)
{
	void* pElement = NULL;

	if (NULL != handle)
	{
		TQueue* pQueue = (TQueue*)handle;

		mutex_lock(&pQueue->accessMutex);

		if (0 != queueGetFreeElements(handle))
		{
			pElement = pQueue->pQueueData[pQueue->writeIndex].pData;
		}

		mutex_unlock(&pQueue->accessMutex);
	}

	return pElement;
}


/**
 * Adds data to the head of the queue.
 *
 * @param  handle - queue handle
 * @return none
 */
void queueAddData(TQueueHandle handle)
{
	TQueue* pQueue = (TQueue*)handle;
	UInt32 	nextIndex;

	if (NULL != handle)
	{
		mutex_lock(&pQueue->accessMutex);

		/* compute pointer to next element */
		nextIndex = pQueue->writeIndex + 1;
		if (nextIndex >= pQueue->size)
		{
			nextIndex = 0;
		}

		/* set Pointer to next element */
		pQueue->writeIndex = nextIndex;

		/* decrement counter of kfree elements */
		pQueue->nkfreeElements--;

		mutex_unlock(&pQueue->accessMutex);
	}
}


/**
 * Returns the number of free elements.
 *
 * @param  handle - queue handle
 * @return number of kfree elements
 */
UInt32 queueGetFreeElements(TQueueHandle handle)
{
	UInt32 nkfreeElements = 0;

	if (NULL != handle)
	{
		TQueue* pQueue = (TQueue*)handle;

		nkfreeElements = pQueue->nkfreeElements;
	}

	return nkfreeElements;
}


/**
 * Returns the number of elements.
 *
 * @param  handle - queue handle
 * @return number of valid elements
 */
UInt32 queueGetNumElements(TQueueHandle handle)
{
	UInt32 nkfreeElements = 0;

	if (NULL != handle)
	{
		TQueue* pQueue = (TQueue*)handle;

		return (pQueue->size - pQueue->nkfreeElements);
	}

	return nkfreeElements;
}


/**
 * Returns the max data size of an element.
 *
 * @param  handle - queue handle
 * @return number of kfree elements
 */
UInt32 queueGetMaxElementSize(TQueueHandle handle)
{
	UInt32 elementSize = 0;
	UInt32 i;

	if (NULL != handle)
	{
		TQueue* pQueue = (TQueue*)handle;

		mutex_lock(&pQueue->accessMutex);
		for (i = 0; i < pQueue->size; i++)
		{
			if (elementSize < pQueue->pQueueData[i].length)
			{
				elementSize = pQueue->pQueueData[i].length;
			}
		}
		mutex_unlock(&pQueue->accessMutex);
	}

	return elementSize;
}


/**
 * Returns the data size of the head element.
 *
 * @param  handle - queue handle
 * @return number of kfree elements
 */
UInt32 queueGetWriteElementSize(TQueueHandle handle)
{
	UInt32 size = 0;

	if (NULL != handle)
	{
		TQueue* pQueue = (TQueue*)handle;

		mutex_lock(&pQueue->accessMutex);
		size = pQueue->pQueueData[pQueue->writeIndex].length;
		mutex_unlock(&pQueue->accessMutex);
	}

	return size;
}
