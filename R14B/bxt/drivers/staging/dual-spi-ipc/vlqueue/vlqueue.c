/*******************************************************************************
 * File:          vlqueue.c
 *
 * Description:   Variable Length Queue. This module implements an efficient
 *                variable length queue where messages are variable length.
 *                The queue operates as FIFO, where messages must be read
 *                in the same order which they are queued.
 *
 * Author:        David Rogala
 *
 * Copyright:     (c) 2018 Harman International, LLC
 ******************************************************************************/
#ifdef LINUX_KERNEL_INC
#include <linux/string.h>
#else
#include <string.h>
#endif

#include "vlqueue.h"


/******************************************************************************
* BUILD TIME CONFIGURABLE PARAMETERS
******************************************************************************/
/* The following macros must be defined to ensure synchronization more than
 * a single thread can queue and dequeue data. If a only single thread can
 * queue and a single (the same or separate) thread will dequeue, no critical
 * sections are necessary */
#ifndef VLQueue_DefineCriticalVars
#define VLQueue_DefineCriticalVars
#endif

#ifndef VLQueue_EnterCritical
#define VLQueue_EnterCritical
#endif

#ifndef VLQueue_ExitCritical
#define VLQueue_ExitCritical
#endif

/******************************************************************************
* MACROS AND CONSTANTS
******************************************************************************/
#define calcFreeSpace(wi, ri, size) \
	(((ri) > (wi)) ? (ri) - (wi) - 1 : (size) - (wi) + (ri) - 1)

typedef enum
{
	ReadType_PopFront = 0,
	ReadType_Front,
	ReadType_Back,
	ReadType_Invalid
} ReadType_t;

/******************************************************************************
* FUNCTION PROTOTYPES
******************************************************************************/
static int32_t encodeLength (uint8_t *buf, int32_t length);
static int32_t decodeLength (uint32_t* index,
									  const uint8_t *buffer,
									  uint32_t size);
static int32_t readInternal (VLQueue_ccb_t * ccb,
									  uint8_t *buf,
									  uint32_t bufSize,
									  ReadType_t type);

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/
/******************************************************************************
* FUNCTION:       encodeLength
* DESCRIPTION:    This function receives the length of a message to be stored
*                 in the buffer, and encodes the header to determine the length
*                 of the messsage:
*                 format:
*                 byte [0], bit 6 & 7: command byte. 0 = 1 byte length field,
*                    1 = 2 byte length field, 2 = 3 byte length field, 3 = 4
*                    byte length field.
*                 byte [0], bits 5-0: most significant bits of length parameter
*                 bytes [1-3] (optional):
* INPUT(S):
*                 length: length of message, to be encoded.
* OUTPUT(S):      buf: output buffer. Must be at least 4 bytes:
* RETURN VALUE:   if <= 0, error. else, number of bytes used to encode message
*                 length.
******************************************************************************/
static int32_t encodeLength (uint8_t *buf, int32_t length)
{
	if(length > 0)
	{
		if(length < 0x40u)
		{
			buf[0] = (0u << 6) | length;
			return 1;
		}
		else if(length < 0x4000u)
		{
			buf[0] = (1u << 6) | ((length & 0x3F00u) >> 8);
			buf[1] = length & 0xFFu;
			return 2;
		}

#if VLQUEUE_16_BIT_LEN == 1
		/* if 16-bit counter mode is enabled, return an error if length > 0xFFFFu*/
		else if(length < 0x10000u)
#else
		else if(length < 0x400000u)
#endif
		{
			buf[0] = (2u << 6) | ((length & 0x3F0000u) >> 16);
			buf[1] = (length & 0xFF00u) >> 8;
			buf[2] = length & 0xFFu;
			return 3;
		}
#if VLQUEUE_16_BIT_LEN != 1
		else if(length < 0x40000000u)
		{
			buf[0] = (3u << 6) | ((length & 0x3F000000u) >> 24);
			buf[1] = (length & 0xFF0000u) >> 16;
			buf[1] = (length & 0xFF00u) >> 8;
			buf[3] = length & 0xFFu;
			return 4;
		}
#endif
		else
		{
			return VLQUEUE_ERR_RANGE;
		}
	}
	else
	{
		return VLQUEUE_ERR_RANGE;
	}
}

/******************************************************************************
* FUNCTION:       decodeLength
* DESCRIPTION:    This function decodes the length field of a message stored
*                 in the variable length queue. the index field is updated to
*                 point to the beginning of the message data.
*                 format:
*                 byte [0], bit 6 & 7: command byte. 0 = 1 byte length field,
*                    1 = 2 byte length field, 2 = 3 byte length field, 3 = 4
*                    byte length field.
*                 byte [0], bits 5-0: most significant bits of length parameter
*                 bytes [1-3] (optional):
* INPUT(S):       *index: read index of buffer
*                 *buffer: input buffer
*                 size buffer size:
* OUTPUT(S):      *index: read index of buffer
*                 buf: output buffer. Must be at least 4 bytes:
* RETURN VALUE:   if <= 0, error. else, number of bytes used to encode message
*                 length.
******************************************************************************/
static int32_t decodeLength (uint32_t* index,
									  const uint8_t *buffer,
									  uint32_t size)
{
	uint32_t myIndex = *index;
	int32_t ret = 0;
	uint32_t cmd;

	/* parse command field to determine number of bytes used to encode length*/
	cmd = (buffer[myIndex] >> 6) & 0x3u;

	/* decode message length */
	ret |= (buffer[myIndex++] & 0x3F) << (cmd*8);

	while(cmd--)
	{
		if (myIndex >= size)
		{
			myIndex = 0;
		}

		ret |= (buffer[myIndex++] & 0xFF) << (cmd*8);
	};

	if (myIndex >= size)
	{
		*index = 0;
	}
	else
	{
		*index = myIndex;
	}

	return ret;
}

/******************************************************************************
* FUNCTION:       readInternal
* DESCRIPTION:    This function performs all types of read operations.
* INPUT(S):       ccb: context block to be initialized (also output)
*                 bufSize: size of the output buffer
*                 type: Read type. possible options:
*                    ReadType_Back: read the message at the back of the buffer
*                       without removing it
*                    ReadType_Front: read the message at the front of the buffer
*                       without removing it
*                    ReadType_PopFront: read the message at the front of the buffer
*                       and remove it
* OUTPUT(S):      ccb: context block
*                 *buf: pointer to output buffer
* RETURN VALUE:   return: less than zero, error; else size of message read
******************************************************************************/
static int32_t readInternal (VLQueue_ccb_t * ccb,
									  uint8_t *buf,
									  uint32_t bufSize,
									  ReadType_t type)
{
	int32_t ret = 0;
	uint8_t *buffer;
	uint32_t bufferLength;
	uint32_t readIndex;
	uint32_t msgCnt  = 0;
	VLQueue_DefineCriticalVars;

	/* error checking of input arguments*/
	if (ccb == NULL ||
			ccb->buffer == NULL ||
			buf == NULL ||
			bufSize <= 0)
	{
		return VLQUEUE_ERR_PARAM;
	}

	/* enter critical section */
	VLQueue_EnterCritical;

	/* cache context block variables to registers*/
	buffer         = ccb->buffer;
	bufferLength   = ccb->bufferLength;
	readIndex      = ccb->readIndex;
	msgCnt         = ccb->msgCnt;

	if(msgCnt == 0)
	{
		/* no messages in queue. return error*/
		ret = VLQUEUE_ERR_QUEUE_EMTPY;
	}
	else
	{
		/* copy the message count to an iterator. This iterator may be
		 * used to find the last message in the back of the queue*/
		uint32_t i = msgCnt;

		/* the following loop handles both reads of the front and back
		 * of the queue*/
		for (;;)
		{
			ret = decodeLength(&readIndex,
							 buffer,
							 bufferLength);

			/* if the read type is back, iterate through messages until
			 * the last message is found (msgCnt == 1)*/
			if ((type == ReadType_Back) &&
					(i-- > 1))
			{
				/* Increase the read index to skip past the current message*/
				readIndex += ret;
				if (readIndex > bufferLength)
				{
					readIndex -= bufferLength;
				}
			}
			else
			{
				/* We have found the desired message, break the loop*/
				break;
			}
		}

		if(ret > bufSize)
		{
			/* message is larger then output buffer. Return an error*/
			ret = VLQUEUE_ERR_READ_BUFFER_SZ;
		}
		else
		{
			i = ret;

			/* read and copy message to output buffer*/
			while(i--)
			{
				*buf++ = buffer[readIndex++];

				if (readIndex >= bufferLength)
				{
					readIndex = 0;
				}
			}

			/* if read type is popFront, update message counter and read index in
			 * the context block. This will remove the message*/
			if (type == ReadType_PopFront)
			{
				ccb->readIndex = readIndex;
				ccb->msgCnt    = msgCnt - 1;
			}
		}
	}

	/* exit critical section */
	VLQueue_ExitCritical;

	/* return negative status or number of bytes read if successful*/
	return ret;
}

/******************************************************************************
* PUBLIC FUNCTIONS
******************************************************************************/
/******************************************************************************
* FUNCTION:       vlqueue_getversion
* DESCRIPTION:    This function returns the version of the vlqueue module
* INPUT(S):       ccb: context block to be initialized (also output)
* OUTPUT(S):      void
* RETURN VALUE:   return: less than 0, error; else buffer free space
******************************************************************************/
uint32_t vlqueue_getversion (VLQueue_ccb_t * ccb)
{
	return VLQUEUE_MODULE_VERSION_BCD;
}

/******************************************************************************
* FUNCTION:       VLQueue_Init
* DESCRIPTION:    This function initializes a VLQueue context block
* INPUT(S):       ccb: context block to be initialized (also output)
*                 buffer: byte array to be used as buffer
*                 length: length of the buffer
*                 *usrPtr: user pointer useful for user defined synchronization
* OUTPUT(S):      ccb: context block
* RETURN VALUE:   return: if 0, success, failure otherwise
******************************************************************************/
#if VLQUEUE_ENABLE_USER_PTR == 1
int32_t vlqueue_init (VLQueue_ccb_t * ccb,
							 uint8_t* buffer,
							 uint32_t length,
							 void *usrPtr)
#else
int32_t vlqueue_init (VLQueue_ccb_t * ccb,
							 uint8_t* buffer,
							 uint32_t length)
#endif
{
	int32_t ret;
	VLQueue_DefineCriticalVars;

	if ((ccb == NULL) ||
		(buffer == NULL) ||
#if VLQUEUE_16_BIT_LEN == 1
		(length >= 0x10000u) ||
#endif
		(length == 0))
	{
		ret = VLQUEUE_ERR_PARAM;
	}
	else
	{
#if VLQUEUE_ENABLE_USER_PTR == 1
		/* assign user pointer. It can be used to customize sync method for
		 * particular environment.
		 * NOTE: ccb->usrPtr must be set prior to calling VLQueue_EnterCritical */
		ccb->usrPtr = usrPtr;
#endif

		VLQueue_EnterCritical;

		memset(buffer, 0, length);
		ccb->bufferLength = length;
		ccb->writeIndex = 0;
		ccb->readIndex = 0;
		ccb->msgCnt = 0;

		ccb->buffer = buffer;

		VLQueue_ExitCritical;
		ret = VLQUEUE_OK;
	}

	return ret;
}

/******************************************************************************
* FUNCTION:       vlqueue_empty
* DESCRIPTION:    This function empties the buffer
* INPUT(S):       ccb: context block to be initialized (also output)
* OUTPUT(S):      none
* RETURN VALUE:   return: less than 0, error; else buffer free space
******************************************************************************/
int32_t vlqueue_empty (VLQueue_ccb_t * ccb)
{
	int32_t ret;
	VLQueue_DefineCriticalVars;

	if (ccb == NULL)
	{
		ret = VLQUEUE_ERR_PARAM;
	}
	else
	{
		VLQueue_EnterCritical;
		ccb->writeIndex = 0;
		ccb->readIndex = 0;
		ccb->msgCnt    = 0;
		VLQueue_ExitCritical;

		ret = VLQUEUE_OK;
	}

	return ret;
}

/******************************************************************************
* FUNCTION:       vlqueue_size
* DESCRIPTION:    This function returns the number of queued messages in the
*                 buffer
* INPUT(S):       ccb: context block to be initialized (also output)
* OUTPUT(S):      void
* RETURN VALUE:   return: less than 0, error; else number of messages queued
******************************************************************************/
int32_t vlqueue_size (VLQueue_ccb_t * ccb)
{
	int32_t ret;
	VLQueue_DefineCriticalVars;

	if (ccb == NULL)
	{
		ret = VLQUEUE_ERR_PARAM;
	}
	else
	{
		VLQueue_EnterCritical;
		ret = ccb->msgCnt;
		VLQueue_ExitCritical;
	}

	return ret;
}

/******************************************************************************
* FUNCTION:       vlqueue_front
* DESCRIPTION:    This function reads the message at the front of the buffer,
*                 without removing it
* INPUT(S):       ccb: context block to be initialized (also output)
*                 bufSize: size of output buffer
* OUTPUT(S):      *buf: output buffer
* RETURN VALUE:   return: less than 0, error; else the size of the message read
******************************************************************************/
int32_t vlqueue_front (VLQueue_ccb_t * ccb,
							  uint8_t *buf,
							  uint32_t bufSize)
{
	return readInternal (ccb,
								buf,
								bufSize,
								ReadType_Front);
}

/******************************************************************************
* FUNCTION:       vlqueue_back
* DESCRIPTION:    This function reads the message from the back of the buffer,
*                 without removing it
* INPUT(S):       ccb: context block to be initialized (also output)
*                 bufSize: size of output buffer
* OUTPUT(S):      *buf: output buffer
* RETURN VALUE:   return: less than 0, error; else the size of the message read
******************************************************************************/
int32_t vlqueue_back (VLQueue_ccb_t * ccb,
							 uint8_t *buf,
							 uint32_t bufSize)
{
	return readInternal (ccb,
								buf,
								bufSize,
								ReadType_Back);
}

/******************************************************************************
* FUNCTION:       VLQueue_WriteMsg
* DESCRIPTION:    This function queues a variable length message into the
*                 circular buffer it free space permits
* INPUT(S):       ccb: context block to be initialized (also output)
*                 *msg: pointer to message
*                 msgSize: message size
* OUTPUT(S):      none
* RETURN VALUE:   return: equal to zero, success; else error
******************************************************************************/
int32_t vlqueue_push (VLQueue_ccb_t * ccb,
							 const uint8_t *msg,
							 uint32_t msgSize)
{
	int32_t freeSpace = 0;
	int32_t ret = 0;
	uint8_t *buffer;
	uint32_t bufferLength;
	uint32_t writeIndex;
	uint32_t readIndex;
	uint8_t  LengthField[4];
	VLQueue_DefineCriticalVars;

	if (ccb == NULL ||
			ccb->buffer == NULL ||
			msg == NULL ||
			msgSize <= 0)
	{
		return VLQUEUE_ERR_PARAM;
	}

	/* encode length field and store it in a temporary buffer. This does a
	 * check to ensure the message is not too long*/
	ret = encodeLength (LengthField, msgSize);
	if(ret < VLQUEUE_OK)
	{
		return ret;
	}

	/* enter critical section */
	VLQueue_EnterCritical;

	/* cache context block parameters to registers*/
	buffer = ccb->buffer;
	bufferLength = ccb->bufferLength;
	writeIndex = ccb->writeIndex;
	readIndex = ccb->readIndex;

	freeSpace = calcFreeSpace (writeIndex,
		readIndex,
		bufferLength);

	if (freeSpace < (msgSize + ret)) //ret = encoded length parameter
	{
		ret = VLQUEUE_ERR_QUEUE_FULL;
	}
	else
	{
		int32_t i;
		/* copy length field */
		for (i=0; i<ret; i++)
		{
			buffer[writeIndex++] = LengthField[i];

			if (writeIndex >= bufferLength)
			{
				writeIndex = 0;
			}
		}

		/* copy data*/
		while(msgSize--)
		{
			buffer[writeIndex++] = *msg++;

			if (writeIndex >= bufferLength)
			{
				writeIndex = 0;
			}
		}

		/* update write index */
		ccb->writeIndex = writeIndex;
		ccb->msgCnt++;

		ret = VLQUEUE_OK;
	}

	VLQueue_ExitCritical;

	return ret;
}

/******************************************************************************
* FUNCTION:       vlqueue_pop
* DESCRIPTION:    This function reads the message at the front of the buffer,
*                 and removes it
* INPUT(S):       ccb: context block to be initialized (also output)
*                 bufSize: size of output buffer
* OUTPUT(S):      *buf: output buffer
* RETURN VALUE:   return: less than 0, error; else the size of the message read
******************************************************************************/
int32_t vlqueue_pop (VLQueue_ccb_t * ccb,
							uint8_t *buf,
							uint32_t bufSize)
{
	return readInternal (ccb,
								buf,
								bufSize,
								ReadType_PopFront);
}

/******************************************************************************
* FUNCTION:       VLQueue_fsize
* DESCRIPTION:    This function returns the size of the message in the front
*                 of the queue
* INPUT(S):       ccb: context block to be initialized (also output)
* RETURN VALUE:   return: less than zero, error; else size of message read
******************************************************************************/
int32_t vlqueue_fsize (VLQueue_ccb_t * ccb)
{
	int32_t ret = 0;
	uint8_t *buffer;
	uint32_t bufferLength;
	uint32_t readIndex;
	VLQueue_DefineCriticalVars;

	if (ccb == NULL ||
			ccb->buffer == NULL)
	{
		return VLQUEUE_ERR_PARAM;
	}

	/* enter critical section */
	VLQueue_EnterCritical;

	/* cache context block variables to registers*/
	buffer = ccb->buffer;
	bufferLength = ccb->bufferLength;
	readIndex = ccb->readIndex;

	if(ccb->msgCnt == 0)
	{
		/* no messages in queue. return error*/
		ret = VLQUEUE_ERR_QUEUE_EMTPY;
	}
	else
	{
		ret = decodeLength(&readIndex,
						 buffer,
						 bufferLength);
	}

	/* exit critical section */
	VLQueue_ExitCritical;

	/* return negative status or number of bytes read if successful*/
	return ret;
}

/******************************************************************************
* FUNCTION:       vlqueue_btotal
* DESCRIPTION:    This function returns the total size of the buffer
* INPUT(S):       ccb: context block to be initialized (also output)
* OUTPUT(S):      void
* RETURN VALUE:   return: less than 0, error; else buffer free space
******************************************************************************/
int32_t vlqueue_btotal (VLQueue_ccb_t * ccb)
{
	int32_t ret;
	VLQueue_DefineCriticalVars;

	if (ccb == NULL)
	{
		ret = VLQUEUE_ERR_PARAM;
	}
	else
	{
		VLQueue_EnterCritical;
		ret = ccb->bufferLength - 1;
		VLQueue_ExitCritical;
	}

	return ret;
}

/******************************************************************************
* FUNCTION:       vlqueue_bfree
* DESCRIPTION:    This function returns the number of free bytes remaining in
*                 the buffer
* INPUT(S):       ccb: context block to be initialized (also output)
* OUTPUT(S):      void
* RETURN VALUE:   return: less than 0, error; else buffer free space
******************************************************************************/
int32_t vlqueue_bfree (VLQueue_ccb_t * ccb)
{
	int32_t ret;
	VLQueue_DefineCriticalVars;

	if (ccb == NULL)
	{
		ret = VLQUEUE_ERR_PARAM;
	}
	else
	{
		VLQueue_EnterCritical;
		ret = calcFreeSpace (ccb->writeIndex,
									ccb->readIndex,
									ccb->bufferLength);
		VLQueue_ExitCritical;
	}

	return ret;
}
