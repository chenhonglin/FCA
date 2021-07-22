/*****************************************************************
 * Project        Harman/Becker dev-spi
 * (c) copyright  2008-2010
 * Company        Harman/Becker Automotive Systems GmbH
 *                All rights reserved
 *****************************************************************/
/**
 * @file          msgqueue.h
 * @ingroup       resource_manager
 * @author        Michael Hupfer
 * @date          26.04.2010
 * modified       10.01.2012
 *
 * @brief Contains data structures and function prototypes for IPC plugin
 */

#ifndef MSGQUEUE_H_
#define MSGQUEUE_H_

#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/aio.h>
#include <linux/uaccess.h>

#define	MSGQUEUE_ALLOC_STEP		128
#define MSGQUEUE_MAX_SIZE		1024

#define _SLOG_CRITICAL	KERN_CRIT
#define _SLOG_ERROR	KERN_ERR
#define	_SLOG_WARNING	KERN_WARNING

#define my_slogf(sev, verb, name, fmt, args...) \
	do {\
		const char *_n = name ? name : "ipc-plugin";\
		pr_debug("\n%s: " fmt, _n, ## args);\
	} while (0)

#define _info1(fmt, args...)\
		my_slogf(KERN_DEFAULT, 1, NULL, fmt, ## args)

/****************************************************/
/* _TRACE, _EPARAM, _EMEM, EINTERN, _error         */
/****************************************************/
#define _TRACE \
	my_slogf(_SLOG_ERROR, 0, NULL, "Passed file %s, line %d.\n", __FILE__,\
		__LINE__)
#define _EPARAM \
	my_slogf(_SLOG_ERROR, 0, NULL, "Parameter error in file %s, line %d\n",\
	 __FILE__, __LINE__)
#define _EMEM \
	my_slogf(_SLOG_ERROR, 0, NULL, "Out of memory error %s,line %d\n",\
	 __FILE__, __LINE__)
#define _EINTERN \
	my_slogf(_SLOG_ERROR, 0, NULL, "Internal error in file %s, line %d\n",\
	__FILE__, __LINE__)

extern int errno;

typedef void (*push_callback_t)(void *arg, void *data, char __user *msg);

/************************************************/
/* struct msgqueue_t                            */
/************************************************/
typedef struct msgqueue_element_hdr_s {
	unsigned long			size;    /**< size of the element */
	struct msgqueue_element_hdr_s	*prev;    /**< pointer to predecessing element */
	struct msgqueue_element_hdr_s	*next;    /**< pointer to subsequent element */
} msgqueue_element_hdr_t;

/************************************************/
/* struct msgqueue_t*/
/************************************************/
/** @brief Holds a threadsafe message FIFO. FIFO uis impelemted by a array of void pointers
 * */
typedef struct msgqueue_s {
	msgqueue_element_hdr_t	*first;		/*< pointer to first element */
	msgqueue_element_hdr_t	*last;		/**< pointer to last element */
	msgqueue_element_hdr_t	*free;          /**< pointer to freed elements */
	unsigned	count;		        /**< number of element in the queue */
	unsigned	free_count; /* number of elements in the freed elements list */
#ifdef USE_QNX
	pthread_mutex_t	lock;		         /**< mutex for thread safety */
	pthread_cond_t cond_enq;	      /**< condvar for signaling new data */
#else
	struct  mutex	lock;		         /**< mutex for thread safety */
	struct  completion	comp_enq;
	struct  completion	comp_nodata_cb;
	unsigned int	ast_rlen;   /*read length for user set from data_cb*/
#endif
	push_callback_t	enq_callback;	/**< Called when new data is pushed*/
	void	*arg;		         /** <argument for callback */
	char	name[16];          /**< name of the queue */
	char	__user *msg;
} msgqueue_t;

#ifdef DEBUG_LOCKS
#define  msgqueue_lock(q)  {\
	_info1("%s, %d, %s: lock q: %p",  __func__, __LINE__, __FILE__, q);\
	mutex_lock(&(q)->lock);\
}

#define  msgqueue_unlock(q)  {\
	_info1("%s,%d,%s: unlock q: %p",  __func__, __LINE__, __FILE__, q);\
	mutex_unlock(&(q)->lock);\
}
#else
#define  msgqueue_lock(q)  mutex_lock(&(q)->lock)
#define  msgqueue_unlock(q) mutex_unlock(&(q)->lock)
#endif


/**
 * @brief Frees the message queue.
 * @param q - message queue structure
 * @returns EOK / error number
 */
int msgqueue_destroy(msgqueue_t *q);


/**
 * @brief Initializes the message queue.
 * @param q - message queue structure
 * @param name - name of the queue, needed for debug messages
 * @returns EOK / error number
 */
int msgqueue_init(msgqueue_t *q, char *name);

/**
 * @brief Enqueues an element into the queue. The element must be allocated using
 * msgqueue_alloc. The element is enqueued after the element "after". If "after"
 * is null then "data" is enqueued at the first position.
 * @param q - message queue structure
 * @param data - element to be enqueued
 * @param after - predecessor element
 * @returns EOK / error number
 */
int msgqueue_enqueue_nolock(msgqueue_t *q, void *data, void *after);


/**
 * @brief Same as msgqueue_enqueue_nolock() but with locking.
 */
int msgqueue_enqueue(msgqueue_t *q, void *data, void *after);

/**
 * @brief Dequeues an element out of the queue and store the pointer in "data". If
 * "element" is null, the last element is dequeued, otherwise "element"
 * @param q - message queue structure
 * @param data - pointer where the data pointer can be stored
 * @param element - element that should be dequeued
 * @returns EOK / error number
 */
int msgqueue_dequeue_nolock(msgqueue_t *q, void **data, void *element);


/**
 * @brief Same as msgqueue_dequeue_nolock but with locking.
 */
int msgqueue_dequeue(msgqueue_t *q, void **data, void *element);

/**
 * @brief Allocates an element for use by the message queues. Size + sizeof(msgqueue_element_hdr_t)
 * bytes are allocated.
 * @param q - message queue structure
 * @param size - element size
 * @returns NULL / pointer to the element
 */
void *msgqueue_alloc(msgqueue_t *q, unsigned size);

/**
 * @brief Same as msgqueue_free_nolock() but with locking
 */
int msgqueue_free(msgqueue_t *q, void *element);

/**
 * @brief Frees an element. This element must be allcoated using msgqueue_alloc,
 * otherwise one probably causes segmentation faults.
 * @param q - message queue structure
 * @param element - element that should be freed
 * @returns EOK / error number
 */
int msgqueue_free_nolock(msgqueue_t *q, void *element);


/**
 * @brief Calculates the number of free entries in the queue.
 * @param q - message queue structure
 * @returns no of free items / -1 (errno is set)
 */
int msgqueue_freespace(msgqueue_t *q);

/**
 * @brief Calculates the number of entries in the queue.
 * @param q - message queue structure
 * @returns no of items / -1 (errno is set)
 */
int msgqueue_used(msgqueue_t *q);

/**
 * @brief Waits for data in the queue. If there's data in the queue,
 * the function returns immediately, otherwise waits for an enqueue event.
 * @param q - message queue structure
 * @param cb - callback to call when data is pushed
 * @param arg - argument for callback
 * @returns no of elements / EOK = wait / -1 for error (errno is set)
 */
int msgqueue_wait_for_data_nolock(msgqueue_t *q, push_callback_t cb, void *arg,
					char __user *msg);

/**
 * @brief Same as msgqueue_wait_for_data_nolock() but with queue locking.
 */
int msgqueue_wait_for_data(msgqueue_t *q, push_callback_t cb, void *arg, char
				__user *msg);


/**
 * @brief Waits for a enqueue event.
 * @param q - message queue structure
 * @param cb - callback to call when data is pushed
 * @param arg - argument for callback
 * @returns no of elements / EOK = wait / -1 for error (errno is set)
 */
int msgqueue_wait_for_enqueue_nolock(msgqueue_t *q);


/**
 * @brief Unblocks a thread waiting for a push event
 * @param q - message queue structure
 * @returns EOK / error number
 */
int msgqueue_unblock_enqueue(msgqueue_t *q);

/**
 * @brief Returns the successor element of "element" or the first element of the queue, if element is NULL
 * @param q - message queue structure
 * @param element - pointer to queue entry
 */
void *msgqueue_getnext_nolock(msgqueue_t *q, void *element);


/**
 * @brief Returns the predecessor element of "element" or the last element of the queue, if element is NULL
 * @param q - message queue structure
 * @param element - pointer to queue entry
 */
void *msgqueue_getprev_nolock(msgqueue_t *q, void *element);

#endif /*MSGQUEUE_H_*/
