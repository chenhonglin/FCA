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

#include "msgqueue.h"
#include "ioc-ipc.h"

#define __HEADER_CAST(p) (p == NULL ? NULL : \
(msgqueue_element_hdr_t *)((void *)p - sizeof(msgqueue_element_hdr_t)))

#define __DATA_CAST(p) (p == NULL ? NULL : \
(void *)((void *)p + sizeof(msgqueue_element_hdr_t)))

/**
 * @brief Checks if the hdr pointed to by "hdr" is in the queue. If not
 * NULL is returned, otherwise the pointer to the element.
   Caution: pass the pointer to the
 * header-struct, not to data!
 * @param q - message queue structure
 * @param element - pointer to check
 * @returns NULL/pointer to element
 */
static msgqueue_element_hdr_t *msgqueue_search(msgqueue_t *q,
	msgqueue_element_hdr_t *hdr);


/****************************************************/
/* msgqueue_destroy                                 */
/****************************************************/
int msgqueue_destroy(msgqueue_t *q)
{
	int                        result = EOK;
	msgqueue_element_hdr_t     *hdr = NULL, *next = NULL;

	if (q) {
                msgqueue_lock(q);
			for (hdr = q->first; hdr; hdr = next) {
				next = hdr->next;
				__free(hdr);
			}

			for (hdr = q->free; hdr; hdr = next) {
				next = hdr->next;
				__free(hdr);
			}
                msgqueue_unlock(q);
#ifdef USE_QNX
			pthread_cond_destroy(&q->cond_enq);
			pthread_mutex_destroy(&q->lock);
#else
			mutex_destroy(&q->lock);
#endif
	} else {
		_EPARAM;
		result = -EINVAL;
	}

	return result;
}
EXPORT_SYMBOL(msgqueue_destroy);

/****************************************************/
/* msgqueue_init                                     */
/****************************************************/
int   msgqueue_init(msgqueue_t *q, char *name)
{
	int      result = EOK;

	if (q) {
		memset(q, 0, sizeof(*q));
		mutex_init(&q->lock);
		init_completion(&q->comp_enq);
		init_completion(&q->comp_nodata_cb);

		if (name && name[0])
			strncpy(q->name, name, sizeof(q->name) - 1);
	} else {
		_EPARAM;
		result = -EINVAL;
	}

	return result;
}
EXPORT_SYMBOL(msgqueue_init);

/****************************************************/
/* msgqueue_search                                  */
/****************************************************/
static msgqueue_element_hdr_t *msgqueue_search(msgqueue_t *q, msgqueue_element_hdr_t *hdr)
{
	/*lint -e613 */
	msgqueue_element_hdr_t     *ptr = NULL, *result = NULL;

	for (ptr = q->first; ptr; ptr = ptr->next) {
		if (ptr == hdr) {
			result = ptr;
			break;
		}
	}

	return result;
	/*lint +e613 */

}


/****************************************************/
/* msgqueue_getnext_nolock                          */
/****************************************************/
void *msgqueue_getnext_nolock(msgqueue_t *q, void *element)
{
	if (q) {
		if (element)
			return __DATA_CAST(__HEADER_CAST(element)->next);
		else
			return __DATA_CAST(q->first);
	} else
		_EPARAM;

	return NULL;
}


/****************************************************/
/* msgqueue_getprev_nolock                          */
/****************************************************/
void *msgqueue_getprev_nolock(msgqueue_t *q, void *element)
{
	if (q) {
		if (element)
			return __DATA_CAST(__HEADER_CAST(element)->prev);
		else
			return __DATA_CAST(q->last);
	} else
		_EPARAM;

	return NULL;
}


/****************************************************/
/* msgqueue_enqueue_nolock                          */
/****************************************************/
int msgqueue_enqueue_nolock(msgqueue_t *q, void *data, void *after)
{
	int                        result = EOK;
	msgqueue_element_hdr_t     *hdr = NULL, *after_hdr = NULL;

	if (q && data) {

		{
			if (q->count < MSGQUEUE_MAX_SIZE) {
				hdr = __HEADER_CAST(data);
				hdr->next = hdr->prev = NULL;

				if (after) {
					after_hdr = __HEADER_CAST(after);

					if (msgqueue_search(q, after_hdr)) {
						hdr->next = after_hdr->next;
						hdr->prev = after_hdr;
						after_hdr->next = hdr;

					if (after_hdr == q->last)
							q->last = hdr;

						q->count++;
					} else {
						result = -EINVAL;
					}
				} else {
					if (q->first)
						q->first->prev = hdr;

					hdr->next = q->first;
					q->first = hdr;

					if (q->last == NULL)
						q->last = hdr;

					q->count++;
				}


				if (q->enq_callback) {
					q->enq_callback(q->arg, data, q->msg);
					q->enq_callback = NULL;
					q->arg = NULL;
					q->msg = NULL;
				} else {
					complete(&q->comp_enq);
				}
			} else
				result = -ENOMEM;
		}
	} else {
		_EPARAM;
		result = -EINVAL;
	}

	return result;
}

/****************************************************/
/* msgqueue_enqueue                                 */
/****************************************************/
int msgqueue_enqueue(msgqueue_t *q, void *data, void *after)
{
	int                        result = EOK;

	if (q && data) {
		msgqueue_lock(q);
		{
			result = msgqueue_enqueue_nolock(q, data, after);

			msgqueue_unlock(q);
		}
	} else {
		_EPARAM;
		result = -EINVAL;
	}

	return result;
}
EXPORT_SYMBOL(msgqueue_enqueue);

/****************************************************/
/* msgqueue_dequeue_nolock                          */
/****************************************************/
int msgqueue_dequeue_nolock(msgqueue_t *q, void **data, void *element)
{

	int result = EOK;
	msgqueue_element_hdr_t *el_hdr = NULL;


	if (q && data) {
		*data = NULL;

		if (element) {
			el_hdr = __HEADER_CAST(element);

			if (msgqueue_search(q, el_hdr)) {
				*data = element;
				q->count--;

				if (el_hdr->prev)
					el_hdr->prev->next = el_hdr->next;
				else
					q->first = el_hdr->next;

				if (el_hdr->next)
					el_hdr->next->prev = el_hdr->prev;
				else
					q->last = el_hdr->prev;
			} else {
				result = -EINVAL;
			}
		} else {
			if (q->last) {
				*data = __DATA_CAST(q->last);
				q->count--;

				if (q->last->prev) {
					q->last = q->last->prev;
					q->last->next = NULL;
				} else {
					q->first = q->last = NULL;
				}
			} else
				result = -ENODATA;
		}
	} else {
		_EPARAM;
		result = -EINVAL;
	}

	return result;
}



/****************************************************/
/* msgqueue_dequeue                                 */
/****************************************************/
int msgqueue_dequeue(msgqueue_t *q, void **data, void *element)
{
	int                        result = EOK;

	if (q && data) {
		msgqueue_lock(q);

		{
			result = msgqueue_dequeue_nolock(q, data, element);

			msgqueue_unlock(q);
		}
	} else {
		_EPARAM;
		result = -EINVAL;
	}

	return result;
}
EXPORT_SYMBOL(msgqueue_dequeue);

/****************************************************/
/* msgqueue_alloc                                   */
/****************************************************/
void *msgqueue_alloc(msgqueue_t *q, unsigned size)
{
	void                          *result = NULL;
	msgqueue_element_hdr_t        *hdr = NULL;

	if (q && size > 0) {
		if (q->free) {
			msgqueue_lock(q);

			for (hdr = q->free; hdr; hdr = hdr->next) {
				if (hdr->size >= size) {
					result = __DATA_CAST(hdr);
					memset(result, 0, hdr->size);
					q->free_count--;

					if (hdr->prev)
						hdr->prev->next = hdr->next;
					else
						q->free = hdr->next;

					if (hdr->next)
						hdr->next->prev = hdr->prev;

					if (hdr->next == NULL && hdr->prev ==
									NULL)
						q->free = NULL;

					break;
				}
			}

			msgqueue_unlock(q);
		}

		if (result == NULL) {
			hdr = __calloc(1, size +
				sizeof(msgqueue_element_hdr_t), 2);

			if (hdr) {
				hdr->size = size;
				result = __DATA_CAST(hdr);
			} else {
				_EMEM;
			}
		}
	} else {
		_EPARAM;
	}

	return result;
}
EXPORT_SYMBOL(msgqueue_alloc);

/****************************************************/
/* msgqueue_free_nolock                             */
/****************************************************/
int msgqueue_free_nolock(msgqueue_t *q, void *element)
{
	int                        result = EOK;
	msgqueue_element_hdr_t     *hdr = NULL, *next = NULL;
	unsigned                   u = 0;

	if (q && element) {
		hdr = __HEADER_CAST(element);

		if (q->free)
			q->free->prev = hdr;

		hdr->next = q->free;
		hdr->prev = NULL;
		q->free = hdr;
		q->free_count++;

		if (q->count && q->free_count > 2 * q->count) {
			for (u = 0, hdr = q->free; u < q->count && hdr; u++,
							hdr = next) {
				next = hdr->next;

				if (next)
					next->prev = NULL;

				__free(hdr);
			}

			q->free = hdr;
			q->free_count -= q->count;
		}
	} else {
		_EPARAM;
		result = -EINVAL;
	}

	return result;
}



/****************************************************/
/* msgqueue_free                                    */
/****************************************************/
int msgqueue_free(msgqueue_t *q, void *element)
{
	int                        result = EOK;

	if (q && element) {
		msgqueue_lock(q);

		{
			result = msgqueue_free_nolock(q, element);

			msgqueue_unlock(q);
		}
	} else {
		_EPARAM;
		result = -EINVAL;
	}

	return result;
}
EXPORT_SYMBOL(msgqueue_free);

/***************************************************/
/* msgqueue_freespace                              */
/***************************************************/
int   msgqueue_freespace(msgqueue_t *q)
{
	int      result = -1;

	if (q)
		result = MSGQUEUE_MAX_SIZE - q->count;
	 else
		_EPARAM;

	return result;
}


/***************************************************/
/* msgqueue_used                                   */
/***************************************************/
int   msgqueue_used(msgqueue_t *q)
{
	int      result = -1;
	
	if (q ) {
		msgqueue_lock(q);
		result = q->count;
		msgqueue_unlock(q);
	}
	 else
		_EPARAM;

	return result;
}



/***************************************************/
/* msgqueue_wait_for_data_nolock                   */
/***************************************************/

int msgqueue_wait_for_data_nolock(msgqueue_t *q, push_callback_t cb, void *arg,
							char __user *msg)
{
	int      result = EOK;

	if (q) {
		if (q->count > 0) {
			result = q->count;
		} else {
			if (cb) {
				q->enq_callback = cb;
				q->arg = arg;
				q->msg = msg;
			} else {
				wait_for_completion(&q->comp_enq);
				_info1("%s: wait over: q(%p)", __func__, q);
				result = q->count;
			}

		}

	} else {
		_EPARAM;
		result = -1;
	}

	return result;
}


/***************************************************/
/* msgqueue_wait_for_data                          */
/***************************************************/
int msgqueue_wait_for_data(msgqueue_t *q, push_callback_t cb, void *arg, char __user *msg)
{
	int                        result = EOK;

	if (q) {
		{
			result = msgqueue_wait_for_data_nolock(q, cb, arg, msg);

		}
	} else {
		_EPARAM;
		result = -1;
	}

	return result;
}


/***************************************************/
/* msgqueue_wait_for_enqueue_nolock                */
/***************************************************/
int msgqueue_wait_for_enqueue_nolock(msgqueue_t *q)
{
	int      result = EOK;

	if (q) {
		_info1("%s: no data - waiting for data(queue:%p)!", __func__,
								q);
		wait_for_completion(&q->comp_enq);
		_info1("%s: wait over: q(%p)",  __func__, q);
		result = q->count;
	} else {
		_EPARAM;
		result = -1;
	}

	return result;
}



/****************************************************/
/* msgqueue_unblock_enqueue                         */
/****************************************************/
int   msgqueue_unblock_enqueue(msgqueue_t *q)
{
	int      result = EOK;

	if (q) {

		{
			q->enq_callback = NULL;
			q->arg = NULL;
			complete(&q->comp_enq);
		}
	} else {
		_EPARAM;
		result = -EINVAL;
	}

	return result;
}
