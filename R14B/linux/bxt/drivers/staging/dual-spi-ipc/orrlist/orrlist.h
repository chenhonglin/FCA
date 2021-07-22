/*
 * priority_list_rr.h
 *
 * Implements a ordered-linked-list with round-robin scheduling of items with
 * identical priority
 *
 *  Created on: Nov 4, 2018
 *      Author: drogala
 */

#ifndef ORRLIST_H_
#define ORRLIST_H_

#ifdef LINUX_KERNEL_INC
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#ifndef NULL
#define NULL ((void*)0)
#endif

struct __orrlist_node_t
{
	struct __orrlist_node_t *previous;
	struct __orrlist_node_t *next;
	struct __orrlist_node_t *adjacent;
	void * data;
	unsigned char orderValue;
	unsigned char adjacentDepth;
	unsigned char adjacentCount;
};
typedef struct __orrlist_node_t orrlist_node_t;

void orrlist_init_node (orrlist_node_t* node, void* data,
								unsigned int orderValue);
void orrlist_insert (orrlist_node_t* head, orrlist_node_t* newnode);
void orrlist_next (orrlist_node_t **node);


#endif /* ORRLIST_H_ */
