/*
 * priority_list_rr.c
 *
 * Implements a ordered-linked-list with round-robin scheduling of items with
 * identical priority
 *
 *  Created on: Nov 4, 2018
 *      Author: drogala
 */
#include "orrlist.h"

void orrlist_init_node (orrlist_node_t* node, void* data,
								unsigned int orderValue)
{
	if (node != NULL)
	{
		node->orderValue = orderValue;
		node->data = data;
	}
}

void orrlist_insert (orrlist_node_t* head, orrlist_node_t* newnode)
{
	orrlist_node_t* previous = head;
	orrlist_node_t* current  = head->next;
	unsigned int orderValue = newnode->orderValue;

	previous = head;
	current = head->next;

	/* Initialize head node to work with the general algorithm*/
	if(previous->adjacent == NULL)
	{
		previous->adjacent = previous;
		previous->adjacentDepth = 1;
	}

	/* traverse linked list until we find the first element of a higher order
	 * value (lower priority). We will insert the node before this */
	while (current != NULL && orderValue > current->orderValue)
	{
		previous = current;
		current = current->next;
	}

	/* if an existing element in the list shares the same value as the node that
	 * we are inserting, we must insert the node adjacently. the adjacent linked
	 * list is managed in a round robin manner*/
	if (current != NULL && current->orderValue == orderValue)
	{
		orrlist_node_t* root = current;

		/* calculate the new depth of the adjacent list*/
		uint8_t newDepth = root->adjacentDepth + 1;

		/* traverse through adjacent nodes until we get to the end*/
		while(root != current->adjacent)
		{
			/* update new length*/
			current->adjacentDepth = newDepth;
			current = current->adjacent;
		}

		/* insert node*/
		current->adjacentDepth = newDepth;
		current->adjacent = newnode;
		newnode->adjacent = root;
		newnode->adjacentDepth = newDepth;
	}
	else
	{
		/* insert new node*/
		previous->next = newnode;

		if(current != NULL)
		{
			current->previous = newnode;
		}

		/* initialize nodes traversal elements*/
		newnode->previous = previous;
		newnode->next = current;
		newnode->adjacent = newnode;
		newnode->adjacentDepth = 1;
	}
}

void orrlist_remove(orrlist_node_t **node)
{
	/* todo: */
}

void orrlist_next (orrlist_node_t **node)
{
	orrlist_node_t *previous;
	orrlist_node_t *current;
	orrlist_node_t *adjacent;
	orrlist_node_t *next;

	if ((node != NULL) && (*node != NULL))
	{
		current  = *node;
		previous = current->previous;
		adjacent = current->adjacent;
		next     = current->next;

		if (current->adjacentDepth <= 1 || adjacent->adjacentCount >= current->adjacentDepth)
		{
			/* point previous to next adjacent block*/
			if (previous)
			{
				previous->next = adjacent;
			}

			adjacent->previous = current->previous;
			if (next)
			{
				adjacent->next = next->adjacent;
			}
			else
			{
				adjacent->next = NULL;
			}

			/* skip to next priority*/
			previous = current->adjacent;
			current = next;
			if(current)
			{
				adjacent = current->adjacent;
				next = current->next;

				current->adjacentCount = 0;

				/* update new adjacent*/
				if(current->adjacentDepth > 1)
				{
					adjacent->adjacentCount = 1;
					adjacent->previous = previous;
					adjacent->next = next;
				}

				if(next)
				{
					next->previous = adjacent;
				}
			}
			else
			{
				adjacent = NULL;
				next = NULL;
			}
		}
		else
		{
			/* skip to next node*/
			current = adjacent;
			adjacent = current->adjacent;

			/* copy adjacent params*/
			adjacent->adjacentCount = current->adjacentCount + 1;
			adjacent->previous      = current->previous;
			adjacent->next          = current->next;

			if (previous)
			{
				previous->next = adjacent;
			}

			if (next)
			{
				next->previous = adjacent;
			}
		}

		/* assign next node to input pointer*/
		*node = current;
	}
}
