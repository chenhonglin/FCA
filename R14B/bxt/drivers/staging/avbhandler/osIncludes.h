/*****************************************************************************
 * Project         HARMAN AVB Stack 2.0
 *
 * (c) copyright   2018
 * Company         Harman International
 *                 All rights reserved
 *                 STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          osIncludes.h
 * @author        Nagaprasad A R
 *
 * Linux OS specific includes.
 */

#ifndef __OSINCLUDES_H
#define __OSINCLUDES_H

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/kthread.h> //kthread_run()
#include <linux/dma-mapping.h> //dma_alloc_coherent
#include <asm/uaccess.h>
#include <linux/slab.h>   /* kmalloc() */

//Interrupts related includes
#include <linux/delay.h>

//Rx iface - File ops
#include <linux/fs.h>

#define EXIT_SUCCESS	0
#define EXIT_FAILURE	1

#endif //#ifndef __OSINCLUDES_H

