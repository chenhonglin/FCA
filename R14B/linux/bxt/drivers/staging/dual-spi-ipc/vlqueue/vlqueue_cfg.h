/*
 * vlqueue_cfg.h
 *
 *  Created on: Sep 28, 2018
 *      Author: drogala
 */

#ifndef SRC_SERVICES_VLQUEUE_VLQUEUE_CFG_H_
#define SRC_SERVICES_VLQUEUE_VLQUEUE_CFG_H_


/*******************************************************************************
 * EXTERNS AND FUNCTION
 ******************************************************************************/

/*    VLQUEUE_16_BIT_LEN:
 * This option optimizes the context block size by defining
 * all counters as 16 bit instead of 32 bit. In an embedded
 * environment, it is unlikely that we will use a buffer larger than 64K
 *    0 = 32 bit buffer and 30 bit maximum message size
 *    1 = 16 bit buffer and 16 bit maximum message size
 * */
#define VLQUEUE_16_BIT_LEN       (1)


/*    VLQUEUE_ENABLE_USER_PTR:
 * This option allows the user to provide a user pointer argument into the
 * context block which may be required for certain methods of synchronization.
 *    0 = user pointer is not allocated to context block and init function
 *    1 = user pointer is allocated to context block and init function
 * */
#define VLQUEUE_ENABLE_USER_PTR  (0)


/*    VLQueue Critical section user-definable logic:
 * These blocks are meant to defined to allow the user to adapt the module to
 * the specific environment and synchronization mechanism desired. If only a
 * single thread reads from the queue and a single thread writes from the queue
 * and message counter functionality is not enabled, no synchronization is
 * required.
 * */
/* variables created on the stack which are required for sync*/
#define VLQueue_DefineCriticalVars
#if 0
/* enter exclusive area functionality */
#define VLQueue_EnterCritical  { OS_CPU_SR  cpu_sr; OS_ENTER_CRITICAL();
/* exit exclusive area functionality*/
#define VLQueue_ExitCritical  OS_EXIT_CRITICAL();}
#else
/* enter exclusive area functionality */
#define VLQueue_EnterCritical
/* exit exclusive area functionality*/
#define VLQueue_ExitCritical
#endif
#endif /* SRC_SERVICES_VLQUEUE_VLQUEUE_CFG_H_ */
