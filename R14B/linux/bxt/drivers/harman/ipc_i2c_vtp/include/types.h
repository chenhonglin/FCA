/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2015
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level    STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	types.h
 * @author        	Ansa Ahammed <Ansa.Ahammed@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * Global type declarations.
 */

#ifndef TYPES_H
#define TYPES_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include <linux/types.h>	/* size_t */


/*****************************************************************************
 * DEFINES
 *****************************************************************************/

#define HB(w)           (((w)>>8) & 0xFF) /**< macro to get the high byte of the given word w */
#define LB(w)           ((w) & 0xFF)      /**< macro to get the low byte of the given word w */

#define MIN(a, b)       ((a > b) ? b:a) /**< macro to get the min value from a or b */
#define MAX(a, b)       ((a > b) ? a:b) /**< macro to get the max value from a or b */

#ifndef         FALSE
#define FALSE           0 /**< global false declaration */
#endif
#ifndef         TRUE
#define TRUE            1 /**< global true declaration */
#endif

#define EOK             0
#define ERROR           (-1)

#define EXIT_SUCCESS    0
#define EXIT_FAILURE    1

typedef s8              Int8;   /**< Signed 8-bit type */
typedef u8              UInt8;  /**< Unsigned 8-bit type */
typedef s16             Int16;  /**< Signed 16-bit type */
typedef u16             UInt16; /**< Unsigned 16-bit type */
typedef s32             Int32;  /**< Signed 32-bit type */
typedef u32             UInt32; /**< Unsigned 32-bit type */
typedef s64             Int64;  /**< Signed 64-bit type */
typedef u64             UInt64; /**< Usigned 64-bit type */


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/


#endif /* TYPES_H */
