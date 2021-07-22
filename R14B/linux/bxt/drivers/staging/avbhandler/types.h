/*****************************************************************************
 * Project         HARMAN AVB Stack(2.0)
 *
 * (c) copyright   2018
 * Company         Harman International
 *                 All rights reserved
 *****************************************************************************/
/**
 * @file          types.h
 * @ingroup       utilities
 * @author        Thorben Link
 *
 * Global type declarations.
 */

#ifndef __TYPES_H
#define __TYPES_H


/*---------------------------------------------------------------------
 * INCLUDES
 *--------------------------------------------------------------------*/
#include <linux/types.h>


/*---------------------------------------------------------------------
 * DEFINES
 *--------------------------------------------------------------------*/
#define HB(w)        (((w)>>8) & 0xFF) /**< macro to get the high byte of the given word w */
#define LB(w)        ((w) & 0xFF)      /**< macro to get the low byte of the given word w */

#define MIN(a, b)    ((a > b) ? b:a) /**< macro to get the min value from a or b */
#define MAX(a, b)    ((a > b) ? a:b) /**< macro to get the max value from a or b */

#ifndef		FALSE
   #define	FALSE	0 /**< global false declaration */
#endif
#ifndef		TRUE
   #define	TRUE	1 /**< global true declaration */
#endif


/*---------------------------------------------------------------------
 * TYPE DEFINITIONS
 *--------------------------------------------------------------------*/
typedef int8_t   Int8;   /**< Signed 8-bit type */
typedef uint8_t  UInt8;  /**< Unsigned 8-bit type */
typedef int16_t  Int16;  /**< Signed 16-bit type */
typedef uint16_t UInt16; /**< Unsigned 16-bit type */
typedef int32_t  Int32;  /**< Signed 32-bit type */
typedef uint32_t UInt32; /**< Unsigned 32-bit type */
typedef int64_t  Int64;  /**< Signed 64-bit type */
typedef uint64_t UInt64; /**< Unsigned 64-bit type */
#if !defined(__cplusplus) && !defined(bool)
#define  bool  unsigned char
#endif
typedef char             Char;

#endif /* __TYPES_H */
