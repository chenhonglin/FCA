/*****************************************************************************
 * Project     		HARMAN connected Car Systems
 *
 * c) copyright		2019
 * Company        	Harman International Industries, Incorporated
 *               	All rights reserved
 * Secrecy Level        STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          	hal_serial.h
 * @author        	Prasad Lavande <Prasad.Lavande@harman.com>
 * @ingroup       	Kernel and Drivers
 *
 * This header file is used to handle HAL functionalities
 */

#ifndef HAL_SERIAL_H
#define	HAL_SERIAL_H


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

Int32 halInit(struct i2c_client *client);
Int32 halWrite(UInt8* buff, Int32 len);
void halCleanup(void);


#endif /* HAL_SERIAL_H */
