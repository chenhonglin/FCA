/*****************************************************************
* Project Harman Car Multimedia System
* (c) copyright 2019
* Company Harman/Becker Automotive Systems GmbH
* All rights reserved
* Secrecy Level STRICTLY CONFIDENTIAL
****************************************************************/
/***************************************************************
* @file fs3ipc_phys.h
* @ingroup fs3ipc Component
* @author David Rogala
* Physical Layer header file
****************************************************************/

#ifndef FS3IPC_PHYS_H_
#define FS3IPC_PHYS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fs3ipc_cfg.h"

/* todo move me*/
typedef uint8_t fs3ipc_phys_spiPortHandle;

/* flags options*/
/**
 * @details this flag enables the SPI Master configuration
 */
#define FS3IPC_PHYS_FLAG_MASTER (1u << 0)
/**
 * @details this flag disables the SPI Slave configuration
 */
#define FS3IPC_PHYS_FLAG_SLAVE (0u << 0)

/**
 * @details this flag enables the timeout expiration time
 */
#define FS3IPC_PHYS_FLAG_TIMEOUT_ENABLED (1u << 1)

/**
 * @details this flag enables the timeout expiration time
 */
#define FS3IPC_PHYS_FLAG_TIMEOUT_DISABLED (0u << 1)

/* configuration block for SPI driver*/

/**
 * Physical layer structure defines the handle and the events to process for
 * this layer
 */
typedef struct
{
	/**< Driver handle for individual SPI channel*/
	fs3ipc_phys_SpiSlaveHandleType handle;

	/**< transfer complete event mask for individual SPI channel*/
	fs3ipc_os_EventMaskType EvCompleteMask;
} fs3ipc_phys_SpiConfigType;

/**
 * The fs3ipc_phys_StatsType counts number of transactions
 */
typedef struct
{
	/**< counts the correct number of transactions*/
	fs3ipc_u32 TransferCount;

	/**< counts the incorrect number of transactions*/
	fs3ipc_u32 TransferErrorCount;
} fs3ipc_phys_StatsType;

/** Physical Application configuration */
typedef struct
{
	/* General*/
	/**< OS abstraction configuration block*/
	fs3ipc_os_EventType *osCfg;

	/**< External Rx Request Event Mask*/
	fs3ipc_os_EventMaskType evExtRq;

	/**< configuration flags, enables disables the timeout and master slave
	 * flags*/
	fs3ipc_u32 flags;

	/**< SpiSlave driver*/
	/**< SPI driver configuration block array*/
	fs3ipc_phys_SpiConfigType *spiConfigs;

	/**< number of SPI ports used for transfer*/
	fs3ipc_u8 spiConfigsCount;

	/**< SPI transfer timeout timer*/
	fs3ipc_os_TimeoutType *xferTimeoutTimer;

	/**< SPI transfer timeout timer period*/
	fs3ipc_os_TickType xferTimeout;

	/**< SPI transfer timeout mask*/
	fs3ipc_os_EventMaskType evXferTimeoutMask;

	/**< statistics */
	fs3ipc_phys_StatsType *stats;

	/**< External abort event mask*/
	fs3ipc_os_EventMaskType xferAbortMask;
} fs3ipc_phys_AppConfigType;

void fs3ipc_phys_ProcessSpiCompletion(fs3ipc_handleType appHndl,
												  fs3ipc_phys_spiPortHandle portHndl);

void fs3ipc_phys_ProcessExtReqEvent(fs3ipc_handleType appHndl);

fs3ipc_StatusType fs3ipc_phys_Transfer(fs3ipc_handleType handle, fs3ipc_cDataPtr outData,
		fs3ipc_length length);

fs3ipc_StatusType fs3ipc_phys_Init(fs3ipc_handleType handle);

fs3ipc_StatusType fs3ipc_phys_setLogging(fs3ipc_handleType handle, fs3ipc_u8 enabled);

#if FS3IPC_PHYS_STATS_ENABLED == 1
const fs3ipc_phys_StatsType *fs3ipc_phys_GetStats(fs3ipc_handleType Hndl);

fs3ipc_StatusType fs3ipc_phys_ClearStats(fs3ipc_handleType Hndl);

#endif

#ifdef __cplusplus
}
#endif

#endif /* FS3IPC_FS3IPC_PHYS_H_ */
