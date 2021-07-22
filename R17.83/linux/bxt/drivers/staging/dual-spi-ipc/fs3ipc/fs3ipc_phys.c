/*****************************************************************
* Project Harman Car Multimedia System
* (c) copyright 2019
* Company Harman/Becker Automotive Systems GmbH
* All rights reserved
* Secrecy Level STRICTLY CONFIDENTIAL
****************************************************************/
/***************************************************************
* @file fs3ipc_phys.c
* @ingroup fs3ipc Component
* @author David Rogala
* Physical Layer source file.
* The Physical layer interacts directly with SpiSlave Driver and FS3IPC Data
* Link Layer, the layers connection are defined by configuration definition,
* and it defines the function to call properly.
****************************************************************/
#include "fs3ipc_cfg.h"
#include "fs3ipc_phys.h"

#ifdef LOG_MODULE
#undef LOG_MODULE
#endif
#define LOG_MODULE "fs3ipc_phys"


#define XFER_TO_COUNT (5)
extern fs3ipc_phys_AppConfigType fs3ipc_phys_appConfigs[];

struct fs3ipc_phys_RtConfig {
	fs3ipc_u8                 initialized;
	fs3ipc_u8                 handle;
	fs3ipc_u8                 commTO;
	fs3ipc_length             rxBufferCount;
	fs3ipc_os_EventMaskType   evXferAllCompleteMask;
	fs3ipc_u8                 rxBuffer[FS3IPC_TRANS_SIZE_MAX];
	fs3ipc_u8                 loggingEnabled;
};

static struct fs3ipc_phys_RtConfig rtConfigs[FS3IPC_PHYS_NUM_OF_INSTANCES];

const static fs3ipc_u8 sentinel[] = {0xa8, 0x2d, 0x39, 0xF2};

/***************************************************************
* Private functions
****************************************************************/

/**
 * @brief aborts ongoing SPI transfers.
 * @param[in/out] cfg   :build time block data pointer
 *
 * @details aborts ongoing SPI transfers.
 */
inline static void abort_transfer(fs3ipc_phys_AppConfigType *cfg)
{
	fs3ipc_u8 spiHandle = 0;
	while (spiHandle < cfg->spiConfigsCount) {
		fs3ipc_phys_ext_SpiAbort(cfg->spiConfigs[spiHandle++].handle);
	}
}

/**
 * @brief if master, signals slave to get ready for transfer.
 * @param[in/out] cfg   :build time block data pointer
 * @param[in/out] rtcfg   :runtime time block data pointer
 *
 * @details if master, signals slave to get ready for transfer. For both,
 *    for both the event flags are set in the correct state. Aborts on timeout
 *    or suspend request
 */
static fs3ipc_StatusType handle_master_req (fs3ipc_phys_AppConfigType *cfg,
		struct fs3ipc_phys_RtConfig *rtcfg)
{
	fs3ipc_os_EventMaskType ev = 0;

	/* nothing to do for a slave transfer. Exit with OK.*/
	if (!(cfg->flags & FS3IPC_PHYS_FLAG_MASTER)) {
		fs3ipc_os_ClearEvent(cfg->osCfg, (cfg->evExtRq|cfg->evXferTimeoutMask));
		return fs3ipc_StatusType_OK;
	}

	fs3ipc_os_GetEvent(cfg->osCfg, &ev);
	while (!(ev & cfg->evExtRq)) {
		fs3ipc_phys_ext_PulseGPIO(rtcfg->handle);
		fs3ipc_os_ArmTimeout(cfg->xferTimeoutTimer, cfg->xferTimeout);
		fs3ipc_os_WaitEvent(cfg->osCfg, (cfg->evExtRq | cfg->evXferTimeoutMask
				|cfg->xferAbortMask));
		fs3ipc_os_GetEvent(cfg->osCfg, &ev);
		fs3ipc_os_StopTimeout(cfg->xferTimeoutTimer);
		fs3ipc_os_ClearEvent(cfg->osCfg, ev & (cfg->evXferTimeoutMask));

		if (ev & cfg->xferAbortMask) {
			LOG_INFO("Suspend Transfer");
			return fs3ipc_StatusType_ErrorInterrupted;
		} else if (ev & (cfg->evXferTimeoutMask)) {
			LOG_WARNING("Master Rq Timeout");
		}
	}

	fs3ipc_os_ClearEvent(cfg->osCfg, ev & (cfg->evExtRq));

	return fs3ipc_StatusType_OK;
}

/**
 * @brief Sets up SPI transfer and in the case of SPI Master, starts transfer.
 * @param[in/out] cfg   :build time block data pointer
 * @param[in/out] rtcfg   :runtime time block data pointer
 * @param[in] outData   :message to be written
 * @param[in] length   :length of transfer
 *
 * @details Sets up SPI transfer and in the case of SPI Master, starts transfer.
 *    Divides message into multiple spi channels.
 */
static fs3ipc_StatusType Handle_Setup_Transfer(fs3ipc_phys_AppConfigType *cfg,
		struct fs3ipc_phys_RtConfig *rtcfg, fs3ipc_cDataPtr outData,
		fs3ipc_length length)
{
	fs3ipc_dataPtr inData = rtcfg->rxBuffer;
	fs3ipc_length transferSize = length / cfg->spiConfigsCount;
	fs3ipc_u8 chanInd = 0;
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;

	rtcfg->rxBufferCount = 0;
	while ((chanInd < cfg->spiConfigsCount) && (stat == fs3ipc_StatusType_OK)){
		fs3ipc_memcpy(inData, sentinel, sizeof(sentinel));
		if (fs3ipc_StatusType_OK == fs3ipc_phys_ext_SetupSpiXfer(
				cfg->spiConfigs[chanInd].handle, (uint8_t*)inData,
				(uint8_t*)outData, transferSize)) {
			inData += transferSize;
			outData += transferSize;
			rtcfg->rxBufferCount += transferSize;
			chanInd++;
		} else {
			LOG_ERROR("SetupSpiXfer %d", chanInd);
			abort_transfer(cfg);
			stat = fs3ipc_StatusType_ErrorGen;
		}
	}
	return stat;
}

/**
 * @brief detects if all channels have started transfers yet.
 * @param[in/out] cfg   :build time block data pointer
 * @param[in/out] rtcfg   :runtime time block data pointer
 * @param[in] length   :length of transfer
 *
 * @details this function detects if all channels have started 
 *    transfers yet. The significance is if a timeout occurs 
 *    after all transfers have started, there has been a DMA 
 *    transfer underrun and the transfer needs to be aborted 
 *    since it will never complete. This is an issue in the 
 *    RH850 only since the SPI driver does direct access and 
 *    each transfer requires 20 Mbps. Todo: Fix the SPI driver 
 *    to use built in fifos.
 */
static fs3ipc_u8 all_transfers_started(fs3ipc_phys_AppConfigType *cfg,
		struct fs3ipc_phys_RtConfig *rtcfg, fs3ipc_length length)
{
	fs3ipc_dataPtr inData = rtcfg->rxBuffer;
	fs3ipc_length transferSize = length / cfg->spiConfigsCount;
	fs3ipc_u8 chanInd = 0;

	while (chanInd < cfg->spiConfigsCount){
		if (fs3ipc_memcmp(inData, sentinel, sizeof(sentinel)) == 0) {
			return FS3IPC_FALSE;
		}
		inData += transferSize;
		chanInd++;
	}
	return FS3IPC_TRUE;
}

/**
 * @brief logs transmit and receive data
 * @param[in/out] rtcfg   :runtime time block data pointer
 * @param[in/out] outData   :transmit data
 * @param[in/out] inData   :received data
 *
 * @details this function logs transmit and receive data.
 */
static void log_data(struct fs3ipc_phys_RtConfig *rtcfg, fs3ipc_u8 *outData,
		fs3ipc_u8 *inData)
{
	if (rtcfg->loggingEnabled == FS3IPC_TRUE) {
		fs3ipc_u16 TxSize = outData[0] + (outData[1] << 8);
		fs3ipc_u16 RxSize = inData[0] + (inData[1] << 8);

		if (TxSize > FS3IPC_TRANS_SIZE_MAX) {
			TxSize = 0;
		}
		if (RxSize > FS3IPC_TRANS_SIZE_MAX) {
			RxSize = 0;
		}

		LOG_DATA (FS3IPC_TRUE, &outData[2], TxSize,
				LOG_DATA_MAX_BYTES, "IPC: Raw Tx L%03d", TxSize);
		LOG_DATA (FS3IPC_TRUE, &inData[2], RxSize,
				LOG_DATA_MAX_BYTES, "IPC: Raw Rx L%03d", RxSize);
	}
}

/**
 * @brief Wait for SPI transfer to conclude
 * @param[in/out] cfg   :build time block data pointer
 * @param[in/out] rtcfg   :runtime time block data pointer
 * @param[in] outData   :message to be written
 * @param[in] length   :length of transfer
 *
 * @details Wait for SPI transfer to conclude. Aborts on timeout or suspend
 *    request. in the case of a slave device, master is signaled that slave is
 *    ready for transfer.
 */
static fs3ipc_StatusType finish_transfer(fs3ipc_phys_AppConfigType *cfg,
		struct fs3ipc_phys_RtConfig *rtcfg, fs3ipc_length length)
{
	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;
	fs3ipc_os_EventMaskType evAll = 0, ev;
	fs3ipc_u8 txfrComplete = FS3IPC_FALSE;
	fs3ipc_os_EventMaskType waitMask = 0;
	fs3ipc_u8 xferStarted = FS3IPC_FALSE;
	fs3ipc_u8 xferTOCount = 0;

	while (!txfrComplete) {
		if (!(cfg->flags & FS3IPC_PHYS_FLAG_MASTER)) {
			fs3ipc_phys_ext_PulseGPIO(rtcfg->handle);
			fs3ipc_os_ArmTimeout(cfg->xferTimeoutTimer, cfg->xferTimeout);
			waitMask = (rtcfg->evXferAllCompleteMask | cfg->evXferTimeoutMask | cfg->xferAbortMask);
		}
		else
		{
			waitMask = (rtcfg->evXferAllCompleteMask);
		}

		for (;;) {
			ev = 0;

			fs3ipc_os_WaitEvent(cfg->osCfg, waitMask);
			fs3ipc_os_GetEvent(cfg->osCfg, &ev);
			fs3ipc_os_ClearEvent(cfg->osCfg,
				ev & (waitMask & ~(cfg->xferAbortMask)));
			evAll |= ev & (waitMask);

			if((evAll & rtcfg->evXferAllCompleteMask) ==
					rtcfg->evXferAllCompleteMask) {
				if (rtcfg->commTO == FS3IPC_TRUE) {
					rtcfg->commTO = FS3IPC_FALSE;
					LOG_WARNING("Xfer TO recovered");
				}
				txfrComplete = FS3IPC_TRUE;
				LOG_INFO("Xfer Complete event");
				break;

			} else if (evAll & cfg->xferAbortMask) {
				LOG_INFO("Suspend Transfer");
				stat = fs3ipc_StatusType_ErrorInterrupted;
				txfrComplete = FS3IPC_TRUE;
				break;

			} else if (evAll & cfg->evXferTimeoutMask) {
				if (xferTOCount < XFER_TO_COUNT) {
					if (++xferTOCount == XFER_TO_COUNT) {
						rtcfg->commTO = FS3IPC_TRUE;
						LOG_WARNING("Xfer TO");
					}
				}

				if (xferStarted) {
					abort_transfer(cfg);
					txfrComplete = FS3IPC_TRUE;
					stat = fs3ipc_StatusType_ErrorGen;
					LOG_ERROR("DMA stuck aborting");
				} else {
					xferStarted = all_transfers_started(cfg, rtcfg, length);
				}

				evAll &= (~cfg->evXferTimeoutMask);
				break;
			} else {
				LOG_INFO("FS3IPC::Entering wait event");
			}
		}
	}

	fs3ipc_os_StopTimeout(cfg->xferTimeoutTimer);
	fs3ipc_os_ClearEvent(cfg->osCfg, 
			(rtcfg->evXferAllCompleteMask | cfg->evXferTimeoutMask));

	return stat;
}

/***************************************************************
* Public functions
****************************************************************/
/**
 * @brief generic callback for handling SPI completion events
 * @param[in] appHndl    Driver handle instance
 * @param[in] portHndl   : SPI port handle that triggers the completion event
 * @warning  if any argument is incorrect, the system will enter to infinitive
 *           loop
 * @details This function is called by call back execution in order to handle
 *          the SPI completion events
 */
void fs3ipc_phys_ProcessSpiCompletion(fs3ipc_handleType appHndl,
												  fs3ipc_phys_spiPortHandle portHndl)
{
	fs3ipc_phys_AppConfigType *appCfg;
	struct fs3ipc_phys_RtConfig * rtcfg;

	if (appHndl >= FS3IPC_PHYS_NUM_OF_INSTANCES) {
		LOG_ERROR("Invalid handle %d", appHndl);
		return;
	}

	appCfg = &fs3ipc_phys_appConfigs[appHndl];
	rtcfg = &rtConfigs[appHndl];

	if (rtcfg->initialized != FS3IPC_TRUE) {
		LOG_ERROR("not initialized");
		return;
	}

	if (portHndl >= appCfg->spiConfigsCount) {
		LOG_ERROR("Invalid spi handle %d", portHndl);
		return;
	}

	fs3ipc_os_SetEvent(appCfg->osCfg,
			appCfg->spiConfigs[portHndl].EvCompleteMask);
}

/**
 * @brief generic callback for handling External Request Input ISR
 * @param[in] appHndl    physical layer handle to process the external request
 * @warning  if the instance is incorrect the system will enter to infinitive
 *           loop
 * @details This function is called by call back execution in order to
 *          handle the request external  event, it is called by ISR interruption
 */
void fs3ipc_phys_ProcessExtReqEvent(fs3ipc_handleType appHndl)
{
	fs3ipc_phys_AppConfigType *appCfg;
	struct fs3ipc_phys_RtConfig * rtcfg;

	if (appHndl >= FS3IPC_PHYS_NUM_OF_INSTANCES) {
		LOG_ERROR("Invalid handle %d", appHndl);
		return;
	}

	appCfg = &fs3ipc_phys_appConfigs[appHndl];
	rtcfg = &rtConfigs[appHndl];

	if (rtcfg->initialized != FS3IPC_TRUE) {
		LOG_ERROR("not initialized");
		return;
	}

	fs3ipc_os_SetEvent(appCfg->osCfg, appCfg->evExtRq);
}

/**
 * @brief Initializes physical layer
 * @param[in] handle   : the driver instance to process
 *
 * @details this function logs transmit and receive data.
 */
fs3ipc_StatusType fs3ipc_phys_Init(fs3ipc_handleType handle) {
	fs3ipc_phys_AppConfigType *cfg;
	struct fs3ipc_phys_RtConfig *rtcfg;

	if (handle >= FS3IPC_PHYS_NUM_OF_INSTANCES) {
		LOG_CRIT("Invalid handle %d", handle);
		return fs3ipc_StatusType_ErrorHandle;
	}

	cfg = &fs3ipc_phys_appConfigs[handle];
	rtcfg = &rtConfigs[handle];

	if (rtcfg->initialized == FS3IPC_TRUE) {
		LOG_ERROR("already initialized");
		return fs3ipc_StatusType_ErrorGen;
	}

	fs3ipc_memset(rtcfg, 0, sizeof(struct fs3ipc_phys_RtConfig));

	/* validate configuration*/
	if (!cfg->osCfg) {
		LOG_CRIT("null osCfg");
		return fs3ipc_StatusType_ErrorCfg;
	}

	if (cfg->evExtRq == 0) {
		LOG_CRIT("evExtRq mask = 0");
		return fs3ipc_StatusType_ErrorCfg;
	}

	if (!cfg->spiConfigs || cfg->spiConfigsCount == 0) {
		LOG_CRIT("spiConfigs %p %d", cfg->spiConfigs, cfg->spiConfigsCount);
		return fs3ipc_StatusType_ErrorCfg;
	} else {
		fs3ipc_u32 i;
		for (i=0; i < cfg->spiConfigsCount; i++) {
			fs3ipc_phys_SpiConfigType *spiConfig = &cfg->spiConfigs[i];
			if (spiConfig->EvCompleteMask == 0) {
				LOG_CRIT("EvCompleteMask mask = 0");
				return fs3ipc_StatusType_ErrorCfg;
			} else {
				rtcfg->evXferAllCompleteMask |= spiConfig->EvCompleteMask;
			}
		}
	}

	if (!cfg->xferTimeoutTimer) {
		LOG_CRIT("xferTimeoutTimer is null");
		return fs3ipc_StatusType_ErrorCfg;
	}

	if (!cfg->xferTimeout) {
		LOG_CRIT("xferTimeout = 0");
		return fs3ipc_StatusType_ErrorCfg;
	}

	if (cfg->evXferTimeoutMask == 0) {
		LOG_CRIT("evXferTimeoutMask mask = 0");
		return fs3ipc_StatusType_ErrorCfg;
	}

	rtcfg->handle = handle;
	rtcfg->loggingEnabled = FS3IPC_FALSE;
	rtcfg->initialized = FS3IPC_TRUE;
	return fs3ipc_StatusType_OK;
}


/**
 * @brief  main handler function for managing physical layer communication
 * @param[in] handle    the driver instance to process
 * @return              none
 * @details This function shall be called after dmaServiceInit and before
 *          SPI_IPC_Init
 */
fs3ipc_StatusType fs3ipc_phys_Transfer(fs3ipc_handleType handle,
		fs3ipc_cDataPtr outData, fs3ipc_length length)
{
	fs3ipc_phys_AppConfigType *cfg;
	struct fs3ipc_phys_RtConfig *rtcfg;

	fs3ipc_StatusType stat = fs3ipc_StatusType_OK;

	if (handle >= FS3IPC_PHYS_NUM_OF_INSTANCES) {
		LOG_CRIT("Invalid handle %d", handle);
		return fs3ipc_StatusType_ErrorHandle;
	}

	cfg = &fs3ipc_phys_appConfigs[handle];
	rtcfg = &rtConfigs[handle];

	if (rtcfg->initialized != FS3IPC_TRUE) {
		LOG_ERROR("not initialized");
		return fs3ipc_StatusType_ErrorGen;
	}

	if (!outData || !length) {
		LOG_ERROR("Invalid param %p %d", outData, length);
		return fs3ipc_StatusType_ErrorGen;
	}

	if (length > FS3IPC_TRANS_SIZE_MAX || length % cfg->spiConfigsCount) {
		LOG_ERROR("Invalid Frame Size %d, spiConfigsCount %d", length, cfg->spiConfigsCount);
		return fs3ipc_StatusType_Invalid_Frame_Size;
	}

	stat = handle_master_req(cfg, rtcfg);

	/* setup transfer*/
	if (stat == fs3ipc_StatusType_OK) {
		stat = Handle_Setup_Transfer(cfg, rtcfg, outData, length);
	}

	if (stat == fs3ipc_StatusType_OK) {
		stat = finish_transfer(cfg, rtcfg, length);

		if (rtcfg->loggingEnabled == FS3IPC_TRUE) {
			log_data(rtcfg, *outData, &rtcfg->rxBuffer[0]);
		}
	}

	if (stat == fs3ipc_StatusType_OK) {
		stat = fs3ipc_phys_ext_Decode(handle, rtcfg->rxBuffer,
				rtcfg->rxBufferCount);
	}

	if (cfg->stats) {
		if (stat == fs3ipc_StatusType_OK) {
			cfg->stats->TransferCount++;
		} else {
			cfg->stats->TransferErrorCount++;
			LOG_DATA(FS3IPC_TRUE, &rtcfg->rxBuffer[0], LOG_DATA_MAX_BYTES,
					LOG_DATA_MAX_BYTES, "IPC: Raw Rx Error");
		}
	}

	return stat;
}

fs3ipc_StatusType fs3ipc_phys_setLogging(fs3ipc_handleType handle, fs3ipc_u8 enabled)
{
	if (handle >= FS3IPC_PHYS_NUM_OF_INSTANCES) {
		LOG_CRIT("Invalid handle %d", handle);
		return fs3ipc_StatusType_ErrorHandle;
	}

	rtConfigs[handle].loggingEnabled = enabled == FS3IPC_TRUE ?
			FS3IPC_TRUE : FS3IPC_FALSE;
	return fs3ipc_StatusType_OK;
}

