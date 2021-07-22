/*
 * fs3ipc_cfg.c
 *
 *  Created on: Oct 23, 2018
 *      Author: DRogala
 */

#include "fs3ipc_cfg.h"
#include "fs3ipc_phys.h"
#include "fs3ipc_hdlc.h"
#include "fs3ipc_app.h"

#include "orrlist.h"

/*
 * Constants
 */
#define FS3IPC_PHYSTIMEOUT (50)
#define FS3IPC_WTIMEOUT (50)
#define FS3IPC_WUFTIMEOUT (50)

extern void fs3ipc_phys_i0_Spi_timeout(fs3ipc_handleType handle);

/*
 * OS abstraction layer configuration
 */
//fs3ipc_os_EventType os_AppEvent_0 = SPI_Task;
#define FS3IPC_TIMERS_I0_COUNT (3)
static fs3ipc_os_TimeoutType timer_I0 [FS3IPC_TIMERS_I0_COUNT] = {
		{
				0,
				0,
				(fs3ipc_TimerCallbackType)&fs3ipc_phys_i0_Spi_timeout,
				(void*)FS3IPC_INSTANCE_0,
		},
		{
				0,
				0,
				(fs3ipc_TimerCallbackType)&FS3IPC_Process_Uframe_Timeout,
				(void*)FS3IPC_INSTANCE_0,
		},
		{
				0,
				0,
				(fs3ipc_TimerCallbackType)&FS3IPC_Process_Iframe_Timeout,
				(void*)FS3IPC_INSTANCE_0,
		}
};

const fs3ipc_timerCtxType fs3ipc_timerCtxs [FS3IPC_NUM_OF_INSTANCES]={
		{
			  /* pointer to array of timers*/
				timer_I0,
			  /*total of timer */
				FS3IPC_TIMERS_I0_COUNT
		}
};

static fs3ipc_os_EventType app_0_hdlcEvent;
static fs3ipc_os_ResourceType app_0_hdlcLock;

/*
 * physical layer configuration
 */
#if FS3IPC_PHYS_STATS_ENABLED == 1
static fs3ipc_phys_StatsType phys_stats_i0;
#endif

static fs3ipc_phys_SpiConfigType
    phys_SpiConfig_0[FS3IPC_PHYS_I0_SPI_PORTS_COUNT] = {
        /* Port 0 - FS3IPC_PHYS_I0_SPI0_HANDLE*/
        {
            0,
            EVENT_PHYS_XFER_COMP_0,
        },
        /* Port 1 - FS3IPC_PHYS_I0_SPI1_HANDLE*/
        {
            1,
            EVENT_PHYS_XFER_COMP_1,
        },
};

fs3ipc_phys_AppConfigType
    fs3ipc_phys_appConfigs[FS3IPC_PHYS_NUM_OF_INSTANCES] = {
        /* instance 0 - FS3IPC_PHYS_I0_HANDLE*/
        {
            /* fs3ipc_os_EventType    *osCfg;*/
            &app_0_hdlcEvent,

            /* fs3ipc_os_EventMaskType    evExtRq;*/
            EVENT_EXT_WAKEUP,

            /* fs3ipc_u32                 flags;*/
            (FS3IPC_PHYS_FLAG_MASTER | FS3IPC_PHYS_FLAG_TIMEOUT_ENABLED),

            /* fs3ipc_phys_SpiConfigType  *spiConfigs; */
            phys_SpiConfig_0,

            /* fs3ipc_u8                  spiConfigsCount; */
            FS3IPC_PHYS_I0_SPI_PORTS_COUNT,

            /* fs3ipc_os_TimeoutType        *xferTimeoutTimer; */
            &timer_I0[0],

            /* fs3ipc_os_TickType         xferTimeout; */
            (fs3ipc_os_TickType)FS3IPC_PHYSTIMEOUT,

            /* fs3ipc_os_EventMaskType    evXferTimeoutMask; */
            EVENT_PHYS_TO,

#if FS3IPC_PHYS_STATS_ENABLED == 1
            /* fs3ipc_phys_StatsType* */
            &phys_stats_i0,
#else //FS3IPC_PHYS_STATS_ENABLED != 1
            NULL,
#endif
            /**< External abort event mask*/
            EVENT_SUSPEND,
        }
};

/*
 * Data-Link (HDLC) layer configuration
 */

#if FS3IPC_HDLC_STATS_ENABLED == 1
static fs3ipc_hdlc_StatsType hdlc_stats_i0;
#endif

fs3ipc_hdlc_AppConfigType
	 fs3ipc_hdlc_appConfigs[FS3IPC_HDLC_NUM_OF_INSTANCES] = {
		  /* instance 0 - FS3IPC_HDLC_I0_HANDLE*/
		  {
				/* flag configuration*/
				/*TODO reenable timeout when SOC is ready otherwise the
				 *re-transmission is generated*/
				FS3IPC_HDLC_FLAG_UFRAME_TIMEOUT_ENABLED |
					FS3IPC_HDLC_FLAG_IFRAME_TIMEOUT_ENABLED,

				/* fs3ipc_os_TimeoutType wIfTimer; */
				&timer_I0[2],

				/* fs3ipc_os_TimeoutType wUfTimer; */
				&timer_I0[1],

				/* fs3ipc_u16 wIfTxTimeout; */
				FS3IPC_WTIMEOUT,

				/* fs3ipc_u16 wUfTxTimeout; */
				FS3IPC_WUFTIMEOUT,

				/* fs3ipc_u8 uWindowSiz; */
				WINDOW_SIZE,

				/* fs3ipc_u16 ubufferSiz; */
				FS3IPC_TRANS_SIZE_MAX,

				/* fs3ipc_hdlc_OSCfgType hdlcOSCfg; */
				{
					/* fs3ipc_os_EventType *OS_Task; */
					&app_0_hdlcEvent,

					/* EventMaskType           event_task; */
					EVENT_EXT_WAKEUP,//Ev_Fs3ipc_TxRqSw,
				},

#if FS3IPC_HDLC_STATS_ENABLED == 1
				/* fs3ipc_hdlc_StatsType* */
				&hdlc_stats_i0,
#endif
				&app_0_hdlcLock,

				/* fs3ipc_u32 flags */
				FS3IPC_HDLC_FLAG_TX_FULL_FRAME,
		  },
};


/*
 * Application Layer Configuration
 */
#if FS3IPC_APP_STATS_ENABLED == 1
static fs3ipc_app_StatsType app_0_stats;
#endif

fs3ipc_app_clientChannelLookupType
	 fs3_ipc_clientChannelLookupTable[fs3ipc_clientHandleCount_All];

#define ONOFF_IPC_PRIO (1)
#define RTC_IPC_PRIO (2)
#define NAV_IPC_PRIO (3)

#define TX_BUFFER_SIZE (1024)
#define RX_BUFFER_SIZE (32768)

#define NUM_CHAN_BUF_OBJ (32-2)

#define LTCHANCFG_DEF(name,inst, chan, prio, handle, txSize, rxSize) \
static fs3ipc_u8 app_##inst##_TxChannel##chan##Buf[txSize];\
static fs3ipc_u8 app_##inst##_RxChannel##chan##Buf[rxSize];\
static VLQueue_ccb_t app_##inst##_TxChannel##chan##Q;\
static VLQueue_ccb_t app_##inst##_RxChannel##chan##Q;\
static struct mutex app_##inst##_TxChannel##chan##QLock;\
static struct mutex app_##inst##_RxChannel##chan##QLock;\
static fs3ipc_os_EventType app_##inst##_Channel##chan##TxEvent;\
static fs3ipc_u8 app_##inst##_RxChannel##chan##loggingEnabled;\
static const fs3ipc_app_LTChannelCfgType name = {\
	&app_##inst##_TxChannel##chan##Buf[0],\
	&app_##inst##_RxChannel##chan##Buf[0],\
	prio,\
	chan,\
	txSize,\
	rxSize,\
	&app_##inst##_TxChannel##chan##Q,\
	&app_##inst##_TxChannel##chan##QLock,\
	&app_##inst##_RxChannel##chan##Q,\
	&app_##inst##_RxChannel##chan##QLock,\
	{\
		handle,\
		&app_##inst##_Channel##chan##TxEvent,\
		(1),\
	},\
	&app_##inst##_RxChannel##chan##loggingEnabled,\
}


LTCHANCFG_DEF (app_0_Chan2Cfg, 0, 2, 1, fs3ipc_clientHandle_0_OnOff,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan3Cfg,0,3,1,fs3ipc_clientHandle_0_RTC,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan4Cfg,0,4,1,fs3ipc_clientHandle_0_NAV,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan5Cfg,0,5,1,fs3ipc_clientHandle_0_NAD,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan6Cfg,0,6,1,fs3ipc_clientHandle_0_HW,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan7Cfg,0,7,1,fs3ipc_clientHandle_0_DIAG,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan8Cfg,0,8,1,fs3ipc_clientHandle_0_VI_0,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan9Cfg,0,9,1,fs3ipc_clientHandle_0_VI_1,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan10Cfg,0,10,1,fs3ipc_clientHandle_0_Channel10,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan11Cfg,0,11,1,fs3ipc_clientHandle_0_Channel11,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan12Cfg,0,12,1,fs3ipc_clientHandle_0_Channel12,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan13Cfg,0,13,1,fs3ipc_clientHandle_0_Channel13,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan14Cfg,0,14,1,fs3ipc_clientHandle_0_Channel14,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan15Cfg,0,15,1,fs3ipc_clientHandle_0_Channel15,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan16Cfg,0,16,1,fs3ipc_clientHandle_0_Channel16,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan17Cfg,0,17,1,fs3ipc_clientHandle_0_Channel17,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan18Cfg,0,18,1,fs3ipc_clientHandle_0_Channel18,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan19Cfg,0,19,1,fs3ipc_clientHandle_0_Channel19,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan20Cfg,0,20,1,fs3ipc_clientHandle_0_Channel20,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan21Cfg,0,21,1,fs3ipc_clientHandle_0_Channel21,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan22Cfg,0,22,1,fs3ipc_clientHandle_0_Channel22,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan23Cfg,0,23,1,fs3ipc_clientHandle_0_Channel23,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan24Cfg,0,24,1,fs3ipc_clientHandle_0_Channel24,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan25Cfg,0,25,1,fs3ipc_clientHandle_0_Channel25,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan26Cfg,0,26,1,fs3ipc_clientHandle_0_Channel26,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan27Cfg,0,27,1,fs3ipc_clientHandle_0_Channel27,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan28Cfg,0,28,1,fs3ipc_clientHandle_0_Channel28,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan29Cfg,0,29,1,fs3ipc_clientHandle_0_Channel29,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan30Cfg,0,30,1,fs3ipc_clientHandle_0_Channel30,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);
LTCHANCFG_DEF (app_0_Chan31Cfg,0,31,1,fs3ipc_clientHandle_0_Channel31,
	TX_BUFFER_SIZE,RX_BUFFER_SIZE);


#define app_0_channelCfgs_size (32)

static orrlist_node_t app_0_ChanlistHead;
static orrlist_node_t app_0_nodes[app_0_channelCfgs_size];
static fs3ipc_app_messageBufferType app_0_rxMsg;
static fs3ipc_app_messageBufferType app_0_txMsg;

/* flow control */
static fs3ipc_os_ResourceType app_0_flowControl_Lock;

fs3ipc_flowCtrlPBCfgType app_0_flowCtrlPBCfg;

fs3ipc_flowCtrlChanPBCfgType
	 app_0_flowCtrlChanPBCfg[app_0_channelCfgs_size];

#define LTFLOWCTRLCHANCFG(inst,chan,callback, xoffpercent) {\
	callback,\
	((sizeof(app_##inst##_RxChannel##chan##Buf) * xoffpercent) / 100),\
}

const fs3ipc_flowCtrlChanLTCfgType
	app_0_flowCtrlChanLTCfg[app_0_channelCfgs_size] = {
		{NULL, 0}, /*special channel, no buffer*/
		{NULL, 0}, /*special channel, no buffer*/
		LTFLOWCTRLCHANCFG(0,2,NULL,25),
		LTFLOWCTRLCHANCFG(0,3,NULL,25),
		LTFLOWCTRLCHANCFG(0,4,NULL,25),
		LTFLOWCTRLCHANCFG(0,5,NULL,25),
		LTFLOWCTRLCHANCFG(0,6,NULL,25),
		LTFLOWCTRLCHANCFG(0,7,NULL,25),
		LTFLOWCTRLCHANCFG(0,8,NULL,25),
		LTFLOWCTRLCHANCFG(0,9,NULL,25),
		LTFLOWCTRLCHANCFG(0,10,NULL,25),
		LTFLOWCTRLCHANCFG(0,11,NULL,25),
		LTFLOWCTRLCHANCFG(0,12,NULL,25),
		LTFLOWCTRLCHANCFG(0,13,NULL,25),
		LTFLOWCTRLCHANCFG(0,14,NULL,25),
		LTFLOWCTRLCHANCFG(0,15,NULL,25),
		LTFLOWCTRLCHANCFG(0,16,NULL,25),
		LTFLOWCTRLCHANCFG(0,17,NULL,25),
		LTFLOWCTRLCHANCFG(0,18,NULL,25),
		LTFLOWCTRLCHANCFG(0,19,NULL,25),
		LTFLOWCTRLCHANCFG(0,20,NULL,25),
		LTFLOWCTRLCHANCFG(0,21,NULL,25),
		LTFLOWCTRLCHANCFG(0,22,NULL,25),
		LTFLOWCTRLCHANCFG(0,23,NULL,25),
		LTFLOWCTRLCHANCFG(0,24,NULL,25),
		LTFLOWCTRLCHANCFG(0,25,NULL,25),
		LTFLOWCTRLCHANCFG(0,26,NULL,25),
		LTFLOWCTRLCHANCFG(0,27,NULL,25),
		LTFLOWCTRLCHANCFG(0,28,NULL,25),
		LTFLOWCTRLCHANCFG(0,29,NULL,25),
		LTFLOWCTRLCHANCFG(0,30,NULL,25),
		LTFLOWCTRLCHANCFG(0,31,NULL,25),
};

const fs3ipc_flowCtrlLTCfgType app_0_flowCtrlLTCfg = {
	 //      fs3ipc_u8                           channelCount;
	 app_0_channelCfgs_size,
	 //      fs3ipc_flowCtrlPBCfgType*           pbStConfig;
	 &app_0_flowCtrlPBCfg,
	 //      fs3ipc_flowCtrlChanPBCfgType*       pbStChanConfig;
	 app_0_flowCtrlChanPBCfg,
	 //      const fs3ipc_flowCtrlChanLTCfgType* ltStChanConfig;
	 app_0_flowCtrlChanLTCfg,
	 //      fs3ipc_os_ResourceType*             resLock;
	 &app_0_flowControl_Lock};

fs3ipc_length app_0_rxOverflowIndex;

const fs3ipc_app_LTconfig
	 fs3ipc_app_appConfigs[FS3IPC_APP_NUM_OF_INSTANCES] = {
		  /* instance 0 - FS3IPC_APP_I0_HANDLE*/
		  {
				/* const fs3ipc_app_LTChannelCfgType*   ltChannelCfg;*/
				{
					 NULL,
					 NULL,
					 &app_0_Chan2Cfg,
					 &app_0_Chan3Cfg,
					 &app_0_Chan4Cfg,
					 &app_0_Chan5Cfg,
					 &app_0_Chan6Cfg,
					 &app_0_Chan7Cfg,
					 &app_0_Chan8Cfg,
					 &app_0_Chan9Cfg,
					 &app_0_Chan10Cfg,
					 &app_0_Chan11Cfg,
					 &app_0_Chan12Cfg,
					 &app_0_Chan13Cfg,
					 &app_0_Chan14Cfg,
					 &app_0_Chan15Cfg,
					 &app_0_Chan16Cfg,
					 &app_0_Chan17Cfg,
					 &app_0_Chan18Cfg,
					 &app_0_Chan19Cfg,
					 &app_0_Chan20Cfg,
					 &app_0_Chan21Cfg,
					 &app_0_Chan22Cfg,
					 &app_0_Chan23Cfg,
					 &app_0_Chan24Cfg,
					 &app_0_Chan25Cfg,
					 &app_0_Chan26Cfg,
					 &app_0_Chan27Cfg,
					 &app_0_Chan28Cfg,
					 &app_0_Chan29Cfg,
					 &app_0_Chan30Cfg,
					 &app_0_Chan31Cfg,
				},

				/* fs3ipc_u8                     channelCount; */
				app_0_channelCfgs_size,

				/* orrlist_node_t* orderedChannelListHead; */
				&app_0_ChanlistHead,

				/* orrlist_node_t* orderedChannelNodes;*/
				app_0_nodes,

				/* fs3ipc_os_EventType* phys_txEventOsCfg; */
				//&os_AppEvent_0,
				&app_0_hdlcEvent,

				/* fs3ipc_os_EventMaskType       phys_txEvent; */
				(EVENT_TX_MSG),

				/* fs3ipc_flowCtrlLTCfgType* flowControlCfg; */
				&app_0_flowCtrlLTCfg,

				/* fs3ipc_app_buffer     rxMsg; */
				&app_0_rxMsg,

				/* fs3ipc_app_buffer     txMsg; */
				&app_0_txMsg,

				/* fs3ipc_length* rxMsgBufferOverflowIndex;*/
				&app_0_rxOverflowIndex,

#if FS3IPC_APP_STATS_ENABLED == 1
				/* fs3ipc_app_StatsType* stats; */
				&app_0_stats,
#endif

		  }
};


void fs3ipc_phys_i0_Spi_timeout(fs3ipc_handleType handle)
{
   if (handle < FS3IPC_PHYS_NUM_OF_INSTANCES)
   {
      fs3ipc_phys_AppConfigType *appCfg = &fs3ipc_phys_appConfigs[handle];

      /* post completion event*/
      fs3ipc_os_SetEvent(appCfg->osCfg, appCfg->evXferTimeoutMask);
   }
   else
   {
      /*error*/
   }
}

/* custom functions */
void app_0_CustomInit(void)
{
	int32_t chan = 0;
	int32_t chanCount = fs3ipc_app_appConfigs[0].channelCount;
	//const fs3ipc_app_LTChannelCfgType* LtChanCfg;

	mutex_init (&app_0_flowControl_Lock);
	mutex_init (&app_0_hdlcLock);

	spin_lock_init (&app_0_hdlcEvent.lock);
	init_waitqueue_head (&app_0_hdlcEvent.wq);

	for(chan=0;chan<chanCount;chan++)
	{
		const fs3ipc_app_LTChannelCfgType* chanCfg = fs3ipc_app_appConfigs[0].ltChannelCfg[chan];

		if (chanCfg)
		{
			fs3ipc_os_EventType *ev = chanCfg->clientCfg.osCfg;

			if (ev)
			{
				spin_lock_init (&ev->lock);
				init_waitqueue_head (&ev->wq);
			}
			else
			{
				log_error("app_0_CustomInit: null eventCfg channel %d ", chan);
			}


			/* initialize buffer spin lock*/
			if (chanCfg->TX_QLock)
			{
				mutex_init (chanCfg->TX_QLock);
			}
			else
			{
				log_error("app_0_CustomInit: null TX_QLock channel %d ", chan);
			}


			if (chanCfg->RX_QLock)
			{
				mutex_init (chanCfg->RX_QLock);
			}
			else
			{
				log_error("app_0_CustomInit: null RX_QLock channel %d ", chan);
			}
		}
	}
}
