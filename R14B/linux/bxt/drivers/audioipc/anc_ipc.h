/*
 * Author: Kun.You <Kun.You@harman.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef ANC_IPC_H
#define ANC_IPC_H

#include "anc_diag.h"
#define ANC_IOC_MAGIC 'p'
#define ANC_IOCSTART _IO(ANC_IOC_MAGIC,0)
#define ANC_IOCSTOP  _IO(ANC_IOC_MAGIC,1)
#define ANC_IOCSENDMIC  _IO(ANC_IOC_MAGIC,2)
#define ANC_IOCMAXNR 2
#define ANC_IORGETCPD  _IOR(ANC_IOC_MAGIC,0,uint32_t)
#define ANC_IORGETVARID  _IOR(ANC_IOC_MAGIC,1,uint32_t)
#define ANC_IOCSETDANG _IOW(ANC_IOC_MAGIC,0,struct anc_diag)

#define HIFI_MODULE_ID				(4096)
#define AM_PARAMS_DATA_LENGTH			(256)
#define TUNING_PARAMS_DATA_LENGTH		(2048)
#define CPD_DEFAULT				(0xDEAD)
#define VARID_DEFAULT				(0xDEAD)
#define COMMON_XTP_BYTE				(20)
#define DVR_ORDER_NUM				(4)
#define CAN_PARAMS_NUM				(7)

#define SKLMSG_TYPE_COMMON				0
#define SKLMSG_TYPE_CAN					1

#define RESP_SUCCESS					0
#define RESP_FAILURE					1

/* each divergence counter order is 4 byte */
typedef uint8_t (*order_map)[4];

typedef struct {
	/** not important for AC, used by AM */
	uint16_t Session_ID;
	/** identifies the command */
	uint16_t Cmd_ID;
	/** the type of command ( set, get, setget...)*/
	uint16_t Cmd_Type;
	/** in case of AM command it defines the audio zone, in case of decoder command it
	*  defines the decoder instance */
	uint16_t InstID;
	/** the number of used data fields in the data array */
	uint16_t Length;
	/** the array carrying additional command data */
	uint8_t Data[AM_PARAMS_DATA_LENGTH];
}t_AM_Params;

typedef struct
{
	/** not important for AC, used by AM */
	uint16_t Session_ID;
	/** identifies the command */
	uint16_t Cmd_ID;
	/** the type of command ( set, get, setget...)*/
	uint16_t Cmd_Type;
	/** in case of AM command it defines the audio zone, in case of decoder command it
	*  defines the decoder instance */
	uint16_t InstID;
	/** the number of used data fields in the data array */
	uint16_t Length;
	/** the array carrying additional command data */
	uint8_t Data[TUNING_PARAMS_DATA_LENGTH];
}t_Tuning_Params;

#pragma pack(1)
typedef struct canparam
{
	//u8		MesId;			//Message ID
	u16		mask;			//Mask
	u8		enable;			//Enable Disable
	u32		RPM;			//RPM
	u8		winPosDrv;		//window position dirver
	u8		winPosPsg;		//window position passenger
	u8		winPosRL;		//window position RL
	u8		winPosRR;		//window position RR
	u8		DrvDooAjar;		//Drv Door Ajar
	u8		Cyl_Deac;		//Cylinder Deactivation
	u16		CPD;			//CPD
}tCanParam;

struct common_xtpmsg
{
	uint8_t cmd_id;
	uint8_t dev_id;
	uint8_t core_id;
	uint8_t instance_id;
	uint8_t blockid_h;
	uint8_t blockid_l;
	uint8_t subblock_id;
	uint8_t off_bytes[4];	/* little endian */
	union {
		uint8_t read_size;
		uint8_t ret_bytes[COMMON_XTP_BYTE];
	} parm;
};

struct can_xtpmsg
{
	uint8_t cmd_id;			/* xTP Command ID. Should be set to 0x6d for xTP Control Set always */
	uint8_t record_num;		/* number of records (Should always be set to 0x06) */
	struct
	{
		uint8_t ctlid_h;	/* higher byte of Control ID */
		uint8_t ctlid_l;	/* lower byte of Control ID */
		uint8_t param_h;		/* higher byte of Parameter to be set */
		uint8_t param_l;		/* lower byte of Parameter to be set */
		uint8_t dummy1;		/* should be set to 0x00 */
		uint8_t dummy2;		/* should be set to 0x00 */
	} param[CAN_PARAMS_NUM];
};


struct sklmsg
{
	uint8_t len;			/* Message length, this byte should not be sent to xTP */
	uint8_t msg_type;
	union {
		struct common_xtpmsg com_xtpmsg;
		struct can_xtpmsg can_xtpmsg;
	} xtpmsg;
};

#pragma pack()
typedef struct
{
	u8 totalMic;
	u8 activeMic;
	u8 micStatus;
}tMicParam;

#endif

