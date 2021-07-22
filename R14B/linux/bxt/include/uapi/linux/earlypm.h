#ifndef _LINUX_UAPI_EARLYPM_H
#define _LINUX_UAPI_EARLYPM_H

#include <linux/types.h>

/*  Vehicle configuration type. */
enum vehicle_camera_cfg_type {
	CAMCFG_UNKNOWN      = 0,
	CAMCFG_FCA_POWERNET = 1,
	CAMCFG_FCA_ATL_MID  = 2,
	CAMCFG_FCA_ATL_HIGH = 3
};

/***
 * Generic data structure for RVC related vehicle config.  cast into one of the
 * other subtypes to use. defining this structure as a place-holder to expand
 * to PowerNet/VehCfg based configuration data. 
 */
struct vehicle_camera_cfg_storage {
	uint32_t     type;
	uint32_t     version;
	char         data[256];
};

/***
 * Data structure for FCA Atlantis CAN specfic configuration data 
 */
struct vehicle_camera_cfg_atlmi {
	uint32_t     type;
	uint32_t     version;
	uint8_t	     rear_view_camera_proxi;
	uint8_t	     gear_box_type_proxi;
	uint8_t	     vehicle_line_configuration;
};


struct shift_lever_info {
	/* Data structure version */
	uint32_t	version;

	/*  1 = PNet, 2 = Atl mid, 3= Atl High */
	uint8_t		can_arch;
 
	/*  Reverse gear info that must be fwd'ed to LVDS. */
	uint8_t		shift_position;
	uint8_t		reverse_gear;

	uint8_t		neutral_sw_sts;  //(PNET Mid only, for now)
	uint8_t		awd_status;
};

struct ignition_status {
	/* interface/message version */
	uint32_t	version;

	/*  1 = PNet, 2 = Atl mid, 3= Atl High */
	uint8_t		can_arch; 

	/* On atlantis: From CAN signal CmdIgnSts */
	uint8_t	        ign_sts; 

	/* ignition fail status (Atlantis only) */
	uint8_t         ign_fail_sts; 
};

struct branding_info {
	uint8_t        version;
	uint8_t        vehicle_brand;
	uint8_t        splash_type;
	uint8_t        special_package;
	uint8_t        audio_brand;
	uint8_t        sxm;
	uint8_t        srt_present;
};

struct ecall_assist_sos_state {
	uint8_t        version;
	uint8_t        call_status;

};

struct lvds_wall_ascm_alert {
	uint8_t        version;
	uint8_t        wallAlert_InnerLeft;
	uint8_t        wallAlert_InnerRight;
	uint8_t        wallAlert_OuterLeft;
	uint8_t        wallAlert_OuterRight;
	uint8_t        ascm_Stat;
};

struct lvds_amb_sens_day_lgt_md {
	uint8_t        version;
   union {
      uint8_t    amb_sens_val;
      uint8_t    lgt_mode_disp;
   };
};

#endif
