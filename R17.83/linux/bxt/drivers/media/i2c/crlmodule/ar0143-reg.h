
#ifndef __AR0143_REG_H__
#define __AR0143_REG_H__

#include "crlmodule-sensor-ds.h"

#define CMD_REG							0x0040
#define SYSMGR_CMD_STATUS_REG			0xDC0A
#define SYSMGR_CMD_COMP_ID_REG			0xDC0B
#define SYSMGR_CMD_COMP_FAIL_ID_REG		0xDC0C

#define CMD_CCIMGR_GET_LOCK     0x8D00
#define CMD_CCIMGR_LOCK_STATUS  0x8D01
#define CMD_CCIMGR_RELEASE_LOCK 0x8D02
#define CMD_CCIMGR_READ         0x8D05
#define CMD_CCIMGR_WRITE        0x8D06
#define CMD_CCIMGR_STATUS       0x8D08
#define CMD_SYSMGR_SET_STATE	0x8100
#define CMD_SEQ_REFRESH			0x8606
#define CMD_SEQ_REFRESH_STATUS	0x8607
#define CMD_SENSOR_MGR_INITIALIZE_SENSOR	0x8E01
#define PARAMS_POOL_0           0xFC00
#define PARAMS_POOL_1           0xFC02
#define PARAMS_POOL_2           0xFC04
#define PARAMS_POOL_3           0xFC06
#define PARAMS_POOL_4           0xFC08
#define PARAMS_POOL_5           0xFC0A

#define SYS_STATE_ENTER_CONFIG_CHANGE	0x2800
#define DOORBELL_BIT			0x8000


int ap0101_write_regs(struct crl_sensor *sensor,
	const struct crl_register_write_rep *regs, int len);


#endif
