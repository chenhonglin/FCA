/*
 * FPDLink Serializer driver
 *
 */
#ifndef __LINUX_BJEV_IVI_DISPLAY_H
#define __LINUX_BJEV_IVI_DISPLAY_H
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/input/synaptics_dsx.h>
#include <linux/cyttsp4_core.h>

extern void bjev_ivi_ser_read_isr(void);

/* The platform data for the FPDLink Serializer driver */
struct bjev_ivi_display_platform_data {
	struct synaptics_dsx_board_data synaptics_platform_data;
	struct cyttsp4_platform_data cypress_platform_data;
};
#endif /* __LINUX_BJEV_IVI_DISPLAY_H */
