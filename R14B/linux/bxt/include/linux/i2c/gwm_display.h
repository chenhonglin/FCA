/*
 * FPDLink Serializer driver
 *
 */
#ifndef __LINUX_GWM_display_SER_H
#define __LINUX_GWM_display_SER_H
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/platform_data/atmel_mxt_ts.h>
#include <linux/input/synaptics_dsx.h>

#if defined(CONFIG_DS2_DISPLAY) || defined(CONFIG_GWM_DISPLAY)
extern int ser_read_isr(void);
extern int gwm_ser_read_isr(void);
#endif

/* The platform data for the FPDLink Serializer driver */
struct gwm_display_platform_data {
	struct mxt_platform_data mxt_platform_data;
	struct synaptics_dsx_board_data synaptics_platform_data;
};
#endif /* __LINUX_GWM_display_SER_H */
