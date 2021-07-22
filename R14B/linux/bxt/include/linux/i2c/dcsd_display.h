/*
 * FPDLink Serializer driver for DCSD display
 *
 */
#ifndef __LINUX_DCSD_DISPLAY_H
#define __LINUX_DCSD_DISPLAY_H
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c/dcsd_ts.h>

/* The platform data for the FPDLink Serializer driver */
struct dcsd_display_platform_data {
    struct dcsd_ts_platform_data dcsd_ts_pdata;
};
#endif /* __LINUX_DCSD_DISPLAY_H */
