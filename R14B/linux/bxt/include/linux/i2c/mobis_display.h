/*
 * FPDLink Serializer driver
 *
 */
#ifndef __LINUX_r1_display_SER_H
#define __LINUX_r1_display_SER_H
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c/mobis_ts.h>

/* The platform data for the FPDLink Serializer driver */
struct mobis_display_platform_data {
    struct mxt_platform_data mxt_platform_data;
};
#endif /* __LINUX_r1_display_SER_H */
