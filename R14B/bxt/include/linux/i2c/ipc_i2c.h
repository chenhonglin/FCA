/*
 * IPC-I2C For DU driver
 *
 */
#ifndef __LINUX_IPC_I2C_H
#define __LINUX_IPC_I2C_H
#include <linux/types.h>
#include <linux/i2c.h>

#define	IVI_SREQ		"ipc_ivi_sreq"
#define	VTP_SREQ		"ipc_vtp_sreq"

/* The platform data for the I2C IPC driver */
struct du_ipc_board_data {
	int gpio;
	int bus;
	char *gpio_label;
};

#endif /* __LINUX_IPC_I2C_H */

