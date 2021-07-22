/*
This file contains NMTP ioctl for SWDL
*/


#ifndef NMTP_SPIDEV_H
#define NMTP_SPIDEV_H
#include <linux/types.h>
#include <linux/ioctl.h>

/* User space versions of kernel symbols for SPI clocking modes,
 * matching <linux/spi/spi.h>
 */

#define SPI_CPHA                0x01
#define SPI_CPOL                0x02

#define SPI_MODE_0              (0|0)
#define SPI_MODE_1              (0|SPI_CPHA)
#define SPI_MODE_2              (SPI_CPOL|0)
#define SPI_MODE_3              (SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH             0x04
#define SPI_LSB_FIRST           0x08
#define SPI_3WIRE               0x10
#define SPI_LOOP                0x20
#define SPI_NO_CS               0x40
#define SPI_READY               0x80
#define SPI_TX_DUAL             0x100
#define SPI_TX_QUAD             0x200
#define SPI_RX_DUAL             0x400
#define SPI_RX_QUAD             0x800

#define NMTP_IOCTL_BASE 	             	0xCC
#define NMTP_SWDL_IOCTL_MAGIC             	NMTP_IOCTL_BASE

struct nmtp_spi_ioc_transfer {
        __u64           tx_buf;
        __u64           rx_buf;

        __u32           len;
        __u32           speed_hz;

        __u16           delay_usecs;
        __u8            bits_per_word;
        __u8            cs_change;
        __u8            tx_nbits;
        __u8            rx_nbits;
        __u16           pad;

        /* If the contents of 'struct spi_ioc_transfer' ever change
         * incompatibly, then the ioctl number (currently 0) must change;
         * ioctls with constant size fields get a bit more in the way of
         * error checking than ones (like this) where that field varies.
         *
         * NOTE: struct layout is the same in 64bit and 32bit userspace.
         */
};

/* NMTP ioctl*/

/* Read / Write of SPI mode (SPI_MODE_0..SPI_MODE_3) (limited to 8 bits) */
#define NMTP_SPI_IOC_RD_MODE                     _IOR(NMTP_SWDL_IOCTL_MAGIC, 4, __u8)
#define NMTP_SPI_IOC_WR_MODE                     _IOW(NMTP_SWDL_IOCTL_MAGIC, 4, __u8)

/* Read / Write SPI bit justification */
#define NMTP_SPI_IOC_RD_LSB_FIRST                _IOR(NMTP_SWDL_IOCTL_MAGIC, 5, __u8)
#define SPI_IOC_WR_LSB_FIRST    	        _IOW(NMTP_SWDL_IOCTL_MAGIC, 5, __u8)

/* Read / Write SPI device word length (1..N) */
#define NMTP_SPI_IOC_RD_BITS_PER_WORD    	_IOR(NMTP_SWDL_IOCTL_MAGIC, 6, __u8)
#define NMTP_SPI_IOC_WR_BITS_PER_WORD    	_IOW(NMTP_SWDL_IOCTL_MAGIC, 6, __u8)

/* Read / Write SPI device default max speed hz */
#define NMTP_SPI_IOC_RD_MAX_SPEED_HZ             _IOR(NMTP_SWDL_IOCTL_MAGIC, 7, __u32)
#define NMTP_SPI_IOC_WR_MAX_SPEED_HZ             _IOW(NMTP_SWDL_IOCTL_MAGIC, 7, __u32)

/* Read / Write of the SPI mode field */
#define NMTP_SPI_IOC_RD_MODE32           	_IOR(NMTP_SWDL_IOCTL_MAGIC, 8, __u32)
#define NMTP_SPI_IOC_WR_MODE32           	_IOW(NMTP_SWDL_IOCTL_MAGIC, 8, __u32)

#define NMTP_SPI_RAW_MODE_START          	_IO(NMTP_SWDL_IOCTL_MAGIC, 9)

#define NMTP_SPI_RAW_MODE_STOP          	_IO(NMTP_SWDL_IOCTL_MAGIC, 10)

#define NMTP_SPI_MSGSIZE(N) \
        ((((N)*(sizeof (struct nmtp_spi_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
                ? ((N)*(sizeof (struct nmtp_spi_ioc_transfer))) : 0)



#define NMTP_SPI_IOC_MESSAGE(N)          	_IOW(NMTP_SWDL_IOCTL_MAGIC, 10, char[NMTP_SPI_MSGSIZE(N)])

#define NMTP_SPI_MODE_MASK           (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
                                | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
                                | SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
                                | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

#endif
