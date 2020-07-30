/*
 * Author: Shrisha Krishna <Shrisha.Krishna@harman.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include "ioc-hal.h"

/**
 * @brief This function is used to transmit the data to SPI
 * @retval 0 on success, Negetive error code on failure
 */
int transmit(ipc_plugin_t *handler, char *buffer, size_t size)
{
	int ret; // , i;

	//debug(" %s: %d - %s()", __FILE__, __LINE__, __func__);

	debug(" spi_write  size %d\n", (int)size);

	/*for (i = 0; i < size; i++)
		debug(" writing buf %x\n", buffer[i]); */

	 dumpBuffer(buffer, size, __func__, __LINE__);

	ret = spi_write(handler->spidev, buffer, size);
	if (ret < 0)
		error("spi_write failed with %d\n", ret);

	debug(" spi_write returned %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(transmit);

/**
 * @brief This function receives the data from the SPI
 * @retval 0 on success, Negetive error code on failure
 */
int receive(ipc_plugin_t *handler, char *buffer, size_t size)
{
	int ret; //, i;

	//debug(" %s: %d - %s()", __FILE__, __LINE__, __func__);

	ret = spi_read(handler->spidev, buffer, size);
	if (ret < 0)
		error("spi_read failed with %d\n", ret);

	/* for (i = 0; i < size; i++)
		debug(" received buffer[%d] : <%x>\n", i, buffer[i]); */

	if(ret > 0)
		dumpBuffer(buffer, ret, __func__, __LINE__);

	debug("spi_read returned %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(receive);
