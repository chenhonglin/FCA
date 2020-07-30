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

#ifndef _IOCHAL_H
#define _IOCHAL_H

#include "ioc-ipc.h"

/**
 * @brief This function is used to transmit the data to SPI
 * @retval 0 on success, Negetive error code on failure
 */
int transmit(ipc_plugin_t *handler, char *buffer, size_t size);

/**
 * @brief This function receives the data from the SPI
 * @retval 0 on success, Negetive error code on failure
 */
int receive(ipc_plugin_t *handler, char *buffer, size_t size);


void dumpBuffer(unsigned char* pBuffer, unsigned int len, const char *func, int lineNum);
#endif
