/************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2010 libopencm3 project
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * LICENSE NOTE FOR EXTERNAL LIBOPENCM3 LIBRARY:
 *
 *   The PX4 development team considers libopencm3 to be
 *   still GPL, not LGPL licensed, as it is unclear if
 *   each and every author agreed to the LGPS -> GPL change.
 *
 ***********************************************************************/

/*
 * USART interface for the bootloader.
 */

#include "hw_config.h"
#include <stdint.h>

#include "bl.h"
#include "uart.h"

uint32_t usart;
#if defined(INTERFACE_USART) && INTERFACE_USART > 0
# error "UART driver for K66 is incomplete - please add it if needed!"
#endif

void
uart_cinit(void *config)
{
}

void
uart_cfini(void)
{
}

int
uart_cin(void)
{
	int c = -1;
	return c;
}

void
uart_cout(uint8_t *buf, unsigned len)
{
	while (len--) {
		buf++;
	}
}
