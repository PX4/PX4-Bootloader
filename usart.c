/************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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

# include <libopencm3/stm32/rcc.h>
# include <libopencm3/stm32/gpio.h>

#include <libopencm3/stm32/usart.h>

#if !defined(USART_SR)
#define USART_SR USART_ISR
#endif
#include "bl.h"
#include "uart.h"

uint32_t usart;

void
uart_cinit(void *config)
{
	usart = (uint32_t)config;

	/* board is expected to do pin and clock setup */

	/* do usart setup */
	//USART_CR1(usart) |= (1 << 15);	/* because libopencm3 doesn't know the OVER8 bit */
	usart_set_baudrate(usart, 115200);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, USART_STOPBITS_1);
	usart_set_mode(usart, USART_MODE_TX_RX);
	usart_set_parity(usart, USART_PARITY_NONE);
	usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);

	/* and enable */
	usart_enable(usart);


#if 0
	usart_send_blocking(usart, 'B');
	usart_send_blocking(usart, 'B');
	usart_send_blocking(usart, 'B');
	usart_send_blocking(usart, 'B');

	while (true) {
		int c;
		c = usart_recv_blocking(usart);
		usart_send_blocking(usart, c);
	}

#endif
}

void
uart_cfini(void)
{
	usart_disable(usart);
}

int
uart_cin(void)
{
	int c = -1;

	if (USART_SR(usart) & USART_SR_RXNE) {
		c = usart_recv(usart);
	}

	return c;
}

void
uart_cout(uint8_t *buf, unsigned len)
{
	while (len--) {
		usart_send_blocking(usart, *buf++);
	}
}
