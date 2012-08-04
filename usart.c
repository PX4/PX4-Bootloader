/*
 * USART interface for the bootloader.
 */

#ifdef STM32F4
# include <libopencm3/stm32/f4/rcc.h>
# include <libopencm3/stm32/f4/gpio.h>
#endif

#include <libopencm3/stm32/usart.h>

#include "bl.h"

uint32_t usart;

void
cinit(void *config)
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
cfini(void)
{
	usart_disable(usart);
}

int
cin(void)
{
	int c = -1;

	if (USART_SR(usart) & USART_SR_RXNE)
		c = usart_recv(usart);
	return c;
}

void
cout(uint8_t *buf, unsigned len)
{
	while (len--)
		usart_send_blocking(usart, *buf++);
}
