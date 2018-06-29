/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file kinetis.c
 *
 * Kinetis functions.
 */

#include "kinetis.h"

void systick_interrupt_enable(void)
{
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

void systick_interrupt_disable(void)
{
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

void systick_counter_enable(void)
{
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void systick_counter_disable(void)
{
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void systick_set_reload(uint32_t value)
{
	SysTick->LOAD = (value & SysTick_LOAD_RELOAD_Msk);
}

uint8_t systick_get_countflag(void)
{
	return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0;
}

void systick_set_clocksource(uint8_t newsrc)
{
	SysTick->CTRL = (SysTick->CTRL & ~SysTick_CTRL_CLKSOURCE_Msk) |
			(newsrc & SysTick_CTRL_CLKSOURCE_Msk);
}

void flash_unlock(void)
{
}


void usart_set_baudrate(uint32_t usart, uint32_t baud) {};
void usart_set_databits(uint32_t usart, int bits) {};
void usart_set_stopbits(uint32_t usart, int usart_stopbits) {};
void usart_set_mode(uint32_t usart, int usart_mode) {};
void usart_set_parity(uint32_t usart, int usart_parity) {};
void usart_set_flow_control(uint32_t usart, int usart_flowcontrol) {};
void usart_enable(uint32_t usart) {};
void usart_disable(uint32_t usart) {};

uint16_t usart_recv(uint32_t usart)
{
	return 0;
}

void usart_send_blocking(uint32_t usart, uint16_t data)
{
};
