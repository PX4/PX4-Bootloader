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
 * @file kinetis.h
 *
 * Kinetis definitions.
 */

#if defined(CPU_MK66FN2M0VMD18)
#  include "MK66F18.h"
#endif
#if !defined(CAT)
#if !defined(_CAT)
#define _CAT(a, b) a ## b
#endif
#define CAT(a, b) _CAT(a, b)
#endif

#define KINETIS_UART(l) CAT(UART,l)
#define KINETIS_CLOCK_UART(l) CAT(kCLOCK_Uart,l)

#define KINETIS_PORT(l) CAT(PORT,l)
#define KINETIS_CLOCK_PORT(l) CAT(kCLOCK_Port,l)
#define KINETIS_GPIO(l) CAT(GPIO,l)
#define KINETIS_MASK(b) (1<<(b))

#define SYSTIC_CLKSOURCE_AHB_DIV8  (0 << SysTick_CTRL_CLKSOURCE_Pos)
#define SYSTIC_CLKSOURCE_AHB       (1 << SysTick_CTRL_CLKSOURCE_Pos)

void systick_interrupt_enable(void);
void systick_interrupt_disable(void);
void systick_counter_enable(void);
void systick_counter_disable(void);
void systick_set_reload(uint32_t value);
uint8_t systick_get_countflag(void);
void systick_set_clocksource(uint8_t newsrc);

void flash_unlock();

void sys_tick_handler(void);


void usart_set_baudrate(uint32_t usart, uint32_t baud);
void usart_set_databits(uint32_t usart, int bits);
void usart_set_stopbits(uint32_t usart, int usart_stopbits);
void usart_set_mode(uint32_t usart, int usart_mode);
void usart_set_parity(uint32_t usart, int usart_parity);
void usart_set_flow_control(uint32_t usart, int usart_flowcontrol);
void usart_enable(uint32_t usart);
void usart_disable(uint32_t usart);

uint16_t usart_recv(uint32_t usart);
void usart_send_blocking(uint32_t usart, uint16_t data);

