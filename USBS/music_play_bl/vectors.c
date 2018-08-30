/******************** (C) COPYRIGHT 2018 merafour ********************
* Author             : 冷月追风@merafour.blog.163.com
* Version            : V1.0.0
* Date               : 30/8/2018
* Description        : vectors.
********************************************************************************
* merafour.blog.163.com
* merafour@163.com
* github.com/Merafour
*******************************************************************************/
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>,
 * Copyright (C) 2012 chrysn <chrysn@fsfe.org>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>

/* load optional platform dependent initialization routines */
#include "../libopencm3/lib/dispatch/vector_chipset.c"
/* load the weak symbols for IRQ_HANDLERS */
#include "../libopencm3/lib/dispatch/vector_nvic.c"

#include "bl.h"
#include "hw_config.h"
#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/scb.h>
# include <libopencm3/stm32/timer.h>

/* Symbols exported by the linker script(s): */
extern unsigned _data_loadaddr, _data, _edata, _ebss, _stack;
typedef void (*funcp_t) (void);
extern funcp_t __preinit_array_start, __preinit_array_end;
extern funcp_t __init_array_start, __init_array_end;
extern funcp_t __fini_array_start, __fini_array_end;

void main(void);
void blocking_handler(void);
void null_handler(void);

void reset_handler_rb1(void);

__attribute__ ((section(".vectors_m")))
vector_table_t vector_table = {
	.initial_sp_value = &_stack,
	.reset = reset_handler_rb1,
	.nmi = nmi_handler,
	.hard_fault = hard_fault_handler,

/* Those are defined only on CM3 or CM4 */
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
	.memory_manage_fault = mem_manage_handler,
	.bus_fault = bus_fault_handler,
	.usage_fault = usage_fault_handler,
	.debug_monitor = debug_monitor_handler,
#endif

	.sv_call = sv_call_handler,
	.pend_sv = pend_sv_handler,
	.systick = sys_tick_handler,
	.irq = {
		IRQ_HANDLERS
	}
};

void __attribute__ ((section(".reset_rb3"))) reset_handler_init(void)
{
	volatile unsigned *src, *dest;
	funcp_t *fp;

	for (src = &_data_loadaddr, dest = &_data;
		dest < &_edata;
		src++, dest++) {
		*dest = *src;
	}

	while (dest < &_ebss) {
		*dest++ = 0;
	}

	/* Ensure 8-byte alignment of stack pointer on interrupts */
	/* Enabled by default on most Cortex-M parts, but not M3 r1 */
	SCB_CCR |= SCB_CCR_STKALIGN;

	/* might be provided by platform specific vector.c */
	pre_main();

	/* Constructors. */
	for (fp = &__preinit_array_start; fp < &__preinit_array_end; fp++) {
		(*fp)();
	}
	for (fp = &__init_array_start; fp < &__init_array_end; fp++) {
		(*fp)();
	}
}
void __attribute__ ((section(".reset_rb3"))) reset_handler_rb3(void)
{
	funcp_t *fp;
	reset_handler_init();
	/* Call the application's entry point. */
	main();
	/* Destructors. */
	for (fp = &__fini_array_start; fp < &__fini_array_end; fp++) {
		(*fp)();
	}
}
extern void main_rb2_init(void);
extern void bootloader_b2(unsigned timeout);
void __attribute__ ((section(".reset_rb2"))) reset_handler_rb2(void)
{
	funcp_t *fp;
	unsigned timeout = 10000;
	reset_handler_init();

	/* Call the application's entry point. */
	//main();
	main_rb2_init();
	while (1) {
		//uart_cout((uint8_t *)"Hello\r\n", 7);
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader_b2(timeout); // 10s
		/* look to see if we can boot the app */
		jump_to_app();
		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
	/* Destructors. */
	for (fp = &__fini_array_start; fp < &__fini_array_end; fp++) {
		(*fp)();
	}
}

extern void board_init(void);
extern void clock_init(void);
extern void encoding(uint32_t sign[8], volatile uint32_t uid[3]);
/*
 * STM32F103 一个山区可能是 1K或 2K, 为方便后期代码移植,这里使用 2K
*/
#define  CODE_SIZE    2048   // 2K
//uint32_t code_vectors[CODE_SIZE/4];    // 2K  , 中断向量表
//uint32_t code_encryption[CODE_SIZE/4]; // 2K  , 加密代码
#define FLASH_START_ADDRESS    0x08000000
const uint16_t enc_offsetB= 0x3800;
const uint16_t enc_offsetW= 0x3800/4;
void erase_code(const uint32_t _vectors[CODE_SIZE/4], const uint32_t _encryption[CODE_SIZE/4], const uint32_t len)
{
	uint32_t i=0;
	//const uint32_t len = sizeof(code_vectors)/4; // 2K
	uint32_t	address = board_info.fw_size;
	// erase all sectors
	flash_unlock();
	if(0x400 == FLASH_SECTOR_SIZE)  // page = 1K
	{
		flash_erase_page(FLASH_START_ADDRESS + (0 * FLASH_SECTOR_SIZE));   // [0-0x400]
		flash_erase_page(FLASH_START_ADDRESS + (1 * FLASH_SECTOR_SIZE));   // [0x400-0x800]
		flash_erase_page(FLASH_START_ADDRESS + (14 * FLASH_SECTOR_SIZE));  // [0x3800-0x3C00]
		flash_erase_page(FLASH_START_ADDRESS + (15 * FLASH_SECTOR_SIZE));  // [0x3C00-0x4000]
	}
	else // page = 2K
	{
		flash_erase_page(FLASH_START_ADDRESS + (0 * FLASH_SECTOR_SIZE));   // [0-0x800]
		flash_erase_page(FLASH_START_ADDRESS + (14 * FLASH_SECTOR_SIZE));  // [0x3800-0x4000]
	}
	address=0;  // code_vectors
	for (i = 0; i < len; i++) {
		// program the word
		flash_program_word(address + FLASH_START_ADDRESS, _vectors[i]);
		address += 4;
	}
	address=0x3800;  // code_encryption
	for (i = 0; i < len; i++) {
		// program the word
		flash_program_word(address + FLASH_START_ADDRESS, _encryption[i]);
		address += 4;
	}
#if 1
	/* just for paranoia's sake */
	flash_lock();
	scb_reset_system();
#else
	jump_to_app();
#endif
}
extern struct boardinfo board_info;
void __attribute__ ((section(".reset_rb1"))) UserLicense(uint8_t flash[], volatile uint32_t uid[3])
{
#define User_UID0    (0xc3037ea1U)
#define User_UID1    (0xb6a6831aU)
#define User_UID2    (0x2f6fb587U)
 uint32_t *sign = (uint32_t *)(flash);
 //uint32_t License = 0x5cb36a04; //sign[40];
	uint32_t License = 0x4ba34568U; //sign[13];
 //printf("License:0x%08X\r\n", License);
	//sign[8] = 0; // 擦除 License
	//sign[13] = 0; // IO config
		//sign += 3;
		//sign += 0x0C80; // 0x3200/4
#if 0
		sign[0] = ((~uid[0])&uid[2]);
		sign[1] = (uid[1]&(~uid[2]));
		sign[2] = uid[0]^uid[1]^License;
		sign[3] = uid[0]^(sign[2]|(~License));
#else
    sign[0] = (uid[2]&uid[1])|((~uid[0])&uid[2]);
    sign[1] = (uid[2]&uid[0])|(uid[1]&(~uid[0]));
    sign[2] = User_UID0^License^uid[2];
		sign[3] = User_UID1^(License|(~uid[2]));
#endif
		sign[4] = (uid[0]+User_UID1+User_UID2) | License;
		//License = License + User_UID1&User_UID2 + sign[3];
		License = License + (User_UID1&User_UID2) + sign[3];
		License = ((License<<16)&0xFFFF0000) | ((License>>16)&0x0000FFFF);
		
		sign[5] = (sign[0]+License)|(User_UID1&(~uid[2]));
		sign[6] = (sign[1]+sign[4])|(User_UID2&(License));
		sign[7] = (sign[2]+sign[5])|(User_UID0&(~uid[2]));
		//match_flag = 1;
		//match_flag.skip1 = BOOT_COERCE_SKIP;
}
uint32_t code_vectors[CODE_SIZE/4];    // 2K  , 中断向量表
uint32_t code_encryption[CODE_SIZE/4]; // 2K  , 加密代码

extern void read_uid(uint32_t uid[3]);
void __attribute__ ((section(".reset_rb1"))) main_rb1(void)
{
	uint32_t	address = 0;
	const uint32_t len = sizeof(code_vectors)/4; // 16K
	uint32_t i=0;
	const uint32_t* src_ver= (const uint32_t*)FLASH_START_ADDRESS;
	const uint32_t* src_enc= (const uint32_t*)(FLASH_START_ADDRESS+enc_offsetB);
	
	//volatile uint32_t* _mtext=NULL;
	uint32_t uid[4]={0};
	
	/* Enable the FPU before we hit any FP instructions */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

	/* do board-specific initialisation */
	board_init();
	/* configure the clock for bootloader activity */
	clock_init();
	
	// copy code
	for(i=0; i<(sizeof(code_vectors)/4); i++)
	{
		code_vectors[i] = src_ver[i];
	}
	//code_vectors[1] = (uint32_t)reset_handler_rb2; // 更改中断向量,下一次上电不再执行这部分代码
	code_vectors[1] = (uint32_t)reset_handler_rb2;
	
	for(i=0; i<len; i++)
	{
		code_encryption[i] = src_enc[i];
	}
	//check
	read_uid(uid);  // 读取产品 ID号
	// erase code
	i = 0; // [0x3800-0x3C00] 没有代码
	address = (uid[0]+uid[1]+uid[2])%0x400+0x400;
	
	for(; i<len; i++)
	{
		code_encryption[i] = src_ver[address++]^0x68fe2433U; // 覆盖原有代码
	}
	// copy code
	i = (uint32_t)&reset_handler_rb2;
	//i=0x08003d00;
#if 0
	i = (i&0x0000FFFC)-enc_offsetB;
	i = (i)>>2 ; // 拷贝 B2阶段代码
#else
	i = ((i&0x0000FFFC)-enc_offsetB)>>2 ; // 拷贝 B2阶段代码
#endif
	for(; i<len; i++)
	{
		code_encryption[i] = src_enc[i];
	}
	i = 1024/4; // 15K, erase code
	code_encryption[i+0]=0x000001FF;
	code_encryption[i+1]=0x000001FF;
	encoding(&code_encryption[i+2], uid);      // bootloader 加密
	UserLicense((uint8_t*)&code_encryption[i+2+8], uid);  // user 加密
	memcpy(&code_encryption[i+2+8+8], &board_info, sizeof(board_info));  // bootloader 版本
	erase_code(code_vectors, code_encryption, CODE_SIZE/4); // 写入操作,擦除原有代码

	// 执行系统复位
	scb_reset_system(); 
	
}
void __attribute__ ((section(".reset_eb2"))) encrypt_b2(const uint8_t encrypt[], const uint8_t encrypt_len)
{
	uint32_t	address = 0;
	const uint32_t len = sizeof(code_vectors)/4; // 16K
	uint32_t i=0;
	const uint32_t* src_ver= (const uint32_t*)0x08000000;
	const uint32_t* src_enc= (const uint32_t*)0x08003800;
	
	//volatile uint32_t* _mtext=NULL;
	uint32_t uid[3]={0};
	
	// copy code
	for(i=0; i<(sizeof(code_vectors)/4); i++)
	{
		code_vectors[i] = src_ver[i];
	}
	code_vectors[1] = (uint32_t)reset_handler_rb3;
	
	for(i=0; i<len; i++)
	{
		code_encryption[i] = src_enc[i];
	}
	//check
	read_uid(uid);  // 读取产品 ID号
	// erase code
	i = (uint32_t)&reset_handler_rb2;
	i = ((i&0x0000FFFC)-enc_offsetB)>>2 ;  // 擦除 B2阶段代码
	address = (uid[0]+uid[1]+uid[2])%0x400+0x400;
	for(; i<len; i++)
	{
		code_encryption[i] = src_ver[address++]^0x68fe2433U;
	}
	i = 1024/4; // 15K, erase code
	code_encryption[i+0]=0x000004FF;
	code_encryption[i+1]=0x000004FF;
	memcpy(&code_encryption[i+2+8+64], encrypt, encrypt_len); // 保存加密信息
	
	erase_code(code_vectors, code_encryption, CODE_SIZE/4); // 写入操作,擦除原有代码
	
	// 执行系统复位
	scb_reset_system(); 
}

void __attribute__ ((section(".reset_rb1"))) reset_handler_rb1(void)
{
	funcp_t *fp;
	reset_handler_init();
	/* Call the application's entry point. */
	main_rb1();
	/* Destructors. */
	for (fp = &__fini_array_start; fp < &__fini_array_end; fp++) {
		(*fp)();
	}
}

void blocking_handler(void)
{
	while (1);
}

void null_handler(void)
{
	/* Do nothing. */
}

#pragma weak nmi_handler = null_handler
#pragma weak hard_fault_handler = blocking_handler
#pragma weak sv_call_handler = null_handler
#pragma weak pend_sv_handler = null_handler
#pragma weak sys_tick_handler = null_handler

/* Those are defined only on CM3 or CM4 */
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
#pragma weak mem_manage_handler = blocking_handler
#pragma weak bus_fault_handler = blocking_handler
#pragma weak usage_fault_handler = blocking_handler
#pragma weak debug_monitor_handler = null_handler
#endif

