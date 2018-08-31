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

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>

/* load optional platform dependent initialization routines */
#include "../libopencm3/lib/dispatch/vector_chipset.c"
/* load the weak symbols for IRQ_HANDLERS */
#include "../libopencm3/lib/dispatch/vector_nvic.c"

#include "usbs_bl.h"
#include <stdlib.h>
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
extern unsigned _mtext_b1, _mtext_b2, _mtext;
typedef void (*funcp_t) (void);
extern funcp_t __preinit_array_start, __preinit_array_end;
extern funcp_t __init_array_start, __init_array_end;
extern funcp_t __fini_array_start, __fini_array_end;

void main(void);
void blocking_handler(void);
void null_handler(void);
void reset_handler_m(void);
void reset_handler_e2(void);
void reset_handler_e1(void);

#define HAND_MASK    (0x57d839D8U)
//__attribute__ ((section(".1vectors_h"))) vector_table_entry_t reset_h2=reset_handler_e2;
//attribute__ ((section(".1vectors_h"))) vector_table_entry_t reset_h3=reset_handler_m;
static const uint32_t reset_h2=(uint32_t)reset_handler_e2;
static const uint32_t reset_h3=(uint32_t)reset_handler_m;
__attribute__ ((section(".vectors_m")))
vector_table_t vector_table = {
	.initial_sp_value = &_stack,
	.reset = reset_handler_e1,
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
#if 0
//void __attribute__ ((weak, naked)) reset_handler(void)
void __attribute__ ((section(".reset_m"))) reset_handler_m(void)
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

	/* Call the application's entry point. */
	main();

	/* Destructors. */
	for (fp = &__fini_array_start; fp < &__fini_array_end; fp++) {
		(*fp)();
	}

}
#endif
void __attribute__ ((section(".reset_m"))) reset_handler_init(void)
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
void __attribute__ ((section(".reset_m"))) reset_handler_m(void)
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

extern void board_init(void);
extern void clock_init(void);
extern void encoding(uint32_t sign[8], volatile uint32_t uid[3]);
uint32_t code[16*1024/4]; // 16K
#if 0
void
flash_func_write_word(uint32_t address, uint32_t word)
{
	flash_program_word(address + APP_LOAD_ADDRESS, word);
}
#endif
//void __attribute__ ((section(".flash_r"))) erase_code(void)
void erase_code(void)
{
	uint32_t i=0;
	const uint32_t len = 16*1024/4; // 16K
	uint32_t	address = board_info.fw_size;
	// erase all sectors
	flash_unlock();
	//flash_func_erase_code(0);
	flash_erase_sector(0, FLASH_CR_PROGRAM_X32);
	address=0;
	for (i = 0; i < len; i++) {
		// program the word
		//flash_func_write_word(address, code[i]);
		flash_program_word(address + 0x08000000, code[i]);
		address += 4;
	}
	//main();
#if 1

	/* just for paranoia's sake */
	flash_lock();
	scb_reset_system();
#else
	jump_to_app();
#endif
}
extern struct boardinfo board_info;
void __attribute__ ((section(".reset_e"))) UserLicense(uint8_t flash[], volatile uint32_t uid[3])
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
#include <string.h>
void __attribute__ ((section(".reset_b2"))) main_e2(void);
void __attribute__ ((section(".reset_e"))) main_e(void)
{
	/* Call the application's entry point. */
	//main();
	//bool try_boot = true;			/* try booting before we drop to the bootloader */
	//unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

	uint32_t	address = board_info.fw_size;
	const uint32_t len = 16*1024/4; // 16K
	uint32_t i=0;
	const uint32_t* src= (const uint32_t*)0x08000000;
	
	volatile uint32_t* _mtext=NULL;
	//uint32_t passwd[8]={0};
	uint32_t uid[3]={0};
	//int match=0;
	//unsigned int led=0;
	
	/* Enable the FPU before we hit any FP instructions */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

#if 0//defined(BOARD_POWER_PIN_OUT)

	/* Here we check for the app setting the POWER_DOWN_RTC_SIGNATURE
	 * in this case, we reset the signature and wait to die
	 */
	if (board_get_rtc_signature() == POWER_DOWN_RTC_SIGNATURE) {
		board_set_rtc_signature(0);

		while (1);
	}

#endif

	/* do board-specific initialisation */
	board_init();

	/* configure the clock for bootloader activity */
	clock_init();
	
	// copy code
	for(i=0; i<len; i++)
	{
		code[i] = src[i];
	}
	//check
	_mtext = (volatile uint32_t*)(0x1FFF0000);
	_mtext += (0x7A10/4);
	uid[0] = _mtext[0];
	uid[1] = _mtext[1];
	uid[2] = _mtext[2];
	// erase code
	//i = 15*1024/4; // 15K, erase code
	i = 0x3600/4; // 13.5K, erase code
	address = (uid[0]+uid[1]+uid[2])%0x400;
	for(; i<len; i++)
	{
		code[i] = code[address++]^0x68fe2433U;
	}
	// copy code
	address = (uint32_t)reset_handler_e2;
	i = (uint32_t)main_e2;
	if(i>address) i=address;
	//i = i&0x0000FFFC ; // 对齐
	//i = (0x3D00)>>2 ; 
	i = (i&0x0000FFFC)>>2 ; // 拷贝 B2阶段代码
	for(; i<len; i++)
	{
		code[i] = src[i];
	}
	i = 15*1024/4; // 15K, erase code
	//encoding(passwd, uid);
	//encoding(&code[i], uid);
	code[i+0]=0x000001FF;
	code[i+1]=0x000001FF;
	encoding(&code[i+2], uid);
	memcpy(&code[i+2+8], &board_info, sizeof(board_info));
	//code[1] = (uint32_t)(&reset_handler_e)|0x1;
	code[1] = (uint32_t)reset_h2;
	i = 0x3200/4;
	UserLicense((uint8_t*)&code[i], uid);
	erase_code();
#if 0
	// erase all sectors
	flash_unlock();
	//flash_func_erase_code(0);
	//flash_erase_sector(0, FLASH_CR_PROGRAM_X32);
	address=0;
	for (i = 0; i < len; i++) {
		// program the word
		//flash_func_write_word(address, code[i]);
		address += 4;
	}
	//main();
#if 1

	/* just for paranoia's sake */
	flash_lock();
	scb_reset_system();
#else
	jump_to_app();
#endif
#endif
}
void __attribute__ ((section(".reset_e"))) reset_handler_e1(void)
{
	//volatile unsigned *src, *dest;
	funcp_t *fp;
#if 0
#if 0
	for (src = &_mdata_loadaddr, dest = &_mdata;
		dest < &_medata;
		src++, dest++) {
		*dest = *src;
	}
#endif
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
#endif
	reset_handler_init();
	/* Call the application's entry point. */
	main_e();

	/* Destructors. */
	for (fp = &__fini_array_start; fp < &__fini_array_end; fp++) {
		(*fp)();
	}

}
void __attribute__ ((section(".reset_eb2"))) encrypt_b2(const uint8_t encrypt[], const uint8_t encrypt_len)
{
	/* Call the application's entry point. */
	//main();
	//bool try_boot = true;			/* try booting before we drop to the bootloader */
	//unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

	uint32_t	address = board_info.fw_size;
	const uint32_t len = 16*1024/4; // 16K
	uint32_t i=0;
	const uint32_t* src= (const uint32_t*)0x08000000;
	
	volatile uint32_t* _mtext=NULL;
	//uint32_t passwd[8]={0};
	uint32_t uid[3]={0};
	
	// copy code
	for(i=0; i<len; i++)
	{
		code[i] = src[i];
	}
	//check
	_mtext = (volatile uint32_t*)(0x1FFF0000);
	_mtext += (0x7A10/4);
	uid[0] = _mtext[0];
	uid[1] = _mtext[1];
	uid[2] = _mtext[2];
	// erase code
	address = (uid[0]+uid[1]+uid[2])%0x400;
	for(; i<len; i++)
	{
		code[i] = code[address++]^0x68fe2433U;
	}
	// erase code
	address = (uint32_t)reset_handler_e2;
	i = (uint32_t)main_e2;
	if(i>address) i=address;
	//i = i&0x0000FFFC ; // 对齐
	//i = (0x3D00)>>2 ; 
	i = (i&0x0000FFFC)>>2 ; // 擦除 B2阶段代码
	address = (uid[0]+uid[1]+uid[2])%0x400;
	for(; i<len; i++)
	{
		code[i] = code[address++]^0x68fe2433U;
	}
	i = 15*1024/4; // 15K, erase code
	//encoding(passwd, uid);
	//encoding(&code[i], uid);
	code[i+0]=0x000004FF;
	code[i+1]=0x000004FF;
	//encoding(&code[i+2], uid); 32 Byte
	//memcpy(&code[i+2+8], &board_info, sizeof(board_info));
	memcpy(&code[i+2+8+64], encrypt, encrypt_len); // 保存加密信息
	//code[1] = (uint32_t)(&reset_handler_e)|0x1;
	code[1] = (uint32_t)reset_h3;
	erase_code();
}
extern void main_e2_init(void);
extern void bootloader_b2(unsigned timeout);
void __attribute__ ((section(".reset_b2"))) main_e2(void)
{
	unsigned timeout = 10000;	/* if nonzero, drop out of the bootloader after this time */

	main_e2_init();


	//uart_cout((uint8_t *)"Hello\r\n", 7);
	while (1) {
		//uart_cout((uint8_t *)"Hello\r\n", 7);
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader_b2(timeout); // 10s

		/* look to see if we can boot the app */
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
}
void __attribute__ ((section(".reset_b2"))) reset_handler_e2(void)
{
	funcp_t *fp;
	reset_handler_init();

	/* Call the application's entry point. */
	main_e2();
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

