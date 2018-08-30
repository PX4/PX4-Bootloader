/******************** (C) COPYRIGHT 2018 merafour ********************
* Author             : 冷月追风@merafour.blog.163.com
* Version            : V1.0.0
* Date               : 30/8/2018
* Description        : new bootloader.
********************************************************************************
* merafour.blog.163.com
* merafour@163.com
* github.com/Merafour
*******************************************************************************/
/*
 * update bootloader frame.
 *
 */

#include <stdint.h>
#include <string.h>

#define bl_path build/bl/usbs_bl.data

__attribute__((section(".bootloader_array1"))) static void bl_flash(void)
{
	__asm__ __volatile__
		(
		 /*" .section    .stm32_apm_param_array, \"a\"\n"*/           /* Create a stack frame to hold 3 parms + lr */
		 " bl  .\n"
		 " .align  4\n"
		 " .incbin \"build/bl/usbs_bl.data\"\n"           /* Create a stack frame to hold 3 parms + lr */
		 " sub sp, sp, #16\n"           /* Create a stack frame to hold 3 parms + lr */
		);
}

//static uint32_t bl_flash_addr = 0x08004100;//((uint32_t)bl_flash)&0xFFFFFFF0;
//static uint32_t bl_flash_addr = 0x08004100;//((uint32_t)bl_flash)&0xFFFFFFF0;
static uint32_t bl_flash_addr = 0x08004100;

__attribute__((section(".bootloader_array2"))) const char* bl_flash_end(void)
{
	const char* addr;
	addr = (const char*) (&bl_flash_end);
	return  addr;
}
#define BL_LOAD_ADDRESS   0x08000000
#define BL_SIZE           0x00004000
#define BL_SIZE32         0x00001000
#define BL_RAM_ADDRESS    0x20000000

__attribute__((section(".bootloader_array_code"))) const uint32_t bl_flash_start(void)
{
	uint32_t addr;
	const uint32_t *app_base;
	addr = ((uint32_t)bl_flash)&0xFFFFFFF0;
	//if(addr == (bl_flash_addr))
	{
		uint16_t i=0;
		// serach sp, reset_handler
		for(i=0; i<512; i++)
		{
			/*app_base = (const uint32_t *)(&addr[i]);
			// check sp
			if(app_base[0]<0x20000000) continue; // ram start.
			if(app_base[0]>0x20003000) continue; // ram end.
			if(app_base[0]&0x3FF) continue;      // sp 1K Align
			// check reset_handler
			if(app_base[1] < BL_LOAD_ADDRESS) continue; // bl start.
			if(app_base[1] >= (BL_LOAD_ADDRESS + BL_SIZE)) continue; // bl end.
			bl_flash_addr = &addr[i];*/
			app_base = (const uint32_t*)(addr+4*i);
			// check sp
			if(BL_RAM_ADDRESS != (app_base[0]&0xFFF00000)) continue; // 0x20020000.
			if(app_base[20] != app_base[21]) continue; // IRQ. NVIC_EXTI3_IRQ==NVIC_EXTI4_IRQ
			// check reset_handler
			if(BL_LOAD_ADDRESS != (app_base[1]&0xFFFF0000)) continue; // 0x08004000.
			if(app_base[1] >= (BL_LOAD_ADDRESS + BL_SIZE)) continue; // bl end.
			bl_flash_addr = addr+4*i;
			break;
		}
	}
	return  bl_flash_addr;
}

/*uint16_t bl_flash_get(char code[BL_SIZE], const uint16_t _size, const uint8_t set_head)
{
	uint16_t len;
	const char* addr;
	if(set_head) 
	{
		bl_flash_start();
		return 0;
	}
	addr = bl_flash_addr;
	// copy bl
	for(len=0; len<_size; len++)
	{
		if(len>=BL_SIZE) break; // end.
		code[len] = addr[len];  // copy
	}
	return len;
}*/
uint16_t bl_flash_get32(uint32_t code[BL_SIZE32], const uint16_t _size, const uint8_t set_head)
{
	uint16_t len;
	const uint32_t* addr;
	if(set_head) 
	{
		bl_flash_start();
		return 0;
	}
	addr = (const uint32_t*)bl_flash_addr;
	// copy bl
	for(len=0; len<_size; len++)
	{
		if(len>=BL_SIZE32) break; // end.
		code[len] = addr[len];  // copy
	}
	return len;
}



