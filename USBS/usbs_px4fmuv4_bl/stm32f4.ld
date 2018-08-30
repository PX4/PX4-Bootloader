/******************** (C) COPYRIGHT 2018 merafour ********************
* Author             : 冷月追风@merafour.blog.163.com
* Version            : V1.0.0
* Date               : 30/8/2018
* Description        : ld.
********************************************************************************
* merafour.blog.163.com
* merafour@163.com
* github.com/Merafour
*******************************************************************************/

/**
 * @file stm32f4.ld
 *
 * Linker script for ST STM32F4 bootloader (use first 16K of flash, all 128K RAM).
 *
 * @author Uwe Hermann <uwe@hermann-uwe.de>
 * @author Stephen Caudle <scaudle@doceme.com>
 */

/* Define memory regions. */
MEMORY
{
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 12K
	eom (rx)  : ORIGIN = 0x08003C00, LENGTH = 1K
	ram (rwx) : ORIGIN = 0x20004000, LENGTH = 128K
}

/* Enforce emmission of the vector table. */
EXTERN (vector_table)

/* Define sections. */
SECTIONS
{
        . = ORIGIN(eom);

        .mtext : {
				. = ALIGN(4);
				_mtext_b1 = .;
				*(.1vectors_h)
                *(.reset_e)     
				. = ALIGN(4);
				_mtext_b2 = .;
				*(.reset_b2)
				*(.reset_eb2)
                /* *(.main_e) */    
                . = ALIGN(4);
                _mtext = .;
        } >eom
		
        . = ORIGIN(rom);

        .text : {
                /* *(.vectors)*/     /* Vector table */
                *(.vectors_m)     /* Vector table */
                *(.reset_m)     
                /* *(.text*) */      /* Program code */
                . = ALIGN(4);
                *(.rodata*)     /* Read-only data */
                . = ALIGN(4);
                _etext = .;
        } >rom

	/* C++ Static constructors/destructors, also used for __attribute__
	 * ((constructor)) and the likes */
	.preinit_array : {
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
	} >rom
	.init_array : {
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
	} >rom
	.fini_array : {
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} >rom

        . = ORIGIN(ram);

        .data : AT(_etext) {
                _data = .;
				*(.text*)       /* Program code */
                *(.data*)       /* Read-write initialized data */
                . = ALIGN(4);
                _edata = .;
        } >ram
	_data_loadaddr = LOADADDR(.data);

        .bss : {
                *(.bss*)        /* Read-write zero initialized data */
                *(COMMON)
                . = ALIGN(4);
                _ebss = .;
        } >ram AT >rom

        /*
         * The .eh_frame section appears to be used for C++ exception handling.
         * You may need to fix this if you're using C++.
         */
        /DISCARD/ : { *(.eh_frame) }

        . = ALIGN(4);
        end = .;
}

PROVIDE(_stack = 0x20020000);
