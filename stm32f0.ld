/************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2010 libopencm3 project (Uwe Hermann, Stephen Caudle)
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

/**
 * @file stm32f0.ld
 *
 * Linker script for ST STM32F0 bootloader (use first 6K of flash, 8K RAM).
 *
 * @author Uwe Hermann <uwe@hermann-uwe.de>
 * @author Stephen Caudle <scaudle@doceme.com>
 */

/* Define memory regions. */
MEMORY
{
	rom (rx)          : ORIGIN = 0x08000000, LENGTH = 6K
    ram_vectors (rwx) : ORIGIN = 0x20000000, LENGTH = 0xC0
	ram (rwx)         : ORIGIN = 0x200000C0, LENGTH = 8K - 0xC0
}

/* Enforce emmission of the vector table. */
EXTERN (vector_table)

/* Define sections. */
SECTIONS
{
        _ram_vector_start = ORIGIN(ram_vectors);
        _ram_vector_end = ORIGIN(ram_vectors) + LENGTH(ram_vectors);
        . = ORIGIN(rom);

        .text : {
                *(.vectors)     /* Vector table */
                *(.text*)       /* Program code */
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

PROVIDE(_stack = 0x20002000);