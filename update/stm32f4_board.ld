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
	rom (rx)  : ORIGIN = 0x08004000, LENGTH = 12K
	rom2 (rx)  : ORIGIN = 0x08008000, LENGTH = 16K
	ram (rwx) : ORIGIN = 0x20003000, LENGTH = 128K
}

/* Enforce emmission of the vector table. */


/* Define sections. */
SECTIONS
{
        . = ORIGIN(rom);

	.also_array : {
								. = ALIGN(4);
								__also_array_start = .;
								KEEP (*(.also_array))
								__also_array_end = .;
	} >rom

  . = ORIGIN(rom2);

	.also_array_boot : {
                . = ALIGN(4);
								also_array_boot_start = .;
								KEEP (*(.also_array_boot))
								also_array_boot_end = .;
	} >rom2
	
        end = .;
}

PROVIDE(_stack = 0x20020000);
