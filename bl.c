/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file bl.c
 *
 * Common bootloader logic.
 *
 * Aside from the header includes below, this file should have no board-specific logic.
 */

#include <inttypes.h>
#include <stdlib.h>

# include <libopencm3/stm32/rcc.h>
# include <libopencm3/stm32/gpio.h>
# include <libopencm3/stm32/flash.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>

#include "bl.h"

// bootloader flash update protocol.
//
// Command format:
//
//      <opcode>[<command_data>]<EOC>
//
// Reply format:
//
//      [<reply_data>]<INSYNC><status>
//
// The <opcode> and <status> values come from the PROTO_ defines below,
// the <*_data> fields is described only for opcodes that transfer data;
// in all other cases the field is omitted.
//
// Expected workflow (protocol 3) is:
//
// GET_SYNC		verify that the board is present
// GET_DEVICE		determine which board (select firmware to upload)
// CHIP_ERASE		erase the program area and reset address counter
// loop:
//      PROG_MULTI      program bytes
// GET_CRC		verify CRC of entire flashable area
// RESET		finalise flash programming, reset chip and starts application
//

// protocol bytes
#define PROTO_INSYNC		0x12    // 'in sync' byte sent before status
#define PROTO_EOC		0x20    // end of command

// Reply bytes
#define PROTO_OK		0x10    // INSYNC/OK      - 'ok' response
#define PROTO_FAILED		0x11    // INSYNC/FAILED  - 'fail' response
#define PROTO_INVALID		0x13	// INSYNC/INVALID - 'invalid' response for bad commands

// Command bytes
#define PROTO_GET_SYNC		0x21    // NOP for re-establishing sync
#define PROTO_GET_DEVICE	0x22    // get device ID bytes
#define PROTO_CHIP_ERASE	0x23    // erase program area and reset program address
#define PROTO_PROG_MULTI	0x27    // write bytes at program address and increment
#define PROTO_GET_CRC		0x29	// compute & return a CRC
#define PROTO_GET_OTP		0x2a	// read a byte from OTP at the given address
#define PROTO_GET_SN		0x2b    // read a word from UDID area ( Serial)  at the given address
#define PROTO_GET_CHIP		0x2c    // read chip version (MCU IDCODE)
#define PROTO_SET_DELAY		0x2d    // set minimum boot delay
#define PROTO_BOOT		0x30    // boot the application
#define PROTO_DEBUG		0x31    // emit debug information - format not defined

#define PROTO_PROG_MULTI_MAX    64	// maximum PROG_MULTI size
#define PROTO_READ_MULTI_MAX    255	// size of the size field

/* argument values for PROTO_GET_DEVICE */
#define PROTO_DEVICE_BL_REV	1	// bootloader revision
#define PROTO_DEVICE_BOARD_ID	2	// board ID
#define PROTO_DEVICE_BOARD_REV	3	// board revision
#define PROTO_DEVICE_FW_SIZE	4	// size of flashable area
#define PROTO_DEVICE_VEC_AREA	5	// contents of reserved vectors 7-10

// address of MCU IDCODE
#define DBGMCU_IDCODE		0xE0042000

static const uint32_t	bl_proto_rev = 4;	// value returned by PROTO_DEVICE_BL_REV

static unsigned head, tail;
static uint8_t rx_buf[256];

static enum led_state {LED_BLINK, LED_ON, LED_OFF} _led_state;

void sys_tick_handler(void);

void
buf_put(uint8_t b)
{
	unsigned next = (head + 1) % sizeof(rx_buf);

	if (next != tail) {
		rx_buf[head] = b;
		head = next;
	}
}

int
buf_get(void)
{
	int	ret = -1;

	if (tail != head) {
		ret = rx_buf[tail];
		tail = (tail + 1) % sizeof(rx_buf);
	}
	return ret;
}

static void
do_jump(uint32_t stacktop, uint32_t entrypoint)
{
	asm volatile(
		"msr msp, %0	\n"
		"bx	%1	\n"
		: : "r" (stacktop), "r" (entrypoint) : );
	// just to keep noreturn happy
	for (;;) ;
}

void
jump_to_app()
{
	const uint32_t *app_base = (const uint32_t *)APP_LOAD_ADDRESS;

	/*
	 * We refuse to program the first word of the app until the upload is marked
	 * complete by the host.  So if it's not 0xffffffff, we should try booting it.
	 */
	if (app_base[0] == 0xffffffff)
		return;
	/*
	 * The second word of the app is the entrypoint; it must point within the
	 * flash area (or we have a bad flash).
	 */
	if (app_base[1] < APP_LOAD_ADDRESS)
		return;
	if (app_base[1] >= (APP_LOAD_ADDRESS + board_info.fw_size))
		return;

	/* just for paranoia's sake */
	flash_lock();

	/* kill the systick interrupt */
	systick_interrupt_disable();
	systick_counter_disable();

	/* and set a specific LED pattern */
	led_off(LED_ACTIVITY);
	led_on(LED_BOOTLOADER);

	/* the interface */
	cfini();

	/* switch exception handlers to the application */
	SCB_VTOR = APP_LOAD_ADDRESS;

	/* extract the stack and entrypoint from the app vector table and go */
	do_jump(app_base[0], app_base[1]);
}

volatile unsigned timer[NTIMERS];

void
sys_tick_handler(void)
{
	unsigned i;

	for (i = 0; i < NTIMERS; i++)
		if (timer[i] > 0)
			timer[i]--;

	if ((_led_state == LED_BLINK) && (timer[TIMER_LED] == 0)) {
		led_toggle(LED_BOOTLOADER);
		timer[TIMER_LED] = 50;
	}
}

void
delay(unsigned msec)
{
	timer[TIMER_DELAY] = msec;

	while(timer[TIMER_DELAY] > 0)
		;
}

static void
led_set(enum led_state state)
{
	_led_state = state;
	switch(state) {
	case LED_OFF:
		led_off(LED_BOOTLOADER);
		break;

	case LED_ON:
		led_on(LED_BOOTLOADER);
		break;

	case LED_BLINK:
		/* restart the blink state machine ASAP */
		timer[TIMER_LED] = 0;
		break;
	}
}

static void
sync_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_OK	// "OK"
	};

	cout(data, sizeof(data));
}

static void
invalid_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_INVALID	// "invalid command"
	};

	cout(data, sizeof(data));
}

static void
failure_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_FAILED	// "command failed"
	};

	cout(data, sizeof(data));
}

static volatile unsigned cin_count;

static int
cin_wait(unsigned timeout)
{
	int c = -1;

	/* start the timeout */
	timer[TIMER_CIN] = timeout;

	do {
		c = cin();
		if (c >= 0) {
			cin_count++;
			break;
		}

	} while (timer[TIMER_CIN] > 0);

	return c;
}

static void
cout_word(uint32_t val)
{
	cout((uint8_t *)&val, 4);
}

static int
cin_word(uint32_t *wp, unsigned timeout)
{
	union {
		uint32_t w;
		uint8_t b[4];
	} u;

	for (unsigned i = 0; i < 4; i++) {
		int c = cin_wait(timeout);
		if (c < 0)
			return c;
		u.b[i] = c & 0xff;
	}

	*wp = u.w;
	return 0;
}

static uint32_t
crc32(const uint8_t *src, unsigned len, unsigned state)
{
	static uint32_t crctab[256];

	/* check whether we have generated the CRC table yet */
	/* this is much smaller than a static table */
	if (crctab[1] == 0) {
		for (unsigned i = 0; i < 256; i++) {
			uint32_t c = i;

			for (unsigned j = 0; j < 8; j++) {
				if (c & 1) {
					c = 0xedb88320U ^ (c >> 1);
				} else {
					c = c >> 1;
				}
			}
			crctab[i] = c;
		}
	}

	for (unsigned i = 0; i < len; i++)
		state = crctab[(state ^ src[i]) & 0xff] ^ (state >> 8);
	return state;
}

void
bootloader(unsigned timeout)
{
	uint32_t	address = board_info.fw_size;	/* force erase before upload will work */
	uint32_t	first_word = 0xffffffff;

	/* (re)start the timer system */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);	/* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();

	/* if we are working with a timeout, start it running */
	if (timeout)
		timer[TIMER_BL_WAIT] = timeout;

	/* make the LED blink while we are idle */
	led_set(LED_BLINK);

	while (true) {
		volatile int c;
		int arg;
		static union {
			uint8_t		c[256];
			uint32_t	w[64];
		} flash_buffer;

		// Wait for a command byte
		led_off(LED_ACTIVITY);
		do {
			/* if we have a timeout and the timer has expired, return now */
			if (timeout && !timer[TIMER_BL_WAIT])
				return;

			/* try to get a byte from the host */
			c = cin_wait(0);

		} while (c < 0);
		led_on(LED_ACTIVITY);

		// handle the command byte
		switch (c) {

			// sync
			//
			// command:		GET_SYNC/EOC
			// reply:		INSYNC/OK
			//
		case PROTO_GET_SYNC:
			/* expect EOC */
			if (cin_wait(1000) != PROTO_EOC)
				goto cmd_bad;
			break;

			// get device info
			//
			// command:		GET_DEVICE/<arg:1>/EOC
			// BL_REV reply:	<revision:4>/INSYNC/EOC
			// BOARD_ID reply:	<board type:4>/INSYNC/EOC
			// BOARD_REV reply:	<board rev:4>/INSYNC/EOC
			// FW_SIZE reply:	<firmware size:4>/INSYNC/EOC
			// VEC_AREA reply	<vectors 7-10:16>/INSYNC/EOC
			// bad arg reply:	INSYNC/INVALID
			//
		case PROTO_GET_DEVICE:
			/* expect arg then EOC */
			arg = cin_wait(1000);
			if (arg < 0)
				goto cmd_bad;
			if (cin_wait(1000) != PROTO_EOC)
				goto cmd_bad;

			switch (arg) {
			case PROTO_DEVICE_BL_REV:
				cout((uint8_t *)&bl_proto_rev, sizeof(bl_proto_rev));
				break;

			case PROTO_DEVICE_BOARD_ID:
				cout((uint8_t *)&board_info.board_type, sizeof(board_info.board_type));
				break;

			case PROTO_DEVICE_BOARD_REV:
				cout((uint8_t *)&board_info.board_rev, sizeof(board_info.board_rev));
				break;

			case PROTO_DEVICE_FW_SIZE:
				cout((uint8_t *)&board_info.fw_size, sizeof(board_info.fw_size));
				break;

			case PROTO_DEVICE_VEC_AREA:
				for (unsigned p = 7; p <= 10; p++) {
					uint32_t bytes = flash_func_read_word(p * 4);

					cout((uint8_t *)&bytes, sizeof(bytes));
				}
				break;

			default:
				goto cmd_bad;
			}
			break;

			// erase and prepare for programming
			//
			// command:		ERASE/EOC
			// success reply:	INSYNC/OK
			// erase failure:	INSYNC/FAILURE
			//
		case PROTO_CHIP_ERASE:
			/* expect EOC */
			if (cin_wait(1000) != PROTO_EOC)
				goto cmd_bad;

			// clear the bootloader LED while erasing - it stops blinking at random
			// and that's confusing
			led_set(LED_ON);

			// erase all sectors
			flash_unlock();
			for (int i = 0; flash_func_sector_size(i) != 0; i++)
				flash_func_erase_sector(i);

			// enable the LED while verifying the erase
			led_set(LED_OFF);

			// verify the erase
			for (address = 0; address < board_info.fw_size; address += 4)
				if (flash_func_read_word(address) != 0xffffffff)
					goto cmd_fail;
			address = 0;

			// resume blinking
			led_set(LED_BLINK);
			break;

			// program bytes at current address
			//
			// command:		PROG_MULTI/<len:1>/<data:len>/EOC
			// success reply:	INSYNC/OK
			// invalid reply:	INSYNC/INVALID
			// readback failure:	INSYNC/FAILURE
			//
		case PROTO_PROG_MULTI:		// program bytes
			// expect count
			arg = cin_wait(1000);
			if (arg < 0)
				goto cmd_bad;

			// sanity-check arguments
			if (arg % 4)
				goto cmd_bad;
			if ((address + arg) > board_info.fw_size)
				goto cmd_bad;
			if (arg > sizeof(flash_buffer.c))
				goto cmd_bad;
			for (int i = 0; i < arg; i++) {
				c = cin_wait(1000);
				if (c < 0)
					goto cmd_bad;
				flash_buffer.c[i] = c;
			}
			if (cin_wait(1000) != PROTO_EOC)
				goto cmd_bad;
			if (address == 0) {
				// save the first word and don't program it until everything else is done
				first_word = flash_buffer.w[0];
				// replace first word with bits we can overwrite later
				flash_buffer.w[0] = 0xffffffff;
			}
			arg /= 4;
			for (int i = 0; i < arg; i++) {

				// program the word
				flash_func_write_word(address, flash_buffer.w[i]);

				// do immediate read-back verify
				if (flash_func_read_word(address) != flash_buffer.w[i])
					goto cmd_fail;
				address += 4;
			}
			break;

			// fetch CRC of the entire flash area
			//
			// command:			GET_CRC/EOC
			// reply:			<crc:4>/INSYNC/OK
			//
		case PROTO_GET_CRC:
			// expect EOC
			if (cin_wait(1000) != PROTO_EOC)
				goto cmd_bad;

			// compute CRC of the programmed area
			uint32_t sum = 0;
			for (unsigned p = 0; p < board_info.fw_size; p += 4) {
				uint32_t bytes;

				if ((p == 0) && (first_word != 0xffffffff)) {
					bytes = first_word;
				} else {
					bytes = flash_func_read_word(p);
				}
				sum = crc32((uint8_t *)&bytes, sizeof(bytes), sum);
			}
			cout_word(sum);
			break;

			// read a word from the OTP
			//
			// command:			GET_OTP/<addr:4>/EOC
			// reply:			<value:4>/INSYNC/OK
		case PROTO_GET_OTP:
			// expect argument
			{
				uint32_t index = 0;

				if (cin_word(&index, 100))
					goto cmd_bad;
				cout_word(flash_func_read_otp(index));
			}
			break;
		case PROTO_GET_SN:
			// expect argument
			{
				uint32_t index = 0;
				if (cin_word(&index, 100))
					goto cmd_bad;
				cout_word(flash_func_read_sn(index));
			}
			break;
		case PROTO_GET_CHIP:
			{
                            cout_word(*(uint32_t *)DBGMCU_IDCODE);
			}
			break;
			// finalise programming and boot the system
			//
			// command:			BOOT/EOC
			// reply:			INSYNC/OK
			//

#ifdef BOOT_DELAY_ADDRESS
                case PROTO_SET_DELAY:
			{
				/*
				  Allow for the bootloader to setup a
				  boot delay signature which tells the
				  board to delay for at least a
				  specified number of seconds on boot.
				 */
				int v = cin_wait(100);
				if (v < 0)
					goto cmd_bad;
				uint8_t boot_delay = v & 0xFF;
				if (boot_delay > BOOT_DELAY_MAX)
					goto cmd_bad;
				
				uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);
				uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS+4);
				
				if (sig1 != BOOT_DELAY_SIGNATURE1 ||
                                    sig2 != BOOT_DELAY_SIGNATURE2)
					goto cmd_bad;
				
				uint32_t value = (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00) | boot_delay;
				flash_func_write_word(BOOT_DELAY_ADDRESS, value);
				if (flash_func_read_word(BOOT_DELAY_ADDRESS) != value)
					goto cmd_fail;
			}
			break;
#endif

		case PROTO_BOOT:
			// expect EOC
			if (cin_wait(1000) != PROTO_EOC)
				goto cmd_bad;

			// program the deferred first word
			if (first_word != 0xffffffff) {
				flash_func_write_word(0, first_word);
				if (flash_func_read_word(0) != first_word)
					goto cmd_fail;

				// revert in case the flash was bad...
				first_word = 0xffffffff;
			}

			// send a sync and wait for it to be collected
			sync_response();
			delay(100);

			// quiesce and jump to the app
			return;

		case PROTO_DEBUG:
			// XXX reserved for ad-hoc debugging as required
			break;

		default:
			continue;
		}
		// we got a command worth syncing, so kill the timeout because
		// we are probably talking to the uploader
		timeout = 0;

		// send the sync response for this command
		sync_response();
		continue;
cmd_bad:
		// send an 'invalid' response but don't kill the timeout - could be garbage
		invalid_response();
		continue;

cmd_fail:
		// send a 'command failed' response but don't kill the timeout - could be garbage
		failure_response();
		continue;
	}
}
