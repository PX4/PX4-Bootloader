/*
 * Common bootloader logic.
 *
 * Aside from the header includes below, this file should have no board-specific logic.
 */

#include <inttypes.h>
#include <stdlib.h>

#if   defined(STM32F4)
# include <libopencm3/stm32/f4/rcc.h>
# include <libopencm3/stm32/f4/gpio.h>
# include <libopencm3/stm32/f4/flash.h>
# include <libopencm3/stm32/f4/scb.h>
#elif defined(STM32F1)
# include <libopencm3/stm32/f1/rcc.h>
# include <libopencm3/stm32/f1/gpio.h>
# include <libopencm3/stm32/f1/flash.h>
# include <libopencm3/stm32/f1/scb.h>
#else
# error Unsupported chip
#endif

#include <libopencm3/stm32/systick.h>

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
// Expected workflow is:
//
// GET_SYNC		verify that the board is present
// GET_DEVICE		determine which board (select firmware to upload)
// CHIP_ERASE		erase the program area and reset address counter
// loop:
//      PROG_MULTI      program bytes
// CHIP_VERIFY		finalise flash programming and reset address counter
// loop:
//	READ_MULTI	readback bytes
// RESET		resets chip and starts application
//

#define PROTO_OK		0x10    // 'ok' response
#define PROTO_FAILED		0x11    // 'fail' response
#define PROTO_INSYNC		0x12    // 'in sync' byte sent before status

#define PROTO_EOC		0x20    // end of command
#define PROTO_GET_SYNC		0x21    // NOP for re-establishing sync
#define PROTO_GET_DEVICE	0x22    // get device ID bytes	<reply_data>: <board info XXX>
#define PROTO_CHIP_ERASE	0x23    // erase program area and reset program address
#define PROTO_CHIP_VERIFY	0x24    // reset program address for verification
#define PROTO_PROG_MULTI	0x27    // write bytes at address + increment	<command_data>: <count><databytes>
#define PROTO_READ_MULTI	0x28    // read bytes at address + increment	<command_data>: <count>,  <reply_data>: <databytes>

#define PROTO_BOOT		0x30    // boot the application

#define PROTO_DEBUG		0x31    // emit debug information - format not defined

#define PROTO_PROG_MULTI_MAX    64	// maximum PROG_MULTI size
#define PROTO_READ_MULTI_MAX    255	// size of the size field

/* argument values for PROTO_GET_DEVICE */
#define PROTO_DEVICE_BL_REV	1
#define PROTO_DEVICE_BOARD_ID	2
#define PROTO_DEVICE_BOARD_REV	3
#define PROTO_DEVICE_FW_SIZE	4

static const uint32_t	bl_proto_rev = 2;	// value returned by PROTO_DEVICE_BL_REV

static unsigned head, tail;
static uint8_t rx_buf[256];

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

	if (timer[TIMER_LED] == 0) {
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
sync_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_OK	// "OK"
	};

	cout(data, sizeof(data));
}

static int
cin_wait(unsigned timeout)
{
	int c = -1;

	/* start the timeout */
	timer[TIMER_CIN] = timeout;

	do {
		c = cin();
		if (c >= 0)
			break;

	} while (timer[TIMER_CIN] > 0);

	return c;
}

static void
cout_word(uint32_t val)
{
	cout((uint8_t *)&val, 4);
}

void
bootloader(unsigned timeout)
{
	int             c;
	int		arg = 0;
	unsigned	i;
	unsigned	address = board_info.fw_size;	/* force erase before upload will work */
	uint32_t	first_word = 0xffffffff;
	static union {
		uint8_t		c[256];
		uint32_t	w[64];
	} flash_buffer;

	/* (re)start the timer system */
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);	/* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();

	/* if we are working with a timeout, start it running */
	if (timeout)
		timer[TIMER_BL_WAIT] = timeout;

	while (true) {
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

		// common argument handling for commands
		switch (c) {
		case PROTO_GET_SYNC:
		case PROTO_CHIP_ERASE:
		case PROTO_CHIP_VERIFY:
		case PROTO_DEBUG:
			/* expect EOC */
			if (cin_wait(1000) != PROTO_EOC)
				goto cmd_bad;
			break;

		case PROTO_PROG_MULTI:
			/* expect count */
			arg = cin_wait(1000);
			if (arg < 0)
				goto cmd_bad;
			break;

		case PROTO_GET_DEVICE:
		case PROTO_READ_MULTI:
			/* expect arg/count then EOC */
			arg = cin_wait(1000);
			if (arg < 0)
				goto cmd_bad;
			if (cin_wait(1000) != PROTO_EOC)
				goto cmd_bad;
			break;
		}

		// handle the command byte
		switch (c) {

		case PROTO_GET_SYNC:            // sync
			break;

		case PROTO_GET_DEVICE:		// report board info

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
			default:
				goto cmd_bad;
			}
			break;

		case PROTO_CHIP_ERASE:          // erase the program area + read for programming
			flash_unlock();
			for (i = 0; flash_func_sector_size(i) != 0; i++)
				flash_func_erase_sector(i);
			address = 0;
			break;

		case PROTO_CHIP_VERIFY:		// reset for verification of the program area
			address = 0;
			break;

		case PROTO_PROG_MULTI:		// program bytes
			if (arg % 4)
				goto cmd_bad;
			if ((address + arg) > board_info.fw_size)
				goto cmd_bad;
			if (arg > sizeof(flash_buffer.c))
				goto cmd_bad;
			for (i = 0; i < arg; i++) {
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
			for (i = 0; i < arg; i++) {
				flash_func_write_word(address, flash_buffer.w[i]);
				address += 4;
			}
			break;

		case PROTO_READ_MULTI:			// readback bytes
			if (arg % 4)
				goto cmd_bad;
			if ((address + arg) > board_info.fw_size)
				goto cmd_bad;
			arg /= 4;

			/* handle readback of the not-yet-programmed first word */
			if ((address == 0) && (first_word != 0xffffffff)) {
				cout((uint8_t *)&first_word, 4);
				address += 4;
				arg--;
			}
			while (arg-- > 0) {
				cout_word(flash_func_read_word(address));
				address += 4;
			}
			break;

		case PROTO_BOOT:
			// program the deferred first word
			if (first_word != 0xffffffff) {
				flash_func_write_word(0, first_word);

				// revert in case the flash was bad...
				first_word = 0xffffffff;
			}

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
		// Currently we do nothing & let the programming tool time out
		// if that's what it wants to do.
		// Let the initial delay keep counting down so that we ignore
		// random chatter from a device.
		while(true);
		continue;
	}
}
