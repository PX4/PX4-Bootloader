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

static const uint32_t	bl_proto_rev = 3;	// value returned by PROTO_DEVICE_BL_REV

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

static const uint32_t crctab[] =
{
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
	0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
	0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
	0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
	0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
	0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
	0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
	0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
	0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
	0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
	0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
	0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
	0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
	0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
	0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
	0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
	0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
	0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
	0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
	0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static uint32_t
crc32(const uint8_t *src, unsigned len, unsigned state)
{
	for (unsigned i = 0; i < len; i++)
		state = crctab[(state ^ src[i]) & 0xff] ^ (state >> 8);
	return state;
}

void
bootloader(unsigned timeout)
{
	unsigned	address = board_info.fw_size;	/* force erase before upload will work */
	uint32_t	first_word = 0xffffffff;

	/* (re)start the timer system */
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);	/* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();

	/* if we are working with a timeout, start it running */
	if (timeout)
		timer[TIMER_BL_WAIT] = timeout;

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
			led_off(LED_BOOTLOADER);

			// erase all sectors
			flash_unlock();
			for (int i = 0; flash_func_sector_size(i) != 0; i++)
				flash_func_erase_sector(i);

			// verify the erase
			for (address = 0; address < board_info.fw_size; address += 4)
				if (flash_func_read_word(address) != 0xffffffff)
					goto cmd_fail;
			address = 0;
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
			// reply:			<crc:4>/INSYNC/EOC
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

			// finalise programming and boot the system
			//
			// command:			BOOT/EOC
			// reply:			INSYNC/OK
			//
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
