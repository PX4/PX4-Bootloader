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
#include "hw_config.h"

#include <inttypes.h>
#include <stdlib.h>

# include <libopencm3/stm32/rcc.h>
# include <libopencm3/stm32/gpio.h>
# include <libopencm3/stm32/flash.h>
#if defined(USE_SYSCFG_CFGR1_MEM_MODE_SRAM)
# include <libopencm3/stm32/syscfg.h>
#endif
# include <libopencm3/stm32/i2c.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>

#include "bl.h"
#include "cdcacm.h"
#include "uart.h"

// bootloader flash update protocol (USART and USB).
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
//
//
// I2C protocol.
//
// I2C update is done with bootloader device in slave mode.
// All integers are Little-endian
//
// All "write" commands, where updater sends some data or requests
// bootloader to perform an action (like erase, for example)
// have to be followed by I2C_REPLY_ACK byte.
// After the command has finished, the updater must perform
// "I2C_COMMAND_GET_PROGRESS" before sending any new writes.
// See bootloader_status_t enum for possible replies,
// Only values starting with BOOT_STATUS are possible in I2C replies,
// Other values are for internal usage only.
//
// All "read" commands, where updater requests some data from bootloader
// are sent as follows: updater sets I2C to updater->bootloader mode
// and sends 1 byte command code from i2c_commands_t,
// after that updater sets I2C to bootloader->updater mode and reads
// 1 byte ACK/NAK reply (see i2c_reply_codes_t),
// and in case reply is I2C_REPLY_ACK, requested data will follow.
// See i2c_commands_t for list of possible commands and their data.
//
// Expected workflow (protocol 5):
// IMPORTANT!!! I2C communication is not guaranteed to work for
// up to X ms time after ERASE or WRITE command is sent, where X
// is set by bootloader firmware and returned in
// I2C_COMMAND_GET_ERASE_TIME and I2C_COMMAND_GET_WRITE_TIME
// commands respectively.
// This is due to chip not being able to write and read flash
// at the same time. Be nice and obey the timeouts.
//
// Send, I2C_COMMAND_GET_STATUS to and check bootloader protocol
// and board type and revision.
//
// Send I2C_COMMAND_GET_ERASE_TIME, I2C_COMMAND_GET_WRITE_TIME,
// I2C_COMMAND_GET_WRITE_SIZE, and I2C_COMMAND_GET_CRC_TIME
// and save the values to be used later.
//
// Send I2C_PERFORM_ERASE_COMMAND and wait for <erase time> ms.
// Send I2C_COMMAND_GET_PROGRESS and check that everything ok.
// Send I2C_PERFORM_WRITE_COMMAND - the program-data payload
// must be <write size> maximum and divisible by 4.
// Wait for <write time> ms and check progress.
// Repeat the write command for all application bytes.
// Send I2C_PREPARE_APP_CRC, wait for <CRC time> and get progress.
// Send I2C_COMMAND_GET_APP_CRC and compare to application CRC32.
// Note that CRC is calculated up to maximum application size, so
// add FF-s until the application size matches the fw_size parameter of
// the I2C_COMMAND_GET_STATUS command.
// If CRC is correct, finish upload and boot the app by sending
// I2C_FINISH_AND_BOOT command.
// The BOOT command also sets progress after a very brief delay,
// but updater might want to wait a bit more - in case
// I2C_FINISH_AND_BOOT command has failed, the status will change
// from OK to FAILURE as soon as boot has failed.
// This change WON'T happen if progress was read by updater before
// failure has occurred.
// The boot command will report "OK" for 500 ms and then try to boot the app.
// Alternatively, if updater reads the OK status and after 500ms is still able
// to communicate with bootloader - this is also an indication that
// bootloader failed to go to app.
//
// Updater can also use I2C_COMMAND_CHECK_APP command to see if
// BOOT is likely to succeed, but only if no erase/writes were performed.

#define BL_PROTOCOL_VERSION 		5		// The revision of the bootloader protocol
// protocol bytes
#define PROTO_INSYNC				0x12    // 'in sync' byte sent before status
#define PROTO_EOC					0x20    // end of command

// Reply bytes
#define PROTO_OK					0x10    // INSYNC/OK      - 'ok' response
#define PROTO_FAILED				0x11    // INSYNC/FAILED  - 'fail' response
#define PROTO_INVALID				0x13	// INSYNC/INVALID - 'invalid' response for bad commands
#define PROTO_BAD_SILICON_REV 		0x14 	// On the F4 series there is an issue with < Rev 3 silicon
// see https://pixhawk.org/help/errata
// Command bytes
#define PROTO_GET_SYNC				0x21    // NOP for re-establishing sync
#define PROTO_GET_DEVICE			0x22    // get device ID bytes
#define PROTO_CHIP_ERASE			0x23    // erase program area and reset program address
#define PROTO_PROG_MULTI			0x27    // write bytes at program address and increment
#define PROTO_GET_CRC				0x29	// compute & return a CRC
#define PROTO_GET_OTP				0x2a	// read a byte from OTP at the given address
#define PROTO_GET_SN				0x2b    // read a word from UDID area ( Serial)  at the given address
#define PROTO_GET_CHIP				0x2c    // read chip version (MCU IDCODE)
#define PROTO_SET_DELAY				0x2d    // set minimum boot delay
#define PROTO_GET_CHIP_DES			0x2e    // read chip version In ASCII
#define PROTO_BOOT					0x30    // boot the application
#define PROTO_DEBUG					0x31    // emit debug information - format not defined

#define PROTO_PROG_MULTI_MAX    64	// maximum PROG_MULTI size
#define PROTO_READ_MULTI_MAX    255	// size of the size field

/* argument values for PROTO_GET_DEVICE */
#define PROTO_DEVICE_BL_REV	1	// bootloader revision
#define PROTO_DEVICE_BOARD_ID	2	// board ID
#define PROTO_DEVICE_BOARD_REV	3	// board revision
#define PROTO_DEVICE_FW_SIZE	4	// size of flashable area
#define PROTO_DEVICE_VEC_AREA	5	// contents of reserved vectors 7-10

static uint8_t bl_type;
#if INTERFACE_USART || INTERFACE_USB
static uint8_t last_input;
#endif

// Returns little-endian indexed (idx of 0 is the LSB) byte of a word
#define BYTE_BY_INDEX(word, idx) (((word) >> (idx) * 8) & 0xFF)

inline void cinit(void *config, uint8_t interface)
{
#if INTERFACE_USB

	if (interface == USB) {
		return usb_cinit();
	}

#endif
#if INTERFACE_USART

	if (interface == USART) {
		return uart_cinit(config);
	}

#endif
}
inline void cfini(void)
{
#if INTERFACE_USB
	usb_cfini();
#endif
#if INTERFACE_USART
	uart_cfini();
#endif
}
inline int cin(void)
{
#if INTERFACE_USB

	if (bl_type == NONE || bl_type == USB) {
		int usb_in = usb_cin();

		if (usb_in >= 0) {
			last_input = USB;
			return usb_in;
		}
	}

#endif

#if INTERFACE_USART

	if (bl_type == NONE || bl_type == USART) {
		int	uart_in = uart_cin();

		if (uart_in >= 0) {
			last_input = USART;
			return uart_in;
		}
	}

#endif

	return -1;
}

inline void cout(uint8_t *buf, unsigned len)
{
#if INTERFACE_USB

	if (bl_type == USB) {
		usb_cout(buf, len);
	}

#endif
#if INTERFACE_USART

	if (bl_type == USART) {
		uart_cout(buf, len);
	}

#endif
}



static const uint32_t	bl_proto_rev = BL_PROTOCOL_VERSION;	// value returned by PROTO_DEVICE_BL_REV

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
		: : "r"(stacktop), "r"(entrypoint) :);

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
	if (app_base[0] == 0xffffffff) {
		return;
	}

	/*
	 * The second word of the app is the entrypoint; it must point within the
	 * flash area (or we have a bad flash).
	 */
	if (app_base[1] < APP_LOAD_ADDRESS) {
		return;
	}

	if (app_base[1] >= (APP_LOAD_ADDRESS + board_info.fw_size)) {
		return;
	}

	/* just for paranoia's sake */
	flash_lock();

	/* kill the systick interrupt */
	systick_interrupt_disable();
	systick_counter_disable();

	/* deinitialise the interface */
	cfini();

	/* reset the clock */
	clock_deinit();

	/* deinitialise the board */
	board_deinit();

#if defined(USE_SYSCFG_CFGR1_MEM_MODE_SRAM)
	/* We can't switch exception handlers to app using SCB_VTOR,
	 * we need to copy them to RAM and switch boot mode to RAM */
	extern uint8_t _ram_vector_start, _ram_vector_end; // exported by linker
	uint8_t *app_vector = (uint8_t *) APP_LOAD_ADDRESS;

	for (uint8_t *destination = &_ram_vector_start; destination < &_ram_vector_end; ++destination) {
		*(destination) = *(app_vector++); // We just hope that app has correct vector table for the same processor
	}

	SYSCFG_CFGR1 = (SYSCFG_CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) | SYSCFG_CFGR1_MEM_MODE_SRAM;
#else
	/* by default use SCB_VTOR to switch exception handlers to the application */
	SCB_VTOR = APP_LOAD_ADDRESS;
#endif

	/* extract the stack and entrypoint from the app vector table and go */
	do_jump(app_base[0], app_base[1]);
}

volatile unsigned timer[NTIMERS];
#ifdef I2C_DEBUG_ENABLE
static volatile uint32_t current_time_ms = 0;
#endif

// TODO: This function is broken during flash writes on at least some MCUs
void
sys_tick_handler(void)
{
	unsigned i;
#ifdef I2C_DEBUG_ENABLE
	current_time_ms++;
#endif

	for (i = 0; i < NTIMERS; i++)
		if (timer[i] > 0) {
			timer[i]--;
		}

	if ((_led_state == LED_BLINK) && (timer[TIMER_LED] == 0)) {
		led_toggle(LED_BOOTLOADER);
		timer[TIMER_LED] = 50;
	}
}

void
delay(unsigned msec)
{
	timer[TIMER_DELAY] = msec;

	while (timer[TIMER_DELAY] > 0)
		;
}

static void
led_set(enum led_state state)
{
	_led_state = state;

	switch (state) {
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

// Used both by common bootloader and I2C bootloader
static volatile union {
	uint8_t		c[256];
	uint32_t	w[64];
} flash_buffer;

enum flash_command_result_t {
	FLASH_COMMAND_OK,
	FLASH_COMMAND_FAILURE,
	FLASH_COMMAND_BAD_SILICON
};

static enum flash_command_result_t
perform_chip_erase(uint32_t *address) {
#if defined(TARGET_HW_PX4_FMU_V4)

	if (check_silicon())
	{
		return FLASH_COMMAND_BAD_SILICON;
	}

#endif
	// clear the bootloader LED while erasing - it stops blinking at random
	// and that's confusing
	led_set(LED_ON);

	// erase all sectors
	flash_unlock();

	for (int i = 0; flash_func_sector_size(i) != 0; i++)
	{
		flash_func_erase_sector(i);
	}

	// enable the LED while verifying the erase
	led_set(LED_OFF);

	// verify the erase
	for (*address = 0; *address < board_info.fw_size; *address += 4)
		if (flash_func_read_word(*address) != 0xffffffff)
		{
			return FLASH_COMMAND_FAILURE;
		}

	*address = 0;

	// resume blinking
	led_set(LED_BLINK);
	return FLASH_COMMAND_OK;
}

static enum flash_command_result_t
perform_flash_write(int write_size, uint32_t *address, uint32_t *first_word) {
	if (*address == 0)
	{

#if defined(TARGET_HW_PX4_FMU_V4)

		if (check_silicon()) {
			return FLASH_COMMAND_BAD_SILICON;
		}

#endif

		// save the first word and don't program it until everything else is done
		*first_word = flash_buffer.w[0];
		// replace first word with bits we can overwrite later
		flash_buffer.w[0] = 0xffffffff;
	}

	for (int i = 0; i < write_size; i++)
	{

		// program the word
		flash_func_write_word(*address, flash_buffer.w[i]);

		// do immediate read-back verify
		if (flash_func_read_word(*address) != flash_buffer.w[i]) {
			return FLASH_COMMAND_FAILURE;
		}

		*address += 4;
	}

	return FLASH_COMMAND_OK;
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

	for (unsigned i = 0; i < len; i++) {
		state = crctab[(state ^ src[i]) & 0xff] ^ (state >> 8);
	}

	return state;
}

static uint32_t
calc_app_crc(uint32_t first_word)
{
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

	return sum;
}

#if defined(INTERFACE_I2C) && INTERFACE_I2C

# ifdef I2C_DEBUG_ENABLE
#  if !defined(INTERFACE_USART) || !INTERFACE_USART
#   error You must enable UART to debug I2C
#  endif

static volatile char i2c_debug_buffer[200];
static volatile uint8_t i2c_debug_start = 0;
static volatile uint8_t i2c_debug_end = 0;
static volatile bool i2c_debug_buffer_overflow = false;

// This function is blocking and should be called from a single interrupt
// So don't worry about possible other callers - we only have to think about the emptying thread
void
i2c_send_to_uart(const char *const data, uint8_t data_size)
{
	uint8_t new_end = i2c_debug_end + data_size;
	bool write_allowed = false;

	// This condition can't get "worse", the "start" can only go up to the end
	if (i2c_debug_end >= i2c_debug_start) {
		if (new_end > sizeof(i2c_debug_buffer)) {
			new_end -= sizeof(i2c_debug_buffer);

			// If end was overlapped over the edge
			// but the start overlapped only after the previous if, though luck
			if (new_end < i2c_debug_start) {
				write_allowed = true;
			}

		} else {
			write_allowed = true;
		}

	} else {
		// If end was overlapped over the edge
		// but the start overlapped only after the previous if, though luck
		if (new_end < i2c_debug_start) {
			write_allowed = true;
		}
	}

	// Nothing to worry about, the space could only get freed while we write
	if (write_allowed) {
		uint8_t data_left = data_size;
		int i = i2c_debug_end;

		for (; data_left && i < sizeof(i2c_debug_buffer); ++i) {
			i2c_debug_buffer[i] = data[i - i2c_debug_end];
			data_left--;
		}

		// No problems if data_left is zero at the same time
		if (i == sizeof(i2c_debug_buffer)) {
			uint8_t data_offset = data_size - data_left;
			i = 0;

			for (; i < data_left; ++i) {
				i2c_debug_buffer[i] = data[data_offset + i];
			}
		}

		i2c_debug_end = i;

	} else {
		i2c_debug_buffer_overflow = true;
	}
}


#  define I2C_UART_DEBUG(x, y) i2c_send_to_uart(x, y)
#  define DIRECT_UART_DEBUG(x, y) uart_cout(x, y)
# else // #ifdef I2C_DEBUG_ENABLE
// Skip all calls
#  define I2C_UART_DEBUG(...)
#  define DIRECT_UART_DEBUG(...)
# endif // #ifdef I2C_DEBUG_ENABLE

// Message templates, preallocated to save time during I2C interrupts
static struct {
	uint32_t protocol;
	uint32_t board_type;
	uint32_t board_rev;
	uint32_t fw_size;
	uint32_t vec_area[4];
} __attribute__((packed)) i2c_status_reply;

enum i2c_states_t {
	I2C_WAIT_FOR_REQUEST,
	I2C_GET_COMMAND,
	I2C_WAIT_FOR_EOC,
	I2C_WAIT_FOR_DATA,
	I2C_SENDING_DATA,
	I2C_WAIT_FOR_STOP
};

enum i2c_commands_t {
	// Bootloader -> outside world commands
	I2C_COMMAND_DEFAULT =          0x00, // Same as GET_STATUS
	I2C_COMMAND_GET_STATUS =       0x01, // See i2c_status_reply
	I2C_COMMAND_GET_PROGRESS =     0x02, // bootloader_status_t, returns only one of BOOT_STATUS_*
	I2C_COMMAND_GET_ERASE_TIME =   0x03, // uint16_t ms, Delay after erase command
	I2C_COMMAND_GET_WRITE_TIME =   0x04, // uint16_t ms, It depends on the received bytes size, we'll report maximum
	I2C_COMMAND_GET_WRITE_SIZE =   0x05, // uint8_t, Maximum allowed write size in bytes
	I2C_COMMAND_GET_CRC_TIME =     0x06, // uint16_t ms, Delay after CRC command
	I2C_COMMAND_CHECK_APP =        0x07, // uint8_t, Check if the app is present, returns non-zero value if present
	I2C_COMMAND_GET_APP_CRC =      0x08, // uint32_t, Returns CRC32, needs to be prepared first
	I2C_READ_COMMAND_MAX,
	// Outside world -> bootloader commands, all need to be followed by ACK to get accepted and then payload
	I2C_WRITE_COMMAND_MIN =        0x80,
	I2C_TEST_PRINT_COMMAND =       0x81, // any payload will be printed to debug USART
	I2C_PERFORM_ERASE_COMMAND =    0x82, // no payload
	I2C_PERFORM_WRITE_COMMAND =    0x83, // payload - 1 byte size, then size bytes app, then CRC32
	I2C_PREPARE_APP_CRC =          0x84, // no payload
	I2C_FINISH_AND_BOOT =          0x85, // no payload
	I2C_WRITE_COMMAND_MAX
};

enum i2c_reply_codes_t {
	I2C_REPLY_ACK =                0x12, // All is fine with the world
	I2C_REPLY_NAK =                0x13, // Unknown command or command rejected
	I2C_REPLY_BUSY =               0x14  // Command rejected, because another command is in progress
};

# define I2C_MAX_PAYLOAD_SIZE 200 // maximum amount of data we are allowed to reply
# define I2C_STATUS_PROCESSED_EVENTS (I2C_ISR_OVR \
				      | I2C_ISR_ARLO \
				      | I2C_ISR_BERR \
				      | I2C_ISR_NACKF \
				      | I2C_ISR_ADDR \
				      | I2C_ISR_RXNE \
				      | I2C_ISR_TXIS)

static volatile uint8_t program_multi_count;
static volatile uint32_t full_app_crc = 0;

// Variable to sync main bootloader thread with I2C interrupt handler
static volatile enum bootloader_status_t {
	BOOT_STATUS_IDLE =        0x00, // No command active
	// Replies main loop -> I2C handler, also used in I2C_COMMAND_GET_PROGRESS
	BOOT_STATUS_FINISHED_OK = 0x01, // Previous command finished successfully
	BOOT_STATUS_IN_PROGRESS = 0x02, // Command processing is in progress
	BOOT_STATUS_BAD_SILICON = 0x03, // Command failed because of bad silicon
	BOOT_STATUS_FAILURE =     0x04, // Command failed for unspecified reason
	BOOT_STATUS_BAD_COMMAND = 0x05, // Command rejected because of malformed packet or bad data
	// Requests I2C handler -> main loop
	BOOT_COMMAND_ERASE =      0x80,
	BOOT_COMMAND_WRITE =      0x81,
	BOOT_COMMAND_CALC_CRC =   0x82,
	BOOT_COMMAND_BOOT_APP =   0x83
} main_bootloader_status = BOOT_STATUS_IDLE;

/*
 * I2C comms handler in interrupt.
 * At least some chips fail flash reads during flash writes.
 * TODO: So it might be useful to move this function to RAM.
 * If called from RAM this function should correctly report "busy" during writes.
 * Moving to RAM shouldn't be too hard - only debug prints
 * and I2C reset function function are external.
 * Reset can be moved to main thread.
 */
void
BOARD_I2C_IRQ_FUNCTION(void)
{
	static uint8_t i2c_rx_tx_idx = I2C_MAX_PAYLOAD_SIZE + 1;
	static enum i2c_states_t current_i2c_state = I2C_WAIT_FOR_REQUEST;
	static enum i2c_commands_t current_command = I2C_COMMAND_DEFAULT;
	// Allow the I2C communication to stop before sending a command that potentially locks flash reads
	static enum bootloader_status_t new_bootloader_command = BOOT_STATUS_FAILURE;
	I2C_UART_DEBUG(".", 1);


	uint32_t i2c_irq_status = I2C_ISR(BOARD_I2C); // Get interrupt status

	if (i2c_irq_status & I2C_ISR_ADDR) { // Matching address event
		I2C_ICR(BOARD_I2C) |= I2C_ICR_ADDRCF; // Clear the address event
		i2c_irq_status &= ~I2C_ISR_ADDR; // Mark address event as processed

		// Outgoing transmission
		if (i2c_irq_status & I2C_ISR_DIR) {
			if (current_command == I2C_COMMAND_DEFAULT) {
				current_command = I2C_COMMAND_GET_STATUS;
			}

			if (current_command >= I2C_READ_COMMAND_MAX) {
				current_i2c_state = I2C_WAIT_FOR_STOP;
				I2C_TXDR(BOARD_I2C) = I2C_REPLY_NAK; // Set next transmit byte

			} else if (main_bootloader_status == BOOT_STATUS_IN_PROGRESS) {
				current_i2c_state = I2C_WAIT_FOR_STOP;
				I2C_TXDR(BOARD_I2C) = I2C_REPLY_BUSY; // Set next transmit byte

			} else {
				// TODO: Mark if crc was calculated and reject operation if not
				current_i2c_state = I2C_SENDING_DATA;
				I2C_TXDR(BOARD_I2C) = I2C_REPLY_ACK; // Set next transmit byte
				i2c_rx_tx_idx = 0;
			}

			I2C_CR1(BOARD_I2C) &= ~I2C_CR1_RXIE; // disable read interrupts
			I2C_CR1(BOARD_I2C) |= I2C_CR1_TXIE | I2C_CR1_NACKIE; // enable transmit interrupts
			I2C_UART_DEBUG("<", 1);
		}

		// Incoming transmission, first byte is the command itself
		else {
			current_i2c_state = I2C_GET_COMMAND;
			I2C_CR1(BOARD_I2C) &= ~I2C_CR1_TXIE | I2C_CR1_NACKIE; // disable transmit interrupts
			I2C_CR1(BOARD_I2C) |= I2C_CR1_RXIE; // enable receive interrupts
			I2C_UART_DEBUG(">", 1);
		}
	}

	switch (current_i2c_state) {
	case I2C_WAIT_FOR_REQUEST:
		// Address event is processed in all states
		break;

	case I2C_GET_COMMAND:
		if (i2c_irq_status & I2C_ISR_RXNE) { // Byte received event
			i2c_irq_status &= ~I2C_ISR_RXNE; // Mark received event as processed
			current_command = I2C_RXDR(BOARD_I2C); // Read the byte

			if (current_command > I2C_WRITE_COMMAND_MIN && current_command < I2C_WRITE_COMMAND_MAX
			    && main_bootloader_status == BOOT_STATUS_IDLE) {
				current_i2c_state = I2C_WAIT_FOR_EOC;
				// If anything happens to us, reply that it was a bad command
				main_bootloader_status = BOOT_STATUS_BAD_COMMAND;

			} else { // either "read", unknown command or we're busy - just wait
				current_i2c_state = I2C_WAIT_FOR_STOP;
			}

			I2C_UART_DEBUG("G", 1);
			I2C_UART_DEBUG((char *)&current_command, sizeof(current_command));
		}

		break;

	case I2C_WAIT_FOR_EOC:
		if (i2c_irq_status & I2C_ISR_RXNE) { // Byte received event
			i2c_irq_status &= ~I2C_ISR_RXNE; // Mark received event as processed

			if (I2C_RXDR(BOARD_I2C) == I2C_REPLY_ACK) { // Received ACK message -> valid command
				switch (current_command) {
				case I2C_TEST_PRINT_COMMAND:
					current_i2c_state = I2C_WAIT_FOR_DATA;
					main_bootloader_status = BOOT_STATUS_FINISHED_OK;
					break;

				case I2C_PERFORM_ERASE_COMMAND:
					current_i2c_state = I2C_WAIT_FOR_STOP;
					new_bootloader_command = BOOT_COMMAND_ERASE;
					break;

				case I2C_PREPARE_APP_CRC:
					current_i2c_state = I2C_WAIT_FOR_STOP;
					new_bootloader_command = BOOT_COMMAND_CALC_CRC;
					break;

				case I2C_FINISH_AND_BOOT:
					current_i2c_state = I2C_WAIT_FOR_STOP;
					new_bootloader_command = BOOT_COMMAND_BOOT_APP;
					break;

				case I2C_PERFORM_WRITE_COMMAND:
					current_i2c_state = I2C_WAIT_FOR_DATA;
					i2c_rx_tx_idx = 0;
					break;

				default:
					current_i2c_state = I2C_WAIT_FOR_STOP;
				} // switch (current_command)

				I2C_UART_DEBUG("E", 1);

			} else {
				I2C_UART_DEBUG("e", 1);
				current_i2c_state = I2C_WAIT_FOR_STOP;
			}
		}

		break;

	case I2C_WAIT_FOR_DATA:
		if (i2c_irq_status & I2C_ISR_RXNE) { // Byte received event
			i2c_irq_status &= ~I2C_ISR_RXNE; // Mark received event as processed

			// Get the bytes to buffer and set program_multi_count
			// In case of an error - don't clear anything - there is no need
			if (current_command == I2C_PERFORM_WRITE_COMMAND) {
				// Get number of bytes
				if (i2c_rx_tx_idx == 0) {
					program_multi_count = I2C_RXDR(BOARD_I2C); // Read the byte
					I2C_UART_DEBUG("W", 1);
					I2C_UART_DEBUG((char *) &program_multi_count, 1);

					if (program_multi_count > I2C_MAX_PAYLOAD_SIZE - 7 || program_multi_count % 4 != 0) {
						// "BAD_COMMAND" has already been set, so just abort
						current_i2c_state = I2C_WAIT_FOR_STOP;
					}

					i2c_rx_tx_idx++;
				}

				// Get app and CRC bytes, CRC bytes just go straight after app
				else if (i2c_rx_tx_idx < program_multi_count + 4) {
					flash_buffer.c[i2c_rx_tx_idx - 1] = I2C_RXDR(BOARD_I2C); // Read the byte
					i2c_rx_tx_idx++;
				}

				// The last CRC byte
				else if (i2c_rx_tx_idx == program_multi_count + 4) {
					flash_buffer.c[i2c_rx_tx_idx - 1] = I2C_RXDR(BOARD_I2C); // Read the byte
					i2c_rx_tx_idx++;
					// We're good to go
					new_bootloader_command = BOOT_COMMAND_WRITE;
					I2C_UART_DEBUG("C", 1);
					I2C_UART_DEBUG((char *) &flash_buffer.c[i2c_rx_tx_idx - 2], 1);
				}

				// Too many bytes received, abort
				else {
					// "BAD_COMMAND" has already been set, so just abort
					new_bootloader_command = BOOT_STATUS_FAILURE;
					I2C_UART_DEBUG("O", 1);
					// If debug is disabled, we still need to read out the register
					volatile char tmp = (char) I2C_RXDR(BOARD_I2C); // Read the byte
					I2C_UART_DEBUG((char *)&tmp, 1);
					(void) tmp;
					current_i2c_state = I2C_WAIT_FOR_STOP;
				}

			} else {
				// If debug is disabled, we still need to read out the register
				volatile char tmp = (char) I2C_RXDR(BOARD_I2C); // Read the byte
				I2C_UART_DEBUG("R", 1);
				I2C_UART_DEBUG((char *)&tmp, 1);
				(void) tmp;
			}
		}

		break;

	case I2C_SENDING_DATA:

		// Previous byte was not sent correctly
		if (i2c_irq_status & I2C_ISR_NACKF) {
			i2c_irq_status &= ~I2C_ISR_NACKF; // Mark NACK as processed

			if (i2c_rx_tx_idx > 0) {
				i2c_rx_tx_idx--;
			}

			I2C_ICR(BOARD_I2C) |= I2C_ICR_NACKCF; // Clear the NACK event
			I2C_UART_DEBUG("N", 1);
		}

		if (i2c_irq_status & I2C_ISR_TXIS) { // I2C is waiting for the next byte from us
			i2c_irq_status &= ~I2C_ISR_TXIS; // Mark TX event as processed

			// -1, because we've already sent ACK
			if (i2c_rx_tx_idx < I2C_MAX_PAYLOAD_SIZE - 1) {
				switch (current_command) {
				case I2C_COMMAND_GET_STATUS:
					if (i2c_rx_tx_idx < sizeof(i2c_status_reply)) {
						// Set the next byte to be sent
						I2C_TXDR(BOARD_I2C) = ((uint8_t *) &i2c_status_reply)[i2c_rx_tx_idx++];

					} else {
						I2C_TXDR(BOARD_I2C) = I2C_REPLY_NAK; // Set the next byte to be sent
						current_i2c_state = I2C_WAIT_FOR_STOP;
					}

					break;

				case I2C_COMMAND_GET_PROGRESS:
					I2C_TXDR(BOARD_I2C) = main_bootloader_status; // Set the next byte to be sent
					main_bootloader_status = BOOT_STATUS_IDLE;
					// stop state will repeat the last byte, so just pass control to it
					current_i2c_state = I2C_WAIT_FOR_STOP;
					break;

				case I2C_COMMAND_GET_ERASE_TIME:
				case I2C_COMMAND_GET_WRITE_TIME:
				case I2C_COMMAND_GET_CRC_TIME: {
						uint16_t timeout = FLASH_ERASE_TIME;

						if (current_command == I2C_COMMAND_GET_WRITE_TIME) {
							timeout = FLASH_PROGRAM_BLOCK_TIME;

						} else if (current_command == I2C_COMMAND_GET_CRC_TIME) {
							timeout = CHECK_CRC_TIME;
						}

						// Little-endian uint16_t
						if (i2c_rx_tx_idx < 2) {
							// Set the next byte to be sent
							I2C_TXDR(BOARD_I2C) = BYTE_BY_INDEX(timeout, i2c_rx_tx_idx);
							i2c_rx_tx_idx++;

						} else {
							// Set the next byte to be sent
							I2C_TXDR(BOARD_I2C) = 0; // in little-endian this won't change the value
							current_i2c_state = I2C_WAIT_FOR_STOP;
						}

						break;
					}

				case I2C_COMMAND_GET_WRITE_SIZE:
					// Set the next byte to be sent
					// Assume that I2C bus maximum packet limit is effective both ways
					// Reserve bytes for command, EOC, size and CRC32
					I2C_TXDR(BOARD_I2C) = I2C_MAX_PAYLOAD_SIZE - 7 - (I2C_MAX_PAYLOAD_SIZE - 7) % 4;
					current_i2c_state = I2C_WAIT_FOR_STOP;
					break;

				case I2C_COMMAND_GET_APP_CRC:
					if (i2c_rx_tx_idx < 4) {
						// Set the next byte to be sent
						I2C_TXDR(BOARD_I2C) = BYTE_BY_INDEX(full_app_crc, i2c_rx_tx_idx);
						i2c_rx_tx_idx++;

					} else {
						// Set the next byte to be sent
						I2C_TXDR(BOARD_I2C) = 0; // in little-endian this won't change the value
						current_i2c_state = I2C_WAIT_FOR_STOP;
					}

					break;

				case I2C_COMMAND_CHECK_APP: {
						const uint32_t *app_base = (const uint32_t *)APP_LOAD_ADDRESS;
						uint8_t app_ready = 1;

						/*
						 * We refuse to program the first word of the app until the upload is marked
						 * complete by the host.  So if it's not 0xffffffff, we should try booting it.
						 */
						if (app_base[0] == 0xffffffff) {
							app_ready = 0;
						}

						/*
						 * The second word of the app is the entrypoint; it must point within the
						 * flash area (or we have a bad flash).
						 */
						else if (app_base[1] < APP_LOAD_ADDRESS) {
							app_ready = 0;

						} else if (app_base[1] >= (APP_LOAD_ADDRESS + board_info.fw_size)) {
							app_ready = 0;
						}

						I2C_TXDR(BOARD_I2C) = app_ready; // Set the next byte to be sent
						current_i2c_state = I2C_WAIT_FOR_STOP;
						break;
					}

				default: // No idea how we've got here
					I2C_TXDR(BOARD_I2C) = I2C_REPLY_NAK; // Set the next byte to be sent
					current_i2c_state = I2C_WAIT_FOR_STOP;
				} // switch (current_command)

			} else {
				I2C_TXDR(BOARD_I2C) = I2C_REPLY_NAK; // Set the next byte to be sent
				current_i2c_state = I2C_WAIT_FOR_STOP;
			}

			// If debug is disabled, we still need to read out the register
			volatile char tmp = (char) I2C_TXDR(BOARD_I2C); // Read the byte
			I2C_UART_DEBUG("T", 1);
			I2C_UART_DEBUG((char *)&tmp, 1);
			(void) tmp;
		}

		break;

	case I2C_WAIT_FOR_STOP:

		// We've got a NACK event
		if (i2c_irq_status & I2C_ISR_NACKF) {
			// Repeat the last byte to be sent again
			I2C_TXDR(BOARD_I2C) = I2C_TXDR(BOARD_I2C);
			I2C_ICR(BOARD_I2C) |= I2C_ICR_NACKCF; // Clear the NACK event
			I2C_UART_DEBUG("n", 1);
		}

		// I2C is waiting for another byte from us
		else if (i2c_irq_status & I2C_ISR_TXIS) {
			// Transmit state is responsible for setting NAK if needed,
			// We just repeat the last byte to be sent again
			I2C_TXDR(BOARD_I2C) = I2C_TXDR(BOARD_I2C);
			I2C_UART_DEBUG("t", 1);
		}

		// Received an unexpected byte, can't do much about it
		if (i2c_irq_status & I2C_ISR_RXNE) {
			// If debug is disabled, we still need to read out the register
			volatile char tmp = (char) I2C_RXDR(BOARD_I2C); // Read the byte
			I2C_UART_DEBUG("n", 1);
			I2C_UART_DEBUG((char *)&tmp, 1);
			(void) tmp;
		}

		// All 3 interrupts are tended to in this state
		i2c_irq_status &= ~(I2C_ISR_NACKF | I2C_ISR_TXIS | I2C_ISR_RXNE);

		break;
	} // switch (current_i2c_state)

	// Check if any of interrupt events was left unprocessed by our state-machine
	bool bus_reset_needed = i2c_irq_status & I2C_STATUS_PROCESSED_EVENTS;

	// Received a STOP event, reset the state machine
	if (i2c_irq_status & I2C_ISR_STOPF || bus_reset_needed) {
		// Disable all custom interrupts - we will be waiting for a new communication
		I2C_CR1(BOARD_I2C) &= ~(I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_NACKIE);
		I2C_ISR(BOARD_I2C) |= I2C_ISR_TXE; // flush any data we might have prepared
		I2C_ICR(BOARD_I2C) |= I2C_ICR_STOPCF; // clear the STOP event
		i2c_rx_tx_idx = I2C_MAX_PAYLOAD_SIZE + 1;
		current_i2c_state = I2C_WAIT_FOR_REQUEST;
		current_command = I2C_COMMAND_DEFAULT;
		I2C_UART_DEBUG("S", 1);
	}

	if (bus_reset_needed) {
		I2C_UART_DEBUG("Y", 1);
		I2C_UART_DEBUG((char *) &i2c_irq_status, sizeof(i2c_irq_status));
		// We've messed up. Try to recover at least.
		i2c_perform_reset();
	}

	// Needs to be the last clause in the function as main thread can potentially lock flash reads
	if (current_i2c_state == I2C_WAIT_FOR_REQUEST && new_bootloader_command != BOOT_STATUS_FAILURE) {
		main_bootloader_status = new_bootloader_command;
		new_bootloader_command = BOOT_STATUS_FAILURE;
	}

}

static void
i2c_main_loop(unsigned timeout)
{
	i2c_status_reply.protocol = bl_proto_rev;
	i2c_status_reply.board_type = board_info.board_type;
	i2c_status_reply.board_rev = board_info.board_rev;
	i2c_status_reply.fw_size = board_info.fw_size;

	for (unsigned i = 0; i < sizeof(i2c_status_reply.vec_area) / sizeof(i2c_status_reply.vec_area[0]); i++) {
		i2c_status_reply.vec_area[i] = flash_func_read_word((i + 7) * 4);
	}

	// A little hardcoded hack, if we're here and last status was OK,
	// it means we've tried to boot and failed
	if (main_bootloader_status == BOOT_STATUS_FINISHED_OK) {
		main_bootloader_status = BOOT_STATUS_FAILURE;
	}

	uint32_t address = board_info.fw_size;
	uint32_t	first_word = 0xffffffff;

	i2c_enable(); // Enable I2C only after all the answers have been initialized

	while (true) {

		// Timer is set outside of our loop function
		if (timeout && !timer[TIMER_BL_WAIT]) {
			return;
		}

# ifdef I2C_DEBUG_ENABLE

		if (i2c_debug_start != i2c_debug_end) {
			uint8_t saved_end = i2c_debug_end;

			// overlap
			if (i2c_debug_start > saved_end) {
				DIRECT_UART_DEBUG((uint8_t *) &i2c_debug_buffer[i2c_debug_start], sizeof(i2c_debug_buffer) - i2c_debug_start);
				i2c_debug_start = 0;
			}

			DIRECT_UART_DEBUG((uint8_t *) &i2c_debug_buffer[i2c_debug_start], saved_end - i2c_debug_start);
			i2c_debug_start = saved_end;
		}

		if (i2c_debug_buffer_overflow) {
			DIRECT_UART_DEBUG((uint8_t *) "\n\rDebug buffer overflow!\n\r", 26);
			i2c_debug_buffer_overflow = false;
		}

# endif

		switch (main_bootloader_status) {
		case BOOT_COMMAND_ERASE: {
				main_bootloader_status = BOOT_STATUS_IN_PROGRESS;
				DIRECT_UART_DEBUG((uint8_t *) "\n\rErasing!\n\r", 12);
				int res = perform_chip_erase(&address);
				DIRECT_UART_DEBUG((uint8_t *) "\n\rFinished\n\r", 12);

				if (res == FLASH_COMMAND_OK) {
					main_bootloader_status = BOOT_STATUS_FINISHED_OK;

				} else if (res == FLASH_COMMAND_BAD_SILICON) {
					main_bootloader_status = BOOT_STATUS_BAD_SILICON;

				} else {
					main_bootloader_status = BOOT_STATUS_FAILURE;
				}

				// We could have overwritten the area, refresh the data to avoid doing so during interrupt
				for (unsigned i = 0; i < sizeof(i2c_status_reply.vec_area) / sizeof(i2c_status_reply.vec_area[0]); i++) {
					i2c_status_reply.vec_area[i] = flash_func_read_word((i + 7) * 4);
				}

				break;
			}

		case BOOT_COMMAND_WRITE: {
				main_bootloader_status = BOOT_STATUS_IN_PROGRESS;
				DIRECT_UART_DEBUG((uint8_t *) "\n\rProgramming!\n\r", 16);
# ifdef I2C_DEBUG_ENABLE
				uint32_t program_time_ms = current_time_ms;
# endif

				if ((address + program_multi_count) > board_info.fw_size) {
					DIRECT_UART_DEBUG((uint8_t *) "\n\rOut of bounds\n\r", 17);
					DIRECT_UART_DEBUG((uint8_t *) "Address: ", 9);
					DIRECT_UART_DEBUG((uint8_t *) address, sizeof(address));
					DIRECT_UART_DEBUG((uint8_t *) " size: ", 7);
					DIRECT_UART_DEBUG((uint8_t *) board_info.fw_size, sizeof(board_info.fw_size));
					DIRECT_UART_DEBUG((uint8_t *) "\n\r", 2);
					main_bootloader_status = BOOT_STATUS_BAD_COMMAND;
					break;
				}


				// it's ok to ignore "volatile" nature of flash_buffer here
				if (flash_buffer.w[program_multi_count / 4] != crc32((uint8_t *) flash_buffer.c, program_multi_count, 0)) {
					DIRECT_UART_DEBUG((uint8_t *) "\n\rBad CRC: ", 11);
					DIRECT_UART_DEBUG((uint8_t *)&flash_buffer.w[program_multi_count / 4], sizeof(flash_buffer.w[0]));
					DIRECT_UART_DEBUG((uint8_t *) "\n\r", 2);
# ifdef I2C_DEBUG_ENABLE
					uint32_t correct_crc = crc32((uint8_t *) flash_buffer.c, program_multi_count, 0);
# endif
					DIRECT_UART_DEBUG((uint8_t *) "Correct CRC: ", 13);
					DIRECT_UART_DEBUG((uint8_t *) &correct_crc, sizeof(correct_crc));
					DIRECT_UART_DEBUG((uint8_t *) "\n\r", 2);
					main_bootloader_status = BOOT_STATUS_BAD_COMMAND;
					break;
				}

				enum flash_command_result_t res = perform_flash_write(program_multi_count / 4, &address, &first_word);
# ifdef I2C_DEBUG_ENABLE
				program_time_ms = current_time_ms - program_time_ms;
# endif
				DIRECT_UART_DEBUG((uint8_t *) "\n\rFinished in ", 14);
				DIRECT_UART_DEBUG((uint8_t *) &program_time_ms, sizeof(program_time_ms));
				DIRECT_UART_DEBUG((uint8_t *) "!\n\r", 3);

				if (res == FLASH_COMMAND_OK) {
					main_bootloader_status = BOOT_STATUS_FINISHED_OK;

				} else if (res == FLASH_COMMAND_BAD_SILICON) {
					main_bootloader_status = BOOT_STATUS_BAD_SILICON;

				} else {
					main_bootloader_status = BOOT_STATUS_FAILURE;
				}

				// We could have overwritten the area, refresh the data to avoid doing so during interrupt
				for (unsigned i = 0; i < sizeof(i2c_status_reply.vec_area) / sizeof(i2c_status_reply.vec_area[0]); i++) {
					i2c_status_reply.vec_area[i] = flash_func_read_word((i + 7) * 4);
				}

				break;
			}

		case BOOT_COMMAND_CALC_CRC: {
				main_bootloader_status = BOOT_STATUS_IN_PROGRESS;
# ifdef I2C_DEBUG_ENABLE
				uint32_t crc_time_ms = current_time_ms;
# endif
				full_app_crc = calc_app_crc(first_word);
# ifdef I2C_DEBUG_ENABLE
				crc_time_ms = current_time_ms - crc_time_ms;
# endif
				DIRECT_UART_DEBUG((uint8_t *) "\n\rFinished in ", 14);
				DIRECT_UART_DEBUG((uint8_t *) &crc_time_ms, sizeof(crc_time_ms));
				DIRECT_UART_DEBUG((uint8_t *) "!\n\r", 3);
				main_bootloader_status = BOOT_STATUS_FINISHED_OK;
				break;
			}

		case BOOT_COMMAND_BOOT_APP:
			main_bootloader_status = BOOT_STATUS_IN_PROGRESS;

			// program the deferred first word
			if (first_word != 0xffffffff) {
				flash_func_write_word(0, first_word);

				if (flash_func_read_word(0) != first_word) {
					main_bootloader_status = BOOT_STATUS_FAILURE;
					break;
				}

				// revert in case the flash was bad...
				first_word = 0xffffffff;
			}

			main_bootloader_status = BOOT_STATUS_FINISHED_OK;

			delay(500); // Allow for the caller to check on us
			return;

		default:
			// nop
			break;
		} // switch (main_bootloader_status)
	}

}

#else // if defined(INTERFACE_I2C) && INTERFACE_I2C

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
bad_silicon_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC,			// "in sync"
		PROTO_BAD_SILICON_REV	// "issue with < Rev 3 silicon"
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

/**
 * Function to wait for EOC
 *
 * @param timeout length of time in ms to wait for the EOC to be received
 * @return true if the EOC is returned within the timeout perio, else false
 */
inline static bool
wait_for_eoc(unsigned timeout)
{
	return cin_wait(timeout) == PROTO_EOC;
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

		if (c < 0) {
			return c;
		}

		u.b[i] = c & 0xff;
	}

	*wp = u.w;
	return 0;
}

#endif // if defined(INTERFACE_I2C) && INTERFACE_I2C

void
bootloader(unsigned timeout)
{
	/* (re)start the timer system */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);	/* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();

	/* if we are working with a timeout, start it running */
	if (timeout) {
		timer[TIMER_BL_WAIT] = timeout;
	}

	/* make the LED blink while we are idle */
	led_set(LED_BLINK);

#if defined(INTERFACE_I2C) && INTERFACE_I2C
	bl_type = I2C;
	i2c_main_loop(timeout);
#else
	bl_type = NONE; // The type of the bootloader, whether loading from USB or USART, will be determined by on what port the bootloader recevies its first valid command.

	uint32_t	address = board_info.fw_size;	/* force erase before upload will work */
	uint32_t	first_word = 0xffffffff;

	while (true) {
		volatile int c;
		int arg;

		// Wait for a command byte
		led_off(LED_ACTIVITY);

		do {
			/* if we have a timeout and the timer has expired, return now */
			if (timeout && !timer[TIMER_BL_WAIT]) {
				return;
			}

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
			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}

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

			if (arg < 0) {
				goto cmd_bad;
			}

			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}

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
		case PROTO_CHIP_ERASE: {

				/* expect EOC */
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				enum flash_command_result_t res = perform_chip_erase(&address);

				if (res == FLASH_COMMAND_BAD_SILICON) {
					goto bad_silicon;

				} else if (res != FLASH_COMMAND_OK) {
					goto cmd_fail;
				}

				break;
			}

		// program bytes at current address
		//
		// command:		PROG_MULTI/<len:1>/<data:len>/EOC
		// success reply:	INSYNC/OK
		// invalid reply:	INSYNC/INVALID
		// readback failure:	INSYNC/FAILURE
		//
		case PROTO_PROG_MULTI: {		// program bytes
				// expect count
				arg = cin_wait(50);

				if (arg < 0) {
					goto cmd_bad;
				}

				// sanity-check arguments
				if (arg % 4) {
					goto cmd_bad;
				}

				if ((address + arg) > board_info.fw_size) {
					goto cmd_bad;
				}

				if (arg > sizeof(flash_buffer.c)) {
					goto cmd_bad;
				}

				for (int i = 0; i < arg; i++) {
					c = cin_wait(1000);

					if (c < 0) {
						goto cmd_bad;
					}

					flash_buffer.c[i] = c;
				}

				if (!wait_for_eoc(200)) {
					goto cmd_bad;
				}

				enum flash_command_result_t res = perform_flash_write(arg / 4, &address, &first_word);

				if (res == FLASH_COMMAND_BAD_SILICON) {
					goto bad_silicon;

				} else if (res != FLASH_COMMAND_OK) {
					goto cmd_fail;
				}

				break;
			}

		// fetch CRC of the entire flash area
		//
		// command:			GET_CRC/EOC
		// reply:			<crc:4>/INSYNC/OK
		//
		case PROTO_GET_CRC:

			// expect EOC
			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}

			// compute CRC of the programmed area
			uint32_t sum = calc_app_crc(first_word);

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

				if (cin_word(&index, 100)) {
					goto cmd_bad;
				}

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				cout_word(flash_func_read_otp(index));
			}
			break;

		// read the SN from the UDID
		//
		// command:			GET_SN/<addr:4>/EOC
		// reply:			<value:4>/INSYNC/OK
		case PROTO_GET_SN:
			// expect argument
			{
				uint32_t index = 0;

				if (cin_word(&index, 100)) {
					goto cmd_bad;
				}

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				cout_word(flash_func_read_sn(index));
			}
			break;

		// read the chip ID code
		//
		// command:			GET_CHIP/EOC
		// reply:			<value:4>/INSYNC/OK
		case PROTO_GET_CHIP: {
				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				cout_word(get_mcu_id());
			}
			break;

		// read the chip  description
		//
		// command:			GET_CHIP_DES/EOC
		// reply:			<value:4>/INSYNC/OK
		case PROTO_GET_CHIP_DES: {
				uint8_t buffer[MAX_DES_LENGTH];
				unsigned len = MAX_DES_LENGTH;

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				len = get_mcu_desc(len, buffer);
				cout_word(len);
				cout(buffer, len);
			}
			break;

# ifdef BOOT_DELAY_ADDRESS

		case PROTO_SET_DELAY: {
				/*
				  Allow for the bootloader to setup a
				  boot delay signature which tells the
				  board to delay for at least a
				  specified number of seconds on boot.
				 */
				int v = cin_wait(100);

				if (v < 0) {
					goto cmd_bad;
				}

				uint8_t boot_delay = v & 0xFF;

				if (boot_delay > BOOT_DELAY_MAX) {
					goto cmd_bad;
				}

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);
				uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS + 4);

				if (sig1 != BOOT_DELAY_SIGNATURE1 ||
				    sig2 != BOOT_DELAY_SIGNATURE2) {
					goto cmd_bad;
				}

				uint32_t value = (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00) | boot_delay;
				flash_func_write_word(BOOT_DELAY_ADDRESS, value);

				if (flash_func_read_word(BOOT_DELAY_ADDRESS) != value) {
					goto cmd_fail;
				}
			}
			break;
# endif

		// finalise programming and boot the system
		//
		// command:			BOOT/EOC
		// reply:			INSYNC/OK
		//
		case PROTO_BOOT:

			// expect EOC
			if (!wait_for_eoc(1000)) {
				goto cmd_bad;
			}

			// program the deferred first word
			if (first_word != 0xffffffff) {
				flash_func_write_word(0, first_word);

				if (flash_func_read_word(0) != first_word) {
					goto cmd_fail;
				}

				// revert in case the flash was bad...
				first_word = 0xffffffff;
			}

			// Set the bootloader port in case BOOT is the first command we've received
			if (bl_type == NONE) {
				bl_type = last_input;
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

		// Set the bootloader port based on the port from which we received the first valid command
		if (bl_type == NONE) {
			bl_type = last_input;
		}

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

bad_silicon:
		// send the bad silicon response but don't kill the timeout - could be garbage
		bad_silicon_response();
		continue;
	}

#endif // if defined(INTERFACE_I2C) && INTERFACE_I2C
}
