/*
 * STM32F0 board support for the bootloader.
 *
 */
#include "hw_config.h"

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f0/nvic.h>

#include "bl.h"
#include "uart.h"

// address of MCU IDCODE
#define DBGMCU_IDCODE		0x40015800


#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG		(void *)BOARD_USART
#else
# define BOARD_INTERFACE_CONFIG		NULL
#endif

/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
#ifdef BOARD_REVISION
	.board_rev	= BOARD_REVISION,
#else
	.board_rev	= 0,
#endif
	.fw_size	= APP_SIZE_MAX,

	.systick_mhz	= OSC_FREQ,
};

static void board_init(void);

static void
board_init(void)
{
#ifdef BOARD_CLOCK_LEDS_REGISTER
	/* initialise LEDs */
	rcc_peripheral_enable_clock(&BOARD_CLOCK_LEDS_REGISTER, BOARD_CLOCK_LEDS);
	gpio_mode_setup(BOARD_PORT_LEDS,
			GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE,
			BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	gpio_set_output_options(BOARD_PORT_LEDS,
				GPIO_OTYPE_PP,
				GPIO_OSPEED_HIGH,
				BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	BOARD_LED_ON(
		BOARD_PORT_LEDS,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
#endif

	/* if we have one, enable the force-bootloader pin */
#ifdef BOARD_FORCE_BL_PIN
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);

	gpio_set(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN);
	gpio_mode_setup(BOARD_FORCE_BL_PORT,
			GPIO_MODE_INPUT,
			BOARD_FORCE_BL_PULL,
			BOARD_FORCE_BL_PIN);
#endif

#if INTERFACE_USART
	/* configure USART pins */
	rcc_peripheral_enable_clock(&BOARD_USART_PIN_CLOCK_REGISTER, BOARD_USART_PIN_CLOCK_BIT);

	/* Setup GPIO pins for USART transmit. */
	gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BOARD_PIN_TX | BOARD_PIN_RX);
	/* Setup USART TX & RX pins as alternate function. */
	gpio_set_af(BOARD_PORT_USART, BOARD_PORT_USART_AF, BOARD_PIN_TX);
	gpio_set_af(BOARD_PORT_USART, BOARD_PORT_USART_AF, BOARD_PIN_RX);

	/* configure USART clock */
	rcc_peripheral_enable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif
#if defined(INTERFACE_I2C) && INTERFACE_I2C
	/* configure I2C pins */
	rcc_peripheral_enable_clock(&BOARD_I2C_PIN_CLOCK_REGISTER, BOARD_I2C_PIN_CLOCK_BIT);

	/* Setup GPIO pins for I2C. */
	gpio_mode_setup(BOARD_PORT_I2C, GPIO_MODE_AF, GPIO_PUPD_NONE, BOARD_PORT_I2C_PIN_SCL | BOARD_PORT_I2C_PIN_SDA);
	/* Setup I2C SCL & SDA pins as alternate function. */
	gpio_set_af(BOARD_PORT_I2C, BOARD_PORT_I2C_AF, BOARD_PORT_I2C_PIN_SCL | BOARD_PORT_I2C_PIN_SDA);

	gpio_set_output_options(BOARD_PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_MED,
				BOARD_PORT_I2C_PIN_SCL | BOARD_PORT_I2C_PIN_SDA);

	/* configure I2C clock */
	BOARD_I2C_CLOCK_SOURCE_REGISTER |= BOARD_I2C_CLOCK_SOURCE_BIT; // set system clock as source for I2C
	rcc_peripheral_enable_clock(&BOARD_I2C_CLOCK_REGISTER, BOARD_I2C_CLOCK_BIT);
#endif
}

void
board_deinit(void)
{
#ifdef BOARD_CLOCK_LEDS_REGISTER
	/* deinitialise LEDs */
	gpio_mode_setup(BOARD_PORT_LEDS,
			GPIO_MODE_INPUT,
			GPIO_PUPD_NONE,
			BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
#endif

	/* if we have one, disable the force-bootloader pin */
#ifdef BOARD_FORCE_BL_PIN
	gpio_mode_setup(BOARD_FORCE_BL_PORT,
			GPIO_MODE_INPUT,
			GPIO_PUPD_NONE,
			BOARD_FORCE_BL_PIN);
	gpio_clear(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN);
#endif

#if INTERFACE_USART
	/* deinitialise GPIO pins for USART transmit. */
	gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_TX | BOARD_PIN_RX);

	/* disable USART peripheral clock */
	rcc_peripheral_disable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif
#if defined(INTERFACE_I2C) && INTERFACE_I2C
	// Disable all the triggers we might have enabled
	I2C_CR1(BOARD_I2C) &= ~(I2C_CR1_PE | I2C_CR1_ADDRIE | I2C_CR1_ERRIE
				| I2C_CR1_STOPIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_NACKIE);

	// disable clocks
	rcc_peripheral_disable_clock(&BOARD_I2C_PIN_CLOCK_REGISTER, BOARD_I2C_PIN_CLOCK_BIT);
	rcc_peripheral_disable_clock(&BOARD_I2C_CLOCK_REGISTER, BOARD_I2C_CLOCK_BIT);

	// reset GPIO config
	gpio_mode_setup(BOARD_PORT_I2C, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PORT_I2C_PIN_SCL | BOARD_PORT_I2C_PIN_SDA);
	gpio_set_af(BOARD_PORT_I2C, GPIO_AF0, BOARD_PORT_I2C_PIN_SCL | BOARD_PORT_I2C_PIN_SDA);
	gpio_set_output_options(BOARD_PORT_I2C, GPIO_OTYPE_PP, GPIO_OSPEED_LOW,
				BOARD_PORT_I2C_PIN_SCL | BOARD_PORT_I2C_PIN_SDA);

	// reset I2C configuration
	BOARD_I2C_CLOCK_SOURCE_REGISTER &= ~BOARD_I2C_CLOCK_SOURCE_BIT;
	I2C_TIMINGR(BOARD_I2C) = 0;
	nvic_disable_irq(BOARD_I2C_IRQ_NUMBER);
	I2C_OAR1(BOARD_I2C) = 0;

#endif

	/* reset the APB2 peripheral clocks */
	RCC_APB2ENR = 0x00000000; // XXX Magic reset number from STM32F0x reference manual

	/* reset then AHB peripheral clocks */
	RCC_AHBENR = 0x00000014; // XXX Magic reset number from STM32F0x reference manual
}

/**
  * @brief  Initializes the RCC clock configuration.
  *
  * @param  clock_setup : The clock configuration to set
  */
static inline void
clock_init(void)
{
#if defined(INTERFACE_USB) && INTERFACE_USB != 0
	rcc_clock_setup_in_hsi_out_48mhz();
#else
	rcc_clock_setup_in_hsi_out_24mhz();
#endif
}

/**
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *            - HSI ON and used as system clock source
  *            - HSE, PLL and PLLI2S OFF
  *            - AHB, APB1 and APB2 prescaler set to 1.
  *            - CSS, MCO1 and MCO2 OFF
  *            - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *            - Peripheral clocks
  *            - LSI, LSE and RTC clocks
  */
void
clock_deinit(void)
{
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Reset the RCC_CFGR register */
	RCC_CFGR = 0x000000;

	/* Stop the HSE, CSS, PLL, PLLI2S, PLLSAI */
	rcc_osc_off(RCC_HSE);
	rcc_osc_off(RCC_PLL);
	rcc_css_disable();

	/* Reset the HSEBYP bit */
	rcc_osc_bypass_disable(RCC_HSE);

	/* Reset the CIR register */
	RCC_CIR = 0x000000;
}

#if defined(INTERFACE_I2C) && INTERFACE_I2C
void
i2c_enable(void)
{
	// Configure I2C timings
	I2C_TIMINGR(BOARD_I2C) =
		0x50420F13; // Set from "Table 76. Examples of timings settings" from reference to 100Hz mode, scaled to 24MHz instead of 48MHz
	// Enable interface and address, error and stop interrupts
	nvic_enable_irq(BOARD_I2C_IRQ_NUMBER);
	I2C_CR1(BOARD_I2C) = I2C_CR1_PE | I2C_CR1_ADDRIE | I2C_CR1_ERRIE | I2C_CR1_STOPIE;
	// Set own address for slave comms
	I2C_OAR1(BOARD_I2C) |= I2C_OAR1_OA1_VAL(I2C_OWN_ADDRESS << 1); // 7-bit addressing shifts by 1 bit
	I2C_OAR1(BOARD_I2C) |= I2C_OAR1_OA1EN; // enable the address
}

void
i2c_perform_reset(void)
{
	// Disable I2C interface
	I2C_CR1(BOARD_I2C) &= ~I2C_CR1_PE;

	/* Reference states that PE must be kept low for at least 3 cycles
	 * And suggests to do it by writing 0, checking 0 and then writing 1
	 */
	if (!(I2C_CR1(BOARD_I2C) & I2C_CR1_PE)) {
		I2C_CR1(BOARD_I2C) |= I2C_CR1_PE;
	}
}
#endif

/* Bootloader API uses "sector", whereas F0 reference manual refers to the same thing as
 * "pages" and refers to bigger units as "sectors".
 */
uint32_t
flash_func_sector_size(unsigned page)
{
	if (page < BOARD_FLASH_PAGES) {
		return FLASH_PAGE_SIZE;
	}

	return 0;
}

/* Bootloader API uses "sector", whereas F0 reference manual refers to the same thing as
 * "pages" and refers to bigger units as "sectors".
 */
void
flash_func_erase_sector(unsigned page)
{
	if (page < BOARD_FLASH_PAGES) {
		flash_erase_page(APP_LOAD_ADDRESS + (page * FLASH_PAGE_SIZE));
	}
}

void
flash_func_write_word(uint32_t address, uint32_t word)
{
	flash_program_word(address + APP_LOAD_ADDRESS, word);
}

uint32_t
flash_func_read_word(uint32_t address)
{
	return *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

uint32_t
flash_func_read_otp(uint32_t address)
{
	return 0;
}

uint32_t get_mcu_id(void)
{
	return *(uint32_t *)DBGMCU_IDCODE;
}

// See F4 version for future enhancement for full decoding

int get_mcu_desc(int max, uint8_t *revstr)
{
	const char none[] = "STM32F0xxx,?";
	int i;

	for (i = 0; none[i] && i < max - 1; i++) {
		revstr[i] = none[i];
	}

	return i;
}

int check_silicon(void)
{
	return 0;
}

uint32_t
flash_func_read_sn(uint32_t address)
{
	// F0 chips don't have a serial
	return 0;
}

void
led_on(unsigned led)
{
#ifdef BOARD_CLOCK_LEDS_REGISTER

	switch (led) {
	case LED_ACTIVITY:
		BOARD_LED_ON(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;

	case LED_BOOTLOADER:
		BOARD_LED_ON(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}

#endif
}

void
led_off(unsigned led)
{
#ifdef BOARD_CLOCK_LEDS_REGISTER

	switch (led) {
	case LED_ACTIVITY:
		BOARD_LED_OFF(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;

	case LED_BOOTLOADER:
		BOARD_LED_OFF(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}

#endif
}

void
led_toggle(unsigned led)
{
#ifdef BOARD_CLOCK_LEDS_REGISTER

	switch (led) {
	case LED_ACTIVITY:
		gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;

	case LED_BOOTLOADER:
		gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}

#endif
}

int
main(void)
{
	unsigned timeout = 0;

	/* do board-specific initialisation */
	board_init();

#if (defined(INTERFACE_I2C) && INTERFACE_I2C || defined(INTERFACE_USART) && INTERFACE_USART || defined (INTERFACE_USB) && INTERFACE_USB) && !defined(START_APP_ONLY_ON_COMMAND)
	/* XXX sniff for a USART connection to decide whether to wait in the bootloader? */
	timeout = BOOTLOADER_DELAY;
#endif

#ifdef BOARD_FORCE_BL_PIN

	/* if the force-BL pin state matches the state of the pin, wait in the bootloader forever */
	if (BOARD_FORCE_BL_VALUE == gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN)) {
		timeout = 0xffffffff;
	}

#endif

#ifdef START_APP_ONLY_ON_COMMAND
	timeout = 0;
#else

	/* if we aren't expected to wait in the bootloader, try to boot immediately */
	if (timeout == 0) {
		/* try to boot immediately */
		jump_to_app();

		/* if we returned, there is no app; go to the bootloader and stay there */
		timeout = 0;
	}

#endif

	/* configure the clock for bootloader activity */
	clock_init();

	/* start the interface */
	cinit(BOARD_INTERFACE_CONFIG, USART);
#ifdef I2C_DEBUG_ENABLE
	uart_cout((uint8_t *) "\r\nI am boot!\r\n", 14);
#endif

	while (1) {
		/* run the bootloader, possibly coming back after the timeout */
		bootloader(timeout);

#ifdef I2C_DEBUG_ENABLE
		// During debug prints we might have sent "stop transmitting"
		// Control character. This would kill TX channel of the debugger
		// So we need to send "continue transmitting" character
		uint8_t tmp = 17; // "continue transmitting"
		uart_cout(&tmp, 1);
		uart_cout((uint8_t *) "\r\nStarting app\r\n", 16);
#endif

		/* look to see if we can boot the app */
		jump_to_app();

		/* boot failed; stay in the bootloader forever next time */
		timeout = 0;
	}
}
