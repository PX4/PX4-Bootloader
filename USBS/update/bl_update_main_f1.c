/*
 * STM32F1 board support for the bootloader.
 *
 */
#include "usbs_hw_config.h"

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/bkp.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

#include "usbs_bl.h"

#define UDID_START      0x1FFFF7E8

// address of MCU IDCODE
#define DBGMCU_IDCODE		0xE0042000


#ifdef INTERFACE_USART
# define BOARD_INTERFACE_CONFIG		(void *)BOARD_USART
#else
# define BOARD_INTERFACE_CONFIG		NULL
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  	NULL
#endif
#if 0
/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= APP_SIZE_MAX,

	.systick_mhz	= OSC_FREQ,
};
#else
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= APP_SIZE_MAX,
#if defined(INTERFACE_USB)
	.systick_mhz	= 48,
#else
	.systick_mhz	= OSC_FREQ,
#endif
	.hw_bl_rev     = (HW_BOARD_REV & 0xFF000000) | (BL_REVR & 0x00FF0000) | (BL_REVX & 0x0000FF00) | (BL_REVY & 0x000000FF),
	.hw_name    = HW_NAME,
};
#endif
void board_init(void);

void
board_init(void)
{
	/* initialise LEDs */
	rcc_peripheral_enable_clock(&BOARD_CLOCK_LEDS_REGISTER, BOARD_CLOCK_LEDS);
	gpio_set_mode(BOARD_PORT_LEDS,
		      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
		      BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	BOARD_LED_ON(
		BOARD_PORT_LEDS,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

	/* if we have one, enable the force-bootloader pin */
#ifdef BOARD_FORCE_BL_PIN
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);

	gpio_set(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN);
	gpio_set_mode(BOARD_FORCE_BL_PORT,
		      GPIO_MODE_INPUT,
		      BOARD_FORCE_BL_PULL,
		      BOARD_FORCE_BL_PIN);
#endif

	/* enable the backup registers */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN);

#ifdef INTERFACE_USART
	/* configure usart pins */
	rcc_peripheral_enable_clock(&BOARD_USART_PIN_CLOCK_REGISTER, BOARD_USART_PIN_CLOCK_BIT);
	gpio_set_mode(BOARD_PORT_USART,
		      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      BOARD_PIN_TX);

	/* configure USART clock */
	rcc_peripheral_enable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif
#ifdef INTERFACE_I2C
# error I2C GPIO config not handled yet
#endif
}

void
board_deinit(void)
{
	/* deinitialise LEDs */
	gpio_set_mode(BOARD_PORT_LEDS,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

	/* if we have one, disable the force-bootloader pin */
#ifdef BOARD_FORCE_BL_PIN
	gpio_set_mode(BOARD_FORCE_BL_PORT,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      BOARD_FORCE_BL_PIN);
	gpio_clear(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN);
#endif

	/* disable the backup registers */
	rcc_peripheral_disable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN);

#ifdef INTERFACE_USART
	/* configure usart pins */
	gpio_set_mode(BOARD_PORT_USART,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      BOARD_PIN_TX);

	/* disable USART peripheral clock */
	rcc_peripheral_disable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif
#ifdef INTERFACE_I2C
# error I2C GPIO config not handled yet
#endif

	/* reset the APB2 peripheral clocks */
	RCC_APB2ENR = 0x00000000; // XXX Magic reset number from STM32F1x reference manual
}
void board_deinit_standby(void)
{
	/* if we have one, disable the force-bootloader pin */
#ifdef BOARD_FORCE_BL_PIN
	gpio_set_mode(BOARD_FORCE_BL_PORT,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      BOARD_FORCE_BL_PIN);
	gpio_clear(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN);
#endif

	/* disable the backup registers */
	rcc_peripheral_disable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN);

#ifdef INTERFACE_USART
	/* configure usart pins */
	gpio_set_mode(BOARD_PORT_USART,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      BOARD_PIN_TX);

	/* disable USART peripheral clock */
	rcc_peripheral_disable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif
#ifdef INTERFACE_I2C
# error I2C GPIO config not handled yet
#endif
}
/**
  * @brief  Initializes the RCC clock configuration.
  *
  * @param  clock_setup : The clock configuration to set
  */
#if 0
static inline void
clock_init(void)
#else
void clock_init(void)
#endif
{
#if defined(INTERFACE_USB)
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

uint32_t
flash_func_sector_size(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS) {
		return FLASH_SECTOR_SIZE;
	}

	return 0;
}

void
flash_func_erase_sector(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS) {
		flash_erase_page(APP_LOAD_ADDRESS + (sector * FLASH_SECTOR_SIZE));
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
	const char none[] = "STM32F1xxx,?";
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
	// read a byte out from unique chip ID area
	// it's 12 bytes, or 3 words.
	return *(uint32_t *)(address + UDID_START);
}

void
led_on(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
		BOARD_LED_ON(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;

	case LED_BOOTLOADER:
		BOARD_LED_ON(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}
}

void
led_off(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
		BOARD_LED_OFF(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;

	case LED_BOOTLOADER:
		BOARD_LED_OFF(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}
}

void
led_toggle(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
		gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;

	case LED_BOOTLOADER:
		gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}
}

uint32_t code[16*1024/4]; // 16K
extern uint16_t bl_flash_get32(uint32_t code[], const uint16_t _size, const uint8_t set_head);
#define FLASH_START_ADDRESS    0x08000000
int main(void)
{
	uint32_t	address = 0;
	uint32_t len = 0;//(APP_LOAD_ADDRESS&0x7FFF)/4; // 最大16K
	uint32_t i=0;
	len = (APP_LOAD_ADDRESS&0x7FFF)/4; // 最大16K
	if(len>=0x4000) len=0x4000;
	/* do board-specific initialisation */
	board_init();
	clock_init();
	
	// copy code
	bl_flash_get32(NULL, 0, 1); // init
	bl_flash_get32(code, sizeof(code)/4, 0);

	// erase all sectors
	flash_unlock();
	if(0x400 == FLASH_SECTOR_SIZE)  // page = 1K
	{
		for(i=0; i<12; i++) // 擦除 12K扇区
		{
			flash_erase_page(FLASH_START_ADDRESS + (i * FLASH_SECTOR_SIZE));   // [0-0x400]
		}
	}
	else // 大容量和互联型产品,page = 2K
	{
		for(i=0; i<6; i++) // 擦除 12K扇区
		{
			flash_erase_page(FLASH_START_ADDRESS + (i * FLASH_SECTOR_SIZE));   // [0-0x400]
		}
	}
	address=0;
	for (i = 0; i < len; i++) {
		// program the word
		flash_program_word(address + FLASH_START_ADDRESS, code[i]);
		address += 4;
	}
	flash_erase_page(FLASH_START_ADDRESS + (APP_LOAD_ADDRESS&0x7FFF));   // 擦除自身所在扇区
#if 1//1

	/* just for paranoia's sake */
	flash_lock();
	//while(1);
	scb_reset_system();
#else
	jump_to_app();
#endif
	
	while (1) {
		;
	}
}
