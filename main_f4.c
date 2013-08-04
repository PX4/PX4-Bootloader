/*
 * STM32F4 board support for the bootloader.
 *
 */

#include <stdlib.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/systick.h>
#include <libopencm3/stm32/pwr.h>

#include "bl.h"

/* flash parameters that we should not really know */
static struct {
	uint32_t	erase_code;
	unsigned	size;
} flash_sectors[] = {
	/* flash sector zero reserved for bootloader */
	{ (0x01 << 3), 16 * 1024},
	{ (0x02 << 3), 16 * 1024},
	{ (0x03 << 3), 16 * 1024},
	{ (0x04 << 3), 64 * 1024},
	{ (0x05 << 3), 128 * 1024},
	{ (0x06 << 3), 128 * 1024},
	{ (0x07 << 3), 128 * 1024},
	{ (0x08 << 3), 128 * 1024},
	{ (0x09 << 3), 128 * 1024},
	{ (0x0a << 3), 128 * 1024},
	{ (0x0b << 3), 128 * 1024},
	/* flash sectors only in 2MiB devices */
	{ (0x10 << 3), 16 * 1024},
	{ (0x11 << 3), 16 * 1024},
	{ (0x12 << 3), 16 * 1024},
	{ (0x13 << 3), 16 * 1024},
	{ (0x14 << 3), 64 * 1024},
	{ (0x15 << 3), 128 * 1024},
	{ (0x16 << 3), 128 * 1024},
	{ (0x17 << 3), 128 * 1024},
	{ (0x18 << 3), 128 * 1024},
	{ (0x19 << 3), 128 * 1024},
	{ (0x1a << 3), 128 * 1024},
	{ (0x1b << 3), 128 * 1024},
};
#define BOOTLOADER_RESERVATION_SIZE	(16 * 1024)

#ifdef BOARD_FMU
# define BOARD_TYPE			5
# define BOARD_FLASH_SECTORS		11
# define BOARD_FLASH_SIZE		(1024 * 1024)

# define OSC_FREQ			24

# define BOARD_PIN_LED_ACTIVITY		GPIO15
# define BOARD_PIN_LED_BOOTLOADER	GPIO14
# define BOARD_PORT_LEDS		GPIOB
# define BOARD_CLOCK_LEDS		RCC_AHB1ENR_IOPBEN
# define BOARD_LED_ON			gpio_clear
# define BOARD_LED_OFF			gpio_set
#endif

#ifdef BOARD_FLOW
# define BOARD_TYPE			6
# define BOARD_FLASH_SECTORS		11
# define BOARD_FLASH_SIZE		(1024 * 1024)

# define OSC_FREQ			24

# define BOARD_PIN_LED_ACTIVITY		GPIO3
# define BOARD_PIN_LED_BOOTLOADER	GPIO2
# define BOARD_PORT_LEDS		GPIOE
# define BOARD_CLOCK_LEDS		RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON			gpio_clear
# define BOARD_LED_OFF			gpio_set
#endif

#ifdef BOARD_DISCOVERY
# define BOARD_TYPE			99
# define BOARD_FLASH_SECTORS		11
# define BOARD_FLASH_SIZE		(1024 * 1024)

# define OSC_FREQ			8

# define BOARD_PIN_LED_ACTIVITY		GPIO12
# define BOARD_PIN_LED_BOOTLOADER	GPIO13
# define BOARD_PORT_LEDS		GPIOD
# define BOARD_CLOCK_LEDS		RCC_AHB1ENR_IOPDEN
# define BOARD_LED_ON			gpio_set
# define BOARD_LED_OFF			gpio_clear
#endif

#ifdef BOARD_FMUV2
# define BOARD_TYPE			9
# define _FLASH_KBYTES			(*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS		((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE		(_FLASH_KBYTES * 1024)

# define OSC_FREQ			24

# define BOARD_PIN_LED_ACTIVITY		0		// no activity LED
# define BOARD_PIN_LED_BOOTLOADER	GPIO12
# define BOARD_PORT_LEDS		GPIOE
# define BOARD_CLOCK_LEDS		RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON			gpio_clear
# define BOARD_LED_OFF			gpio_set

# define BOARD_FORCE_BL_PIN_OUT		GPIO14
# define BOARD_FORCE_BL_PIN_IN		GPIO11
# define BOARD_FORCE_BL_PORT		GPIOE
# define BOARD_FORCE_BL_CLOCK_REGISTER	RCC_AHB1ENR
# define BOARD_FORCE_BL_CLOCK_BIT	RCC_AHB1ENR_IOPEEN
# define BOARD_FORCE_BL_PULL		GPIO_PUPD_PULLUP

//# define BOARD_BOOT_FAIL_DETECT		/* V2 boards should support boot failure detection */
#endif


#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - BOOTLOADER_RESERVATION_SIZE)

/* context passed to cinit */
#define BOARD_INTERFACE_CONFIG		NULL

/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,

	.systick_mhz	= 168,
};

static void board_init(void);

#define BOOT_RTC_SIGNATURE	0xb007b007
#define BOOT_RTC_REG		MMIO32(RTC_BASE + 0x50)

/* standard clocking for all F4 boards */
static const clock_scale_t clock_setup =
{
	.pllm = OSC_FREQ,
	.plln = 336,
	.pllp = 2,
	.pllq = 7,
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE_DIV_4,
	.ppre2 = RCC_CFGR_PPRE_DIV_2,
	.power_save = 0,
	.flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_5WS,
	.apb1_frequency = 42000000,
	.apb2_frequency = 84000000,
};

static uint32_t
board_get_rtc_signature()
{
	/* enable the backup registers */
	PWR_CR |= PWR_CR_DBP;
	RCC_BDCR |= RCC_BDCR_RTCEN;

	uint32_t result = BOOT_RTC_REG;

	/* disable the backup registers */
	RCC_BDCR &= RCC_BDCR_RTCEN;
	PWR_CR &= ~PWR_CR_DBP;

	return result;
}

static void
board_set_rtc_signature(uint32_t sig)
{
	/* enable the backup registers */
	PWR_CR |= PWR_CR_DBP;
	RCC_BDCR |= RCC_BDCR_RTCEN;

	BOOT_RTC_REG = sig;

	/* disable the backup registers */
	RCC_BDCR &= RCC_BDCR_RTCEN;
	PWR_CR &= ~PWR_CR_DBP;
}

static bool
board_test_force_pin()
{
#ifdef BOARD_FORCE_BL_PIN_IN
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	/* (re)configure the force BL pins */
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN_IN);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_OUT);
	gpio_set_output_options(BOARD_FORCE_BL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, BOARD_FORCE_BL_PIN_OUT);

	for (volatile unsigned cycles = 0; cycles < 10; cycles++) {
		gpio_set(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_OUT);
		for (unsigned count = 0; count < 20; count++) {
			if (gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_IN) != 0)
				vote++;
			samples++;
		}
		gpio_clear(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_OUT);
		for (unsigned count = 0; count < 20; count++) {
			if (gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_IN) == 0)
				vote++;
			samples++;
		}
	}
	/* revert the driver pin */
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_OUT);

	/* the idea here is to reject wire-to-wire coupling, so require > 90% agreement */
	if ((vote * 100) > (samples * 90))
		return true;
#endif

	return false;
}

static void
board_init(void)
{
	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX,

#ifdef INTERFACE_USB
	/* enable GPIO9 with a pulldown to sniff VBUS */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO9);
#endif

	/* initialise LEDs */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, BOARD_CLOCK_LEDS);
	gpio_mode_setup(
		BOARD_PORT_LEDS, 
		GPIO_MODE_OUTPUT, 
		GPIO_PUPD_NONE,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	gpio_set_output_options(
		BOARD_PORT_LEDS,
		GPIO_OTYPE_PP,
		GPIO_OSPEED_2MHZ,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	BOARD_LED_ON (
		BOARD_PORT_LEDS,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

	/* enable the power controller clock */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);

}



unsigned
flash_func_sector_size(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS)
		return flash_sectors[sector].size;
	return 0;
}

void
flash_func_erase_sector(unsigned sector)
{
	if (sector >= BOARD_FLASH_SECTORS)
		return;

	/* get the base address of the sector */
	unsigned address = 0;
	for (unsigned i = 0; i < sector; i++)
		address += flash_func_sector_size(i);

	/* blank-check the sector */
	unsigned size = flash_func_sector_size(sector);
	bool blank = true;
	for (unsigned i = 0; i < size; i += sizeof(uint32_t)) {
		if (flash_func_read_word(address + i) != 0xffffffff) {
			blank = false;
			break;
		}
	}

	/* erase the sector if it failed the blank check */
	if (!blank)
		flash_erase_sector(flash_sectors[sector].erase_code, FLASH_PROGRAM_X32);
}

void
flash_func_write_word(unsigned address, uint32_t word)
{
	flash_program_word(address + APP_LOAD_ADDRESS, word, FLASH_PROGRAM_X32);
}

uint32_t 
flash_func_read_word(unsigned address)
{
	return *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

void
led_on(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
		BOARD_LED_ON (BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;
	case LED_BOOTLOADER:
		BOARD_LED_ON (BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}
}

void
led_off(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
		BOARD_LED_OFF (BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;
	case LED_BOOTLOADER:
		BOARD_LED_OFF (BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
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

/* we should know this, but we don't */
#ifndef SCB_CPACR
# define SCB_CPACR (*((volatile uint32_t *) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif

int
main(void)
{
	bool try_boot = true;			/* try booting before we drop to the bootloader */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

	/* Enable the FPU before we hit any FP instructions */
	SCB_CPACR |= ((3UL << 10*2) | (3UL << 11*2)); /* set CP10 Full Access and set CP11 Full Access */

	/* do board-specific initialisation */
	board_init();

	/* 
	 * Check the force-bootloader register; if we find the signature there, don't
	 * try booting.
	 */
	if (board_get_rtc_signature() == BOOT_RTC_SIGNATURE) {

		/*
		 * Don't even try to boot before dropping to the bootloader.
		 */
		try_boot = false;

		/*
		 * Don't drop out of the bootloader until something has been uploaded.
		 */
		timeout = 0;

		/* 
		 * Clear the signature so that if someone resets us while we're
		 * in the bootloader we'll try to boot next time.
		 */
		board_set_rtc_signature(0);
	}

#ifdef INTERFACE_USB
	/*
	 * Check for USB connection - if present, don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
	if (gpio_get(GPIOA, GPIO9) != 0) {

		/* don't try booting before we set up the bootloader */
		try_boot = false;
	}
#endif

	/* Try to boot the app if we think we should just go straight there */
	if (try_boot) {

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* try to boot immediately */
		jump_to_app();

		/* booting failed, stay in the bootloader forever */
		timeout = 0;
	}

	/* configure the clock for bootloader activity */
	rcc_clock_setup_hse_3v3(&clock_setup);

	/* start the interface */
	cinit(BOARD_INTERFACE_CONFIG);

#if 0
	// MCO1/02
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8);
	gpio_set_af(GPIOA, GPIO_AF0, GPIO8);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF0, GPIO9);
#endif

	while (1)
	{
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader(timeout);

		/* if the force-bootloader pins are strapped, just loop back */
		if (board_test_force_pin())
			continue;

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* look to see if we can boot the app */
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
}
