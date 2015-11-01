/*
 * STM32F4 board support for the bootloader.
 *
 */

#include "hw_config.h"

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/pwr.h>
# include <libopencm3/stm32/timer.h>

#include "bl.h"
#include "uart.h"

/* flash parameters that we should not really know */
static struct {
	uint32_t	sector_number;
	uint32_t	size;
} flash_sectors[] = {
	/* flash sector zero reserved for bootloader */
		{0x01, 16 * 1024},
		{0x02, 16 * 1024},
		{0x03, 16 * 1024},
		{0x04, 64 * 1024},
		{0x05, 128 * 1024},
		{0x06, 128 * 1024},
		{0x07, 128 * 1024},
		{0x08, 128 * 1024},
		{0x09, 128 * 1024},
		{0x0a, 128 * 1024},
		{0x0b, 128 * 1024},
		/* flash sectors only in 2MiB devices */
		{0x10, 16 * 1024},
		{0x11, 16 * 1024},
		{0x12, 16 * 1024},
		{0x13, 16 * 1024},
		{0x14, 64 * 1024},
		{0x15, 128 * 1024},
		{0x16, 128 * 1024},
		{0x17, 128 * 1024},
		{0x18, 128 * 1024},
		{0x19, 128 * 1024},
		{0x1a, 128 * 1024},
		{0x1b, 128 * 1024},
};
#define BOOTLOADER_RESERVATION_SIZE	(16 * 1024)

#define OTP_BASE			0x1fff7800
#define OTP_SIZE			512
#define UDID_START		        0x1FFF7A10



#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - BOOTLOADER_RESERVATION_SIZE)

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	(void *)BOARD_USART
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  	NULL
#endif

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
	.flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_5WS,
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
#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* two pins strapped together */
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
	/* revert the pins */
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_OUT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_IN);

	/* the idea here is to reject wire-to-wire coupling, so require > 90% agreement */
	if ((vote * 100) > (samples * 90))
		return true;
#endif
#if defined(BOARD_FORCE_BL_PIN)
	/* single pin pulled up or down */
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	/* (re)configure the force BL pins */
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN);

	for (samples = 0; samples < 200; samples++) {
		if ((gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN) ? 1 : 0) == BOARD_FORCE_BL_STATE)
			vote++;
	}

	/* revert the pin */
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN);

	/* reject a little noise */
	if ((vote * 100) > (samples * 90))
		return true;
#endif
	return false;
}

#if INTERFACE_USART
static bool
board_test_usart_receiving_break()
{
	/* (re)start the SysTick timer system */
	systick_interrupt_disable(); // Kill the interrupt if it is still active
	systick_counter_disable(); // Stop the timer
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);

	/* Set the timer period to be half the bit rate
	 *
	 * Baud rate = 115200, therefore bit period = 8.68us
	 * Half the bit rate = 4.34us
	 * Set period to 4.34 microseconds (timer_period = timer_tick / timer_reset_frequency = 168MHz / (1/4.34us) = 729.12 ~= 729)
	 */
	systick_set_reload(729);  /* 4.3us tick, magic number */
	systick_counter_enable(); // Start the timer 
		
	uint8_t cnt_consecutive_low = 0;
	uint8_t cnt = 0;

	/* Loop for 3 transmission byte cycles and count the low and high bits. Sampled at a rate to be able to count each bit twice.
	 * 
	 * One transmission byte is 10 bits (8 bytes of data + 1 start bit + 1 stop bit)
	 * We sample at every half bit time, therefore 20 samples per transmission byte,
	 * therefore 60 samples for 3 transmission bytes
	 */
	while (cnt < 60)
	{
		// Only read pin when SysTick timer is true
		if (systick_get_countflag() == 1)
    		{
			if(gpio_get(BOARD_PORT_USART, BOARD_PIN_RX) == 0)
			{
				cnt_consecutive_low++;	// Increment the consecutive low counter		
			}
			else
			{
				cnt_consecutive_low = 0; // Reset the consecutive low counter				
			}

			cnt++;
		}

		// If 9 consecutive low bits were received break out of the loop
		if(cnt_consecutive_low >= 18)
		{
			break;
		}
		
	}

	systick_counter_disable(); // Stop the timer

	/*
	 * If a break is detected, return true, else false
	 *
	 * Break is detected if line was low for 9 consecutive bits.
	 */ 
	if(cnt_consecutive_low >= 18)
	{
		return true;
	}

	return false;
}
#endif

static void
board_init(void)
{
	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX,

#if INTERFACE_USB
	/* enable GPIO9 with a pulldown to sniff VBUS */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO9);
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

void
board_deinit(void)
{
#if INTERFACE_USB
	/* disable GPIO9 (used to sniff VBUS) */
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO9);		
#endif

#if INTERFACE_USART
	  /* Setup GPIO pins for USART transmit. */
	  gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_TX | BOARD_PIN_RX);

	  /* disable USART peripheral clock */
	  rcc_peripheral_disable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif

	/* deinitialise LEDs */	
	gpio_mode_setup(
		BOARD_PORT_LEDS, 
		GPIO_MODE_INPUT, 
		GPIO_PUPD_NONE,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	
	/* disable the power controller clock */
	rcc_peripheral_disable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);

	/* disable the GPIO port peripheral clocks */
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPFEN);
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPGEN);
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPHEN);
	rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPIEN);
}

/**
  * @brief  Initializes the RCC clock configuration.
  *
  * @param  clock_setup : The clock configuration to set
  */
static inline void
clock_init(void)
{
	rcc_clock_setup_hse_3v3(&clock_setup);
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
	rcc_osc_on(HSI);
	rcc_wait_for_osc_ready(HSI);

	/* Reset the RCC_CFGR register */
	RCC_CFGR = 0x000000;

	/* Stop the HSE, CSS, PLL, PLLI2S, PLLSAI */
	rcc_osc_off(HSE);
	rcc_osc_off(PLL);
	rcc_css_disable();

	/* Reset the RCC_PLLCFGR register */
	RCC_PLLCFGR = 0x24003010; // XXX Magic reset number from STM32F4xx reference manual

	/* Reset the HSEBYP bit */
	rcc_osc_bypass_disable(HSE);

	/* Reset the CIR register */
	RCC_CIR = 0x000000;	
}

uint32_t
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
	uint32_t address = 0;
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
		flash_erase_sector(flash_sectors[sector].sector_number, FLASH_CR_PROGRAM_X32);
}

void
flash_func_write_word(uint32_t address, uint32_t word)
{
	flash_program_word(address + APP_LOAD_ADDRESS, word);
}

uint32_t 
flash_func_read_word(uint32_t address)
{
	if (address & 3)
		return 0;
	return *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

uint32_t
flash_func_read_otp(uint32_t address)
{
	if (address & 3)
		return 0;
	if (address > OTP_SIZE)
		return 0;

	return *(uint32_t *)(address + OTP_BASE);
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

	/* configure the clock for bootloader activity */
	clock_init();	
	
	/* start the interface */
#if INTERFACE_USART
	cinit(BOARD_INTERFACE_CONFIG_USART, USART);
#endif
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif



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

#ifdef BOOT_DELAY_ADDRESS
        {
            /*
              if a boot delay signature is present then delay the boot
              by at least that amount of time in seconds. This allows
              for an opportunity for a companion computer to load a
              new firmware, while still booting fast by sending a BOOT
              command
             */
            uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);
            uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS+4);
            if (sig2 == BOOT_DELAY_SIGNATURE2 &&
                (sig1 & 0xFFFFFF00) == (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00)) {
                unsigned boot_delay = sig1 & 0xFF;
                if (boot_delay <= BOOT_DELAY_MAX) {
                    try_boot = false;
                    if (timeout < boot_delay*1000)
                        timeout = boot_delay*1000;
                }
            }
        }
#endif

	/* 
	 * Check if the force-bootloader pins are strapped; if strapped,
	 * don't try booting.
	 */
	if (board_test_force_pin()) {
		try_boot = false;		
	}

#if INTERFACE_USB
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

#if INTERFACE_USART
	/*
	 * Check for if the USART port RX line is receiving a break command, or is being held low. If yes, 
	 * don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
	if(board_test_usart_receiving_break()) {
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

		// If it failed to boot, reset the boot signature and stay in bootloader
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);

		/* booting failed, stay in the bootloader forever */
		timeout = 0;
	}


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

#if INTERFACE_USART
		/* if the USART port RX line is still receiving a break, just loop back */
		if (board_test_usart_receiving_break())
			continue;
#endif

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
