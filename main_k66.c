/*
 * Kinetis K66 board support for the bootloader.
 *
 */

#include "kinetis.h"
#include "gpio/fsl_gpio.h"
#include "port/fsl_port.h"
#include "smc/smc.h"
#include "flash/fsl_flash.h"

#include "hw_config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "bl.h"
#include "uart.h"

#define BOOTLOADER_RESERVATION_SIZE	(32 * 1024)
#define FIRST_FLASH_SECTOR_TO_ERASE  (BOARD_FIRST_FLASH_SECTOR_TO_ERASE + (BOOTLOADER_RESERVATION_SIZE/FLASH_SECTOR_SIZE))

#define BOARD_RESETCLOCKRUN_CORE_CLOCK             20971520U  /*!< Core clock frequency: 20971520Hz */

#define MCG_IRCLK_DISABLE                                 0U  /*!< MCGIRCLK disabled */
#define MCG_PLL_DISABLE                                   0U  /*!< MCGPLLCLK disabled */
#define OSC_CAP0P                                         0U  /*!< Oscillator 0pF capacitor load */
#define OSC_ER_CLK_DISABLE                                0U  /*!< Disable external reference clock */
#define SIM_OSC32KSEL_OSC32KCLK_CLK                       0U  /*!< OSC32KSEL select: OSC32KCLK clock */
#define SIM_PLLFLLSEL_MCGFLLCLK_CLK                       0U  /*!< PLLFLL select: MCGFLLCLK clock */

// SIM_SDID
#define KINETIS_UNKNOWN	0
#define KINETIS_K66

#define PIN_MASK       0x0000000f
#define FAM_MASK       0x00000070
#define DIEID_MASK     0x00000f80
#define REVID_MASK     0x0000f000
#define RESID_MASK     0x000f0000
#define SERIESID_MASK  0x00f00000
#define SUBFAMID_MASK  0x0f000000
#define FAMID_MASK     0x00000000

#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	(void *)BOARD_USART
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  	NULL
#endif

flash_config_t g_flashState;                       //!< Flash driver instance.

static uint32_t s_flashRunCommand[kFLASH_ExecuteInRamFunctionMaxSizeInWords];
static uint32_t s_flashCacheClearCommand[kFLASH_ExecuteInRamFunctionMaxSizeInWords];
static flash_execute_in_ram_function_config_t s_flashExecuteInRamFunctionInfo = {
	.activeFunctionCount = 0,
	.flashRunCommand = s_flashRunCommand,
	.flashCacheClearCommand = s_flashCacheClearCommand,
};


/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,
	.systick_mhz	= 120,
};

static void board_init(void);

#define BOOT_RTC_SIGNATURE          0xb007b007
#define POWER_DOWN_RTC_SIGNATURE    0xdeaddead // Written by app fw to not re-power on.
#define BOOT_RTC_REG                0

void flash_lock(void)
{

}

/* standard clocking for all K66 boards */



static uint32_t
board_get_rtc_signature()
{
	return RFVBAT->REG[BOOT_RTC_REG];
}

static void
board_set_rtc_signature(uint32_t sig)
{
	RFVBAT->REG[BOOT_RTC_REG] = sig;
}


static bool
board_test_force_pin()
{
	return false;
}
#if INTERFACE_USART == 1
static bool
board_test_usart_receiving_break()
{
#if !defined(SERIAL_BREAK_DETECT_DISABLED)
	/* (re)start the SysTick timer system */
	systick_interrupt_disable(); // Kill the interrupt if it is still active
	systick_counter_disable(); // Stop the timer
	systick_set_clocksource(SYSTIC_CLKSOURCE_AHB);

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
	while (cnt < 60) {
		// Only read pin when SysTick timer is true
		if (systick_get_countflag() == 1) {
			if (GPIO_ReadPinInput(BOARD_PORT_USART, BOARD_PIN_RX) == 0) {
				cnt_consecutive_low++;	// Increment the consecutive low counter

			} else {
				cnt_consecutive_low = 0; // Reset the consecutive low counter
			}

			cnt++;
		}

		// If 9 consecutive low bits were received break out of the loop
		if (cnt_consecutive_low >= 18) {
			break;
		}

	}

	systick_counter_disable(); // Stop the timer

	/*
	 * If a break is detected, return true, else false
	 *
	 * Break is detected if line was low for 9 consecutive bits.
	 */
	if (cnt_consecutive_low >= 18) {
		return true;
	}

#endif // !defined(SERIAL_BREAK_DETECT_DISABLED)

	return false;
}
#endif

static void
board_init(void)
{
	exit_vlpr();

	g_flashState.flashExecuteInRamFunctionInfo = &s_flashExecuteInRamFunctionInfo.activeFunctionCount;
	FLASH_PrepareExecuteInRamFunctions(&g_flashState);

	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX;

#if defined(BOARD_POWER_PIN_OUT)

	/* Configure the Power pins */

	/*  Sets the ports clocking */

	CLOCK_EnableClock(KINETIS_CLOCK_PORT(BOARD_POWER_PORT));

	port_pin_config_t power_port_config = {
		.pullSelect           = kPORT_PullDown,
		.slewRate             = kPORT_FastSlewRate,
		.passiveFilterEnable  = kPORT_PassiveFilterDisable,
		.openDrainEnable      = kPORT_OpenDrainDisable,
		.driveStrength        = kPORT_LowDriveStrength,
		.mux                  = kPORT_MuxAsGpio,
		.lockRegister         = kPORT_UnLockRegister,
	};

	/*  Sets the port configuration */

	PORT_SetPinConfig(KINETIS_PORT(BOARD_POWER_PORT), BOARD_POWER_PIN_OUT, &power_port_config);

	gpio_pin_config_t power_pin_config = {
		kGPIO_DigitalOutput,
		1,
	};

	/*  Sets the pin configuration */

	GPIO_PinInit(KINETIS_GPIO(BOARD_POWER_PORT), BOARD_POWER_PIN_OUT, &power_pin_config);

	BOARD_POWER_ON(KINETIS_GPIO(BOARD_POWER_PORT), BOARD_POWER_PIN_OUT);
#endif

#if INTERFACE_USB

	// Disable the MPU otherwise USB cannot access the bus

	MPU->CESR = 0;

	/* enable Port pin to sample VBUS */

	CLOCK_EnableClock(KINETIS_CLOCK_PORT(BOARD_PORT_VBUS));

	port_pin_config_t vbus_port_config = {
		.pullSelect           = kPORT_PullDown,
		.slewRate             = kPORT_FastSlewRate,
		.passiveFilterEnable  = kPORT_PassiveFilterDisable,
		.openDrainEnable      = kPORT_OpenDrainDisable,
		.driveStrength        = kPORT_LowDriveStrength,
		.mux                  = kPORT_MuxAsGpio,
		.lockRegister         = kPORT_UnLockRegister,
	};

	/*  Sets the port configuration */

	PORT_SetPinConfig(KINETIS_PORT(BOARD_PORT_VBUS), BOARD_PIN_VBUS, &vbus_port_config);

	gpio_pin_config_t vbus_pin_config = {
		kGPIO_DigitalInput,
		0,
	};

	/*  Sets the pin configuration */

	GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_VBUS), BOARD_PIN_VBUS, &vbus_pin_config);

#endif

#if INTERFACE_USART

	/* configure USART clock */

	CLOCK_EnableClock(KINETIS_CLOCK_UART(BOARD_USART));

	/* configure USART pins */

	CLOCK_EnableClock(KINETIS_CLOCK_PORT(BOARD_PORT_USART));

	port_pin_config_t uart_port_config = {
		.pullSelect           = kPORT_PullDisable,
		.slewRate             = kPORT_FastSlewRate,
		.passiveFilterEnable  = kPORT_PassiveFilterDisable,
		.openDrainEnable      = kPORT_OpenDrainDisable,
		.driveStrength        = kPORT_LowDriveStrength,
		.mux                  = BOARD_PORT_USART_AF,
		.lockRegister         = kPORT_UnLockRegister,
	};

	/*  Sets the port configuration */

	PORT_SetMultiplePinsConfig(KINETIS_PORT(BOARD_PORT_USART), KINETIS_MASK(BOARD_PIN_TX) | KINETIS_MASK(BOARD_PIN_RX),
				   &uart_port_config);

#endif

	/* initialise LEDs */

	CLOCK_EnableClock(KINETIS_CLOCK_PORT(BOARD_PORT_LEDS));

	port_pin_config_t led_port_config = {
		.pullSelect           = kPORT_PullDisable,
		.slewRate             = kPORT_FastSlewRate,
		.passiveFilterEnable  = kPORT_PassiveFilterDisable,
		.openDrainEnable      = kPORT_OpenDrainDisable,
		.driveStrength        = kPORT_LowDriveStrength,
		.mux                  = kPORT_MuxAsGpio,
		.lockRegister         = kPORT_UnLockRegister,
	};

	uint32_t leds = 0;
#if defined(BOARD_PIN_LED_ACTIVITY)
	leds |= KINETIS_MASK(BOARD_PIN_LED_ACTIVITY);
#endif

#if defined(BOARD_PIN_LED_BOOTLOADER)
	leds |= KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER);
#endif

	PORT_SetMultiplePinsConfig(KINETIS_PORT(BOARD_PORT_LEDS), leds, &led_port_config);

	gpio_pin_config_t led_pin_config = {
		kGPIO_DigitalOutput,
		1,
	};

	/*  Sets the pin configuration */

#if defined(BOARD_PIN_LED_ACTIVITY)
	GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_LEDS), BOARD_PIN_LED_ACTIVITY, &led_pin_config);
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
	GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_LEDS), BOARD_PIN_LED_BOOTLOADER, &led_pin_config);
#endif
	BOARD_LED_ON(KINETIS_GPIO(BOARD_PORT_LEDS), leds);
}

void
board_deinit(void)
{

	port_pin_config_t unconfigure_port_config = {
		kPORT_PullDisable,
		kPORT_FastSlewRate,
		kPORT_PassiveFilterDisable,
		kPORT_OpenDrainDisable,
		kPORT_LowDriveStrength,
		kPORT_PinDisabledOrAnalog,
		kPORT_UnLockRegister,
	};

#if INTERFACE_USART

	/* Un-configure USART pins */

	PORT_SetMultiplePinsConfig(KINETIS_PORT(BOARD_PORT_USART), KINETIS_MASK(BOARD_PIN_TX) | KINETIS_MASK(BOARD_PIN_RX),
				   &unconfigure_port_config);

	/* Un-configure USART clock */

	CLOCK_DisableClock(KINETIS_CLOCK_UART(BOARD_USART));
#endif

#if defined(BOARD_POWER_PIN_OUT) && defined(BOARD_POWER_PIN_RELEASE)
	/* deinitialize the POWER pin - with the assumption the hold up time of
	 * the voltage being bleed off by an inupt pin impedance will allow
	 * enough time to boot the app
	 */
	PORT_SetPinConfig(KINETIS_PORT(BOARD_POWER_PORT), BOARD_POWER_PIN, &unconfigure_port_config);
#endif

	/* deinitialise LEDs */
	uint32_t leds = 0;
#if defined(BOARD_PIN_LED_ACTIVITY)
	leds |= KINETIS_MASK(BOARD_PIN_LED_ACTIVITY);
#endif

#if defined(BOARD_PIN_LED_BOOTLOADER)
	leds |= KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER);
#endif

	PORT_SetMultiplePinsConfig(KINETIS_PORT(BOARD_PORT_LEDS), leds, &unconfigure_port_config);
}


static void CLOCK_CONFIG_FllStableDelay(void)
{
	uint32_t i = 30000U;

	while (i--) {
		__NOP();
	}
}

void
clock_deinit(void)
{


	const mcg_config_t mcgConfig_BOARD_BootClockRUN = {
		.mcgMode = kMCG_ModeFEI,                  /* FEI - FLL Engaged Internal */
		.irclkEnableMode = MCG_IRCLK_DISABLE,     /* MCGIRCLK disabled */
		.ircs = kMCG_IrcSlow,                     /* Slow internal reference clock selected */
		.fcrdiv = 0x1U,                           /* Fast IRC divider: divided by 2 */
		.frdiv = 0x0U,                            /* FLL reference clock divider: divided by 1 */
		.drs = kMCG_DrsLow,                       /* Low frequency range */
		.dmx32 = kMCG_Dmx32Default,               /* DCO has a default range of 25% */
		.oscsel = kMCG_OscselOsc,                 /* Selects System Oscillator (OSCCLK) */
		.pll0Config =
		{
			.enableMode = MCG_PLL_DISABLE,    /* MCGPLLCLK disabled */
			.prdiv = 0x0U,                    /* PLL Reference divider: divided by 1 */
			.vdiv = 0x0U,                     /* VCO divider: multiplied by 16 */
		},
		.pllcs = kMCG_PllClkSelPll0,              /* PLL0 output clock is selected */
	};
	const sim_clock_config_t simConfig_BOARD_BootClockRUN = {
		.pllFllSel = SIM_PLLFLLSEL_MCGFLLCLK_CLK, /* PLLFLL select: MCGFLLCLK clock */
		.pllFllDiv = 0,                           /* PLLFLLSEL clock divider divisor: divided by 1 */
		.pllFllFrac = 0,                          /* PLLFLLSEL clock divider fraction: multiplied by 1 */
		.er32kSrc = SIM_OSC32KSEL_OSC32KCLK_CLK,  /* OSC32KSEL select: OSC32KCLK clock */
		.clkdiv1 = 0x110000U,                     /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /1, OUTDIV3: /2, OUTDIV4: /2 */
	};

	/* Set the system clock dividers in SIM to safe value. */
	CLOCK_SetSimSafeDivs();
	/* Configure the Internal Reference clock (MCGIRCLK). */
	CLOCK_SetInternalRefClkConfig(mcgConfig_BOARD_BootClockRUN.irclkEnableMode,
				      mcgConfig_BOARD_BootClockRUN.ircs,
				      mcgConfig_BOARD_BootClockRUN.fcrdiv);
	/* Set MCG to FEI mode. */
	CLOCK_BootToFeiMode(mcgConfig_BOARD_BootClockRUN.dmx32,
			    mcgConfig_BOARD_BootClockRUN.drs,
			    CLOCK_CONFIG_FllStableDelay);
	/* Set the clock configuration in SIM module. */
	CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
	/* Set SystemCoreClock variable. */
	SystemCoreClock = BOARD_RESETCLOCKRUN_CORE_CLOCK;
}

uint32_t
flash_func_sector_size(unsigned sector)
{
	return (sector < BOARD_FLASH_SECTORS) ? BOARD_FLASH_SIZE : 0;
}

void flash_erase_sector(unsigned sector)
{

}

void flash_wait_for_last_operation()
{

}
void
flash_func_erase_sector(unsigned sector)
{
	if (sector >= BOARD_FLASH_SECTORS || sector < FIRST_FLASH_SECTOR_TO_ERASE) {
		return;
	}

	/* Calculate the logical base address of the sector
	 * flash_func_read_word will add APP_LOAD_ADDRESS
	 */
	uint32_t address = 0;

	for (unsigned i = BOARD_FIRST_FLASH_SECTOR_TO_ERASE; i < sector; i++) {
		address += flash_func_sector_size(i);
	}

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
	if (!blank) {
		flash_erase_sector(sector);
	}
}

void
flash_func_write_word(uint32_t address, uint32_t word)
{
	address += APP_LOAD_ADDRESS;

	/* Ensure that all flash operations are complete. */

	flash_wait_for_last_operation();

	/* Program the 32bits. */

	/* Enable writes to flash. */


	/* Program the word. */


	/* Use DSB to complete write etal above. So that wait is not skipped */

	__asm__ volatile("DSB \n");

	flash_wait_for_last_operation();
}

uint32_t
flash_func_read_word(uint32_t address)
{
	if (address & 3) {
		return 0;
	}

	return *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

uint32_t
flash_func_read_otp(uint32_t address)
{
	return 0;
}

uint32_t get_mcu_id(void)
{
	return SIM->SDID;
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	const char none[] = "MK66FN2M0VMD18";
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
	const volatile uint32_t *p = &SIM->UIDH;
	return p[address];
}

void
led_on(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		BOARD_LED_ON(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		BOARD_LED_ON(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

void
led_off(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		BOARD_LED_OFF(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		BOARD_LED_OFF(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

void
led_toggle(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		GPIO_TogglePinsOutput(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		GPIO_TogglePinsOutput(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

int
main(void)
{

	// Start up code has initalised Clocks and MPU, FPU
	bool try_boot = true;			/* try booting before we drop to the bootloader */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

#if defined(BOARD_POWER_PIN_OUT)

	/* Here we check for the app setting the POWER_DOWN_RTC_SIGNATURE
	 * in this case, we reset the signature and wait to die
	 */
	if (board_get_rtc_signature() == POWER_DOWN_RTC_SIGNATURE) {
		board_set_rtc_signature(0);

		while (1);
	}

#endif

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
		uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS + 4);

		if (sig2 == BOOT_DELAY_SIGNATURE2 &&
		    (sig1 & 0xFFFFFF00) == (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00)) {
			unsigned boot_delay = sig1 & 0xFF;

			if (boot_delay <= BOOT_DELAY_MAX) {
				try_boot = false;

				if (timeout < boot_delay * 1000) {
					timeout = boot_delay * 1000;
				}
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
#if defined(BOARD_USB_VBUS_SENSE_DISABLED)
	try_boot = false;
#else

	if (GPIO_ReadPinInput(KINETIS_GPIO(BOARD_PORT_VBUS), BOARD_PIN_VBUS) != 0) {

		/* don't try booting before we set up the bootloader */
		try_boot = false;
	}

#endif
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
	if (board_test_usart_receiving_break()) {
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


	/* start the interface */
#if INTERFACE_USART
	cinit(BOARD_INTERFACE_CONFIG_USART, USART);
#endif
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif

	while (1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader(timeout);

		/* if the force-bootloader pins are strapped, just loop back */
		if (board_test_force_pin()) {
			continue;
		}

#if INTERFACE_USART

		/* if the USART port RX line is still receiving a break, just loop back */
		if (board_test_usart_receiving_break()) {
			continue;
		}

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

void _start()
{
	main();
}


void SysTick_Handler()
{
	sys_tick_handler();
}

int DbgConsole_Printf(char *fmt_s, ...)
{
	return 0;
}
