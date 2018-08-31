/******************** (C) COPYRIGHT 2018 merafour ********************
* Author             : 冷月追风@merafour.blog.163.com
* Version            : V1.0.0
* Date               : 30/8/2018
* Description        : vectors.
********************************************************************************
* merafour.blog.163.com
* merafour@163.com
* github.com/Merafour
*******************************************************************************/
/*
 * STM32F4 board support for the bootloader.
 *
 */

#include "usbs_hw_config.h"

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/timer.h>


#include "usbs_bl.h"
#include "uart.h"

/* flash parameters that we should not really know */
static struct {
	uint32_t	sector_number;
	uint32_t	size;
} flash_sectors[] = {

	/* Physical FLASH sector 0 is reserved for bootloader and is not
	 * the table below.
	 * N sectors may aslo be reserved for the app fw in which case
	 * the zero based define BOARD_FIRST_FLASH_SECTOR_TO_ERASE must
	 * be defined to begin the erase above of the reserved sectors.
	 * The default value of BOARD_FIRST_FLASH_SECTOR_TO_ERASE is 0
	 * and begins flash erase operations at phsical sector 1 the 0th entry
	 * in the table below.
	 * A value of 1 for BOARD_FIRST_FLASH_SECTOR_TO_ERASE would reserve
	 * the 0th entry and begin erasing a index 1 the third physical sector
	 * on the device.
	 *
	 * When BOARD_FIRST_FLASH_SECTOR_TO_ERASE is defined APP_RESERVATION_SIZE
	 * must also be defined to remove that additonal reserved FLASH space
	 * from the BOARD_FLASH_SIZE. See APP_SIZE_MAX below.
	 */

	{0x01, 16 * 1024},
	{0x02, 16 * 1024},
	{0x03, 16 * 1024},
#if 0
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
#endif
};
#define BOOTLOADER_RESERVATION_SIZE	(16 * 1024)

#define OTP_BASE			0x1fff7800
#define OTP_SIZE			512
#define UDID_START		        0x1FFF7A10

// address of MCU IDCODE
#define DBGMCU_IDCODE		0xE0042000
#define STM32_UNKNOWN	0
#define STM32F40x_41x	0x413
#define STM32F42x_43x	0x419
#define STM32F42x_446xx	0x421

#define REVID_MASK	0xFFFF0000
#define DEVID_MASK	0xFFF

/* magic numbers from reference manual */

typedef enum mcu_rev_e {
	MCU_REV_STM32F4_REV_A = 0x1000,
	MCU_REV_STM32F4_REV_Z = 0x1001,
	MCU_REV_STM32F4_REV_Y = 0x1003,
	MCU_REV_STM32F4_REV_1 = 0x1007,
	MCU_REV_STM32F4_REV_3 = 0x2001
} mcu_rev_e;

typedef struct mcu_des_t {
	uint16_t mcuid;
	const char *desc;
	char  rev;
} mcu_des_t;

// The default CPU ID  of STM32_UNKNOWN is 0 and is in offset 0
// Before a rev is known it is set to ?
// There for new silicon will result in STM32F4..,?
mcu_des_t mcu_descriptions[] = {
	{ STM32_UNKNOWN,	"STM32F???",    '?'},
	{ STM32F40x_41x, 	"STM32F40x",	'?'},
	{ STM32F42x_43x, 	"STM32F42x",	'?'},
	{ STM32F42x_446xx, 	"STM32F446XX",	'?'},
};

typedef struct mcu_rev_t {
	mcu_rev_e revid;
	char  rev;
} mcu_rev_t;

/*
 * This table is used in 2 ways. One to look look up the revision
 * of a given chip. Two to see it a revsion is in the group of "Bad"
 * silicon.
 *
 * Therefore when adding entries for good silicon rev, they must be inserted
 * at the beginning of the table. The value of FIRST_BAD_SILICON_OFFSET will
 * also need to be increased to that of the value of the first bad silicon offset.
 *
 */
const mcu_rev_t silicon_revs[] = {
	{MCU_REV_STM32F4_REV_3, '3'}, /* Revision 3 */

	{MCU_REV_STM32F4_REV_A, 'A'}, /* Revision A */  // FIRST_BAD_SILICON_OFFSET (place good ones above this line and update the FIRST_BAD_SILICON_OFFSET accordingly)
	{MCU_REV_STM32F4_REV_Z, 'Z'}, /* Revision Z */
	{MCU_REV_STM32F4_REV_Y, 'Y'}, /* Revision Y */
	{MCU_REV_STM32F4_REV_1, '1'}, /* Revision 1 */
};

#define FIRST_BAD_SILICON_OFFSET 1

#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

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

	.systick_mhz	= 48, // 168
};

static void board_init(void);

#define BOOT_RTC_SIGNATURE          0xb007b007
#define POWER_DOWN_RTC_SIGNATURE    0xdeaddead // Written by app fw to not re-power on.
#define BOOT_RTC_REG                MMIO32(RTC_BASE + 0x50)
#if 0
/* standard clocking for all F4 boards */
static const struct rcc_clock_scale clock_setup = {
	.pllm = OSC_FREQ,
	.plln = 336,
	.pllp = 2,
	.pllq = 7,
#if defined(STM32F446) || defined(STM32F469)
	.pllr = 2,
#endif
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE_DIV_4,
	.ppre2 = RCC_CFGR_PPRE_DIV_2,
	.power_save = 0,
	.flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_5WS,
	.apb1_frequency = 42000000,
	.apb2_frequency = 84000000,
};
#endif
static const struct rcc_clock_scale clock_setup = {
	.pllm = OSC_FREQ,
	.plln = 192,
	.pllp = 4,
	.pllq = 4,
#if defined(STM32F446) || defined(STM32F469)
	.pllr = 2,
#endif
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE_DIV_2,
	.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
	.power_save = 0,
	.flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_5WS,
	.apb1_frequency = 24000000,
	.apb2_frequency = 48000000,
};

static void
board_init(void)
{
	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX;
#if defined(TARGET_HW_PX4_FMU_V2) || defined(TARGET_HW_PX4_FMU_V4)

	if (check_silicon() && board_info.fw_size == (2 * 1024 * 1024) - BOOTLOADER_RESERVATION_SIZE) {
		board_info.fw_size = (1024 * 1024) - BOOTLOADER_RESERVATION_SIZE;
	}

#endif

#if defined(BOARD_POWER_PIN_OUT)
	/* Configure the Power pins */
	rcc_peripheral_enable_clock(&BOARD_POWER_CLOCK_REGISTER, BOARD_POWER_CLOCK_BIT);
	gpio_mode_setup(BOARD_POWER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_POWER_PIN_OUT);
	gpio_set_output_options(BOARD_POWER_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, BOARD_POWER_PIN_OUT);
	BOARD_POWER_ON(BOARD_POWER_PORT, BOARD_POWER_PIN_OUT);
#endif

#if INTERFACE_USB

	/* enable Port A GPIO9 to sample VBUS */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
#endif

#if INTERFACE_USART
	/* configure USART pins */
	rcc_peripheral_enable_clock(&BOARD_USART_PIN_CLOCK_REGISTER, BOARD_USART_PIN_CLOCK_BIT_TX);
	rcc_peripheral_enable_clock(&BOARD_USART_PIN_CLOCK_REGISTER, BOARD_USART_PIN_CLOCK_BIT_RX);

	/* Setup GPIO pins for USART transmit. */
	gpio_mode_setup(BOARD_PORT_USART_TX, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BOARD_PIN_TX);
	gpio_mode_setup(BOARD_PORT_USART_RX, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BOARD_PIN_RX);
	/* Setup USART TX & RX pins as alternate function. */
	gpio_set_af(BOARD_PORT_USART_TX, BOARD_PORT_USART_AF, BOARD_PIN_TX);
	gpio_set_af(BOARD_PORT_USART_RX, BOARD_PORT_USART_AF, BOARD_PIN_RX);

	/* configure USART clock */
	rcc_peripheral_enable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* configure the force BL pins */
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN_IN);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_OUT);
	gpio_set_output_options(BOARD_FORCE_BL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, BOARD_FORCE_BL_PIN_OUT);
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* configure the force BL pins */
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN);
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
	BOARD_LED_ON(
		BOARD_PORT_LEDS,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

	/* enable the power controller clock */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);
}

void
board_deinit(void)
{

#if INTERFACE_USART
	/* deinitialise GPIO pins for USART transmit. */
	gpio_mode_setup(BOARD_PORT_USART_TX, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_TX);
	gpio_mode_setup(BOARD_PORT_USART_RX, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_RX);

	/* disable USART peripheral clock */
	rcc_peripheral_disable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* deinitialise the force BL pins */
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_OUT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_IN);
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* deinitialise the force BL pin */
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN);
#endif

#if defined(BOARD_POWER_PIN_OUT) && defined(BOARD_POWER_PIN_RELEASE)
	/* deinitialize the POWER pin - with the assumption the hold up time of
	 * the voltage being bleed off by an inupt pin impedance will allow
	 * enough time to boot the app
	 */
	gpio_mode_setup(BOARD_POWER_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_POWER_PIN);
#endif

	/* deinitialise LEDs */
	gpio_mode_setup(
		BOARD_PORT_LEDS,
		GPIO_MODE_INPUT,
		GPIO_PUPD_NONE,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

	/* disable the power controller clock */
	rcc_peripheral_disable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);

	/* disable the AHB peripheral clocks */
	RCC_AHB1ENR = 0x00100000; // XXX Magic reset number from STM32F4x reference manual
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
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Reset the RCC_CFGR register */
	RCC_CFGR = 0x000000;

	/* Stop the HSE, CSS, PLL, PLLI2S, PLLSAI */
	rcc_osc_off(RCC_HSE);
	rcc_osc_off(RCC_PLL);
	rcc_css_disable();

	/* Reset the RCC_PLLCFGR register */
	RCC_PLLCFGR = 0x24003010; // XXX Magic reset number from STM32F4xx reference manual

	/* Reset the HSEBYP bit */
	rcc_osc_bypass_disable(RCC_HSE);

	/* Reset the CIR register */
	RCC_CIR = 0x000000;
}

uint32_t
flash_func_sector_size(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS) {
		return flash_sectors[sector].size;
	}

	return 0;
}

void
flash_func_erase_sector(unsigned sector)
{
	if (sector >= BOARD_FLASH_SECTORS || sector < BOARD_FIRST_FLASH_SECTOR_TO_ERASE) {
		return;
	}

	/* Caculate the logical base address of the sector
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
		flash_erase_sector(flash_sectors[sector].sector_number, FLASH_CR_PROGRAM_X32);
	}
}

void
flash_func_write_word(uint32_t address, uint32_t word)
{
	//flash_program_word(address + APP_LOAD_ADDRESS, word);
	flash_program_word(address + 0x08000000, word);
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
	if (address & 3) {
		return 0;
	}

	if (address > OTP_SIZE) {
		return 0;
	}

	return *(uint32_t *)(address + OTP_BASE);
}

uint32_t get_mcu_id(void)
{
	return *(uint32_t *)DBGMCU_IDCODE;
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	uint32_t idcode = (*(uint32_t *)DBGMCU_IDCODE);
	int32_t mcuid = idcode & DEVID_MASK;
	mcu_rev_e revid = (idcode & REVID_MASK) >> 16;

	mcu_des_t des = mcu_descriptions[STM32_UNKNOWN];

	for (int i = 0; i < arraySize(mcu_descriptions); i++) {
		if (mcuid == mcu_descriptions[i].mcuid) {
			des = mcu_descriptions[i];
			break;
		}
	}

	for (int i = 0; i < arraySize(silicon_revs); i++) {
		if (silicon_revs[i].revid == revid) {
			des.rev = silicon_revs[i].rev;
		}
	}

	uint8_t *endp = &revstr[max - 1];
	uint8_t *strp = revstr;

	while (strp < endp && *des.desc) {
		*strp++ = *des.desc++;
	}

	if (strp < endp) {
		*strp++ = ',';
	}

	if (strp < endp) {
		*strp++ = des.rev;
	}

	return  strp - revstr;
}


int check_silicon(void)
{
#if defined(TARGET_HW_PX4_FMU_V2) || defined(TARGET_HW_PX4_FMU_V4)
	uint32_t idcode = (*(uint32_t *)DBGMCU_IDCODE);
	mcu_rev_e revid = (idcode & REVID_MASK) >> 16;

	for (int i = FIRST_BAD_SILICON_OFFSET; i < arraySize(silicon_revs); i++) {
		if (silicon_revs[i].revid == revid) {
			return -1;
		}
	}

#endif
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

/* we should know this, but we don't */
#ifndef SCB_CPACR
# define SCB_CPACR (*((volatile uint32_t *) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif

uint32_t code[16*1024/4]; // 16K
extern uint16_t bl_flash_get32(uint32_t code[], const uint16_t _size, const uint8_t set_head);
int main(void)
{
	uint32_t	address = 0;
	const uint32_t len = 16*1024/4; // 16K
	uint32_t i=0;
	
	/* Enable the FPU before we hit any FP instructions */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

	/* do board-specific initialisation */
	board_init();

	/* configure the clock for bootloader activity */
	clock_init();
	
	// copy code
	bl_flash_get32(NULL, 0, 1); // init
	bl_flash_get32(code, sizeof(code)/4, 0);

	// erase all sectors
	flash_unlock();
	//flash_func_erase_sector(0);
	flash_func_erase_sector(1);
	flash_erase_sector(0, FLASH_CR_PROGRAM_X32);
	address=0;
	for (i = 0; i < len; i++) {
		// program the word
		flash_func_write_word(address, code[i]);
		address += 4;
	}
	flash_func_erase_sector(0);
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
