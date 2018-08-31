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

static bool
should_wait(void)
{
	bool result = false;

	PWR_CR |= PWR_CR_DBP;

	if (BKP_DR1 == BL_WAIT_MAGIC) {
		result = true;
		BKP_DR1 = 0;
	}

	PWR_CR &= ~PWR_CR_DBP;

	return result;
}
#if 0
#define  point_base   0x08000000
//#define  point_offset   0x3C00  // 15K
#define  point_offset   0x2C00  // 11K
#define  point_save   8  // sizeof(uint32_t)*2
#define  point_addr   (point_base+point_offset+point_save)
#else
#define  _point_base   (APP_LOAD_ADDRESS-1024)   // APP地址的前面 1K用于存储加密数据
#define  point_save   8  // sizeof(uint32_t)*2
#define  point_addr   (_point_base+point_save)
#endif

#define UID0    (0x37cc24eaU)
#define UID1    (0x4fd83d68U)
#define UID2    (0xe5345ed9U)
int erase_flag=0; // 通过该变量检测是否强行跳过了加密校验代码
void encoding(uint32_t sign[8], volatile uint32_t uid[3])
{
		uint32_t order = 0xb10be924; //(uint32_t)(&clock_setup.flash_config);
    sign[0] = (order&uid[1])|     ((~uid[0])&UID2);
    sign[1] = (order&uid[0])|     (uid[1]&(~UID2));
#if 0
    sign[2] = (uid[0]&UID1)  &     (uid[2]|order);
		sign[3] = (uid[0]|(~UID2)) ^ (uid[1]+order);
#else
    sign[2] = uid[0]^UID1^order;
		sign[3] = uid[1]^((order+uid[0])|(~UID2));
		//printf("order0: %08X %08X %08X\r\n", (unsigned int)order, (unsigned int)uid[0], (unsigned int)uid[1]);
		order = order + (uid[0]&uid[1]);
		//printf("order1: %08X\r\n", (unsigned int)order);
		order = ((order<<12)&0xFFFFF000) | ((order>>12)&0x000FFFFF);
		//printf("order2: %08X\r\n", (unsigned int)order);
#endif
		sign[4] = (sign[0]^order)|     (uid[1]^(~sign[2]));
		sign[5] = (sign[0]|UID2) &     (sign[2]+order);  // 
		sign[6] = (~sign[0]|UID0)&     (sign[2]+order);   // 
		sign[7] = (sign[1]|UID1) &     (~(sign[2]+order)); // 
		erase_flag = 1;
}
extern void led_blink_off(void);
extern void led_blink_on(void);
void __attribute__ ((section(".reset_rb3"))) test_entry()
{
	//unsigned int led=0;
	//unsigned int clock=0;
	//const uint16_t delay_led[]={500, 500, 500, 500, 500, 500, 500, 500};
	//uint16_t count=0;
	//led_blink_off();
//	clock = 0;
//	Qi.led_on(&led1, 1);
//	Qi.led_on(&led2, 0);

	/* configure the clock for bootloader activity */
	//rcc_clock_setup_hse_3v3(&clock_setup);
	//rcc_clock_setup_hsi_3v3(&clock_setup);
	
	/* (re)start the timer system */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);	/* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();
	
	//led = clock+40; // 0.04s
	//for(clock=0; clock<10000; clock += 10)
	//for(clock=0; ; clock += 10) 
	while(1)
	{
		//delay(100);
		led_blink_on();
		delay(2000);
		led_blink_off();
		delay(2000);
		/*for(count=0; count<8; count++)
		{
			//led = clock+250; // 0.05s 
			led = delay_led[count]; // 0.05s
			led_toggle(LED_BOOTLOADER);
			delay(led);
		}*/
	}
}

void read_uid(uint32_t uid[3])
{
	volatile uint32_t* _mtext=NULL;
	// 0x1FFF F7E8, 产品唯一身份标识寄存器(96位)
	_mtext = (volatile uint32_t*)(0x1FFF0000);
	_mtext += (0xF7E8/4);
	uid[0] = _mtext[0];
	uid[1] = _mtext[1];
	uid[2] = _mtext[2];
}
int main_init(void)
{
	volatile uint32_t* _mtext=NULL;
	uint32_t passwd[8]={0};
	uint32_t uid[3]={0};
	int match=0;
	unsigned int led=0;
	unsigned timeout = 0;

	/* do board-specific initialisation */
	board_init();
	clock_init();
	
	//check
	read_uid(uid);
	//uid[0]++;
	encoding(passwd, uid);

	_mtext = (uint32_t*)(point_addr);
	for(led=0; led<8; led++)
	{
		if(passwd[led] != _mtext[led])
		{
			//printf("not match%d: %08X %08X\r\n", led, passwd[led], _mtext[led]);
			match=1;
		}
	}	
	/*if(0==match)
	{
		entry();
	}*/	
	if(match || (0==erase_flag))
	//if(match)
	{
		while (1) 
		{
			test_entry();	
		}
	}

#if defined(INTERFACE_USART) || defined (INTERFACE_USB)
	/* XXX sniff for a USART connection to decide whether to wait in the bootloader? */
	timeout = BOOTLOADER_DELAY;
#endif

#ifdef INTERFACE_I2C
# error I2C bootloader detection logic not implemented
#endif

	/* if the app left a cookie saying we should wait, then wait */
	if (should_wait()) {
		timeout = BOOTLOADER_DELAY;
	}

#ifdef BOARD_FORCE_BL_PIN

	/* if the force-BL pin state matches the state of the pin, wait in the bootloader forever */
	if (BOARD_FORCE_BL_VALUE == gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN)) {
		timeout = 0xffffffff;
	}

#endif

	/* look for the magic wait-in-bootloader value in backup register zero */


	/* if we aren't expected to wait in the bootloader, try to boot immediately */
	if (timeout == 0) {
		/* try to boot immediately */
		jump_to_app();

		/* if we returned, there is no app; go to the bootloader and stay there */
		timeout = 0;
	}

	/* configure the clock for bootloader activity */
	//clock_init();
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif
	/* start the interface */
	cinit(BOARD_INTERFACE_CONFIG, USART);

	return timeout;
}
int main(void)
{
#if 0
	volatile uint32_t* _mtext=NULL;
	uint32_t passwd[8]={0};
	uint32_t uid[3]={0};
	int match=0;
	unsigned int led=0;
	unsigned timeout = 0;

	/* do board-specific initialisation */
	board_init();
	clock_init();
	
	//check
	read_uid(uid);
	//uid[0]++;
	encoding(passwd, uid);

	_mtext = (uint32_t*)(point_addr);
	for(led=0; led<8; led++)
	{
		if(passwd[led] != _mtext[led])
		{
			//printf("not match%d: %08X %08X\r\n", led, passwd[led], _mtext[led]);
			match=1;
		}
	}	
	/*if(0==match)
	{
		entry();
	}*/	
	if(match || (0==erase_flag))
	//if(match)
	{
		while (1) 
		{
			test_entry();	
		}
	}

#if defined(INTERFACE_USART) || defined (INTERFACE_USB)
	/* XXX sniff for a USART connection to decide whether to wait in the bootloader? */
	timeout = BOOTLOADER_DELAY;
#endif

#ifdef INTERFACE_I2C
# error I2C bootloader detection logic not implemented
#endif

	/* if the app left a cookie saying we should wait, then wait */
	if (should_wait()) {
		timeout = BOOTLOADER_DELAY;
	}

#ifdef BOARD_FORCE_BL_PIN

	/* if the force-BL pin state matches the state of the pin, wait in the bootloader forever */
	if (BOARD_FORCE_BL_VALUE == gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN)) {
		timeout = 0xffffffff;
	}

#endif

	/* look for the magic wait-in-bootloader value in backup register zero */


	/* if we aren't expected to wait in the bootloader, try to boot immediately */
	if (timeout == 0) {
		/* try to boot immediately */
		jump_to_app();

		/* if we returned, there is no app; go to the bootloader and stay there */
		timeout = 0;
	}

	/* configure the clock for bootloader activity */
	//clock_init();
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif
	/* start the interface */
	cinit(BOARD_INTERFACE_CONFIG, USART);
#else
	unsigned timeout = 0;
	timeout = main_init();
#endif
	while (1) {
		/* run the bootloader, possibly coming back after the timeout */
		bootloader(timeout);

		/* look to see if we can boot the app */
		jump_to_app();

		/* boot failed; stay in the bootloader forever next time */
		timeout = 0;
	}
}

void __attribute__ ((section(".reset_eb2"))) main_rb2_init(void)
{
	main_init();
}
