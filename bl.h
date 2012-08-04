/*
 * Common bootloader definitions.
 */

#pragma once

/***************************************************************************** 
 * Generic bootloader functions.
 */

/* board info forwarded from board-specific code to booloader */
struct boardinfo {
	uint32_t	board_type;
	uint32_t	board_rev;
	uint32_t	fw_size;
	uint32_t	systick_mhz;		/* systick input clock */

} __attribute__((packed));

extern struct boardinfo board_info;

extern void jump_to_app(void);
extern void bootloader(unsigned timeout);

/* generic timers */
#define NTIMERS		4
#define TIMER_BL_WAIT	0
#define TIMER_CIN	1
#define TIMER_LED	2
#define TIMER_DELAY	3
extern volatile unsigned timer[NTIMERS];	/* each timer decrements every millisecond if > 0 */

/* generic receive buffer for async reads */
extern void buf_put(uint8_t b);
extern int buf_get(void);

/***************************************************************************** 
 * Chip/board functions.
 */

/* LEDs */
#define LED_ACTIVITY	1
#define LED_BOOTLOADER	2

extern void led_on(unsigned led);
extern void led_off(unsigned led);
extern void led_toggle(unsigned led);

/* flash helpers from main_*.c */
extern unsigned flash_func_sector_size(unsigned sector);
extern void flash_func_erase_sector(unsigned sector);
extern void flash_func_write_word(unsigned address, uint32_t word);
extern uint32_t flash_func_read_word(unsigned address);

/*****************************************************************************
 * Interface in/output.
 */
extern void cinit(void *config);
extern void cfini(void);
extern int cin(void);
extern void cout(uint8_t *buf, unsigned len);
