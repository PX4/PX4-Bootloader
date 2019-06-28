/*
 * hw_config.h
 *
 *  Created on: May 17, 2015
 *      Author: david_s5
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

/****************************************************************************
 * 10-8--2016:
 *  To simplify the ripple effect on the tools, we will be using
 *  /dev/serial/by-id/<asterisk>PX4<asterisk> to locate PX4 devices. Therefore
 *  moving forward all Bootloaders must contain the prefix "PX4 BL "
 *  in the USBDEVICESTRING
 *  This Change will be made in an upcoming BL release
 ****************************************************************************/
/*
 * Define usage to configure a bootloader
 *
 *
 * Constant                example          Usage
 * APP_LOAD_ADDRESS     0x08004000            - The address in Linker Script, where the app fw is org-ed
 * BOOTLOADER_DELAY     5000                  - Ms to wait while under USB pwr or bootloader request
 * BOARD_FMUV2
 * INTERFACE_USB        1                     - (Optional) Scan and use the USB interface for bootloading
 * INTERFACE_USART      1                     - (Optional) Scan and use the Serial interface for bootloading
 * USBDEVICESTRING      "PX4 BL FMU v2.x"     - USB id string
 * USBPRODUCTID         0x0011                - PID Should match defconfig
 * BOOT_DELAY_ADDRESS   0x000001a0            - (Optional) From the linker script from Linker Script to get a custom
 *                                               delay provided by an APP FW
 * BOARD_TYPE           9                     - Must match .prototype boad_id
 * _FLASH_KBYTES        (*(uint16_t *)0x1fff7a22) - Run time flash size detection
 * BOARD_FLASH_SECTORS  ((_FLASH_KBYTES == 0x400) ? 11 : 23) - Run time determine the physical last sector
 * BOARD_FLASH_SECTORS   11                   - Hard coded zero based last sector
 * BOARD_FLASH_SIZE     (_FLASH_KBYTES*1024)-   Total Flash size of device, determined at run time.
 *                         (1024 * 1024)      - Hard coded Total Flash of device - The bootloader and app reserved will be deducted
 *                                              programmatically
 *
 * BOARD_FIRST_FLASH_SECTOR_TO_ERASE  2        - Optional sectors index in the flash_sectors table  (F4 only), to begin erasing.
 *                                               This is to allow sectors to be reserved for app fw usage. That will NOT be erased
 *                                               during a FW upgrade.
 *                                               The default is 0, and selects the first sector to be erased, as the 0th entry in the
 *                                               flash_sectors table. Which is the second physical sector of FLASH in the device.
 *                                               The first physical sector of FLASH is used by the bootloader, and is not defined
 *                                               in the table.
 *
 * APP_RESERVATION_SIZE (BOARD_FIRST_FLASH_SECTOR_TO_ERASE * 16 * 1024) - Number of bytes reserved by the APP FW. This number plus
 *                                                                        BOOTLOADER_RESERVATION_SIZE  will be deducted from
 *                                                                        BOARD_FLASH_SIZE to determine the size of the App FW
 *                                                                        and hence the address space of FLASH to erase and program.
 * USBMFGSTRING            "PX4 AP"            - Optional USB MFG string (default is '3D Robotics' if not defined.)
 * SERIAL_BREAK_DETECT_DISABLED                -  Optional prevent break selection on Serial port from entering or staying in BL
 *
 * * Other defines are somewhat self explanatory.
 */

/* Boot device selection list*/
#define USB0_DEV	0x01
#define SERIAL0_DEV	0x02
#define SERIAL1_DEV	0x04

/****************************************************************************
 * TARGET_HW_PX4_FMU_V4PRO
 ****************************************************************************/

#if defined(TARGET_HW_PX4_FMU_V4PRO)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v4.x PRO"
# define USBPRODUCTID                   0x0013
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     13
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO3
# define BOARD_PIN_LED_BOOTLOADER       GPIO11|GPIO1
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART  					USART2
# define BOARD_USART_CLOCK_REGISTER 	RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT      	RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART   			GPIOD
# define BOARD_PORT_USART_AF 			GPIO_AF7
# define BOARD_PIN_TX     				GPIO5
# define BOARD_PIN_RX		     		GPIO6
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT  	RCC_AHB1ENR_IOPDEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
*/

#endif

#if !defined(USBMFGSTRING)
# define USBMFGSTRING "3D Robotics"
#endif

#if !defined(USBVENDORID)
#  define USBVENDORID 0x26AC
#endif

#if !defined(APP_RESERVATION_SIZE)
#  define APP_RESERVATION_SIZE 0
#endif

#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
#  define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 0
#endif

#if defined(OVERRIDE_USART_BAUDRATE)
#  define USART_BAUDRATE OVERRIDE_USART_BAUDRATE
#else
#  define USART_BAUDRATE 115200
#endif

#if INTERFACE_USART
#  if !defined(BOARD_PORT_USART_TX)
#    define BOARD_PORT_USART_TX BOARD_PORT_USART
#    define BOARD_PORT_USART_RX BOARD_PORT_USART
#  endif
#  if !defined(BOARD_USART_PIN_CLOCK_BIT_TX)
#    define BOARD_USART_PIN_CLOCK_BIT_TX BOARD_USART_PIN_CLOCK_BIT
#    define BOARD_USART_PIN_CLOCK_BIT_RX BOARD_USART_PIN_CLOCK_BIT
#  endif
#endif

#if !defined(USB_DATA_ALIGN)
# define USB_DATA_ALIGN
#endif

#ifndef BOOT_DEVICES_SELECTION
#  define BOOT_DEVICES_SELECTION USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif

#ifndef BOOT_DEVICES_FILTER_ONUSB
#  define BOOT_DEVICES_FILTER_ONUSB USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif

#endif /* HW_CONFIG_H_ */
