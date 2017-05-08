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

*  BOARD_TYPE           9                     - Must match .prototype boad_id
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
#if  defined(TARGET_HW_PX4_FMU_V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMU
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v1.x"
# define USBPRODUCTID                   0x0010
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     5
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO15
# define BOARD_PIN_LED_BOOTLOADER       GPIO14
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART  					USART1
# define BOARD_USART_CLOCK_REGISTER 	RCC_APB2ENR
# define BOARD_USART_CLOCK_BIT      	RCC_APB2ENR_USART1EN

# define BOARD_PORT_USART   			GPIOB
# define BOARD_PORT_USART_AF 			GPIO_AF7
# define BOARD_PIN_TX     				GPIO6
# define BOARD_PIN_RX		     		GPIO7
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT  	RCC_AHB1ENR_IOPBEN

/*
 * Uncommenting this allows to force the bootloader through
 * the PPM-in pin. Some receivers pull their PPM output low
 * when the transmitter is off, resulting in a stuck bootup,
 * so this feature is best disabled.
 *
 * # define BOARD_FORCE_BL_PIN             GPIO10
 * # define BOARD_FORCE_BL_PORT            GPIOA
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPAEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
 * # define BOARD_FORCE_BL_STATE           0
 */

/****************************************************************************
 * TARGET_HW_PX4_FMU_V2
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V2)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v2.x"
# define USBPRODUCTID                   0x0011
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     9
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO12
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
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

/****************************************************************************
 * TARGET_HW_PX4_FMU_V4
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V4)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v4.x"
# define USBPRODUCTID                   0x0012
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     11
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

# define BOARD_USART  					USART1
# define BOARD_USART_CLOCK_REGISTER 	RCC_APB2ENR
# define BOARD_USART_CLOCK_BIT      	RCC_APB2ENR_USART1EN

# define BOARD_PORT_USART   			GPIOB
# define BOARD_PORT_USART_AF 			GPIO_AF7
# define BOARD_PIN_TX     				GPIO6
# define BOARD_PIN_RX		     		GPIO7
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT  	RCC_AHB1ENR_IOPBEN

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
/****************************************************************************
 * TARGET_HW_PX4_FMU_V4_PRO
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V4_PRO)

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

/****************************************************************************
 * TARGET_HW_PX4_FMU_V4_PRO
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V5)

# define APP_LOAD_ADDRESS               0x08008000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v5.x"
# define USBPRODUCTID                   0x0032
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     50
# define _FLASH_KBYTES                  (*(uint16_t *)0x1ff0f442)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 7 : 11)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       16

# define BOARD_PIN_LED_ACTIVITY         GPIO7 // BLUE
# define BOARD_PIN_LED_BOOTLOADER       GPIO6 // GREEN
# define BOARD_PORT_LEDS                GPIOC
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPCEN
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

/****************************************************************************
 * TARGET_HW_MINDPX_V2
 ****************************************************************************/

#elif  defined(TARGET_HW_MINDPX_V2)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "MindPX BL FMU v2.x"
# define USBPRODUCTID                   0x0030
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     88
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO8
# define BOARD_PORT_LEDS                GPIOA
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPAEN
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

/****************************************************************************
 * TARGET_HW_PX4_FLOW_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FLOW_V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FLOW
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "PX4 BL FLOW v1.3"
# define USBPRODUCTID                   0x0015

# define BOARD_TYPE                     6
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO3
# define BOARD_PIN_LED_BOOTLOADER       GPIO2
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

/****************************************************************************
 * TARGET_HW_PX4_DISCOVERY_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_DISCOVERY_V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_DISCOVERY
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "PX4 BL DISCOVERY"
# define USBPRODUCTID                   0x0001

# define BOARD_TYPE                     99
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         GPIO12
# define BOARD_PIN_LED_BOOTLOADER       GPIO13
# define BOARD_PORT_LEDS                GPIOD
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPDEN
# define BOARD_LED_ON                   gpio_set
# define BOARD_LED_OFF                  gpio_clear

/****************************************************************************
 * TARGET_HW_PX4_PIO_V1 or TARGET_HW_PX4_PIO_V2
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_PIO_V1) ||  defined(TARGET_HW_PX4_PIO_V2)

# define APP_LOAD_ADDRESS               0x08001000
# define APP_SIZE_MAX                   0xf000
# define BOOTLOADER_DELAY               200
# define BOARD_PIO
# define INTERFACE_USB                	0
# define INTERFACE_USART                1
# define USBDEVICESTRING                ""
# define USBPRODUCTID                   -1

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO14
# define BOARD_PIN_LED_BOOTLOADER       GPIO15
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS_REGISTER      RCC_APB2ENR
# define BOARD_CLOCK_LEDS               RCC_APB2ENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOA
# define BOARD_PIN_TX                   GPIO_USART2_TX
# define BOARD_PIN_RX                   GPIO_USART2_RX
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_APB2ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_APB2ENR_IOPAEN

# define BOARD_FORCE_BL_PIN             GPIO5
# define BOARD_FORCE_BL_PORT            GPIOB
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_APB2ENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_APB2ENR_IOPBEN
# define BOARD_FORCE_BL_PULL            GPIO_CNF_INPUT_FLOAT // depend on external pull
# define BOARD_FORCE_BL_VALUE           BOARD_FORCE_BL_PIN

# define BOARD_FLASH_SECTORS            60
# define BOARD_TYPE                     10
# define FLASH_SECTOR_SIZE              0x400

/****************************************************************************
 * TARGET_HW_PX4_PIO_V3
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_PIO_V3)

# define APP_LOAD_ADDRESS               0x08001000
# define APP_SIZE_MAX                   0x3f000
# define BOOTLOADER_DELAY               200
# define BOARD_PIO
# define INTERFACE_USB                	0
# define INTERFACE_USART                1
# define USBDEVICESTRING                ""
# define USBPRODUCTID                   -1

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         0
# define BOARD_PIN_LED_BOOTLOADER       GPIO13
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS_REGISTER      RCC_AHBENR
# define BOARD_CLOCK_LEDS               RCC_AHBENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOA
# define BOARD_PORT_USART_AF 			GPIO_AF7
# define BOARD_PIN_TX     				GPIO2
# define BOARD_PIN_RX		     		GPIO3
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHBENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHBENR_IOPAEN

# define BOARD_FORCE_BL_PIN             GPIO5
# define BOARD_FORCE_BL_PORT            GPIOB
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHBENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHBENR_IOPBEN
# define BOARD_FORCE_BL_PULL            GPIO_PUPD_NONE // depend on external pull
# define BOARD_FORCE_BL_VALUE           BOARD_FORCE_BL_PIN

# define BOARD_FLASH_SECTORS            (128-4) /* application #sec - bootloader - sec */
# define BOARD_TYPE                     13
# define FLASH_SECTOR_SIZE              0x800

/****************************************************************************
 * TARGET_HW_PX4_AEROCORE_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_AEROCORE_V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_AEROCORE
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "Gumstix BL AEROCORE"
# define USBPRODUCTID                   0x1001

# define BOARD_TYPE                     98
# define BOARD_FLASH_SECTORS            23
# define BOARD_FLASH_SIZE               (2048 * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO10  // Yellow
# define BOARD_PIN_LED_BOOTLOADER       GPIO9   // Blue
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_FORCE_BL_PIN_OUT         GPIO0   // J11 header, pin 1
# define BOARD_FORCE_BL_PIN_IN          GPIO1   // J11 header, pin 3
# define BOARD_FORCE_BL_PORT            GPIOB
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPBEN
# define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP

/****************************************************************************
 * TARGET_HW_TAP_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_TAP_V1)

# define APP_LOAD_ADDRESS               0x0800C000
# define BOOTLOADER_DELAY               5000
# define BOARD_TAP
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL TAP v1.x"
# define USBPRODUCTID                   0x0040
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     64
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)
# define BOARD_FIRST_FLASH_SECTOR_TO_ERASE  2
# define APP_RESERVATION_SIZE			(2 * 16 * 1024) /* 2 16 Kib Sectors */
# define OSC_FREQ                       16

# define BOARD_USART  					USART2
# define BOARD_USART_CLOCK_REGISTER 	RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT      	RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART   			GPIOA
# define BOARD_PORT_USART_AF 			GPIO_AF7
# define BOARD_PIN_TX     				GPIO2
# define BOARD_PIN_RX		     		GPIO3
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT  	RCC_AHB1ENR_IOPAEN

# define BOARD_PIN_LED_ACTIVITY         GPIO4
# define BOARD_PIN_LED_BOOTLOADER       GPIO5
# define BOARD_PORT_LEDS                GPIOC
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPCEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_POWER_PIN_OUT            GPIO4
# define BOARD_POWER_PORT               GPIOA
# define BOARD_POWER_CLOCK_REGISTER     RCC_AHB1ENR
# define BOARD_POWER_CLOCK_BIT          RCC_AHB1ENR_IOPAEN
# define BOARD_POWER_ON                 gpio_set
# define BOARD_POWER_OFF                gpio_clear
# undef  BOARD_POWER_PIN_RELEASE		/* Leave pin enabling Power - un comment to release (disable power)*/
# define USBMFGSTRING                   "The Autopilot"
# define USB_FORCE_DISCONNECT			1
#define  SERIAL_BREAK_DETECT_DISABLED   1

/****************************************************************************
 * TARGET_HW_CRAZYFLIE
 ****************************************************************************/

#elif  defined(TARGET_HW_CRAZYFLIE)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_CRAZYFLIE
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "Crazyflie BL"
# define USBPRODUCTID                   0x0016

# define BOARD_TYPE                     12
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         GPIO0
# define BOARD_PIN_LED_BOOTLOADER       GPIO2
# define BOARD_PORT_LEDS                GPIOC
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPCEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USB_VBUS_SENSE_DISABLED

# define USBMFGSTRING                   "Bitcraze AB"

/****************************************************************************
 * TARGET_HW_AUAV_X2V1
 ****************************************************************************/

#elif  defined(TARGET_HW_AUAV_X2V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL AUAV X2.1"
# define USBPRODUCTID                   0x0021
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     33
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO12
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
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

# define USBMFGSTRING                   "AUAV"

/****************************************************************************
 * TARGET_HW_PX4_AEROFC_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_AEROFC_V1)

# define APP_LOAD_ADDRESS			0x0800C000
# define BOOTLOADER_DELAY			5000
# define INTERFACE_USB				0
# define USBDEVICESTRING			""
# define USBPRODUCTID				0

# define INTERFACE_USART			1
# define BOOT_DELAY_ADDRESS			0x000001a0

# define BOARD_TYPE				65
# define BOARD_FLASH_SECTORS			11
# define BOARD_FLASH_SIZE			(1024 * 1024)
# define BOARD_FIRST_FLASH_SECTOR_TO_ERASE	2
# define APP_RESERVATION_SIZE			(2 * 16 * 1024) /* 2 16 Kib Sectors */

# define OSC_FREQ				16

# define BOARD_PIN_LED_ACTIVITY			GPIO12
# define BOARD_PIN_LED_BOOTLOADER		GPIO9 | GPIO10 | GPIO11 | GPIO13 | GPIO14 | GPIO15
# define BOARD_PORT_LEDS			GPIOE
# define BOARD_CLOCK_LEDS			RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON				gpio_clear
# define BOARD_LED_OFF				gpio_set

# define BOARD_USART				USART2
# define BOARD_USART_CLOCK_REGISTER		RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT			RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART			GPIOA
# define BOARD_PORT_USART_AF			GPIO_AF7
# define BOARD_PIN_TX				GPIO2
# define BOARD_PIN_RX				GPIO3
# define BOARD_USART_PIN_CLOCK_REGISTER		RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT		RCC_AHB1ENR_IOPAEN

# define BOARD_FORCE_BL_PIN			GPIO11
# define BOARD_FORCE_BL_PORT			GPIOA
# define BOARD_FORCE_BL_CLOCK_REGISTER		RCC_AHB1ENR
# define BOARD_FORCE_BL_CLOCK_BIT		RCC_AHB1ENR_IOPAEN
# define BOARD_FORCE_BL_PULL			GPIO_PUPD_PULLDOWN
# define BOARD_FORCE_BL_STATE			1

#else
# error Undefined Target Hardware
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

#endif /* HW_CONFIG_H_ */
