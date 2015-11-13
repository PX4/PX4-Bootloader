/*
 * hw_config.h
 *
 *  Created on: May 17, 2015
 *      Author: david_s5
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

/****************************************************************************
 * TARGET_HW_PX4_FMU_V1
 ****************************************************************************/

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

# define BOARD_FORCE_BL_PIN             GPIO10
# define BOARD_FORCE_BL_PORT            GPIOA
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPAEN
# define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
# define BOARD_FORCE_BL_STATE           0

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

# define BOARD_FORCE_BL_PIN_OUT         GPIO14
# define BOARD_FORCE_BL_PIN_IN          GPIO11
# define BOARD_FORCE_BL_PORT            GPIOE
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
# define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP

/****************************************************************************
 * TARGET_HW_PX4_FMU_V3
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V3)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v3.x"
# define USBPRODUCTID                   0x0012
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     11
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO11|GPIO10
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

# define BOARD_FORCE_BL_PIN_OUT         GPIO14
# define BOARD_FORCE_BL_PIN_IN          GPIO11
# define BOARD_FORCE_BL_PORT            GPIOE
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
# define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP

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
 * TARGET_HW_PX4_MAVSTATION_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_MAVSTATION_V1)

# define APP_LOAD_ADDRESS               0x08003000
# define APP_SIZE_MAX                   0x1C000
# define BOOTLOADER_DELAY               3000
# define BOARD_MAVSTATION
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "MAVSTATION BL v0.1"
# define USBPRODUCTID                   0x0014


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
# define BOARD_PIN_TX                   GPIO_USART1_TX
# define BOARD_PIN_RX                   GPIO_USART1_RX
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_APB2ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_APB2ENR_IOPAEN

# define BOARD_FORCE_BL_PIN             GPIO4
# define BOARD_FORCE_BL_PORT            GPIOA
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_APB2ENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_APB2ENR_IOPAEN
# define BOARD_FORCE_BL_PULL            GPIO_CNF_INPUT_PULL_UPDOWN // depend on external pull
# define BOARD_FORCE_BL_VALUE           0

# define BOARD_FLASH_SECTORS            116
# define BOARD_TYPE                     0x14
# define FLASH_SECTOR_SIZE              0x400

#else
# error Undefined Target Hardware
#endif

#endif /* HW_CONFIG_H_ */
