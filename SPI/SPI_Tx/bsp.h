/******************************************************************************
 * 						@ board support package
 ********************************************************************************
* ----------SPI pins for Alternate function mode 05 ------------
* SPI1_NSS --> PA4
* SPI1_SCK --> PA5, PB3
* SPI1_MISO --> PA6
* SPI1_MOSI --> PA7
*
* SPI4_NSS --> PE4
* SPI4_SCK --> PE2
* SPI4_MISO --> PE5
* SPI4_MOSI --> PE6
*
*
***********************************************************************************/

#ifndef __BSP_H__
#define __BSP_H__

#include "stm32f411xx_drivers_gpio.h"
#include "stm32f411xx_drivers_spi.h"


//macros for gpio
#define USER_BUTTON     GPIO_PIN_0

// macros for spi


#define USE_SPI4		1
#define USE_NSS			1		// setting this macro to 1 automatically makes all changes for hardware slave management

/*if spi1 is used*/
#ifdef USE_SPI1

#define SPI_No					SPI1
#define SPI_PORT				GPIOA
#define ALT_FN_SPI				5

#define SPI_NSS_PIN				GPIO_PIN_4		// also 15 for port a
#define SPI_NSS_GPIO_PORT		GPIOA

#define SPI_SCK_PIN				GPIO_PIN_3		// 5 for port a, 3 for port b
#define SPI_SCK_GPIO_PORT		GPIOB

#define SPI_MISO_PIN			GPIO_PIN_3
#define SPI_MISO_GPIO_PORT		GPIOA

#define SPI_MOSI_PIN			GPIO_PIN_7		// 7 for port a, 5 for port b
#define SPI_MOSI_GPIO_PORT		GPIOA

#endif	// USE_SPI1

/*if spi4 is used*/
#if USE_SPI4

#define SPI_No					SPI4
#define SPI_PORT				GPIOE
#define ALT_FN_SPI				5

#define SPI_NSS_PIN				GPIO_PIN_11		//pin 4,11 for port e,
#define SPI_NSS_GPIO_PORT		SPI_PORT

#define SPI_SCK_PIN				GPIO_PIN_12		//pin 12,2 for port e,
#define SPI_SCK_GPIO_PORT		SPI_PORT

#define SPI_MISO_PIN			GPIO_PIN_5
#define SPI_MISO_GPIO_PORT		SPI_PORT

#define SPI_MOSI_PIN			GPIO_PIN_14		//pin 14,6 for port e,
#define SPI_MOSI_GPIO_PORT		SPI_PORT

#endif		//USE_SPI4



void Init_SPI1Pins(void);
void InitSPI1(void);
void Init_Button(void);


#endif
