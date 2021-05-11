/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: 27-Jan-2021
 *      Author: rohit
 */

#ifndef __STM32F411XX_DRIVERS_GPIO_H__
#define __STM32F411XX_DRIVERS_GPIO_H__

#include "stm32f411xx.h"




typedef struct
{
	uint8 GPIO_PinNumber;
	uint8 GPIO_PinMode;				//possible values from @GPIO_PIN_MODES
	uint8 GPIO_PinSpeed;				//possible values from @GPIO_PIN_SPEED
	uint8 GPIO_PinOPType;				//possible values from @GPIO_PIN_OP_TYPE
	uint8 GPIO_PinPuPdControl;		//possible values from @GPIO_PIN_PUPD_CONFIG
	uint8 GPIO_PinAltFunction;		//possible values from @GPIO_PIN_ALTFUN
}GPIO_PinConfig_t;



typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;



/*
 * gpio pin number
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0		//input mode
#define GPIO_MODE_OUT		1		//output mode
#define GPIO_MODE_ALTFN		2		//alternate function mode
#define GPIO_MODE_ANALOG	3		//analog input mode
#define GPIO_MODE_IT_FT		4		//input interrupt falling edge
#define GPIO_MODE_IT_RT		5		//input interrupt rising edge
#define GPIO_MODE_IT_RFT	6		//input interrupt rising edge falling edge



/*
 * @GPIO_PIN_OP_TYPE
 * gpio pin possible output types
 */
#define GPIO_OP_TYPE_PP		0               // push pull type
#define GPIO_OP_TYPE_OD		1               // open drain type



/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible speed
 */
#define GPIO_SPEED_SLOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3



/*
 * @GPIO_PIN_PUPD_CONFIG
 * gpio pin pull up pull down pin configuration
 */
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2



/***********************************************************************************************************
 * 								APIs supported by the MCU
 * 				for information on the APIs check the function definition
 *
 ************************************************************************************************************/

// periferal clock setup
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8 EnorDi);




// gpio initialize and deinitialize
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



// data read and write
uint8 GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8 PinNumber);
uint16 GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8 PinNumber, uint8 Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16 Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8 PinNumber);


// IRQ setup and handling
void GPIO_IRQInterruptConfig(uint8 IRQNumber, uint8 EnOrDi);
void GPIO_IRQPriorityConfig( uint8 IRQNumber, uint8 IRQPriority);
void GPIO_IRQHandling(uint8 PinNumber);


// other spi peripheral functions



#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */








