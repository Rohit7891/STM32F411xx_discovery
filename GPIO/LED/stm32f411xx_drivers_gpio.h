/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: 27-Jan-2021
 *      Author: rohit
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"




typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				//possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;				//possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinOPType;				//possible values from @GPIO_PIN_OP_TYPE
	uint8_t GPIO_PinPuPdControl;		//possible values from @GPIO_PIN_PUPD_CONFIG
	uint8_t GPIO_PinAltFunction;		//possible values from @GPIO_PIN_ALTFUN
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
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);




// gpio initialize and deinitialize
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



// data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


// IRQ setup and handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
