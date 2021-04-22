/*
 * stm32f411xx_gpio.c
 *
 *  Created on: 27-Jan-2021
 *      Author: rohit
 */

#include "stm32f411xx_drivers_gpio.h"



GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t *) GPIOA_BASE_ADDR;
GPIO_RegDef_t *pGPIOB = (GPIO_RegDef_t *) GPIOB_BASE_ADDR;
GPIO_RegDef_t *pGPIOC = (GPIO_RegDef_t *) GPIOC_BASE_ADDR;
GPIO_RegDef_t *pGPIOD = (GPIO_RegDef_t *) GPIOD_BASE_ADDR;
GPIO_RegDef_t *pGPIOE = (GPIO_RegDef_t *) GPIOE_BASE_ADDR;
GPIO_RegDef_t *pGPIOH = (GPIO_RegDef_t *) GPIOH_BASE_ADDR;




// periferal clock setup
/******************************************************************
 * @fn			-	GPIO_PCLK_Control
 *
 * @brief		- the function enables or disables the peripheral clock if given GPIO PORT
 *
 * @param[in]	- base address of the peripheral gpio
 * @param[in]	- ENABLE or DISABLE
 *
 * @return		- none
 *
 * @note		- none

 */
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == GPIO_PIN_ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}




// gpio initialize and deinitialize
/******************************************************************
 * @fn			-	GPIO_Init
 *
 * @brief		-	It initializes the gpio port
 *
 * @param[in]	-	base address of the gpio PORT
 *
 * @return		-	none
 *
 * @note		-	none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1. configure mode for gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction <= GPIO_MODE_IT_FT)
                {
                  //configutr FTSR
                }
                else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction <= GPIO_MODE_IT_RT)
                {
                  //configure RTSR
                }
                else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction <= GPIO_MODE_IT_RFT)
                {
                  //configure both, FTSR and RTSR
                }
                
                //configure GPIO port in SYSCFG_EXTICR
                // ENABLE int EXTI interrupt delivery using IMR

	}

	//2. configure speed for gpio
	temp = 0;
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR &= (3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. configure pull up pull down setting
	temp = 0;
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR &= (3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. configure output type for gpio
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure alternate functionality
	temp=0;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFR[0] &= (4 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFR[0] |= temp;
		}
		else
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8));
			pGPIOHandle->pGPIOx->AFR[0] &= (4 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8));
			pGPIOHandle->pGPIOx->AFR[1] |= temp;
		};
		/*
		 * //alternate logic:
		 uint8_t temp1, temp2;
		 temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		 temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << 4 * temp2);
		 pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
		 */
	}

}


/******************************************************************
 * @fn			-	GPIO_DeInit
 *
 * @brief		- it deinitializes the gpio to its initial state
 *
 * @param[in]	-	base address of the gpio port
 *
 * @return		-	none
 *
 * @note		-	none

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
        if (pGPIOx == GPIOA)
        {
                GPIOA_REG_RESET();
        }
        else if (pGPIOx == GPIOB)
        {
                GPIOB_REG_RESET();
        }
        else if (pGPIOx == GPIOC)
        {
                GPIOC_REG_RESET();
        }
        else if (pGPIOx == GPIOD)
        {
                GPIOD_REG_RESET();
        }
        else if (pGPIOx == GPIOE)
        {
                GPIOE_REG_RESET();
        }
        else if (pGPIOx == GPIOH)
        {
                GPIOH_REG_RESET();
        }
}



// data read and write
/******************************************************************
 * @fn			-	GPIO_ReadFromInputPin
 *
 * @brief		-	it reads the input value from the gpio input pin
 *
 * @param[in]	-	base address of the gpio port
 * @param[in]	-	pin number whose input value is to be read
 *
 * @return		-	the input data read from the input pin
 *
 * @note		-	none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
      uint8_t value = 0;
      value = (uint8_t)(( pGPIOx->IDR >> PinNumber ) & 0x00000001);
      return value;
}


/******************************************************************
 * @fn			-	GPIO_ReadFromInputPort
 *
 * @brief		-	it reads the data from entire input PORT
 *
 * @param[in]	-	base address of the gpio port
 *
 * @return		-	data read from the entire port
 *
 * @note		-	none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
  uint16_t value = 0;
  value = (uint16_t)(pGPIOx->IDR);
  return value;
}



/******************************************************************
 * @fn
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
  if (Value == GPIO_PIN_SET)
  {
    pGPIOx->ODR |= (1 << PinNumber);
  }
  else
  {
    pGPIOx->ODR &= ~(1 << PinNumber);
  }
  

}


/******************************************************************
 * @fn
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
  pGPIOx->ODR = Value;

}

/******************************************************************
 * @fn
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
  pGPIOx->ODR ^= (1 << PinNumber);

}


// IRQ setup and handling

/******************************************************************
 * @fn
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-

 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
{

}

/******************************************************************
 * @fn
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

