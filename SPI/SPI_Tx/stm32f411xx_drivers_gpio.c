/***********************************************************************************
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
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8 EnorDi)
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
	uint32 temp = 0;
        uint8 temp1 = 0;
        uint8 port_code =0;

        // enable peripheral clock for gpio
        GPIO_PCLK_Control(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure mode for gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
                {
                  //configutr FTSR
                  EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                  // clear corresponding RTSR bit
                  EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                }
                else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
                {
                  //configure RTSR
                  EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                  //clear corresponding FTSR bit
                  EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                }
                else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
                {
                  //configure both, FTSR and RTSR
                  EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                  EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                }

                //configure GPIO port in SYSCFG_EXTICR
                temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
                temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
                SYSCFG_PCLK_EN();
                port_code = GPIO_PORT_NUMBER(pGPIOHandle->pGPIOx);
                SYSCFG->EXTICR[temp] = (port_code << (temp1 * 4));
                // ENABLE int EXTI interrupt delivery using IMR
                EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
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
		 uint8 temp1, temp2;
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
 * @fn		-	GPIO_ReadFromInputPin
 *
 * @brief	-	it reads the input value from the gpio input pin
 *
 * @param[in]	-	base address of the gpio port
 * @param[in]	-	pin number whose input value is to be read
 *
 * @return	-	data at the input pin

 ********************************************************************/
uint8 GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8 PinNumber)
{
      uint8 value = 0;
      value = (uint8)(( pGPIOx->IDR >> PinNumber ) & 0x00000001);
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

 *********************************************************************/
uint16 GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
  uint16 value = 0;
  value = (uint16)(pGPIOx->IDR);
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

 ********************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8 PinNumber, uint8 Value)
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

 ***********************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16 Value)
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

 ***********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8 PinNumber)
{
  pGPIOx->ODR ^= (1 << PinNumber);

}


// IRQ setup and handling

/******************************************************************
 * @fn                   GPIO_IRQInterruptConfig(uint8
 *
 * @brief       - enables or disables the interrupt register
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note        - refer Cortex-M4 Devices Generic User Guide
 ***********************************************************************/
void GPIO_IRQInterruptConfig(uint8 IRQNumber, uint8 EnOrDi)
{
  if(EnOrDi == ENABLE)
  {
    if(IRQNumber <= 31)
    {
      *NVIC_ISER0 |= (1 << IRQNumber);
    }
    else if(IRQNumber > 31 && IRQNumber <= 63)
    {
      *NVIC_ISER1 |= (1 << (IRQNumber % 32));
    }
    else if(IRQNumber > 63 && IRQNumber <= 95)
    {
      *NVIC_ISER2 |= (1 << (IRQNumber % 32));
    }
  }
  else
  {
    if(IRQNumber <= 31)
    {
      *NVIC_ICER0 |= (1 << IRQNumber);
    }
    else if(IRQNumber > 31 && IRQNumber <= 63)
    {
      *NVIC_ICER1 |= (1 << (IRQNumber % 32));
    }
    else if(IRQNumber > 63 && IRQNumber <= 95)
    {
      *NVIC_ICER2 |= (1 << (IRQNumber % 32));
    }
  }
}

/******************************************************************
 * @fn          GPIO_IRQPriorityConfig
 *
 * @brief       - configures the priority of the interrupt of given IRQ number
 *
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note        - refer Cortex-M4 Devices Generic User Guide
 ***********************************************************************/
void GPIO_IRQPriorityConfig( uint8 IRQNumber, uint8 IRQPriority)
{
  uint8 register_no, offset, shift_value;
  //uint32 volatile addr=0;
  register_no = (IRQNumber / 4);                //to get the IPRx number
  offset = (IRQNumber % 4);                     //to get the section in IPRx. One IPRx register has 4 priority registers. IPRx register is of 32 bit
  shift_value = (8 * offset) + (8 - PRIORITY_BITS_IMPLEMENTED) ;                      //priority is set in higher nibble of register.
  //addr = *(NVIC_IPR_BASE_ADDR + (register_no*4));
  *(NVIC_IPR_BASE_ADDR + register_no) |= (IRQPriority << shift_value);     //IPRx register is of 32 bit, so register_no is multiplied by 4
    /*
  *(NVIC_IPR_BASE_ADDR + (IRQNumber - offset)) |= (IRQPriority << shift_value);  // alternate way...not tested yet
    */
}


/******************************************************************
 * @fn			-
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

 **************************************************************************/
void GPIO_IRQHandling(uint8 PinNumber)
{
    //clear the exti pr register corresponding to pin number
    if(EXTI->PR & (1 << PinNumber))
    {
      //clear pr register
      EXTI->PR |= (1 << PinNumber);             // PR register bit can be cleared by setting corresponding bit to 1
    }
}

