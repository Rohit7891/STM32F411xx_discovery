#include "stm32f411xx_drivers_spi.h"
#include "stm32f411xx.h"



static void spi_tx_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rx_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_overrun_err_interrupt_handle(SPI_Handle_t *pSPIHandle);





// periferal clock setup

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
void SPI_PCLK_Control(SPI_RegDef_t *pSPIx, uint8 EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}
	}
	else
	{

	}
}


uint8 SPI_CheckStatusOfControlRegister1Flag(SPI_RegDef_t *pSPIx, uint16 FlagName)
{
	if(pSPIx->SPI_CR1 & (1 << FlagName))
		return FLAG_SET;
	else
		return FLAG_RESET;
}



/////////////////////////////// spi initialize and deinitialize/////////////////////////////////

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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//configure spi cr1 register
	uint16 temp = 0;

	//enable spi peripheral clock
	SPI_PCLK_Control(pSPIHandle->pSPIx, ENABLE);

	//configure device mode
	temp |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR_BIT;

	//configure bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//clear bidi mode bit
		temp &= ~(1 << SPI_CR1_BIDI_MODE_BIT);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//set bidi mode bit
		temp |= (1 << SPI_CR1_BIDI_MODE_BIT);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//clear bidi mode bit
		temp &= ~(1 << SPI_CR1_BIDI_MODE_BIT);
		//set rx only bit
		temp |= (1 << SPI_CR1_RXONLY_BIT);
	}

	// configure ssm
	temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM_BIT;

	//configure clock speed
	temp |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR_BIT ;

	//configure data frame format
	temp |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF_BIT;

	//configure clock polarity
	temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL_BIT;

	//configure clock phase
	temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA_BIT;

	pSPIHandle->pSPIx->SPI_CR1 = temp;

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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}


//////////////////////////////////////////////// data send and recive////////////////////////////

/******************************************************************
 * @fn			-SPI_SendData
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @note		- this is data transfer in blocking mode (since the controller is blocked until the data is transfered).
 * 				  it is also known as polling mode/method.

 **************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8 *pTxBuffer, uint32 Length)
{
	while(Length > 0)
	{
		while (! (pSPIx->SPI_SR & (1 << SPI_SR_TXE_BIT )));
		//while (! (pSPIx->SPI_SR & (1 << SPI_SR_TXE_BIT )));			//orig
		if( SPI_CheckStatusOfControlRegister1Flag(pSPIx, SPI_CR1_DFF_FLAG) == FLAG_RESET)
		{
			// 8 bit data transfer
			pSPIx->SPI_DR = *pTxBuffer;
			pTxBuffer++;
			Length--;
		}
		else
		{
			// 16 bit data transfer
			pSPIx->SPI_DR = *((uint16 *)pTxBuffer);
			(uint16 *)pTxBuffer++;
			Length--;
			Length--;
		}
	}
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
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8 *pRxBuffer, uint32 Length)
{
	while(Length > 0)
	{
		// wait until rxne bit is set
		while (! (pSPIx->SPI_SR & (1 << SPI_SR_RXNE_BIT )));
		if( SPI_CheckStatusOfControlRegister1Flag(pSPIx, SPI_CR1_DFF_FLAG) == FLAG_RESET)
		{
			// 8 bit data receive
			*pRxBuffer = pSPIx->SPI_DR;
			pRxBuffer++;
			Length--;
		}
		else
		{
			// 16 bit data receive
			*((uint16 *)pRxBuffer) = pSPIx->SPI_DR;
			(uint16 *)pRxBuffer++;
			Length--;
			Length--;
		}
	}
}



// data send and receive using interrupt
uint8 SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8 *pTxBuffer, uint32 Length)
{
	//check the spi status
	uint8 spi_state = pSPIHandle->TxState;
	if (spi_state != SPI_BUSY_IN_TX)
	{
		//save txbuffer address and length in global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Length;

		//mark spi state as busy so that no other code can take over same spi peripheral
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//enable TXIE bit to receive interrupt whenever TXE bit is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE_BIT);
	}

	return spi_state;
}


uint8 SPI_RecieveData_IT(SPI_Handle_t *pSPIHandle, uint8 *pRxBuffer, uint32 Length)
{
	//check spi status
	uint8 spi_state = pSPIHandle->RxState;
	if(spi_state != SPI_BUSY_IN_RX)
	{
		// save rxbuffer and length in global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Length;

		//mark spi state as busy so that no other code can take over same spi peripheral
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//enable RXNEIE bit to receive interrupt whenever RXNE bit is set
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE_BIT);

	}

	return spi_state;
}



///////////////////////////////////////// IRQ setup and handling//////////////////////////////////////////////


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
 * @note        - refer Cortex-M4 Devices Generic User Guide
 ***********************************************************************/
void SPI_IRQInterruptConfig(uint8 IRQNumber, uint8 EnOrDi)
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
 * @note        - refer Cortex-M4 Devices Generic User Guide
 ***********************************************************************/
void SPI_IRQPriorityConfig( uint8 IRQNumber, uint8 IRQPriority)
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
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint16 temp1, temp2;

	//check for tx
	temp1 = (pSPIHandle->pSPIx->SPI_CR2) & (1 << SPI_CR2_TXEIE_BIT);
	temp2 = (pSPIHandle->pSPIx->SPI_SR) & (1 << SPI_SR_TXE_BIT);

	if(temp1 && temp2)
	{
		//call tx handling function
		spi_tx_interrupt_handle(pSPIHandle);
	}

	//check for rx
	temp1 = (pSPIHandle->pSPIx->SPI_CR2) & (1 << SPI_CR2_RXNEIE_BIT);
	temp2 = (pSPIHandle->pSPIx->SPI_SR) & (1 << SPI_SR_RXNE_BIT);

	if(temp1 && temp2)
	{
		//call tx handling function
		spi_rx_interrupt_handle(pSPIHandle);
	}

	//check for overrun error
	temp1 = (pSPIHandle->pSPIx->SPI_CR2) & (1 << SPI_CR2_ERRIE_BIT);
	temp2 = (pSPIHandle->pSPIx->SPI_SR) & (1 << SPI_SR_OVR_BIT);

	if(temp1 && temp2)
	{
		//call tx handling function
		spi_overrun_err_interrupt_handle(pSPIHandle);
	}


}


/******************************************************************
 * @fn			-SPIPeripheralControl
 *
 * @brief		- enables or disables the spi peripheral
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
**************************************************************************/
void SPIPeripheralControl(SPI_RegDef_t *pSPIx, uint8 EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE_BIT);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE_BIT);
	}
}



void SPI_SSI_config(SPI_RegDef_t *pSPIx, uint8 EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI_BIT);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI_BIT);
	}
}



void SPI_SSOE_config(SPI_RegDef_t *pSPIx, uint8 EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE_BIT);
	}
	else
	{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE_BIT);
	}
}



static void spi_tx_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if( SPI_CheckStatusOfControlRegister1Flag(pSPIHandle->pSPIx, SPI_CR1_DFF_FLAG) == FLAG_RESET)
		{
			// 8 bit data transfer
			pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
			pSPIHandle->pTxBuffer++;
			pSPIHandle->TxLen--;
		}
		else
		{
			// 16 bit data transfer
			pSPIHandle->pSPIx->SPI_DR = *((uint16 *)pSPIHandle->pTxBuffer);
			(uint16 *)pSPIHandle->pTxBuffer++;
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
		}
	if(! pSPIHandle->TxLen)
	{
		//tx is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


static void spi_rx_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if( SPI_CheckStatusOfControlRegister1Flag(pSPIHandle->pSPIx, SPI_CR1_DFF_FLAG) == FLAG_RESET)
		{
			// 8 bit data receive
			*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->SPI_DR;
			pSPIHandle->pRxBuffer++;
			pSPIHandle->RxLen--;
		}
		else
		{
			// 16 bit data receive
			*((uint16 *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
			(uint16 *)pSPIHandle->pRxBuffer++;
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
		}
	if(! pSPIHandle->RxLen)
	{
		//Rx is over
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


static void spi_overrun_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFLag(pSPIHandle);
	}
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}



void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE_BIT);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = SPI_READY;
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE_BIT);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_ClearOVRFLag(SPI_Handle_t *pSPIHandle)
{
	uint8 temp = 0;
	temp = pSPIHandle->pSPIx->SPI_DR;
	temp = pSPIHandle->pSPIx->SPI_SR;
	(void)temp;
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8 AppEvent)
{
	//this is a weak function implementation and may be changed by the application
}









