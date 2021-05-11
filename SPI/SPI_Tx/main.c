/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Rohit
 * @brief          : Main program body
 *****************************************************************************/

#include "stm32f411xx_drivers_gpio.h"
#include "stm32f411xx_drivers_spi.h"
#include "bsp.h"
#include<string.h>


void delay(uint32 ms);

int main(void)
{
	uint8 data[] = "hello world...i am working on spi";
	uint8 len = sizeof(data);
	//configure gpio pin as input
	Init_Button();
	// configure gpio pins to act as spi
	Init_SPI1Pins();
	//configure spi1 registers
	InitSPI1();
	/*
	 * making SSOE 1 makes NSS output enable.
	 * the nss pin is automatically managed by the hardware
	 * i.e. when spe=1, nss will be pulled to low
	 * and nss pin will be high when spe=0
	 */
#if USE_NSS
	SPI_SSOE_config(SPI_No, ENABLE);
#else
	// set ssi bit to avoid MODF error
	SPI_SSI_config(SPI_No, ENABLE );
#endif

	// enable spi peripheral after all parameters are configured
    /* Loop forever */
	while(1)
	{
		//wait until button is pressed
		if(GPIO_ReadFromInputPin(GPIOA, USER_BUTTON))
		{
			delay(100);			//considering debouncing effect
			if(GPIO_ReadFromInputPin(GPIOA, USER_BUTTON))
			{
				//enable spi peripheral
				SPIPeripheralControl(SPI_No, ENABLE);

				//send length information of user data (to arduino)
				SPI_SendData(SPI_No, &len, 1);

				//send data
				SPI_SendData(SPI_No, data, sizeof(data));

				///wait untill spi is busy
				while (SPI_No->SPI_SR & (1 << SPI_SR_BSY_BIT ));

				//close spi communication after transmission is complete
				SPIPeripheralControl(SPI_No, DISABLE);
				delay(50);
			}
		}
	}
	return 0;
}


void delay(uint32 ms)
{
	uint32 i,j;
	for(i=1600;i>0;i--)
		for(j=ms;j>0;j--);
}
