


#include "bsp.h"



/******************************************************************
 * @fn			-Init_SPI1Pins
 *
 * @brief		- the function configures the gpio pins needed for SPIx.
 *
 * @note		- the gpio port and pins for spi can be configured from bsp.h file.

 **************************************************************************/
void Init_SPI1Pins(void)
{
	GPIO_Handle_t SPI_pins;
	SPI_pins.pGPIOx = SPI_PORT;
	SPI_pins.GPIO_PinConfig.GPIO_PinAltFunction = ALT_FN_SPI;
	SPI_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//configure mosi pin
	SPI_pins.pGPIOx = SPI_MOSI_GPIO_PORT;					//not needed if all spi pins are on the same port
	SPI_pins.GPIO_PinConfig.GPIO_PinNumber = SPI_MOSI_PIN;
	GPIO_Init(&SPI_pins);

	//configure sck pin
	SPI_pins.pGPIOx = SPI_SCK_GPIO_PORT;					//not needed if all spi pins are on the same port
	SPI_pins.GPIO_PinConfig.GPIO_PinNumber = SPI_SCK_PIN;
	GPIO_Init(&SPI_pins);

#if USE_NSS
	//configure nss pin
	SPI_pins.pGPIOx = SPI_NSS_GPIO_PORT;					//not needed if all spi pins are on the same port
	SPI_pins.GPIO_PinConfig.GPIO_PinNumber = SPI_NSS_PIN;
	GPIO_Init(&SPI_pins);
#endif

}


/******************************************************************
 * @fn			-InitSPI1
 *
 * @brief		- it configures the parameters for spi1
 *
 * @param[in]	-none
 * @return		-none
 *
 **************************************************************************/
void InitSPI1(void)
{
	SPI_Handle_t SPI1_Handle;
	SPI1_Handle.pSPIx = SPI_No;
	SPI1_Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1_Handle.SPI_Config.SPI_CPHA = SPI_CPHASE_LOW;
	SPI1_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1_Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI1_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1_Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
#if USE_NSS
	SPI1_Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;
#else
	SPI1_Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;			//disable for hardware slave management. enable for software slave management for nss pin
#endif

	SPI_Init(&SPI1_Handle);
}


void Init_Button(void)
{
	GPIO_Handle_t Button;
	Button.pGPIOx = GPIOA;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Button.GPIO_PinConfig.GPIO_PinNumber = USER_BUTTON;
	Button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&Button);
}
