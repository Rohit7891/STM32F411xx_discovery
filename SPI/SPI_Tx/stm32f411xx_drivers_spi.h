#ifndef __STM32F411XX_DRIVERS_SPI_H__
#define __STM32F411XX_DRIVERS_SPI_H__

#include "stm32f411xx.h"






typedef struct
{
	uint8 SPI_DeviceMode;					//either master or slave
	uint8 SPI_BusConfig;					// full duplex, half duplex etc
	uint8 SPI_SclkSpeed;					// clocl speed
	uint8 SPI_DFF;							//data frame format
	uint8 SPI_CPOL;							//clock polarity
	uint8 SPI_CPHA;							//clock phase
	uint8 SPI_SSM;							//SOFTWARE SLAVE MANAGEMENT
}SPI_Config_t;




typedef struct
{
	SPI_RegDef_t 	*pSPIx;			// pointer to hold the base address of SPIx(1,2,3,4,5)
	SPI_Config_t	SPI_Config;		// structure to hold configuration details of desired spi
	uint32			TxLen;			// to store tx length
	uint32			RxLen;			// to store rx length
	uint8 			*pTxBuffer;		// to store application tx buffer address
	uint8 			*pRxBuffer;		// to store application rx buffer address
	uint8			TxState;		// to store tx state
	uint8 			RxState;		// to store rx state
}SPI_Handle_t;


/*
 * SPI appliction states
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * Appliction Event
 */
#define SPI_EVENT_TX_CMPLT			0
#define SPI_EVENT_RX_CMPLT			1
#define SPI_EVENT_OVR_ERR			2
#define SPI_EVENT_CRC_ERR			3



/*
 @SPI_DeviceMode
*/
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1		//full duplex
#define SPI_BUS_CONFIG_HD					2		//half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3		//simplex with only rx

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7


/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1


/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0


/*
 * @SPI_CPHA
 */
#define SPI_CPHASE_HIGH						1
#define SPI_CPHASE_LOW						0


/*
 * @SPI_SSM
 */
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0



/*
 * @for spi_cr1 register
 */
#define SPI_CR1_CPHA_FLAG			( 1 << SPI_CR1_CPHA_BIT )
#define SPI_CR1_CPOL_FLAG			( 1 << SPI_CR1_CPOL_BIT )
#define SPI_CR1_MSTR_FLAG			( 1 << SPI_CR1_MSTR_BIT )
#define SPI_CR1_BR_FLAG				( 1 << SPI_CR1_BR_BIT )
#define SPI_SPE_CPHA_FLAG			( 1 << SPI_SPE_CPHA_BIT )
#define SPI_CR1_LSB_FIRST_FLAG		( 1 << SPI_CR1_LSB_FIRST_BIT )
#define SPI_CR1_SSI_FLAG			( 1 << SPI_CR1_SSI_BIT )
#define SPI_CR1_SSM_FLAG			( 1 << SPI_CR1_SSM_BIT )
#define SPI_CR1_RXONLY_FLAG			( 1 << SPI_CR1_RXONLY_BIT )
#define SPI_CR1_DFF_FLAG			( 1 << SPI_CR1_DFF_BIT )
#define SPI_CR1_CRC_NEXT_FLAG		( 1 << SPI_CR1_CRC_NEXT_BIT )
#define SPI_CR1_CRC_EN_FLAG			( 1 << SPI_CR1_CRC_EN_BIT )
#define SPI_CR1_BIDI_OE_FLAG		( 1 << SPI_CR1_BIDI_OE_BIT )
#define SPI_CR1_BIDI_MODE_FLAG		( 1 << SPI_CR1_BIDI_MODE_BIT )


/*
 * @ for spi_cr2
 */
#define SPI_CR2_RXDMAEN_FLAG		( 1 << SPI_CR2_RXDMAEN_BIT )
#define SPI_CR2_TXDMAEN_FLAG		( 1 << SPI_CR2_TXDMAEN_BIT )
#define SPI_CR2_SSOE_FLAG			( 1 << SPI_CR2_SSOE_BIT )
#define SPI_CR2_FRF_FLAG			( 1 << SPI_CR2_FRF_BIT )
#define SPI_CR2_ERRIE_FLAG			( 1 << SPI_CR2_ERRIE_BIT )
#define SPI_CR2_RXNEIE_FLAG			( 1 << SPI_CR2_RXNEIE_BIT )
#define SPI_CR2_TXEIE_FLAG			( 1 << SPI_CR2_TXEIE_BIT )


/*
 * FOR SPI_SR flag
 */
#define SPI_SR_RXNE_FLAG			( 1 << SPI_SR_RXNE_BIT )
#define SPI_SR_TXE_FLAG				( 1 << SPI_SR_TXE_BIT )
#define SPI_SR_CHSIDE_FLAG			( 1 << SPI_SR_CHSIDE_BIT )
#define SPI_SR_UDR_FLAG				( 1 << SPI_SR_UDR_BIT )
#define SPI_SR_CRC_ERR_FLAG			( 1 << SPI_SR_CRC_ERR_BIT )
#define SPI_SR_MODF_FLAG			( 1 << SPI_SR_MODF_BIT )
#define SPI_SR_OVR_FLAG				( 1 << SPI_SR_OVR_BIT )
#define SPI_SR_BSY_FLAG				( 1 << SPI_SR_BSY_BIT )
#define SPI_SR_FRE_FLAG				( 1 << SPI_SR_FRE_BIT )




/***********************************************************************************************************
 * 								APIs supported by the MCU
 * 				for information on the APIs check the function definition
 *
 ************************************************************************************************************/

// periferal clock setup
void SPI_PCLK_Control(SPI_RegDef_t *pSPIx, uint8 EnorDi);




// spi initialize and deinitialize
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


// data send and recive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8 *pTxBuffer, uint32 Length);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8 *pRxBuffer, uint32 Length);

// data send and receive using interrupt
uint8 SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8 *pTxBuffer, uint32 Length);
uint8 SPI_RecieveData_IT(SPI_Handle_t *pSPIHandle, uint8 *pRxBuffer, uint32 Length);

//check status register flag
uint8 SPI_CheckStatusOfControlRegister1Flag(SPI_RegDef_t *pSPIx, uint16 FlagName);

// IRQ setup and handling
void SPI_IRQInterruptConfig(uint8 IRQNumber, uint8 EnOrDi);
void SPI_IRQPriorityConfig( uint8 IRQNumber, uint8 IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


// other spi peripheral functions
void SPIPeripheralControl(SPI_RegDef_t *pSPIx, uint8 EnOrDi);
void SPI_SSI_config(SPI_RegDef_t *pSPIx, uint8 EnOrDi);
void SPI_SSOE_config(SPI_RegDef_t *pSPIx, uint8 EnOrDi);

void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFLag(SPI_Handle_t *pSPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8 AppEvent);

#endif
