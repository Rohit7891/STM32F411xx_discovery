#ifndef __STM32F411XX_H__
#define __STM32F411XX_H__


#define uint8           unsigned char
#define uint16          unsigned short int
#define uint32          unsigned int
#define __vo 	        volatile
#define int8            signed char
#define int16           signed short int
#define int32           signed int
#define __weak			__attribute__ ((weak))


/****************************************************************************************************
 * 								Processor specific details
 ****************************************************************************************************/

//Arm Cortex Mx processor Interrupt set-enable register address
#define NVIC_ISER0              ((__vo uint32*)0xE000E100UL)
#define NVIC_ISER1              ((__vo uint32*)0xE000E104UL)
#define NVIC_ISER2              ((__vo uint32*)0xE000E104UL)
#define NVIC_ISER3              ((__vo uint32*)0xE000E10CUL)
#define NVIC_ISER4              ((__vo uint32*)0xE000E120UL)
#define NVIC_ISER5              ((__vo uint32*)0xE000E124UL)
#define NVIC_ISER6              ((__vo uint32*)0xE000E128UL)
#define NVIC_ISER7              ((__vo uint32*)0xE000E130UL)

//Arm Cortex Mx processor Interrupt clear-enable register address
#define NVIC_ICER0              ((__vo uint32*)0xE000E180UL)
#define NVIC_ICER1              ((__vo uint32*)0xE000E184UL)
#define NVIC_ICER2              ((__vo uint32*)0xE000E188UL)
#define NVIC_ICER3              ((__vo uint32*)0xE000E18CUL)
#define NVIC_ICER4              ((__vo uint32*)0xE000E190UL)
#define NVIC_ICER5              ((__vo uint32*)0xE000E194UL)
#define NVIC_ICER6              ((__vo uint32*)0xE000E198UL)
#define NVIC_ICER7              ((__vo uint32*)0xE000E19CUL)





#define PRIORITY_BITS_IMPLEMENTED       4               // it is different for different processor make...it is 3 in casse of TI



/****************************************************************************************************
 * 								Base address
 ****************************************************************************************************/
#define FALSH_BASE_ADDR			0x08000000UL
#define SRAM1_BASE_ADDR			0x20000000UL
#define ROM						FALSH_BASE_ADDR

#define NVIC_IPR_BASE_ADDR      (__vo uint32*)0xE000E400UL

//periferals
#define AHB1_BASE_ADDR 			0x40020000UL
#define AHB2_BASE_ADDR			0x40010000UL
#define APB1_BASE_ADDR			0x40000000UL
#define APB2_BASE_ADDR			0x40010000UL
#define RCC_BASE_ADDR			0x40023800UL
/*
#define RCC_CR_ADDR				(RCC_BASE_ADDR + 0x00UL)
#define RCC_CFGR_ADDR			(RCC_BASE_ADDR + 0x08UL)
#define RCC_AHB1RSTR_ADDR		(RCC_BASE_ADDR + 0x10UL)
#define RCC_AHB2RSTR_ADDR		(RCC_BASE_ADDR + 0x14UL)
#define RCC_APB1RSTR_ADDR		(RCC_BASE_ADDR + 0x20UL)
#define RCC_APB2RSTR_ADDR		(RCC_BASE_ADDR + 0x24UL)
#define RCC_AHB1ENR_ADDR		(RCC_BASE_ADDR + 0x30UL)
#define RCC_AHB2ENR_ADDR		(RCC_BASE_ADDR + 0x34UL)
#define RCC_APB1ENR_ADDR		(RCC_BASE_ADDR + 0x40UL)
#define RCC_APB2ENR_ADDR		(RCC_BASE_ADDR + 0x44UL)
*/   //the RCC address is not needed since the structure is created for the same below

//GPIO Base Address
#define GPIOA_BASE_ADDR			AHB1_BASE_ADDR
#define GPIOB_BASE_ADDR			0x40020400UL
#define GPIOC_BASE_ADDR			0x40020800UL
#define GPIOD_BASE_ADDR			0x40020C00UL
#define GPIOE_BASE_ADDR			0x40021000UL
#define GPIOH_BASE_ADDR			0x40021C00UL

// Base address of peripherals on APB2 bus
#define TIM1_BASE_ADDR                   APB2_BASE_ADDR
#define USART1_BASE_ADDR                (APB2_BASE_ADDR + 0x1000UL)
#define USART6_BASE_ADDR                (APB2_BASE_ADDR + 0x1400UL)
#define ADC1_BASE_ADDR                  (APB2_BASE_ADDR + 0x2000UL)
#define SDIO_BASE_ADDR                  (APB2_BASE_ADDR + 0x2C00UL)
#define SPI1_BASE_ADDR                  (APB2_BASE_ADDR + 0x3000UL)             // base address for spi1 and i2s1 are same
#define I2S1_BASE_ADDR                  (APB2_BASE_ADDR + 0x3000UL)             // base address for spi1 and i2s1 are same
#define SPI4_BASE_ADDR                  (APB2_BASE_ADDR + 0x3400UL)             // base address for spi4 and i2s4 are same
#define I2S4_BASE_ADDR                  (APB2_BASE_ADDR + 0x3400UL)             // base address for spi4 and i2s4 are same
#define SYSCFG_BASE_ADDR                (APB2_BASE_ADDR + 0x3800UL)
#define EXTI_BASE_ADDR                  (APB2_BASE_ADDR + 0x3C00UL)
#define TIM9_BASE_ADDR                  (APB2_BASE_ADDR + 0x4000UL)
#define TIM10_BASE_ADDR                 (APB2_BASE_ADDR + 0x4400UL)
#define TIM11_BASE_ADDR                 (APB2_BASE_ADDR + 0x4800UL)
#define SPI5_BASE_ADDR                  (APB2_BASE_ADDR + 0x5000UL)             // base address for spi5 and i2s5 are same
#define I2S5_BASE_ADDR                  (APB2_BASE_ADDR + 0x5000UL)             // base address for spi5 and i2s5 are same


// Base address of peripherals on APB1 bus
#define TIM2_BASE_ADDR					APB1_BASE_ADDR
#define TIM3_BASE_ADDR					(APB1_BASE_ADDR + 0x0400)
#define TIM4_BASE_ADDR					(APB1_BASE_ADDR + 0x0800)
#define TIM5_BASE_ADDR					(APB1_BASE_ADDR + 0x0C00)
#define RTC_BASE_ADDR					(APB1_BASE_ADDR + 0x2800)
#define WWDG_BASE_ADDR					(APB1_BASE_ADDR + 0x2C00)
#define IWDG_BASE_ADDR					(APB1_BASE_ADDR + 0x3000)
#define I2S2ext_BASE_ADDR				(APB1_BASE_ADDR + 0x3400)
#define SPI2_BASE_ADDR					(APB1_BASE_ADDR + 0x3800)             // base address for spi2 and i2s2 are same
#define I2S2_BASE_ADDR					(APB1_BASE_ADDR + 0x3800)             // base address for spi2 and i2s2 are same
#define SPI3_BASE_ADDR					(APB1_BASE_ADDR + 0x3C00)             // base address for spi3 and i2s3 are same
#define I2S3_BASE_ADDR					(APB1_BASE_ADDR + 0x3C00)             // base address for spi3 and i2s3 are same
#define I2S3ext_BASE_ADDR				(APB1_BASE_ADDR + 0x4000)
#define USART2_BASE_ADDR				(APB1_BASE_ADDR + 0x4400)
#define I2C1_BASE_ADDR					(APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR					(APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR					(APB1_BASE_ADDR + 0x5C00)
#define PWR_BASE_ADDR					(APB1_BASE_ADDR + 0x7000)



/****************************************************************************************************
 * 								register definitions
 ****************************************************************************************************/

//register structure definition for GPIO
typedef struct
{
	__vo uint32 MODER;						//GPIO port mode register
	__vo uint32 OTYPER;					//GPIO port output type register
	__vo uint32 OSPEEDR;					//GPIO port output speed register
	__vo uint32 PUPDR;						//GPIO port pull-up/pull-down register
	__vo uint32 IDR;					//GPIO port input data register
	__vo uint32 ODR;						//GPIO port output data register
	__vo uint32 BSRR;					//GPIO port bit set/reset register
	__vo uint32 LCKR;						//GPIO port configuration lock register
	__vo uint32 AFR[2];					//AFR[0] for GPIO alternate function low register and AFR[1] for GPIO alternate function high register
} GPIO_RegDef_t;



// register definition for exti registers
typedef struct
{
  __vo uint32 IMR;                    // offset 0x00, Interrupt mask register
  __vo uint32 EMR;                    // offset 0x04, Event mask register
  __vo uint32 RTSR;                   // offset 0x08, Rising trigger selection Register
  __vo uint32 FTSR;                   // offset 0x0C, falling trigger selection register
  __vo uint32 SEIWR;                  // offset 0x10, software interrupt event regoster
  __vo uint32 PR;                     // offset 0x14, pending register
}EXTI_RegDef_t;



// register definition for RCC registers
typedef struct
{
	__vo uint32 CR;
	__vo uint32 PLLCFGR;
	__vo uint32 CFGR;
	__vo uint32 CIR;
	__vo uint32 AHB1RSTR;
	__vo uint32 AHB2RSTR;
	uint32 RESERVED0[2];
	__vo uint32 APB1RSTR;
	__vo uint32 APB2RSTR;
	uint32 RESERVED1[2];
	__vo uint32 AHB1ENR;
	__vo uint32 AHB2ENR;
	uint32 RESERVED2[2];
	__vo uint32 APB1ENR;
	__vo uint32 APB2ENR;
	uint32 RESERVED3[2];
	__vo uint32 AHB1LPENR;
	__vo uint32 AHB2LPENR;
	uint32 RESERVED4[2];
	__vo uint32 APB1LPENR;
	__vo uint32 APB2LPENR;
	uint32 RESERVED5[2];
	__vo uint32 BDCR;
	__vo uint32 CSR;
	uint32 RESERVED6[2];
	__vo uint32 SSCGR;
	__vo uint32 PLL2SCFGR;
	uint32 RESERVED7;
	__vo uint32 DCKCFGR;
}RCC_RegDef_t;



// register definition structure for SYSCFG
typedef struct
{
  __vo uint32 MEMRMP;                 // offset: 0x00
  __vo uint32 PMC;                    // offset: 0x04
  __vo uint32 EXTICR[4];                 // offset: 0x08-0x14, EXTICR[0] is EXTICR1 similarly EXTICR[3] is EXTICR4
  uint32 RESERVED[2];                  // offset: 0x18-1C
  __vo uint32 CMPCR;                   // offset: 0x20
}SYSCFG_RegDef_t;



// register definition structure for SPI
typedef struct
{
	__vo uint16 SPI_CR1;				//offset 0x00 // spi control register1
	uint16 RESERVED1;
	__vo uint16 SPI_CR2;				//offset 0x04 // spi control register2
	uint16 RESERVED2;
	__vo uint16 SPI_SR;					//offset 0x08 // spi status register
	uint16 RESERVED3;
	__vo uint16 SPI_DR;					//offset 0x0C // spi data register
	uint16 RESERVED4;
	__vo uint16 SPI_CRCPR;				//offset 0x10 // spi crc polynomial
	uint16 RESERVED5;
	__vo uint16 SPI_RXCRCR;				//offset 0x14 // spi rx register
	uint16 RESERVED6;
	__vo uint16 SPI_TXCRCR;				//offset 0x1C // spi tx configuration  register
	uint16 RESERVED7;
	__vo uint16 SPI_CFGR;			//offset 0x1C // spi i2s configuration  register
	uint16 RESERVED8;
	__vo uint16 SPI_PR;				//offset 0x20 // spi i2s prescaler register
	uint16 RESERVED9;
}SPI_RegDef_t;


//register definition structure for usart
typedef struct
{
	__vo uint32 USART_SR;				//offset 0x00 // usart status register
	__vo uint32 USART_DR;				//offset 0x04 // usart data register
	__vo uint32 USART_BRR;				//offset 0x08 // usart baud rate register
	__vo uint32 USART_CR1;				//offset 0x0C // usart control register1
	__vo uint32 USART_CR2;				//offset 0x10 // usart control register2
	__vo uint32 USART_CR3;				//offset 0x14 // usart control register3
	__vo uint32 USART_GTPR;				//offset 0x18 // usart guard time and prescalar register
}USART_RegDef_t;

/**************************************************************************************
 * 							Peripheral definition
 *
 **************************************************************************************/


#define SYSCFG                          ((SYSCFG_RegDef_t *) SYSCFG_BASE_ADDR)
#define EXTI                            ((EXTI_RegDef_t *) EXTI_BASE_ADDR)
#define RCC								((RCC_RegDef_t *) RCC_BASE_ADDR)

// GPIO peripheral definition
#define GPIOA 			((GPIO_RegDef_t *) GPIOA_BASE_ADDR)
#define GPIOB			((GPIO_RegDef_t *) GPIOB_BASE_ADDR)
#define GPIOC			((GPIO_RegDef_t *) GPIOC_BASE_ADDR)
#define GPIOD			((GPIO_RegDef_t *) GPIOD_BASE_ADDR)
#define GPIOE			((GPIO_RegDef_t *) GPIOE_BASE_ADDR)
#define GPIOH			((GPIO_RegDef_t *) GPIOH_BASE_ADDR)


// SPI peripheral definition
#define SPI1						(( SPI_RegDef_t *)(SPI1_BASE_ADDR))
#define SPI2						(( SPI_RegDef_t *)(SPI2_BASE_ADDR))
#define SPI3						(( SPI_RegDef_t *)(SPI3_BASE_ADDR))
#define SPI4						(( SPI_RegDef_t *)(SPI4_BASE_ADDR))
#define SPI5						(( SPI_RegDef_t *)(SPI5_BASE_ADDR))


// USART peripheral definition
#define USART1						((USART_RegDef_t *)(USART1_BASE_ADDR))
#define USART2						((USART_RegDef_t *)(USART2_BASE_ADDR))
#define USART6						((USART_RegDef_t *)(USART6_BASE_ADDR))


/****************************************************************************************************
 * 								peripheral clock macros
 ****************************************************************************************************/

// Clock Enable macros for GPIO peripherals

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7))


// clock enable for APB2 peripherals
#define SYSCFG_PCLK_EN()        ( RCC->APB2ENR |= (1 << 14))


// Clock Disable macros for GPIO peripherals
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 7))


//clock enable macros for spi peripherals
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()			(RCC->APB2ENR |= (1 << 20))


//clock disable macros for spi peripherals
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 20))




//clock enable macros for i2s peripherals
#define I2S1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define I2S2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define I2S3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define I2S4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13))
#define I2S5_PCLK_EN()			(RCC->APB2ENR |= (1 << 20))


//clock enable macros for usart peripherals
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()		(RCC->APB1ENR |= (1 << 5))


//clock disable macros for usart peripherals
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 5))



// macros to reset registers
#define GPIOA_REG_RESET()              do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while (0)
/* #define GPIOA_REG_RESET ()            (RCC->AHB1RSTR |= (1 << 0));\
                                        (RCC->AHB1RSTR $= ~(1 << 0)) */                 //alternate way
#define GPIOB_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); } while (0)
#define GPIOC_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); } while (0)
#define GPIOD_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); } while (0)
#define GPIOE_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); } while (0)
#define GPIOH_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); } while (0)





//IRQ (interrupt Request) Number for STM32f411xx
// complete the rest of the numbers
#define IRQ_NO_EXTI0                    6
#define IRQ_NO_EXTI1                    7
#define IRQ_NO_EXTI2                    8
#define IRQ_NO_EXTI3                    9
#define IRQ_NO_EXTI4                    10
#define IRQ_NO_EXTI9_5                  23
#define IRQ_NO_EXTI15_10                40

#define IRQ_NO_USART1					37
#define IRQ_NO_USART2					38
#define IRQ_NO_USART6					71

#define IRQ_NO_SPI1						35
#define IRQ_NO_SPI2						36
#define IRQ_NO_SPI3						51
#define IRQ_NO_SPI4						84
#define IRQ_NO_SPI5						85



// IRQ possible priority levels
#define NVIC_IRQ_PR0                    0
#define NVIC_IRQ_PR1                    1
#define NVIC_IRQ_PR2                    2
#define NVIC_IRQ_PR3                    3
#define NVIC_IRQ_PR4                    4
#define NVIC_IRQ_PR5                    5
#define NVIC_IRQ_PR6                    6
#define NVIC_IRQ_PR7                    7
#define NVIC_IRQ_PR8                    8
#define NVIC_IRQ_PR9                    9
#define NVIC_IRQ_PR10                   10
#define NVIC_IRQ_PR11                   11
#define NVIC_IRQ_PR12                   12
#define NVIC_IRQ_PR13                   13
#define NVIC_IRQ_PR14                   14
#define NVIC_IRQ_PR15                   15




//macro used to return the PORT number...GPIOA return 0 while GPIOH returns 7
#define GPIO_PORT_NUMBER(x)             ( (x == GPIOA) ? 0 :\
                                        (x == GPIOB) ? 1 :\
                                        (x == GPIOC) ? 2 :\
                                        (x == GPIOD) ? 3 :\
                                        (x == GPIOE) ? 4 :\
                                          (x == GPIOH) ? 7: 0)






/*****************************************************************************************
 *
 *                   bit position definitions for spi periferals
 *
 *******************************************************************************************/
/*
 * @for spi_cr1 register
 */
#define SPI_CR1_CPHA_BIT			0
#define SPI_CR1_CPOL_BIT			1
#define SPI_CR1_MSTR_BIT			2
#define SPI_CR1_BR_BIT				3		//BAUD RATE 3 BITS ARE USED
#define SPI_CR1_SPE_BIT				6		// SPI ENABLE
#define SPI_CR1_LSB_FIRST_BIT		7		// FRAME FORMAT
#define SPI_CR1_SSI_BIT				8		// internal slave select
#define SPI_CR1_SSM_BIT				9		//software slave management
#define SPI_CR1_RXONLY_BIT			10		// receive only
#define SPI_CR1_DFF_BIT				11		// data frame format
#define SPI_CR1_CRC_NEXT_BIT		12		// crc transfer next
#define SPI_CR1_CRC_EN_BIT			13		// hardware crc calculation enable
#define SPI_CR1_BIDI_OE_BIT			14		// output enable in bidirectional mode
#define SPI_CR1_BIDI_MODE_BIT		15		// bidirectional data mode enable


/*
 * @ for spi_cr2
 */
#define SPI_CR2_RXDMAEN_BIT			0				// rx buffer dma enable
#define SPI_CR2_TXDMAEN_BIT			1				// tx buffer dma enable
#define SPI_CR2_SSOE_BIT			2				// ss output enable
#define SPI_CR2_FRF_BIT				4				// frame format
#define SPI_CR2_ERRIE_BIT			5				// error interrupt enable
#define SPI_CR2_RXNEIE_BIT			6				// rx buffer not empty interrupt enable
#define SPI_CR2_TXEIE_BIT			7				// tx buffer empty interrupt enable


/*
 * FOR SPI_SR
 */
#define SPI_SR_RXNE_BIT				0				// receice buffer not empty
#define SPI_SR_TXE_BIT				1				// transmit buffer empty , set by software when tx buffer is empty
#define SPI_SR_CHSIDE_BIT			2				// channel flag
#define SPI_SR_UDR_BIT				3				//underrun flag
#define SPI_SR_CRC_ERR_BIT			4				// crc error flag
#define SPI_SR_MODF_BIT				5				// mode fault
#define SPI_SR_OVR_BIT				6				// overrun flag
#define SPI_SR_BSY_BIT				7				//busy flag
#define SPI_SR_FRE_BIT				8				//frame format error



/*****************************************************************************************
 *
 *                   bit position definitions for some of USART periferals
 *
 *******************************************************************************************/

//FOR USART_CR1
#define	USART_CR1_SBK_BIT					0
#define	USART_CR1_RWU_BIT					1
#define	USART_CR1_RE_BIT					2
#define	USART_CR1_TE_BIT					3
#define	USART_CR1_IDLEIE_BIT				4
#define	USART_CR1_RXNEIE_BIT				5
#define	USART_CR1_TCIE_BIT					6
#define	USART_CR1_TXEIE_BIT					7
#define	USART_CR1_PEIE_BIT					8
#define	USART_CR1_PS_BIT					9
#define	USART_CR1_PCE_BIT					10
#define	USART_CR1_WAKE_BIT					11
#define	USART_CR1_M_BIT						12
#define	USART_CR1_UE_BIT					13
#define	USART_CR1_OVER8_BIT					15

/*
#define SPI_CR1_CPHA_BIT			0
#define SPI_CR1_CPHA_BIT			0
#define SPI_CR1_CPHA_BIT			0
#define SPI_CR1_CPHA_BIT			0
*/

//FOR USART_CR2
#define	USART_CR2_ADD_BIT				0
#define	USART_CR2_LBDL_BIT				5
#define	USART_CR2_LBDIE_BIT				6
#define	USART_CR2_LBCL_BIT				8
#define	USART_CR2_CPHA_BIT				9
#define	USART_CR2_CPOL_BIT				10
#define	USART_CR2_CLKEN_BIT				11
#define	USART_CR2_STOP_BIT				12


//FOR USART_CR3
#define	USART_CR3_EIE_BIT				0
#define	USART_CR3_IREN_BIT				1
#define	USART_CR3_IRLP_BIT				2
#define	USART_CR3_HDSEL_BIT				3
#define	USART_CR3_NACK_BIT				4
#define	USART_CR3_SCEN_BIT				5
#define	USART_CR3_DMAR_BIT				6
#define	USART_CR3_DMAT_BIT				7
#define	USART_CR3_RTSE_BIT				8
#define	USART_CR3_CTSE_BIT				9
#define	USART_CR3_CTSIE_BIT				10
#define	USART_CR3_ONEBIT_BIT			11


//FOR USART_SR
#define	USART_SR_PE_BIT					0
#define	USART_SR_FE_BIT					1
#define	USART_SR_NF_BIT					2
#define	USART_SR_ORE_BIT				3
#define	USART_SR_IDLE_BIT				4
#define	USART_SR_RXNE_BIT				5
#define	USART_SR_TC_BIT					6
#define	USART_SR_TXE_BIT				7
#define	USART_SR_LBD_BIT				8
#define	USART_SR_DTS_BIT				9




/*****************************************************************************************
 *
 *                   				General Macros
 *
 *******************************************************************************************/
#define ENABLE                              1
#define DISABLE                             0
#define GPIO_PIN_ENABLE 					ENABLE
#define GPIO_PIN_DISABLE					DISABLE
#define GPIO_PIN_SET 						GPIO_PIN_ENABLE
#define GPIO_PIN_RESET 						GPIO_PIN_DISABLE
#define SET                        		 	ENABLE
#define RESET	                   		 	DISABLE
#define FLAG_RESET							RESET
#define FLAG_SET							SET
#define NULL 								((void *)0)




#endif /* INC_STM32F411XX_H_ */
