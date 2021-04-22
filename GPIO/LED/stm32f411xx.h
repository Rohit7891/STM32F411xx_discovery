/*
 * stm32f411xx.h
 *
 *  Created on: Jan 17, 2021
 *      Author: rohit
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_


#define uint8_t         unsigned char
#define uint16_t        unsigned short int
#define uint32_t        unsigned int
#define __vo 		volatile
#define int8_t          signed char
#define int16_t         signed short int
#define int32_t         signed int


#define FALSH_BASE_ADDR			0x08000000UL
#define SRAM1_BASE_ADDR			0x20000000UL
#define ROM				FALSH_BASE_ADDR


// RCC registers





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


//register structure definition for GPIO

typedef struct
{
	__vo uint32_t MODER;						//GPIO port mode register
	__vo uint32_t OTYPER;					//GPIO port output type register
	__vo uint32_t OSPEEDR;					//GPIO port output speed register
	__vo uint32_t PUPDR;						//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;					//GPIO port input data register
	__vo uint32_t ODR;						//GPIO port output data register
	__vo uint32_t BSRR;					//GPIO port bit set/reset register
	__vo uint32_t LCKR;						//GPIO port configuration lock register
	__vo uint32_t AFR[2];					//AFR[0] for GPIO alternate function low register and AFR[1] for GPIO alternate function high register
} GPIO_RegDef_t;




typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t RESERVED0[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t RESERVED4[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLL2SCFGR;
	uint32_t RESERVED7;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

#define RCC				((RCC_RegDef_t *) RCC_BASE_ADDR)




#define GPIOA 			((GPIO_RegDef_t *) GPIOA_BASE_ADDR)
#define GPIOB			((GPIO_RegDef_t *) GPIOB_BASE_ADDR)
#define GPIOC			((GPIO_RegDef_t *) GPIOC_BASE_ADDR)
#define GPIOD			((GPIO_RegDef_t *) GPIOD_BASE_ADDR)
#define GPIOE			((GPIO_RegDef_t *) GPIOE_BASE_ADDR)
#define GPIOH			((GPIO_RegDef_t *) GPIOH_BASE_ADDR)






// Clock Enable macros for GPIOs peripherals

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7))



// Clock Disable macros for GPIOs peripherals

#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 7))




// macros to reset registers
#define GPIOA_REG_RESET()              do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while (0)
/* #define GPIOA_REG_RESET ()            (RCC->AHB1RSTR |= (1 << 0));\
                                        (RCC->AHB1RSTR $= ~(1 << 0)) */                 //alternate way
#define GPIOB_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); } while (0)
#define GPIOC_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); } while (0)
#define GPIOD_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); } while (0)
#define GPIOE_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); } while (0)
#define GPIOH_REG_RESET()               do{ RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); } while (0)
            
             
//Generc Macros
#define ENABLE                                  1
#define DISABLE                                 0
#define GPIO_PIN_ENABLE 			ENABLE
#define GPIO_PIN_DISABLE			DISABLE
#define GPIO_PIN_SET 				GPIO_PIN_ENABLE
#define GPIO_PIN_RESET 				GPIO_PIN_DISABLE





#endif /* INC_STM32F411XX_H_ */
