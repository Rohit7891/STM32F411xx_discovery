#include "stm32f411xx_drivers_gpio.h"

#define OUTPUT_LED      GPIO_PIN_12
#define GREEN           GPIO_PIN_12
#define RED             GPIO_PIN_14
#define ORANGE          GPIO_PIN_13
#define BLUE            GPIO_PIN_15

void delay(void);
int main()
{
uint8_t val = 1;
uint8_t i=0;
  GPIO_Handle_t led;
  GPIO_PCLK_Control(GPIOD, ENABLE);
  uint8_t led_pin[] = { GREEN, RED, ORANGE, BLUE};
  
  for(i=0;i<=3;i++)
  {
    led.pGPIOx = GPIOD;
    led.GPIO_PinConfig.GPIO_PinNumber = led_pin[i];
    led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&led);
  }
  
  while(1)
  {
    for(i=0;i<=3;i++)
    {
      GPIO_WriteToOutputPin(GPIOD, led_pin[i], val);
      delay();
    }
    val ^= GPIO_PIN_SET;
    delay();
  }

  return 0;
}


void delay(void)
{
  uint32_t i;
  for(i=0;i<=100000;i++);
}