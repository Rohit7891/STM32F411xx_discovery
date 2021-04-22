#include "stm32f411xx_drivers_gpio.h"

#define OUTPUT_LED      GPIO_PIN_12
#define GREEN           GPIO_PIN_12
#define RED             GPIO_PIN_14
#define ORANGE          GPIO_PIN_13
#define BLUE            GPIO_PIN_15
#define USER_BUTTON     GPIO_PIN_0

void delay(uint32_t time);
int main()
{
uint8_t val = 1;
uint8_t i=0;
  GPIO_Handle_t led, button;
  GPIO_PCLK_Control(GPIOD, ENABLE);
  GPIO_PCLK_Control(GPIOA, ENABLE);
  uint8_t led_pin[] = { GREEN, BLUE, RED, ORANGE};
  

    led.pGPIOx = GPIOD;
    led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  for(i=0;i<=3;i++)
  {
    led.GPIO_PinConfig.GPIO_PinNumber = led_pin[i];
    GPIO_Init(&led);
  }
  
  button.pGPIOx = GPIOA;
  button.GPIO_PinConfig.GPIO_PinNumber = USER_BUTTON;
  button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
  GPIO_Init(&button);
  
  while(1)
  {
    if(GPIO_ReadFromInputPin(GPIOA, USER_BUTTON) == SET)
    {
        for(i=0;i<=3;i++)
        {
          GPIO_WriteToOutputPin(GPIOD, led_pin[i], val);
          delay(100);
        }
        val ^= GPIO_PIN_SET;
        delay(200);
    }
  }

  return 0;
}


void delay(uint32_t time)
{
  uint32_t i,j;
  for(j=0;j<=time;j++)
  for(i=0;i<=3000;i++);
}
