#include "Sensor.h"
#include "stm32f4xx.h"                  // Device header
#include "bsp_led.h"

void Sensor_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SensorPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(SensorGPIO, &GPIO_InitStructure);
}


uint8_t SensorGPIO_Scan(void)
{
    if (GPIO_ReadInputDataBit(SensorGPIO,SensorPin) == 1 ) 
    {
        return 1;
    }
    return 0;
}


