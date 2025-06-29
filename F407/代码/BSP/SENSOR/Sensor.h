#ifndef __SENSOR_H
#define __SENSOR_H

#include "stdint.h"

#define SensorPin      GPIO_Pin_8
#define SensorGPIO     GPIOD

void Sensor_Init(void);
uint8_t SensorGPIO_Scan(void);

#endif

