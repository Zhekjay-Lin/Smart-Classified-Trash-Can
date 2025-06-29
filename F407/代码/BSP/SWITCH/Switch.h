#ifndef __SWITCH_H
#define __SWITCH_H

#include "stm32f4xx.h"                  // Device header

#define       switchPin      GPIO_Pin_3

void Switch_Init(void);
uint8_t SwitchStatus(void);

#endif
