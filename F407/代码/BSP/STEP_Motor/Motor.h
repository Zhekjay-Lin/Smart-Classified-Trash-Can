#ifndef __MOTOR_H
#define __MOTOR_H

#include "Motor_Control.h"              //main函数无需再引用“Motor_Control.h”

void Motor_GPIO_Init(void);
void Motor_TIM_Init(void);
void Motor_Init(void);

void Motor_EN(void);
void Motor_DIS(void);

#endif

