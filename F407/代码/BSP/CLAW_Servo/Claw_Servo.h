#ifndef __CLAW_SERVO_H
#define __CLAW_SERVO_H

#include "stdint.h"

void claw_servo_gpio_Init(void);
void CS_PWM_SetCompare(uint16_t Compare);
void claw_servo_Init(void);
void claw_servo_setAngle(float angle);
void claw_servo_home(void);
#endif

