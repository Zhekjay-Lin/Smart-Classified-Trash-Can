#ifndef __BUS_SERVO_H
#define __BUS_SERVO_H

#include "stdint.h"

void Gimbal_Servo(uint8_t type);
void Door_Servo(uint8_t doorStatus);
void Wrist_ServoMove(int wristangle);

#endif

