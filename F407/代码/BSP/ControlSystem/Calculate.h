#ifndef __CALCULATE_H
#define __CALCULATE_H

#include "stm32f4xx.h"
#include "string.h"
#include <stdlib.h>

int String2int(uint8_t *inputdata, uint16_t inputdataNum, uint8_t task);
int32_t Calulate_Step(int32_t tar_location, int32_t cur_location);
int Calulate_WristAngle(int wristangle);

#endif

