/******************************************************************************
 * 功  能： 推杆电机运动初始化与初步控制
 * 备  注： 无 
 ******************************************************************************/

#include "stm32f4xx.h"                  // Device header
#include "Linear_Actuator.h"
#include "Delay.h"

//接开发板引脚与继电器IN端： PE12-IN3,   PE13-IN4    PE14-IN1,   PE15-IN2
void LinearActuator_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
  
    LinearAcuator_Stop();
}

//IN1,IN3接通，伸长
void LinearAcuator_Strech(void)
{
    GPIO_SetBits(GPIOE, GPIO_Pin_12);
    GPIO_ResetBits(GPIOE, GPIO_Pin_13);
    GPIO_SetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    delay_ms(2600);
}

//IN2,IN4接通，缩短
void LinearAcuator_Shorten(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_12);
    GPIO_SetBits(GPIOE, GPIO_Pin_13);
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
    GPIO_SetBits(GPIOE, GPIO_Pin_15);
    delay_ms(2600);
}

//全断开，停止
void LinearAcuator_Stop(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_12);
    GPIO_ResetBits(GPIOE, GPIO_Pin_13);
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
}

//控制函数统一封装  0-停止；1-伸长；2-缩短
void LinearAcuator(int State)
{
    switch(State)
    {
      case 0: LinearAcuator_Stop();break;
      case 1: LinearAcuator_Strech();break;
      case 2: LinearAcuator_Shorten();break;
    }
}

