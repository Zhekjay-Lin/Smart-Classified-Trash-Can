/******************************************************************************
 * 功  能： pwm舵机运动初始化与初步控制
 * 备  注： 角度>=128 
 ******************************************************************************/

#include "stm32f4xx.h"                  // Device header
#include "Claw_Servo.h"
#include "Delay.h"

void claw_servo_gpio_Init(void)
{
    //TIM12 PB14 CH1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOE,&GPIO_InitStructure);
  
    //周期（200*8400）/84000000 = 0.01s = 20ms
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_Period = 20000-1;
    TIM_InitStructure.TIM_Prescaler = 84-1;
    TIM_TimeBaseInit(TIM12, &TIM_InitStructure); 
  
    GPIO_PinAFConfig(GPIOB,  GPIO_PinSource14,  GPIO_AF_TIM12);
  
    TIM_OCInitTypeDef TimOCInitStructure;
    TIM_OCStructInit(&TimOCInitStructure);
    TimOCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
    TimOCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TimOCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High; 
    TimOCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM12, &TimOCInitStructure);
    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable); 
    TIM_Cmd(TIM12, ENABLE); 
    //TIM_CtrlPWMOutputs(TIM12, ENABLE);
}


void CS_PWM_SetCompare(uint16_t Compare)
{
    TIM_SetCompare1(TIM12, Compare);
}

void claw_servo_Init(void)
{
    claw_servo_gpio_Init();
    claw_servo_setAngle(0);
}

//angle 取值：0至90 
void claw_servo_setAngle(float angle)
{
    angle = 130 - angle;    
    float compare;
    compare = (angle/180*2000 + 500);
    CS_PWM_SetCompare(compare);
    delay_ms(100);    
}

void claw_servo_home(void)
{
    claw_servo_setAngle(0);
}

