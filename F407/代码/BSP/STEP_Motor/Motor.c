#include "stm32f4xx.h"                  // Device header
#include "Motor.h"
#include "Motor_Control.h"
#include "Delay.h"

void Motor_GPIO_Init(void)
{
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 				
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  
  
	/* 步进电机驱动器：脉冲输出PA6-x轴，PA2-y轴, PA3-z轴 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									// 复用输出
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	/* 步进电机驱动器：方向控制 PB4-x轴，PB5-y轴, PB9-z轴*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  /* 步进电机驱动器：使能 PD7*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
  
}

void Motor_TIM_Init()
{
  /*TIM5定时器初始化*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 65535;
  TIM_InitStructure.TIM_Prescaler = TIM_PRESCALER;
  TIM_TimeBaseInit(TIM4, &TIM_InitStructure);
  
  /*输出引脚复用*/
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource6,  GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource7,  GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource8,  GPIO_AF_TIM4);
  
  /*输出通道初始配置*/
  TIM_OCInitTypeDef TIM_OCInitStructure;	
	TIM_OCStructInit(&TIM_OCInitStructure);	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;  						//翻转模式
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;															//CCR 比较值
  
  /*输出通道初始化：PA6-TIM3-CH1,PA7-TIM3-CH2，PB0-TIM3-CH3*/
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Disable);    						//关闭预装载
	TIM_CCxCmd(TIM4,TIM_Channel_1,TIM_CCx_Disable);
  
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Disable);    						
	TIM_CCxCmd(TIM4,TIM_Channel_2,TIM_CCx_Disable);
  
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Disable);    						
	TIM_CCxCmd(TIM4,TIM_Channel_3,TIM_CCx_Disable);
  
  /*TIM3中断初始化*/
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢占式优先级为0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //设置子优先级为1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /*TIM3各通道启动*/
  TIM_ClearFlag(TIM4,TIM_FLAG_CC1);
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
  
  TIM_ClearFlag(TIM4,TIM_FLAG_CC2);
  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
  
  TIM_ClearFlag(TIM4,TIM_FLAG_CC3);
  TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
 
}

void Motor_Init(void)
{
  Motor_TIM_Init();
  delay_ms(1);
  Motor_GPIO_Init();
  delay_ms(1);
  Motor_DIS();         //初始化关闭电机使能引脚减小系统总电流
}

/*使能引脚控制函数*/
void Motor_EN(void)                   
{
  GPIO_ResetBits(GPIOD, GPIO_Pin_7);
}

void Motor_DIS(void)
{
  GPIO_SetBits(GPIOD, GPIO_Pin_7);
}


