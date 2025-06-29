#include "Switch.h" 

void Switch_Init()
{
  RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOE, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);     //开启定时器TIM6
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 999;
  TIM_InitStructure.TIM_Prescaler = 83;         //0.001s计时    (9999+1)*(83+1)/84M = 0.01s
  //TIM_ARRPreloadConfig(TIM6, ENABLE);           //开启自动重装载
  TIM_TimeBaseInit(TIM6, &TIM_InitStructure);
  
  TIM_ClearFlag(TIM6,TIM_FLAG_Update);
  
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占式优先级为0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //设置子优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
}

uint8_t SwitchStatus(void)
{
  if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == 0 | GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 0 | GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0 | GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == 0)            
  {
    return 1;                                                       // 遮挡
  }
  else
    return 0;
}



