#ifndef __BSP__LED_H
#define __BSP__LED_H
/***********************************************************************************************************************************
 ** 【文件名称】  led.h
 ** 【编写人员】  魔女开发板团队
 ** 【淘    宝】  魔女开发板      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【文件功能】  简化常用的系统函数、初始化函数
 **                
 ** 【硬件重点】  1- 注意LED是代电平还是高电平点亮，可查原理图
 **               2- 
 **
 ** 【代码重点】  1- 引脚工作模式：推挽输出
 **
 ** 【移植说明】  
 **
 ** 【更新记录】  
 **
***********************************************************************************************************************************/  
#include "stm32f4xx.h"
#include <stdio.h>





//移植参数区 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define  LED_RED_GPIO      GPIOC
#define  LED_RED_PIN       GPIO_Pin_5   

#define  LED_BLUE_GPIO     GPIOB
#define  LED_BLUE_PIN      GPIO_Pin_2
//END 移植 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



// 按色简化功能, 移植时不用修改
// red                      
#define LED_RED_ON         (LED_RED_GPIO->BSRR = LED_RED_PIN << 16)    // 点亮，置低电平
#define LED_RED_OFF        (LED_RED_GPIO->BSRR = LED_RED_PIN)          // 熄灭，置高电平
#define LED_RED_TOGGLE     (LED_RED_GPIO->ODR  ^= LED_RED_PIN)         // 反转，电平取反
// blue
#define LED_BLUE_ON        (LED_BLUE_GPIO->BSRR  = LED_BLUE_PIN << 16) // 点亮，置低电平
#define LED_BLUE_OFF       (LED_BLUE_GPIO->BSRR  = LED_BLUE_PIN)       // 熄灭，置高电平
#define LED_BLUE_TOGGLE    (LED_BLUE_GPIO->ODR   ^= LED_BLUE_PIN)      // 反转，电平取反



/*****************************************************************************
 ** 声明全局函数
****************************************************************************/
void Led_Init(void);
void BLUE_blink(void);


#endif

