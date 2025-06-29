#ifndef _BSP_KEY_H
#define _BSP_KEY_H
#include "stm32f4xx.h"
#include "bsp_led.h"
#include "bsp_USART.h"


/*****************************************************************************
 ** 移植配置区
****************************************************************************/
// KEY_1_WKUP, 闲时下拉，按下时被置高电平
#define KEY_1_CLK                      RCC_AHB1Periph_GPIOA  // 端口时钟
#define KEY_1_GPIO                     GPIOA                 // 引脚所用端口
#define KEY_1_PIN                      GPIO_Pin_0            // 引脚编号
#define KEY_1_PUPD                     GPIO_PuPd_DOWN        // 闲时内部上下拉状态
#define KEY_1_EXTI_PORT                EXTI_PortSourceGPIOA  // 引脚的中断端口  
#define KEY_1_EXTI_PIN                 EXTI_PinSource0       // 引脚的中断编号
#define KEY_1_EXTI_LINE                EXTI_Line0            // 外部中断线编号 
#define KEY_1_EXTI_TRIGGER             EXTI_Trigger_Rising   // 触发方式; 上升沿触发:EXTI_Trigger_Rising， 下降沿触发:EXTI_Trigger_Falling
#define KEY_1_INTERRUPT_NUMBER         EXTI0_IRQn            // 中断编号，即在中断向量表中的编号
#define KEY_1_IRQHANDLER               EXTI0_IRQHandler      // 中断服务函数; 中断服务函数的名称，必须与启动文件中所声明的一样，否则中断时会因为找不到函数入口而卡死;
// KEY_2, 闲时上拉，按下时被置低电平 
#define KEY_2_CLK                      RCC_AHB1Periph_GPIOA  // 端口时钟
#define KEY_2_GPIO                     GPIOA                 // 引脚所用端口
#define KEY_2_PIN                      GPIO_Pin_1            // 引脚编号
#define KEY_2_PUPD                     GPIO_PuPd_UP          // 闲时内部上下拉状态
#define KEY_2_EXTI_PORT                EXTI_PortSourceGPIOA  // 引脚的中断端口  
#define KEY_2_EXTI_PIN                 EXTI_PinSource1       // 引脚的中断编号
#define KEY_2_EXTI_LINE                EXTI_Line1            // 外部中断线编号 
#define KEY_2_EXTI_TRIGGER             EXTI_Trigger_Falling  // 触发方式; 上升沿触发:EXTI_Trigger_Rising， 下降沿触发:EXTI_Trigger_Falling
#define KEY_2_INTERRUPT_NUMBER         EXTI1_IRQn            // 中断编号，即在中断向量表中的编号
#define KEY_2_IRQHANDLER               EXTI1_IRQHandler      // 中断服务函数; 中断服务函数的名称，必须与启动文件中所声明的一样，否则中断时会因为找不到函数入口而卡死;
// KEY_2, 闲时上拉，按下时被置低电平
#define KEY_3_CLK                      RCC_AHB1Periph_GPIOA  // 端口时钟
#define KEY_3_GPIO                     GPIOA                 // 引脚所用端口
#define KEY_3_PIN                      GPIO_Pin_4            // 引脚编号
#define KEY_3_PUPD                     GPIO_PuPd_UP          // 闲时内部上下拉状态
#define KEY_3_EXTI_PORT                EXTI_PortSourceGPIOA  // 引脚的中断端口  
#define KEY_3_EXTI_PIN                 EXTI_PinSource4       // 引脚的中断编号
#define KEY_3_EXTI_LINE                EXTI_Line4            // 外部中断线编号 
#define KEY_3_EXTI_TRIGGER             EXTI_Trigger_Falling  // 触发方式; 上升沿触发:EXTI_Trigger_Rising， 下降沿触发:EXTI_Trigger_Falling
#define KEY_3_INTERRUPT_NUMBER         EXTI4_IRQn            // 中断编号，即在中断向量表中的编号
#define KEY_3_IRQHANDLER               EXTI4_IRQHandler      // 中断服务函数; 中断服务函数的名称，必须与启动文件中所声明的一样，否则中断时会因为找不到函数入口而卡死;
   


/*****************************************************************************
 ** 声明全局函数
****************************************************************************/
void    Key_Init(void);  // 使用h文件中的参数，初始化引脚
uint8_t Key_Scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t targetStatus);  // 引脚端口、引脚编号、期待电平

#endif

