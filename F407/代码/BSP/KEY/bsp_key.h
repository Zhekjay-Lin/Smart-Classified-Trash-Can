#ifndef _BSP_KEY_H
#define _BSP_KEY_H
#include "stm32f4xx.h"
#include "bsp_led.h"
#include "bsp_USART.h"


/*****************************************************************************
 ** ��ֲ������
****************************************************************************/
// KEY_1_WKUP, ��ʱ����������ʱ���øߵ�ƽ
#define KEY_1_CLK                      RCC_AHB1Periph_GPIOA  // �˿�ʱ��
#define KEY_1_GPIO                     GPIOA                 // �������ö˿�
#define KEY_1_PIN                      GPIO_Pin_0            // ���ű��
#define KEY_1_PUPD                     GPIO_PuPd_DOWN        // ��ʱ�ڲ�������״̬
#define KEY_1_EXTI_PORT                EXTI_PortSourceGPIOA  // ���ŵ��ж϶˿�  
#define KEY_1_EXTI_PIN                 EXTI_PinSource0       // ���ŵ��жϱ��
#define KEY_1_EXTI_LINE                EXTI_Line0            // �ⲿ�ж��߱�� 
#define KEY_1_EXTI_TRIGGER             EXTI_Trigger_Rising   // ������ʽ; �����ش���:EXTI_Trigger_Rising�� �½��ش���:EXTI_Trigger_Falling
#define KEY_1_INTERRUPT_NUMBER         EXTI0_IRQn            // �жϱ�ţ������ж��������еı��
#define KEY_1_IRQHANDLER               EXTI0_IRQHandler      // �жϷ�����; �жϷ����������ƣ������������ļ�����������һ���������ж�ʱ����Ϊ�Ҳ���������ڶ�����;
// KEY_2, ��ʱ����������ʱ���õ͵�ƽ 
#define KEY_2_CLK                      RCC_AHB1Periph_GPIOA  // �˿�ʱ��
#define KEY_2_GPIO                     GPIOA                 // �������ö˿�
#define KEY_2_PIN                      GPIO_Pin_1            // ���ű��
#define KEY_2_PUPD                     GPIO_PuPd_UP          // ��ʱ�ڲ�������״̬
#define KEY_2_EXTI_PORT                EXTI_PortSourceGPIOA  // ���ŵ��ж϶˿�  
#define KEY_2_EXTI_PIN                 EXTI_PinSource1       // ���ŵ��жϱ��
#define KEY_2_EXTI_LINE                EXTI_Line1            // �ⲿ�ж��߱�� 
#define KEY_2_EXTI_TRIGGER             EXTI_Trigger_Falling  // ������ʽ; �����ش���:EXTI_Trigger_Rising�� �½��ش���:EXTI_Trigger_Falling
#define KEY_2_INTERRUPT_NUMBER         EXTI1_IRQn            // �жϱ�ţ������ж��������еı��
#define KEY_2_IRQHANDLER               EXTI1_IRQHandler      // �жϷ�����; �жϷ����������ƣ������������ļ�����������һ���������ж�ʱ����Ϊ�Ҳ���������ڶ�����;
// KEY_2, ��ʱ����������ʱ���õ͵�ƽ
#define KEY_3_CLK                      RCC_AHB1Periph_GPIOA  // �˿�ʱ��
#define KEY_3_GPIO                     GPIOA                 // �������ö˿�
#define KEY_3_PIN                      GPIO_Pin_4            // ���ű��
#define KEY_3_PUPD                     GPIO_PuPd_UP          // ��ʱ�ڲ�������״̬
#define KEY_3_EXTI_PORT                EXTI_PortSourceGPIOA  // ���ŵ��ж϶˿�  
#define KEY_3_EXTI_PIN                 EXTI_PinSource4       // ���ŵ��жϱ��
#define KEY_3_EXTI_LINE                EXTI_Line4            // �ⲿ�ж��߱�� 
#define KEY_3_EXTI_TRIGGER             EXTI_Trigger_Falling  // ������ʽ; �����ش���:EXTI_Trigger_Rising�� �½��ش���:EXTI_Trigger_Falling
#define KEY_3_INTERRUPT_NUMBER         EXTI4_IRQn            // �жϱ�ţ������ж��������еı��
#define KEY_3_IRQHANDLER               EXTI4_IRQHandler      // �жϷ�����; �жϷ����������ƣ������������ļ�����������һ���������ж�ʱ����Ϊ�Ҳ���������ڶ�����;
   


/*****************************************************************************
 ** ����ȫ�ֺ���
****************************************************************************/
void    Key_Init(void);  // ʹ��h�ļ��еĲ�������ʼ������
uint8_t Key_Scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t targetStatus);  // ���Ŷ˿ڡ����ű�š��ڴ���ƽ

#endif

