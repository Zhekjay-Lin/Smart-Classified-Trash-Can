/***********************************************************************************************************************************
 ** ���ļ����ơ�  led.c
 ** ����д��Ա��  ħŮ�������Ŷ�
 ** ����    ����  ħŮ������      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** ���ļ����ܡ�  ʵ��LEDָʾ�Ƴ��õĳ�ʼ�����������ܺ���
 **
 ** ����ֲ˵����
 **
 ** �����¼�¼��
 **
***********************************************************************************************************************************/
#include "bsp_led.h"
#include "Delay.h"


void Led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;    // ����һ��GPIO_InitTypeDef���͵Ľṹ��

    // ʹ��LED_RED�������Ŷ˿�ʱ�ӣ�ʹ�ö˿��жϵķ���ʹ��ʱ��, ��ʹ������ֲ������
    if (LED_RED_GPIO == GPIOA)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    if (LED_RED_GPIO == GPIOB)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    if (LED_RED_GPIO == GPIOC)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    if (LED_RED_GPIO == GPIOD)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    if (LED_RED_GPIO == GPIOE)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    if (LED_RED_GPIO == GPIOF)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    if (LED_RED_GPIO == GPIOG)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    // ʹ��LED_BLUE�������Ŷ˿�ʱ�ӣ�ʹ�ö˿��жϵķ���ʹ��ʱ��, ��ʹ������ֲ������
    if (LED_BLUE_GPIO == GPIOA)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    if (LED_BLUE_GPIO == GPIOB)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    if (LED_BLUE_GPIO == GPIOC)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    if (LED_BLUE_GPIO == GPIOD)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    if (LED_BLUE_GPIO == GPIOE)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    if (LED_BLUE_GPIO == GPIOF)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    if (LED_BLUE_GPIO == GPIOG)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    // ����LED_RED���Ź���ģʽ
    GPIO_InitStructure.GPIO_Pin   = LED_RED_PIN;     // ѡ��Ҫ���Ƶ�GPIO����
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   // ����ģʽ�����ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   // ������ͣ��������
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;    // ��������  ����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // �������ʣ�2MHz
    GPIO_Init(LED_RED_GPIO, &GPIO_InitStructure);    // ���ÿ⺯����ʹ����������ó�ʼ��GPIO

    // ����LED_RED���Ź���ģʽ
    GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN;      // ѡ��Ҫ���Ƶ�GPIO����
    GPIO_Init(LED_BLUE_GPIO, &GPIO_InitStructure);   // ʹ�øղ�LED_RED��ͬ������������������ص�GPIO����

    // ������ɫLED����ɫLED
    //GPIO_ResetBits(LED_RED_GPIO, LED_RED_PIN);       // ����LED_RED�� �͵�ƽ����
    //GPIO_ResetBits(LED_BLUE_GPIO, LED_BLUE_PIN);     // ����LED_BLUE���͵�ƽ���� 

    // �ر�LED
    LED_BLUE_OFF;
    LED_RED_ON;
            
}

void BLUE_blink(void)
{
     LED_BLUE_TOGGLE;
     delay_ms(1000);
}



