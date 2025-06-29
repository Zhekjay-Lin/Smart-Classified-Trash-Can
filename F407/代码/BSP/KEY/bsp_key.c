#include "bsp_key.h"



// �������غ���
static void key_1_Init(void);                                     // ����key_1��ʼ������; static���޸ĺ�������Ч��Χ���Ӷ�������λ����Դ�ļ��ڿɷ���;
static void key_2_Init(void);                                     // ����key_2��ʼ������; static���޸ĺ�������Ч��Χ���Ӷ�������λ����Դ�ļ��ڿɷ���;
static void key_3_Init(void);                                     // ����key_3��ʼ������; static���޸ĺ�������Ч��Χ���Ӷ�������λ����Դ�ļ��ڿɷ���;






// ����_1�ĳ�ʼ������
static void key_1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;                          // ����GPIO�ṹ�壬�����������Ź���ģʽ
    EXTI_InitTypeDef EXTI_InitStructure;                          // ����EXTI�ṹ�壬���������ⲿ�ж��ߣ������ŵ��жϷ�ʽ
    NVIC_InitTypeDef NVIC_InitStructure;                          // ����NVIC�ṹ�壬���������ж����ȼ�

    // ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(KEY_1_CLK, ENABLE);                    // ʹ��KEY_1�������Ŷ˿�ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);        // ʹ��ϵͳ������SYSCFG��ʱ�ӣ�ʹ��GPIO�ⲿ�жϱ���ʹ��SYSCFGʱ��

    // �������Ź���ģʽ: PA0�� ���롢��ʱ�����������øߵ�ƽ
    GPIO_InitStructure.GPIO_Pin   = KEY_1_PIN;                    // ѡ��Ҫ���Ƶ����ű��; �˴�ʹ���˺궨�壬�Է�����ֲ�޸�
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;                 // ����ģʽ������ģʽ
    GPIO_InitStructure.GPIO_PuPd  = KEY_1_PUPD;                   // ������״̬
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;              // �������ʣ�2MHz
    GPIO_Init(KEY_1_GPIO, &GPIO_InitStructure);                   // ��ʼ��, ��������Ĳ���д��Ĵ���

    // �������������ж���
    SYSCFG_EXTILineConfig(KEY_1_EXTI_PORT, KEY_1_EXTI_PIN);       // �������������ж���

    // ����EXTI�ж���
    EXTI_InitStructure.EXTI_Line = KEY_1_EXTI_LINE;               // �ж���
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;           // �ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = KEY_1_EXTI_TRIGGER;         // ������ʽ
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     // ʹ��
    EXTI_Init(&EXTI_InitStructure);                               // ��ʼ��, ��������Ĳ���д��Ĵ���

    // ���� NVIC, ���ж����ȼ�
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);            // ����NVICΪ���ȼ���2; ��������������ֻ������һ�Σ�ȫ����Ч; ���������ã�����ʱ�����һ������Ϊ׼; �����������÷���main������һ��;
    NVIC_InitStructure.NVIC_IRQChannel = KEY_1_INTERRUPT_NUMBER;  // �жϱ�ţ���ʾ�������жϵı�ţ����������ļ��У���оƬ�ڲ���ƶ�Ԥ���趨�õı�ŵġ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // ʹ��
    NVIC_Init(&NVIC_InitStructure);                               // ��ʼ��, ������Ĳ���д��Ĵ���
}



// KEY_1�������ŵ��жϺ���; ����ʹ���˺궨�����ƣ��Է�����ֲ
void KEY_1_IRQHANDLER(void)
{
    LED_RED_TOGGLE ;                                              // ��ת��ɫLED
    printf("�� 1 ������������, ���Ʒ�ת\r");                      // ��Ҫ��ʾ��printf�ǲ������뺯�����жϷ�������ʹ�ã����ܻ��������Ԥ��Ĵ�������ʹ��printf��ֻ�ô������ʹ�ã���      // ħŮ������İ���ʹ�õ��ݽ���Ӳ������,������ʹ�������ʱ����
    EXTI_ClearITPendingBit(EXTI_Line0);                           // ����жϱ�־λ;
}



// ����_2�ĳ�ʼ������
static void key_2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;                          // ����GPIO�ṹ�壬�����������Ź���ģʽ
    EXTI_InitTypeDef EXTI_InitStructure;                          // ����EXTI�ṹ�壬���������ⲿ�ж��ߣ������ŵ��жϷ�ʽ
    NVIC_InitTypeDef NVIC_InitStructure;                          // ����NVIC�ṹ�壬���������ж����ȼ�

    // ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(KEY_2_CLK, ENABLE);                    // ʹ��KEY_1�������Ŷ˿�ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);        // ʹ��ϵͳ������SYSCFG��ʱ�ӣ�ʹ��GPIO�ⲿ�жϱ���ʹ��SYSCFGʱ��

    // �������Ź���ģʽ: PA0�� ���롢��ʱ�����������øߵ�ƽ
    GPIO_InitStructure.GPIO_Pin   = KEY_2_PIN;                    // ѡ��Ҫ���Ƶ����ű��; �˴�ʹ���˺궨�壬�Է�����ֲ�޸�
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;                 // ����ģʽ������ģʽ
    GPIO_InitStructure.GPIO_PuPd  = KEY_2_PUPD;                   // ������״̬
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;              // �������ʣ�2MHz
    GPIO_Init(KEY_2_GPIO, &GPIO_InitStructure);                   // ��ʼ��, ��������Ĳ���д��Ĵ���

    // �������������ж���
    SYSCFG_EXTILineConfig(KEY_2_EXTI_PORT, KEY_2_EXTI_PIN);       // �������������ж���

    // ����EXTI�ж���
    EXTI_InitStructure.EXTI_Line = KEY_2_EXTI_LINE;               // �ж���
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;           // �ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = KEY_2_EXTI_TRIGGER;         // ������ʽ
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     // ʹ��
    EXTI_Init(&EXTI_InitStructure);                               // ��ʼ��, ��������Ĳ���д��Ĵ���

    // ���� NVIC, ���ж����ȼ�
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);            // ����NVICΪ���ȼ���2; ��������������ֻ������һ�Σ�ȫ����Ч; ���������ã�����ʱ�����һ������Ϊ׼; �����������÷���main������һ��;
    NVIC_InitStructure.NVIC_IRQChannel = KEY_2_INTERRUPT_NUMBER;  // �жϱ�ţ���ʾ�������жϵı�ţ����������ļ��У���оƬ�ڲ���ƶ�Ԥ���趨�õı�ŵġ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // ʹ��
    NVIC_Init(&NVIC_InitStructure);                               // ��ʼ��, ������Ĳ���д��Ĵ���
}



// ����_2�������ŵ��жϺ���; ����ʹ���˺궨�����ƣ��Է�����ֲ
void KEY_2_IRQHANDLER(void)
{                                                                 
    LED_RED_TOGGLE;                                               // ��ת��ɫLED
    printf("�� 2 ������������, ���Ʒ�ת\r");                      // ��Ҫ��ʾ��printf�ǲ������뺯�����жϷ�������ʹ�ã����ܻ��������Ԥ��Ĵ�������ʹ��printf��ֻ�ô������ʹ�ã���      // ħŮ������İ���ʹ�õ��ݽ���Ӳ������,������ʹ�������ʱ����
    EXTI_ClearITPendingBit(KEY_2_EXTI_LINE);                      // ����жϱ�־λ;
}



// ����_3�ĳ�ʼ������
static void key_3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;                          // ����GPIO�ṹ�壬�����������Ź���ģʽ
    EXTI_InitTypeDef EXTI_InitStructure;                          // ����EXTI�ṹ�壬���������ⲿ�ж��ߣ������ŵ��жϷ�ʽ
    NVIC_InitTypeDef NVIC_InitStructure;                          // ����NVIC�ṹ�壬���������ж����ȼ�

    // ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(KEY_3_CLK, ENABLE);                    // ʹ��KEY_1�������Ŷ˿�ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);        // ʹ��ϵͳ������SYSCFG��ʱ�ӣ�ʹ��GPIO�ⲿ�жϱ���ʹ��SYSCFGʱ��

    // �������Ź���ģʽ: PA0�� ���롢��ʱ�����������øߵ�ƽ
    GPIO_InitStructure.GPIO_Pin   = KEY_3_PIN;                    // ѡ��Ҫ���Ƶ����ű��; �˴�ʹ���˺궨�壬�Է�����ֲ�޸�
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;                 // ����ģʽ������ģʽ
    GPIO_InitStructure.GPIO_PuPd  = KEY_3_PUPD;                   // ������״̬
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;              // �������ʣ�2MHz
    GPIO_Init(KEY_3_GPIO, &GPIO_InitStructure);                   // ��ʼ��, ��������Ĳ���д��Ĵ���

    // �������������ж���
    SYSCFG_EXTILineConfig(KEY_3_EXTI_PORT, KEY_3_EXTI_PIN);       // �������������ж���

    // ����EXTI�ж���
    EXTI_InitStructure.EXTI_Line = KEY_3_EXTI_LINE;               // �ж���
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;           // �ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = KEY_3_EXTI_TRIGGER;         // ������ʽ
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     // ʹ��
    EXTI_Init(&EXTI_InitStructure);                               // ��ʼ��, ��������Ĳ���д��Ĵ���

    // ���� NVIC, ���ж����ȼ�
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);            // ����NVICΪ���ȼ���2; ��������������ֻ������һ�Σ�ȫ����Ч; ���������ã�����ʱ�����һ������Ϊ׼; �����������÷���main������һ��;
    NVIC_InitStructure.NVIC_IRQChannel = KEY_3_INTERRUPT_NUMBER;  // �жϱ�ţ���ʾ�������жϵı�ţ����������ļ��У���оƬ�ڲ���ƶ�Ԥ���趨�õı�ŵġ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // ʹ��
    NVIC_Init(&NVIC_InitStructure);                               // ��ʼ��, ������Ĳ���д��Ĵ���
}



// ����_3�������ŵ��жϺ���; ����ʹ���˺궨�����ƣ��Է�����ֲ
void KEY_3_IRQHANDLER(void)
{
    LED_RED_TOGGLE;                                               // ��ת��ɫLED
    printf("�� 3 ������������, ���Ʒ�ת\r");                      // ��Ҫ��ʾ��printf�ǲ������뺯�����жϷ�������ʹ�ã����ܻ��������Ԥ��Ĵ�������ʹ��printf��ֻ�ô������ʹ�ã���      // ħŮ������İ���ʹ�õ��ݽ���Ӳ������,������ʹ�������ʱ����
    EXTI_ClearITPendingBit(KEY_3_EXTI_LINE);                      // ����жϱ�־λ;EXTI_Line4
}



/******************************************************************************
 * ��  ���� Key_Init
 * ��  �ܣ� ��ʼ������
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
void Key_Init(void)
{
    key_1_Init();
    key_2_Init();
    key_3_Init();
    printf("���� ��ʼ��              �������\r");
}


