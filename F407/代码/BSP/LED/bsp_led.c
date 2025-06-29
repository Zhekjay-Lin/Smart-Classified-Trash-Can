/***********************************************************************************************************************************
 ** 【文件名称】  led.c
 ** 【编写人员】  魔女开发板团队
 ** 【淘    宝】  魔女开发板      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【文件功能】  实现LED指示灯常用的初始化函数、功能函数
 **
 ** 【移植说明】
 **
 ** 【更新记录】
 **
***********************************************************************************************************************************/
#include "bsp_led.h"
#include "Delay.h"


void Led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;    // 定义一个GPIO_InitTypeDef类型的结构体

    // 使能LED_RED所用引脚端口时钟；使用端口判断的方法使能时钟, 以使代码移植更方便
    if (LED_RED_GPIO == GPIOA)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    if (LED_RED_GPIO == GPIOB)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    if (LED_RED_GPIO == GPIOC)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    if (LED_RED_GPIO == GPIOD)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    if (LED_RED_GPIO == GPIOE)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    if (LED_RED_GPIO == GPIOF)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    if (LED_RED_GPIO == GPIOG)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    // 使能LED_BLUE所用引脚端口时钟；使用端口判断的方法使能时钟, 以使代码移植更方便
    if (LED_BLUE_GPIO == GPIOA)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    if (LED_BLUE_GPIO == GPIOB)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    if (LED_BLUE_GPIO == GPIOC)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    if (LED_BLUE_GPIO == GPIOD)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    if (LED_BLUE_GPIO == GPIOE)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    if (LED_BLUE_GPIO == GPIOF)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    if (LED_BLUE_GPIO == GPIOG)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    // 配置LED_RED引脚工作模式
    GPIO_InitStructure.GPIO_Pin   = LED_RED_PIN;     // 选择要控制的GPIO引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   // 引脚模式：输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   // 输出类型：推挽输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;    // 上下拉：  上拉模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // 引脚速率：2MHz
    GPIO_Init(LED_RED_GPIO, &GPIO_InitStructure);    // 调用库函数，使用上面的配置初始化GPIO

    // 配置LED_RED引脚工作模式
    GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN;      // 选择要控制的GPIO引脚
    GPIO_Init(LED_BLUE_GPIO, &GPIO_InitStructure);   // 使用刚才LED_RED相同的其它参数，配置相关的GPIO引脚

    // 点亮红色LED、蓝色LED
    //GPIO_ResetBits(LED_RED_GPIO, LED_RED_PIN);       // 点亮LED_RED， 低电平点亮
    //GPIO_ResetBits(LED_BLUE_GPIO, LED_BLUE_PIN);     // 点亮LED_BLUE，低电平点亮 

    // 关闭LED
    LED_BLUE_OFF;
    LED_RED_ON;
            
}

void BLUE_blink(void)
{
     LED_BLUE_TOGGLE;
     delay_ms(1000);
}



