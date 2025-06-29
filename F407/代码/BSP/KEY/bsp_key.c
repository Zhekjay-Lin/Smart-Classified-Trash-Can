#include "bsp_key.h"



// 声明本地函数
static void key_1_Init(void);                                     // 声明key_1初始化函数; static是修改函数的有效范围：从定义声明位置起，源文件内可访问;
static void key_2_Init(void);                                     // 声明key_2初始化函数; static是修改函数的有效范围：从定义声明位置起，源文件内可访问;
static void key_3_Init(void);                                     // 声明key_3初始化函数; static是修改函数的有效范围：从定义声明位置起，源文件内可访问;






// 按键_1的初始化函数
static void key_1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;                          // 定义GPIO结构体，用于配置引脚工作模式
    EXTI_InitTypeDef EXTI_InitStructure;                          // 定义EXTI结构体，用于配置外部中断线，即引脚的中断方式
    NVIC_InitTypeDef NVIC_InitStructure;                          // 定义NVIC结构体，用于配置中断优先级

    // 时钟使能
    RCC_AHB1PeriphClockCmd(KEY_1_CLK, ENABLE);                    // 使能KEY_1所用引脚端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);        // 使能系统配置器SYSCFG的时钟，使用GPIO外部中断必须使能SYSCFG时钟

    // 配置引脚工作模式: PA0、 输入、闲时下拉、按下置高电平
    GPIO_InitStructure.GPIO_Pin   = KEY_1_PIN;                    // 选择要控制的引脚编号; 此处使用了宏定义，以方便移植修改
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;                 // 引脚模式：输入模式
    GPIO_InitStructure.GPIO_PuPd  = KEY_1_PUPD;                   // 上下拉状态
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;              // 引脚速率：2MHz
    GPIO_Init(KEY_1_GPIO, &GPIO_InitStructure);                   // 初始化, 即把上面的参数写入寄存器

    // 把引脚连接至中断线
    SYSCFG_EXTILineConfig(KEY_1_EXTI_PORT, KEY_1_EXTI_PIN);       // 把引脚连接至中断线

    // 配置EXTI中断线
    EXTI_InitStructure.EXTI_Line = KEY_1_EXTI_LINE;               // 中断线
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;           // 中断模式
    EXTI_InitStructure.EXTI_Trigger = KEY_1_EXTI_TRIGGER;         // 触发方式
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     // 使能
    EXTI_Init(&EXTI_InitStructure);                               // 初始化, 即把上面的参数写入寄存器

    // 配置 NVIC, 即中断优先级
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);            // 配置NVIC为优先级组2; 本设置整个工程只需配置一次，全局有效; 如果多次配置，运行时以最后一次配置为准; 建议把这个配置放在main函数第一行;
    NVIC_InitStructure.NVIC_IRQChannel = KEY_1_INTERRUPT_NUMBER;  // 中断编号，提示：各种中断的编号，是在启动文件中，按芯片内部设计而预先设定好的编号的。
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // 使能
    NVIC_Init(&NVIC_InitStructure);                               // 初始化, 即上面的参数写入寄存器
}



// KEY_1所用引脚的中断函数; 这里使用了宏定义名称，以方便移植
void KEY_1_IRQHANDLER(void)
{
    LED_RED_TOGGLE ;                                              // 反转红色LED
    printf("第 1 个按键被按下, 蓝灯反转\r");                      // 重要提示：printf是不可重入函数，中断服务函数中使用，可能会产生不可预测的错误。这里使用printf，只用代码测试使用！！      // 魔女开发板的按键使用电容进行硬件消抖,无需再使用软件延时消抖
    EXTI_ClearITPendingBit(EXTI_Line0);                           // 清除中断标志位;
}



// 按键_2的初始化函数
static void key_2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;                          // 定义GPIO结构体，用于配置引脚工作模式
    EXTI_InitTypeDef EXTI_InitStructure;                          // 定义EXTI结构体，用于配置外部中断线，即引脚的中断方式
    NVIC_InitTypeDef NVIC_InitStructure;                          // 定义NVIC结构体，用于配置中断优先级

    // 时钟使能
    RCC_AHB1PeriphClockCmd(KEY_2_CLK, ENABLE);                    // 使能KEY_1所用引脚端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);        // 使能系统配置器SYSCFG的时钟，使用GPIO外部中断必须使能SYSCFG时钟

    // 配置引脚工作模式: PA0、 输入、闲时下拉、按下置高电平
    GPIO_InitStructure.GPIO_Pin   = KEY_2_PIN;                    // 选择要控制的引脚编号; 此处使用了宏定义，以方便移植修改
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;                 // 引脚模式：输入模式
    GPIO_InitStructure.GPIO_PuPd  = KEY_2_PUPD;                   // 上下拉状态
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;              // 引脚速率：2MHz
    GPIO_Init(KEY_2_GPIO, &GPIO_InitStructure);                   // 初始化, 即把上面的参数写入寄存器

    // 把引脚连接至中断线
    SYSCFG_EXTILineConfig(KEY_2_EXTI_PORT, KEY_2_EXTI_PIN);       // 把引脚连接至中断线

    // 配置EXTI中断线
    EXTI_InitStructure.EXTI_Line = KEY_2_EXTI_LINE;               // 中断线
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;           // 中断模式
    EXTI_InitStructure.EXTI_Trigger = KEY_2_EXTI_TRIGGER;         // 触发方式
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     // 使能
    EXTI_Init(&EXTI_InitStructure);                               // 初始化, 即把上面的参数写入寄存器

    // 配置 NVIC, 即中断优先级
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);            // 配置NVIC为优先级组2; 本设置整个工程只需配置一次，全局有效; 如果多次配置，运行时以最后一次配置为准; 建议把这个配置放在main函数第一行;
    NVIC_InitStructure.NVIC_IRQChannel = KEY_2_INTERRUPT_NUMBER;  // 中断编号，提示：各种中断的编号，是在启动文件中，按芯片内部设计而预先设定好的编号的。
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // 使能
    NVIC_Init(&NVIC_InitStructure);                               // 初始化, 即上面的参数写入寄存器
}



// 按键_2所用引脚的中断函数; 这里使用了宏定义名称，以方便移植
void KEY_2_IRQHANDLER(void)
{                                                                 
    LED_RED_TOGGLE;                                               // 反转红色LED
    printf("第 2 个按键被按下, 蓝灯反转\r");                      // 重要提示：printf是不可重入函数，中断服务函数中使用，可能会产生不可预测的错误。这里使用printf，只用代码测试使用！！      // 魔女开发板的按键使用电容进行硬件消抖,无需再使用软件延时消抖
    EXTI_ClearITPendingBit(KEY_2_EXTI_LINE);                      // 清除中断标志位;
}



// 按键_3的初始化函数
static void key_3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;                          // 定义GPIO结构体，用于配置引脚工作模式
    EXTI_InitTypeDef EXTI_InitStructure;                          // 定义EXTI结构体，用于配置外部中断线，即引脚的中断方式
    NVIC_InitTypeDef NVIC_InitStructure;                          // 定义NVIC结构体，用于配置中断优先级

    // 时钟使能
    RCC_AHB1PeriphClockCmd(KEY_3_CLK, ENABLE);                    // 使能KEY_1所用引脚端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);        // 使能系统配置器SYSCFG的时钟，使用GPIO外部中断必须使能SYSCFG时钟

    // 配置引脚工作模式: PA0、 输入、闲时下拉、按下置高电平
    GPIO_InitStructure.GPIO_Pin   = KEY_3_PIN;                    // 选择要控制的引脚编号; 此处使用了宏定义，以方便移植修改
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;                 // 引脚模式：输入模式
    GPIO_InitStructure.GPIO_PuPd  = KEY_3_PUPD;                   // 上下拉状态
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;              // 引脚速率：2MHz
    GPIO_Init(KEY_3_GPIO, &GPIO_InitStructure);                   // 初始化, 即把上面的参数写入寄存器

    // 把引脚连接至中断线
    SYSCFG_EXTILineConfig(KEY_3_EXTI_PORT, KEY_3_EXTI_PIN);       // 把引脚连接至中断线

    // 配置EXTI中断线
    EXTI_InitStructure.EXTI_Line = KEY_3_EXTI_LINE;               // 中断线
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;           // 中断模式
    EXTI_InitStructure.EXTI_Trigger = KEY_3_EXTI_TRIGGER;         // 触发方式
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     // 使能
    EXTI_Init(&EXTI_InitStructure);                               // 初始化, 即把上面的参数写入寄存器

    // 配置 NVIC, 即中断优先级
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);            // 配置NVIC为优先级组2; 本设置整个工程只需配置一次，全局有效; 如果多次配置，运行时以最后一次配置为准; 建议把这个配置放在main函数第一行;
    NVIC_InitStructure.NVIC_IRQChannel = KEY_3_INTERRUPT_NUMBER;  // 中断编号，提示：各种中断的编号，是在启动文件中，按芯片内部设计而预先设定好的编号的。
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // 使能
    NVIC_Init(&NVIC_InitStructure);                               // 初始化, 即上面的参数写入寄存器
}



// 按键_3所用引脚的中断函数; 这里使用了宏定义名称，以方便移植
void KEY_3_IRQHANDLER(void)
{
    LED_RED_TOGGLE;                                               // 反转红色LED
    printf("第 3 个按键被按下, 蓝灯反转\r");                      // 重要提示：printf是不可重入函数，中断服务函数中使用，可能会产生不可预测的错误。这里使用printf，只用代码测试使用！！      // 魔女开发板的按键使用电容进行硬件消抖,无需再使用软件延时消抖
    EXTI_ClearITPendingBit(KEY_3_EXTI_LINE);                      // 清除中断标志位;EXTI_Line4
}



/******************************************************************************
 * 函  数： Key_Init
 * 功  能： 初始化按键
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void Key_Init(void)
{
    key_1_Init();
    key_2_Init();
    key_3_Init();
    printf("按键 初始化              配置完成\r");
}


