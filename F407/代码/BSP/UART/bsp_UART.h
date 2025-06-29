#ifndef __BSP__UART_H
#define __BSP__UART_H
/***********************************************************************************************************************************
 
 ** 【文件名称】  bsp_UART.h
 **
 ** 【 功  能 】  串口通信底层驱动(UART1、UART2、UART3、UART4、UART5、UART6)
 **               波特率-None-8-1
 **               调用全局声明中的串口函数，即可完成初始化、发送、接收.
 **
 ** 【 约  定 】  本文件所用串口通信，均为异步通信。
 **               2024年起更新的示例，串口通信函数命名时，统一使用"UART"，而不使用旧文件中的"USART".
 **
 ** 【适用平台】  STM32F407 + keil5 + 标准库/标准库
 **
 ** 【串口引脚】  各个串口的初始化函数UARTx_Init()，将按以下默认引脚进行初始化：
 **               1- UART1  TX_PA9   RX_PA10       特别说明：魔女开发板系列，均板载USB转TTL，PCB已布线连接好UART1, 使用和烧录用的同一USB接口，即可通过UART1和电脑进行通信。具体查看资料文件夹中的说明文件。
 **               2- UART2  TX_PA2   RX_PA3
 **               3- UART3  TX_PB10  RX_PB11
 **               4- UART4  TX_PC10  RX_PC11
 **               5- UART5  TX_PC12  RX_PD2
 **               6- UART6  TX_PC6   RX_PC7
 **
 ** 【移植说明】  1- 如果使用CubeMX配置的工程，无须对UART进行任何配置。
 **               2- 适用于HAL库、标准库工程，通过下面操作，均可直接使用。
 **               3- 复制本工程的UART文件夹，到目标工程文件夹中;
 **               4- 添加头文件路径： Keil > Option > C/C++ > Include Paths;
 **               5- 添加C文件到工程: Keil > 左侧工程管理器中双击目标文件夹 > 选择 bsp_UART.c;
 **               6- 添加文件引用:    #include "bsp_uart.h＂，即哪个文件要用串口功能，就在其代码文件顶部添加引用;
 **
 ** 【代码使用】  每组串口，已封装好7个函数 (初始化1个、发送3个、接收3个)。下面以UART1作示范说明：               
 **               1、初始化:             void      UART1_Init(uint32_t ulBaudrate);                                      // 初始化串口; 配置GPIO引脚PA9+PA10、配置通信参数：波特率-None-8-1、配置中断
 **               2、发送指定长度的数据  void      UART1_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定长度的数据; 参数：数据地址、字节数
 **               3、发送字符串          void      UART1_SendString(const char *pcString,...);                           // 发送字符串;   参数：字符串地址; 使用方法如同printf
 **               4、发送AT指令          uint8_t   UART1_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
 **               5、获取接收的字节数:   uint16_t  UART1_GetRxNum(void);                                                 // 获取接收到的字节数; 如果返回值大于0，即为接收到新一帧数据
 **               6、获取接收的数据：    uint8_t*  UART1_GetRxData(void);                                                // 获取接收到的数据(缓存的地址)
 **               7、清零接收标志：      void      UART1_ClearRx(void);                                                  // 清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0，方可进入下轮接收判断
 **
 ** 【更新记录】  2024-06-20  优化AT指令函数：名称、机制;
 **               2024-06-07  增加AT指令等待反馈处理函数：UARTx_WaitACK();
 **               2024-06-07  优化函数：UARTx_SendString();
 **               2024-06-06  优化代码适配，使文件能适用于标准库、HAL库工程
 **               2024-04-04  UARTx_SendString()，取消临时缓存的static，以减少RAM占用，适配LVGL移植;
 **               2024-03-27  修改xUSATR_TypeDef结构体，优化缓冲的存储方式
 **               2024-02-04  简化接收函数命名，函数名中使用Rx字符，代替旧函数名称中的Reveived字符;
 **               2024-01-09  取消部分静态变量，并入结构体中，使用调用更直观
 **               2024-01-03  完善函数、注释
 **               2023-12-25  增加接收函数的封装，取消接收变量(全局)的使用
 **               2023-12-23  完成对HAL库的支持
 **               2023-01-27  增加宏定义、完善注释
 **               2021-12-16  完善接收机制：取消接收标志，判断接收字节数>0即为接收到新数据
 **               2021-11-03  完善接收函数返回值处理
 **               2021-08-14  增加宏定义：接收缓存区大小设定值，使空间占用更可控；
 **               2021-08-14  修改发送函数名称：UARTx_Printf()，修改为：UARTx_SendString();
 **               2020-09-02  文件建立、UART1接收中断、空闲中断、发送中断、DMA收发
 **               2021-06-04  UART1、2、3及UART4、5 的收发完善代码
 **               2021-06-09  完善文件格式、注释
 **               2021-06-22  完善注释说明
 **
************************************************************************************************************************************/
#include <stdarg.h>                               // 用于支持不定长参数
#include "stdio.h"                                // 用于支持函数printf、spritnf等
#include "string.h"                               // 用于支持字符串处理函数strset、strcpy、memset、metcpy等


#ifdef USE_STDPERIPH_DRIVER
    #include "stm32f4xx.h"                        // 标准库
#endif

#ifdef USE_HAL_DRIVER
    #include "stm32f4xx_hal.h"                    // HAL库
#endif



/*****************************************************************************
 ** 移植配置修改区
 ** 备注：除非有特殊要求，否则，下面参数已通用：RS485、蓝牙模块、ESP8266模块、串口屏等
****************************************************************************/
// 串口开关
#define UART1_EN                       1          // 串口1，0=关、1=启用;  倘若没用到UART1, 置0，就不会开辟UART1发送缓存、接收缓存，省一点资源;
#define UART2_EN                       1          // 串口2，0=关、1=启用;  同上;
#define UART3_EN                       1          // 串口3，0=关、1=启用;  同上;
#define UART4_EN                       1          // 串口4，0=关、1=启用;  同上;
#define UART5_EN                       1          // 串口5，0=关、1=启用;  同上;
#define UART6_EN                       1          // 串口5，0=关、1=启用;  同上;
// 发送缓冲区大小
#define UART1_TX_BUF_SIZE           2048          // 配置每组UART发送环形缓冲区数组的大小，单位：字节数; 
#define UART2_TX_BUF_SIZE            512          // -- 只有在前面串口开关在打开状态，才会定义具体的缓冲区数组
#define UART3_TX_BUF_SIZE            512          // -- 默认值：512，此值已能适配大部场景的通信; 如果与ESP8266之类的设备通信，可适当增大此值。
#define UART4_TX_BUF_SIZE            512          // -- 值范围：1~65535; 注意初始化后，不要超过芯片最大RAM值。
#define UART5_TX_BUF_SIZE            512          // -- 注意此值是一个环形缓冲区大小，决定每一帧或多帧数据进入发送前的总缓存字节数，先进先出。
#define UART6_TX_BUF_SIZE            512          //
// 接收缓冲区大小
#define UART1_RX_BUF_SIZE            512          // 配置每组UART接收缓冲区的大小，单位：字节; 此值影响范围：中断里的接收缓存大小，接收后数据缓存的大小
#define UART2_RX_BUF_SIZE            512          // --- 当接收到的一帧数据字节数，小于此值时，数据正常；
#define UART3_RX_BUF_SIZE            512          // --- 当接收到的一帧数据字节数，超过此值时，超出部分，将在中断中直接弃舍，直到此帧接收结束(发生空闲中断); 
#define UART4_RX_BUF_SIZE            512          // 
#define UART5_RX_BUF_SIZE            512          // 
#define UART6_RX_BUF_SIZE            512          // 
// 结束-配置修改



/*****************************************************************************
 ** 声明全局函数
 **
 ** 每个串口的函数:
 ** 初始化    1个   波特率-None-8-1
 ** 发送      3个   发送指定长度数据、字符串、AT指令 
 ** 接收      3个   获取字节数、获取数据、清0
****************************************************************************/
// UART1
void      UART1_Init(uint32_t ulBaudrate);                                      // 初始化串口1; GPIO引脚PA9+PA10、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART1_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART1_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART1_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART1_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART1_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART1_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART2                                                                        
void      UART2_Init(uint32_t ulBaudrate);                                      // 初始化串口2; GPIO引脚PA2+PA3、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART2_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART2_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART2_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART2_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART2_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART2_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART3                                                                        
void      UART3_Init(uint32_t ulBaudrate);                                      // 初始化串口3; GPIO引脚PB10+PB11、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART3_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART3_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART3_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART3_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART3_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART3_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART4                                                            
void      UART4_Init(uint32_t ulBaudrate);                                      // 初始化串口4; GPIO引脚PC10+PC11、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART4_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART4_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART4_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART4_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART4_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART4_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART5                                                                        
void      UART5_Init(uint32_t ulBaudrate);                                      // 初始化串口5; GPIO引脚PC12+PD2、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART5_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART5_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART5_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART5_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART5_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART5_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART6                                                                        
void      UART6_Init(uint32_t ulBaudrate);                                      // 初始化串口6; GPIO引脚PC6+PC7、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART6_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART6_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART6_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART6_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART6_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART6_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
                                                                                
// 调试辅助                                                                     
void      showData(uint8_t *puRxData, uint16_t usRxNum);                        // 经printf，发送到串口助手上，方便观察

//发送给上位机
void Send_To_Upper(const char *pcString, ...);

#endif

