#include "stm32f4xx.h"                  // Device header
#include "UART_Debug.h"
#include "bsp_UART.h"
#include "stdlib.h"

void uart_debugfunction(void)
{
  if (UART1_GetRxNum())                                            // 判断串口是否接收到数据
  {
     uint16_t  usRxNum = UART1_GetRxNum();                        // 获取串口接收最后一帧的字节数
     uint8_t  *puRxData = UART1_GetRxData();                      // 获取串口接收最后一帧的数据
     printf("\r<<<<<< USART1 接收到新数据 <<<<<<");               // 准备输出
     printf("\r 长度(字节数)：%d", usRxNum);                      // 长度; 单位:字节
     printf("\r 数据(16进制)：");                                 // 16进制方式显示数据，方便观察真实数据
     for (uint16_t i = 0; i < usRxNum; i++)                       // 输出每一字节数值
        printf("0x%X ", puRxData[i]);                            // 格式化输出
     printf("\r 数据(ASCII) ：%s\r", puRxData);                   // ASCII方式显示数据，方便观察字符串数据;         
     UART4_SendData(puRxData, usRxNum);
     UART1_ClearRx();                                             // 清0接收; 重要：每次处理完数据了，要调用本函数，方可进行下一轮的判断; 注意：本函数清0的是接收的字节数
  }

   if (UART4_GetRxNum())                                            // 判断串口是否接收到数据
   {
     uint16_t  usRxNum = UART4_GetRxNum();                        // 获取串口接收最后一帧的字节数
     uint8_t  *puRxData = UART4_GetRxData();                      // 获取串口接收最后一帧的数据
     printf("\r<<<<<< USART4 接收到新数据 <<<<<<");               // 准备输出
     printf("\r 长度(字节数)：%d", usRxNum);                      // 长度; 单位:字节
     printf("\r 数据(16进制)：");                                 // 16进制方式显示数据，方便观察真实数据
     for (uint16_t i = 0; i < usRxNum; i++)                       // 输出每一字节数值
        printf("0x%X ", puRxData[i]);                            // 格式化输出
     printf("\r 数据(ASCII) ：%s\r", puRxData);                   // ASCII方式显示数据，方便观察字符串数据;         
                                                                  // 清0接收; 重要：每次处理完数据了，要调用本函数，方可进行下一轮的判断; 注意：本函数清0的是接收的字节数         
   }
}

void usart_init(void)
{
     uint8_t ak[6] = {0,1,2,3,4,5};     
     UART2_SendString("ok!");
     UART2_SendData(ak,6);
}
