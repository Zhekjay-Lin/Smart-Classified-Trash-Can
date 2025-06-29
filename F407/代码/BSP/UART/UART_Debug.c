#include "stm32f4xx.h"                  // Device header
#include "UART_Debug.h"
#include "bsp_UART.h"
#include "stdlib.h"

void uart_debugfunction(void)
{
  if (UART1_GetRxNum())                                            // �жϴ����Ƿ���յ�����
  {
     uint16_t  usRxNum = UART1_GetRxNum();                        // ��ȡ���ڽ������һ֡���ֽ���
     uint8_t  *puRxData = UART1_GetRxData();                      // ��ȡ���ڽ������һ֡������
     printf("\r<<<<<< USART1 ���յ������� <<<<<<");               // ׼�����
     printf("\r ����(�ֽ���)��%d", usRxNum);                      // ����; ��λ:�ֽ�
     printf("\r ����(16����)��");                                 // 16���Ʒ�ʽ��ʾ���ݣ�����۲���ʵ����
     for (uint16_t i = 0; i < usRxNum; i++)                       // ���ÿһ�ֽ���ֵ
        printf("0x%X ", puRxData[i]);                            // ��ʽ�����
     printf("\r ����(ASCII) ��%s\r", puRxData);                   // ASCII��ʽ��ʾ���ݣ�����۲��ַ�������;         
     UART4_SendData(puRxData, usRxNum);
     UART1_ClearRx();                                             // ��0����; ��Ҫ��ÿ�δ����������ˣ�Ҫ���ñ����������ɽ�����һ�ֵ��ж�; ע�⣺��������0���ǽ��յ��ֽ���
  }

   if (UART4_GetRxNum())                                            // �жϴ����Ƿ���յ�����
   {
     uint16_t  usRxNum = UART4_GetRxNum();                        // ��ȡ���ڽ������һ֡���ֽ���
     uint8_t  *puRxData = UART4_GetRxData();                      // ��ȡ���ڽ������һ֡������
     printf("\r<<<<<< USART4 ���յ������� <<<<<<");               // ׼�����
     printf("\r ����(�ֽ���)��%d", usRxNum);                      // ����; ��λ:�ֽ�
     printf("\r ����(16����)��");                                 // 16���Ʒ�ʽ��ʾ���ݣ�����۲���ʵ����
     for (uint16_t i = 0; i < usRxNum; i++)                       // ���ÿһ�ֽ���ֵ
        printf("0x%X ", puRxData[i]);                            // ��ʽ�����
     printf("\r ����(ASCII) ��%s\r", puRxData);                   // ASCII��ʽ��ʾ���ݣ�����۲��ַ�������;         
                                                                  // ��0����; ��Ҫ��ÿ�δ����������ˣ�Ҫ���ñ����������ɽ�����һ�ֵ��ж�; ע�⣺��������0���ǽ��յ��ֽ���         
   }
}

void usart_init(void)
{
     uint8_t ak[6] = {0,1,2,3,4,5};     
     UART2_SendString("ok!");
     UART2_SendData(ak,6);
}
