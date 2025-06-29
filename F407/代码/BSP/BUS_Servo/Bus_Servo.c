/******************************************************************************
 * ��  �ܣ� ���߶���˶���������
 * ��  ע�� ���      ���        ����            ��ʼ                     ��Χ
 *          001       ��̨        Gimbal          503 ����Ӧ�ɻ���Ͱ��
 *          002       Ͷ����      Door            660����Ӧ���ţ�
 *          003       ��צ��    Wrist           1128 ����Ӧ90�㣩      1128-1785��0�㣩
 ******************************************************************************/


#include "stm32f4xx.h"                  // Device header
#include "Bus_Servo.h"
#include "bsp_UART.h"
#include "Calculate.h"


void Gimbal_Servo(uint8_t type)
{
  switch(type)
  {
    case 1:
      UART3_SendString("#001P0500T0400!");   // �ɻ�������Ͱ
      break;
    case 2:
      UART3_SendString("#001P1145T0400!");   // ��������Ͱ
      break;
    case 3:
      UART3_SendString("#001P1838T0400!");   // �к�����Ͱ
      break;
    case 4:
      UART3_SendString("#001P2468T0400!");   // ��������Ͱ
      break;    
  }      
}


void Door_Servo(uint8_t doorStatus)
{
  if(doorStatus == 2)
  {
    UART3_SendString("#002P0660T0400!");   // ����
  }
  else
  {
    UART3_SendString("#002P1110T0400!");   // ����
  }
}

void Wrist_ServoMove(int wristangle)
{
  int WristAngle = Calulate_WristAngle(wristangle) + 1128;      // ��ʵ�ʽǶ�ת��Ϊ���������
  
  char str[16] = "0";
  sprintf(str, "#003P%dT0400!", WristAngle);
  UART3_SendString(str);
}

