/******************************************************************************
 * 功  能： 总线舵机运动初步控制
 * 备  注： 编号      舵机        代号            初始                     范围
 *          001       云台        Gimbal          503 （对应可回收桶）
 *          002       投放门      Door            660（对应关门）
 *          003       夹爪腕部    Wrist           1128 （对应90°）      1128-1785（0°）
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
      UART3_SendString("#001P0500T0400!");   // 可回收垃圾桶
      break;
    case 2:
      UART3_SendString("#001P1145T0400!");   // 厨余垃圾桶
      break;
    case 3:
      UART3_SendString("#001P1838T0400!");   // 有害垃圾桶
      break;
    case 4:
      UART3_SendString("#001P2468T0400!");   // 其他垃圾桶
      break;    
  }      
}


void Door_Servo(uint8_t doorStatus)
{
  if(doorStatus == 2)
  {
    UART3_SendString("#002P0660T0400!");   // 关门
  }
  else
  {
    UART3_SendString("#002P1110T0400!");   // 开门
  }
}

void Wrist_ServoMove(int wristangle)
{
  int WristAngle = Calulate_WristAngle(wristangle) + 1128;      // 将实际角度转换为舵机控制数
  
  char str[16] = "0";
  sprintf(str, "#003P%dT0400!", WristAngle);
  UART3_SendString(str);
}

