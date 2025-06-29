#include "stm32f4xx.h"
#include "bsp_led.h"
#include "bsp_UART.h"
#include "Delay.h"
#include "Sensor.h"
#include "Motor.h"
#include "Switch.h"
#include "UART_Debug.h"
#include "Linear_Actuator.h"
#include "Bus_Servo.h"
#include "Calculate.h"
#include "Claw_Servo.h"

#include "stdlib.h"

extern speedRampData srdx;
extern speedRampData srdy;
extern speedRampData srdz;

/**/
uint8_t XbackStatus = 0;
uint8_t Xmin = 0;


/*系统状态宏定义*/
#define       Wait          0       // 等待信号
#define       Judge         1       // 判断类别
#define       Motor         2       // 滑台运动
#define       Claw          3       // 夹爪运动
#define       Compress      4       // 推杆运动
#define       Test          5       // 满载检测

/*垃圾类别宏定义*/
#define       Recycle       1       // 可回收垃圾
#define       Harm          3       // 有害垃圾
#define       Kitchen       2       // 厨余垃圾
#define       Dry           4       // 其他垃圾（干垃圾）

/*夹爪状态宏定义*/
#define       GrabGo        1       // 开始抓取
#define       Release       2       // 释放
#define       GrabBack      3       // 抓完回程

/*挡板状态宏定义*/
#define       Open          1       // 打开
#define       Close         2       // 关闭



/*保存系统当前状态*/
uint8_t SystemStatus = Wait;

/*保存当前识别垃圾类型*/
uint8_t Type;

/*保存夹爪方向和张开角度*/
int32_t wristAngle;
int32_t clawAngle;
uint8_t clawpower;

/*保存当前挡板状态*/
uint8_t DoorStatus;

/*保存光电开关状态、秒数计数器和定时器6状态*/
uint8_t switchstatus = 0;
uint32_t countNum = 0;
uint8_t TIM6flag = 0;

/*夹爪结构体*/
typedef struct
{
  uint8_t ClawStatus;       // 夹爪状态
  int32_t wristTarget;      // 夹爪目标方向
  int32_t angleTarget;      // 夹爪目标张开角度  
  //uint8_t power;          // 是否需要保持力矩
}ClawStruct;

typedef struct
{
  uint32_t SetSpeed;        // 最大速度    单位为0.1rad/sec
  uint32_t SetAccel;        // 加速度设置  单位为0.1rad/sec^2
  uint32_t SetDecel;        // 减速度设置  单位为0.1rad/sec^2
  
  int32_t LocaTarget;       // 目标位置
  int32_t LocaCurrent;      // 当前位置
  
  int32_t stepTarget;       // 目标脉冲数
  int32_t stepBack;         // 回程脉冲数
  uint8_t flag;             // 函数状态   
}Slide;



int main(void)
{
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                      // 设置系统中断优先级分组2
                                                                  
    Led_Init();
    delay_init(168);                                                    // 延时函数和系统时钟初始化
      
    Motor_Init();  
    LinearActuator_Init();
  
    claw_servo_Init();
        
    UART1_Init(115200);
    UART3_Init(115200);
    UART4_Init(115200);
    
    delay_ms(1000);                                                    // 等待系统启动完成
    
    ClawStruct claw ={GrabGo, 90, 140};                             // 初始化夹爪状态
    Slide X_Slide = {20000,18000,18000,0,0,0,0,1};                   // 初始化各轴参数
    Slide Y_Slide = {15000,12000,12000,0,0,0,0,1};
    Slide Z_Slide = {22000,18000,18000,0,0,0,0,0};

    

    while (1)                                                            
    {  
      switch(SystemStatus)
      {
          /*待完善夹爪目标角度*/
          case Wait:                           
              uart_debugfunction();                              // 没有获取到最新字节数，一直循环等待新字节数输入
              if(UART4_GetRxNum())
              {                                                     // 获取到新字节数，开始解包           
                 uint16_t  usRxNum = UART4_GetRxNum();                // 获取串口接收最后一帧的字节数
                 uint8_t  *puRxData = UART4_GetRxData();              // 获取串口接收最后一帧的数据的地址

                 UART4_ClearRx();                                      // 读完数值清除缓冲区
                                                                        
                  /*数据拆分并转换为数值*/
                 X_Slide.LocaTarget = -String2int(puRxData, usRxNum, 0);         // 更新X轴目标位置
                
                 Y_Slide.LocaTarget = String2int(puRxData, usRxNum, 1);          // 更新Y轴目标位置
                
                 claw.wristTarget = String2int(puRxData, usRxNum, 2);            // 更新夹爪方向
                 claw.angleTarget = String2int(puRxData, usRxNum, 3) + 5;        // 更新夹爪张开角度
                 Type = String2int(puRxData, usRxNum, 4);                        // 更新垃圾类别
                               
                 X_Slide.stepTarget = Calulate_Step(X_Slide.LocaTarget, X_Slide.LocaCurrent) + 500;  // 更新X目标脉冲数
                 Y_Slide.stepTarget = Calulate_Step(Y_Slide.LocaTarget, Y_Slide.LocaCurrent);        // 更新Y目标脉冲数
                
                 /*限位判断*/
                 if(X_Slide.stepTarget <= 0)
                 {
                   X_Slide.stepTarget = 0;
                   Xmin = 1;                                                // X轴最小极限标志置1
                 }
                 else if(X_Slide.stepTarget >= 1300 )                 
                 {
                   X_Slide.stepTarget = 1300;
                 }                
                  
                 //Send_To_Upper("OK");                                     // 收到指令向上位机反馈
                 
                 //printf("\r X轴目标：%d \rY轴目标：%d \r夹爪方向：%d \r 夹爪张开角度：%d \r垃圾类别：%d \rX、Y目标脉冲：%d，%d", X_Slide.LocaTarget, Y_Slide.LocaTarget, claw.wristTarget, claw.angleTarget, Type, X_Slide.stepTarget, Y_Slide.stepTarget);                                                                             
                 SystemStatus = Judge;                                // 跳转到下一状态              
              }
              break;
          
            case Judge:                                  
               Gimbal_Servo(Type);                                  // 根据垃圾类别转动云台
//               if(Type == Recycle)                                  // 更新挡板状态 可回收，则关闭挡板
//                  DoorStatus = Close;
//               else
                 DoorStatus = Open;
               
               Door_Servo(DoorStatus);                              // 移动挡板
               SystemStatus = Motor;                                 
               delay_ms(500);               // 等待此状态动作完成               
               break;
                        
            case Motor:
                Motor_EN();                                         // 电机驱动使能             
                if(claw.ClawStatus == Release)
                {
                  Wrist_ServoMove(90);                              // 三轴运动的同时夹爪回正
                }             
                if(X_Slide.flag == 1)                                    // 只在函数状态为1的情况下使用Motor_Move函数                
                {
                  MOTOR_Move(X_Slide.stepTarget, X_Slide.SetAccel, X_Slide.SetDecel, X_Slide.SetSpeed, X);                
                }             
                if(Y_Slide.flag == 1)
                {
                  MOTOR_Move(Y_Slide.stepTarget, Y_Slide.SetAccel, Y_Slide.SetDecel, Y_Slide.SetSpeed, Y);               
                }
                if(Z_Slide.flag == 1)
                {
                  MOTOR_Move(Z_Slide.stepTarget, Z_Slide.SetAccel, Z_Slide.SetDecel, Z_Slide.SetSpeed, Z);                
                }
                if(XbackStatus == 1)
                {
                  if(Xmin != 1)
                  {
                    MOTOR_Move(-500, X_Slide.SetAccel, X_Slide.SetDecel, X_Slide.SetSpeed, X);     // X轴回零
                  }
                }                                          
                if(srdx.run_state == STOP)             
                {
                
                  if((claw.ClawStatus == GrabGo) && X_Slide.flag)
                  {
                    if(Xmin == 0)
                    {
                      X_Slide.stepBack = X_Slide.stepTarget - 500;
                      //printf("\r %d", X_Slide.stepBack);
                    }
                    else
                      X_Slide.stepBack = 0;
                  }
                  X_Slide.stepTarget = 0;
                  X_Slide.flag = 0;               
                }
                
                if(srdy.run_state == STOP)
                {
                
                  if((claw.ClawStatus == GrabGo) && Y_Slide.flag)
                  {
                    Y_Slide.stepBack = Y_Slide.stepTarget;                 
                  }
                  delay_ms(1);
                  Y_Slide.stepTarget = 0;
                  Y_Slide.flag = 0;               
                }
              
                 if(srdz.run_state == STOP)
                { 
                  if(Z_Slide.flag && (claw.ClawStatus == GrabBack))  // 一个分类周期内只保存第二次进入Motor状态的Z回程值
                  {
                    Z_Slide.stepBack = Z_Slide.stepTarget;
                  }
                  Z_Slide.stepTarget = 0;
                  Z_Slide.flag = 0;
                }                                
                
                if(srdx.run_state == STOP && srdy.run_state == STOP && srdz.run_state == STOP)
                {
                   
                   Motor_DIS();                                       // 每次退出电机运动状态，驱动失能，减小系统电流
                   
                   //printf("\r XSTEP：%d YSTEP：%d ZSTEP：%d", X_Slide.stepBack, Y_Slide.stepBack, Z_Slide.stepBack);
                  
                   if(XbackStatus == 1)                 // 此时为最后一次进入motor状态
                   {
//                         if(Type == Recycle)            // 如果是可回收垃圾
//                         {
//                          SystemStatus = Compress;     // 将进入压缩状态
//                          DoorStatus = Open;           // 更新挡板状态，方便下阶段挡板运动
//                         }
//                         else
//                         {
                           SystemStatus = Wait;         // 不是可回收垃圾，直接进入检测状态
                           //Send_To_Upper("send");
//                         }
                           
                      
                         claw.ClawStatus = GrabGo;      // 一个周期完成，夹爪恢复待夹取状态
                         X_Slide.flag = 1;              // 函数标志置一，准备下一分类周期执行X，Y运动
                         Y_Slide.flag = 1;
                          
                         X_Slide.stepBack = 0;
                         Y_Slide.stepBack = 0;
                         Z_Slide.stepBack = 0;
                         
                         XbackStatus = 0;
                         Xmin = 0;                       // 清除X轴最小极限标志
                   }
                   else                                                 // 不是最后一次进入motor状态
                     SystemStatus = Claw;                               // 进入夹爪运动状态
                }
                                                        
                break;
 
              case Claw: 
                delay_ms(1);
                if(claw.ClawStatus == GrabGo)
                {
                  Wrist_ServoMove(claw.wristTarget);               // 夹爪转到对应方向
                  delay_ms(100);
                  claw_servo_setAngle(claw.angleTarget);                        // 夹爪张开准备抓取
                  delay_ms(200);
                  
                  Z_Slide.stepTarget = -200*5;                     // 设置Z轴步数，5圈
                  Z_Slide.flag = 1;                                // 函数标志置一，准备下次执行Z轴运动
                  claw.ClawStatus = GrabBack;
                  SystemStatus = Motor;
                  //printf("\r 1");                
                }
                else if(claw.ClawStatus == GrabBack)
                {
                  claw_servo_setAngle(4);                           // 夹爪夹住垃圾
//                  for(int i=claw.angleTarget;i>=0;i--)
//                  {
//                    claw_servo_setAngle(i);
//                    //delay_ms(1);
//                    if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 1)
//                    {
//                      printf("\r 角度为：%d", i);
//                      printf("\r 按钮按下");
//                      break;
//                    }                            
//                  }
                  
                  delay_ms(500);                  
                  //claw.power = 1;                                 // 夹爪保持力矩
                  
                  X_Slide.flag = 1;                                // 函数标志置一，准备下次执行三轴运动
                  Y_Slide.flag = 1;
                  Z_Slide.flag = 1;
                  delay_ms(1);
                  
                  
                  X_Slide.stepTarget = -X_Slide.stepBack;                       // 将保存的回程步数设为目标步数                  
                  Y_Slide.stepTarget = -Y_Slide.stepBack;
                  Z_Slide.stepTarget = -Z_Slide.stepBack;
                  //printf("\r %d", X_Slide.stepTarget);
                  
                  claw.ClawStatus = Release;                       // 更新夹爪下一状态
                  SystemStatus = Motor;                            // 返回Motor状态 滑台回程
                  //printf("\r 2");                                  
                }
                else if(claw.ClawStatus == Release)
                {                                   
                  //claw.power = 0;
                  claw_servo_setAngle(claw.angleTarget + 5);      // 释放
                  delay_ms(500);
                  claw_servo_home();
                  
                  XbackStatus = 1;                          // 最后一次进入Motor状态
                  SystemStatus = Motor;
                                 
                  
//                  if(Type == Recycle)            // 如果是可回收垃圾
//                  {
//                    SystemStatus = Compress;     // 将进入压缩状态
//                    DoorStatus = Open;           // 更新挡板状态，方便下阶段挡板运动
//                  }
//                  else
//                    SystemStatus = Wait;         // 不是可回收垃圾，直接进入检测状态
//                  
//                  claw.ClawStatus = GrabGo;      // 一个周期完成，夹爪恢复待夹取状态
//                  X_Slide.flag = 1;              // 函数标志置一，准备下一分类周期执行X，Y运动
//                  Y_Slide.flag = 1;
//                  
//                  X_Slide.stepBack = 0;
//                  Y_Slide.stepBack = 0;
//                  Z_Slide.stepBack = 0;
                  
                  //printf("\r 3");
                }
                break;
                
//              case Compress:
//                /*待完善力学传感器*/
//                delay_ms(200);                 // 等待垃圾掉落
//                  
//                LinearAcuator(1);              // 推杆电机运动
//                LinearAcuator(0);
//                LinearAcuator(2);
//                LinearAcuator(0);                        
//                
//                Door_Servo(DoorStatus);        // 开门
//                
//                delay_ms(100);                 // 等待动作完成
//                    
//                SystemStatus = Wait;
//                //Send_To_Upper("send");
//                break;
            } 
              //case Test:                
                switchstatus = SwitchStatus();  // 每个循环均扫描一次红外开关状态
                if(switchstatus)                // 如果遮挡
                {
                  if(TIM6flag == 0)
                  {
                    TIM_Cmd(TIM6, ENABLE);        // 开启定时器6计时2s
                    TIM6flag = 1;                 // 每次只开一次定时器
                  }                                 
                }
                else                              // 如果遮挡不到两秒
                {
                  countNum = 0;                   // 秒数计数清零
                  TIM_SetCounter(TIM6, 0);        // CNT计数清零，方便下次计数
                  TIM_Cmd(TIM6, DISABLE);         // 关闭定时器6
                  TIM6flag = 0;                   // 更新定时器状态
                                    
                  //SystemStatus = Wait;          // 转到等待信号状态
                  //printf("\r 投放完成 \r");                  
                  delay_ms(10); 
                  //Send_To_Upper("send");
                }
                //break;
            //}

//         /*闪烁蓝灯检测是否运行正常*/
//         BLUE_blink();           
    }        
}

/*TIM6定时器中断*/
void TIM6_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM6,TIM_IT_Update);              // 清除更新中断标志位
    countNum ++;
    if(countNum == 200)
    {
      //Send_To_Upper("404");                                 // 如果持续接收电平信号2s，则证明已满载      
      
      countNum = 0;                                         // 秒数计数清零
      
      TIM6flag = 0;
      
      SystemStatus = Wait;                                  // 转到等待信号状态
      //printf("\r 投放完成 \r");
      delay_ms(100);
      
      TIM_SetCounter(TIM6, 0);                              // CNT计数清零，方便下次计数
      TIM_Cmd(TIM6, DISABLE);                               // 关闭定时器      
    }
  }
}

