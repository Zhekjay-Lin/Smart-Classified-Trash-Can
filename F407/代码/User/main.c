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


/*ϵͳ״̬�궨��*/
#define       Wait          0       // �ȴ��ź�
#define       Judge         1       // �ж����
#define       Motor         2       // ��̨�˶�
#define       Claw          3       // ��צ�˶�
#define       Compress      4       // �Ƹ��˶�
#define       Test          5       // ���ؼ��

/*�������궨��*/
#define       Recycle       1       // �ɻ�������
#define       Harm          3       // �к�����
#define       Kitchen       2       // ��������
#define       Dry           4       // ������������������

/*��צ״̬�궨��*/
#define       GrabGo        1       // ��ʼץȡ
#define       Release       2       // �ͷ�
#define       GrabBack      3       // ץ��س�

/*����״̬�궨��*/
#define       Open          1       // ��
#define       Close         2       // �ر�



/*����ϵͳ��ǰ״̬*/
uint8_t SystemStatus = Wait;

/*���浱ǰʶ����������*/
uint8_t Type;

/*�����צ������ſ��Ƕ�*/
int32_t wristAngle;
int32_t clawAngle;
uint8_t clawpower;

/*���浱ǰ����״̬*/
uint8_t DoorStatus;

/*�����翪��״̬�������������Ͷ�ʱ��6״̬*/
uint8_t switchstatus = 0;
uint32_t countNum = 0;
uint8_t TIM6flag = 0;

/*��צ�ṹ��*/
typedef struct
{
  uint8_t ClawStatus;       // ��צ״̬
  int32_t wristTarget;      // ��צĿ�귽��
  int32_t angleTarget;      // ��צĿ���ſ��Ƕ�  
  //uint8_t power;          // �Ƿ���Ҫ��������
}ClawStruct;

typedef struct
{
  uint32_t SetSpeed;        // ����ٶ�    ��λΪ0.1rad/sec
  uint32_t SetAccel;        // ���ٶ�����  ��λΪ0.1rad/sec^2
  uint32_t SetDecel;        // ���ٶ�����  ��λΪ0.1rad/sec^2
  
  int32_t LocaTarget;       // Ŀ��λ��
  int32_t LocaCurrent;      // ��ǰλ��
  
  int32_t stepTarget;       // Ŀ��������
  int32_t stepBack;         // �س�������
  uint8_t flag;             // ����״̬   
}Slide;



int main(void)
{
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                      // ����ϵͳ�ж����ȼ�����2
                                                                  
    Led_Init();
    delay_init(168);                                                    // ��ʱ������ϵͳʱ�ӳ�ʼ��
      
    Motor_Init();  
    LinearActuator_Init();
  
    claw_servo_Init();
        
    UART1_Init(115200);
    UART3_Init(115200);
    UART4_Init(115200);
    
    delay_ms(1000);                                                    // �ȴ�ϵͳ�������
    
    ClawStruct claw ={GrabGo, 90, 140};                             // ��ʼ����צ״̬
    Slide X_Slide = {20000,18000,18000,0,0,0,0,1};                   // ��ʼ���������
    Slide Y_Slide = {15000,12000,12000,0,0,0,0,1};
    Slide Z_Slide = {22000,18000,18000,0,0,0,0,0};

    

    while (1)                                                            
    {  
      switch(SystemStatus)
      {
          /*�����Ƽ�צĿ��Ƕ�*/
          case Wait:                           
              uart_debugfunction();                              // û�л�ȡ�������ֽ�����һֱѭ���ȴ����ֽ�������
              if(UART4_GetRxNum())
              {                                                     // ��ȡ�����ֽ�������ʼ���           
                 uint16_t  usRxNum = UART4_GetRxNum();                // ��ȡ���ڽ������һ֡���ֽ���
                 uint8_t  *puRxData = UART4_GetRxData();              // ��ȡ���ڽ������һ֡�����ݵĵ�ַ

                 UART4_ClearRx();                                      // ������ֵ���������
                                                                        
                  /*���ݲ�ֲ�ת��Ϊ��ֵ*/
                 X_Slide.LocaTarget = -String2int(puRxData, usRxNum, 0);         // ����X��Ŀ��λ��
                
                 Y_Slide.LocaTarget = String2int(puRxData, usRxNum, 1);          // ����Y��Ŀ��λ��
                
                 claw.wristTarget = String2int(puRxData, usRxNum, 2);            // ���¼�צ����
                 claw.angleTarget = String2int(puRxData, usRxNum, 3) + 5;        // ���¼�צ�ſ��Ƕ�
                 Type = String2int(puRxData, usRxNum, 4);                        // �����������
                               
                 X_Slide.stepTarget = Calulate_Step(X_Slide.LocaTarget, X_Slide.LocaCurrent) + 500;  // ����XĿ��������
                 Y_Slide.stepTarget = Calulate_Step(Y_Slide.LocaTarget, Y_Slide.LocaCurrent);        // ����YĿ��������
                
                 /*��λ�ж�*/
                 if(X_Slide.stepTarget <= 0)
                 {
                   X_Slide.stepTarget = 0;
                   Xmin = 1;                                                // X����С���ޱ�־��1
                 }
                 else if(X_Slide.stepTarget >= 1300 )                 
                 {
                   X_Slide.stepTarget = 1300;
                 }                
                  
                 //Send_To_Upper("OK");                                     // �յ�ָ������λ������
                 
                 //printf("\r X��Ŀ�꣺%d \rY��Ŀ�꣺%d \r��צ����%d \r ��צ�ſ��Ƕȣ�%d \r�������%d \rX��YĿ�����壺%d��%d", X_Slide.LocaTarget, Y_Slide.LocaTarget, claw.wristTarget, claw.angleTarget, Type, X_Slide.stepTarget, Y_Slide.stepTarget);                                                                             
                 SystemStatus = Judge;                                // ��ת����һ״̬              
              }
              break;
          
            case Judge:                                  
               Gimbal_Servo(Type);                                  // �����������ת����̨
//               if(Type == Recycle)                                  // ���µ���״̬ �ɻ��գ���رյ���
//                  DoorStatus = Close;
//               else
                 DoorStatus = Open;
               
               Door_Servo(DoorStatus);                              // �ƶ�����
               SystemStatus = Motor;                                 
               delay_ms(500);               // �ȴ���״̬�������               
               break;
                        
            case Motor:
                Motor_EN();                                         // �������ʹ��             
                if(claw.ClawStatus == Release)
                {
                  Wrist_ServoMove(90);                              // �����˶���ͬʱ��צ����
                }             
                if(X_Slide.flag == 1)                                    // ֻ�ں���״̬Ϊ1�������ʹ��Motor_Move����                
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
                    MOTOR_Move(-500, X_Slide.SetAccel, X_Slide.SetDecel, X_Slide.SetSpeed, X);     // X�����
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
                  if(Z_Slide.flag && (claw.ClawStatus == GrabBack))  // һ������������ֻ����ڶ��ν���Motor״̬��Z�س�ֵ
                  {
                    Z_Slide.stepBack = Z_Slide.stepTarget;
                  }
                  Z_Slide.stepTarget = 0;
                  Z_Slide.flag = 0;
                }                                
                
                if(srdx.run_state == STOP && srdy.run_state == STOP && srdz.run_state == STOP)
                {
                   
                   Motor_DIS();                                       // ÿ���˳�����˶�״̬������ʧ�ܣ���Сϵͳ����
                   
                   //printf("\r XSTEP��%d YSTEP��%d ZSTEP��%d", X_Slide.stepBack, Y_Slide.stepBack, Z_Slide.stepBack);
                  
                   if(XbackStatus == 1)                 // ��ʱΪ���һ�ν���motor״̬
                   {
//                         if(Type == Recycle)            // ����ǿɻ�������
//                         {
//                          SystemStatus = Compress;     // ������ѹ��״̬
//                          DoorStatus = Open;           // ���µ���״̬�������½׶ε����˶�
//                         }
//                         else
//                         {
                           SystemStatus = Wait;         // ���ǿɻ���������ֱ�ӽ�����״̬
                           //Send_To_Upper("send");
//                         }
                           
                      
                         claw.ClawStatus = GrabGo;      // һ��������ɣ���צ�ָ�����ȡ״̬
                         X_Slide.flag = 1;              // ������־��һ��׼����һ��������ִ��X��Y�˶�
                         Y_Slide.flag = 1;
                          
                         X_Slide.stepBack = 0;
                         Y_Slide.stepBack = 0;
                         Z_Slide.stepBack = 0;
                         
                         XbackStatus = 0;
                         Xmin = 0;                       // ���X����С���ޱ�־
                   }
                   else                                                 // �������һ�ν���motor״̬
                     SystemStatus = Claw;                               // �����צ�˶�״̬
                }
                                                        
                break;
 
              case Claw: 
                delay_ms(1);
                if(claw.ClawStatus == GrabGo)
                {
                  Wrist_ServoMove(claw.wristTarget);               // ��צת����Ӧ����
                  delay_ms(100);
                  claw_servo_setAngle(claw.angleTarget);                        // ��צ�ſ�׼��ץȡ
                  delay_ms(200);
                  
                  Z_Slide.stepTarget = -200*5;                     // ����Z�Ჽ����5Ȧ
                  Z_Slide.flag = 1;                                // ������־��һ��׼���´�ִ��Z���˶�
                  claw.ClawStatus = GrabBack;
                  SystemStatus = Motor;
                  //printf("\r 1");                
                }
                else if(claw.ClawStatus == GrabBack)
                {
                  claw_servo_setAngle(4);                           // ��צ��ס����
//                  for(int i=claw.angleTarget;i>=0;i--)
//                  {
//                    claw_servo_setAngle(i);
//                    //delay_ms(1);
//                    if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 1)
//                    {
//                      printf("\r �Ƕ�Ϊ��%d", i);
//                      printf("\r ��ť����");
//                      break;
//                    }                            
//                  }
                  
                  delay_ms(500);                  
                  //claw.power = 1;                                 // ��צ��������
                  
                  X_Slide.flag = 1;                                // ������־��һ��׼���´�ִ�������˶�
                  Y_Slide.flag = 1;
                  Z_Slide.flag = 1;
                  delay_ms(1);
                  
                  
                  X_Slide.stepTarget = -X_Slide.stepBack;                       // ������Ļس̲�����ΪĿ�경��                  
                  Y_Slide.stepTarget = -Y_Slide.stepBack;
                  Z_Slide.stepTarget = -Z_Slide.stepBack;
                  //printf("\r %d", X_Slide.stepTarget);
                  
                  claw.ClawStatus = Release;                       // ���¼�צ��һ״̬
                  SystemStatus = Motor;                            // ����Motor״̬ ��̨�س�
                  //printf("\r 2");                                  
                }
                else if(claw.ClawStatus == Release)
                {                                   
                  //claw.power = 0;
                  claw_servo_setAngle(claw.angleTarget + 5);      // �ͷ�
                  delay_ms(500);
                  claw_servo_home();
                  
                  XbackStatus = 1;                          // ���һ�ν���Motor״̬
                  SystemStatus = Motor;
                                 
                  
//                  if(Type == Recycle)            // ����ǿɻ�������
//                  {
//                    SystemStatus = Compress;     // ������ѹ��״̬
//                    DoorStatus = Open;           // ���µ���״̬�������½׶ε����˶�
//                  }
//                  else
//                    SystemStatus = Wait;         // ���ǿɻ���������ֱ�ӽ�����״̬
//                  
//                  claw.ClawStatus = GrabGo;      // һ��������ɣ���צ�ָ�����ȡ״̬
//                  X_Slide.flag = 1;              // ������־��һ��׼����һ��������ִ��X��Y�˶�
//                  Y_Slide.flag = 1;
//                  
//                  X_Slide.stepBack = 0;
//                  Y_Slide.stepBack = 0;
//                  Z_Slide.stepBack = 0;
                  
                  //printf("\r 3");
                }
                break;
                
//              case Compress:
//                /*��������ѧ������*/
//                delay_ms(200);                 // �ȴ���������
//                  
//                LinearAcuator(1);              // �Ƹ˵���˶�
//                LinearAcuator(0);
//                LinearAcuator(2);
//                LinearAcuator(0);                        
//                
//                Door_Servo(DoorStatus);        // ����
//                
//                delay_ms(100);                 // �ȴ��������
//                    
//                SystemStatus = Wait;
//                //Send_To_Upper("send");
//                break;
            } 
              //case Test:                
                switchstatus = SwitchStatus();  // ÿ��ѭ����ɨ��һ�κ��⿪��״̬
                if(switchstatus)                // ����ڵ�
                {
                  if(TIM6flag == 0)
                  {
                    TIM_Cmd(TIM6, ENABLE);        // ������ʱ��6��ʱ2s
                    TIM6flag = 1;                 // ÿ��ֻ��һ�ζ�ʱ��
                  }                                 
                }
                else                              // ����ڵ���������
                {
                  countNum = 0;                   // ������������
                  TIM_SetCounter(TIM6, 0);        // CNT�������㣬�����´μ���
                  TIM_Cmd(TIM6, DISABLE);         // �رն�ʱ��6
                  TIM6flag = 0;                   // ���¶�ʱ��״̬
                                    
                  //SystemStatus = Wait;          // ת���ȴ��ź�״̬
                  //printf("\r Ͷ����� \r");                  
                  delay_ms(10); 
                  //Send_To_Upper("send");
                }
                //break;
            //}

//         /*��˸���Ƽ���Ƿ���������*/
//         BLUE_blink();           
    }        
}

/*TIM6��ʱ���ж�*/
void TIM6_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM6,TIM_IT_Update);              // ��������жϱ�־λ
    countNum ++;
    if(countNum == 200)
    {
      //Send_To_Upper("404");                                 // ����������յ�ƽ�ź�2s����֤��������      
      
      countNum = 0;                                         // ������������
      
      TIM6flag = 0;
      
      SystemStatus = Wait;                                  // ת���ȴ��ź�״̬
      //printf("\r Ͷ����� \r");
      delay_ms(100);
      
      TIM_SetCounter(TIM6, 0);                              // CNT�������㣬�����´μ���
      TIM_Cmd(TIM6, DISABLE);                               // �رն�ʱ��      
    }
  }
}

