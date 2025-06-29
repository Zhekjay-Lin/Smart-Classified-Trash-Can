#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "math.h"
#include "stm32f4xx.h"

typedef struct
{
	 uint8_t run_state;    //�����ת״̬
	 uint8_t dir ;         //�����ת����
	 int step_delay;       //�¸��������ڣ�ʱ����������ʱΪ���ٶ�
	 int decel_start;      //��������λ��
	 int decel_val;        //���ٽ׶β���
	 int min_delay;        //��С�������ڣ�����ٶȣ������ٶε��ٶȣ�
	 int accel_count;      //���ٽ׶μ���ֵ
}speedRampData;

#define TRUE     1
#define FALSE    0
 
 
/*����ٶȾ����е��ĸ�״̬*/
#define STOP              0 // ֹͣ״̬
#define ACCEL             1 // ����״̬
#define DECEL             2 // ����״̬
#define RUN               3 // ����״̬

/*����*/
#define       X       0
#define       Y       1
#define       Z       2 

/*Z��Ƚ�ֵ*/
#define    Z_Move          1   //Z���ƶ�
 
 
#define TIM_PRESCALER      83
#define T1_FREQ            (SystemCoreClock/(TIM_PRESCALER+1))     //��ʱ��Ƶ��
 
/*�����Ȧ����*/
#define STEP_ANGLE				1.8									//��������Ĳ���� 
#define FSPR              200                 //���������Ȧ������
 
#define MICRO_STEP        1         				//ϸ����ϸ���� 
#define SPR               (FSPR*MICRO_STEP)  //1ϸ�ֵĲ��� 200������һȦ
 
//��ѧ����������MSD_MOVE�����ļ򻯼���
#define ALPHA             ((float)(2*3.14159/SPR))       // ��= 2*pi/spr    
#define A_T_x10           ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148       ((float)((T1_FREQ*0.676)/10)) // 0.69Ϊ�������ֵ(������̣��ĵ�����д)
#define A_SQ              ((float)(2*100000*ALPHA)) 
#define A_x200            ((float)(200*ALPHA))
   
void MOTOR_Move(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed, uint16_t stepnum);				
extern void speed_decision(void);


#endif

