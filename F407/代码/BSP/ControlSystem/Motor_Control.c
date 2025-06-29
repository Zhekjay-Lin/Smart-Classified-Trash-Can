#include "stm32f4xx.h"                  // Device header
#include "Motor_Control.h"
#include "Delay.h"


/*y�Ἣ��220 �ٶ�700*/

speedRampData srdx= {STOP,0,0,0,0,0,0}; // X�� �Ӽ��ٱ���
speedRampData srdy= {STOP,0,0,0,0,0,0}; // Y�� �Ӽ��ٱ���
speedRampData srdz= {STOP,0,0,0,0,0,0}; // Z�� �Ӽ��ٱ���
 
uint8_t  motor_sta[3] = {0, 0, 0};     //������״̬

//static uint32_t X_Step_Target = 0;
//static uint32_t Z_Step_Target = 0;
//static uint32_t Y_Step_Target = 0;

/*
step   �ƶ�����������Ϊ��ת������Ϊ��ʱ�룩
accel  ���ٶ�,ʵ��ֵΪaccel*0.1*rad/sec^2  10������2��������һ������������
decel  ���ٶ�,ʵ��ֵΪdecel*0.1*rad/sec^2
speed  ����ٶ�,ʵ��ֵΪspeed*0.1*rad/sec
 */
void MOTOR_Move(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed, uint16_t stepnum)
{
		
    uint16_t tim_count; 																										 //����ж�ʱ�̵ļ���ֵ
    unsigned int max_s_lim;                                     					 	 //�ﵽ����ٶ�ʱ�Ĳ���    
    unsigned int accel_lim;																									 //���뿪ʼ���ٵĲ����������û�м��ٶȵ�����ٶ�ʱ��
 
  
    switch(stepnum)
    {
      case 0:
        if(motor_sta[0]!= STOP)  																									 //ֻ�����������ֹͣ��ʱ��ż���
          return;    				
        if(step < 0)   																													 //��ʱ��
        {       
            GPIO_SetBits(GPIOB,GPIO_Pin_4);																		   //PA4Ϊ�������ţ�����ߵ�ƽ
            step = -step;
        }		
        else   																																	 //˳ʱ��
        {        
            GPIO_ResetBits(GPIOB,GPIO_Pin_4);          																   		
        }
        //X_Step_Target = step;
        if(step == 1)   																											   // ���ֻ�ƶ�һ��
        {       
            srdx.accel_count = -1; 																								 // ֻ�ƶ�һ��
        
            srdx.run_state = DECEL;																								 // ����״̬
        
            srdx.step_delay = 1000;																								 // ����ʱ
 
        }
    
        else if(step != 0)  																			 								// ������Ϊ����ƶ�
        { 					
            srdx.min_delay = (int32_t)(A_T_x10/speed);															// ��������ٶȼ���, ����min_delay���ڶ�ʱ���ļ�������ֵmin_delay = (alpha / tt)/ w   
            srdx.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);		// ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶ�,����accel��λΪ0.01rad/sec^2
																																							// step_delay = 1/tt * sqrt(2*alpha/accel)
																																							// step_delay = ( tfreq*0.69/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100 
            max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));								//������ٲ�֮��ﵽ����ٶȵ����� max_s_lim = speed^2 / (2*alpha*accel)
    
            if(max_s_lim == 0)																										//����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0,��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ� 
            {
               max_s_lim = 1;
            }    
            accel_lim = (uint32_t)(step*decel/(accel+decel)); 										// ������ٲ�֮�����Ǳ��뿪ʼ����,n1 = (n1+n2)decel / (accel + decel)
   
            if(accel_lim == 0) 																										// ���Ǳ����������1�����ܿ�ʼ����
            {
              accel_lim = 1;
            }
   
            if(accel_lim <= max_s_lim)																						//���ٽ׶ε���������ٶȾ͵ü��١�����ʹ�������������ǿ��Լ�������ٽ׶β��� 
            {
              srdx.decel_val = accel_lim - step;																		//���ٶεĲ���
            }
            else
            {
              srdx.decel_val = -(max_s_lim*accel/decel);														//���ٶεĲ��� 
            }
       
            if(srdx.decel_val == 0) 																								// ����һ�� ��һ������ 
            {
              srdx.decel_val = -1;
            }    
            srdx.decel_start = step + srdx.decel_val;																//���㿪ʼ����ʱ�Ĳ���
     
        
            if(srdx.step_delay <= srdx.min_delay)																		// ���һ��ʼc0���ٶȱ����ٶ��ٶȻ��󣬾Ͳ���Ҫ���м����˶���ֱ�ӽ�������
            {
              srdx.step_delay = srdx.min_delay;
              srdx.run_state = RUN;
            }
            else
            {
              srdx.run_state = ACCEL;
            }
        
            srdx.accel_count = 0;																									// ��λ���ٶȼ���ֵ
        
        }

//        if(step != 0)
//        {
//          srdx.run_state = RUN;
//        }
        
        motor_sta[0] = 1;  																												// ���Ϊ�˶�״̬
        tim_count = TIM_GetCapture1(TIM4);																				//��ȡ����ֵ
          
        TIM_SetCompare1(TIM4,tim_count + srdx.step_delay/2);												//���ö�ʱ���Ƚ�ֵ
        //TIM_SetCompare1(TIM4,tim_count + 1000);
        TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);																		//ʹ�ܶ�ʱ��ͨ�� 
        TIM_CCxCmd(TIM4,TIM_Channel_1,TIM_CCx_Enable);
        break;
      
      case 1:
        if(motor_sta[1]!= STOP)  																									 
          return;    				
        if(step < 0)   																													 
        {       
            GPIO_SetBits(GPIOB,GPIO_Pin_5);																		   
            step = -step;
        }		
        else   																																	 
        {        
            GPIO_ResetBits(GPIOB,GPIO_Pin_5);          																   		
        }
        //Y_Step_Target = step;
        
        if(step == 1)   																											   
        {       
            srdy.accel_count = -1; 																								 
        
            srdy.run_state = DECEL;																								 
        
            srdy.step_delay = 1000;																								 
 
        }
    
        else if(step != 0)  																			 								
        { 					
            srdy.min_delay = (int32_t)(A_T_x10/speed);															
            srdy.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);		
																																							
																																							 
            max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));								
    
            if(max_s_lim == 0)																										 
            {
               max_s_lim = 1;
            }    
            accel_lim = (uint32_t)(step*decel/(accel+decel)); 										
   
            if(accel_lim == 0) 																										
            {
              accel_lim = 1;
            }
   
            if(accel_lim <= max_s_lim)																						 
            {
              srdy.decel_val = accel_lim - step;																		
            }
            else
            {
              srdy.decel_val = -(max_s_lim*accel/decel);														 
            }
       
            if(srdy.decel_val == 0) 																								 
            {
              srdy.decel_val = -1;
            }    
            srdy.decel_start = step + srdy.decel_val;																
     
        
            if(srdy.step_delay <= srdy.min_delay)																		
            {
              srdy.step_delay = srdy.min_delay;
              srdy.run_state = RUN;
            }
            else
            {
              srdy.run_state = ACCEL;
            }
        
            srdy.accel_count = 0;																									
        
        }

//        if(step != 0)
//        {
//          srdy.run_state = RUN;
//        }
        
        motor_sta[1] = 1;  																												
        tim_count = TIM_GetCapture2(TIM4);																				
          
        TIM_SetCompare2(TIM4,tim_count + srdy.step_delay/2);												 
        TIM_ITConfig(TIM4,TIM_IT_CC2,ENABLE);																		 
        TIM_CCxCmd(TIM4,TIM_Channel_2,TIM_CCx_Enable);
        break;
        
      case 2:
        if(motor_sta[2]!= STOP)  																									 
          return;    				
        if(step < 0)   																													 
        {       
            GPIO_SetBits(GPIOB,GPIO_Pin_9);																		   
            step = -step;
        }		
        else   																																	 
        {        
            GPIO_ResetBits(GPIOB,GPIO_Pin_9);          																   		
        }

        //Z_Step_Target = step;
        if(step == 1)   																											   
        {       
            srdz.accel_count = -1; 																								 
        
            srdz.run_state = DECEL;																								 
        
            srdz.step_delay = 1000;																								 
 
        }
    
        else if(step != 0)  																			 								
        { 					
            srdz.min_delay = (int32_t)(A_T_x10/speed);															   
            srdz.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);		
																																							
																																							 
            max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));								
    
            if(max_s_lim == 0)																										 
            {
               max_s_lim = 1;
            }    
            accel_lim = (uint32_t)(step*decel/(accel+decel)); 										
   
            if(accel_lim == 0) 																										
            {
              accel_lim = 1;
            }
   
            if(accel_lim <= max_s_lim)																						
            {
              srdz.decel_val = accel_lim - step;																		
            }
            else
            {
              srdz.decel_val = -(max_s_lim*accel/decel);														 
            }
       
            if(srdz.decel_val == 0) 																								 
            {
              srdz.decel_val = -1;
            }    
            srdz.decel_start = step + srdz.decel_val;																
     
        
            if(srdz.step_delay <= srdz.min_delay)																		
            {
              srdz.step_delay = srdz.min_delay;
              srdz.run_state = RUN;
            }
            else
            {
              srdz.run_state = ACCEL;
            }
        
            srdz.accel_count = 0;																									
        
        }
        
//        if(step != 0)
//        {
//          srdz.run_state = RUN;
//        }
        motor_sta[2] = 1;  																												
        tim_count = TIM_GetCapture3(TIM4);																				
          
        TIM_SetCompare4(TIM4,tim_count + srdz.step_delay/2);												
        TIM_ITConfig(TIM4,TIM_IT_CC3,ENABLE);																		 
        TIM_CCxCmd(TIM4,TIM_Channel_3,TIM_CCx_Enable);
        break;
    }
              			
			TIM_Cmd(TIM4, ENABLE);																									//������ʱ��
}
 
 
 
void speed_decision()                                                                      //�ж�ִ�к���
{
	__IO uint32_t tim_count[3]={0, 0, 0};
	__IO uint32_t tmp[3] = {0, 0, 0};  
  uint16_t new_step_delay[3] = {0, 0, 0};                                                  // �����£��£�һ����ʱ����  
  __IO static uint16_t last_accel_delay[3] = {0, 0, 0};                                    // ���ٹ��������һ����ʱ���������ڣ�. 
  __IO static uint32_t step_count[3] = {0, 0, 0};  																			   // ���ƶ�����������  
  __IO static int32_t rest[3] = {0, 0, 0};																								 // ��¼new_step_delay�е������������һ������ľ���  
  __IO static uint8_t i[3] = {0, 0, 0};																										 //��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������
  
  __IO static uint8_t Z_Step_Remain = 0;
  
  /*X��*/
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1)== SET)
  {	  
		
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);																	// �����ʱ���ж�		
	  tim_count[0] = TIM_GetCapture1(TIM4);																					//��ȡ����ֵ
		tmp[0] = tim_count[0]+srdx.step_delay/2;
    //tmp[0] = tim_count[0]+1000;
	  TIM_SetCompare1(TIM4, tmp[0]);																								// ���ñȽ�ֵ
		i[0]++; 
		if(i[0]==2)																																	//�ж�����Ϊһ������
		{
			i[0]=0; 
			switch(srdx.run_state)
			{
				case STOP:																														//ֹͣ״̬
					step_count[0] = 0;
					rest[0] = 0;
					
					TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
				  TIM_CCxCmd(TIM4,TIM_Channel_1,TIM_CCx_Disable);
					motor_sta[0] = 0;  
					break;
				
				case ACCEL:																														//����״̬
          step_count[0]++;
          srdx.accel_count++;
          new_step_delay[0] = srdx.step_delay - (((2 *srdx.step_delay) + rest[0])/(4 * srdx.accel_count + 1));//������(��)һ����������(ʱ����)
          rest[0] = ((2 * srdx.step_delay)+rest[0])%(4 * srdx.accel_count + 1);					// �����������´μ��㲹���������������
				
					if(step_count[0] >= srdx.decel_start) 																	//����ǹ�Ӧ�ÿ�ʼ����
					{
						srdx.accel_count = srdx.decel_val;																	//���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
						srdx.run_state = DECEL;																						//�¸����������ٽ׶� 
					}
					
					else if(new_step_delay[0] <= srdx.min_delay)														//����Ƿ񵽴�����������ٶ�
					{
						last_accel_delay[0] = new_step_delay[0];																//������ٹ��������һ����ʱ���������ڣ�
						new_step_delay[0] = srdx.min_delay;   																// ʹ��min_delay����Ӧ����ٶ�speed�� 
						rest[0] = 0;            																							//������ֵ               
						srdx.run_state = RUN;																							//����Ϊ��������״̬ 
					}
					break;
					
				case RUN:
          step_count[0]++;  																											// ������1				  
          new_step_delay[0] = srdx.min_delay;   																  // ʹ��min_delay����Ӧ����ٶ�speed��				 
          if(step_count[0] >= srdx.decel_start)   																// ��Ҫ��ʼ����
					{
            srdx.accel_count = srdx.decel_val;  																// ���ٲ�����Ϊ���ټ���ֵ
            new_step_delay[0] = last_accel_delay[0];																// �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
            srdx.run_state = DECEL;           																  // ״̬�ı�Ϊ����
          }
//          if(step_count[0] == X_Step_Target)
//          {            
//            srdx.run_state = STOP;
//            delay_ms(10);
//          }
          break;

					
				case DECEL:
          step_count[0]++;  																											// ������1
 
          srdx.accel_count++; 																									// �Ǹ�����
          new_step_delay[0] = srdx.step_delay - (((2 * srdx.step_delay) + rest[0])/(4 * srdx.accel_count + 1)); //������(��)һ����������(ʱ����)
          rest[0] = ((2 * srdx.step_delay)+rest[0])%(4 * srdx.accel_count + 1);				// �����������´μ��㲹���������������
          if(srdx.accel_count >= 0) 																						//����Ƿ�Ϊ���һ��  �Ǹ���������Ҫ�ж� ���ڵ�����ʱ Ӧ�þ��Ǽ��ٽ���
          {
            srdx.run_state = STOP;
          }
          break;
			}
			 srdx.step_delay = new_step_delay[0]; 																			// Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
		}
	}
  
  /*Y��*/
  if (TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET)
  {	  
		
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);																			
	  tim_count[1] = TIM_GetCapture2(TIM4);																					
		tmp[1] = tim_count[1] + srdy.step_delay/2;
	  TIM_SetCompare2(TIM4, tmp[1]);																								
		i[1]++; 
		if(i[1]==2)																																	
		{
			i[1]=0; 
			switch(srdy.run_state)
			{
				case STOP:																														
					step_count[1] = 0;
					rest[1] = 0;
					
					TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
				  TIM_CCxCmd(TIM4,TIM_Channel_2,TIM_CCx_Disable);				  
					motor_sta[1] = 0;  
					break;
				
				case ACCEL:																														
          step_count[1]++;
          srdy.accel_count++;
          new_step_delay[1] = srdy.step_delay - (((2 *srdy.step_delay) + rest[1])/(4 * srdy.accel_count + 1));
          rest[1] = ((2 * srdy.step_delay)+rest[1])%(4 * srdy.accel_count + 1);					
				
					if(step_count[1] >= srdy.decel_start) 																	
					{
						srdy.accel_count = srdy.decel_val;																	
						srdy.run_state = DECEL;																						 
					}
					
					else if(new_step_delay[1] <= srdy.min_delay)														
					{
						last_accel_delay[1] = new_step_delay[1];																
						new_step_delay[1] = srdy.min_delay;   																 
						rest[1] = 0;            																							               
						srdy.run_state = RUN;																							
					}
					break;
					
				case RUN:
          step_count[1]++;  																														  
          new_step_delay[1] = srdy.min_delay;   																  				 
          if(step_count[1] >= srdy.decel_start)   																
					{
            srdy.accel_count = srdy.decel_val;  																
            new_step_delay[1] = last_accel_delay[1];																
            srdy.run_state = DECEL;           																  
          }
          
//          if(step_count[1] == Y_Step_Target)
//          {
//            srdy.run_state = STOP;
//          }
          break;
					
				case DECEL:
          step_count[1]++;  																											
 
          srdy.accel_count++; 																									
          new_step_delay[1] = srdy.step_delay - (((2 * srdy.step_delay) + rest[1])/(4 * srdy.accel_count + 1)); 
          rest[1] = ((2 * srdy.step_delay)+rest[1])%(4 * srdy.accel_count + 1);				
          if(srdy.accel_count >= 0) 																						
          {
            srdy.run_state = STOP;
          }
          break;
			}
			 srdy.step_delay = new_step_delay[1]; 																			
		}
	}
  
  /*Z��*/
  if (TIM_GetITStatus(TIM4, TIM_IT_CC3)== SET)
  {	  
		
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);																			
	  tim_count[2] = TIM_GetCapture3(TIM4);																					
		tmp[2] = tim_count[2] + srdz.step_delay/2;
	  TIM_SetCompare3(TIM4, tmp[2]);																								
		i[2]++; 
		if(i[2]==2)																																	
		{
			i[2]=0;            
			switch(srdz.run_state)
			{
				case STOP:																														
					step_count[2] = 0;
					rest[2] = 0;
					
					TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
				  TIM_CCxCmd(TIM4,TIM_Channel_3,TIM_CCx_Disable);
					motor_sta[2] = 0;                
					break;
				
				case ACCEL:																														
          step_count[2]++;
          srdz.accel_count++;
          new_step_delay[2] = srdz.step_delay - (((2 *srdz.step_delay) + rest[2])/(4 * srdz.accel_count + 1));
          rest[2] = ((2 * srdz.step_delay)+rest[2])%(4 * srdz.accel_count + 1);					
				
					if(step_count[2] >= srdz.decel_start) 																	
					{
						srdz.accel_count = srdz.decel_val;																	
						srdz.run_state = DECEL;																						 
					}
					
					else if(new_step_delay[2] <= srdz.min_delay)														
					{
						last_accel_delay[2] = new_step_delay[2];																
						new_step_delay[2] = srdz.min_delay;   																 
						rest[2] = 0;            																							               
						srdz.run_state = RUN;																							
					}
					break;
					
				case RUN:
          step_count[2]++;  																															  
          new_step_delay[2] = srdz.min_delay;   																  				 
          if(step_count[2] >= srdz.decel_start)   																
					{
            srdz.accel_count = srdz.decel_val;  																
            new_step_delay[2] = last_accel_delay[2];																
            srdz.run_state = DECEL;           																  
          }
//          if(step_count[2] == Z_Step_Target)
//          {
//            srdz.run_state = STOP;
//          }          
          break;
					
				case DECEL:
          step_count[2]++;  																											
 
          srdz.accel_count++; 																									
          new_step_delay[2] = srdz.step_delay - (((2 * srdz.step_delay) + rest[2])/(4 * srdz.accel_count + 1)); 
          rest[2] = ((2 * srdz.step_delay)+rest[2])%(4 * srdz.accel_count + 1);				
          if(srdz.accel_count >= 0) 																						
          {
            srdz.run_state = STOP;
          }
          break;
			}
			 srdz.step_delay = new_step_delay[2]; 																			
		}
	}
}
	
 
void TIM4_IRQHandler(void)
{
	speed_decision();
}
 
 
 
