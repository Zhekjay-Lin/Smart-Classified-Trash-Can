#include "stm32f4xx.h"                  // Device header
#include "Motor_Control.h"
#include "Delay.h"


/*y轴极限220 速度700*/

speedRampData srdx= {STOP,0,0,0,0,0,0}; // X轴 加减速变量
speedRampData srdy= {STOP,0,0,0,0,0,0}; // Y轴 加减速变量
speedRampData srdz= {STOP,0,0,0,0,0,0}; // Z轴 加减速变量
 
uint8_t  motor_sta[3] = {0, 0, 0};     //三轴电机状态

//static uint32_t X_Step_Target = 0;
//static uint32_t Z_Step_Target = 0;
//static uint32_t Y_Step_Target = 0;

/*
step   移动步数（正数为正转，负数为逆时针）
accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
decel  减速度,实际值为decel*0.1*rad/sec^2
speed  最大速度,实际值为speed*0.1*rad/sec
 */
void MOTOR_Move(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed, uint16_t stepnum)
{
		
    uint16_t tim_count; 																										 //存放中断时刻的计数值
    unsigned int max_s_lim;                                     					 	 //达到最大速度时的步数    
    unsigned int accel_lim;																									 //必须开始减速的步数（如果还没有加速度到最大速度时）
 
  
    switch(stepnum)
    {
      case 0:
        if(motor_sta[0]!= STOP)  																									 //只允许步进电机在停止的时候才继续
          return;    				
        if(step < 0)   																													 //逆时针
        {       
            GPIO_SetBits(GPIOB,GPIO_Pin_4);																		   //PA4为方向引脚，输出高电平
            step = -step;
        }		
        else   																																	 //顺时针
        {        
            GPIO_ResetBits(GPIOB,GPIO_Pin_4);          																   		
        }
        //X_Step_Target = step;
        if(step == 1)   																											   // 如果只移动一步
        {       
            srdx.accel_count = -1; 																								 // 只移动一步
        
            srdx.run_state = DECEL;																								 // 减速状态
        
            srdx.step_delay = 1000;																								 // 短延时
 
        }
    
        else if(step != 0)  																			 								// 步数不为零才移动
        { 					
            srdx.min_delay = (int32_t)(A_T_x10/speed);															// 设置最大速度极限, 计算min_delay用于定时器的计数器的值min_delay = (alpha / tt)/ w   
            srdx.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);		// 通过计算第一个(c0) 的步进延时来设定加速度,其中accel单位为0.01rad/sec^2
																																							// step_delay = 1/tt * sqrt(2*alpha/accel)
																																							// step_delay = ( tfreq*0.69/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100 
            max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));								//计算多少步之后达到最大速度的限制 max_s_lim = speed^2 / (2*alpha*accel)
    
            if(max_s_lim == 0)																										//如果达到最大速度小于0.5步，我们将四舍五入为0,但实际我们必须移动至少一步才能达到想要的速度 
            {
               max_s_lim = 1;
            }    
            accel_lim = (uint32_t)(step*decel/(accel+decel)); 										// 计算多少步之后我们必须开始减速,n1 = (n1+n2)decel / (accel + decel)
   
            if(accel_lim == 0) 																										// 我们必须加速至少1步才能开始减速
            {
              accel_lim = 1;
            }
   
            if(accel_lim <= max_s_lim)																						//加速阶段到不了最大速度就得减速。。。使用限制条件我们可以计算出减速阶段步数 
            {
              srdx.decel_val = accel_lim - step;																		//减速段的步数
            }
            else
            {
              srdx.decel_val = -(max_s_lim*accel/decel);														//减速段的步数 
            }
       
            if(srdx.decel_val == 0) 																								// 不足一步 按一步处理 
            {
              srdx.decel_val = -1;
            }    
            srdx.decel_start = step + srdx.decel_val;																//计算开始减速时的步数
     
        
            if(srdx.step_delay <= srdx.min_delay)																		// 如果一开始c0的速度比匀速段速度还大，就不需要进行加速运动，直接进入匀速
            {
              srdx.step_delay = srdx.min_delay;
              srdx.run_state = RUN;
            }
            else
            {
              srdx.run_state = ACCEL;
            }
        
            srdx.accel_count = 0;																									// 复位加速度计数值
        
        }

//        if(step != 0)
//        {
//          srdx.run_state = RUN;
//        }
        
        motor_sta[0] = 1;  																												// 电机为运动状态
        tim_count = TIM_GetCapture1(TIM4);																				//获取计数值
          
        TIM_SetCompare1(TIM4,tim_count + srdx.step_delay/2);												//设置定时器比较值
        //TIM_SetCompare1(TIM4,tim_count + 1000);
        TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);																		//使能定时器通道 
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
              			
			TIM_Cmd(TIM4, ENABLE);																									//开启定时器
}
 
 
 
void speed_decision()                                                                      //中断执行函数
{
	__IO uint32_t tim_count[3]={0, 0, 0};
	__IO uint32_t tmp[3] = {0, 0, 0};  
  uint16_t new_step_delay[3] = {0, 0, 0};                                                  // 保存新（下）一个延时周期  
  __IO static uint16_t last_accel_delay[3] = {0, 0, 0};                                    // 加速过程中最后一次延时（脉冲周期）. 
  __IO static uint32_t step_count[3] = {0, 0, 0};  																			   // 总移动步数计数器  
  __IO static int32_t rest[3] = {0, 0, 0};																								 // 记录new_step_delay中的余数，提高下一步计算的精度  
  __IO static uint8_t i[3] = {0, 0, 0};																										 //定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲
  
  __IO static uint8_t Z_Step_Remain = 0;
  
  /*X轴*/
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1)== SET)
  {	  
		
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);																	// 清楚定时器中断		
	  tim_count[0] = TIM_GetCapture1(TIM4);																					//获取计数值
		tmp[0] = tim_count[0]+srdx.step_delay/2;
    //tmp[0] = tim_count[0]+1000;
	  TIM_SetCompare1(TIM4, tmp[0]);																								// 设置比较值
		i[0]++; 
		if(i[0]==2)																																	//中断两次为一个脉冲
		{
			i[0]=0; 
			switch(srdx.run_state)
			{
				case STOP:																														//停止状态
					step_count[0] = 0;
					rest[0] = 0;
					
					TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
				  TIM_CCxCmd(TIM4,TIM_Channel_1,TIM_CCx_Disable);
					motor_sta[0] = 0;  
					break;
				
				case ACCEL:																														//加速状态
          step_count[0]++;
          srdx.accel_count++;
          new_step_delay[0] = srdx.step_delay - (((2 *srdx.step_delay) + rest[0])/(4 * srdx.accel_count + 1));//计算新(下)一步脉冲周期(时间间隔)
          rest[0] = ((2 * srdx.step_delay)+rest[0])%(4 * srdx.accel_count + 1);					// 计算余数，下次计算补上余数，减少误差
				
					if(step_count[0] >= srdx.decel_start) 																	//检查是够应该开始减速
					{
						srdx.accel_count = srdx.decel_val;																	//加速计数值为减速阶段计数值的初始值
						srdx.run_state = DECEL;																						//下个脉冲进入减速阶段 
					}
					
					else if(new_step_delay[0] <= srdx.min_delay)														//检查是否到达期望的最大速度
					{
						last_accel_delay[0] = new_step_delay[0];																//保存加速过程中最后一次延时（脉冲周期）
						new_step_delay[0] = srdx.min_delay;   																// 使用min_delay（对应最大速度speed） 
						rest[0] = 0;            																							//清零余值               
						srdx.run_state = RUN;																							//设置为匀速运行状态 
					}
					break;
					
				case RUN:
          step_count[0]++;  																											// 步数加1				  
          new_step_delay[0] = srdx.min_delay;   																  // 使用min_delay（对应最大速度speed）				 
          if(step_count[0] >= srdx.decel_start)   																// 需要开始减速
					{
            srdx.accel_count = srdx.decel_val;  																// 减速步数做为加速计数值
            new_step_delay[0] = last_accel_delay[0];																// 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
            srdx.run_state = DECEL;           																  // 状态改变为减速
          }
//          if(step_count[0] == X_Step_Target)
//          {            
//            srdx.run_state = STOP;
//            delay_ms(10);
//          }
          break;

					
				case DECEL:
          step_count[0]++;  																											// 步数加1
 
          srdx.accel_count++; 																									// 是个负数
          new_step_delay[0] = srdx.step_delay - (((2 * srdx.step_delay) + rest[0])/(4 * srdx.accel_count + 1)); //计算新(下)一步脉冲周期(时间间隔)
          rest[0] = ((2 * srdx.step_delay)+rest[0])%(4 * srdx.accel_count + 1);				// 计算余数，下次计算补上余数，减少误差
          if(srdx.accel_count >= 0) 																						//检查是否为最后一步  是个负数所以要判断 大于等于零时 应该就是减速结束
          {
            srdx.run_state = STOP;
          }
          break;
			}
			 srdx.step_delay = new_step_delay[0]; 																			// 为下个(新的)延时(脉冲周期)赋值
		}
	}
  
  /*Y轴*/
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
  
  /*Z轴*/
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
 
 
 
