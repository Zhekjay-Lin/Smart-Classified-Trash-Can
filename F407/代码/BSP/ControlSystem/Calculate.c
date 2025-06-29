#include "Calculate.h"

/*���ݷָ����ֵת��*/
/*���굥λ��Ϊmm*/
int String2int(uint8_t *inputdata, uint16_t inputdataNum, uint8_t task)
{
  int data=0;    
  if(task == 0)
  {
    char xloca[5] = "0";
    for(uint16_t i=0;i<4;i++)
    {
      xloca[i]=inputdata[i];
    }
    data = atoi(xloca);
  }
  if(task == 1)
  {
    char yloca[5] = "0";
    for(uint16_t i=4;i<8;i++)
    {    
      yloca[i-4]=inputdata[i];
    }
    data = atoi(yloca);
  }
  if(task == 2)
  {
    char wrist[3] = "0";
    for(uint16_t i=8;i<10;i++)
    {    
      wrist[i-8]=inputdata[i];
    }
    data = atoi(wrist);
  }
  if(task == 3)
  {
    char angle[3] = "0";
    for(uint16_t i=10;i<12;i++)
    {    
      angle[i-10]=inputdata[i];
    }
    data = atoi(angle);
  }
  if(task == 4)
  {
    char type[2]="0";
    type[0]=inputdata[inputdataNum-1];
    data = atoi(type);
  }

  return data;
}


//int Valid_values(int inputdata,int k)
//{
//  int in = inputdata;
//  int outputdata = 0;
//  if(k == 0)
//  {
//    outputdata = in/100000;

//    if(outputdata/1000 == 2)
//      outputdata = -(outputdata%1000);
//    else
//      outputdata = outputdata%1000;
//  }
//  else if(k == 1)
//  {
//    outputdata = (in-(in/100000)*100000)/10;
//    if(outputdata/1000 == 2)
//      outputdata = -(outputdata%1000);
//    else
//      outputdata = outputdata%1000;
//  }

//  else if(k == 4)
//  {
//    outputdata = in%10;
//  }
//  
//  return outputdata;
//}


/*�������*/
/*���㷽ʽ������������*/
int32_t Calulate_Step(int32_t tar_location, int32_t cur_location)
{
  int32_t step = 0;  
  step = (tar_location - cur_location) / 0.07;                     // һ��������0.07mm
  return step;
}

int Calulate_WristAngle(int wristangle)
{
  double k = 0;
  k = (1 - wristangle / 90.0) * 657;
  
  if((k - (int)k) >= 0.5)                                         // �������뱣֤����
    return ((int)k+1);
  else
    return (int)k;
}

