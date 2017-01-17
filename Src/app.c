/* USER CODE BEGIN 0 */
/*
	edit by link 2017-1-9
*/

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "app.h"
#include "math.h"
#include "string.h"
/*local variable*/

uint8_t countMax = 0;
uint8_t maxIndex;		//��ǰADֵ�����жϽ��
uint8_t maxAdIndexFormer;  //maxAd��Ӧ��Index,�жϽ��
uint8_t preIndex;   //��ǰ�ƺŵ���һ���ƺţ���3�ŵ� ���ϸ��ƺ���2��0  ��maxAdIndexFormer��������maxAdIndexFormer��ADֵ�жϵĽ��������ǵƵ�����λ�õ��жϽ��
uint32_t maxStay=0;	//���ֵͣ����ʱ��
uint8_t overPeak;	//��ǰADֵ�Ƿ����AD�ļ���.����趨����ܴ󡣴����
uint32_t overPeakCount = 0; //������ļ�����ֵ
int maxStayPre = 0;// ��һ�����ֵͣ����ʱ��
int8_t Direction = 1; //ת������ -1������ -1������
float speed;					//��ת�ٶ�
float speedPre;				//�ϴ��ٶ�
int speedMoni = 0;
uint32_t overPeakCountDown = 0;
uint32_t test = 0;   // use to test
uint32_t topTenMax[10] = {0,0,0,0,0,0,0,0,0,0};
uint32_t topTenMin[10] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};


uint32_t topTenMax0[10] = {0,0,0,0,0,0,0,0,0,0};
uint32_t topTenMin0[10] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};

uint32_t topTenMax1[10] = {0,0,0,0,0,0,0,0,0,0};
uint32_t topTenMin1[10] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};

uint32_t topTenMax2[10] = {0,0,0,0,0,0,0,0,0,0};
uint32_t topTenMin2[10] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};

uint32_t topTenMax3[10] = {0,0,0,0,0,0,0,0,0,0};
uint32_t topTenMin3[10] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
uint32_t minAD;
uint32_t maxAD; //��������ADֵ��������Сֵ������������һ�εƵ����ݹ�һ��,ȡtopTenMax�ľ�ֵ��
uint32_t maxAdEachLight[4]; //������һ��������AD�����ֵ
uint32_t changeMaxAd[4]; // 4���任����ʱ��ADֵ��С
uint32_t pre1,pre2,pre3,pre4,pre5;

uint32_t maxMoni;
//  0-1-2-3-0		changeMaxAd�������ֲ�	
// 0-1-2-3-0			�Ƶķֲ�������������ת����1
float angleRaw;
float angleRaw180;
float angle;
float lambda;
float angle180;
float angle180Pre=0;
uint32_t angle180Moni;
uint32_t angleRaw180Moni;
typedef struct
{
	uint8_t index[4];
	uint32_t data[4];
	
}Light;

Light LightData;
Light LightDataPre;
Light LightDataSort;
Light LightDataSortPre;
extern __IO uint16_t My_AD[4];
/**********�����е�ADֵ���д�С����*************/
void Bubble_sort(Light *L)
{
	int i,j,temp;
	 for(i = 0; i < 4; i++)
    {
        for(j = 0; j  < 4-i-1; j++)
        {
            if(L->data[j] < L->data[j + 1])
            {
                temp = L->data[j];
                L->data[j] = L->data[j + 1];
                L->data[j + 1] = temp;
							
							  temp = L->index[j];
                L->index[j] = L->index[j + 1];
                L->index[j + 1] = temp;
            }		
        }
    }
}

/**********���������������**********/
void Bubble_sort_all(uint32_t *in, uint8_t len)
{
	int i,j,temp;
	 for(i = 0; i < len; i++)
    {
        for(j = 0; j  < len-i-1; j++)
        {
            if(in[j] < in[j + 1])
            {
                temp = in[j];
                in[j] = in[j + 1];
                in[j + 1] = temp;

            }		
        }
    }
}




void timer3DataProcess()
{
	LightData.index[0] = 0;
	LightData.index[1] = 1;
  LightData.index[2] = 2;
  LightData.index[3] = 3;		
	LightData.data[0] = My_AD[3];
	LightData.data[1] = My_AD[1];
	LightData.data[2] = My_AD[2];
	LightData.data[3] = My_AD[0];	 
	

	LightDataSort = LightData;
	Bubble_sort(&LightDataSort);
	//��10������10����С��ADֵ
	findTenMaxAndMinAD();
	findEachLightMaxEveryPeriod();
	
	if(maxIndex == LightDataSort.index[0]-1 || (LightDataSort.index[0] == 0 && maxIndex == 3))
	{
		Direction = 1;
	}	
	else if (maxIndex == LightDataSort.index[0]+1 || (LightDataSort.index[0] == 3 && maxIndex == 0))
	{
		Direction = -1;
	}
	else
	{
		
	}
	if (Direction == 1)
	{	
		if(LightDataSort.index[0]==0)  //��ǰ������������һ������  �ݶ�Ϊ��
		{
			preIndex = 3;
		}
		else
		{
			preIndex = LightDataSort.index[0]-1;
		}
	}
	else
	{
		if(LightDataSort.index[0]==3)  //��ǰ������������һ������  �ݶ�Ϊ��
		{
			preIndex = 0;
		}
		else
		{
			preIndex = LightDataSort.index[0]+1;
		}
	}
	
	//if (speed / maxStay)
	if(maxStay < maxStayPre)  //��û�м�⵽��һ���Ƶ�ʱ�� �ٶ�Ĭ�Ϻ���һ֡һ��
		speed = speedPre;
	else 											// ��⵽�ٶ��Ѿ�������һ֡���ٶ���  �����¼����ٶ�
		speed = 1 / ((float)maxStay*4.0f*0.001f)  * Direction;
	if ((LightDataSort.data[0] >  1.01f*LightData.data[preIndex]) && LightDataSort.index[0] != maxIndex) // ��������ˣ��ж�����������һ�������ĵ�ѹ3���������Σ��ж����µķ�ֵ
		countMax++;
	if (countMax>1)  //�������� ȷ�����AD�����˱仯
	{

		speed = 1 / ((float)maxStay*4.0f*0.001f) * Direction;	 // (r/s)         get speed	
		speedPre = speed; 
		maxStayPre = maxStay;
		maxStay = 0;
		countMax = 0;
		overPeak = 0;
		
		pre1 = LightDataSort.data[0] - 10;
		pre2 = pre1;
		pre3 = pre2;
		pre4 = pre3;
		pre5 = pre4;
		
		maxAD = sumArray(topTenMax,10)/10;
		minAD = sumArray(topTenMin,10)/10;
		
		maxAdIndexFormer = maxIndex;
		maxIndex = LightDataSort.index[0];
		memset(topTenMax,0x00,sizeof(topTenMax));
		memset(topTenMin,0xFF,sizeof(topTenMin));
		//findADmax()
		if(maxIndex == 0)   //ת����һ�ܣ�����һ��ÿ���Ƶ�AD���ֵ��
		{
			//����
			maxAdEachLight[0] = sumArray(topTenMax0,10)/10;
			maxAdEachLight[1] = sumArray(topTenMax1,10)/10;
			maxAdEachLight[2] = sumArray(topTenMax2,10)/10;
			maxAdEachLight[3] = sumArray(topTenMax3,10)/10;

			memset(topTenMax0,0x00,sizeof(topTenMax0));
			memset(topTenMax1,0x00,sizeof(topTenMax1));
			memset(topTenMax2,0x00,sizeof(topTenMax2));
			memset(topTenMax3,0x00,sizeof(topTenMax3));
		}
		
		//����OverPeak���AD���ֵ
		
		//�����л����ADֵ
	  if((Direction==1 && maxIndex==1) || (Direction==-1 && maxIndex==0))
	  {	
			changeMaxAd[0] = LightDataSort.data[0];	
		}
		else if((Direction==1 && maxIndex==2) || (Direction==-1 && maxIndex==1))
		{
			changeMaxAd[1] = LightDataSort.data[0];
		}
		else if((Direction==1 && maxIndex==3) || (Direction==-1 && maxIndex==2))
		{
			changeMaxAd[2] = LightDataSort.data[0];
		}
		else if((Direction==1 && maxIndex==0) || (Direction==-1 && maxIndex==3))
		{
			changeMaxAd[3] = LightDataSort.data[0];
		}
	}
	

	findAngel();
	LightDataSortPre = LightDataSort;
	speedMoni = (int)(speed * 1000);
}

uint32_t sumArray(uint32_t *in, uint8_t len)
{
	uint32_t sum = 0;
	uint8_t i ;
	for(i=0; i<len; i++)
	{
		sum += in[i];
	}
	return sum; 
}

void findTenMaxAndMinAD()
{
	//findMax
	if(LightDataSort.data[0] > topTenMax[9]) //��ǰ�����������������С��
	{
		topTenMax[9] = LightDataSort.data[0];
		Bubble_sort_all(topTenMax, 10);	
	}else{}
		
			if(LightDataSort.data[3] < topTenMin[0])//��ǰ��СС����С�����е����
	{
		topTenMin[0] = LightDataSort.data[3];
		Bubble_sort_all(topTenMin, 10);	
	}else{}
	// 0
}

void findEachLightMaxEveryPeriod()
{

	if(LightData.data[0] > topTenMax0[9]) //��ǰ�����������������С��
	{
		topTenMax0[9] = LightData.data[0];
		Bubble_sort_all(topTenMax0, 10);	
	}else{}
	//1
		if(LightData.data[1] > topTenMax1[9]) //��ǰ�����������������С��
	{
		topTenMax1[9] = LightData.data[1];
		Bubble_sort_all(topTenMax1, 10);	
	}else{}
	//2
	if(LightData.data[2] > topTenMax2[9]) //��ǰ�����������������С��
	{
		topTenMax2[9] = LightData.data[2];
		Bubble_sort_all(topTenMax2, 10);	
	}else{}
	//3	
	if(LightData.data[3] > topTenMax3[9]) //��ǰ�����������������С��
	{
		topTenMax3[9] = LightData.data[3];
		Bubble_sort_all(topTenMax3, 10);	
	}else{}
}
	
/* USER CODE END 0 */
void findAngel()//��bug����Ҫ���ĵط�
{
	if(LightDataSort.data[0] < LightDataSortPre.data[0]) //����ADֵ��ʼ��С��
	{
		overPeakCount ++;
	  if(overPeakCount>2)
		{
			overPeak = 1;
			overPeakCountDown=0;
		}
	}
	else																									//������ӣ�overPeak��0,���������ֵ�л���ʱ��Ҳ��overPeak��0�ˡ�
	{
		overPeakCountDown++;
		if(overPeakCountDown>2)
		{
		  overPeak = 0;
			overPeakCount = 0;
		}
	}

	
	if((Direction == 1 && overPeak == 1) || (Direction == -1 && overPeak == 0)) // ����ת�٣�����ǵ�ǰ���������Ƕ�,����ת������ǵ�ǰ
	{
		if((maxAdEachLight[LightDataSort.index[0]] - changeMaxAd[LightDataSort.index[0]]) != 0) //��ֹ����
			lambda = (float)(maxAdEachLight[LightDataSort.index[0]] - LightDataSort.data[0]) / (float)(maxAdEachLight[LightDataSort.index[0]] - changeMaxAd[LightDataSort.index[0]]);	
		else 
			lambda =1;
		
			if(lambda>1)
				lambda =1;
			else if (lambda<0)
				lambda =0;	
			angleRaw = lambda * 0.7854f;
			angle = angleRaw;
	}
	else																										// �����������������Ƕȡ�
	{

			if(LightDataSort.index[0] == 0 && (maxAdEachLight[LightDataSort.index[0]] - changeMaxAd[3])!=0)
				lambda = (float)(maxAdEachLight[LightDataSort.index[0]] - LightDataSort.data[0]) / (float)(maxAdEachLight[LightDataSort.index[0]] - changeMaxAd[3]);	
			else if(LightDataSort.index[0] != 0 && (maxAdEachLight[LightDataSort.index[0]] - changeMaxAd[LightDataSort.index[0]-1]) !=0)
				lambda = (float)(maxAdEachLight[LightDataSort.index[0]] - LightDataSort.data[0]) / (float)(maxAdEachLight[LightDataSort.index[0]] - changeMaxAd[LightDataSort.index[0]-1]);	
		  else
				lambda =1;
			
			if(lambda>1)
				lambda =1;
			else if (lambda<0)
				lambda =0;
			angleRaw = lambda * 0.7854f;
			angle = -angleRaw;
	}
	
		
		angle180 = angle/3.1416f*180.0f;
		angleRaw180 = angleRaw/3.1416f*180.f;
		angle180 = LightDataSort.index[0]*90 + angle180;
		if(angle180>=360)
			angle180 -= 360;
		else if (angle180 < 0)
			angle180 += 360;
		angle180 = 0.95f*angle180Pre+(1.0f-0.95f)*angle180;
		angle180Pre = angle180;
		maxMoni = LightDataSort.data[0];
		
		angle180Moni = angle180;
		angleRaw180Moni=angleRaw180;
}

