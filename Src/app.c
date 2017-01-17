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
uint8_t maxIndex;		//当前AD值最大的判断结果
uint8_t maxAdIndexFormer;  //maxAd对应的Index,判断结果
uint8_t preIndex;   //当前灯号的上一个灯号，如3号灯 则上个灯号是2或0  和maxAdIndexFormer的区别是maxAdIndexFormer是AD值判断的结果，这个是灯的物理位置的判断结果
uint32_t maxStay=0;	//最大值停留的时间
uint8_t overPeak;	//当前AD值是否过了AD的尖峰点.这个设定问题很大。待解决
uint32_t overPeakCount = 0; //过尖峰点的计算阈值
int maxStayPre = 0;// 上一次最大值停留的时间
int8_t Direction = 1; //转动方向 -1：正向 -1：反向
float speed;					//旋转速度
float speedPre;				//上次速度
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
uint32_t maxAD; //上两个灯AD值的最大和最小值，用来进行下一次灯的数据归一化,取topTenMax的均值。
uint32_t maxAdEachLight[4]; //各个灯一个周期内AD的最大值
uint32_t changeMaxAd[4]; // 4个变换索引时的AD值大小
uint32_t pre1,pre2,pre3,pre4,pre5;

uint32_t maxMoni;
//  0-1-2-3-0		changeMaxAd的索引分布	
// 0-1-2-3-0			灯的分布，从左到右是旋转方向1
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
/**********对所有灯AD值进行大小排序*************/
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

/**********输入数组进行排序**********/
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
	//找10个最大和10个最小的AD值
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
		if(LightDataSort.index[0]==0)  //当前最大的索引的上一个索引  暂定为减
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
		if(LightDataSort.index[0]==3)  //当前最大的索引的上一个索引  暂定为减
		{
			preIndex = 0;
		}
		else
		{
			preIndex = LightDataSort.index[0]+1;
		}
	}
	
	//if (speed / maxStay)
	if(maxStay < maxStayPre)  //在没有检测到下一个灯的时候 速度默认和上一帧一致
		speed = speedPre;
	else 											// 检测到速度已经低于上一帧的速度了  则重新计算速度
		speed = 1 / ((float)maxStay*4.0f*0.001f)  * Direction;
	if ((LightDataSort.data[0] >  1.01f*LightData.data[preIndex]) && LightDataSort.index[0] != maxIndex) // 如果最大变了，判读他大于上上一个索引的电压3倍以上三次，判断是新的峰值
		countMax++;
	if (countMax>1)  //连续三次 确定最大AD发生了变化
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
		if(maxIndex == 0)   //转过了一周，更新一次每个灯的AD最大值。
		{
			//更新
			maxAdEachLight[0] = sumArray(topTenMax0,10)/10;
			maxAdEachLight[1] = sumArray(topTenMax1,10)/10;
			maxAdEachLight[2] = sumArray(topTenMax2,10)/10;
			maxAdEachLight[3] = sumArray(topTenMax3,10)/10;

			memset(topTenMax0,0x00,sizeof(topTenMax0));
			memset(topTenMax1,0x00,sizeof(topTenMax1));
			memset(topTenMax2,0x00,sizeof(topTenMax2));
			memset(topTenMax3,0x00,sizeof(topTenMax3));
		}
		
		//更新OverPeak点的AD最大值
		
		//更新切换点的AD值
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
	if(LightDataSort.data[0] > topTenMax[9]) //当前最大大于最大数组中最小。
	{
		topTenMax[9] = LightDataSort.data[0];
		Bubble_sort_all(topTenMax, 10);	
	}else{}
		
			if(LightDataSort.data[3] < topTenMin[0])//当前最小小于最小数组中的最大
	{
		topTenMin[0] = LightDataSort.data[3];
		Bubble_sort_all(topTenMin, 10);	
	}else{}
	// 0
}

void findEachLightMaxEveryPeriod()
{

	if(LightData.data[0] > topTenMax0[9]) //当前最大大于最大数组中最小。
	{
		topTenMax0[9] = LightData.data[0];
		Bubble_sort_all(topTenMax0, 10);	
	}else{}
	//1
		if(LightData.data[1] > topTenMax1[9]) //当前最大大于最大数组中最小。
	{
		topTenMax1[9] = LightData.data[1];
		Bubble_sort_all(topTenMax1, 10);	
	}else{}
	//2
	if(LightData.data[2] > topTenMax2[9]) //当前最大大于最大数组中最小。
	{
		topTenMax2[9] = LightData.data[2];
		Bubble_sort_all(topTenMax2, 10);	
	}else{}
	//3	
	if(LightData.data[3] > topTenMax3[9]) //当前最大大于最大数组中最小。
	{
		topTenMax3[9] = LightData.data[3];
		Bubble_sort_all(topTenMax3, 10);	
	}else{}
}
	
/* USER CODE END 0 */
void findAngel()//出bug了需要检查的地方
{
	if(LightDataSort.data[0] < LightDataSortPre.data[0]) //最大的AD值开始减小了
	{
		overPeakCount ++;
	  if(overPeakCount>2)
		{
			overPeak = 1;
			overPeakCountDown=0;
		}
	}
	else																									//如果增加，overPeak置0,另外在最大值切换的时候，也将overPeak置0了。
	{
		overPeakCountDown++;
		if(overPeakCountDown>2)
		{
		  overPeak = 0;
			overPeakCount = 0;
		}
	}

	
	if((Direction == 1 && overPeak == 1) || (Direction == -1 && overPeak == 0)) // 正向转速，最大不是当前，输出正向角度,反向转，最大是当前
	{
		if((maxAdEachLight[LightDataSort.index[0]] - changeMaxAd[LightDataSort.index[0]]) != 0) //防止除零
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
	else																										// 另外两种情况输出正角度。
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

