#include "ccd.h"
#include "adc.h"
#include "math.h"
#include "usart.h"
#include "stdlib.h"
#include "delay.h"
#include "timer.h"
static u8 Pixel[128]={0},Pixel2[128]={0};        //����
static uint8_t MidValue=64;      //����λ��
static int16_t MidValueold=64;   //��һ�ΰ���λ��

static uint8_t extime=0;         //�ع�ʱ�� ms
static uint16_t timee=8000-1;

extern u8 TIME0flag_5ms;         //�ع�ʱ�����ڱ�־λ

static uint8_t LeftBranch;       //������߽�
static uint8_t RightBranch;      //�����ұ߽�
static u8 bianjiecha;            //�������

static uint8_t   PixelAverageValue;
static uint8_t   PixelgoodAverage=100;//��С�ع�ʱ����Լ�С��ֵ	

VIEW record_view;
void ccd_init(void)
{
	LandzoCCD_init();             // CCD��ʼ�� 
	StartIntegration();
}
void LandzoCCD_init(void)
{	
 GPIO_InitTypeDef  GPIO_InitStructure;
	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2;	 
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 	
 GPIO_ResetBits(GPIOA,GPIO_Pin_3);			     
 GPIO_ResetBits(GPIOA,GPIO_Pin_2);			     

 Adc_Init();		
}

/**
  * @name     ccd
  * @brief    ccd���ص�ɼ� ���ص㴦��
  * @param    
  * @retval      
  */
static uint8_t state=0;	
void ccd(void)
{	
   if( TIME0flag_5ms == 1 )
	{
		TIME0flag_5ms = 0 ; 
//		���ص�ɼ�
        ImageCapture(Pixel);
		
//		���ص㴦��
		findboundary();		
				switch(state)
					{
						case 0:
					    zerotip();
                            break;
						case 1:
						firsttip();
							break;							 
					}
					
//		ά�־�ֵ�������P�����ع�ʱ��		
			  CalculateIntegrationTime();
					
//					ͨ����λ������128�����ص��ͼ��鿴
//					 SendImageData(Pixel);
	}
}

static int contrastvalue=0;
static int fengcha=0;
static uint8_t MaxValue;
static uint8_t MaxValue=0;
static u8 zuixiaozhi;
void findboundary(void)
{
	u8 i=0;
	PixelAverageValue=PixelAverage(128,Pixel);
	MaxValue=Pixel[0];
	for(i=1; i<128; i++)      
	{
		if(Pixel[i]>MaxValue)
		{
			MaxValue=Pixel[i];
			MaxValue=i;
		}
	}
	contrastvalue=(PixelAverageValue+MaxValue)>>1;
	for(i=0;i<128;i++)
	{
		if(Pixel[i]>contrastvalue)
		{Pixel2[i]=1;}		
		else
		{Pixel2[i]=0;}
	}
   	if(MaxValue>90)
	{fengcha=Pixel[MaxValue]-Pixel[(MaxValue-12)];}
	else
	{fengcha=Pixel[MaxValue]-Pixel[(MaxValue+12)];}
	 for(i=3;i<126;i++)             //  ȥ����
	 {
		   if(Pixel2[i]==0)
		   {              //  ȥ����Ϊ0��ʱ��
			 if((Pixel2[i-1]==1)&&(Pixel2[i+1]==1))
			 {Pixel2[i]=1;}
		   }
		   if(Pixel[i]==1)
		   {                 //  ȥ����Ϊ1��ʱ��
		     if((Pixel2[i-1]==0)&&(Pixel2[i+1]==0))
			 {Pixel2[i]=0;}
		   }
	  } 
	  	  
	 if( (MaxValue<right_size) && (MaxValue>left_size) )
	 {
		 for(i=MaxValue;i>left_size;i--)
		 {
			 if(Pixel2[i]==0)
			 {LeftBranch=i;break;}			 
		 }
		 if(i==left_size)
			 LeftBranch=0;
		 for(i=MaxValue;i<right_size;i++)
		 {
			 if(Pixel2[i]==0)
			 {RightBranch=i;break;}
		 }
		 if(i==right_size)
			 RightBranch=128;
		 bianjiecha=RightBranch-LeftBranch;
     }
}
u16 zuo(void)
{
	return LeftBranch;
}
u16 you(void)
{
	return RightBranch;
}
void StartIntegration(void) {
    unsigned char i;
    SI_SetVal();            /* SI  = 1 */
    SamplingDelay();	
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */

    for(i=0; i<127; i++) {
        SamplingDelay();
        SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        SamplingDelay();
        SamplingDelay();
        CLK_ClrVal();       /* CLK = 0 */
    }
    SamplingDelay();
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */
}

void ImageCapture(uint8_t * ImageData) {
uint8_t i;
extern uint8_t AtemP;

    SI_SetVal();            /* SI  = 1 */
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    SamplingDelay();

    for(i = 0; i < 200; i++) {                    //����250����CCD��ͼ����ȥ�Ƚ�ƽ����
      SamplingDelay();  //200ns                  //�Ѹ�ֵ�Ĵ���߸�С�ﵽ�Լ�����Ľ����
    }
    CLK_SetVal();
    *ImageData = Get_Adc(ADC_Channel_1)>>4;     //4096/16=256  12λ    �Ҷ�ֵ��8λ 
    ImageData++;
                
     CLK_ClrVal();delay_us(10); //delay_us(20);
    
    for(i=0; i<127; i++) 
		{						
		SamplingDelay();
        SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        SamplingDelay();
        SamplingDelay();	
       *ImageData =  Get_Adc(ADC_Channel_1)>>4;	 
       ImageData ++ ; 
		CLK_ClrVal();          
    }             
    SamplingDelay();
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SamplingDelay();
    CLK_ClrVal();   
}

//uint8_t   IntegrationTime = 10;
static uint16_t  timeeold=8000-1;
static uint8_t	  Kpp=20;
float a;
int exposure=0;
void CalculateIntegrationTime(void) 
{
	exposure=PixelgoodAverage-PixelAverageValue;
	if( abs(exposure)<40 )
		{
			Kpp=20;
		}
    else
		{
			Kpp=5;
		}
		if(   PixelAverageValue  >110  )
		{		
			if(timee<1500) 
				{
					timee=1500;
				
				}
				else
				{
					timee+=((PixelgoodAverage-PixelAverageValue)*Kpp);
				  
				}
			TIM3->ARR=timee;		
		}
		else if(   PixelAverageValue  <90  )
		{
			if(timee>64000)
			{timee=65000;}
			else 
			{
				
				timee+=((PixelgoodAverage-PixelAverageValue)*Kpp);
			}
			TIM3->ARR=timee;
		}		 
		else
		{			
		}		
		extime=(u8)(timee/2000);			
}

uint8_t PixelAverage(uint8_t len, uint8_t *data) {
  unsigned char i;
  unsigned int sum = 0;
  for(i = 0; i<len; i++) {
    sum = sum + *data++;
  }
  return ((unsigned char)(sum/len));
}
uint8_t PixelAveragejici(uint8_t jicishu) 
{
	extern uint8_t Pixel2[128];
    unsigned int i,sum=0;
	for(i=0;i<jicishu;i++)
    {sum+=PixelAverage(128,Pixel2);}
	sum=sum/jicishu;
	return (sum);
}
void SamplingDelay(void){
   volatile uint8_t i ;
   for(i=0;i<1;i++) {
    __NOP();
    __NOP();}
   
}

void zerotip(void)
{
	static u8 continuation;
	 MidValue=(int)((LeftBranch+RightBranch)/2);
		 if(bianjiecha<20 && (MidValue=MidValueold))
		 {   
			 continuation++;
			 if(continuation>50)
			 {state=1;continuation=0;}
		 }
		 else
		 {continuation=0;}
	 MidValueold=MidValue;
}

void firsttip(void)
{
	MidValue=(uint8_t)((LeftBranch+RightBranch)/2);
	if( (bianjiecha>16)|| (bianjiecha<4) )
	{record_view=no;}
	else if(fengcha<20)
	{record_view=no;}
	else 
	{record_view=yes;}	
   if(record_view==yes)	                       	         
	{			
		MidValueold=MidValue;		
	}
	else
	{
		MidValue=MidValueold;
	}	
}
u8 getwhiteStripes(void)
{
	return bianjiecha;
}
u8 getmiddle(void)
{
	return MidValue;
}
u8 getexposuretime(void)
{
	if(extime<5)
	{TIM4->ARR=(extime*1000);}
	else
	{TIM4->ARR=5000;}
	return extime;
}