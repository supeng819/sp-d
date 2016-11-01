#include "ccd.h"
#include "adc.h"
#include "math.h"
#include "usart.h"
#include "stdlib.h"
#include "delay.h"
#include "timer.h"
static u8 Pixel[128]={0},Pixel2[128]={0};        //像素
static uint8_t MidValue=64;      //白条位置
static int16_t MidValueold=64;   //上一次白条位置

static uint8_t extime=0;         //曝光时间 ms
static uint16_t timee=8000-1;

extern u8 TIME0flag_5ms;         //曝光时间周期标志位

static uint8_t LeftBranch;       //白条左边界
static uint8_t RightBranch;      //白条右边界
static u8 bianjiecha;            //白条宽度

static uint8_t   PixelAverageValue;
static uint8_t   PixelgoodAverage=100;//减小曝光时间可以减小此值	

VIEW record_view;
void ccd_init(void)
{
	LandzoCCD_init();             // CCD初始化 
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
  * @brief    ccd像素点采集 像素点处理
  * @param    
  * @retval      
  */
static uint8_t state=0;	
void ccd(void)
{	
   if( TIME0flag_5ms == 1 )
	{
		TIME0flag_5ms = 0 ; 
//		像素点采集
        ImageCapture(Pixel);
		
//		像素点处理
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
					
//		维持均值不变进行P调节曝光时间		
			  CalculateIntegrationTime();
					
//					通过上位机进行128个像素点的图像查看
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
	 for(i=3;i<126;i++)             //  去噪声
	 {
		   if(Pixel2[i]==0)
		   {              //  去噪声为0的时候
			 if((Pixel2[i-1]==1)&&(Pixel2[i+1]==1))
			 {Pixel2[i]=1;}
		   }
		   if(Pixel[i]==1)
		   {                 //  去噪声为1的时候
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

    for(i = 0; i < 200; i++) {                    //更改250，让CCD的图像看上去比较平滑，
      SamplingDelay();  //200ns                  //把该值改大或者改小达到自己满意的结果。
    }
    CLK_SetVal();
    *ImageData = Get_Adc(ADC_Channel_1)>>4;     //4096/16=256  12位    灰度值是8位 
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