#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "adc.h"
#include "timer.h"
#include "ccd.h"


extern u8 TIME4flag_5ms;
static u8 temp=0,counttime=0;
static uint16_t huizong=0;
u8 kuandu=0;
int main(void)
{	 
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
	 
	#ifdef CanPa11_12
	CAN_Config(CAN1, 500,GPIOA,GPIO_Pin_11, GPIO_Pin_12);
	#endif
	#ifdef CanPd0_1
	CAN_Config(CAN1, 500,GPIOD,GPIO_Pin_0, GPIO_Pin_1);
	#endif
	#ifdef CanPb8_9
	CAN_Config(CAN1, 500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
	#endif
	
	
	//TIM3控制曝光时间（帧率）时钟72M
	TIM3_Int_Init(8999,35); 
	//TIM4控制发送周期 时钟72M
	TIM4_Int_Init(5000-1,71);
	
	//SI CLK AO 初始化
	ccd_init();
	

	while(1)
	{
		ccd();				
		if(TIME4flag_5ms ==1)//每隔5ms 发一次数发送曝光时间  中间值，白条
		{
			TIME4flag_5ms=0;
			USART_SendData(USART1,'a');	
			USART_SendData(USART1,getmiddle());	
			USART_SendData(USART1,getexposuretime());	
			USART_SendData(USART1,getwhiteStripes());	
		}
	}	
}

