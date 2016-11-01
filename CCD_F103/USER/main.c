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
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	 
	#ifdef CanPa11_12
	CAN_Config(CAN1, 500,GPIOA,GPIO_Pin_11, GPIO_Pin_12);
	#endif
	#ifdef CanPd0_1
	CAN_Config(CAN1, 500,GPIOD,GPIO_Pin_0, GPIO_Pin_1);
	#endif
	#ifdef CanPb8_9
	CAN_Config(CAN1, 500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
	#endif
	
	
	//TIM3�����ع�ʱ�䣨֡�ʣ�ʱ��72M
	TIM3_Int_Init(8999,35); 
	//TIM4���Ʒ������� ʱ��72M
	TIM4_Int_Init(5000-1,71);
	
	//SI CLK AO ��ʼ��
	ccd_init();
	

	while(1)
	{
		ccd();				
		if(TIME4flag_5ms ==1)//ÿ��5ms ��һ���������ع�ʱ��  �м�ֵ������
		{
			TIME4flag_5ms=0;
			USART_SendData(USART1,'a');	
			USART_SendData(USART1,getmiddle());	
			USART_SendData(USART1,getexposuretime());	
			USART_SendData(USART1,getwhiteStripes());	
		}
	}	
}

