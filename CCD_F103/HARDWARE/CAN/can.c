#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include <stm32f10x.h>
//All rights reserved	
//********************************************************************************
//V1.1修改说明 20150528
//修正了CAN初始化函数的相关注释，更正了波特率计算公式
////////////////////////////////////////////////////////////////////////////////// 	 
 
//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024;  tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//则波特率为:36M/((8+9+1)*4)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败; 
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{ 
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
	NVIC_InitTypeDef  		NVIC_InitStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO

	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;			//非时间触发通信模式  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//软件自动离线管理	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=DISABLE;			//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= mode;	        //模式设置： mode:0,普通模式;1,回环模式; 
	//设置波特率
	CAN_InitStructure.CAN_SJW=tsjw;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;        //分频系数(Fdiv)为brp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//初始化CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=0;	//过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32位宽 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//激活过滤器0

	CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化
	

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0消息挂号中断允许.		    

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	return 0;
}   
 
//中断服务函数	
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
}






//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 Can_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x30;			// 标准标识符 
	TxMessage.ExtId=0x30;			// 设置扩展标示符 
	TxMessage.IDE=CAN_Id_Standard; 	// 标准帧
	TxMessage.RTR=CAN_RTR_Data;		// 数据帧
	TxMessage.DLC=len;				// 要发送的数据长度
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;	 
}

static u8 data=0;

u8 Can_Sendres(u8 result)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x30;			// 标准标识符 
	TxMessage.ExtId=0x30;			// 设置扩展标示符 
	TxMessage.IDE=CAN_Id_Standard; 	// 标准帧
	TxMessage.RTR=CAN_RTR_Data;		// 数据帧
	TxMessage.DLC=1;				// 要发送的数据长度
	TxMessage.Data[0]=result;			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	data=result;

	while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok)&&(i<0XFFF))i++;	//等待发送结束
	
	if(i>=0XFFF) return 1;
	return 0;	 
}
u8 Can_Sendresfailgoon(void)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x30;			// 标准标识符 
	TxMessage.ExtId=0x30;			// 设置扩展标示符 
	TxMessage.IDE=CAN_Id_Standard; 	// 标准帧
	TxMessage.RTR=CAN_RTR_Data;		// 数据帧
	TxMessage.DLC=1;				// 要发送的数据长度
	TxMessage.Data[0]=data;			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	

	while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok)&&(i<0XFFF))i++;	//等待发送结束
	
	if(i>=0XFFF) return 1;
	return 0;	 
}
u8 stirball(void)
{
	static u8 state=0;
	u8 direc=0;
	switch(state)
	{
		case 0:
			direc=90;
			state=1;
			break;
		case 1:
			direc=80;
		    state=2;
			break;
		case 2:
			direc=80;
		    state=3;
		break;
		case 3:
			direc=90;
		    state=0;
			break;
		default :
			break;
	}
	return direc;
}

u8 Can_Send2master(u8 result)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x33;			// 标准标识符 
	TxMessage.ExtId=0x33;			// 设置扩展标示符 
	TxMessage.IDE=CAN_Id_Standard; 	// 标准帧
	TxMessage.RTR=CAN_RTR_Data;		// 数据帧
	TxMessage.DLC=1;				// 要发送的数据长度
	TxMessage.Data[0]=result;			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;	 
}


//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<8;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

void CAN_Config(CAN_TypeDef* CANx, 
				uint32_t CAN_BaudRate,
				GPIO_TypeDef * GPIOx,
				uint16_t CAN_RxPin,
				uint16_t CAN_TxPin)
{
	GPIO_InitTypeDef       GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  	   NVIC_InitStructure;
	uint8_t CAN_RxSource=0;
	uint8_t CAN_TxSource=0;
    uint8_t GPIO_AF_CANx=0;

  /* CAN GPIOs configuration */
	
	//确定CAN_RxPin
	
	#ifdef CanPd0_1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);	
	#endif
	#ifdef CanPb8_9
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
	#endif
	
	

	switch(CAN_RxPin)
	{    
		case GPIO_Pin_0:
		{
           CAN_RxSource=GPIO_PinSource0;
		   break;
		}
		case GPIO_Pin_1:
		{
           CAN_RxSource=GPIO_PinSource1;
		   break;
		}
		case GPIO_Pin_2:
		{
           CAN_RxSource=GPIO_PinSource2;
		   break;
		}
		case GPIO_Pin_3:
		{
           CAN_RxSource=GPIO_PinSource3;
		   break;
		}
        case GPIO_Pin_4:
		{
           CAN_RxSource=GPIO_PinSource4;
		   break;
		}
		case GPIO_Pin_5:
		{
           CAN_RxSource=GPIO_PinSource5;
		   break;
		}
		case GPIO_Pin_6:
		{
           CAN_RxSource=GPIO_PinSource6;
		   break;
		}
		case GPIO_Pin_7:
		{
           CAN_RxSource=GPIO_PinSource7;
		   break;
		}
		case GPIO_Pin_8:
		{
           CAN_RxSource=GPIO_PinSource8;
		   break;
		}
		case GPIO_Pin_9:
		{
           CAN_RxSource=GPIO_PinSource9;
		   break;
		}
		case GPIO_Pin_10:
		{
           CAN_RxSource=GPIO_PinSource10;
		   break;
		}
		case GPIO_Pin_11:
		{
           CAN_RxSource=GPIO_PinSource11;
		   break;
		}
		case GPIO_Pin_12:
		{
           CAN_RxSource=GPIO_PinSource12;
		   break;
		}
		case GPIO_Pin_13:
		{
           CAN_RxSource=GPIO_PinSource13;
		   break;
		}
		case GPIO_Pin_14:
		{
           CAN_RxSource=GPIO_PinSource14;
		   break;
		}
		case GPIO_Pin_15:
		{
           CAN_RxSource=GPIO_PinSource15;
		   break;
		}
		
		default: break;
	}

	//确定CAN_TxPin
	switch(CAN_TxPin)
	{    
		case GPIO_Pin_0:
		{
           CAN_TxSource=GPIO_PinSource0;
		   break;
		}
		case GPIO_Pin_1:
		{
           CAN_TxSource=GPIO_PinSource1;
		   break;
		}
		case GPIO_Pin_2:
		{
           CAN_TxSource=GPIO_PinSource2;
		   break;
		}
		case GPIO_Pin_3:
		{
           CAN_TxSource=GPIO_PinSource3;
		   break;
		}
        case GPIO_Pin_4:
		{
           CAN_TxSource=GPIO_PinSource4;
		   break;
		}
		case GPIO_Pin_5:
		{
           CAN_TxSource=GPIO_PinSource5;
		   break;
		}
		case GPIO_Pin_6:
		{
           CAN_TxSource=GPIO_PinSource6;
		   break;
		}
		case GPIO_Pin_7:
		{
           CAN_TxSource=GPIO_PinSource7;
		   break;
		}
		case GPIO_Pin_8:
		{
           CAN_TxSource=GPIO_PinSource8;
		   break;
		}
		case GPIO_Pin_9:
		{
           CAN_TxSource=GPIO_PinSource9;
		   break;
		}
		case GPIO_Pin_10:
		{
           CAN_TxSource=GPIO_PinSource10;
		   break;
		}
		case GPIO_Pin_11:
		{
           CAN_TxSource=GPIO_PinSource11;
		   break;
		}
		case GPIO_Pin_12:
		{
           CAN_TxSource=GPIO_PinSource12;
		   break;
		}
		case GPIO_Pin_13:
		{
           CAN_TxSource=GPIO_PinSource13;
		   break;
		}
		case GPIO_Pin_14:
		{
           CAN_TxSource=GPIO_PinSource14;
		   break;
		}
		case GPIO_Pin_15:
		{
           CAN_TxSource=GPIO_PinSource15;
		   break;
		}
		
		default: break;
	}
  /* CANx clock source enable */ 
  switch((uint32_t)CANx)
  {
	//CANs on APB1
    case CAN1_BASE: 
    {
		CAN_FilterInitStructure.CAN_FilterNumber=0;	    //Filter 0
		NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		break;
    }
	case CAN2_BASE: 
    {
		CAN_FilterInitStructure.CAN_FilterNumber=14;	//Filter 14
//		NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN2_RX0_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
		break;
    }
	
    default: break;
  } 
  /* Enable GPIOx, clock */  
	switch((uint32_t)GPIOx)
	{
		case GPIOA_BASE: 
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		break;
		}
		case GPIOB_BASE: 
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		break;
		}
		case GPIOC_BASE: 
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		break;
		}
		case GPIOD_BASE: 
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
		break;
		}
		case GPIOE_BASE: 
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
		break;
		}
		case GPIOF_BASE: 
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
		break;
		}
		case GPIOG_BASE: 
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
		break;
		}
		default: 
			break;
	}
	

	/* Connect CAN pins to AF */
// PA11 为上拉输入（CAN_RX 引脚）PA12 为复用输出（CAN_TX 引脚） 
	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = CAN_TxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = CAN_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOx, &GPIO_InitStructure);			//初始化IO
	
	/* CAN register init */
//	CAN_DeInit(CANx);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;         //time triggered communication mode
	CAN_InitStructure.CAN_ABOM = DISABLE;         //automatic bus-off management
	CAN_InitStructure.CAN_AWUM = DISABLE;         //automatic wake-up mode
	CAN_InitStructure.CAN_NART = DISABLE;         //non-automatic retransmission mode
	CAN_InitStructure.CAN_RFLM = DISABLE;         //Receive FIFO Locked mode
	CAN_InitStructure.CAN_TXFP = DISABLE;         //transmit FIFO priority
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //CAN operating mode
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;   // keep CAN_SJW == 1, never change it
    CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq; //max=16
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq; //max=8
	/* CAN Baudrate =APB1_CLK/((CAN_SJW_tq+CAN_BS1_tq+CAN_BS2_tq)*CAN_Prescaler) */ //?
//	
//		    can.CAN_SJW = CAN_SJW_1tq;
//    can.CAN_BS1 = CAN_BS1_5tq;
//    can.CAN_BS2 = CAN_BS2_3tq;
    switch(CAN_BaudRate)
	{			
		case 10:
		{
			CAN_InitStructure.CAN_Prescaler = 200;
			break;
		}
		case 20:
		{
			CAN_InitStructure.CAN_Prescaler = 100;
			break;
		}
		case 50:
		{
			CAN_InitStructure.CAN_Prescaler = 40;	
			break;
		}
		case 100:
		{
			CAN_InitStructure.CAN_Prescaler = 20;			
			break;
		}
		case 125:
		{
			CAN_InitStructure.CAN_Prescaler = 16;			
			break;
		}
		case 250:
		{
			CAN_InitStructure.CAN_Prescaler = 8;	
			break;
		}
		case 500:
		{
			CAN_InitStructure.CAN_Prescaler = 4;			
			break;
		}
		case 1000:
		{
			CAN_InitStructure.CAN_Prescaler = 2;	
			break;
		}

		default: break;
	}
	
	
	

    CAN_InitStructure.CAN_Prescaler = 8; 
	
    CAN_Init(CANx, &CAN_InitStructure);
	
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32 Bit
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;               //32 Bis ID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;           //32 Bit Mask
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;           //activate Filter
	CAN_FilterInit(&CAN_FilterInitStructure);                        //intialize Filter

	/* Enable FIFO 0 message pending Interrupt */

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);		
}	



u8 Data_Send(u8 data)
{
     uint8_t mbox;	 
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x30;					 // standard identifier=0
	TxMessage.ExtId=0x30;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=1;
	
	    //msg[4].data=*(unsigned long long*)&data[i];	  	
	TxMessage.Data[0] = data;

		mbox= CAN_Transmit(CAN1, &TxMessage);   	
		while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok)&&(i<0XFFF))
		i++;
	if(i>=0XFFF)
		return 1;
	return 0;	

}

uint8_t CAN_RxMsg(CAN_TypeDef* CANx,
				  uint32_t * StdId,
				  uint8_t * buf,
				  uint8_t len)
{
	uint8_t i=0;
	CanRxMsg RxMessage;
    if(CAN_MessagePending(CANx,CAN_FIFO0)==0)return 0;		//if there is no data, get out of this function
    CAN_Receive(CANx, CAN_FIFO0, &RxMessage);				//reveive data	
    for(i=0;i<len;i++)
		buf[i]=RxMessage.Data[i]; 
	*StdId=RxMessage.StdId;
	return 1;
}











