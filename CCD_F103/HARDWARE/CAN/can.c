#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include <stm32f10x.h>
//All rights reserved	
//********************************************************************************
//V1.1�޸�˵�� 20150528
//������CAN��ʼ�����������ע�ͣ������˲����ʼ��㹫ʽ
////////////////////////////////////////////////////////////////////////////////// 	 
 
//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//������Ϊ:36M/((8+9+1)*4)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{ 
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
	NVIC_InitTypeDef  		NVIC_InitStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//����Զ����߹���	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=DISABLE;			//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	        //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; 
	//���ò�����
	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0

	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
	

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	return 0;
}   
 
//�жϷ�����	
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
}






//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 Can_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x30;			// ��׼��ʶ�� 
	TxMessage.ExtId=0x30;			// ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=len;				// Ҫ���͵����ݳ���
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;	 
}

static u8 data=0;

u8 Can_Sendres(u8 result)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x30;			// ��׼��ʶ�� 
	TxMessage.ExtId=0x30;			// ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=1;				// Ҫ���͵����ݳ���
	TxMessage.Data[0]=result;			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	data=result;

	while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	
	if(i>=0XFFF) return 1;
	return 0;	 
}
u8 Can_Sendresfailgoon(void)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x30;			// ��׼��ʶ�� 
	TxMessage.ExtId=0x30;			// ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=1;				// Ҫ���͵����ݳ���
	TxMessage.Data[0]=data;			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	

	while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	
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
	TxMessage.StdId=0x33;			// ��׼��ʶ�� 
	TxMessage.ExtId=0x33;			// ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=1;				// Ҫ���͵����ݳ���
	TxMessage.Data[0]=result;			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;	 
}


//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
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
	
	//ȷ��CAN_RxPin
	
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

	//ȷ��CAN_TxPin
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
// PA11 Ϊ�������루CAN_RX ���ţ�PA12 Ϊ���������CAN_TX ���ţ� 
	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = CAN_TxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = CAN_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOx, &GPIO_InitStructure);			//��ʼ��IO
	
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











