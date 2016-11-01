#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.1 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved	
//********************************************************************************
//V1.1�޸�˵�� 20150528
//������CAN��ʼ�����������ע�ͣ������˲����ʼ��㹫ʽ
////////////////////////////////////////////////////////////////////////////////// 	 
 
//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
 
u8 Can_Send_Msg(u8* msg,u8 len);						//��������

u8 Can_Receive_Msg(u8 *buf);							//��������


void CAN_Config(CAN_TypeDef* CANx, uint32_t CAN_BaudRate,GPIO_TypeDef * GPIOx,uint16_t CAN_RxPin,uint16_t CAN_TxPin);


u8 Data_Send(u8 data);
uint8_t CAN_RxMsg(CAN_TypeDef* CANx,uint32_t * StdId,uint8_t * buf,uint8_t len);


//#define CanPa11_12
//#define CanPb8_9
//#define CanPd0_1


u8 Can_Sendres(u8 result);
u8 Can_Send2master(u8 result);



void Clearmasterdata(void);
u8 stirball(void);

u8 Getdividirchange(void);

u8 Getdividir(void);

void Cleardirchange(void);

u8 Can_Sendresfailgoon(void);


#endif

















