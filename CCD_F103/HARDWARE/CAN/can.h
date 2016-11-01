#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.1 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved	
//********************************************************************************
//V1.1修改说明 20150528
//修正了CAN初始化函数的相关注释，更正了波特率计算公式
////////////////////////////////////////////////////////////////////////////////// 	 
 
//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
u8 Can_Send_Msg(u8* msg,u8 len);						//发送数据

u8 Can_Receive_Msg(u8 *buf);							//接收数据


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

















