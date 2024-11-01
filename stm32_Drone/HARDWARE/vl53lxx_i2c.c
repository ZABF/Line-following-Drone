#include "vl53lxx_i2c.h"
#include "delay.h"
#include "stdbool.h"	
#include "i2c.h"
/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * VL53 IIC驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 增加对vl53l1x的IIC驱动。
********************************************************************************/

static void vl53IICStart(void);			//发送iic开始信号
static void vl53IICStop(void);	  		//发送iic停止信号
static void vl53IICAck(void);			//iic发送ACK信号
static void vl53IICNAck(void);			//iic不发送ACK信号 
static u8 vl53IICWaitAck(void);			//iic等待ACK信号
static void vl53IICSendByte(u8 txd);	//iic发送一个字节
static u8 vl53IICReceiveByte(u8 ack);	//iic读取一个字节

//初始化iic
void vl53IICInit(void)
{	
	IIC_Init();
}
//产生VL53起始信号

static void vl53IICStart(void)
{
	I2c_Start();
}	  
//产生VL53停止信号
static void vl53IICStop(void)
{
	I2c_Stop();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static u8 vl53IICWaitAck(void)
{   
	return I2c_WaitAck();  
} 
//产生ACK应答
static void vl53IICAck(void)
{
	I2c_Ack();
}
//不产生ACK应答		    
static void vl53IICNAck(void)
{
	I2c_NoAck();
}					 				     
//VL53发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
static void vl53IICSendByte(u8 txd)
{                        
   I2c_SendByte(txd);
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
static u8 vl53IICReceiveByte(u8 ack)
{  
    return I2c_Soft_ReadByte(ack);//I2c_ReadByte();
}

//从指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data)
{				  
	u8 temp=0;		  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr);//发送器件写命令 	   
	vl53IICWaitAck(); 
	vl53IICSendByte(addr);   //发送低地址
	vl53IICWaitAck();	

	vl53IICStart();  	 	   
	vl53IICSendByte(devaddr|1);//发送器件读命令			   
	vl53IICWaitAck();	 
	temp=vl53IICReceiveByte(0);			   
	vl53IICStop();//产生一个停止条件	 
	*data = temp;
	return temp;
}
//连续读多个字节
//addr:起始地址
//rbuf:读数据缓存
//len:数据长度
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr);//地址自增  
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//最后一个字节不应答
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//连续读多个字节
//addr:起始地址
//rbuf:读数据缓存
//len:数据长度
void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //地址高位
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //地址低位
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//最后一个字节不应答
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//从指定地址写入一个数据
//WriteAddr :写入数据的目的地址    
//DataToWrite:要写入的数据
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data)
{				   	  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr); //发送器件写命令 	 
	vl53IICWaitAck();	   
	vl53IICSendByte(addr);   //发送低地址
	vl53IICWaitAck(); 	 										  		   
	vl53IICSendByte(data); //发送字节							   
	vl53IICWaitAck();  		    	   
	vl53IICStop();		//产生一个停止条件 	 
}

//连续写多个字节
//addr:起始地址
//wbuf:写数据缓存
//len:数据的长度
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr);  //地址自增
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//连续写多个字节
//addr:起始地址
//wbuf:写数据缓存
//len:数据的长度
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //地址高位
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //地址低位
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//iic 写入某个位
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data)
{
    u8 byte;
    vl53IICReadByte(devaddr, addr, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    vl53IICWriteByte(devaddr, addr, byte);
	return true;
}







