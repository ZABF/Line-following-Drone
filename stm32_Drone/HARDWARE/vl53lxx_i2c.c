#include "vl53lxx_i2c.h"
#include "delay.h"
#include "stdbool.h"	
#include "i2c.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * VL53 IIC��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 *
 * �޸�˵��:
 * �汾V1.3 ���Ӷ�vl53l1x��IIC������
********************************************************************************/

static void vl53IICStart(void);			//����iic��ʼ�ź�
static void vl53IICStop(void);	  		//����iicֹͣ�ź�
static void vl53IICAck(void);			//iic����ACK�ź�
static void vl53IICNAck(void);			//iic������ACK�ź� 
static u8 vl53IICWaitAck(void);			//iic�ȴ�ACK�ź�
static void vl53IICSendByte(u8 txd);	//iic����һ���ֽ�
static u8 vl53IICReceiveByte(u8 ack);	//iic��ȡһ���ֽ�

//��ʼ��iic
void vl53IICInit(void)
{	
	IIC_Init();
}
//����VL53��ʼ�ź�

static void vl53IICStart(void)
{
	I2c_Start();
}	  
//����VL53ֹͣ�ź�
static void vl53IICStop(void)
{
	I2c_Stop();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
static u8 vl53IICWaitAck(void)
{   
	return I2c_WaitAck();  
} 
//����ACKӦ��
static void vl53IICAck(void)
{
	I2c_Ack();
}
//������ACKӦ��		    
static void vl53IICNAck(void)
{
	I2c_NoAck();
}					 				     
//VL53����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
static void vl53IICSendByte(u8 txd)
{                        
   I2c_SendByte(txd);
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
static u8 vl53IICReceiveByte(u8 ack)
{  
    return I2c_Soft_ReadByte(ack);//I2c_ReadByte();
}

//��ָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data)
{				  
	u8 temp=0;		  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr);//��������д���� 	   
	vl53IICWaitAck(); 
	vl53IICSendByte(addr);   //���͵͵�ַ
	vl53IICWaitAck();	

	vl53IICStart();  	 	   
	vl53IICSendByte(devaddr|1);//��������������			   
	vl53IICWaitAck();	 
	temp=vl53IICReceiveByte(0);			   
	vl53IICStop();//����һ��ֹͣ����	 
	*data = temp;
	return temp;
}
//����������ֽ�
//addr:��ʼ��ַ
//rbuf:�����ݻ���
//len:���ݳ���
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr);//��ַ����  
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//���һ���ֽڲ�Ӧ��
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//����������ֽ�
//addr:��ʼ��ַ
//rbuf:�����ݻ���
//len:���ݳ���
void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //��ַ��λ
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //��ַ��λ
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//���һ���ֽڲ�Ӧ��
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//��ָ����ַд��һ������
//WriteAddr :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data)
{				   	  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr); //��������д���� 	 
	vl53IICWaitAck();	   
	vl53IICSendByte(addr);   //���͵͵�ַ
	vl53IICWaitAck(); 	 										  		   
	vl53IICSendByte(data); //�����ֽ�							   
	vl53IICWaitAck();  		    	   
	vl53IICStop();		//����һ��ֹͣ���� 	 
}

//����д����ֽ�
//addr:��ʼ��ַ
//wbuf:д���ݻ���
//len:���ݵĳ���
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr);  //��ַ����
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//����д����ֽ�
//addr:��ʼ��ַ
//wbuf:д���ݻ���
//len:���ݵĳ���
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //��ַ��λ
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //��ַ��λ
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//iic д��ĳ��λ
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data)
{
    u8 byte;
    vl53IICReadByte(devaddr, addr, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    vl53IICWriteByte(devaddr, addr, byte);
	return true;
}







