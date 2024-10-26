
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "WIFI_UFO.h"
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "flow.h"
#include "spl06.h"

#define SUCCESS 0
#undef FAILED
#define FAILED  1

u16 test_flag,set_flag;
u16 TEMP_AUX3;
u16 TEMP_AUX4;
u16 TEMP_AUX5;
u16 TEMP_AUX6;

u8 Rc_Connect_flag;
void Rc_Connect(float dt)
{
	if(NRF_Connect()==0){
		Rc_Connect_flag =0;
	}else
		Rc_Connect_flag =1;
}

/*****************************************************************************************
 *  ͨ�����ݴ���
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
uint8_t RC_rxData[32];
void remote_unlock(void);	

void RC_Analy(float dt)  
{
		static uint16_t cnt,cnt_tmp;
/*             Receive  and check RC data                               */	
//	static u8 Connect_flag;
	 ///ģʽѡ����г���

		
	//�����������ȼ�
	if(Rc_Connect_flag == 1) { 	
		{
			const float yaw_ratio =  0.0015f;    
			
			if(Remote.yaw>1820){
//				pidYaw.desired -= 0.75f;	
				pidYaw.desired -= 0.25f;
			}
			else if(Remote.yaw <1180){
//				pidYaw.desired += 0.75f;	
				pidYaw.desired += 0.25f;	
			}											
			
			//����1.2�ף�
			if(Remote.thr > 1800){
				ALL_flag.take_off = 1;
//				pidHeightHigh.desired = 120;
				Climbing(dt,60); // hovring height
			}else if(Remote.thr < 1500){
				if(Descending(dt)) ALL_flag.take_off = 0;;
				
			}
		}
		remote_unlock(); // �����ж�
  }		
  else
	{			
				NRF24L01_init();		
				cnt++;								//��Ҫ�Ľ���ʱ�����̻�ӳ������жϾ���Ϊ0 ʱ�رյ��
				if(cnt>(3/dt))						//3��û��ң�����ź� �ж�ң����ʧ�� �źŶ��� �Զ��½����� 10ms*300
				{	
					Remote.roll = 1500;  //ͨ��1    ���ݹ���
					LIMIT(Remote.roll,1000,2000);
					Remote.pitch = 1500;  //ͨ��2		���ݹ���
					LIMIT(Remote.pitch,1000,2000);
					Remote.yaw =  1500;   //ͨ��4		���ݹ���
					LIMIT(Remote.yaw,1000,2000);	
					
					if(Descending(dt)){
						cnt = 0;			
						ALL_flag.unlock = 0; 				//�˳�����
						LED.status = AllFlashLight; //��ʼ����		
						NRF24L01_init();						//��λһ��2.4Gģ��		
					}
				} 
	}	
}


u8 Descending(float dt){
	static int cnt_tmp;
	u8 Landing_flag;
	if(ALL_flag.unlock == 1&&(mini.flow_High<10||Remote.thr < 1030))						//�ж����� //��Ҫ��Ϊ�жϸ߶�
	{
			Landing_flag = 1; //����ɹ�
			Remote.thr =1000;						//�ر�����		
	}
	else
	{	
		Landing_flag = 0;
		if(cnt_tmp++>(0.1/dt))                 //�������ż�С��ʱ�� //10ms *10 = 100ms						
		{
			cnt_tmp=0;
//							printf("Remote.thr: %d  \r\n",Remote.thr);				//����1�Ĵ�ӡ			
//							Remote.thr = 	Remote.thr-LIMIT(mini.flow_High,2,10);   //ͨ��3 ����ͨ����ԭ���Ļ������Զ�������С  �𵽷ɻ������½�
			
			pidHeightHigh.desired = LIMIT(pidHeightHigh.desired - LIMIT(mini.flow_High,2,10)/5,0,200);
		}				
	}	
	return Landing_flag;
}

u8 Climbing(float dt, float Height){
	static int cnt_tmp;
	u8 Reached_flag;
	if(pidHeightHigh.desired >= Height)						//�ж����� //��Ҫ��Ϊ�жϸ߶�
	{
		Reached_flag = 1; //d����Ŀ��߶�	
	}
	else
	{	
		Reached_flag = 0;
		if(pidHeightHigh.desired < 30) pidHeightHigh.desired = 30; //30��ʼ
		if(cnt_tmp++>(0.1/dt))                 //�����������ٶ� 100cm/s ->1cm/100ms					
		{
			cnt_tmp=0;
			if(pidHeightHigh.desired < mini.flow_High) pidHeightHigh.desired = mini.flow_High;
			pidHeightHigh.desired = LIMIT(pidHeightHigh.desired ++,0,200);
		}				
	}	
	return Reached_flag;
}

/*****************************************************************************************
 *  �����ж�
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
void remote_unlock(void)   //�������ݽ���
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Remote.thr<1200 &&Remote.yaw>1800)                         //����ң�����½������ɻ�
	{
		status = EXIT_255;
	}
	
	switch(status)
	{
		case WAITING_1://�ȴ�����
			if(Remote.thr<1150)           //���������࣬�������->�������->������� ����LED�Ʋ����� ����ɽ���
			{			 
					 status = WAITING_2;				 
			}		
			break;
		case WAITING_2:
			if(Remote.thr>1800)        //��������  
			{		
						static uint8_t cnt = 0;
					 	cnt++;		
						if(cnt>5) //��������豣��200ms���ϣ���ֹң�ؿ�����ʼ��δ��ɵĴ�������
						{	
								cnt=0;
								status = WAITING_3;
						}
			}			
			break;
		case WAITING_3:
			if(Remote.thr<1150)     //�������Ž���     
			{			 
					 status = WAITING_4;			//������־λ	
					 baro_start=1;            //��ѹ�����־ 
			}			
			break;			
		case WAITING_4:	//�����ɹ�
				ALL_flag.unlock = 1;  
				status = PROCESS_31;
				LED.status = AlwaysOn;
		TEMP_AUX3 = Remote.AUX3;		
		TEMP_AUX4 = Remote.AUX3;
		TEMP_AUX5 = Remote.AUX3;
		TEMP_AUX6 = Remote.AUX3;
				 break;		
		case PROCESS_31:	//�������״̬
			if(Remote.AUX3!=TEMP_AUX3)                         //e-stop һ������exit
	{
		status = EXIT_255;
	}
				if(Remote.thr<1020)
				{
					if(cnt++ > 2000)                                     // ������  ��������ң�˴������6S�Զ�����
					{								
						status = EXIT_255;								
					}
				}
				else if(!ALL_flag.unlock)                           //Other conditions lock 
				{
					status = EXIT_255;				
				}
				else					
					cnt = 0;
			break;
		case EXIT_255: //��������
			LED.status = AllFlashLight;	                                 //exit
			cnt = 0;
			LED.FlashTime = 300; //300*3ms		
			ALL_flag.unlock = 0;
			status = WAITING_1;
			break;
		default:
			status = EXIT_255;
			break;
	}
}
/***********************END OF FILE*************************************/







