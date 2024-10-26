#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "spl06.h"
#include "ANO_Data_Transfer.h"
#include "WIFI_UFO.h"
#include "flow.h"
#include "ADC.h"


//int a, b, c;///////
loop_t loop; 
u32 time[10],time_sum;
u32 	FlightPidControl_period_temp, GetAngle_period_temp,	HeightPidControl_period_temp,	Flow_Pos_Controler_period_temp;
float FlightPidControl_period			,	GetAngle_period			,	HeightPidControl_period			,	Flow_Pos_Controler_period			;		

int count_my = 0;
 
void Loop_check()
{
	loop.cnt_2ms++;
	loop.cnt_4ms++;
	loop.cnt_6ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_1000ms++;

	if( loop.check_flag >= 1)
	{
		loop.err_flag ++;// 2ms 
	}
	else
	{
		loop.check_flag += 1;   //�ñ�־λ��ѭ��������0
	}
}
void main_loop()
{
	if( loop.check_flag >= 1 )
	{
		
		if( loop.cnt_2ms >= 1 )
		{
			loop.cnt_2ms = 0;
			
			Duty_2ms();	 					//����2ms������
		}
		if( loop.cnt_4ms >= 2 )
		{
			loop.cnt_4ms = 0;
			Duty_4ms();						//����4ms������
		}
		if( loop.cnt_6ms >= 3 )
		{
			loop.cnt_6ms = 0;
			Duty_6ms();						//����6ms������
		}
		if( loop.cnt_10ms >= 5 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//����10ms������
		} 
		if( loop.cnt_20ms >= 10 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//����20ms������
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//����50ms������
		}
		if( loop.cnt_1000ms >= 500)
		{
			loop.cnt_1000ms = 0;
			Duty_1000ms();				//����1s������
		}
		loop.check_flag = 0;		//ѭ��������ϱ�־
	}
}
/////////////////////////////////////////////////////////
void Duty_2ms()
{
	time[0] = GetSysTime_us();
//	a=TIM_GetCounter(TIM1);
//	b=TIM_GetCounter(TIM2);
//	c=TIM_GetCounter(TIM3);
	MpuGetData();				          //��ȡ����������
	
	FlightPidControl_period = (GetSysTime_us() - FlightPidControl_period_temp)/1000000.0;
	FlightPidControl_period_temp = GetSysTime_us();
	FlightPidControl(FlightPidControl_period);//0.002f);     /// ��̬����
	
	
	MotorControl();               //�������
	
	time[0] = GetSysTime_us() - time[0];
	
}
//////////////////////////////////////////////////////////
void Duty_4ms()
{
	time[1] = GetSysTime_us();
	
	ANO_NRF_Check_Event();    //ɨ�����2.4G�ź�
	ANO_DT_Data_Exchange();		//���طɻ����ݵ�ң����
	Rc_Connect(0.004f);  						//����ң��������
	Mode_Controler(0.004f); 	//ģʽ����ѡ�����
	time[1] = GetSysTime_us() - time[1];
}
//////////////////////////////////////////////////////////
void Duty_6ms()
{
	time[2] = GetSysTime_us();
	GetAngle_period = (time[2]-GetAngle_period_temp)/1000000.0;
	GetAngle_period_temp = GetSysTime_us();
	GetAngle(&MPU6050,&Angle,GetAngle_period);//0.006f);   //������̬����
	GetAngle_nf(&MPU6050_nf,&Angle_nf,GetAngle_period*2);   //������̬����
	
	
	time[2] = GetSysTime_us() - time[2];
}
/////////////////////////////////////////////////////////
void Duty_10ms()
{
	time[3] = GetSysTime_us();
	
	RC_Analy(0.01f);	     					//ң��������ָ���
	
	Flow_Pos_Controler_period = (GetSysTime_us() - Flow_Pos_Controler_period_temp)/1000000.0;
	Flow_Pos_Controler_period_temp = GetSysTime_us();
	Pixel_Flow_Fix(0.01f);  			//mini���������ں�	
	Flow_Pos_Controler(0.01f);		//�����������	
	
	Height_Get(0.01f);			//��ȡ�߶�����	
	High_Data_Calc(10);			//�߶������ں�		
		
	HeightPidControl_period = (GetSysTime_us() - HeightPidControl_period_temp)/1000000.0;
	HeightPidControl_period_temp = GetSysTime_us();
	HeightPidControl(HeightPidControl_period); 		//��ѹ�߶ȿ���
	
	time[3] = GetSysTime_us() - time[3];
}
/////////////////////////////////////////////////////////
void Duty_20ms()
{
	time[4] = GetSysTime_us();

	ANTO_polling(); 	//����3�ڷɻ� ������ݵ� ������λ��

	
	time[4] = GetSysTime_us() - time[4];
}
//////////////////////////////////////////////////////////
void Duty_50ms()
{
	time[5] = GetSysTime_us();
	
	PilotLED(); 						//LEDˢ��
	
	Flag_Check();   			 //������״̬��־
	
	Voltage_Check();			//�ɿص�ѹ���
	
//	Hcsr04_Strat();			//������
	read_vl53l1x();
//	Line_Following();
	//printf("20ms\n");
	
	//pidYaw.desired += 0.11;
	
	time[5] = GetSysTime_us() - time[5];
}
/////////////////////////////////////////////////////////////
void Duty_1000ms()
{
	u8 i;
  NRF_SSI = NRF_SSI_CNT;  //NRF�ź�ǿ��
	NRF_SSI_CNT = 0;
	
	WIFI_SSI = WIFI_SSI_CNT;//WiFi�ź�ǿ��
	WIFI_SSI_CNT = 0;
	
	Locat_SSI = Locat_SSI_CNT;//�Ӿ�λ������Ƶ��
	Locat_SSI_CNT = 0;
	
	Flow_SSI = Flow_SSI_CNT;  //��������Ƶ��
	Flow_SSI_CNT = 0;
	
	MV_SSI = MV_SSI_CNT;	//�������Ƶ��
	MV_SSI_CNT = 0;
	
		//����Ӿ���λģ���Ƿ����
	if(Locat_SSI>10)  Locat_Err = 0;
	else Locat_Mode=0,Locat_Err = 1;
	
	  //������ģ���Ƿ����
	if(Flow_SSI>10)  Flow_Err = 0;
	else 						 Flow_Err = 1;
	
	count_my++;



//	if(count_my > 10) pidYaw.desired += 36;
//#define test_1
#ifdef test_1	
	if(count_my == 5){
		Flow_PosPid_x.desired = Flow_PosPid_x.desired + 40;
//		Flow_PosPid_y.desired = Flow_PosPid_y.desired + 1;
//		count_my=0;
	}
	if(count_my == 10){
		Flow_PosPid_x.desired = Flow_PosPid_x.desired + 40;
//		Flow_PosPid_y.desired = Flow_PosPid_y.desired + 1;
	}

	if(count_my > 15){
		pidYaw.desired += 36;	
	}

	if(count_my == 20) count_my = 0;
#endif
	if(!ALL_flag.unlock) count_my = 0;

	time_sum = 0;
	for(i=0;i<6;i++)	time_sum += time[i];
	
}



//////////////////////////end///////////////////////////////////////////
