
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
 *  通道数据处理
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
	 ///模式选择飞行程序

		
	//控制数据优先级
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
			
			//定高1.2米：
			if(Remote.thr > 1800){
				ALL_flag.take_off = 1;
//				pidHeightHigh.desired = 120;
				Climbing(dt,60); // hovring height
			}else if(Remote.thr < 1500){
				if(Descending(dt)) ALL_flag.take_off = 0;;
				
			}
		}
		remote_unlock(); // 解锁判断
  }		
  else
	{			
				NRF24L01_init();		
				cnt++;								//需要改进：时间缩短或加长，并判断距离为0 时关闭电机
				if(cnt>(3/dt))						//3秒没有遥控器信号 判断遥控器失联 信号断线 自动下降保护 10ms*300
				{	
					Remote.roll = 1500;  //通道1    数据归中
					LIMIT(Remote.roll,1000,2000);
					Remote.pitch = 1500;  //通道2		数据归中
					LIMIT(Remote.pitch,1000,2000);
					Remote.yaw =  1500;   //通道4		数据归中
					LIMIT(Remote.yaw,1000,2000);	
					
					if(Descending(dt)){
						cnt = 0;			
						ALL_flag.unlock = 0; 				//退出控制
						LED.status = AllFlashLight; //开始闪灯		
						NRF24L01_init();						//复位一下2.4G模块		
					}
				} 
	}	
}


u8 Descending(float dt){
	static int cnt_tmp;
	u8 Landing_flag;
	if(ALL_flag.unlock == 1&&(mini.flow_High<10||Remote.thr < 1030))						//判断油门 //需要改为判断高度
	{
			Landing_flag = 1; //降落成功
			Remote.thr =1000;						//关闭油门		
	}
	else
	{	
		Landing_flag = 0;
		if(cnt_tmp++>(0.1/dt))                 //控制油门减小的时间 //10ms *10 = 100ms						
		{
			cnt_tmp=0;
//							printf("Remote.thr: %d  \r\n",Remote.thr);				//串口1的打印			
//							Remote.thr = 	Remote.thr-LIMIT(mini.flow_High,2,10);   //通道3 油门通道在原来的基础上自动慢慢减小  起到飞机慢慢下降
			
			pidHeightHigh.desired = LIMIT(pidHeightHigh.desired - LIMIT(mini.flow_High,2,10)/5,0,200);
		}				
	}	
	return Landing_flag;
}

u8 Climbing(float dt, float Height){
	static int cnt_tmp;
	u8 Reached_flag;
	if(pidHeightHigh.desired >= Height)						//判断油门 //需要改为判断高度
	{
		Reached_flag = 1; //d到达目标高度	
	}
	else
	{	
		Reached_flag = 0;
		if(pidHeightHigh.desired < 30) pidHeightHigh.desired = 30; //30开始
		if(cnt_tmp++>(0.1/dt))                 //控制爬升的速度 100cm/s ->1cm/100ms					
		{
			cnt_tmp=0;
			if(pidHeightHigh.desired < mini.flow_High) pidHeightHigh.desired = mini.flow_High;
			pidHeightHigh.desired = LIMIT(pidHeightHigh.desired ++,0,200);
		}				
	}	
	return Reached_flag;
}

/*****************************************************************************************
 *  解锁判断
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
void remote_unlock(void)   //解锁数据解析
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Remote.thr<1200 &&Remote.yaw>1800)                         //油门遥杆右下角锁定飞机
	{
		status = EXIT_255;
	}
	
	switch(status)
	{
		case WAITING_1://等待解锁
			if(Remote.thr<1150)           //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁
			{			 
					 status = WAITING_2;				 
			}		
			break;
		case WAITING_2:
			if(Remote.thr>1800)        //拉高油门  
			{		
						static uint8_t cnt = 0;
					 	cnt++;		
						if(cnt>5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
						{	
								cnt=0;
								status = WAITING_3;
						}
			}			
			break;
		case WAITING_3:
			if(Remote.thr<1150)     //拉低油门解锁     
			{			 
					 status = WAITING_4;			//解锁标志位	
					 baro_start=1;            //气压清零标志 
			}			
			break;			
		case WAITING_4:	//解锁成功
				ALL_flag.unlock = 1;  
				status = PROCESS_31;
				LED.status = AlwaysOn;
		TEMP_AUX3 = Remote.AUX3;		
		TEMP_AUX4 = Remote.AUX3;
		TEMP_AUX5 = Remote.AUX3;
		TEMP_AUX6 = Remote.AUX3;
				 break;		
		case PROCESS_31:	//进入解锁状态
			if(Remote.AUX3!=TEMP_AUX3)                         //e-stop 一键进入exit
	{
		status = EXIT_255;
	}
				if(Remote.thr<1020)
				{
					if(cnt++ > 2000)                                     // 解锁后  不动油门遥杆处于最低6S自动上锁
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
		case EXIT_255: //进入锁定
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







