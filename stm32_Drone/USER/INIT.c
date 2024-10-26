#include "ALL_DEFINE.h" 
#include "ALL_DATA.h"
#include "spl06.h" 
#include "LED.h"
#include "Uart1.h"
#include "USART2.h"
#include "ANO_Data_Transfer.h"
#include "ADC.h"

volatile uint32_t SysTick_count; //系统时间计数
volatile uint8_t spl_flag; //系统时间计数
_st_Mpu MPU6050;   //MPU6050原始数据
_st_Mpu MPU6050_nf;   //MPU6050原始数据
_st_Mag AK8975;   
_st_AngE Angle;    //当前角度姿态值
_st_AngE Angle_nf;    //未滤波当前角度姿态值
_st_Remote Remote; //遥控通道值


volatile uint32_t ST_CpuID;
 
 
_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等



 _st_FlightData FlightData;
 //飞控命令
st_Command Command;

PidObject pidRateX; //内环PID数据
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID数据
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate;
PidObject pidHeightHigh;

PidObject Flow_PosPid_x;    //外环光流
PidObject Flow_PosPid_y;

PidObject Flow_SpeedPid_x;  //内环光流
PidObject Flow_SpeedPid_y;

_st_IMU IMU;

void pid_param_Init(void); //PID控制参数初始化，改写PID并不会保存数据，请调试完成后直接在程序里更改 再烧录到飞控


//获取CPU的ID
void GetLockCode(void)
{
	ST_CpuID = *(vu32*)(0x1ffff7e8);//低字节芯片ID用来做通讯对频通道
}

///////////////全部初始化//////////////////////////////////
void ALL_Init(void)
{
	float STBy;

	IIC_Init();             //I2C初始化
		
	pid_param_Init();       //PID参数初始化
	  
	LEDInit();              //LED闪灯初始化

	MpuInit();              //MPU6050初始化
	
		//ADC初始化
	ADC1_Init();
	
	ANO_Uart1_Init(19200);   //接光流模块
//	printf("ANO_Uart1_Init  \r\n")	
	if (FLY_TYPE == 2) 
	{UART2_Init(115200); }      //
	
	USART3_Config(500000);        //上位机串口初始化
	
//	printf("USART3_Config  \r\n");

	NRF24L01_init();				//2.4G遥控通信初始化
	
	if(spl0601_init() >=1)	 //气压计初始化
	{	  	
		  while(1)
			{ 
				STBy++;
				GPIOB->BSRR = GPIO_Pin_9;  //亮一个LED
				if(STBy>=100000)
				{
						spl_flag=0; 
						SPL_Err = 1;
					  break;
				}								
			}
	}
	else
	{
			spl_flag=1;
		  SPL_Err = 0;
	}
	
	TIM2_PWM_Config();			//2路PWM初始化		
	TIM3_PWM_Config();			//2路PWM初始化		
	
//	TIM1_Echo_Config();	//超声波模块初始化
//	Hcsr04_Init();			//超声波模块初始化
	
	vl53lxxInit();	//激光初始化
}

 

void pid_param_Init(void)
{
	
//////////////////内环speed PID///////////////////////	
	
	pidRateX.kp = 2.0f;//2.0f;
	pidRateY.kp = 2.0f;//2.0f;
	pidRateZ.kp = 3.0f;
	
	pidRateX.ki = 0.0f;
	pidRateY.ki = 0.0f;
	pidRateZ.ki = 0.0f;	
	
	pidRateX.kd = 0.08f;
	pidRateY.kd = 0.08f;
	pidRateZ.kd = 0.05f;	
	
/////////////外环angle PID///////////////////////////
	
	pidPitch.kp = 7.0f;
	pidRoll.kp = 7.0f;
	pidYaw.kp = 7.0f;	
	
	pidPitch.ki = 0.0f;
	pidRoll.ki = 0.0f;
	pidYaw.ki = 0.0f;	
	
	pidPitch.kd = 0.0f;
	pidRoll.kd = 0.0f;
	pidYaw.kd = 0.0f;	
	
	
//	pidHeightRate.kp = 1.3f; //0.5f
//	pidHeightRate.kd = 0.35f;
//	
//	pidHeightHigh.ki = 0.05f;
//	pidHeightHigh.kp = 0.3f;//0.32f

#define ORGI_LIFT
//#define SLOW_LIFT
#define FAST_LIFT
		//内环PID参数 速度
//	pidHeightRate.kp = 4.8f;//1.2f; //1.2f
//	pidHeightRate.ki = 0.04f;
//	pidHeightRate.kd = 0.085f;
		//外环PID参数
//	pidHeightHigh.kp = 1.2f;//1.2f
//	pidHeightHigh.ki = 0.00f;
//	pidHeightHigh.kd = 0.085f;//0.085f

#ifdef ORGI_LIFT	
	pidHeightRate.kp = 1.2f;//3.0f;//2.5f;//1.2f; //1.2f
	pidHeightRate.ki = 0.04f;//0.2f;
	pidHeightRate.kd = 0.085f;//0.3f;
#else
	pidHeightRate.kp = 3.0f;//2.5f;//1.2f; //1.2f
	pidHeightRate.ki = 0.2f;
	pidHeightRate.kd = 00.3f;

#endif
		//外环PID参数
#ifdef ORGI_LIFT
	pidHeightHigh.kp = 1.2f;//1.2f
	pidHeightHigh.ki = 0.00f;
	pidHeightHigh.kd = 0.085f;//0.085f
#endif

#ifdef FAST_LIFT
	pidHeightHigh.kp = 10.0f;//1.2f
	pidHeightHigh.ki = 0.5f;
	pidHeightHigh.kd = 4.452f;//0.085f	
#endif

#ifdef SLOW_LIFT
	pidHeightHigh.kp = 3.792;//1.2f
	pidHeightHigh.ki = 0.792f;
	pidHeightHigh.kd = 3.211f;//0.085f
#endif

/////////////////////////////////////////////////////////////////////

	//X内环光流PID参数  速度
	
	Flow_SpeedPid_x.kp = 0.610f;//1.5f;//0.610f;//比例  0.600f
	Flow_SpeedPid_x.ki = 0.5f;//0.000f;//积分
	Flow_SpeedPid_x.kd = 0.4f;//0.400f;//微分
	
	//X外环光流PID参数  位置
	
	Flow_PosPid_x.kp = 2.200f;//比例  2.000f
	Flow_PosPid_x.ki = 0.000f;//积分
	Flow_PosPid_x.kd = 0.006f;//微分
	
	//////////////////////////////////////////////////////////
	
		//Y内环光流PID参数 速度
	
	Flow_SpeedPid_y.kp = 0.610f;//比例
	Flow_SpeedPid_y.ki = 0.5f;//积分
	Flow_SpeedPid_y.kd = 0.400f;//微分
	
	//Y外环光流PID参数 位置 
	
	Flow_PosPid_y.kp = 2.200f;//比例
	Flow_PosPid_y.ki = 0.00f;//积分
	Flow_PosPid_y.kd = 0.006f;//微分

///////////////////////////////////////////////////////////////////
	


	Command.FlightMode = NORMOL;  //初始化为姿态飞行模式
}










