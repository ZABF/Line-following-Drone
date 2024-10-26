#include "ALL_DEFINE.h" 
#include "ALL_DATA.h"
#include "spl06.h" 
#include "LED.h"
#include "Uart1.h"
#include "USART2.h"
#include "ANO_Data_Transfer.h"
#include "ADC.h"

volatile uint32_t SysTick_count; //ϵͳʱ�����
volatile uint8_t spl_flag; //ϵͳʱ�����
_st_Mpu MPU6050;   //MPU6050ԭʼ����
_st_Mpu MPU6050_nf;   //MPU6050ԭʼ����
_st_Mag AK8975;   
_st_AngE Angle;    //��ǰ�Ƕ���ֵ̬
_st_AngE Angle_nf;    //δ�˲���ǰ�Ƕ���ֵ̬
_st_Remote Remote; //ң��ͨ��ֵ


volatile uint32_t ST_CpuID;
 
 
_st_ALL_flag ALL_flag; //ϵͳ��־λ������������־λ��



 _st_FlightData FlightData;
 //�ɿ�����
st_Command Command;

PidObject pidRateX; //�ڻ�PID����
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //�⻷PID����
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate;
PidObject pidHeightHigh;

PidObject Flow_PosPid_x;    //�⻷����
PidObject Flow_PosPid_y;

PidObject Flow_SpeedPid_x;  //�ڻ�����
PidObject Flow_SpeedPid_y;

_st_IMU IMU;

void pid_param_Init(void); //PID���Ʋ�����ʼ������дPID�����ᱣ�����ݣ��������ɺ�ֱ���ڳ�������� ����¼���ɿ�


//��ȡCPU��ID
void GetLockCode(void)
{
	ST_CpuID = *(vu32*)(0x1ffff7e8);//���ֽ�оƬID������ͨѶ��Ƶͨ��
}

///////////////ȫ����ʼ��//////////////////////////////////
void ALL_Init(void)
{
	float STBy;

	IIC_Init();             //I2C��ʼ��
		
	pid_param_Init();       //PID������ʼ��
	  
	LEDInit();              //LED���Ƴ�ʼ��

	MpuInit();              //MPU6050��ʼ��
	
		//ADC��ʼ��
	ADC1_Init();
	
	ANO_Uart1_Init(19200);   //�ӹ���ģ��
//	printf("ANO_Uart1_Init  \r\n")	
	if (FLY_TYPE == 2) 
	{UART2_Init(115200); }      //
	
	USART3_Config(500000);        //��λ�����ڳ�ʼ��
	
//	printf("USART3_Config  \r\n");

	NRF24L01_init();				//2.4Gң��ͨ�ų�ʼ��
	
	if(spl0601_init() >=1)	 //��ѹ�Ƴ�ʼ��
	{	  	
		  while(1)
			{ 
				STBy++;
				GPIOB->BSRR = GPIO_Pin_9;  //��һ��LED
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
	
	TIM2_PWM_Config();			//2·PWM��ʼ��		
	TIM3_PWM_Config();			//2·PWM��ʼ��		
	
//	TIM1_Echo_Config();	//������ģ���ʼ��
//	Hcsr04_Init();			//������ģ���ʼ��
	
	vl53lxxInit();	//�����ʼ��
}

 

void pid_param_Init(void)
{
	
//////////////////�ڻ�speed PID///////////////////////	
	
	pidRateX.kp = 2.0f;//2.0f;
	pidRateY.kp = 2.0f;//2.0f;
	pidRateZ.kp = 3.0f;
	
	pidRateX.ki = 0.0f;
	pidRateY.ki = 0.0f;
	pidRateZ.ki = 0.0f;	
	
	pidRateX.kd = 0.08f;
	pidRateY.kd = 0.08f;
	pidRateZ.kd = 0.05f;	
	
/////////////�⻷angle PID///////////////////////////
	
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
		//�ڻ�PID���� �ٶ�
//	pidHeightRate.kp = 4.8f;//1.2f; //1.2f
//	pidHeightRate.ki = 0.04f;
//	pidHeightRate.kd = 0.085f;
		//�⻷PID����
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
		//�⻷PID����
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

	//X�ڻ�����PID����  �ٶ�
	
	Flow_SpeedPid_x.kp = 0.610f;//1.5f;//0.610f;//����  0.600f
	Flow_SpeedPid_x.ki = 0.5f;//0.000f;//����
	Flow_SpeedPid_x.kd = 0.4f;//0.400f;//΢��
	
	//X�⻷����PID����  λ��
	
	Flow_PosPid_x.kp = 2.200f;//����  2.000f
	Flow_PosPid_x.ki = 0.000f;//����
	Flow_PosPid_x.kd = 0.006f;//΢��
	
	//////////////////////////////////////////////////////////
	
		//Y�ڻ�����PID���� �ٶ�
	
	Flow_SpeedPid_y.kp = 0.610f;//����
	Flow_SpeedPid_y.ki = 0.5f;//����
	Flow_SpeedPid_y.kd = 0.400f;//΢��
	
	//Y�⻷����PID���� λ�� 
	
	Flow_PosPid_y.kp = 2.200f;//����
	Flow_PosPid_y.ki = 0.00f;//����
	Flow_PosPid_y.kd = 0.006f;//΢��

///////////////////////////////////////////////////////////////////
	


	Command.FlightMode = NORMOL;  //��ʼ��Ϊ��̬����ģʽ
}









