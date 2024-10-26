#include "hcsr04.h"
#include "sys.h"
#include "USART2.h"
#include "delay.h"

#define Trig GPIO_Pin_12  //PA12

#define Echo GPIO_Pin_11	//PA11

int temp_flag_hcsr04;
float Distance, Distance_temp;
int t1count;
uint8_t rising_flag=0;				//���ڼ�¼�ж��ź��������ػ����½���
uint32_t number=0;			//��¼��ʱ���жϵĴ���
uint32_t times=0;			//��¼�����źŵĳ���ʱ��


void Hcsr04_Init(void){
	GPIO_InitTypeDef  GPIO_InitSture;
	EXTI_InitTypeDef  EXTI_InitSture;
	NVIC_InitTypeDef  NVIC_InitSture;
	//����ⲿ�жϵĻ���һ��ʹ��AFIO���ù���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);
	
	
	//����IO�˿�
	GPIO_InitSture.GPIO_Mode=GPIO_Mode_Out_PP;   //�������ģʽ
	GPIO_InitSture.GPIO_Pin=Trig;                //��PA12��Trig����
	GPIO_InitSture.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA,&GPIO_InitSture);
	
	GPIO_InitSture.GPIO_Mode=GPIO_Mode_IPD;      //������ģʽ
	GPIO_InitSture.GPIO_Pin=Echo;                //��PA11��Echo����
	GPIO_InitSture.GPIO_Speed=GPIO_Speed_10MHz;  
	GPIO_Init(GPIOA,&GPIO_InitSture);
	
	//�жϺ�11�˿�ӳ��һ��
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource11);
	
	//�ⲿ�ж�����
	EXTI_InitSture.EXTI_Line=EXTI_Line11;
	EXTI_InitSture.EXTI_LineCmd=ENABLE;
	EXTI_InitSture.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitSture.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitSture);
	
	//�ж����ȼ�����
	NVIC_InitSture.NVIC_IRQChannel=EXTI15_10_IRQn;
	NVIC_InitSture.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitSture.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitSture.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitSture);
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line11)!=RESET)
	{
		if(rising_flag==0){				//������
			number = 0; rising_flag = 1;
			TIM_SetCounter(TIM1,0);
			t1count=TIM_GetCounter(TIM1);
			TIM_Cmd(TIM1,ENABLE);
		}else{
			TIM_Cmd(TIM1,DISABLE);
			rising_flag = 0;
			t1count=TIM_GetCounter(TIM1);
			TIM_Cmd(TIM1,DISABLE);
			Distance_temp = t1count/0.58;
//			Distance = Distance_temp;
			Distance += (Distance_temp-Distance) * 0.2;
//			if(Distance < 30) ALL_flag.obstacle = 1;
//			else ALL_flag.obstacle = 0;
//			ALL_flag.obstacle = 1;
			
//			if(Distance_temp>0)
//			{	temp_flag_hcsr04++;
//				Distance += Distance_temp;
//				printf("Distance:%f cm\r\n",Distance);
//				printf("temp_flag_hcsr04:%d \r\n",temp_flag_hcsr04);
//			}
		}
			
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

void Hcsr04_Strat(void)
{
	GPIO_SetBits(GPIOA,Trig);   //��Trig����Ϊ�ߵ�ƽ
	delay_us(20);	             //��������10us����������������ģ�鹤��
	GPIO_ResetBits(GPIOA,Trig); 
	temp_flag_hcsr04 = 0;
	
}


float Hcsr04_Get_Distance(){
	return Distance;
}
