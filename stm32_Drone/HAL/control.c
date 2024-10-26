#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
#include "attitude_process.h"
#include "flow.h"
#include "spl06.h"
//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1 
#undef EMERGENT
#define EMERGENT 0

#define REMOTE_THR Remote.thr
#define REMOTE_PITCH Remote.pitch
#define REMOTE_ROLL Remote.roll
#define REMOTE_YAW Remote.yaw
//#define measured FeedBack
#define Expect desired 	

 float Throttle_out; //�ɿ��������ֵ //ң�� + ���� ���ֵ
 int flag_line_follow_last;
 
 u8 flight_mode;
 u8 flight_mode_last;
 int AUX1_temp = 1505;
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //�ṹ�����飬��ÿһ�������һ��pid�ṹ�壬�����Ϳ���������������PID��������  �������ʱ������λpid�������ݣ�����������仰�����þͿ�����
		,&pidHeightRate,&pidHeightHigh,&Flow_SpeedPid_x,&Flow_PosPid_x,&Flow_SpeedPid_y,&Flow_PosPid_y,
};


//�涨 ң�غ�pid��desired�Ĺ�ϵ
//#define ROTATE_TEST
//#define TUNING_BUTTON
//#ifdef NORMAL_MODE
#define Follow_Line
void Mode_Controler(float dt)//0.004 4ms
{
		const float roll_pitch_ratio = 0.04f;
// if obstacle	
		if(vl53lxx.distance < 70){
			ALL_flag.obstacle = 1;
		}else if(vl53lxx.distance > 100){
			ALL_flag.obstacle = 0;
		}			
//		ALL_flag.obstacle = 0;
			
			
		if(Remote.AUX2 < 1700)   //Hovring
		{
			Command.FlightMode = HEIGHT;
			ALL_flag.height_lock = 1;
			ALL_flag.line_follow = 0;
			ALL_flag.line_follow_y =0;
			ALL_flag.line_follow_slope =0;
			Flow_mode_two();       // ң��-����������
			set_flag=0x21;         // OLED���߶���ģʽ��ʾ
		}
		else                     //Line following
		{  
#ifdef NORMAL_MODE
				Command.FlightMode = NORMOL;	//������ͨģʽ
				ALL_flag.height_lock = 0;
				set_flag=0x00;        //OLED��̬ģʽ��ʾ
				
				pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;  //ҡ�˿���
				pidRoll.desired  = -(Remote.roll-1500) *roll_pitch_ratio;  //ҡ�˿���
#endif
				
#ifdef ROTATE_TEST
				Command.FlightMode = HEIGHT;
				ALL_flag.height_lock = 1;
				Flow_mode_two();       // ң��-����������
				set_flag=0x21;         // OLED���߶���ģʽ��ʾ
				
				pidYaw.desired += 72*dt;				
#endif
				
#ifdef Follow_Line_test
				Command.FlightMode = HEIGHT;
				ALL_flag.height_lock = 1;
				ALL_flag.line_follow = 1;
				Flow_mode_two();
		
#endif
#ifdef Follow_Line
				Command.FlightMode = HEIGHT;
				ALL_flag.height_lock = 1;
				ALL_flag.line_follow = 1;
				Flow_mode_two();
//				Angle.yaw = mv.fix_slope;
//				if(mv.fix_slope>0){
////					pidYaw.desired -= 0.1f;
//				}else if(mv.fix_slope<0){
////					pidYaw.desired += 0.1f;
//				}
				pidYaw.desired -= mv.fix_slope * 0.004f;
				
//				if(mv.mid_fix_y_cm > 0){
//						Flow_PosPid_y.desired -= 0.1f;
//				}else if(mv.mid_fix_y_cm < 0){
//						Flow_PosPid_y.desired += 0.1f;
//				}
				Flow_PosPid_y.desired -= mv.mid_fix_y_cm * 0.004f;
				
				ALL_flag.line_follow_y =1;
				 //pixel_flow.loc_y = mv.mid_fix_y_cm;
				/*
				if(mv.up_y > mv.mid_x + 10) pidYaw.desired += (mv.up_y-mv.up_y)/2*dt;	
				
				if(mv.up_y < mv.mid_x - 10) pidYaw.desired -= (mv.mid_y-mv.up_y)/2*dt; //5-24 deg/s				
*/				
#endif
				
			}	
			if( Remote.AUX1 > AUX1_temp)
				pidHeightHigh.desired += 1;
			else if( Remote.AUX1 < AUX1_temp)
				pidHeightHigh.desired -= 1;
			AUX1_temp = Remote.AUX1;
#ifdef TUNING_BUTTON
	//			Flow_PosPid_x.desired = (Remote.AUX1-1505)*2;		//΢��λ�ƿ���
			ALL_flag.line_follow = 1;
			if(Remote.AUX1!=1505)
				Flow_SpeedPid_x.desired = (Remote.AUX1-1505);			//΢���ٶȿ���
				pidHeightHigh.desired = 60 + (Remote.AUX1 -1505);
#endif
							
}



/**************************************************************
 *  Height control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/

//static uint8_t set_high_desired = 0; //���߸߶����趨

int16_t  HIGH_START =100;   //һ�����Ŀ��߶�

uint32_t Control_high = 0; //��ǰ�߶�
/**************************************************************
 *  //�߶ȿ�����     ��ѹ
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void HeightPidControl(float dt)  //�߶ȿ�����     ��ѹ
{
	volatile static uint8_t status=WAITING_1;
		int16_t acc;       //��ǰ��������ļ��ٶ�ֵ
   	int16_t acc_error; //��ǰ���ٶȼ�ȥ�������ٶ���Ϊ�����ƶ��ļ��ٶ�
   static int16_t acc_offset;//�������ٶ�ֵ	 
		static float thr_hold = 0; //����߶�ʱ��¼��ǰ����ֵ
		static uint8_t set_high = 0,High_breaktime;
	
	 if(mini.flow_High<400)
	 {
			Control_high=mini.flow_High;      //���¼���߶�
	 }
	 else
	 {
	 	  Control_high=FlightData.High.bara_height ;	 //������ѹ�߶�
	 }
		 
	
	//----------------------------------------------	
	{ //��ȡ��ֱ�ٶ�����

		  acc = (int16_t)GetAccz(); //��ȡZ��ACC  
		
			if(!ALL_flag.unlock)      //ȡ�þ�̬ACCֵ 
			{
				acc_offset = acc;
			}
			acc_error = acc - acc_offset;	
						
			{//�˴���һ���ٶ���߶ȵĻ����˲� 
				static float last_high;
				pidHeightRate.measured = (pidHeightRate.measured + acc_error * dt)*0.98f + 0.02f*(Control_high - last_high)/dt; //�ٶȻ���Զ�����������Ի����˲��ؼ���ץ���ٶȻ�����
				last_high =  pidHeightHigh.measured = Control_high;  //ʵʱ�߶ȷ������⻷
			}	
	}
	//----------------------------------------------������ֹ����
	if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;
	//----------------------------------------------����
	switch(status)
	{
		case WAITING_1: //��ⶨ��
		  if(ALL_flag.height_lock && ALL_flag.unlock) //���߶Ȳ�����
			{
				LED.status = DANGEROURS;
				status = WAITING_2;
				thr_hold=0; 				        //�����¼����ʱ������
			}
			break;
		case WAITING_2: //����ǰ׼��
			thr_hold = Remote.thr -1000;  //��¼����ʱ������             ����500
		  set_high = 0;
		  pidHeightHigh.desired = HIGH_START;  //�����߶� �趨ֵ				180
 			status = PROCESS_31;
			break; 
		
		case PROCESS_31://���붨��					
				pidUpdate(&pidHeightHigh,dt);    //����PID�������������⻷	������PID	
				pidHeightRate.desired = pidHeightHigh.out;  //�߶Ȼ�����߶��ٶ��趨ֵ
						 					 										 
				pidUpdate(&pidHeightRate,dt); //�ٵ��ø߶��ٶ��ڻ�
				
				if(Remote.thr < 1030)//����
				{		
					pidRest(&pPidObject[6],1);	//�����ǰ�Ķ������ֵ
					pidRest(&pPidObject[7],1);
				}
				
				if(!ALL_flag.height_lock)     //�˳�����
				{
					LED.status = AlwaysOn ;
					status = EXIT_255;
				}
			break;
		case EXIT_255: //�˳�����
			pidRest(&pPidObject[6],1);	//�����ǰ�Ķ������ֵ
			pidRest(&pPidObject[7],1);
			status = WAITING_1;//�ص��ȴ����붨��
			break;
		default:
			status = WAITING_1;
			break;	
	}	
				
}

u32 altHold_Pos_Save = 0,Pos_breaktime = 0,Pos_break_save = 0;

///////////////////////// ң��-����������̬////////////////////////////////////////////////
void Flow_mode_two(void)
{
		const uint16_t DEADBAND=150;
		const float roll_pitch_ratio = 8.00f;	
	
		const float speed_pos = 0.3;
		const float speed_spd = 30;

		if((mini.ok == 1) && (Control_high>10) && (Control_high<400))//�ж��Ƿ���ڹ���  �߶ȸ���20CM �����ſ��Զ���
		{		
			//Flow_SpeedPid_x.desired = (-(Remote.pitch-1500)*0.06f)*roll_pitch_ratio;   //ֱ�ӿ����ٶ� ���ҹص��⻷����
			//Flow_SpeedPid_y.desired = (-(Remote.roll-1500)*0.06f)*roll_pitch_ratio;  					
			
///*��ʱ�����⻷����			
			
			if(Remote.pitch>1700){
				if(ALL_flag.obstacle == 0){
					if(flight_mode == 1)
						//Flow_PosPid_x.desired = -100;//
						Flow_PosPid_x.desired += -speed_pos;	//λ��ҡ��
					else if(flight_mode == 2)
						Flow_SpeedPid_x.desired = 0;		//�ٶ�ҡ��
				}
			}
			
			if(Remote.pitch<1300){
				if(flight_mode == 1)
					//Flow_PosPid_x.desired = 100;
					Flow_PosPid_x.desired += speed_pos;	//λ��ҡ��
				else if(flight_mode == 2)
					Flow_SpeedPid_x.desired = 0;		//�ٶ�ҡ��
			}
			
			if(Remote.roll>1700){
//				if(ALL_flag.line_follow == 0)
					Flow_PosPid_y.desired += -speed_pos;	//λ��ҡ��
//				else
//					Flow_SpeedPid_y.desired = -10;
			}
			
			if(Remote.roll<1300){
//				if(ALL_flag.line_follow == 0)
					Flow_PosPid_y.desired += speed_pos;		//λ��ҡ��
//				else
//					Flow_SpeedPid_y.desired = 10;
			}
			
			if(flight_mode == 1){	//Ѳ��ģʽ x�����ٶ�ң�У�����ʱҪ��λ
				if((Remote.pitch<(1500+DEADBAND))&&(Remote.pitch>(1500-DEADBAND))){
//						Flow_PosPid_x.desired = 0;
				}
				if((Remote.roll>(1500+DEADBAND))&&(Remote.roll<(1500-DEADBAND))){
//						Flow_PosPid_y.desired = 0;
				}
			}		
			if(flight_mode == 2){	//Ѳ��ģʽ x�����ٶ�ң�У�����ʱҪ��λ
				if((Remote.pitch<(1500+DEADBAND))&&(Remote.pitch>(1500-DEADBAND))){
						Flow_SpeedPid_x.desired = -speed_spd;
				}
				if((Remote.roll>(1500+DEADBAND))&&(Remote.roll<(1500-DEADBAND))){
//				Flow_SpeedPid_y.desired = 0;	//Ѳ��ģʽ y����λ��ң�У����в��ۼӼ���
				}
			}			

				pidPitch.desired =LIMIT(Flow_SpeedPid_x.out*0.1,-15,15) ; //��̬�⻷����ֵ
				pidRoll.desired = LIMIT(Flow_SpeedPid_y.out*0.1,-15,15) ; //��̬�⻷����ֵ
			
		}
		else 
		{
			pidPitch.desired = -(Remote.pitch-1500)*0.04f; 
			pidRoll.desired  = -(Remote.roll-1500)*0.04f;  
		}
}

/**************************************************************
 * //λ�ö��������  ����
 * @param[in]  
 * @param[out] 
 * @return     
 ***************************************************************/

void Flow_Pos_Controler(float dt){
	if((mini.ok == 1) && (Control_high>10) && (Control_high<400))//�ж��Ƿ���ڹ���  �߶ȸ���20CM �����ſ��Զ���
	{
// y
//	if(ALL_flag.line_follow_y){
//		//�⻷
//		Flow_PosPid_y.measured =  mv.mid_fix_y_cm;//ʵʱλ�÷���
//		pidUpdate(&Flow_PosPid_y,dt);//λ������PID
//		
//		//����
//		Flow_SpeedPid_y.desired = LIMIT(Flow_PosPid_y.out,-1000,1000);
//		
//		//�ڻ�
//		Flow_SpeedPid_y.measured =  mv.mid_fix_y_cm;//�ٶȷ���
//		pidUpdate(&Flow_SpeedPid_y,dt);//�ٶ�����
//	}else{
		//�⻷
		Flow_PosPid_y.measured = pixel_flow.loc_y;//ʵʱλ�÷���
		pidUpdate(&Flow_PosPid_y,dt);//λ������PID
		
		//����
		Flow_SpeedPid_y.desired = LIMIT(Flow_PosPid_y.out,-1000,1000);
		
		//�ڻ�
		Flow_SpeedPid_y.measured = pixel_flow.loc_y;//�ٶȷ���
		pidUpdate(&Flow_SpeedPid_y,dt);//�ٶ�����
//	}
	
// x	
	// mode select
	if(ALL_flag.height_lock == 1 && ALL_flag.line_follow == 0) flight_mode = 1;
	else if (ALL_flag.line_follow == 1 && vl53lxx.distance > 50+20) flight_mode = 2;//����Ҫ��
	
	if(ALL_flag.obstacle == 1) flight_mode = 1;
	
	if(flight_mode_last != flight_mode){
		mini.flow_x_i = 0;
		pixel_flow.fix_x_i = 0;	
		pixel_flow.out_x_i = 0;
		pixel_flow.x = 0;
		pixel_flow.out_x_i_o = 0;
		pixel_flow.fix_x = 0;
		pidRest(&pPidObject[8],1);
		pidRest(&pPidObject[9],1);
		if(flight_mode == 1){
			Flow_SpeedPid_x.kp = 0.610f;//1.5f;//0.610f;//����  0.600f
			Flow_SpeedPid_x.ki = 0.5f;//0.000f;//����
			Flow_SpeedPid_x.kd = 0.4f;//0.400f;//΢��
		}else if(flight_mode == 2){
			Flow_SpeedPid_x.kp = 1.5f;//1.5f;//0.610f;//����  0.600f
			Flow_SpeedPid_x.ki = 0.5f;//0.000f;//����
			Flow_SpeedPid_x.kd = 0.4f;//0.400f;//΢��
		}
	}
	flight_mode_last = flight_mode;
	// mode exe
	switch (flight_mode){
		case 1:		
			//�⻷
			Flow_PosPid_x.measured = pixel_flow.loc_x;//ʵʱλ�÷���
			pidUpdate(&Flow_PosPid_x,dt);//λ������PID
					
			Flow_SpeedPid_x.desired = LIMIT(Flow_PosPid_x.out,-1000,1000);//����
			
			//�ڻ�
			Flow_SpeedPid_x.measured = pixel_flow.loc_x;//�ٶȷ���
			pidUpdate(&Flow_SpeedPid_x,dt);//�ٶ�����
			break;
		case 2:
			Flow_SpeedPid_x.measured = pixel_flow.loc_xs;//�ٶȷ���
			pidUpdate(&Flow_SpeedPid_x,dt);//�ٶ�����
	
		break;	
	default:
		
		break;
	}
	}else
	{
					mini.flow_x_i = 0;				//������λ
					mini.flow_y_i = 0;				//������λ
					Flow_SpeedPid_x.out = 0;	//��������
					Flow_SpeedPid_y.out = 0;	//��������
		
					Flow_PosPid_y.desired = 0; // ң�ظ�λ
					Flow_PosPid_x.desired = 0; //ң�ظ�λ
		
					Flow_PosPid_x.integ = 0;
					Flow_PosPid_y.integ = 0;
	}

}

/**************************************************************
 * ��̬����
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void FlightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //�ȴ�����
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //׼���������
			pidRest(pPidObject,12); //������λPID���ݣ���ֹ�ϴ�����������Ӱ�챾�ο���

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //����ƫ����
		
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //��ʽ�������
			
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
			pidRateY.measured = MPU6050.gyroY * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
		
			pidPitch.measured = Angle.pitch; 		//�⻷����ֵ ��λ���Ƕ�
		  pidRoll.measured = Angle.roll;			//�⻷����ֵ ��λ���Ƕ�
			pidYaw.measured = Angle.yaw;				//�⻷����ֵ ��λ���Ƕ�
		
		 	pidUpdate(&pidRoll,dt);    //����PID�������������⻷	�����PID		
			pidRateX.desired = pidRoll.out; //���⻷��PID�����Ϊ�ڻ�PID������ֵ��Ϊ����PID
			pidUpdate(&pidRateX,dt);  //�ٵ����ڻ�

		 	pidUpdate(&pidPitch,dt);    //����PID�������������⻷	������PID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //�ٵ����ڻ�

			CascadePID(&pidRateZ,&pidYaw,dt);	//Ҳ����ֱ�ӵ��ô���PID����������
			break;
		case EXIT_255:  						//�˳�����
			pidRest(pPidObject,12);		//��λPID����
			status = WAITING_1;				//���صȴ�����
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;
}


int16_t motor[4];
#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3] 

void MotorControl(void) //�������
{	
	volatile static uint8_t status=WAITING_1;
	
	
	if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: 	     //�ȴ�����	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //������������������Ϊ0
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2: //������ɺ��ж�ʹ�����Ƿ�ʼ����ң�˽��з��п���
			if(Remote.thr>1100)
			{
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			{
				int16_t thr_temp;
								
				if(ALL_flag.height_lock) //����ģʽ�� ����ң����Ϊ�����߶�ʹ��   
				{		
					thr_temp = pidHeightRate.out + (1500 -1000); //�����������Ƕ������ֵ
				}
				else 										 //��������״̬����������ʹ��
				{
					thr_temp = Remote.thr -1000; //�������������������ֵ
				}
				
				if(Remote.thr<1020)		//����̫���ˣ����������  ��Ȼ�ɻ���ת												
				{
					MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4=0;
					break;
				}
				
				if(ALL_flag.take_off == 1){
					MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr_temp,0,900); //��100����̬����
				
					MOTOR1 +=    + pidRateX.out - pidRateY.out - pidRateZ.out;//; ��̬����������������Ŀ�����
					MOTOR2 +=    + pidRateX.out + pidRateY.out + pidRateZ.out ;//;
					MOTOR3 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
					MOTOR4 +=    - pidRateX.out - pidRateY.out + pidRateZ.out;//;
				}	
			}
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //������������������Ϊ0
			status = WAITING_1;	
			break;
		default:
			break;
	}
	
	
//	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);  //����PWM1
//	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);  //����PWM2
//	TIM3->CCR1 = LIMIT(MOTOR3,0,1000);  //����PWM3
//	TIM3->CCR2 = LIMIT(MOTOR4,0,1000);  //����PWM4
////	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);  //����PWM3
////	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);  //����PWM4
	
	#if (FLY_TYPE == 1 || FLY_TYPE == 2)
	
	PWM0 = LIMIT(MOTOR1,0,1000);  //����PWM1
	PWM1 = LIMIT(MOTOR2,0,1000);  //����PWM2
	PWM2 = LIMIT(MOTOR3,0,1000);  //����PWM3
	PWM3 = LIMIT(MOTOR4,0,1000);  //����PWM4
	
#elif (FLY_TYPE >= 3)
	
	PWM0 = 1000 + LIMIT(MOTOR1,0,1000);  //����PWM1
	PWM1 = 1000 + LIMIT(MOTOR2,0,1000);  //����PWM2
	PWM2 = 1000 + LIMIT(MOTOR3,0,1000);  //����PWM3] ;     
	PWM3 = 1000 + LIMIT(MOTOR4,0,1000);  //����PWM4
	
#else
	#error Please define FLY_TYPE!
		
#endif

}

//u8 Landing(float dt){
//	static int cnt_tmp;
//	u8 Landed_flag;
//	if(ALL_flag.unlock == 1&&(mini.flow_High<10||Remote.thr < 1030))						//�ж����� //��Ҫ��Ϊ�жϸ߶�
//	{
//			Landed_flag = 1; //����ɹ�
//			Remote.thr =1000;						//�ر�����		
//	}
//	else
//	{	
//		Landed_flag = 0;
////						cnt = 810;
//		if(cnt_tmp++>(0.1/dt))                 //�������ż�С��ʱ�� //10ms *10 = 100ms						
//		{
//			cnt_tmp=0;
////							printf("Remote.thr: %d  \r\n",Remote.thr);				//����1�Ĵ�ӡ			
////							Remote.thr = 	Remote.thr-LIMIT(mini.flow_High,2,10);   //ͨ��3 ����ͨ����ԭ���Ļ������Զ�������С  �𵽷ɻ������½�
//			
//			pidHeightHigh.desired = LIMIT(pidHeightHigh.desired - LIMIT(mini.flow_High,2,10)/5,0,200);
//		}				
//	}
////	LIMIT(Remote.thr,1000,2000);		
//	return Landed_flag;
//}


/************************************END OF FILE********************************************/ 



