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

 float Throttle_out; //飞控油门输出值 //遥控 + 定高 输出值
 int flag_line_follow_last;
 
 u8 flight_mode;
 u8 flight_mode_last;
 int AUX1_temp = 1505;
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //结构体数组，将每一个数组放一个pid结构体，这样就可以批量操作各个PID的数据了  比如解锁时批量复位pid控制数据，新手明白这句话的作用就可以了
		,&pidHeightRate,&pidHeightHigh,&Flow_SpeedPid_x,&Flow_PosPid_x,&Flow_SpeedPid_y,&Flow_PosPid_y,
};


//规定 遥控和pid的desired的关系
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
			Flow_mode_two();       // 遥控-光流控制姿
			set_flag=0x21;         // OLED定高定点模式显示
		}
		else                     //Line following
		{  
#ifdef NORMAL_MODE
				Command.FlightMode = NORMOL;	//否则普通模式
				ALL_flag.height_lock = 0;
				set_flag=0x00;        //OLED姿态模式显示
				
				pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;  //摇杆控制
				pidRoll.desired  = -(Remote.roll-1500) *roll_pitch_ratio;  //摇杆控制
#endif
				
#ifdef ROTATE_TEST
				Command.FlightMode = HEIGHT;
				ALL_flag.height_lock = 1;
				Flow_mode_two();       // 遥控-光流控制姿
				set_flag=0x21;         // OLED定高定点模式显示
				
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
	//			Flow_PosPid_x.desired = (Remote.AUX1-1505)*2;		//微调位移控制
			ALL_flag.line_follow = 1;
			if(Remote.AUX1!=1505)
				Flow_SpeedPid_x.desired = (Remote.AUX1-1505);			//微调速度控制
				pidHeightHigh.desired = 60 + (Remote.AUX1 -1505);
#endif
							
}



/**************************************************************
 *  Height control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/

//static uint8_t set_high_desired = 0; //定高高度已设定

int16_t  HIGH_START =100;   //一键起飞目标高度

uint32_t Control_high = 0; //当前高度
/**************************************************************
 *  //高度控制器     气压
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void HeightPidControl(float dt)  //高度控制器     气压
{
	volatile static uint8_t status=WAITING_1;
		int16_t acc;       //当前你飞行器的加速度值
   	int16_t acc_error; //当前加速度减去重力加速度则为上下移动的加速度
   static int16_t acc_offset;//重力加速度值	 
		static float thr_hold = 0; //进入高度时记录当前油门值
		static uint8_t set_high = 0,High_breaktime;
	
	 if(mini.flow_High<400)
	 {
			Control_high=mini.flow_High;      //更新激光高度
	 }
	 else
	 {
	 	  Control_high=FlightData.High.bara_height ;	 //更新气压高度
	 }
		 
	
	//----------------------------------------------	
	{ //获取垂直速度数据

		  acc = (int16_t)GetAccz(); //获取Z轴ACC  
		
			if(!ALL_flag.unlock)      //取得静态ACC值 
			{
				acc_offset = acc;
			}
			acc_error = acc - acc_offset;	
						
			{//此处做一个速度与高度的互补滤波 
				static float last_high;
				pidHeightRate.measured = (pidHeightRate.measured + acc_error * dt)*0.98f + 0.02f*(Control_high - last_high)/dt; //速度环永远是主调，所以互补滤波关键就抓在速度环这里
				last_high =  pidHeightHigh.measured = Control_high;  //实时高度反馈给外环
			}	
	}
	//----------------------------------------------紧急终止飞行
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
	//----------------------------------------------控制
	switch(status)
	{
		case WAITING_1: //检测定高
		  if(ALL_flag.height_lock && ALL_flag.unlock) //定高度并解锁
			{
				LED.status = DANGEROURS;
				status = WAITING_2;
				thr_hold=0; 				        //清除记录定高时的油门
			}
			break;
		case WAITING_2: //定高前准备
			thr_hold = Remote.thr -1000;  //记录定高时的油门             比如500
		  set_high = 0;
		  pidHeightHigh.desired = HIGH_START;  //期望高度 设定值				180
 			status = PROCESS_31;
			break; 
		
		case PROCESS_31://进入定高					
				pidUpdate(&pidHeightHigh,dt);    //调用PID处理函数来处理外环	俯仰角PID	
				pidHeightRate.desired = pidHeightHigh.out;  //高度环输出高度速度设定值
						 					 										 
				pidUpdate(&pidHeightRate,dt); //再调用高度速度内环
				
				if(Remote.thr < 1030)//降落
				{		
					pidRest(&pPidObject[6],1);	//清除当前的定高输出值
					pidRest(&pPidObject[7],1);
				}
				
				if(!ALL_flag.height_lock)     //退出定高
				{
					LED.status = AlwaysOn ;
					status = EXIT_255;
				}
			break;
		case EXIT_255: //退出定高
			pidRest(&pPidObject[6],1);	//清除当前的定高输出值
			pidRest(&pPidObject[7],1);
			status = WAITING_1;//回到等待进入定高
			break;
		default:
			status = WAITING_1;
			break;	
	}	
				
}

u32 altHold_Pos_Save = 0,Pos_breaktime = 0,Pos_break_save = 0;

///////////////////////// 遥控-光流控制姿态////////////////////////////////////////////////
void Flow_mode_two(void)
{
		const uint16_t DEADBAND=150;
		const float roll_pitch_ratio = 8.00f;	
	
		const float speed_pos = 0.3;
		const float speed_spd = 30;

		if((mini.ok == 1) && (Control_high>10) && (Control_high<400))//判断是否存在光流  高度高于20CM 光流才可以定点
		{		
			//Flow_SpeedPid_x.desired = (-(Remote.pitch-1500)*0.06f)*roll_pitch_ratio;   //直接控制速度 并且关掉外环计算
			//Flow_SpeedPid_y.desired = (-(Remote.roll-1500)*0.06f)*roll_pitch_ratio;  					
			
///*暂时接上外环测试			
			
			if(Remote.pitch>1700){
				if(ALL_flag.obstacle == 0){
					if(flight_mode == 1)
						//Flow_PosPid_x.desired = -100;//
						Flow_PosPid_x.desired += -speed_pos;	//位移摇杆
					else if(flight_mode == 2)
						Flow_SpeedPid_x.desired = 0;		//速度摇杆
				}
			}
			
			if(Remote.pitch<1300){
				if(flight_mode == 1)
					//Flow_PosPid_x.desired = 100;
					Flow_PosPid_x.desired += speed_pos;	//位移摇杆
				else if(flight_mode == 2)
					Flow_SpeedPid_x.desired = 0;		//速度摇杆
			}
			
			if(Remote.roll>1700){
//				if(ALL_flag.line_follow == 0)
					Flow_PosPid_y.desired += -speed_pos;	//位移摇杆
//				else
//					Flow_SpeedPid_y.desired = -10;
			}
			
			if(Remote.roll<1300){
//				if(ALL_flag.line_follow == 0)
					Flow_PosPid_y.desired += speed_pos;		//位移摇杆
//				else
//					Flow_SpeedPid_y.desired = 10;
			}
			
			if(flight_mode == 1){	//巡线模式 x轴是速度遥感，居中时要归位
				if((Remote.pitch<(1500+DEADBAND))&&(Remote.pitch>(1500-DEADBAND))){
//						Flow_PosPid_x.desired = 0;
				}
				if((Remote.roll>(1500+DEADBAND))&&(Remote.roll<(1500-DEADBAND))){
//						Flow_PosPid_y.desired = 0;
				}
			}		
			if(flight_mode == 2){	//巡线模式 x轴是速度遥感，居中时要归位
				if((Remote.pitch<(1500+DEADBAND))&&(Remote.pitch>(1500-DEADBAND))){
						Flow_SpeedPid_x.desired = -speed_spd;
				}
				if((Remote.roll>(1500+DEADBAND))&&(Remote.roll<(1500-DEADBAND))){
//				Flow_SpeedPid_y.desired = 0;	//巡线模式 y轴是位移遥感，居中不累加即可
				}
			}			

				pidPitch.desired =LIMIT(Flow_SpeedPid_x.out*0.1,-15,15) ; //姿态外环期望值
				pidRoll.desired = LIMIT(Flow_SpeedPid_y.out*0.1,-15,15) ; //姿态外环期望值
			
		}
		else 
		{
			pidPitch.desired = -(Remote.pitch-1500)*0.04f; 
			pidRoll.desired  = -(Remote.roll-1500)*0.04f;  
		}
}

/**************************************************************
 * //位置定点控制器  光流
 * @param[in]  
 * @param[out] 
 * @return     
 ***************************************************************/

void Flow_Pos_Controler(float dt){
	if((mini.ok == 1) && (Control_high>10) && (Control_high<400))//判断是否存在光流  高度高于20CM 光流才可以定点
	{
// y
//	if(ALL_flag.line_follow_y){
//		//外环
//		Flow_PosPid_y.measured =  mv.mid_fix_y_cm;//实时位置反馈
//		pidUpdate(&Flow_PosPid_y,dt);//位置运算PID
//		
//		//连接
//		Flow_SpeedPid_y.desired = LIMIT(Flow_PosPid_y.out,-1000,1000);
//		
//		//内环
//		Flow_SpeedPid_y.measured =  mv.mid_fix_y_cm;//速度反馈
//		pidUpdate(&Flow_SpeedPid_y,dt);//速度运算
//	}else{
		//外环
		Flow_PosPid_y.measured = pixel_flow.loc_y;//实时位置反馈
		pidUpdate(&Flow_PosPid_y,dt);//位置运算PID
		
		//连接
		Flow_SpeedPid_y.desired = LIMIT(Flow_PosPid_y.out,-1000,1000);
		
		//内环
		Flow_SpeedPid_y.measured = pixel_flow.loc_y;//速度反馈
		pidUpdate(&Flow_SpeedPid_y,dt);//速度运算
//	}
	
// x	
	// mode select
	if(ALL_flag.height_lock == 1 && ALL_flag.line_follow == 0) flight_mode = 1;
	else if (ALL_flag.line_follow == 1 && vl53lxx.distance > 50+20) flight_mode = 2;//可能要改
	
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
			Flow_SpeedPid_x.kp = 0.610f;//1.5f;//0.610f;//比例  0.600f
			Flow_SpeedPid_x.ki = 0.5f;//0.000f;//积分
			Flow_SpeedPid_x.kd = 0.4f;//0.400f;//微分
		}else if(flight_mode == 2){
			Flow_SpeedPid_x.kp = 1.5f;//1.5f;//0.610f;//比例  0.600f
			Flow_SpeedPid_x.ki = 0.5f;//0.000f;//积分
			Flow_SpeedPid_x.kd = 0.4f;//0.400f;//微分
		}
	}
	flight_mode_last = flight_mode;
	// mode exe
	switch (flight_mode){
		case 1:		
			//外环
			Flow_PosPid_x.measured = pixel_flow.loc_x;//实时位置反馈
			pidUpdate(&Flow_PosPid_x,dt);//位置运算PID
					
			Flow_SpeedPid_x.desired = LIMIT(Flow_PosPid_x.out,-1000,1000);//连接
			
			//内环
			Flow_SpeedPid_x.measured = pixel_flow.loc_x;//速度反馈
			pidUpdate(&Flow_SpeedPid_x,dt);//速度运算
			break;
		case 2:
			Flow_SpeedPid_x.measured = pixel_flow.loc_xs;//速度反馈
			pidUpdate(&Flow_SpeedPid_x,dt);//速度运算
	
		break;	
	default:
		
		break;
	}
	}else
	{
					mini.flow_x_i = 0;				//光流复位
					mini.flow_y_i = 0;				//光流复位
					Flow_SpeedPid_x.out = 0;	//抵消惯性
					Flow_SpeedPid_y.out = 0;	//抵消惯性
		
					Flow_PosPid_y.desired = 0; // 遥控复位
					Flow_PosPid_x.desired = 0; //遥控复位
		
					Flow_PosPid_x.integ = 0;
					Flow_PosPid_y.integ = 0;
	}

}

/**************************************************************
 * 姿态控制
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void FlightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //等待解锁
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //准备进入控制
			pidRest(pPidObject,12); //批量复位PID数据，防止上次遗留的数据影响本次控制

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //锁定偏航角
		
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //正式进入控制
			
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //内环测量值 角度/秒
			pidRateY.measured = MPU6050.gyroY * Gyro_G; //内环测量值 角度/秒
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G; //内环测量值 角度/秒
		
			pidPitch.measured = Angle.pitch; 		//外环测量值 单位：角度
		  pidRoll.measured = Angle.roll;			//外环测量值 单位：角度
			pidYaw.measured = Angle.yaw;				//外环测量值 单位：角度
		
		 	pidUpdate(&pidRoll,dt);    //调用PID处理函数来处理外环	横滚角PID		
			pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
			pidUpdate(&pidRateX,dt);  //再调用内环

		 	pidUpdate(&pidPitch,dt);    //调用PID处理函数来处理外环	俯仰角PID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //再调用内环

			CascadePID(&pidRateZ,&pidYaw,dt);	//也可以直接调用串级PID函数来处理
			break;
		case EXIT_255:  						//退出控制
			pidRest(pPidObject,12);		//复位PID参数
			status = WAITING_1;				//返回等待解锁
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
}


int16_t motor[4];
#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3] 

void MotorControl(void) //电机控制
{	
	volatile static uint8_t status=WAITING_1;
	
	
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: 	     //等待解锁	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2: //解锁完成后判断使用者是否开始拨动遥杆进行飞行控制
			if(Remote.thr>1100)
			{
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			{
				int16_t thr_temp;
								
				if(ALL_flag.height_lock) //定高模式下 油门遥杆作为调整高度使用   
				{		
					thr_temp = pidHeightRate.out + (1500 -1000); //输出给电机的是定高输出值
				}
				else 										 //正常飞行状态，油门正常使用
				{
					thr_temp = Remote.thr -1000; //输出给电机的是油门输出值
				}
				
				if(Remote.thr<1020)		//油门太低了，则限制输出  不然飞机乱转												
				{
					MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4=0;
					break;
				}
				
				if(ALL_flag.take_off == 1){
					MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr_temp,0,900); //留100给姿态控制
				
					MOTOR1 +=    + pidRateX.out - pidRateY.out - pidRateZ.out;//; 姿态输出分配给各个电机的控制量
					MOTOR2 +=    + pidRateX.out + pidRateY.out + pidRateZ.out ;//;
					MOTOR3 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
					MOTOR4 +=    - pidRateX.out - pidRateY.out + pidRateZ.out;//;
				}	
			}
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
			status = WAITING_1;	
			break;
		default:
			break;
	}
	
	
//	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);  //更新PWM1
//	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);  //更新PWM2
//	TIM3->CCR1 = LIMIT(MOTOR3,0,1000);  //更新PWM3
//	TIM3->CCR2 = LIMIT(MOTOR4,0,1000);  //更新PWM4
////	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);  //更新PWM3
////	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);  //更新PWM4
	
	#if (FLY_TYPE == 1 || FLY_TYPE == 2)
	
	PWM0 = LIMIT(MOTOR1,0,1000);  //更新PWM1
	PWM1 = LIMIT(MOTOR2,0,1000);  //更新PWM2
	PWM2 = LIMIT(MOTOR3,0,1000);  //更新PWM3
	PWM3 = LIMIT(MOTOR4,0,1000);  //更新PWM4
	
#elif (FLY_TYPE >= 3)
	
	PWM0 = 1000 + LIMIT(MOTOR1,0,1000);  //更新PWM1
	PWM1 = 1000 + LIMIT(MOTOR2,0,1000);  //更新PWM2
	PWM2 = 1000 + LIMIT(MOTOR3,0,1000);  //更新PWM3] ;     
	PWM3 = 1000 + LIMIT(MOTOR4,0,1000);  //更新PWM4
	
#else
	#error Please define FLY_TYPE!
		
#endif

}

//u8 Landing(float dt){
//	static int cnt_tmp;
//	u8 Landed_flag;
//	if(ALL_flag.unlock == 1&&(mini.flow_High<10||Remote.thr < 1030))						//判断油门 //需要改为判断高度
//	{
//			Landed_flag = 1; //降落成功
//			Remote.thr =1000;						//关闭油门		
//	}
//	else
//	{	
//		Landed_flag = 0;
////						cnt = 810;
//		if(cnt_tmp++>(0.1/dt))                 //控制油门减小的时间 //10ms *10 = 100ms						
//		{
//			cnt_tmp=0;
////							printf("Remote.thr: %d  \r\n",Remote.thr);				//串口1的打印			
////							Remote.thr = 	Remote.thr-LIMIT(mini.flow_High,2,10);   //通道3 油门通道在原来的基础上自动慢慢减小  起到飞机慢慢下降
//			
//			pidHeightHigh.desired = LIMIT(pidHeightHigh.desired - LIMIT(mini.flow_High,2,10)/5,0,200);
//		}				
//	}
////	LIMIT(Remote.thr,1000,2000);		
//	return Landed_flag;
//}


/************************************END OF FILE********************************************/ 



