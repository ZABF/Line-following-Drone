#include "esp32cam.h"
#include "sys.h"
#include "USART2.h"
#include "delay.h"
#include "math.h"

u8 MV_SSI_CNT, MV_SSI;

struct _LineFollow_ mv;
	
void MV_Receive(u8 data) //串口2解析
{
	static u8 RxBuffer[32];
	static u8 _data_cnt = 0;
	static u8 state = 0; 
	u8 sum = 0;
	static u8 fault_cnt;
	
				float cpi;
	 
	switch(state)
	{
		case 0:
			if(data==0xF5)  //包头
			{
				state=1;
				RxBuffer[_data_cnt++]=data;
			}else state = 0;
		break;
		case 1:
			if(data==0x02)
			{
				state=2;
				RxBuffer[_data_cnt++]=data;
			}else state = 0;
		break;
		case 2:
			RxBuffer[_data_cnt++]=data;
			if(_data_cnt==4)	
			{
				state = 0;
				_data_cnt = 0;
				MV_SSI_CNT++;//数据频率
				mv.up_y = RxBuffer[2]; 
				mv.up_x = 30;
				mv.mid_y = RxBuffer[3];		
				mv.mid_x = 80;		
				
				mv.up_fix_y += ((mv.up_y - mv.up_fix_y) *0.2);
				mv.mid_fix_y += ((mv.mid_y - mv.mid_fix_y) *0.2);
				
				mv.fix_slope = arctan((mv.up_fix_y-mv.mid_fix_y)/50.0) *57.29578;
				if(mv.mid_fix_y != 0){
//					mv.mid_fix_y_cm = mini.flow_High * tan(((mv.mid_fix_y-60)/3+Angle.roll)*0.01745);
					mv.mid_fix_y_cm = mini.flow_High * tan(((mv.mid_fix_y-60)/1.54+Angle.roll)*0.01745);
					mv.mid_y_cm = mini.flow_High * tan(((mv.mid_fix_y-60)/1.54)*0.01745);
				}else mv.mid_fix_y_cm = 0;
			}
		break;
		default:
			state = 0;
			_data_cnt = 0;
		break;
	}
}


void Line_Following(){
	/*
	if(1){
		Flow_PosPid_y.measured = 
		Flow_PosPid_y.desired = tan(mv.mid_x*ANG_2_RAD/3) * mini.flow_High;
		
		pidYaw.desired ++
		//if(mv.up_x > )
	}	
	*/
}