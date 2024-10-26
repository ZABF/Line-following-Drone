#ifndef __ESP32CAM_H__
#define __ESP32CAM_H__

#include "stm32f10x.h"

struct _LineFollow_
{
	int up_x;
	int up_y;
	int mid_x;
	int mid_y;
	int up_fix_x;
	int up_fix_y;
	int mid_fix_x;
	int mid_fix_y;
	float mid_fix_y_cm;
	float mid_y_cm;
	float slope;
	float fix_slope;
};

extern u8 MV_SSI_CNT, MV_SSI;
extern struct _LineFollow_ mv;

extern void MV_Receive(u8 data);
extern void Line_Following(void);

#endif
