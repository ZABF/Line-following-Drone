#ifndef _REMOTE_H
#define _REMOTE_H
#include "stm32f10x.h"

extern u16 test_flag,set_flag;
extern void RC_Analy(float dt); 
extern void Rc_Connect(float dt);
extern u8 Descending(float dt);
extern u8 Climbing(float dt, float height);
#endif

