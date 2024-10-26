#ifndef __HCSR04_H__
#define __HCSR04_H__


extern void Hcsr04_Init(void);
void EXTI15_10_IRQHandler(void);
extern void Hcsr04_Strat(void);
extern float Hcsr04_Get_Distance(void);
	
#endif

