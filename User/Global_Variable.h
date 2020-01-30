#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "stm32f4xx.h"

	extern double map (double x, double in_min, double in_max, double out_min, double out_max);
	extern void resetIMU();
	extern void interruptInit();
	extern void USART2_IRQHandler(void);
	extern void KalibrasiIMU();
	extern void TIM1_CC_IRQHandler(void);
	extern void TIM4_IRQHandler(void);
	extern void TIM3_IRQHandler(void);
	extern void TIM5_IRQHandler(void);
	extern void TIM1_BRK_TIM9_IRQHandler(void);
	extern void TIM8_CC_IRQHandler(void);
	extern void InitPWMIN();
	extern void InitPWMOUT();
	extern void GetPWM();
	extern void InitBmp();
	extern void GetBMP();
	extern void KalibrasiBMP();
	extern void ParsedataGPS(void);
	extern void ParsedataGPS_GNGLL(void);
	extern float dms_dd(float in_coords, char angin);
	extern void GPSAccess(void);
	extern void BateraiMonitor(); 
	extern void Airspeed();
	extern void SendGCS(int waktu);

#endif