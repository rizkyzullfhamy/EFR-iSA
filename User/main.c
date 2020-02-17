//############################################ Include Library ##############################################//
	/* Include core modules */
	#include "Global_Variable.h"
	#include "stm32f4xx.h"
	/* Include my libraries here */
	#include "defines.h"
	#include "string.h"
	#include "stdio.h"
	#include "stdlib.h"
	#include "tm_stm32f4_timer_properties.h"
	#include "tm_stm32f4_pwmin.h"
	#include "tm_stm32f4_pwm.h"
	#include "tm_stm32f4_servo.h"
	#include "tm_stm32f4_bmp180.h"
	#include "tm_stm32f4_delay.h"
	#include "tm_stm32f4_usart.h"
	#include "tm_stm32f4_usart_dma.h"
	#include "tm_stm32f4_mpu6050.h"
	#include "tm_stm32f4_adc.h"
	#include "tm_stm32f4_exti.h"
	#include "arm_math.h"
	
		 	union Floating{
	uint8_t bytes[4];
	float value;	
	};
	union Doublee{
	uint8_t bytes[8];
	double value1;	
	};
	
//################ PID CONTROL SYSTEM #########################//

	float TimeSampling 	= 0.01;
	float KP_PITCH 			= 1.2;     
  float KI_PITCH 			= 0.9;       
  float KD_PITCH 			= 1.0;   
	
	float KP_ROLL 			= 1.2;    
  float KI_ROLL 			= 0.9;        
  float KD_ROLL 			= 1.0;  
	
	float KP_YAW 				= 1.2;     
  float KI_YAW 				= 0.9;        
  float KD_YAW	 			= 1.0;   
	
	float ErrPitch, ErrRoll, ErrYaw;
	float LastErrPitch, LastErrRoll, LastErrYaw;
	float IntegralErrPitch, IntegralErrRoll, IntegralErrYaw;
	float DerivatifErrPitch, DerivatifErrRoll, DerivatifErrYaw;
	 
	float PropotionalPitch, PropotionalRoll, PropotionalYaw;
	float IntegralPitch, IntegralRoll, IntegralYaw;
	float DerivatifPitch, DerivatifRoll, DerivatifYaw;
	 
	float PID_Pitch, PID_Roll, PID_Yaw;
	float OutPidPitch, OutPidRoll, OutPidYaw;
	
//####################################################################//	
//#########DEKLARASI VARIABEL GLOBAL##########//
//BUFFER
	 char buffer[50];
	 char bufferkal[100];
	 unsigned long timer1=0, timer2=0, timer3=0;
// MODE
	 char serial3[100];
	 int takeoff_flag=0, landing_flag=0;
	 int fmode;
//VAR-GPS
	 char gps[200];
	 char serial4[200];
	 int cek2,a2,b2,flag2,nomor_parsing2=0,ke2=0;
	 float data_lat,data_longi;
	 int data_time;
	 char lat[15],lat_char[15],longi[15],longi_char[15],valid[15],time[15];  
	 float latitude,longitude,latitude_zero,longitude_zero,selisih_gps_lat,selisih_gps_long;
	 union Floating currentLat, currentLong;
	 union Doublee currentLat1, currentLong1;		
//VAR-AIRSPEED & BateraiMonitor
	 const float offset = 0.6; // The offset voltage is 1.0V when there is no pressure difference.
	 const float sensitivity = 0.6; // The sensitivity is 1.0V per kPa for the sensor.
	 int rawADC, rawADC1;
	 union Floating baterai;
	 float voltage;
	 float pressure;
	 union Floating windspeed;
//IMU
	 float YPR[3];
	 unsigned char Re_buf[8],counter=0;
	 unsigned char Re_buf[8];
	 unsigned char sign=0;
	 union Floating dataYaw,dataPitch,dataRoll;
	 float dataYaw_Zero, dataPitch_Zero, dataRoll_Zero;
	 int resetKalIMU=0;
//TIM IN
	 double input1, input2, input3, input4, input5, input6;
	 double input1_gui, input2_gui, input3_gui, input4_gui, input5_gui, input6_gui;
	 TM_PWMIN_t PWMIN_TIM1;
	 TM_PWMIN_t PWMIN_TIM4;
	 TM_PWMIN_t PWMIN_TIM3;
	 TM_PWMIN_t PWMIN_TIM5;
	 TM_PWMIN_t PWMIN_TIM9;
	 TM_PWMIN_t PWMIN_TIM8;
//TIM OUT
	 int setServo = 90;
	 int BRUSHLESS_one;
	 int BRUSHLESS_one_Decrement;
	 TM_PWM_TIM_t TIM2_Data;
	 TM_SERVO_t Servo1, Servo2, Servo3;
	 NVIC_InitTypeDef NVIC_InitStructure;
// BMP 180
	 TM_BMP180_t BMP180_Data;
	 float data_altitude_zero;
	 union Floating altitude;
	 int data_altitude;
	 int resetKalBMP=0;
	 
	 
	 
//######## DEKLARASI FUNGSI ########### //
double map (double x, double in_min, double in_max, double out_min, double out_max){
	return (x - in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
//IMU
void resetIMU(){
	TM_USART_Puts(USART1,"Masuk IMU");
	TM_USART_Putc(USART2, 0xA5);
	TM_USART_Putc(USART2, 0x55);
	Delayms(170);
	TM_USART_Putc(USART2,0xA5);
	TM_USART_Putc(USART2,0x54);
  Delayms(170);
	TM_USART_Putc(USART2,0xA5);
	TM_USART_Putc(USART2,0x52);
	TM_USART_Puts(USART1,"keluar IMU");
}
void interruptInit(){
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void USART2_IRQHandler(void){
		if(USART_GetITStatus(USART2, USART_IT_RXNE)) {
			Re_buf[counter] = USART_ReceiveData(USART2);
			if(counter == 0 && Re_buf[0] != 0xAA) return;
			counter++;
			if (counter == 8) {
				counter = 0;
				sign = 1;
			}
		}
		if(sign == 1 &&Re_buf[0] == 0xAA && Re_buf[7] == 0x55) { sign = 0;
			YPR[0] = (Re_buf[1] << 8 | Re_buf[2]) * 0.01f;
			YPR[1] = (Re_buf[3] << 8 | Re_buf[4]) * 0.01f;
			YPR[2] = (Re_buf[5] << 8 | Re_buf[6]) * 0.01f;
			
			//Setelah nilai IMU 179 lalu melompat ke 475
			//range sudut 0-179 lalu 475-655 
			if (YPR[0] > 179)		{dataYaw.value = (655-YPR[0]);
																if(YPR[0]>475){dataYaw.value=dataYaw.value;} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else									dataYaw.value = -1* YPR[0];
			
			if (YPR[1] > 179)		{dataPitch.value = (655-YPR[1]);
															if(YPR[1]>475){dataPitch.value=dataPitch.value;} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else									dataPitch.value = -1* YPR[1];
			
			if (YPR[2] > 179)		{dataRoll.value = (655-YPR[2]);
																if(YPR[2]>475){dataRoll.value=-dataRoll.value;} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else									dataRoll.value = YPR[2];	
		}
}
void KalibrasiIMU(){
	while(1){
		dataYaw_Zero 		= dataYaw_Zero + dataYaw.value;
		dataPitch_Zero  = dataPitch_Zero + dataPitch.value;
		dataRoll_Zero 	= dataRoll_Zero + dataRoll.value;
		resetKalIMU 		= resetKalIMU + 1;
		sprintf(bufferkal,"\n\rBISMILLAH KALIBRASI IMU...%d %%", resetKalIMU );
		Delayms(50);
		TM_USART_Puts(USART1, bufferkal);
		if (resetKalIMU == 100){
			dataYaw_Zero   = dataYaw_Zero   / 100.0;
			dataPitch_Zero = dataPitch_Zero / 100.0;
			dataRoll_Zero  = dataRoll_Zero  / 100.0;
			TM_USART_Puts(USART1,"\n\rALHAMDULILLAH KALIBRASI IMU BERHASIL...");
			Delayms(1000);
			resetKalIMU=0;
			break;
		}
	}
}
//PWM INPUT & OUTPUT (INTERRUPT TIMER)
void TIM1_CC_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM1);
}
void TIM4_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM4);
}
void TIM3_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM3);
}
void TIM5_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM5);
}
void TIM1_BRK_TIM9_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM9);
}
void TIM8_CC_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM8);
}
void InitPWMIN(){
	TM_PWMIN_InitTimer(TIM1, &PWMIN_TIM1, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_1, 50, TIM1_CC_IRQn);       //PA8
	TM_PWMIN_InitTimer(TIM4, &PWMIN_TIM4, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_2, 50, TIM4_IRQn);          //PD12
  TM_PWMIN_InitTimer(TIM3, &PWMIN_TIM3, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_2, 50, TIM3_IRQn);          //PB4
  TM_PWMIN_InitTimer(TIM5, &PWMIN_TIM5, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_1, 50, TIM5_IRQn);          //PA0
	TM_PWMIN_InitTimer(TIM9, &PWMIN_TIM9, TM_PWMIN_Channel_2, TM_PWMIN_PinsPack_2, 50, TIM1_BRK_TIM9_IRQn); //PE6
	TM_PWMIN_InitTimer(TIM8, &PWMIN_TIM8, TM_PWMIN_Channel_2, TM_PWMIN_PinsPack_1, 50, TIM8_CC_IRQn );			//PC7
}
void InitPWMOUT(){
	//Init TimerPWM TIM2
	TM_PWM_InitTimer(TIM2, &TIM2_Data, 50);
	//Brushless
	TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_2);    //PA5
	TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_3);    //PA15
	//Servo
	TM_SERVO_Init(&Servo1, TIM2, TM_PWM_Channel_2, TM_PWM_PinsPack_1);			//PA1 ROLL(EILERON)
	TM_SERVO_Init(&Servo2, TIM2, TM_PWM_Channel_3, TM_PWM_PinsPack_1);			//PA2 YAW(RUDER)
	TM_SERVO_Init(&Servo3, TIM2, TM_PWM_Channel_4, TM_PWM_PinsPack_1);			//PA3 PITCH(ELEVATOR) 
	// Set Nilai awal servo = 90 Degrees //
	TM_SERVO_SetDegrees(&Servo1, setServo);
	TM_SERVO_SetDegrees(&Servo2, setServo);
	TM_SERVO_SetDegrees(&Servo3, setServo);
}
void GetPWM(){
	if(TM_DELAY_Time() - timer2 >= 50){
			TM_PWMIN_Get(&PWMIN_TIM1);
			TM_PWMIN_Get(&PWMIN_TIM4);
			TM_PWMIN_Get(&PWMIN_TIM3);
			TM_PWMIN_Get(&PWMIN_TIM5);
			TM_PWMIN_Get(&PWMIN_TIM9);
			TM_PWMIN_Get(&PWMIN_TIM8);
		
			input1 = PWMIN_TIM1.DutyCycle;
			input2 = PWMIN_TIM4.DutyCycle;
			input3 = PWMIN_TIM3.DutyCycle;
			input4 = PWMIN_TIM5.DutyCycle;
			input5 = PWMIN_TIM9.DutyCycle;
			input6 = PWMIN_TIM8.DutyCycle;
		
		  input1 = map(input1, 5, 10, 10, 170);
		  input2 = map(input2, 5, 10, 170, 10);
		  input3 = map(input3, 5, 10, 1000, 2000);
		  input4 = map(input4, 5, 10, 10, 170);
		
		  if (input1 < 10)  {input1 = 10;  }					// ROLL(EILERON)
			if (input1 > 170) {input1 = 170; }
			
			if (input2 < 10)  {input2 = 10;  }          // RUDER(YAW)
			if (input2 > 170) {input2 = 170; }
			
			if (input3 < 1000)  {input3 = 1000;  }       //BRUSHLEESS
			if (input3 > 2000)  {input3 = 2000;  }
			
			if (input4 < 10)  {input4 = 10;  }				  // ELEVTOR(PITCH
			if (input4 > 170) {input4 = 170; }
			timer2=TM_DELAY_Time();
	}
}
// BMP ALTITUDE 
void InitBmp(){
	if (TM_BMP180_Init(&BMP180_Data) == TM_BMP180_Result_Ok ) {
		//TM_USART_Puts(USART1, "\n\rBMP_OK READY TO USE");
	}
	else {
				TM_USART_Puts(USART1, "\n\rBMP_ERROR");
				while(1);
	}
}
void GetBMP(){
	if(TM_DELAY_Time() -timer3>= 200) {
			TM_BMP180_StartTemperature(&BMP180_Data);
			Delay(BMP180_Data.Delay);
			TM_BMP180_ReadTemperature(&BMP180_Data);
			TM_BMP180_StartPressure(&BMP180_Data, TM_BMP180_Oversampling_UltraHighResolution);
			Delay(BMP180_Data.Delay);
		
			/* Read pressure value */
      TM_BMP180_ReadPressure(&BMP180_Data);
			data_altitude=BMP180_Data.Altitude-data_altitude_zero;
			altitude.value = data_altitude;
			timer3=TM_DELAY_Time();
	}
}
void KalibrasiBMP(){
	while(1){
		TM_BMP180_StartTemperature(&BMP180_Data);
		Delay(BMP180_Data.Delay);
		
		//Read Temperature
		TM_BMP180_ReadTemperature(&BMP180_Data);
		// Start Pressure Ultra High Resulution
		TM_BMP180_StartPressure(&BMP180_Data, TM_BMP180_Oversampling_UltraHighResolution);
		Delay(BMP180_Data.Delay);
		//Read Pressure
		TM_BMP180_ReadPressure(&BMP180_Data);
		
		data_altitude_zero = data_altitude_zero + BMP180_Data.Altitude;
		resetKalBMP =resetKalBMP+1;
		sprintf(bufferkal,"\n\rBISMILLAH KALIBRASI BMP...%d %%", resetKalBMP );
		TM_USART_Puts(USART1, bufferkal);
		
	if(resetKalBMP == 100){
		data_altitude_zero = data_altitude_zero /100.0;
		TM_USART_Puts(USART1,"\n\rALHAMDULILLAH KALIBRASI BMP BERHASIL...");
		Delayms(1000);
		resetKalBMP=0;
		break;
		}
	}
}

// GPS LAT LONG
void ParsedataGPS_GNGLL(void){
				if (flag2==2)
				{
					for(cek2=a2; cek2<sizeof(gps);cek2++)
					{
							if(gps[cek2] == ',')
								{
										nomor_parsing2++;
										ke2 = 0;
										continue;
								}
							else
								{				
									if (nomor_parsing2 == 1)
									{
										lat[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 2)
									{
									  lat_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 3)
									{
									  longi[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 4)
									{
										longi_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 5)
									{
										time[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 6)
									{
										valid[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 > 6)
									{
										nomor_parsing2 = 0;
										flag2 = 0;
										break;
									}
									ke2++;	
								}
						}
				}
			data_time	=	atoi(time);
			data_lat = atof(lat);
			data_longi = atof(longi);
}
void ParsedataGPS_GNGGA(void){
				if (flag2==2)
				{
					for(cek2=a2; cek2<sizeof(gps);cek2++)
					{
							if(gps[cek2] == ',')
								{
										nomor_parsing2++;
										ke2 = 0;
										continue;
								}
							else
								{				
									if (nomor_parsing2 == 1)
									{
										time[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 2)
									{
									  lat[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 3)
									{
									  lat_char[ke2] = gps[cek2]; 
									}
									else if (nomor_parsing2 == 4)
									{
										longi[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 5)
									{
										longi_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 6)
									{
										valid[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 > 6)
									{
										nomor_parsing2 = 0;
										flag2 = 0;
										break;
									}
									ke2++;	
								}
						}
				}
			data_time	=	atoi(time);
			data_lat = atof(lat);
			data_longi = atof(longi);
}
void ParsedataGPS_GNRMC(void){
				if (flag2==2)
				{
					for(cek2=a2; cek2<sizeof(gps);cek2++)
					{
							if(gps[cek2] == ',')
								{
										nomor_parsing2++;
										ke2 = 0;
										continue;
								}
							else
								{				
									if (nomor_parsing2 == 1)
									{
										time[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 2)
									{
									  valid[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 3)
									{
									  lat[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 4)
									{
										lat_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 5)
									{
										longi[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 6)
									{
										longi_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 > 6)
									{
										nomor_parsing2 = 0;
										flag2 = 0;
										break;
									}
									ke2++;	
								}
						}
				}
			data_time	=	atoi(time);
			data_lat = atof(lat);
			data_longi = atof(longi);
}
void GPS_Validasi(void){
				for (cek2 = 0; cek2 < sizeof(gps); cek2++){
					if(('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('G' == gps[cek2+3]) && ('L' == gps[cek2+4]) && ('L' == gps[cek2+5]) && (flag2 == 0))
					{
						if(flag2 == 0) {
							a2 = cek2+6;
							flag2 = 2;
						}
						ParsedataGPS_GNGLL();
					}
					else if (('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('G' == gps[cek2+3]) && ('G' == gps[cek2+4]) && ('A' == gps[cek2+5]) && (flag2 == 0))
					{
						if(flag2 == 0) {
							a2 = cek2+6;
							flag2 = 2;
						}
						ParsedataGPS_GNGGA();
					}
					else if(('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('R' == gps[cek2+3]) && ('M' == gps[cek2+4]) && ('C' == gps[cek2+5]) && (flag2 == 0))
					{
							if(flag2 == 0) {
							a2 = cek2+6;
							flag2 = 2;
					}
							ParsedataGPS_GNRMC();
				}
			}
}
float dms_dd(float in_coords, char angin){
	float f=in_coords;
	char arah=angin;
//	float per;
	int firstdig=((int)f)/100;
	float nextdig=f-(float)(firstdig*100);
		
	if('W'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*-1.0);
		return final;
	}
		
	if('N'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*1.0);
		return final;
	}
	
	if('E'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*1.0);
		return final;
	}
	
	if('S'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*-1.0);
		return final;	
	}
// W-,N+,E+,S-
	}

void GPSAccess(void){
			if (TM_USART_Gets(USART3, serial4, sizeof(serial4))) {		// GPS 
			strcpy(gps, serial4);
			GPS_Validasi();
			currentLat1.value1  	= dms_dd(data_lat,lat_char[0]);
			currentLong1.value1 	= dms_dd(data_longi,longi_char[0]);
			currentLat.value 			= currentLat1.value1;
			currentLong.value			= currentLong1.value1;
		}
}

// BateraiMonitor & Airspeed
void BateraiMonitor(){
	rawADC  = TM_ADC_Read(ADC1, ADC_Channel_14);
	baterai.value = (rawADC * 3.04985) * 0.00433641;
}
void Airspeed(){
	rawADC1  = TM_ADC_Read(ADC1, ADC_Channel_14);
	voltage  = (float) rawADC / 4095.0 * 3.0; // Voltage at Arduino pin. Range is 5V, 10 bits.
	pressure = (voltage - offset) / sensitivity;  // differential pressure in kPa
	
	if (pressure <0.0) pressure = 0;
	windspeed.value =  sqrt (2.0 * pressure / 1.2 ) * 1.943844;
}
// MODE
void ParseMode(void){
		/*if (TM_USART_Gets(USART1, serial3, sizeof(serial3))) {
			if (serial3[0] =='A') { takeoff_flag = 1; }
			if (serial3[0] =='M') { takeoff_flag = 0; }
		}
	*/
			switch(TM_USART_Getc(USART1))
			{
				case 0xAA:						// AKTIF TAKEOFF MODE
					landing_flag = 0;
					takeoff_flag = 1;
					break;
				case 0xBB:						//NON AKTIF SEMUA MODE TAKE OFF AND LANDING 
					landing_flag = 0;
					takeoff_flag = 0;
					break;
				case 0xCC:						// AKTIF LANDING MODE
					takeoff_flag = 0;
					landing_flag = 1;
					break;
			}
}

void TakeOff_Mode(void) {
	TM_SERVO_SetDegrees(&Servo1, OutPidRoll);   	//rollPID();
	TM_SERVO_SetDegrees(&Servo3, input2);	      	//PITCH//ELEVATOR
	TM_SERVO_SetDegrees(&Servo2, input4);	     	 //YAW//RUDDER
	input3=1900;
	BRUSHLESS_one=input3;
	BRUSHLESS_one=input3;
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one); //BRUSHLESS
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one); //BRUSHLESS
}
void Loiter_Mode(void){
	TM_SERVO_SetDegrees(&Servo1, OutPidRoll);   //rollPID();
	TM_SERVO_SetDegrees(&Servo3, OutPidPitch);   //pitchPID();
	
  TM_SERVO_SetDegrees(&Servo2, 170);
	
	BRUSHLESS_one=input3;
	BRUSHLESS_one=input3;
	
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one); //BRUSHLESS
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one); //BRUSHLESS
}
void FBWA_Mode(void){
	TM_SERVO_SetDegrees(&Servo1, OutPidRoll);     //rollPID();
	TM_SERVO_SetDegrees(&Servo3, OutPidRoll);     //pitchPID();
	TM_SERVO_SetDegrees(&Servo2, OutPidYaw);     //yawPID();
 
	BRUSHLESS_one=input3;
	BRUSHLESS_one=input3;
	
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one); //BRUSHLESS
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one); //BRUSHLESS
}
void Manual_Mode(void) {
	TM_SERVO_SetDegrees(&Servo1, input1);	//ROLL//EILERON       
	TM_SERVO_SetDegrees(&Servo3, input2);	//PITCH//ELEVATOR
	TM_SERVO_SetDegrees(&Servo2, input4);	//YAW//RUDDER
	
	BRUSHLESS_one=input3;
	BRUSHLESS_one=input3;
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one); //BRUSHLESS
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one); //BRUSHLESS
}
void Auto_Mode(void){
	TM_SERVO_SetDegrees(&Servo1, OutPidRoll);   	 //rollPID();
	TM_SERVO_SetDegrees(&Servo3, OutPidPitch);     //pitchPID();
	TM_SERVO_SetDegrees(&Servo2, OutPidYaw);			// YawPID();
}
void Landing_Mode(void){
	BRUSHLESS_one_Decrement= 1350;
	while(1){
			BRUSHLESS_one_Decrement -= 5;
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one_Decrement); //BRUSHLESS
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, BRUSHLESS_one_Decrement); //BRUSHLESS
			TM_SERVO_SetDegrees(&Servo1, OutPidRoll);     //rollPID();
			TM_SERVO_SetDegrees(&Servo3, OutPidPitch);     //pitchPID();
			TM_SERVO_SetDegrees(&Servo2, OutPidYaw);			//yawPid();
			sprintf(bufferkal,"\n\rPROSES DEC : %d %%", BRUSHLESS_one_Decrement );
			TM_USART_Puts(USART1, buffer);
			Delayms(500);
			if (BRUSHLESS_one_Decrement == 1100 || landing_flag == 0 ){
					TM_USART_Puts(USART1,"\n\rALHAMDULILLAH LANDING TROTHLLE SLESAI");
					Delayms(1000);
					break;
				}
			}
	//TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, HasilPidAl); //BRUSHLESS ALTITUDE_PID
	//TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, HasilPidAl); //BRUSHLESS ALTITUDE_PID
}
void Mode_Flag(void){
	// #################### RUN FLIGHT MODE ################### //	
			if (takeoff_flag == 0 || landing_flag == 0) {
				if (input5 <6 ) { 
				Manual_Mode(); 
				fmode = 0; 
				}
				if (input5 >6  && input5 <9 ) {
					if(input1 >=95 || input1 <=85 || input2 >=95 || input2 <85 || input4 >=95 || input4 <=85) 
						Manual_Mode();
					else 
						FBWA_Mode();
						fmode = 1;
				}
				if (input5 > 8 ) { 
					Loiter_Mode(); 
					fmode=2;}
				}
			//MODE TAKEOFF
			if (takeoff_flag == 1){
				TakeOff_Mode();
				fmode = 3;
			if (data_altitude == 2) 
				takeoff_flag = 0;
			}
			//MODE LANDING
			if(landing_flag == 1){
				Landing_Mode();
				fmode = 4;
				if (data_altitude <= 0)
					 FBWA_Mode(); 
					landing_flag = 0; 	//Aktifkan saat terbang karna baca altitude diatas
				}
}
// Send GCS
void SendGCS(int waktu){
	
		if (TM_DELAY_Time() -timer1 >= waktu){
		/*	sprintf(buffer, "\n\rROLL: %.2f ,PITCH: %.2f ,YAW: %.2f ,ALTITUDE: %d ,LONGITUDE: %f ,LATITUDE: %f ,IN1: %.2f ,IN2: %.2f ,IN3: %.2f ,IN4: %.2f ,IN5: %.2f ,IN6: %.2f ",
								dataRoll-dataRoll_Zero,
								dataPitch-dataPitch_Zero,
								dataYaw-dataYaw_Zero, 
								data_altitude,
								currentLong,
								currentLat,
								input1, 
								input2, 
								input3, 
								input4, 
								input5, 
								input6);		
		*/
				/*sprintf(buffer, "%.2f,%.2f,%.2f,%.2f,%d,%f,%f\n\r",
									//dataYaw-dataYaw_Zero,
									//dataPitch-dataPitch_Zero,
									//dataRoll-dataRoll_Zero,
									windspeed,
									data_altitude,
									currentLat,
									currentLong);
									*/
				//######################## AKTIFKAN UNTUK CHECK INPUT PWM REMOTE #####################//
				//sprintf(buffer, "IN1: %.2f IN2: %.2f IN3: %.2f IN4: %.2f IN5: %.2f IN6: %.2f\n\r", 
				//input1, input2, input3, input4, input5, input6);
				//sprintf(buffer,"%s\n\r", serial4);		//dummy gps
				TM_USART_Send(USART1, (uint8_t *)buffer, strlen(buffer));
				timer1 = TM_DELAY_Time();
		}
}

void SendGCS4Bit (int waktu){
		if (TM_DELAY_Time() -timer1 >= waktu){
		
			//FORMAT GCS 4BIT TRANSFER DATA MODE ENKRIP  //	
				TM_USART_Putc(USART1, fmode);
				TM_USART_Send(USART1, dataYaw.bytes, 4);
				TM_USART_Send(USART1, dataPitch.bytes, 4);
				TM_USART_Send(USART1, dataRoll.bytes, 4);
				TM_USART_Send(USART1, windspeed.bytes, 2);
				TM_USART_Send(USART1, altitude.bytes, 4);
				TM_USART_Send(USART1, currentLat1.bytes, 8);
				TM_USART_Send(USART1, currentLong1.bytes, 8);
				TM_USART_Send(USART1, baterai.bytes, 4);
				TM_USART_Putc(USART1,0x0A);
		
			//FORMAT SEND TO ANTENNA TRACKER
				//sprintf(buffer,"t,%d,%.2f,%.2f,%.2f,%.2f,%d,%f,%f,%.2f\n\r",
				//				fmode,error2,error3,error1,windspeed.value,
				//				data_altitude,currentLat1.value1,currentLong1.value1,baterai.value);
								
				//TM_USART_Send(USART1, (uint8_t *)buffer, strlen(buffer));
				timer1 = TM_DELAY_Time();
			}
}
//#####################################################################################################################################################################################################################################################//

// MAIN PROGRAM 
int main(void){
	SystemInit(); 		// Initialize system //
	TM_DELAY_Init(); 	// Initialize delay //
	
	// Initialize USART1, 115200 baud, TX: PA9 RX:PA10     		    
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 57600); 		//(TELEMETRY) 
	// Initialize USART2, 115200 baud, TX: PD5, RX: PD6    		
	TM_USART_Init(USART2, TM_USART_PinsPack_2, 115200); 		//(GY25/RAZOR)
	// Initialize USART3    9600 baud, TX: PC10, RX: PC11 
	TM_USART_Init(USART3, TM_USART_PinsPack_2, 9600);			  //GPS  
	
	InitBmp();		 		//  Initialize BMP Altitude
	
	interruptInit();  // Initialize Interrupt
	resetIMU();    		// Initialize IMU 
	
	InitPWMIN();   		// Initialize PWM INPUT
	InitPWMOUT(); 	 	// Initialize PWM OUTPUT
	
	// KALIBRASI ALTITIDE
	KalibrasiBMP();
	// KALIBRASI IMU
	KalibrasiIMU();
		
	//########>>>>>>>>START<<<<<<<########//
	TM_USART_Puts(USART1,"\n\rBISMILLAH SYSTEM EFRISA GLIDER PLANE... ");
	Delayms(1000);
		
	while(1){
			GPSAccess();			  // Aktif GPS
			GetBMP();					  // Aktif BMP Altitude
			BateraiMonitor();   // Aktif Bat Monitor 
			ParseMode();		    // Mode TakeOFF/NonTakeOFF
			GetPWM();				    // Aktif PWMIN
			Airspeed();			    // Aktif Airspeed
// #################### RUN FLIGHT MODE ################### //
		Mode_Flag();
		//Kirim Ke Ground Control Station
			//SendGCS(200);				// GCS OLD
			SendGCS4Bit(200);		// GCS NEW
	}
}