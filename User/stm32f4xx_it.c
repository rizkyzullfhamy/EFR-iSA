/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "tm_stm32f4_delay.h"
#include "main.h"
#include "Global_Variable.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

// ########################## PID CONTROL SYSTEM #############################//
	extern float TimeSampling;
	extern float KP_PITCH,KI_PITCH,KD_PITCH;
	extern float KP_ROLL,KI_ROLL,KD_ROLL;
	extern float KP_YAW,KI_YAW,KD_YAW;
	
	extern float ErrPitch, ErrRoll, ErrYaw;
	extern float LastErrPitch, LastErrRoll, LastErrYaw;
	extern float IntegralErrPitch, IntegralErrRoll, IntegralErrYaw;
	extern float DerivatifErrPitch, DerivatifErrRoll, DerivatifErrYaw;
	 
	extern float PropotionalPitch, PropotionalRoll, PropotionalYaw;
	extern float IntegralPitch, IntegralRoll, IntegralYaw;
	extern float DerivatifPitch, DerivatifRoll, DerivatifYaw;
	 
	extern float PID_Pitch, PID_Roll, PID_Yaw;
	extern float OutPidPitch, OutPidRoll, OutPidYaw;
	
	extern float dataYaw,dataPitch,dataRoll, dataYaw_Zero, dataPitch_Zero, dataRoll_Zero;
	extern int setServo, BRUSHLESS_one;
	

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function decrement timing variable
  *	@with __weak parameter to prevent errors
  * @param  None
  * @retval None
  */
__weak void TimingDelay_Decrement(void) {

}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

/*
void AltitudePID(){
	errorAltitude = data_altitude_zero - data_altitude;
	errorAltitude *= -1;
	hasilErrorAltitude = errorAltitude - lastErrorAltitude;
	lastErrorAltitude = errorAltitude;
	
	pidAl = ((errorAltitude *kpAl) + (hasilErrorAltitude*kdAl)) / 0.001;
	HasilPidAl = BRUSHLESS_one + pidAl;    // Menurunkan Hingga Posisi Awal
	
	if(HasilPidAl >= 1700) {HasilPidAl = 1700;}
	if(HasilPidAl <= 1000) {HasilPidAl = 1000;}
}
*/


// FUNCTION PID GLIDER PLANE 
void pitchPID(){
	ErrPitch = dataPitch_Zero - dataPitch;
	
	IntegralErrPitch += (ErrPitch * TimeSampling);
		if (IntegralErrPitch >= 1000) IntegralErrPitch = 1000;
		else if(IntegralErrPitch <= -1000) IntegralPitch = -1000;
	
	DerivatifErrPitch = (ErrPitch - LastErrPitch) / TimeSampling;
	LastErrPitch = ErrPitch;
	
	PropotionalPitch 	= ErrPitch  * KP_PITCH;
	IntegralPitch			= IntegralErrPitch * KI_PITCH;
	DerivatifPitch 		= DerivatifErrPitch * KD_PITCH;
	
	PID_Pitch 				= PropotionalPitch + IntegralPitch + DerivatifPitch;
	
	//HasilPidP  = setServo + pidp;
}
void yawPID(){
	ErrYaw = dataYaw_Zero - dataYaw;
	
	IntegralErrYaw 	+= (ErrYaw * TimeSampling);
		if (IntegralErrYaw >= 1000) IntegralErrYaw = 1000;
		else if (IntegralErrYaw <= -1000) IntegralErrYaw = -1000;
	
	DerivatifErrYaw = (ErrYaw - LastErrYaw) / TimeSampling;
	LastErrYaw = ErrYaw;
	
	PropotionalYaw 	= ErrYaw  * KP_YAW;
	IntegralYaw			= IntegralErrYaw * KI_YAW;
	DerivatifYaw 		= DerivatifErrYaw * KD_YAW;
	
	PID_Yaw				= PropotionalYaw + IntegralYaw + DerivatifYaw;
	
	//HasilPidY = setServo + pidy; 
}
void rollPID(){
	ErrRoll = dataRoll_Zero - dataRoll;
	
	IntegralErrRoll += (ErrRoll * TimeSampling);
	  if(IntegralErrRoll >= 1000) IntegralErrRoll = 1000;
		else if (IntegralErrRoll <= -1000) IntegralErrRoll = -1000;
	
	DerivatifErrRoll = (ErrRoll - LastErrRoll) / TimeSampling;
	LastErrRoll = ErrRoll;
	
	PropotionalRoll 	= ErrRoll  * KP_ROLL;
	IntegralRoll			= IntegralErrRoll * KI_ROLL;
	DerivatifRoll 		= DerivatifErrRoll * KD_ROLL;
	
	PID_Roll				= PropotionalRoll + IntegralRoll + DerivatifRoll;

	//HasilPidR = setServo - pidr; 
}
void TrustControl(){
	
	/* JIKA Pergerakan TERBALIK GANTI PLUS (+) KE MINUS (-)
	*/
	
	
	OutPidPitch		= setServo + PID_Pitch;
	OutPidYaw			= setServo + PID_Yaw;
	OutPidRoll		= setServo + PID_Roll;
	
	if (OutPidPitch >= 170) OutPidPitch = 170;
	else if (OutPidPitch <= 10) OutPidPitch = 10;
	
	if (OutPidYaw >= 170) OutPidYaw = 170;
	else if (OutPidYaw <= 10) OutPidYaw = 10;

	if (OutPidRoll >= 170) OutPidRoll = 170;
	else if (OutPidRoll <= 10) OutPidRoll = 10;
	
}
int counter1=0;
void SysTick_Handler(void)
{
	counter1++;
	if (counter1 >= 10){
		yawPID();
		pitchPID();
		rollPID();
		TrustControl();
		counter1 = 0;
	}
	TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
