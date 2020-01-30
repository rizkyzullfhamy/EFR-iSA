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

extern float dataYaw,dataPitch,dataRoll, dataYaw_Zero, dataPitch_Zero, dataRoll_Zero;
extern float lastError1, hasilError1,lastError2, hasilError2, lastError3, hasilError3, lastError4, hasilError4, lastError5, hasilError5, lastError6, hasilError6;
extern float kpr,kdr, kpy,kdy, kpp,kdp, kpb,kdb, kpAl, kdAl;	
extern float setPointRoll, setPointYaw, setPointPitch, pidkpr,pidkdr, pidkpy,pidkdy, pidkpp,pidkdp, sudut_belok;
extern float pidp,pidr,pidy;
extern float error1,error2,error3;
extern int HasilPidP,HasilPidR,HasilPidY;	
extern int setServo, BRUSHLESS_one;
extern float lastErrorAltitude, hasilErrorAltitude, setPointAltitude, pidkpAl, pidkdAl, errorAltitude, pidAl, HasilPidAl;   // Altitude PID
extern float data_altitude_zero;
extern int data_altitude;
	

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
//PID CONTROL GLIDER PLANE 
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
void pitchPID(){
	error3 = dataPitch_Zero - dataPitch;
	error3 *= -1;
	hasilError3 = error3 - lastError3;
	lastError3 = error3;
	
	pidp = ((error3*kpp) + (hasilError3*kdp)) / 0.001;
	HasilPidP  = setServo + pidp;
	
	if (HasilPidP >= 160 ) {HasilPidP = 160; }
	if (HasilPidP <=  20 ) {HasilPidP =  20; }
}
void yawPID(){
	//error2 = setPointYaw - (dataYaw+70) ;
	error2 = dataYaw_Zero - dataYaw;
	
	pidy = (error2*kpy)+(hasilError2*kdy);
	hasilError2 = error2 - lastError2;
	lastError2 = error2;
	
	HasilPidY = setServo + pidy; 
	if (HasilPidY>=160){HasilPidY=160;}
	if (HasilPidY<=20) {HasilPidY=20;}
}
void rollPID(){
	error1=dataRoll_Zero - dataRoll;
	hasilError1 = error1 - lastError1;
	lastError1 = error1;
	
	pidr = ((error1*kpr)+(hasilError1*kdr))/0.001;
	
	HasilPidR = setServo - pidr; 
	
	if (HasilPidR>=160){HasilPidR=160;}
	if (HasilPidR<=20) {HasilPidR=20;}
}
void SysTick_Handler(void)
{
	rollPID();
	pitchPID();
	yawPID();
	Delayms(1);
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
