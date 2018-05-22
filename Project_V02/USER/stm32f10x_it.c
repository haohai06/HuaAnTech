/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
#include "main.h" 

extern unsigned char g_ucRS485_RcvTime;
extern unsigned char g_ucRS485_RcvFlag;
extern unsigned char g_ucRS485_RecBuf[RS485_RCV_NUM];
extern unsigned char g_ucRS485_RecNum;

extern unsigned char g_ucLovo_RcvTime;
extern unsigned char g_ucLovo_RcvFlag;
extern unsigned char g_ucLovo_RecBuf[LOVO_RCV_NUM];
extern unsigned char g_ucLovo_RecNum;

extern unsigned char g_ucWIFI_RcvTime;
extern unsigned char g_ucWIFI_RcvFlag;
extern unsigned char g_ucWIFI_RecBuf[LOVO_RCV_NUM];
extern unsigned char g_ucWIFI_RecNum;

extern unsigned short int g_uiTimeCnt;

void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
void SysTick_Handler(void)
{
	if(g_ucRS485_RcvTime<RCV_MAX_TIME){
		g_ucRS485_RcvTime++;
		if(g_ucRS485_RcvTime>=RCV_MAX_TIME)
			g_ucRS485_RcvFlag = 1;	
	}
	if(g_ucLovo_RcvTime<RCV_MAX_TIME){
		g_ucLovo_RcvTime++;
		if(g_ucLovo_RcvTime>=RCV_MAX_TIME)
			g_ucLovo_RcvFlag = 1;	
	}	
	if(g_ucWIFI_RcvTime<RCV_MAX_TIME){
		g_ucWIFI_RcvTime++;
		if(g_ucWIFI_RcvTime>=RCV_MAX_TIME)
			g_ucWIFI_RcvFlag = 1;	
	}
	if(g_uiTimeCnt < 1000){
		g_uiTimeCnt ++;
	}
	
}


void USART1_IRQHandler(void)  //WIFI
{
	unsigned char ucTempReceiveData = 0;
	static unsigned char ucIndex = 0;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET){ 
		ucTempReceiveData = USART_ReceiveData(USART1);
			if(g_ucWIFI_RcvTime >= RCV_MAX_TIME)
				ucIndex = 0;	
			if(g_ucWIFI_RcvFlag == 0){        //buffer available
				g_ucWIFI_RecBuf[ucIndex++] = ucTempReceiveData;
				if(ucIndex > WIFI_RCV_NUM){
					ucIndex = WIFI_RCV_NUM; 
				}
				g_ucWIFI_RecNum = ucIndex;
				g_ucWIFI_RcvTime = 0;
			}	
	}
	USART_ClearFlag(USART1, USART_FLAG_TC);	
}

void USART2_IRQHandler(void)  //Lovo
{
//  unsigned char ucTempReceiveData = 0;
//	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET){ 
//	  ucTempReceiveData = USART_ReceiveData(USART1);
//	}
// USART_ClearFlag(USART2, USART_FLAG_TC);
	unsigned char ucTempReceiveData = 0;
	static unsigned char ucIndex = 0;
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!= RESET){ 
		ucTempReceiveData = USART_ReceiveData(USART2);
			if(g_ucLovo_RcvTime >= RCV_MAX_TIME)
				ucIndex = 0;	
			if(g_ucLovo_RcvFlag == 0){        //buffer available
				g_ucLovo_RecBuf[ucIndex++] = ucTempReceiveData;
				if(ucIndex > LOVO_RCV_NUM){
					ucIndex = LOVO_RCV_NUM; 
				}
				g_ucLovo_RecNum = ucIndex;
				g_ucLovo_RcvTime = 0;
			}	
	}
	USART_ClearFlag(USART2, USART_FLAG_TC);	
}


void USART3_IRQHandler(void)  //RS485
{
//  unsigned char ucTempReceiveData = 0;
//	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET){ 
//	  ucTempReceiveData = USART_ReceiveData(USART1);
//	}
// USART_ClearFlag(USART1, USART_FLAG_TC);
	unsigned char ucTempReceiveData = 0;
	static unsigned char ucIndex = 0;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!= RESET){ 
		ucTempReceiveData = USART_ReceiveData(USART3);
			if(g_ucRS485_RcvTime >= RCV_MAX_TIME)
				ucIndex = 0;	
			if(g_ucRS485_RcvFlag == 0){        //buffer available
				g_ucRS485_RecBuf[ucIndex++] = ucTempReceiveData;
				if(ucIndex > RS485_RCV_NUM){
					ucIndex = RS485_RCV_NUM; 
				}
				g_ucRS485_RecNum = ucIndex;
				g_ucRS485_RcvTime = 0;
			}	
	}
	USART_ClearFlag(USART3, USART_FLAG_TC);	
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
