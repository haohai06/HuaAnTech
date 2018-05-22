#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "SystemInit.h"
#include "main.h"
#include "PCF8653.h"

/************************************************

************************************************/
unsigned char  g_ucRS485_RcvTime = RCV_MAX_TIME; 
unsigned char  g_ucRS485_RcvFlag = 0;
unsigned char g_ucRS485_RecBuf[RS485_RCV_NUM];
unsigned char g_ucRS485_RecNum = 0;

unsigned char  g_ucLovo_RcvTime = RCV_MAX_TIME; 
unsigned char  g_ucLovo_RcvFlag = 0;
unsigned char g_ucLovo_RecBuf[RS485_RCV_NUM];
unsigned char g_ucLovo_RecNum = 0;

unsigned char  g_ucWIFI_RcvTime = RCV_MAX_TIME; 
unsigned char  g_ucWIFI_RcvFlag = 0;
unsigned char g_ucWIFI_RecBuf[WIFI_RCV_NUM];
unsigned char g_ucWIFI_RecNum = 0;

unsigned short int g_uiTimeCnt = 0;

PCF8653TypeDef ScmPCF8653;

unsigned char ucTable[3] = {0};

 void Delay(u32 count)
 {
   u32 i=0;
   for(;i<count;i++);
 }

int main(void)
{	
	RCC_Configuration();
	SysTick_Configuration();
	STM32_UART_Init();
	
	g_ucRS485_RecBuf[0] = 0x01;
	g_ucRS485_RecBuf[1] = 0x02;
	g_ucRS485_RecBuf[2] = 0x03;
	RS485_SendString(g_ucRS485_RecBuf,3);
	//GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
	PCF8563_Init();
//PCF8563_Read(PCF8653TypeDef * pcf8563);

	while(1)
	{
		
		
		
//		if(g_ucLovo_RcvFlag == 1){
//			RS485_SendString(g_ucLovo_RecBuf,g_ucLovo_RecNum );
//			memset(g_ucLovo_RecBuf,0,10);
//			g_ucLovo_RcvFlag = 0;
//		}
//		if(g_ucRS485_RcvFlag == 1){
//			Lovo_SendString(g_ucRS485_RecBuf,g_ucRS485_RecNum);
//			//RS485_SendString(g_ucRS485_RecBuf,g_ucRS485_RecNum );
//			memset(g_ucRS485_RecBuf,0,10);
//			g_ucRS485_RcvFlag = 0;
//		}
//		if(g_ucWIFI_RcvFlag == 1){
//			RS485_SendString(g_ucWIFI_RecBuf,g_ucWIFI_RecNum );
//			memset(g_ucWIFI_RecBuf,0,10);
//			g_ucWIFI_RcvFlag = 0;
//		}
//		if(g_ucRS485_RcvFlag == 1){
//			WIFI_SendString(g_ucRS485_RecBuf,g_ucRS485_RecNum);
//			//RS485_SendString(g_ucRS485_RecBuf,g_ucRS485_RecNum );
//			memset(g_ucRS485_RecBuf,0,10);
//			g_ucRS485_RcvFlag = 0;
//		}
		if(g_uiTimeCnt >= 1000){
			g_uiTimeCnt = 0;
			PCF8563_Read(&ScmPCF8653);
			ucTable[0] = ScmPCF8653.ucHour/10 +0x30;
			ucTable[1] = ScmPCF8653.ucHour%10 +0x30;
			ucTable[2] = ':';
			RS485_SendString(ucTable,3 );				
			ucTable[0] = ScmPCF8653.ucMinute/10 +0x30;
			ucTable[1] = ScmPCF8653.ucMinute%10 +0x30;
			ucTable[2] = ':';
			RS485_SendString(ucTable,3 );				
			
			ucTable[0] = ScmPCF8653.ucSecond/10 +0x30;
			ucTable[1] = ScmPCF8653.ucSecond%10 +0x30;
			ucTable[2] = '\0';
			RS485_SendString(ucTable,2 );						 			
		}	
		
	}
}
