#ifndef _SystemInit_H
#define _SystemInit_H
void RCC_Configuration(void);
void SysTick_Configuration(void);
void STM32_UART_Init(void);

void RS485_SendString(unsigned char *pString,unsigned char ucLen);	
void Lovo_SendString(unsigned char *pString,unsigned char ucLen);
void WIFI_SendString(unsigned char *pString,unsigned char ucLen);
#endif



