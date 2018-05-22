#include "stm32f10x.h"



void RCC_Configuration(void)
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	//HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(RCC_WaitForHSEStartUp()==SUCCESS){
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		FLASH_SetLatency(FLASH_Latency_2);
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource()!=0x08);						
	}
}


void SysTick_Configuration(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //72MHz/8=9MHz
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
	SysTick->LOAD = 9000-1; 						//每1/delay_ostickspersec秒中断一次	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK    
 }





void LED_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_8); 	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	GPIO_ResetBits(GPIOB, GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
}







/*********
ESP8266 USART1
PA9  USART1_TX
PA10 USART1_RX

Lovo 433 USART2
PA2  USART2_TX
PA3  USART2_RX

RS485 USART3
PB10  USART3_TX
PB11  USART3_RX
PC13  EN

*******************/

void STM32_UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13); 

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	//GPIO_SetBits(GPIOB, GPIO_Pin_8); 	
	
	/*UART1 TXD*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	/*UART1 RXD*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 //************************************************************* 
	/*UART2 TXD*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	/*UART2 RXD*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

//************************************************************* 
	/*UART3 TXD*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);   
	/*UART3 RXD*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

//*************************************************************
   /*UART1 ESP8266*/
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_GetFlagStatus(USART1, USART_FLAG_TC);//????,??USART_SR,???????
   /*UART2 Lovo*/
	USART_DeInit(USART2);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_GetFlagStatus(USART2, USART_FLAG_TC);//????,??USART_SR,???????
   /*UART3 RS485*/
	USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);
	USART_GetFlagStatus(USART3, USART_FLAG_TC);//
	
	/*UART1*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;        //
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	/*UART2*/
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;        //
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	/*UART3*/
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;        //
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
   
//*************************************************************
}


/*******************************************************************************
* Function Name : 
* Description   : 
*******************************************************************************/
void GC9200_SendByte(unsigned char byte)   //
{	
  USART_SendData(USART1, byte);        //
   	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);
}
/*******************************************************************************
* Function Name : 
* Description   : 
*******************************************************************************/
void GC9200_SendString(unsigned char *pString,unsigned char ucLen)
{
  volatile unsigned char i;
	GPIO_SetBits(GPIOC, GPIO_Pin_6);   // Enable send
	USART_GetFlagStatus(USART1, USART_FLAG_TC);
	for(i=0;i< 20;i++){
	   ;
	}
	for(i=0;i<ucLen;i++){
	    GC9200_SendByte(pString[i]);
	}
	for(i=0;i< 20;i++){
	   ;
	}
	GPIO_ResetBits(GPIOC, GPIO_Pin_6); // Enable receive
}
void GC9200_SendArray(const unsigned char *pString,unsigned char ucLen)
{
  volatile unsigned char i;
	GPIO_SetBits(GPIOC, GPIO_Pin_6);   // Enable send
	USART_GetFlagStatus(USART1, USART_FLAG_TC);
	for(i=0;i< 20;i++){
	   ;
	}
	for(i=0;i<ucLen;i++){
	    GC9200_SendByte(pString[i]);
	}
	for(i=0;i< 20;i++){
	   ;
	}
	GPIO_ResetBits(GPIOC, GPIO_Pin_6); // Enable receive
}



void RS485_SendString(unsigned char *pString,unsigned char ucLen)
{
	volatile unsigned char i;
	GPIO_SetBits(GPIOC, GPIO_Pin_13);   // Enable send
	USART_GetFlagStatus(USART3, USART_FLAG_TC);
	for(i=0;i< 20;i++){
		;
	}
	for(i=0;i<ucLen;i++){
		USART_SendData(USART3, *pString);        //	
		pString++;	
		while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);
	}
	for(i=0;i< 20;i++){
		;
	}
	GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Enable receive
}


void Lovo_SendString(unsigned char *pString,unsigned char ucLen)
{
	volatile unsigned char i;
	USART_GetFlagStatus(USART2, USART_FLAG_TC);
	for(i=0;i<ucLen;i++){
		USART_SendData(USART2, *pString);        //	
		pString++;	
		while( USART_GetFlagStatus(USART2,USART_FLAG_TC)!= SET);
	}
}

void WIFI_SendString(unsigned char *pString,unsigned char ucLen)
{
	volatile unsigned char i;
	USART_GetFlagStatus(USART1, USART_FLAG_TC);
	for(i=0;i<ucLen;i++){
		USART_SendData(USART1, *pString);        //	
		pString++;	
		while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);
	}
}
