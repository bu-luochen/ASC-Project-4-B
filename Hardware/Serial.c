#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

char Serial_RxPacket[101];
uint8_t Serial_RxFlag;

void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array,uint16_t Length);

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=9600;//波特率
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//硬件流控
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity=USART_Parity_No;//校验
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//字长
	USART_Init(USART1,&USART_InitStructure);
	
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//开启USART到NVIC的输出
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
}



//void Serial_SendPacket(void)
//{
//	Serial_SendByte(0xFF);
//	Serial_SendArray(Serial_TxPacket,4);
//	Serial_SendByte(0xFE);
//}

//uint8_t Serial_GetRxFlag(void)
//{
//	if(Serial_RxFlag == 1)
//	{
//		Serial_RxFlag = 0;
//		return 1;
//	}
//	return 0;
//}



void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET){
		uint8_t RxData = USART_ReceiveData(USART1);
		switch(RxState){
			case 0:
				if(RxData == '@' && Serial_RxFlag == 0){
					RxState = 1;
					pRxPacket = 0;
				}
				break;
			case 1:
				if(RxData == '\r'){
					RxState = 2;
				} else {
					Serial_RxPacket[pRxPacket] = RxData;
					pRxPacket ++;
				}
				break;
			case 2:
				if(RxData == '\n'){
					RxState = 0;
					Serial_RxPacket[pRxPacket] ='\0';
					Serial_RxFlag = 1;
				}
				break;
			}
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET){}
	
}
void Serial_SendArray(uint8_t *Array,uint16_t Length)
{
	for(int16_t i = 0 ; i < Length; i++){
		USART_SendData(USART1,Array[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET){}
	
	}
	
	
}

void Serial_SendString(char *String)
{
	for(int16_t i = 0 ; String[i] != '\0'; i++){
		USART_SendData(USART1,String[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET){}
	
	}
}

uint32_t Serial_Pow(uint32_t X,uint32_t Y)
{
	uint32_t result = 1;
	while(Y--){
		result *= X;
	}
	return result;
}

void Serial_SendNum(uint32_t Num,uint8_t Length)
{
	for(int16_t i = 0 ; i < Length; i++){
		
		USART_SendData(USART1,(Num / Serial_Pow(10,Length - i - 1))% 10 + '0');
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET){}
	
	}	
	
}

int fputc(int ch,FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg,format);
	vsprintf(String,format,arg);
	va_end(arg);
	Serial_SendString(String);
	
	
}

void Serial_HeartBeat(void)
{
	Serial_Printf("@H\r\n");
}


void HeartBeat_Tick(void)
{
	static uint8_t Time_HB;
	Time_HB ++;
	if(Time_HB >= 10){
		Time_HB = 0;
		Serial_HeartBeat();
	}
	
}
