#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdio.h>

extern char Serial_RxPacket[101];
extern uint8_t Serial_RxFlag;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array,uint16_t Length);
void Serial_SendString(char *String);
uint32_t Serial_Pow(uint32_t X,uint32_t Y);
void Serial_SendNum(uint32_t Num,uint8_t Length);
void Serial_Printf(char *format, ...);
uint8_t Serial_GetRxFlag(void);
void Serial_SendPacket(void);
void Serial_HeartBeat(void);
void HeartBeat_Tick(void);
#endif
