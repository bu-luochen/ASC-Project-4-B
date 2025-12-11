#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "LED.h"
#include "Key.h"
#include "Buzzer.h"
#include "LightSenser.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Serial.h"
#include "Timer.h"
#include <math.h>
#include "stdio.h"
#include "KF.h"
#include "Madgwick.h"
uint8_t KeyNum;
int16_t AX,AY,AZ,GX,GY,GZ;
int8_t id;
uint16_t time = 0,Flag = 0;

static uint8_t count;

float Yaw_g = 0,Pitch_g = 0,Roll_g = 0,Pitch_a = 0,Roll_a = 0;
float Yaw ,Pitch,Roll;
float rate = 0.01;

void MPU6050_AngleSolve(void);
MPU6050_DataInitDef Result = {
	.sumAX = 0,
	.sumAY = 0,
	.sumAZ = 0,
	.sumGX = 0,
	.sumGY = 0,
	.sumGZ = 0,
};

KalmanFilter1D X = {
    .Angle_Hat = 0,  
    .P = 1.0,     
    .Q = 0.006,   
    .R = 0.1     
};

KalmanFilter1D Y = {
    .Angle_Hat = 0,  
    .P = 1.0,     
    .Q = 0.006,   
    .R = 0.1    
};

KalmanFilter1D Z = {
    .Angle_Hat = 0,  
    .P = 1.0,     
    .Q = 0.006,   
    .R = 0.1    
};
MadgwickFilter MF = {
	.q0 = 1,
	.q1 = 0,
	.q2 = 0,
	.q3 = 0
};
int main ()
{	
	OLED_Init();
	MPU6050_Init();
	Serial_Init();
	Timer_Init();
	id = MPU6050_GetID();
	OLED_ShowString(0,0,"ID:",OLED_8X16);
	OLED_ShowHexNum(16,0,id,2,OLED_8X16);
	while(1)
	{
		
		if(Serial_RxFlag == 1){
			if(Serial_RxPacket[0] == 'H'){
				Serial_HeartBeat();
			}
			Serial_RxFlag = 0;
		}
		
		MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);//10ms
		
		
		if(MPU6050_DataInit(&AX,&AY,&AZ,&GX,&GY,&GZ,&Result) == 1){
				
			GX -= Result.sumGX;
			GY -= Result.sumGY;
			GZ -= Result.sumGZ;
			AX -= Result.sumAX;
			AY -= Result.sumAY;
			AZ -= (Result.sumAZ - 2048);
			MPU6050_AngleSolve();
				
				
		}
		
		
		if(count >= 10){
			
//				OLED_ShowSignedNum(0,16,AX,5,OLED_8X16);
//				OLED_ShowSignedNum(0,32,AY,5,OLED_8X16);
//				OLED_ShowSignedNum(0,48,AZ,5,OLED_8X16);
//				OLED_ShowSignedNum(64,16,GX,5,OLED_8X16);
//				OLED_ShowSignedNum(64,32,GY,5,OLED_8X16);
//				OLED_ShowSignedNum(64,48,GZ,5,OLED_8X16);
			
			OLED_ShowFloatNum(0,16,X.Angle_Hat,3,3,OLED_8X16);
			OLED_ShowFloatNum(0,32,Z.Angle_Hat,3,3,OLED_8X16);
			OLED_ShowFloatNum(0,48,Y.Angle_Hat,3,3,OLED_8X16);
			
			//Serial_Printf("@(%.2f)(%.2f)(%.2f)\r\n",X.Angle_Hat,Z.Angle_Hat,Y.Angle_Hat);//23ms
			Serial_Printf("%.2f,%.2f,%.2f\r\n",-X.Angle_Hat,Z.Angle_Hat,Y.Angle_Hat);//23ms
			count = 0;
			OLED_ShowNum(96,0,time,4,OLED_8X16);//20ms
			OLED_Update();
		} 
		
		
		
		time = 0;
			
		
	}
	
}



void TIM2_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET){
		
		time ++;
		count ++;
		
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}

void MPU6050_AngleSolve(void)
{

		float dt = 0.061;
			
		float dX = GX / 32768.0  * 2000,dY = GY / 32768.0  * 2000,dZ = GZ / 32768.0  * 2000;
		
		float gyro[3] = {dX * 3.14159 / 180,dY * 3.14159 / 180,dZ * 3.14159 / 180};
			
		float accel[3] = {AX * 9.8 / 2048 ,AY * 9.8 / 2048 ,AZ * 9.8 / 2048};
		
		if(fabs(GZ) > 1){Madgwick_Update(&MF,gyro,accel,dt);}
		
		Yaw = Madgwick_QuatToYaw(&MF) * 180 /3.14159;
		Pitch = Madgwick_QuatToPitch(&MF) * 180 /3.14159;
		Roll = Madgwick_QuatToRoll(&MF) * 180 /3.14159;
		
//		if(fabs(GZ) > 1){Yaw_g = Yaw + dZ * dt;}
//		if(fabs(GY) > 1){Pitch_g = Pitch + dY * dt;}
//		if(fabs(GX) > 1){Roll_g = Roll + dX * dt;} 
//			
//			
//		Pitch_a = atan2(AY,AZ) / 3.14159 * 180;
//		Roll_a = atan2(AX,AZ) / 3.14159 * 180;
//			
//		Yaw = Yaw_g;
//		Pitch = Pitch_g;
//		Roll = Roll_g;
//		Pitch = Pitch_a * rate + Pitch_g * (1 - rate);
//		Roll = Roll_a * rate + Roll_g * (1 - rate);
		
		KalmanFilter1D_Update(&X,dX,Roll,dt);
		KalmanFilter1D_Update(&Y,dY,Pitch,dt);
		KalmanFilter1D_Update(&Z,dZ,Yaw,dt);
		
	
}
