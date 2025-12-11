#ifndef __MPU6050_H
#define __MPU6050_H
void MPU6050_WriteReg(uint8_t RegAddresss,uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddresss);
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ);
void MPU6050_Tick(void);

typedef struct{
	int32_t sumAX;
	int32_t sumAY;
	int32_t sumAZ;
	int32_t sumGX;
	int32_t sumGY;
	int32_t sumGZ;
}MPU6050_DataInitDef;

uint8_t MPU6050_DataInit(int16_t *AccX,int16_t *AccY,int16_t *AccZ,int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ,MPU6050_DataInitDef *Result);


#endif
