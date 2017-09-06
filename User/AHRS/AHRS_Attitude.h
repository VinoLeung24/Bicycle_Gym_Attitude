#ifndef __AHRS_Attitude_H
#define __AHRS_Attitude_H

#include "stm32f10x.h"
#include "stdint.h"

typedef struct
{
	float q0, q1, q2, q3;
	float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
	float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;
	float Yaw_Filter[10];						//对使用IMU算法得出的Yaw进行滤波
	float halfT;
	float exInt, eyInt, ezInt;        // scaled integral error
	float Pitch, Roll, Yaw;
	float maxMagX;
	float minMagX;
	float maxMagY;
	float minMagY;
	float maxMagZ;
	float minMagZ;
	float MXgain;
	float MYgain;
	float MZgain;
	float MXoffset;
	float MYoffset;
	float MZoffset;
	
}MPU;

enum i2c_state
{
	i2c_L,
	i2c_R,
	i2c_B,
};

void get_mpu9150_data(MPU *mMPU, uint8_t state);
void init_quaternion(MPU *mMPU, uint8_t state);
void AHRSupdate(MPU *mMPU) ;
float invSqrt(float x);
void Get_Attitude(MPU *mMPU);
int get_gyro_bias(MPU *mMPU, uint8_t state);
void get_compass_bias(MPU *mMPU, uint8_t state);
void compass_calibration(MPU *mMPU);
void Read_MPU9150_Mag(MPU *mMPU, uint8_t state);
int Init_MPU9150_Mag(uint8_t state);
void Delay(__IO uint32_t nCount);
#endif
