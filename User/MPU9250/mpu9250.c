#include "mpu9250.h"
#include "I2C.h"
#include "delay.h"
#include "usart.h"	
					   
										   
/******************************************************************************
** 功能：   	设置MPU9250陀螺仪满量程范围
** 参数：	u8 fsr  0-3
** 返回值:   0：成功    1：失败
** 说明：
fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
********************************************************************************/
void MPU9250_Set_Gyro_Fsr(u8 fsr,uint8_t state)
{
	I2CSigleWrite(MPU_ADDR,MPU_GYRO_CFG_REG,fsr<<3,state);
}



/******************************************************************************
** 功能：   	设置MPU9250加速度计满量程范围
** 参数：	u8 fsr  0-3
** 返回值:   0：成功    1：失败
** 说明：
fsr:0,±2g;1,±4g;2,±8g;3,±16g
********************************************************************************/
void MPU9250_Set_Accel_Fsr(u8 fsr,uint8_t state)
{
	I2CSigleWrite(MPU_ADDR,MPU_ACCEL_CFG_REG,fsr<<3,state);//设置加速度计满量程范围
}

/******************************************************************************
** 功能：   	设置MPU9250陀螺仪数字低通滤波器
** 参数：	u16 数字低通滤波器
** 返回值:   0：成功    1：失败
** 说明：
				Gyroscope
DLPF_CFG	带宽(HZ)	延时(ms)	采样率(KHZ)
	0		 250		 0.97			8
	1		 184		 2.9			1
	2		 92			 3.9			1
	3		 41		     5.9			1
	4		 20			 9.9			1
	5		 10			 17.85			1
	6		 5			 33.48			1
	7		 3600		 0.17   		8
********************************************************************************/
void MPU9250_Set_LPF(u16 lpf,uint8_t state)
{
	u8 data = 0;
	if(lpf >= 184)data=1;
	else if(lpf>=92)data=2;
	else if(lpf>=41)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	I2CSigleWrite(MPU_ADDR,MPU_CFG_REG,data,state);//设置数字低通滤波器
}

/******************************************************************************
** 功能：   	设置MPU9250采样率
** 参数：	u8 rate  采样率  单位HZ
** 返回值:   0：成功    1：失败
** 说明：
SAMPLE_RATE = 陀螺仪输出频率/(1+SMPLRT_DIV)==>SMPLRT_DIV=1KHZ/SAMPLE_RATE-1
DLPF=0/7   	陀螺仪输出频率=8KHZ
DLPF=1-6    陀螺仪输出频率=1KHZ
DLPF滤波频率通常设为采样频率的一半(MPU6050)
				Gyroscope
DLPF_CFG	带宽(HZ)	延时(ms)	采样率(KHZ)
	0		 250		 0.97			8
	1		 184		 2.9			1
	2		 92			 3.9			1
	3		 41		     5.9			1
	4		 20			 9.9			1
	5		 10			 17.85			1
	6		 5			 33.48			1
	7		 3600		 0.17   		8
********************************************************************************/
void MPU9250_Set_Rate(u16 rate,uint8_t state)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data = 1000/rate-1;//data为SAMPLE_DIV       rate为采样频率 
	data = I2CSigleWrite(MPU_ADDR,MPU_SAMPLE_RATE_REG,data,state);//设置数字低通滤波器
	MPU9250_Set_LPF(50,state);//设置MPU9250-LPF为采样率一半
}

/******************************************************************************
** 功能：   	 初始化MPU9250
** 参数：	 void
** 返回值:   0：成功    1：失败
** 说明： 
原始版本
Single_Write(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	
Single_Write(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
Single_Write(GYRO_ADDRESS,CONFIG, 0x06);
Single_Write(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
Single_Write(GYRO_ADDRESS,ACCEL_CONFIG, 0x01);
********************************************************************************/
void init_mpu9250(MPU *mMPU, uint8_t state) 
{ 
	I2CSigleWrite(MPU_ADDR,MPU_PWR_MGMT1_REG,0x80,state);	//复位MPU9250
	delay_ms(100);
	I2CSigleWrite(MPU_ADDR,MPU_PWR_MGMT1_REG, 0x00,state);	//唤醒MPU9250
	MPU9250_Set_Gyro_Fsr(1,state);						//设置陀螺仪量程	±500dps	
	MPU9250_Set_Accel_Fsr(1,state);						//设置加速度计量程	±4g
	MPU9250_Set_Rate(1000,state);							//设置采样率 其中会设置DLPF为采样率滤波带宽为采样率一半 50HZ
	I2CSigleWrite(MPU_ADDR,MPU_INT_EN_REG,0x00,state);		//关闭所有中断
	I2CSigleWrite(MPU_ADDR,MPU_USER_CTRL_REG,0x00,state);	//I2C主模式关闭
	I2CSigleWrite(MPU_ADDR,MPU_FIFO_EN_REG,0x00,state);	//关闭FIFO
	I2CSigleWrite(MPU_ADDR,MPU_INTBP_CFG_REG,0x80,state);	//INT引脚低电平有效

	I2CSigleWrite(MPU_ADDR,MPU_PWR_MGMT1_REG,0x01,state);	//设置CLKSEL,PLL X轴为参考
	I2CSigleWrite(MPU_ADDR,MPU_PWR_MGMT2_REG,0x00,state);	//加速度与陀螺仪都工作

	//通过5000次静置加权平均求出陀螺仪零偏OFFSET
	get_gyro_bias(mMPU,state);
	
	//初始化mag
	Init_MPU9150_Mag(state);
	
}





