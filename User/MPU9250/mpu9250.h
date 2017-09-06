#ifndef __MPU9250_H__
#define __MPU9250_H__


#include "sys.h"
#include "usart.h"	
#include "AHRS_Attitude.h"	

#define DEFAULT_MPU_HZ  1000

//注释此宏可关闭打印信息
//#define MPU9250_DUBUG 1

#define MPU9150_Addr            0x68
#define P_M_1                   0x6B
#define Accel_FILTER_98HZ		98
#define Accel_Xout_H		    0x3B
#define Gyro_Xout_H		        0x43
#define Bypass_Enable_Cfg		0x37
#define User_Ctrl               0x6A
#define Compass_Addr            0x0C
#define Compass_CNTL            0x0A
#define Compass_ST1             0x02
#define	Compass_HXL             0x03

#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z


#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高8位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器0x71

//#define MPU_ADDR				0xD0	//ADO接GND，最低位为0
#define MPU_ADDR				0x68	//ADO接GND，最低位为0
#define	GYRO_ADDRESS   			0xD0	//陀螺仪地址
#define ACCEL_ADDRESS  			0xD0   	//加速度计地址

#define MAG_ADDRESS   			0x0C   	//磁力计地址
//#define MAG_ADDRESS   			0x18   	//磁力计地址
#define AKM8963_DEVICE_ID_REG	0x00	//AKM8963器件ID寄存器  0x48
#define AKM8963_INFO_REG		0x01	//AKM8963器件信息寄存器
#define AKM8963_STATUS1_REG		0x02	//AKM8963状态寄存器1

#define AKM8963_MAG_XOUTL_REG	0x03	//AKM8963磁力计值,X轴低8位寄存器
#define AKM8963_MAG_XOUTH_REG	0x04	//AKM8963磁力计值,X轴高8位寄存器
#define AKM8963_MAG_YOUTL_REG	0x05	//AKM8963磁力计值,Y轴低8位寄存器
#define AKM8963_MAG_YOUTH_REG	0x06	//AKM8963磁力计值,Y轴高8位寄存器
#define AKM8963_MAG_ZOUTL_REG	0x07	//AKM8963磁力计值,Z轴低8位寄存器
#define AKM8963_MAG_ZOUTH_REG	0x08	//AKM8963磁力计值,Z轴高8位寄存器

#define AKM8963_STATUS2_REG		0x09	//AKM8963状态寄存器2
#define	AKM8963_CNTL1_REG		0x0A	//AKM8963控制寄存器1
#define	AKM8963_CNTL2_REG		0x0B	//AKM8963控制寄存器2
#define AKM8963_SELF_TEST_REG	0x0C	//AKM8963自检控制寄存器
#define AKM8963_TEST1_REG		0x0D	//AKM8963测试寄存器1
#define AKM8963_TEST2_REG		0x0E	//AKM8963测试寄存器2
#define AKM8963_I2C_DISABLE_REG	0x0F	//AKM8963-I2C失能寄存器
#define AKM8963_ASAX_REG		0x10	//AKM8963磁力计X轴灵敏度校正寄存器
#define AKM8963_ASAY_REG		0x11	//AKM8963磁力计Y轴灵敏度校正寄存器
#define AKM8963_ASAZ_REG		0x12	//AKM8963磁力计Z轴灵敏度校正寄存器



enum _MPU9250_STATUS
{
	MPU9250_OK = 0,
	MPU9250_FAIL = 1,
};

typedef enum _MPU9250_STATUS MPU9250_STATUS;

void init_mpu9250(MPU *mMPU, uint8_t state);	


#endif
