#include "mpu9250.h"
#include "I2C.h"
#include "delay.h"
#include "usart.h"	
					   
										   
/******************************************************************************
** ���ܣ�   	����MPU9250�����������̷�Χ
** ������	u8 fsr  0-3
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
********************************************************************************/
void MPU9250_Set_Gyro_Fsr(u8 fsr,uint8_t state)
{
	I2CSigleWrite(MPU_ADDR,MPU_GYRO_CFG_REG,fsr<<3,state);
}



/******************************************************************************
** ���ܣ�   	����MPU9250���ٶȼ������̷�Χ
** ������	u8 fsr  0-3
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
fsr:0,��2g;1,��4g;2,��8g;3,��16g
********************************************************************************/
void MPU9250_Set_Accel_Fsr(u8 fsr,uint8_t state)
{
	I2CSigleWrite(MPU_ADDR,MPU_ACCEL_CFG_REG,fsr<<3,state);//���ü��ٶȼ������̷�Χ
}

/******************************************************************************
** ���ܣ�   	����MPU9250���������ֵ�ͨ�˲���
** ������	u16 ���ֵ�ͨ�˲���
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
				Gyroscope
DLPF_CFG	����(HZ)	��ʱ(ms)	������(KHZ)
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
	I2CSigleWrite(MPU_ADDR,MPU_CFG_REG,data,state);//�������ֵ�ͨ�˲���
}

/******************************************************************************
** ���ܣ�   	����MPU9250������
** ������	u8 rate  ������  ��λHZ
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
SAMPLE_RATE = ���������Ƶ��/(1+SMPLRT_DIV)==>SMPLRT_DIV=1KHZ/SAMPLE_RATE-1
DLPF=0/7   	���������Ƶ��=8KHZ
DLPF=1-6    ���������Ƶ��=1KHZ
DLPF�˲�Ƶ��ͨ����Ϊ����Ƶ�ʵ�һ��(MPU6050)
				Gyroscope
DLPF_CFG	����(HZ)	��ʱ(ms)	������(KHZ)
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
	data = 1000/rate-1;//dataΪSAMPLE_DIV       rateΪ����Ƶ�� 
	data = I2CSigleWrite(MPU_ADDR,MPU_SAMPLE_RATE_REG,data,state);//�������ֵ�ͨ�˲���
	MPU9250_Set_LPF(50,state);//����MPU9250-LPFΪ������һ��
}

/******************************************************************************
** ���ܣ�   	 ��ʼ��MPU9250
** ������	 void
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵���� 
ԭʼ�汾
Single_Write(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	
Single_Write(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
Single_Write(GYRO_ADDRESS,CONFIG, 0x06);
Single_Write(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
Single_Write(GYRO_ADDRESS,ACCEL_CONFIG, 0x01);
********************************************************************************/
void init_mpu9250(MPU *mMPU, uint8_t state) 
{ 
	I2CSigleWrite(MPU_ADDR,MPU_PWR_MGMT1_REG,0x80,state);	//��λMPU9250
	delay_ms(100);
	I2CSigleWrite(MPU_ADDR,MPU_PWR_MGMT1_REG, 0x00,state);	//����MPU9250
	MPU9250_Set_Gyro_Fsr(1,state);						//��������������	��500dps	
	MPU9250_Set_Accel_Fsr(1,state);						//���ü��ٶȼ�����	��4g
	MPU9250_Set_Rate(1000,state);							//���ò����� ���л�����DLPFΪ�������˲�����Ϊ������һ�� 50HZ
	I2CSigleWrite(MPU_ADDR,MPU_INT_EN_REG,0x00,state);		//�ر������ж�
	I2CSigleWrite(MPU_ADDR,MPU_USER_CTRL_REG,0x00,state);	//I2C��ģʽ�ر�
	I2CSigleWrite(MPU_ADDR,MPU_FIFO_EN_REG,0x00,state);	//�ر�FIFO
	I2CSigleWrite(MPU_ADDR,MPU_INTBP_CFG_REG,0x80,state);	//INT���ŵ͵�ƽ��Ч

	I2CSigleWrite(MPU_ADDR,MPU_PWR_MGMT1_REG,0x01,state);	//����CLKSEL,PLL X��Ϊ�ο�
	I2CSigleWrite(MPU_ADDR,MPU_PWR_MGMT2_REG,0x00,state);	//���ٶ��������Ƕ�����

	//ͨ��5000�ξ��ü�Ȩƽ�������������ƫOFFSET
	get_gyro_bias(mMPU,state);
	
	//��ʼ��mag
	Init_MPU9150_Mag(state);
	
}





