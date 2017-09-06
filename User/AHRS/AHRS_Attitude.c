#include "AHRS_Attitude.h"
#include "delay.h"
#include "I2C.h"
#include "math.h"
#include <stdio.h>
#include "mpu9250.h"
#include "time.h"


//���岻ͬ������Χ�Ŀ̶�����
#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f

#define Accel_Xout_Offset		-130
#define Accel_Yout_Offset		 96
#define Accel_Zout_Offset		 460

#define Kp 2.0f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f   //integral gain governs rate of convergence of gyroscope biases

uint16_t IMU_count = 0;
uint16_t imu_flag = 0;

uint16_t IMU_count_L = 0;
uint16_t imu_flag_L = 0;

void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/*******************************************************************************
* Function Name  : get_mpu9150_data
* Description    : ��ȡmpu9150�ļ��ٶȼ� ������ ���������ݲ���У׼���˲�->
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void get_mpu9150_data(MPU *mMPU, uint8_t state)
{
   signed short int gyro[3], accel[3], mag[3]; 
   unsigned char tmp[7], data_write[14];

   if(!i2cread(MPU9150_Addr, Accel_Xout_H, 14, data_write,state))
   {
		accel[0]=(((signed short int)data_write[0])<<8) | data_write[1];
		accel[1]=(((signed short int)data_write[2])<<8) | data_write[3];
		accel[2]=(((signed short int)data_write[4])<<8) | data_write[5];
		gyro[0] =(((signed short int)data_write[8])<<8) | data_write[9];
        gyro[1] =(((signed short int)data_write[10])<<8) | data_write[11];
		gyro[2] =(((signed short int)data_write[12])<<8) | data_write[13];
	 
		mMPU->init_ax=(float)(accel[0] + Accel_Xout_Offset);   	  
		mMPU->init_ay=(float)(accel[1] + Accel_Yout_Offset);
        mMPU->init_az=(float)(accel[2] + Accel_Zout_Offset);  		 
		mMPU->init_gx=((float)gyro[0] - mMPU->Gyro_Xout_Offset) * 0.000266;    //��λת���ɣ�����/s��0.000266=1/(Gyro_500_Scale_Factor * 57.295780)
		mMPU->init_gy=((float)gyro[1] - mMPU->Gyro_Yout_Offset) * 0.000266;
		mMPU->init_gz=((float)gyro[2] - mMPU->Gyro_Zout_Offset) * 0.000266;


        tmp[6]=0x00;
        data_write[6]=0x01;
        i2cread(Compass_Addr, Compass_ST1, 1, tmp+6,state);
        if(tmp[6] == 1)
		{
			i2cread(Compass_Addr, Compass_HXL, 6, tmp,state);//��ȡcompass
			mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
			mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
			mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];
			
			mag[0] = ((long)mag[0] * 286) >> 8;  //�����ȵ���	//286 288 300
			mag[1] = ((long)mag[1] * 288) >> 8;
			mag[2] = ((long)mag[2] * 300) >> 8;

			mMPU->init_mx = (float)mag[1] * mMPU->MXgain + mMPU->MXoffset;
			mMPU->init_my = (float)mag[0] * mMPU->MYgain + mMPU->MYoffset;
			mMPU->init_mz = (float)-mag[2] * mMPU->MZgain + mMPU->MZoffset;
			i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write+6,state);	 //����compass��single measurement mode
		}
   }
   get_compass_bias(mMPU,state);
   compass_calibration(mMPU);
}


/*******************************************************************************
* Function Name  : init_quaternion
* Description    : �����ʼ����Ԫ��q0 q1 q2 q3->
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(MPU *mMPU, uint8_t state)
{ 
		signed short int accel[3], mag[3];
		float init_Yaw, init_Pitch, init_Roll;
		unsigned char tmp[7], data_write[7];
		uint8_t i;

		if(!i2cread(MPU9150_Addr, Accel_Xout_H, 6, data_write,state))
		{
			accel[0]=((((signed short int)data_write[0])<<8) | data_write[1]) + Accel_Xout_Offset;
			accel[1]=((((signed short int)data_write[2])<<8) | data_write[3]) + Accel_Yout_Offset;
			accel[2]=((((signed short int)data_write[4])<<8) | data_write[5]) + Accel_Zout_Offset;
	  	    
			mMPU->init_ax=((float)accel[0]) / Accel_4_Scale_Factor;	   //��λת�����������ٶȵĵ�λ��g
			mMPU->init_ay=((float)accel[1]) / Accel_4_Scale_Factor;
			mMPU->init_az=((float)accel[2]) / Accel_4_Scale_Factor;

			tmp[6]=0x00;
			data_write[6]=0x01;
			i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write+6,state);	 //����compass��single measurement mode
			delay_ms(10);  //wait data ready
			i2cread(Compass_Addr, Compass_ST1, 1, tmp+6,state);
			if(tmp[6] == 1)
			{
				i2cread(Compass_Addr, Compass_HXL, 6, tmp,state);
				mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
				mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
				mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];
				
				mag[0] = ((long)mag[0] * 286) >> 8;  //�����ȵ���	//286 288 300
				mag[1] = ((long)mag[1] * 288) >> 8;
				mag[2] = ((long)mag[2] * 300) >> 8;
				
				mMPU->init_mx = (float)mag[1] * mMPU->MXgain + mMPU->MXoffset;					
				mMPU->init_my = (float)mag[0] * mMPU->MYgain + mMPU->MYoffset;
				mMPU->init_mz = (float)-mag[2] * mMPU->MZgain + mMPU->MZoffset;
				i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write+6,state);	 //����compass��single measurement mode
			}

			  
			init_Roll = -atan2(mMPU->init_ax, mMPU->init_az);
			init_Pitch=  asin(mMPU->init_ay);             
			init_Yaw  =  atan2(mMPU->init_mx*cos(init_Roll) + mMPU->init_my*sin(init_Roll)*sin(init_Pitch) + mMPU->init_mz*sin(init_Roll)*cos(init_Pitch),
												 mMPU->init_my*cos(init_Pitch) - mMPU->init_mz*sin(init_Pitch));            
			mMPU->q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
			mMPU->q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��pitch
			mMPU->q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��roll
			mMPU->q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw


			init_Roll  = init_Roll * 57.295780;	 //����ת�Ƕ�
			init_Pitch = init_Pitch * 57.295780;
			init_Yaw   = init_Yaw * 57.295780;
			if(init_Yaw < 0){init_Yaw = init_Yaw + 360;}      //��Yaw�ķ�Χת��0-360
			if(init_Yaw > 360){init_Yaw = init_Yaw - 360;}
			
			for(i=0; i<10; i++)
			{
				mMPU->Yaw_Filter[i] = init_Yaw;
			}
		}
}

void init_quaternion_imu(MPU *mMPU)
{ 
	float init_Yaw, init_Pitch, init_Roll;
	init_Pitch = mMPU->Pitch/57.295780;
	init_Roll = mMPU->Roll/57.295780;
	init_Yaw = mMPU->Yaw/57.295780;
			mMPU->q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
			mMPU->q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��pitch
			mMPU->q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��roll
			mMPU->q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw

}

void IMUupdate(MPU *mMPU,uint8_t state) 
{

	uint8_t i = 0;
	float Yaw_sum = 0;
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;         
	
	// ����������
	norm = sqrt(mMPU->init_ax*mMPU->init_ax + mMPU->init_ay*mMPU->init_ay + mMPU->init_az*mMPU->init_az);       
	mMPU->init_ax = mMPU->init_ax / norm;
	mMPU->init_ay = mMPU->init_ay / norm;
	mMPU->init_az = mMPU->init_az / norm;      
	
	// ���Ʒ��������
	vx = 2*(mMPU->q1*mMPU->q3 - mMPU->q0*mMPU->q2);
	vy = 2*(mMPU->q0*mMPU->q1 + mMPU->q2*mMPU->q3);
	vz = mMPU->q0*mMPU->q0 - mMPU->q1*mMPU->q1 - mMPU->q2*mMPU->q2 + mMPU->q3*mMPU->q3;
	
	// ���������ͷ��򴫸��������ο�����֮��Ľ���˻����ܺ�
	ex = (mMPU->init_ay*vz - mMPU->init_az*vy);
	ey = (mMPU->init_az*vx - mMPU->init_ax*vz);
	ez = (mMPU->init_ax*vy - mMPU->init_ay*vx);
	
	mMPU->halfT=GET_NOWTIME();
	
	// ������������������
	mMPU->exInt = mMPU->exInt + ex*Ki;
	mMPU->eyInt = mMPU->eyInt + ey*Ki;
	mMPU->ezInt = mMPU->ezInt + ez*Ki;
	
	// ������������ǲ���
	mMPU->init_gx = mMPU->init_gx + Kp*ex + mMPU->exInt;
	mMPU->init_gy = mMPU->init_gy + Kp*ey + mMPU->eyInt;
	mMPU->init_gz = mMPU->init_gz + Kp*ez + mMPU->ezInt;
	
	// ������Ԫ���ʺ�������
	mMPU->q0 = mMPU->q0 + (-mMPU->q1*mMPU->init_gx - mMPU->q2*mMPU->init_gy - mMPU->q3*mMPU->init_gz)*mMPU->halfT;
	mMPU->q1 = mMPU->q1 + (mMPU->q0*mMPU->init_gx + mMPU->q2*mMPU->init_gz - mMPU->q3*mMPU->init_gy)*mMPU->halfT;
	mMPU->q2 = mMPU->q2 + (mMPU->q0*mMPU->init_gy - mMPU->q1*mMPU->init_gz + mMPU->q3*mMPU->init_gx)*mMPU->halfT;
	mMPU->q3 = mMPU->q3 + (mMPU->q0*mMPU->init_gz + mMPU->q1*mMPU->init_gy - mMPU->q2*mMPU->init_gx)*mMPU->halfT;  
	
	// ��������Ԫ��
	norm = sqrt(mMPU->q0*mMPU->q0 + mMPU->q1*mMPU->q1 + mMPU->q2*mMPU->q2 + mMPU->q3*mMPU->q3);
	mMPU->q0 = mMPU->q0 / norm;
	mMPU->q1 = mMPU->q1 / norm;
	mMPU->q2 = mMPU->q2 / norm;
	mMPU->q3 = mMPU->q3 / norm;
	
	//	//��Ԫ��ת����ŷ����
	mMPU->Roll = asin(2*mMPU->q0*mMPU->q2-2*mMPU->q1*mMPU->q3)/3.14*180;
	mMPU->Pitch = atan2(2*mMPU->q0*mMPU->q1+2*mMPU->q2*mMPU->q3,1-2*mMPU->q1*mMPU->q1-2*mMPU->q2*mMPU->q2)/3.14*180;//	Yaw = atan2(2*q0q3+2*q1*q2,1-2*q2*q2-2*q3*q3)/3->14*180;
	mMPU->Yaw = atan2(2*(mMPU->q1*mMPU->q2 + mMPU->q0*mMPU->q3),mMPU->q0*mMPU->q0+mMPU->q1*mMPU->q1-mMPU->q2*mMPU->q2-mMPU->q3*mMPU->q3)/3.14*180;
	if(mMPU->Yaw < 0 ){mMPU->Yaw = mMPU->Yaw + 360;}
	if(mMPU->Yaw > 360 ){mMPU->Yaw = mMPU->Yaw - 360;}
	
	for(i=0; i<9; i++)
	{
		mMPU->Yaw_Filter[i] = mMPU->Yaw_Filter[i+1];
		Yaw_sum += mMPU->Yaw_Filter[i];
	}
	
	mMPU->Yaw_Filter[9] = mMPU->Yaw;
	Yaw_sum += mMPU->Yaw_Filter[9];
	mMPU->Yaw = Yaw_sum/10;
	
	
	printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", mMPU->Yaw, mMPU->Pitch, mMPU->Roll);
	
	switch(state)
	{
		case i2c_L:
		if(IMU_count_L > 1000)
		{
			imu_flag_L++;
			if(imu_flag_L > 2000)
			{
				init_quaternion_imu(mMPU);   //�õ���ʼ����Ԫ��
				imu_flag_L = 0;
			}
			IMU_count_L = 0;
		}
		break;
		
		case i2c_R:
		if(IMU_count > 1000)
		{
			imu_flag++;
			if(imu_flag > 2000)
			{
				init_quaternion_imu(mMPU);   //�õ���ʼ����Ԫ��
				imu_flag = 0;
			}
			IMU_count = 0;
		}
		break;
		
		default:
		break;
		
	}
	
}

void AHRSupdate(MPU *mMPU) 
{
	 
	float norm;
	float hx, hy, hz, bz, by;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;

	/*����֮��ĳ���ʹ�ã����ټ���ʱ��*/
	//auxiliary variables to reduce number of repeated operations��
	float q0q0 = mMPU->q0*mMPU->q0;
	float q0q1 = mMPU->q0*mMPU->q1;
	float q0q2 = mMPU->q0*mMPU->q2;
	float q0q3 = mMPU->q0*mMPU->q3;
	float q1q1 = mMPU->q1*mMPU->q1;
	float q1q2 = mMPU->q1*mMPU->q2;
	float q1q3 = mMPU->q1*mMPU->q3;
	float q2q2 = mMPU->q2*mMPU->q2;   
	float q2q3 = mMPU->q2*mMPU->q3;
	float q3q3 = mMPU->q3*mMPU->q3;
          
	/*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
	//normalise the measurements
	norm = invSqrt(mMPU->init_ax*mMPU->init_ax + mMPU->init_ay*mMPU->init_ay + mMPU->init_az*mMPU->init_az);       
	mMPU->init_ax = mMPU->init_ax * norm;
	mMPU->init_ay = mMPU->init_ay * norm;
	mMPU->init_az = mMPU->init_az * norm;
	norm = invSqrt(mMPU->init_mx*mMPU->init_mx + mMPU->init_my*mMPU->init_my + mMPU->init_mz*mMPU->init_mz);          
	mMPU->init_mx = mMPU->init_mx * norm;
	mMPU->init_my = mMPU->init_my * norm;
	mMPU->init_mz = mMPU->init_mz * norm;         
        
	//compute reference direction of flux
	hx = 2*mMPU->init_mx*(0.5 - q2q2 - q3q3) + 2*mMPU->init_my*(q1q2 - q0q3) + 2*mMPU->init_mz*(q1q3 + q0q2);
	hy = 2*mMPU->init_mx*(q1q2 + q0q3) + 2*mMPU->init_my*(0.5 - q1q1 - q3q3) + 2*mMPU->init_mz*(q2q3 - q0q1);
	hz = 2*mMPU->init_mx*(q1q3 - q0q2) + 2*mMPU->init_my*(q2q3 + q0q1) + 2*mMPU->init_mz*(0.5 - q1q1 - q2q2);


//   bx = sqrtf((hx*hx) + (hy*hy));
	by = sqrtf((hx*hx) + (hy*hy));
	bz = hz;        
    
// estimated direction of gravity and flux (v and w)����������Ǵ���������ϵ������������ϵ��ת����ʽ(ת�þ���)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;


	wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           

// 	 error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (mMPU->init_ay*vz - mMPU->init_az*vy) + (mMPU->init_my*wz - mMPU->init_mz*wy);
	ey = (mMPU->init_az*vx - mMPU->init_ax*vz) + (mMPU->init_mz*wx - mMPU->init_mx*wz);
	ez = (mMPU->init_ax*vy - mMPU->init_ay*vx) + (mMPU->init_mx*wy - mMPU->init_my*wx);

    mMPU->halfT=GET_NOWTIME();		//�õ�ÿ����̬���µ����ڵ�һ��
//   
    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      
    {
		// integral error scaled integral gain
		mMPU->exInt = mMPU->exInt + ex*Ki * mMPU->halfT;			
		mMPU->eyInt = mMPU->eyInt + ey*Ki * mMPU->halfT;
		mMPU->ezInt = mMPU->ezInt + ez*Ki * mMPU->halfT;
		// adjusted gyroscope measurements
		mMPU->init_gx = mMPU->init_gx + Kp*ex + mMPU->exInt;
		mMPU->init_gy = mMPU->init_gy + Kp*ey + mMPU->eyInt;
		mMPU->init_gz = mMPU->init_gz + Kp*ez + mMPU->ezInt;
    }         

   // integrate quaternion rate and normalise����Ԫ�������㷨
	mMPU->q0 = mMPU->q0 + (-mMPU->q1*mMPU->init_gx - mMPU->q2*mMPU->init_gy - mMPU->q3*mMPU->init_gz)*mMPU->halfT;
	mMPU->q1 = mMPU->q1 + (mMPU->q0*mMPU->init_gx + mMPU->q2*mMPU->init_gz - mMPU->q3*mMPU->init_gy)*mMPU->halfT;
	mMPU->q2 = mMPU->q2 + (mMPU->q0*mMPU->init_gy - mMPU->q1*mMPU->init_gz + mMPU->q3*mMPU->init_gx)*mMPU->halfT;
	mMPU->q3 = mMPU->q3 + (mMPU->q0*mMPU->init_gz + mMPU->q1*mMPU->init_gy - mMPU->q2*mMPU->init_gx)*mMPU->halfT;  
        
   // normalise quaternion
	norm = invSqrt(mMPU->q0*mMPU->q0 + mMPU->q1*mMPU->q1 + mMPU->q2*mMPU->q2 + mMPU->q3*mMPU->q3);
	mMPU->q0 = mMPU->q0 * norm;       //w
	mMPU->q1 = mMPU->q1 * norm;       //x
	mMPU->q2 = mMPU->q2 * norm;       //y
	mMPU->q3 = mMPU->q3 * norm;       //z
        

	mMPU->Yaw = atan2(2*mMPU->q0*mMPU->q3 + 2*mMPU->q1*mMPU->q2,1 - 2*mMPU->q2*mMPU->q2 - 2*mMPU->q3*mMPU->q3) * 57.295780;
//	if(mMPU->Yaw < 0 ){mMPU->Yaw = mMPU->Yaw + 360;}
//	if(mMPU->Yaw > 360 ){mMPU->Yaw = mMPU->Yaw - 360;}
	mMPU->Pitch =  asin(2*mMPU->q2*mMPU->q3 + 2*mMPU->q0*mMPU->q1) * 57.295780; //�����ǣ���x��ת��	  
	mMPU->Roll  = -atan2(-2*mMPU->q0*mMPU->q2 + 2*mMPU->q1*mMPU->q3, -2 * mMPU->q1 * mMPU->q1 - 2 * mMPU->q2* mMPU->q2 + 1) * 57.295780; //�����ǣ���y��ת��
//	printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", mMPU->Yaw, mMPU->Pitch, mMPU->Roll);
}

/*******************************************************************************
��̬����-������Ԫ��	
*******************************************************************************/
void Get_Attitude(MPU *mMPU)
{ 
	AHRSupdate(mMPU);
}

/*******************************************************************************
У׼��������ƫ	
*******************************************************************************/
int get_gyro_bias(MPU *mMPU, uint8_t state)
{
	unsigned short int i;
	signed short int gyro[3];
	signed int gyro_x=0, gyro_y=0, gyro_z=0;
	static unsigned short count=0;
	unsigned char data_write[6];
	for(i=0;i<5000;i++)
	{
		if(!i2cread(MPU9150_Addr, Gyro_Xout_H, 6, data_write,state))
		{
			gyro[0] = ((((signed short int)data_write[0])<<8) | data_write[1]);
			gyro[1] = ((((signed short int)data_write[2])<<8) | data_write[3]);
			gyro[2] = ((((signed short int)data_write[4])<<8) | data_write[5]);
			gyro_x += gyro[0];
			gyro_y	+= gyro[1];
			gyro_z	+= gyro[2];
			count++;
		}
	}
	
	mMPU->Gyro_Xout_Offset = (float)gyro_x / count;
	mMPU->Gyro_Yout_Offset = (float)gyro_y / count;
	mMPU->Gyro_Zout_Offset = (float)gyro_z / count;

		return 0;
}
/*******************************************************************************
�õ�mag��Xmax��Xmin��Ymax��Ymin��Zmax��Zmin	
*******************************************************************************/
void get_compass_bias(MPU *mMPU, uint8_t state)
{
	Read_MPU9150_Mag(mMPU,state);

	if(mMPU->init_mx > mMPU->maxMagX)
		mMPU->maxMagX = mMPU->init_mx;
	if(mMPU->init_mx < mMPU->minMagX)
		mMPU->minMagX = mMPU->init_mx;

	if(mMPU->init_my > mMPU->maxMagY)
		mMPU->maxMagY = mMPU->init_my;
	if(mMPU->init_my < mMPU->minMagY)
		mMPU->minMagY = mMPU->init_my;

	if(mMPU->init_mz > mMPU->maxMagZ)
		mMPU->maxMagZ = mMPU->init_mz;
	if(mMPU->init_mz < mMPU->minMagZ)
		mMPU->minMagZ = mMPU->init_mz;

}
/*******************************************************************************
�ռ�У׼compass	
*******************************************************************************/
void compass_calibration(MPU *mMPU)
{ //���������Ӧ�����������Ϊ1
	if(((mMPU->maxMagX - mMPU->minMagX) >= (mMPU->maxMagY - mMPU->minMagY)) && ((mMPU->maxMagX - mMPU->minMagX) >= (mMPU->maxMagZ - mMPU->minMagZ)))
	{
		mMPU->MXgain = 1.0;
		mMPU->MYgain = (mMPU->maxMagX - mMPU->minMagX) / (mMPU->maxMagY - mMPU->minMagY);
		mMPU->MZgain = (mMPU->maxMagX - mMPU->minMagX) / (mMPU->maxMagZ - mMPU->minMagZ);
		mMPU->MXoffset = -0.5 * (mMPU->maxMagX + mMPU->minMagX);
		mMPU->MYoffset = -0.5 * mMPU->MYgain * (mMPU->maxMagY + mMPU->minMagY);
		mMPU->MZoffset = -0.5 * mMPU->MZgain * (mMPU->maxMagZ + mMPU->minMagZ);	 
	}
	if(((mMPU->maxMagY - mMPU->minMagY) > (mMPU->maxMagX - mMPU->minMagX)) && ((mMPU->maxMagY - mMPU->minMagY) >= (mMPU->maxMagZ - mMPU->minMagZ)))
	{
		mMPU->MXgain = (mMPU->maxMagY - mMPU->minMagY) / (mMPU->maxMagX - mMPU->minMagX);
		mMPU->MYgain = 1.0;
		mMPU->MZgain = (mMPU->maxMagY - mMPU->minMagY) / (mMPU->maxMagZ - mMPU->minMagZ);
		mMPU->MXoffset = -0.5 * mMPU->MXgain * (mMPU->maxMagX + mMPU->minMagX);
		mMPU->MYoffset = -0.5 * (mMPU->maxMagY + mMPU->minMagY);
		mMPU->MZoffset = -0.5 * mMPU->MZgain * (mMPU->maxMagZ + mMPU->minMagZ);    
	}
	if(((mMPU->maxMagZ - mMPU->minMagZ) > (mMPU->maxMagX - mMPU->minMagX)) && ((mMPU->maxMagZ - mMPU->minMagZ) > (mMPU->maxMagY - mMPU->minMagY)))
	{
		mMPU->MXgain = (mMPU->maxMagZ - mMPU->minMagZ) / (mMPU->maxMagX - mMPU->minMagX);
		mMPU->MYgain = (mMPU->maxMagZ - mMPU->minMagZ) / (mMPU->maxMagY - mMPU->minMagY);
		mMPU->MZgain = 1.0;
		mMPU->MXoffset = -0.5 * mMPU->MXgain * (mMPU->maxMagX + mMPU->minMagX);
		mMPU->MYoffset = -0.5 * mMPU->MYgain * (mMPU->maxMagY + mMPU->minMagY);
		mMPU->MZoffset = -0.5 * (mMPU->maxMagZ + mMPU->minMagZ);    
	}
     
}
/*******************************************************************************
��ȡcompass���ݣ�����compassУ׼	
*******************************************************************************/
void Read_MPU9150_Mag(MPU *mMPU, uint8_t state)
{
	signed short int mag[3];
	unsigned char tmp[7], data_write[1];
	  
	tmp[6]=0x00;
	data_write[0]=0x01;
	i2cread(Compass_Addr, Compass_ST1, 1, tmp+6,state);
	if(tmp[6] == 1)
	{
		i2cread(Compass_Addr, Compass_HXL, 6, tmp,state);
		mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
		mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
		mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];
		
		mag[0] = ((long)mag[0] * 286) >> 8;  //�����ȵ���	//286 288 300
		mag[1] = ((long)mag[1] * 288) >> 8;
		mag[2] = ((long)mag[2] * 300) >> 8;

		mMPU->init_mx = (float)mag[1];					
		mMPU->init_my = (float)mag[0];
		mMPU->init_mz = (float)-mag[2];
		i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write,state);	 //����compass��single measurement mode
	}  
}

/*******************************************************************************
��ȡcompass���ݣ��ڳ�ʼ��mpu9150���ȶ�����mag�����ݣ���Ϊǰ���ζ�ȡ��mag������
����оƬbug	
*******************************************************************************/
int Init_MPU9150_Mag(uint8_t state) 
{
	unsigned char data_write[3];
  
    data_write[0]=0x02;       
    data_write[1]=0x00;
    data_write[2]=0x01;
  
    i2cwrite(MPU9150_Addr, Bypass_Enable_Cfg, 1, data_write,state);	 //����bypass
    delay_ms(10);                     
    i2cwrite(MPU9150_Addr, User_Ctrl, 1, data_write+1,state);	 //�ر�MPU9150��I2C_MASTERģʽ������Ҫ�����
    delay_ms(10);
    i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write+2,state);	 //����compass��single measurement mode

    return 0;  
}

/*******************************************************************************
���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4�� 	
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
