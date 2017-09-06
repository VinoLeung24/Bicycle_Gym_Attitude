/******************************************************************************
** ���̣�   STM32-MPU9x5x����
** ���ߣ�	vino
** �޸���־��

2016-8-7
����IMU�㷨

2016-8-10
 ��ʹ��DMP ����ֱ�Ӷ�ȡ�Ĵ���������ʼ��MPU9250 �� �ɼ�����
 ����MPU9x5x
 
 2016-8-15
 ���ת������ϵ������Ư������

********************************************************************************/

#include "AHRS_Attitude.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "I2C.h"
#include "math.h"
#include "time.h"
#include "mpu9250.h"
#include "time.h"
#include "AttitudeRedress.h"   

#define RIGHT 1
#define LEFT  0

MPU mMPU_L;
MPU mMPU_R;
MPU mMPU_B;

BICYCLE mBicycle;

extern int BPM;
uint8_t temp[5];

uint8_t mpu_send;

extern uint8_t sys_count;

int main(void)
{ 
	static int Yaw_L,Yaw_R;
	static uint8_t direction;
	static uint8_t twist;
	static uint8_t back_flag,shouder_flag;
	static uint8_t warm1_flag = 1;
	
	SYSTICK_INIT();
	TIM_INIT(); 
	delay_init();	    								//��ʱ������ʼ��	  
	USART1_Config();
	USART1_DMA_Config();
	IIC_Init_R();										//I2C��ʼ��//SDA-PC11   SCL-PC12	 
	IIC_Init_L();
	IIC_Init_B();
	delay_ms(500);
	
	init_mpu9250(&mMPU_R,i2c_R);
	
	get_compass_bias(&mMPU_R,i2c_R); 
	compass_calibration(&mMPU_R);
	init_quaternion(&mMPU_R,i2c_R);  			 		//�õ���ʼ����Ԫ��
	
	init_mpu9250(&mMPU_L,i2c_L);
	
	
	get_compass_bias(&mMPU_L,i2c_L); 
	compass_calibration(&mMPU_L);
	init_quaternion(&mMPU_L,i2c_L);   					//�õ���ʼ����Ԫ��
	
	init_mpu9250(&mMPU_B,i2c_B);
	
	get_compass_bias(&mMPU_B,i2c_B); 
	compass_calibration(&mMPU_B);
	init_quaternion(&mMPU_B,i2c_B);   					//�õ���ʼ����Ԫ��

	while(1)
	{
		switch(mBicycle.outer_status)
		{
			case STATE_START_MODE:
				
			break;
			
			case STATE_FREEDOM_MODE:					//����ģʽ  (������) ������ʾ ����ֵ �ĵ�ͼ �ٶ� ����
			    get_mpu9150_data(&mMPU_L,i2c_L);
				Get_Attitude(&mMPU_L);

				get_mpu9150_data(&mMPU_R,i2c_R);
				Get_Attitude(&mMPU_R);

				if((mMPU_R.Pitch>=13) && (mMPU_R.Pitch < 35) && (mMPU_L.Pitch >= 13) && (mMPU_L.Pitch < 35))
				{
					if((back_flag == 1) || (shouder_flag == 1))
					{
						send_cmd(STATE_DATA_RIGHT);
						delay_ms(20);
					}
					back_flag = 0;
					shouder_flag = 0;
				}
				
				if(((mMPU_R.Pitch>=35) && (mMPU_R.Pitch <= 50)) || ((mMPU_L.Pitch >= 35) && (mMPU_L.Pitch <= 50)))
				{
					if(shouder_flag == 0)
					{
						send_cmd(STATE_SHOUDER_ERR);
						delay_ms(20);
					}
					shouder_flag = 1;
				}
				else
				{
					if(shouder_flag == 1)
					{
						send_cmd(STATE_SHOUDER_OK);
						delay_ms(20);
					}
					shouder_flag = 0;
				}
				
				if((mMPU_R.Pitch >= 44) || (mMPU_L.Pitch >= 44) || (mMPU_R.Pitch < 13) || (mMPU_L.Pitch < 13))
				{
					if(back_flag == 0)
					{
						send_cmd(STATE_BACK_ERR);
						delay_ms(20);
					}
					back_flag = 1;
				}
				else
				{
					if(back_flag == 1)
					{
						send_cmd(STATE_BACK_OK);
						delay_ms(20);
					}
					back_flag = 0;
				}
			break;     
			     
			case STATE_SPORT_MODE_LEVEL:				//����ģʽ  (ƽ·ģʽ) ������ʾ ����ֵ �ĵ�ͼ �ٶ� ����
				get_mpu9150_data(&mMPU_L,i2c_L);
				Get_Attitude(&mMPU_L);

				get_mpu9150_data(&mMPU_R,i2c_R);
				Get_Attitude(&mMPU_R);

				if((mMPU_R.Pitch>=13) && (mMPU_R.Pitch < 35) && (mMPU_L.Pitch >= 13) && (mMPU_L.Pitch < 35))
				{
					if((back_flag == 1) || (shouder_flag == 1))
					{
						send_cmd(STATE_DATA_RIGHT);
						delay_ms(20);
					}
					back_flag = 0;
					shouder_flag = 0;
				}
				if(((mMPU_R.Pitch>=35) && (mMPU_R.Pitch <= 50)) || ((mMPU_L.Pitch >= 35) && (mMPU_L.Pitch <= 50)))
				{
					if(shouder_flag == 0)
					{
						send_cmd(STATE_SHOUDER_ERR);
						delay_ms(20);
					}
					shouder_flag = 1;
				}
				else
				{
					if(shouder_flag == 1)
					{
						send_cmd(STATE_SHOUDER_OK);
						delay_ms(20);
					}
					shouder_flag = 0;
				}
				if((mMPU_R.Pitch >= 44) || (mMPU_L.Pitch >= 44) || (mMPU_R.Pitch < 13) || (mMPU_L.Pitch < 13))
				{
					if(back_flag == 0)
					{
						send_cmd(STATE_BACK_ERR);
						delay_ms(20);
					}
					back_flag = 1;
				}
				else
				{
					if(back_flag == 1)
					{
						send_cmd(STATE_BACK_OK);
						delay_ms(20);
					}
					back_flag = 0;
				}
			break;
			     	
			case STATE_SPORT_MODE_CLIMB:				//����ģʽ  (����ģʽ) ������ʾ ����ֵ �ĵ�ͼ �ٶ� ����
			    get_mpu9150_data(&mMPU_L,i2c_L);
				Get_Attitude(&mMPU_L);

				get_mpu9150_data(&mMPU_R,i2c_R);
				Get_Attitude(&mMPU_R);
				if(mMPU_R.Pitch >= 0 && mMPU_R.Pitch <= 15 && mMPU_L.Pitch >= 0 && mMPU_L.Pitch <= 15)
				{
					if((back_flag == 1) || (shouder_flag == 1))
					{
						send_cmd(STATE_DATA_RIGHT);
						delay_ms(20);
					}
					back_flag = 0;
					shouder_flag = 0;
				}
				else
				{
					if((back_flag == 0) || (shouder_flag == 0))
					{
						send_cmd(STATE_CLIMB_ERR);
						delay_ms(20);
					}
					back_flag = 1;
					shouder_flag = 1;
				}
			break;       
			    
			case STATE_SPORT_MODE_WARM1:				//����ģʽ  (����ģʽ1)
				get_mpu9150_data(&mMPU_L,i2c_L);
				Get_Attitude(&mMPU_L);

				get_mpu9150_data(&mMPU_R,i2c_R);
				Get_Attitude(&mMPU_R);
			
				if((mMPU_R.Pitch <= 33) && (mMPU_L.Pitch <= 33))
				{
					
					if(warm1_flag == 1)
					{
						send_cmd(STATE_WARM1_OK);
						delay_ms(20);
						send_cmd(STATE_WARM1_OK);
						delay_ms(20);
					}	
					warm1_flag = 0;
				}
				else
				{
					if(warm1_flag == 0)
					{
						send_cmd(STATE_WARM1_ERR);
						delay_ms(20);
						send_cmd(STATE_WARM1_ERR);
						delay_ms(20);
					}
					warm1_flag = 1;					
				}
			break;
			
			case STATE_SPORT_MODE_WARM2:				//����ģʽ  (����ģʽ2)
				get_mpu9150_data(&mMPU_R,i2c_R);
				Get_Attitude(&mMPU_R);
				if(((int)mMPU_R.Yaw - Yaw_R) > 40) 
				{
					if(direction == LEFT)
					{
						direction = RIGHT;
						send_cmd(STATE_WARM2_RIGHT);
						delay_ms(20);
					}
				}	
				
				if((Yaw_R - (int)mMPU_R.Yaw) > 40)
				{
					if(direction == RIGHT)
					{
						direction = LEFT;
						send_cmd(STATE_WARM2_LEFT);
						delay_ms(20);
					}
				}
			break;
		
			case STATE_SPORT_MODE_WARM3:				//����ģʽ  (����ģʽ3)			
				get_mpu9150_data(&mMPU_L,i2c_L);
				Get_Attitude(&mMPU_L);

				get_mpu9150_data(&mMPU_R,i2c_R);
				Get_Attitude(&mMPU_R);

				if((mMPU_L.Roll > 0) && (mMPU_L.Roll < 105))
				{
					if(twist == LEFT)
					{
						twist = RIGHT;
						send_cmd(STATE_WARM3_OK);
						delay_ms(20);
					}
				}
				
				if((mMPU_R.Roll > -95) && (mMPU_R.Roll < 0))
				{
					if(twist == RIGHT)
					{   
						twist = LEFT;
						send_cmd(STATE_WARM3_OK);
						delay_ms(20);
					}
				}
			break;
		
			case STATE_USER_REMENBER_DATA:
				get_mpu9150_data(&mMPU_L,i2c_L);
				Get_Attitude(&mMPU_L);
				get_mpu9150_data(&mMPU_R,i2c_R);
				Get_Attitude(&mMPU_R);
				get_mpu9150_data(&mMPU_B,i2c_B);
				Get_Attitude(&mMPU_B);
				if(sys_count > 200)
				{
					sys_count = 0;
					send_euler();
				}
			break;
			
			case STATE_USER_CHECK:
				get_mpu9150_data(&mMPU_L,i2c_L);
				Get_Attitude(&mMPU_L);
				get_mpu9150_data(&mMPU_R,i2c_R);
				Get_Attitude(&mMPU_R);
				get_mpu9150_data(&mMPU_B,i2c_B);
				Get_Attitude(&mMPU_B);
				if(sys_count > 200)
				{
					sys_count = 0;
					send_euler();
				}
			break;
				
			case STATE_CALIBRATION:							
				get_mpu9150_data(&mMPU_L,i2c_L);
				Get_Attitude(&mMPU_L);

				get_mpu9150_data(&mMPU_R,i2c_R);
				Get_Attitude(&mMPU_R);
			
				Yaw_L = (int)mMPU_L.Yaw;
				Yaw_R = (int)mMPU_R.Yaw;
			
				mBicycle.outer_status = STATE_START_MODE;
			break;
		
		}    
	}
}

