#include "AttitudeRedress.h"   
#include "AHRS_Attitude.h"
#include "usart.h"

extern MPU mMPU_L;
extern MPU mMPU_R;
extern MPU mMPU_B;
 
 
void send_cmd(uint8_t cmd)
{
//	Usart_SendByte(USART1,0xAA);
//	Usart_SendByte(USART1,0x00);
//	Usart_SendByte(USART1,0x01);
//	Usart_SendByte(USART1,0x55);
	Usart_SendByte(USART1,'s');
	Usart_SendByte(USART1,'2');
	Usart_SendByte(USART1,CMD);
	Usart_SendByte(USART1,cmd);
}

void send_euler(void)
{
	uint8_t pitch = 0;
	uint8_t roll = 0;
	uint8_t yaw = 0;
	
//	Usart_SendByte(USART1,0xAA);
//	Usart_SendByte(USART1,0x00);
//	Usart_SendByte(USART1,0x01);
//	Usart_SendByte(USART1,0x55);
	Usart_SendByte(USART1,'s');
	Usart_SendByte(USART1,'2');
	Usart_SendByte(USART1,DATA);
	
	if(mMPU_R.Pitch >= 0)
		pitch = mMPU_R.Pitch/2;
	else
	{
		pitch = (uint8_t)mMPU_R.Pitch/2;
		pitch |= 0x80;
	}
		Usart_SendByte(USART1,pitch);
	
	if(mMPU_R.Roll >= 0)
		roll = mMPU_R.Roll/2;
	else
	{
		roll = (uint8_t)mMPU_R.Roll/2;
		roll |= 0x80;
	}
	Usart_SendByte(USART1,roll);
	
	if(mMPU_R.Yaw >= 0)
		yaw = mMPU_R.Yaw/2;
	else
	{
		yaw = (uint8_t)mMPU_R.Yaw/2;
		yaw |= 0x80;
	}
	Usart_SendByte(USART1,yaw);
	
	
	if(mMPU_L.Pitch >= 0)
		pitch = mMPU_L.Pitch/2;
	else
	{
		pitch = (uint8_t)mMPU_L.Pitch/2;
		pitch |= 0x80;
	}
	Usart_SendByte(USART1,pitch);
	
	if(mMPU_L.Roll >= 0)
		roll = mMPU_L.Roll/2;
	else
	{
		roll = (uint8_t)mMPU_L.Roll/2;
		roll |= 0x80;
	}
	Usart_SendByte(USART1,roll);
	
	if(mMPU_L.Yaw >= 0)
		yaw = mMPU_L.Yaw/2;
	else
	{
		yaw = (uint8_t)mMPU_L.Yaw/2;
		yaw |= 0x80;
	}
	Usart_SendByte(USART1,yaw);
	
	
	if(mMPU_B.Pitch >= 0)
		pitch = mMPU_B.Pitch/2;
	else
	{
		pitch = (uint8_t)mMPU_B.Pitch/2;
		pitch |= 0x80;
	}
	Usart_SendByte(USART1,pitch);
	
	if(mMPU_B.Roll >= 0)
		roll = mMPU_B.Roll/2;
	else
	{
		roll = (uint8_t)mMPU_B.Roll/2;
		roll |= 0x80;
	}
	Usart_SendByte(USART1,roll);
	
	if(mMPU_B.Yaw >= 0)
		yaw = mMPU_B.Yaw/2;
	else
	{
		yaw = (uint8_t)mMPU_B.Yaw/2;
		yaw |= 0x80;
	}
	Usart_SendByte(USART1,yaw);
}

