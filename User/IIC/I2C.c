#include "I2C.h"
#include "delay.h"
#include "AHRS_Attitude.h"

//  MPU9250_LEFT
//	SCL_L		PE3
//	SDA_L		PE2
	
//  MPU9250_RIGHT
//	SCL_R		PG7
//	SDA_R		PG6

//  MPU9250_BUTTON
//	SCL_B		PC11
//	SDA_B		PC10

//初始化IIC
void IIC_Init_R(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOG, ENABLE );	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
 
	IIC_SCL_R=1;
	IIC_SDA_R=1;
}

void IIC_Init_L(void) 
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOE, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
 
	IIC_SCL_L=1;
	IIC_SDA_L=1;

}

void IIC_Init_B(void) 
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
 
	IIC_SCL_B=1;
	IIC_SDA_B=1;

}

//产生IIC起始信号
void IIC_Start_R(void)
{
	SDA_OUT_R();     //sda线输出
	IIC_SDA_R=1;	  	  
	IIC_SCL_R=1;
	delay_us(4);
 	IIC_SDA_R=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_R=0;//钳住I2C总线，准备发送或接收数据 
}	

void IIC_Start_L(void)
{
	SDA_OUT_L();     //sda线输出
	IIC_SDA_L=1;	  	  
	IIC_SCL_L=1;
	delay_us(4);
 	IIC_SDA_L=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_L=0;//钳住I2C总线，准备发送或接收数据 
}	  

void IIC_Start_B(void)
{
	SDA_OUT_B();     //sda线输出
	IIC_SDA_B=1;	  	  
	IIC_SCL_B=1;
	delay_us(4);
 	IIC_SDA_B=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_B=0;//钳住I2C总线，准备发送或接收数据 
}	  

//产生IIC停止信号
void IIC_Stop_R(void)
{
	SDA_OUT_R();//sda线输出
	IIC_SCL_R=0;
	IIC_SDA_R=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_R=1; 
	IIC_SDA_R=1;//发送I2C总线结束信号
	delay_us(4);							   	
}

void IIC_Stop_L(void)
{
	SDA_OUT_L();//sda线输出
	IIC_SCL_L=0;
	IIC_SDA_L=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_L=1; 
	IIC_SDA_L=1;//发送I2C总线结束信号
	delay_us(4);							   	
}

void IIC_Stop_B(void)
{
	SDA_OUT_B();//sda线输出
	IIC_SCL_B=0;
	IIC_SDA_B=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_B=1; 
	IIC_SDA_B=1;//发送I2C总线结束信号
	delay_us(4);							   	
}


//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack_R(void)
{
	u8 ucErrTime=0;
	SDA_IN_R();      //SDA设置为输入  
	IIC_SDA_R=1;delay_us(1);	   
	IIC_SCL_R=1;delay_us(1);	 
	while(READ_SDA_R)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop_R();
			return 1;
		}
	}
	IIC_SCL_R=0;//时钟输出0 	   
	return 0;  
} 

u8 IIC_Wait_Ack_L(void)
{
	u8 ucErrTime=0;
	SDA_IN_L();      //SDA设置为输入  
	IIC_SDA_L=1;delay_us(1);	   
	IIC_SCL_L=1;delay_us(1);	 
	while(READ_SDA_L)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop_L();
			return 1;
		}
	}
	IIC_SCL_L=0;//时钟输出0 	   
	return 0;  
} 

u8 IIC_Wait_Ack_B(void)
{
	u8 ucErrTime=0;
	SDA_IN_B();      //SDA设置为输入  
	IIC_SDA_B=1;delay_us(1);	   
	IIC_SCL_B=1;delay_us(1);	 
	while(READ_SDA_B)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop_B();
			return 1;
		}
	}
	IIC_SCL_B=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void IIC_Ack_R(void)
{
	IIC_SCL_R=0;
	SDA_OUT_R();
	IIC_SDA_R=0;
	delay_us(2);
	IIC_SCL_R=1;
	delay_us(2);
	IIC_SCL_R=0;
}

void IIC_Ack_L(void)
{
	IIC_SCL_L=0;
	SDA_OUT_L();
	IIC_SDA_L=0;
	delay_us(2);
	IIC_SCL_L=1;
	delay_us(2);
	IIC_SCL_L=0;
}

void IIC_Ack_B(void)
{
	IIC_SCL_B=0;
	SDA_OUT_B();
	IIC_SDA_B=0;
	delay_us(2);
	IIC_SCL_B=1;
	delay_us(2);
	IIC_SCL_B=0;
}

//不产生ACK应答		    
void IIC_NAck_R(void)
{
	IIC_SCL_R=0;
	SDA_OUT_R();
	IIC_SDA_R=1;
	delay_us(2);
	IIC_SCL_R=1;
	delay_us(2);
	IIC_SCL_R=0;
}	

void IIC_NAck_L(void)
{
	IIC_SCL_L=0;
	SDA_OUT_L();
	IIC_SDA_L=1;
	delay_us(2);
	IIC_SCL_L=1;
	delay_us(2);
	IIC_SCL_L=0;
}	

void IIC_NAck_B(void)
{
	IIC_SCL_B=0;
	SDA_OUT_B();
	IIC_SDA_B=1;
	delay_us(2);
	IIC_SCL_B=1;
	delay_us(2);
	IIC_SCL_B=0;
}	

//IIC发送一个字节	  
void IIC_Send_Byte_R(u8 txd)
{                        
    u8 t;   
	SDA_OUT_R(); 	    
    IIC_SCL_R=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA_R=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL_R=1;
		delay_us(2); 
		IIC_SCL_R=0;	
		delay_us(2);
    }	 
} 	 

void IIC_Send_Byte_L(u8 txd)
{                        
    u8 t;   
	SDA_OUT_L(); 	    
    IIC_SCL_L=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA_L=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL_L=1;
		delay_us(2); 
		IIC_SCL_L=0;	
		delay_us(2);
    }	 
} 

void IIC_Send_Byte_B(u8 txd)
{                        
    u8 t;   
	SDA_OUT_B(); 	    
    IIC_SCL_B=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA_B=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL_B=1;
		delay_us(2); 
		IIC_SCL_B=0;	
		delay_us(2);
    }	 
} 


//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte_R(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN_R();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_R=0; 
        delay_us(2);
		IIC_SCL_R=1;
        receive<<=1;
        if(READ_SDA_R)receive++;   
		delay_us(1); 
    }					 

    return receive;
}

u8 IIC_Read_Byte_L(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN_L();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_L=0; 
        delay_us(2);
		IIC_SCL_L=1;
        receive<<=1;
        if(READ_SDA_L)receive++;   
		delay_us(1); 
    }					 

    return receive;
}

u8 IIC_Read_Byte_B(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN_B();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_B=0; 
        delay_us(2);
		IIC_SCL_B=1;
        receive<<=1;
        if(READ_SDA_B)receive++;   
		delay_us(1); 
    }					 

    return receive;
}

bool i2cReadBuffer_R(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
	IIC_Start_R();
	IIC_Send_Byte_R(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack_R())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop_R();
		return false;//0
	}
	IIC_Send_Byte_R(reg);
	IIC_Wait_Ack_R();
	IIC_Start_R();
	IIC_Send_Byte_R(addr << 1 | I2C_Receiver);
    IIC_Wait_Ack_R();
	while (len) 
    {
        *buf = IIC_Read_Byte_R(1);
        if (len == 1)
            IIC_NAck_R();
        else
            IIC_Ack_R();
        buf++;
        len--;
    }
    IIC_Stop_R();
    return true;
}

bool i2cReadBuffer_L(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
	IIC_Start_L();
	IIC_Send_Byte_L(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack_L())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop_L();
		return false;//0
	}
	IIC_Send_Byte_L(reg);
	IIC_Wait_Ack_L();
	IIC_Start_L();
	IIC_Send_Byte_L(addr << 1 | I2C_Receiver);
    IIC_Wait_Ack_L();
	while (len) 
    {
        *buf = IIC_Read_Byte_L(1);
        if (len == 1)
            IIC_NAck_L();
        else
            IIC_Ack_L();
        buf++;
        len--;
    }
    IIC_Stop_L();
    return true;
}

bool i2cReadBuffer_B(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
	IIC_Start_B();
	IIC_Send_Byte_B(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack_B())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop_B();
		return false;//0
	}
	IIC_Send_Byte_B(reg);
	IIC_Wait_Ack_B();
	IIC_Start_B();
	IIC_Send_Byte_B(addr << 1 | I2C_Receiver);
    IIC_Wait_Ack_B();
	while (len) 
    {
        *buf = IIC_Read_Byte_B(1);
        if (len == 1)
            IIC_NAck_B();
        else
            IIC_Ack_B();
        buf++;
        len--;
    }
    IIC_Stop_B();
    return true;
}

bool i2cWriteBuffer_R(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	IIC_Start_R();
	IIC_Send_Byte_R(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack_R())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop_R();
		return false;//0
	}
	IIC_Send_Byte_R(reg);
	IIC_Wait_Ack_R();
	for(i = 0; i < len; i++)
	{
		IIC_Send_Byte_R(data[i]);
		if(IIC_Wait_Ack_R())//接收应答成功返回0，不成功返回1
		{
			IIC_Stop_R();
			return false;//0
		}
	}
	IIC_Stop_R();
	return true;//1
}

bool i2cWriteBuffer_L(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	IIC_Start_L();
	IIC_Send_Byte_L(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack_L())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop_L();
		return false;//0
	}
	IIC_Send_Byte_L(reg);
	IIC_Wait_Ack_L();
	for(i = 0; i < len; i++)
	{
		IIC_Send_Byte_L(data[i]);
		if(IIC_Wait_Ack_L())//接收应答成功返回0，不成功返回1
		{
			IIC_Stop_L();
			return false;//0
		}
	}
	IIC_Stop_L();
	return true;//1
}

bool i2cWriteBuffer_B(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	IIC_Start_B();
	IIC_Send_Byte_B(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack_B())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop_B();
		return false;//0
	}
	IIC_Send_Byte_B(reg);
	IIC_Wait_Ack_B();
	for(i = 0; i < len; i++)
	{
		IIC_Send_Byte_B(data[i]);
		if(IIC_Wait_Ack_B())//接收应答成功返回0，不成功返回1
		{
			IIC_Stop_B();
			return false;//0
		}
	}
	IIC_Stop_B();
	return true;//1
}

bool i2cSigleWrite_R(uint8_t addr, uint8_t reg, uint8_t data)
{
	IIC_Start_R();
	IIC_Send_Byte_R(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack_R())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop_R();
		return false;//0
	}
	IIC_Send_Byte_R(reg);
	IIC_Wait_Ack_R();
		IIC_Send_Byte_R(data);
		if(IIC_Wait_Ack_R())//接收应答成功返回0，不成功返回1
		{
			IIC_Stop_R();
			return false;//0
		}
	IIC_Stop_R();
	return true;//1
}

bool i2cSigleWrite_L(uint8_t addr, uint8_t reg, uint8_t data)
{
	IIC_Start_L();
	IIC_Send_Byte_L(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack_L())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop_L();
		return false;//0
	}
	IIC_Send_Byte_L(reg);
	IIC_Wait_Ack_L();
		IIC_Send_Byte_L(data);
		if(IIC_Wait_Ack_L())//接收应答成功返回0，不成功返回1
		{
			IIC_Stop_L();
			return false;//0
		}
	IIC_Stop_L();
	return true;//1
}

bool i2cSigleWrite_B(uint8_t addr, uint8_t reg, uint8_t data)
{
	IIC_Start_B();
	IIC_Send_Byte_B(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack_B())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop_B();
		return false;//0
	}
	IIC_Send_Byte_B(reg);
	IIC_Wait_Ack_B();
		IIC_Send_Byte_B(data);
		if(IIC_Wait_Ack_B())//接收应答成功返回0，不成功返回1
		{
			IIC_Stop_B();
			return false;//0
		}
	IIC_Stop_B();
	return true;//1
}

bool I2CSigleWrite(uint8_t addr, uint8_t reg, uint8_t data,uint8_t state)
{
	switch(state)
	{
		case i2c_L:
		i2cSigleWrite_L(addr, reg, data);
		break;
		
		case i2c_R:
		i2cSigleWrite_R(addr, reg, data);
		break;
		
		case i2c_B:
		i2cSigleWrite_B(addr, reg, data);
		break;
		
		default:
		break;
	}
}


uint8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data,uint8_t state)
{
	switch(state)
	{	
		case i2c_L:
			if(i2cWriteBuffer_L(addr,reg,len,data))
			{
				return TRUE;
			}
			else
			{
				return FALSE;
			}
			
		break;
		
		case i2c_R:
			if(i2cWriteBuffer_R(addr,reg,len,data))
			{
				return TRUE;
			}
			else
			{
				return FALSE;
			}
		break;
			
		case i2c_B:
			if(i2cWriteBuffer_B(addr,reg,len,data))
			{
				return TRUE;
			}
			else
			{
				return FALSE;
			}
		break;
		
		default:
		break;
	}

}

uint8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf,uint8_t state)
{
	switch(state)
	{	
		case i2c_L:
			if(i2cReadBuffer_L(addr,reg,len,buf))
			{
				return TRUE;
			}
			else
			{
				return FALSE;
			}
		break;
		
		case i2c_R:
			if(i2cReadBuffer_R(addr,reg,len,buf))
			{
				return TRUE;
			}
			else
			{
				return FALSE;
			}
		break;
			
		case i2c_B:
			if(i2cReadBuffer_B(addr,reg,len,buf))
			{
				return TRUE;
			}
			else
			{
				return FALSE;
			}
		break;
		
		default:
		break;
	}
	
}


















