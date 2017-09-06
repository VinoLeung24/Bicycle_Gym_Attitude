#ifndef __I2C_H
#define __I2C_H
#include "stm32f10x.h"
#include "sys.h"

//SDA    PG7
//SCL    PG6

//IO��������
//#define SDA_IN()  {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=8<<12;}
//#define SDA_OUT() {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=3<<12;}
#define SDA_IN_R()  {GPIOG->CRL&=0XF0FFFFFF;GPIOG->CRL|=0X08000000;}
#define SDA_OUT_R() {GPIOG->CRL&=0XF0FFFFFF;GPIOG->CRL|=0X03000000;}


//SDA_L   PE2
//SCL_L   PE3
#define SDA_IN_L()  {GPIOE->CRL&=0XFFFFF0FF;GPIOE->CRL|=0X00000800;}
#define SDA_OUT_L() {GPIOE->CRL&=0XFFFFF0FF;GPIOE->CRL|=0X00000300;}

//SDA_B   PC10
//SCL_B   PC11
#define SDA_IN_B()  {GPIOC->CRH&=0XFFFFF0FF;GPIOC->CRH|=0X00000800;}
#define SDA_OUT_B() {GPIOC->CRH&=0XFFFFF0FF;GPIOC->CRH|=0X00000300;}


//IO��������	 
#define IIC_SCL_R    PGout(7) //SCL
#define IIC_SDA_R    PGout(6) //SDA	 
#define READ_SDA_R   PGin(6)  //����SDA 

//IO��������	 
#define IIC_SCL_L    PEout(3) //SCL
#define IIC_SDA_L    PEout(2) //SDA	 
#define READ_SDA_L   PEin(2)  //����SDA 

//IO��������	 
#define IIC_SCL_B    PCout(11) //SCL
#define IIC_SDA_B    PCout(10) //SDA	 
#define READ_SDA_B   PCin(10)  //����SDA 

//0��ʾд
#define	I2C_Transmitter   0
//����ʾ��
#define	I2C_Receiver      1	

#define true 1
#define false 0 
#define bool  uint8_t


#define TRUE  0
#define FALSE -1




//IIC���в�������
void IIC_Init_R(void);                //��ʼ��IIC��IO��				 
void IIC_Start_R(void);				//����IIC��ʼ�ź�
void IIC_Stop_R(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte_R(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte_R(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack_R(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack_R(void);					//IIC����ACK�ź�
void IIC_NAck_R(void);				//IIC������ACK�ź�

bool i2cSigleWrite_R(uint8_t addr, uint8_t reg, uint8_t data);
bool i2cWriteBuffer_R(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
bool i2cReadBuffer_R(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);

//IIC���в�������
void IIC_Init_L(void);                //��ʼ��IIC��IO��				 
void IIC_Start_L(void);				//����IIC��ʼ�ź�
void IIC_Stop_L(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte_L(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte_L(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack_L(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack_L(void);					//IIC����ACK�ź�
void IIC_NAck_L(void);				//IIC������ACK�ź�

bool i2cSigleWrite_L(uint8_t addr, uint8_t reg, uint8_t data);
bool i2cWriteBuffer_L(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
bool i2cReadBuffer_L(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);

//IIC���в�������
void IIC_Init_B(void);                //��ʼ��IIC��IO��				 
void IIC_Start_B(void);				//����IIC��ʼ�ź�
void IIC_Stop_B(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte_B(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte_B(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack_B(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack_B(void);					//IIC����ACK�ź�
void IIC_NAck_B(void);				//IIC������ACK�ź�

bool i2cSigleWrite_B(uint8_t addr, uint8_t reg, uint8_t data);
bool i2cWriteBuffer_B(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
bool i2cReadBuffer_B(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);

uint8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data,uint8_t state);
uint8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf,uint8_t state);
bool I2CSigleWrite(uint8_t addr, uint8_t reg, uint8_t data,uint8_t state);
#endif
