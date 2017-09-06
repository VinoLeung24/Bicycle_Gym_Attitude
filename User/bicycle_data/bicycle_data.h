#ifndef __DATA_H
#define	__DATA_H


#include "stm32f10x.h"

#define ADDR_BICYCLE_NET  0X0000
#define ADDR_BACK_NET     0X4F52
#define ADDR_GLOVE_NET    0X15DD

/******************************
new_data  �����ݱ�־
pack_addr ���յ������ݵ�Դ��ַ  ���� 00    ��  01   ����02
pack_cmd  ����ָ��
          ����         ���� addr glove   02     cmd 00:STATE_DATA_NONE           data none
							addr back    01     cmd 00:STATE_DATA_NONE           data none
							addr glove   02     cmd 01:STATE_DATA_SHOW_RECEIVE   data none
					   
					   ���� addr glove   02     cmd 01: STATE_DATA_SHOW_RECEIVE  data ���� ����  ADCֵ 
					   
		  ����         ���� addr bicycle 00   	cmd 01: STATE_DATA_SHOW_SEND     data none
					   ����	addr bicycle 00	  	cmd 01: STATE_DATA_SHOW_SEND     data ���� ����  ADCֵ 
		  
		  ��         
********************************/

typedef struct
{
	uint8_t new_data;
	uint8_t pack_addr;
	uint8_t pack_cmd;
	uint8_t pack_size;
	uint8_t pack_data[9];
}USART5_RX;

enum data_status
{
	STATE_DATA_NONE,
	STATE_DATA_SHOW_RECEIVE,
};

enum addr
{
	ADDR_BICYCLE,
	ADDR_BACK,
	ADDR_GLOVE,
	
};

void send_bicycle(uint16_t addr, uint8_t len,uint8_t *data,uint8_t data_size);
#endif /* __DATA_H */
