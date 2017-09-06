#ifndef __DATA_H
#define	__DATA_H


#include "stm32f10x.h"

#define ADDR_BICYCLE_NET  0X0000
#define ADDR_BACK_NET     0X4F52
#define ADDR_GLOVE_NET    0X15DD

/******************************
new_data  新数据标志
pack_addr 接收到的数据的源地址  车把 00    后背  01   手套02
pack_cmd  数据指令
          车把         发送 addr glove   02     cmd 00:STATE_DATA_NONE           data none
							addr back    01     cmd 00:STATE_DATA_NONE           data none
							addr glove   02     cmd 01:STATE_DATA_SHOW_RECEIVE   data none
					   
					   接收 addr glove   02     cmd 01: STATE_DATA_SHOW_RECEIVE  data 心率 体温  ADC值 
					   
		  手套         接收 addr bicycle 00   	cmd 01: STATE_DATA_SHOW_SEND     data none
					   发送	addr bicycle 00	  	cmd 01: STATE_DATA_SHOW_SEND     data 心率 体温  ADC值 
		  
		  后背         
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
