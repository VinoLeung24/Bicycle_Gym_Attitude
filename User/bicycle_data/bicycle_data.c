#include "bicycle_data.h"   
#include "usart.h"

uint8_t pack_head[3] = {0x5a,0xaa,0xa2};

void send_bicycle(uint16_t addr, uint8_t len,uint8_t *data,uint8_t data_size)
{
	Usart_SendStr_length(USART1,pack_head,3);
	Usart_SendByte(USART1,len);
	Usart_SendByte(USART1, addr >> 8);
	Usart_SendByte(USART1, addr & 0x00ff);
	Usart_SendStr_length(USART1,data,data_size);
}

/*********************************************END OF FILE**********************/
