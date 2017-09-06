#ifndef __ATTITUDEREDRESS_H
#define	__ATTITUDEREDRESS_H


#include "stm32f10x.h"
#include "AHRS_Attitude.h"	

#define ADDR_BICYCLE_NET  0X0000
#define ADDR_BACK_NET     0X4F52
#define ADDR_GLOVE_NET    0X15DD
#define USART1_SEND_SIZE 100


typedef struct 
{
	enum state{
        STATE_START_MODE,
		STATE_FREEDOM_MODE,
		STATE_SPORT_MODE,
		STATE_SPORT_MODE_WARM1,
		STATE_SPORT_MODE_WARM2,
		STATE_SPORT_MODE_WARM3,
		STATE_SPORT_MODE_LEVEL,
		STATE_SPORT_MODE_CLIMB,
		STATE_LESSON_MODE,
		STATE_LESSON_MODE_WARM1,
		STATE_LESSON_MODE_WARM2,
		STATE_LESSON_MODE_WARM3,
		STATE_LESSON_MODE_LEVEL,
		STATE_LESSON_MODE_CLIMB,
        STATE_OUTDOOR_MODE,
		STATE_USER_REMENBER_DATA,
		STATE_USER_CHECK,
		STATE_CALIBRATION,
	};
	uint8_t outer_status;
	uint8_t	inner_status;
}BICYCLE;

enum BACK_CHECK
{
	NONE,
	CMD,
	DATA,
	STATE_DATA_RIGHT,
	STATE_SHOUDER_ERR,
	STATE_SHOUDER_OK,
	STATE_BACK_ERR,
	STATE_BACK_OK,
	STATE_CLIMB_ERR,
	STATE_WARM1_OK,
	STATE_WARM1_ERR,
	STATE_WARM2_RIGHT,
	STATE_WARM2_LEFT,
	STATE_WARM3_OK,
};

void user_remember(void);
void send_bicycle(uint16_t addr, uint8_t len,uint8_t *data) ;
void send_cmd(uint8_t cmd);
void send_check(uint8_t cmd,uint8_t state);
void send_data(MPU *mMPU,uint8_t cmd, uint8_t state);
void send_euler(void);

#endif /* __ATTITUDEREDRESS_H*/
