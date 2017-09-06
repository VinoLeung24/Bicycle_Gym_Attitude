#ifndef __TIME_H__
#define __TIME_H__

#include "sys.h"

#define true 1
#define false 0

void SYSTICK_INIT(void);
void TIM_INIT(void);
float GET_NOWTIME(void);
void get_ms(unsigned long *time);


#endif

