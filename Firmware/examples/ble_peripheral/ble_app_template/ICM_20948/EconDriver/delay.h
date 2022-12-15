#ifndef __DELAY_H
#define __DELAY_H 			   

#include "main.h"


void delay_ns(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
int get_tick_count(long long *count);


#endif

