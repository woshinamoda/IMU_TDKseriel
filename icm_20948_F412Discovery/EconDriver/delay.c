#include "delay.h"

#include <stdbool.h>
#include <stdint.h>
#include "tim.h"

void delay_us(uint32_t nus)
{
	TIM7->CNT = 0;
	HAL_TIM_Base_Start(&htim7);
	while(TIM7->CNT < nus){}
	HAL_TIM_Base_Stop(&htim7);
}


void delay_ms(uint16_t nms)
{
		uint16_t time;
		time = nms*2;
		for(uint16_t i=0;i<time;i++)
		{
			delay_us(500);
		}
}


//static long long g_ul_ms_ticks = 0;
//int get_tick_count(long long *count)
//{
//        count[0] = g_ul_ms_ticks;
//	return 0;
//}






