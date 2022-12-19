#include "delay.h"

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_delay.h"

void delay_us(uint32_t nus)
{
	nrf_delay_us(nus);
}


void delay_ms(uint16_t nms)
{
	nrf_delay_ms(nms);
}


static long long g_ul_ms_ticks = 0;
int get_tick_count(long long *count)
{
        count[0] = g_ul_ms_ticks;
	return 0;
}






