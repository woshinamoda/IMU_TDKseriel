#include <stdio.h>
#include <string.h>

#include "inv_mems_hw_config.h"
#include "invn_types.h"

#include "delay.h"
#include "icm20948.h"
#include "icm_twi_driver.h"
#include "main.h"


int inv_serial_interface_write_hook(uint16_t reg, uint32_t length, uint8_t *data)
{
    unsigned char rx;
    int result = 0;
		result = icm_20948_inv_write_hook(reg,length,data);
    return result;
}

int inv_serial_interface_read_hook(uint16_t reg, uint32_t length, uint8_t *data)
{
    unsigned char rx;
    int result = 0;
		result = icm_20948_inv_read_hook(reg,length,data);
    return result;
}
//外部中断服务程序
void EXTI_INT_IRQHandler(void)
{

	
}

/**
 *  @brief  Sleep function.
**/
void inv_sleep(unsigned long mSecs)
{
    delay_ms(mSecs);
}

void inv_sleep_100us(unsigned long nHowMany100MicroSecondsToSleep)
{
    delay_us(100 * nHowMany100MicroSecondsToSleep);
}

/**
 *  @brief  获取系统tick，用做时间参考
 *  @return 返回tick值
**/
long long inv_get_tick_count(void)
{
    long long count;

    get_tick_count(&count);

    return (long long)count;
}

