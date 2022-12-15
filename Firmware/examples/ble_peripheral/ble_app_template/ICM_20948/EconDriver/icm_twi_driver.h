#ifndef ICM_TWI_DRIVER_H
#define ICM_TWI_DRIVER_H


#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"

#include "main.h"

#define ICM_ADDRESS_H				0x68
#define ICM_ADDRESS_L				0x69

void Nordic_TWI_init(void);
uint8_t	icm_20948_Write_Byte(uint8_t reg, uint8_t value);
uint8_t icm_20948_ReadOne_Byte(uint8_t reg);

int	icm_20948_inv_write_hook(uint8_t reg, uint8_t len, uint8_t *Bufp);
int icm_20948_inv_read_hook(uint8_t reg, uint8_t len, uint8_t *Bufp);





#endif