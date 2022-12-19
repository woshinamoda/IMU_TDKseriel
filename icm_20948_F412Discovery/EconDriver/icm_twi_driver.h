#ifndef ICM_TWI_DRIVER_H
#define ICM_TWI_DRIVER_H


#include "i2c.h"
#include "main.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include "stdbool.h"



/* define ICM-20948 Device I2C address*/
	#define ICM_ADDRESS_H									 0xD0		//ad0拉低电平的IIC地址  0x68
	#define ICM_ADDRESS_L									 0xD2
	
	#define ICM_AK09916_ADD								 0x0C
	#define	ICM_AK09916_READ							 0x80
	#define	ICM_AK09916_WRITE							 0x00
	
	
/* define ICM-20948 Register */
/* user bank 0 register */
	#define REG_WHO_AM_I             			 0x00
	#define REG_VAL_WIA           		 		 0xEA		//id返回值
	
	#define USER_CTRL											 0x03		//P36.设置读取状态
	#define REG_VAL_BIT_DMP_EN        	   0x80		/*bit7：使能DMP   */	
	#define REG_VAL_BIT_FIFO_EN        	   0x40		/*bit6：使能FIFO   */
	#define REG_VAL_BIT_I2C_MST_EN				 0x20		/*bit5：使能IIC主机模块，引脚ES_DA和ES_SCL与SDA/SCLK隔离   */
	#define REG_VAL_BIT_I2C_IF_DIS				 0x10		/*bit4：使重置IIC，串口接到SPI模式   */
	#define REG_VAL_BIT_DMP_RST        	   0x08		/*bit3：重置DMP   */
	#define REG_VAL_BIT_DIAMOND_DMP_RST 	 0x04		/*bit2：重置sram    */
	
	#define PWR_MEMT_1										 0x06	  //P37.电源管理
	#define PWR_MEMT_2										 0x07		//陀螺仪 加速度电源使能 默认0x00，上电状态
	#define INT_PIN_CFG										 0x0F		//P38. 中断引脚功能设置
	#define	INT_ANYRD_2CLEAR							 0x10		/*bit4：0：读取状态才清除		1：任何时间清除status*/
	#define	INT1_LATCH_EN									 0x20		/*bit5：0：中断50us					1：中断电平变化*/
	#define	INT1_OPEN											 0x40		/*bit6：0：推挽							1：开漏*/
	#define INT1_ACTL										   0x80		/*bit7：0：逻辑电平高电平		1：逻辑电平低电平*/
	
	
	#define	INT_ENABLE										 0x10		//P38. 中断引脚使能0
	#define	INT_ENABLE_1									 0x11
	#define	RAW_DATA_0_RDY_EN							 0x01		/*bit0  1：使能原始数据准备好，就进入中断*/
	#define	INT_ENABLE_2									 0x12
	#define	INT_ENABLE_3									 0x13




	//reg bank add
	#define	REG_BANK_SEL									 0x7F		//P76.select bank	
	#define BANK_set_0  									 0x00
	#define BANK_set_1  									 0x10
	#define BANK_set_2  									 0x20
	#define BANK_set_3  									 0x30

	//read data for acc reg
	#define ACCEL_XOUT_H									 0x2D
	#define ACCEL_XOUT_L									 0x2E
	#define ACCEL_YOUT_H									 0x2F
	#define ACCEL_YOUT_L									 0x30
	#define ACCEL_ZOUT_H									 0x31
	#define ACCEL_ZOUT_L									 0x32
	
	//read data for gry reg
	#define GYRO_XOUT_H								  	 0x33
	#define GYRO_XOUT_L										 0x34
	#define GYRO_YOUT_H										 0x35
	#define GYRO_YOUT_L										 0x36
	#define GYRO_ZOUT_H										 0x37
	#define GYRO_ZOUT_L										 0x38
	
	//read tempture data
	#define TEMP_OUT_H								  	 0x39
	#define TEMP_OUT_L										 0x3A
	
	//read ext slv sens data
	#define EXT_SLV_SENS_DATA_00					 0x3B
	#define	INT_STATUS_1									 0x1A
	
	
	
/* user bank 1 register */

/* user bank 2 register */
	//gyro set config
	#define GYRO_SMPLRT_DIV							   0x00		//P59.陀螺仪采样率分频	设置GYRO_SMPLRT_DIV[7:0]
	#define GYRO_CONFIG_1									 0x01		//P59.陀螺仪参数设置 滤波、精度、使能
	#define GYRO_CONFIG_2									 0x02		//P60.陀螺仪自测与带宽		
	#define XG_OFFS_USRH									 0x03		
	#define XG_OFFS_USRL									 0x04	
	#define YG_OFFS_USRH									 0x05		
	#define YG_OFFS_USRL									 0x06	
	#define ZG_OFFS_USRH									 0x07		
	#define ZG_OFFS_USRL									 0x08		//各个轴的自校准偏移	
	#define REG_VAL_BIT_GYRO_DLPCFG_2  		 0x10 /* bit[5:3] */
	#define REG_VAL_BIT_GYRO_DLPCFG_4  		 0x20 /* bit[5:3] */
	#define REG_VAL_BIT_GYRO_DLPCFG_6  		 0x30 /* bit[5:3] */		//设置参考手册table.15
	#define REG_VAL_BIT_GYRO_FS_250DPS 		 0x00 /* bit[2:1] */
	#define REG_VAL_BIT_GYRO_FS_500DPS 		 0x02 /* bit[2:1] */
	#define REG_VAL_BIT_GYRO_FS_1000DPS		 0x04 /* bit[2:1] */
	#define REG_VAL_BIT_GYRO_FS_2000DPS		 0x06 /* bit[2:1] */    
	#define REG_VAL_BIT_GYRO_DLPF      		 0x01 /* bit[0]   */		//使能GYRO_FCHOICE = 1

	//accel set config
	#define	ACCEL_SMPLRT_DIV_1						 0x10		
	#define	ACCEL_SMPLRT_DIV_2						 0x11		//P63.采样率	
	#define ACCEL_CONFIG									 0x13		//P64.加速度设置
	#define REG_VAL_BIT_ACCEL_DLPCFG_2 		 0x10 /* bit[5:3] */
	#define REG_VAL_BIT_ACCEL_DLPCFG_4 		 0x20 /* bit[5:3] */
	#define REG_VAL_BIT_ACCEL_DLPCFG_6 		 0x30 /* bit[5:3] */
	#define REG_VAL_BIT_ACCEL_FS_2g    		 0x00/* bit[5:3] */
	#define REG_VAL_BIT_ACCEL_FS_4g    		 0x02 /* bit[2:1] */
	#define REG_VAL_BIT_ACCEL_FS_8g   		 0x04 /* bit[2:1] */
	#define REG_VAL_BIT_ACCEL_FS_16g   		 0x06 /* bit[2:1] */    
	#define REG_VAL_BIT_ACCEL_DLPF     		 0x01 /* bit[0]   */	
	
	#define	FSYNC_CONFIG									 0x52		//P66.FSYNC设置
	#define TEMP_CONFIG										 0x53		//温度计的低通设置
	#define	MOD_CTRL_USR									 0x54		//使能DMP运动单元


/* user bank 3 register */

	#define	I2C_MST_ODR_CONFIG						 0x00		//P68.MST主机时钟设置
	#define	I2C_MST_CTRL						 			 0x01		
	#define	I2C_MST_DELAY_CTRL						 0x02		//p69. 从属设备延时到接收数据时间设置
	#define	I2C_SLV0_ADDR									 0x03		
	#define	I2C_SLV0_REG									 0x04	
	#define I2C_SLV0_CTRL									 0x05		

	#define I2C_SLV1_ADDR									 0x07
	#define I2C_SLV1_REG									 0x08
	#define I2C_SLV1_CTRL									 0x09
	#define I2C_SLV1_DO										 0x0A
	#define REG_VAL_BIT_SLV0_EN						 0x80/* bit[7] */			//1：允许从机读取  0：禁用从属服务函数
	#define REG_VAL_BIT_MASK_LEN					 0x07/* bit[3:0] */		//Number of bytes to be read from I2C slave 1.

/* define ICM-20948 Register  end */

/* define ICM-20948 MAG Register  */

	#define MAG_WIA2   										 0x01
	#define MAG_ST1  			  						   0x10		//p78. bit0:DRDY		bit1:DOR
	#define	MAG_DATA											 0x11		//P78. 11H - 16H都是磁力计数据
	#define	MAG_CNLT2											 0x31
	#define REG_VAL_MAG_MODE_PD  			     0x00
	#define REG_VAL_MAG_MODE_SM  			     0x01
	#define REG_VAL_MAG_MODE_10HZ  			   0x02
	#define REG_VAL_MAG_MODE_20HZ  			   0x04
	#define REG_VAL_MAG_MODE_50HZ  				 0x05
	#define REG_VAL_MAG_MODE_100HZ 				 0x08
	#define REG_VAL_MAG_MODE_ST    				 0x10		//P79. self-test mode
/* define ICM-20948 MAG Register  end */



typedef struct	icm20948_data_tag
{
	int16_t		D_gry[3];
	int16_t		D_acc[3];
	int16_t		D_mag[3];

}Sensor_DataTypedef;









void Nordic_TWI_init(void);
uint8_t	icm_20948_Write_Byte(uint8_t reg, uint8_t value);
uint8_t icm_20948_ReadOne_Byte(uint8_t reg);



void icm_20948_init();
bool icm_20948_Check();
void icm_20948GyroRead();
void icm_20948AccelRead();
void ak_09916MagRead();
void icm_20948WriteSecondary(uint8_t IICAddr, uint8_t RegAddr, uint8_t data);
void icm20948ReadSecondary(uint8_t IICAddr, uint8_t RegAddr, uint8_t Len, uint8_t *data);
bool ak_09916MagCheck();
	






#endif