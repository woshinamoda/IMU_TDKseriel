#include "icm_twi_driver.h"
#include "nrf_drv_twi.h"

extern Sensor_DataTypedef	IMU_Data;

//TWI驱动程序实例ID，ID和外设编号对应，0:TWI0		1:TWI1
#define		TWI_INSTANCE_ID		0

//定义TWI传输完成标志
static volatile bool	m_xfer_done = false;
//定义TWI驱动实例，名称m_twi
static	const	nrf_drv_twi_t	m_twi=	NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//TWI事件处理函数
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
  //设置twi传输完成  
	m_xfer_done = true;
}

/* IIC drive icm_20948 start*/
void Nordic_TWI_init(void)
{//TWI初始化
    ret_code_t err_code;
	    //定义并初始化TWI配置结构体
    const nrf_drv_twi_config_t twi_config = {
		   .scl                = TWI_SCL_M,  //定义TWI SCL引脚
       .sda                = TWI_SDA_M,  //定义TWI SDA引脚
       .frequency          = TWI_FREQUENCY_FREQUENCY_K100, //TWI速率
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //TWI优先级
       .clear_bus_init     = false//初始化期间不发送9个SCL时钟
    };
    //初始化TWI
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
		//检查返回的错误代码
    APP_ERROR_CHECK(err_code);
    //使能TWI
    nrf_drv_twi_enable(&m_twi);
}

uint8_t	icm_20948_Write_Byte(uint8_t reg, uint8_t value)
{//icm_20948对应寄存器写入数据
	  ret_code_t err_code;
	  uint8_t tx_buf[2];	
	  tx_buf[0] = reg;
    tx_buf[1] = value;
		//写入数据
    err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, 2, false);


	
}

uint8_t icm_20948_ReadOne_Byte(uint8_t reg)
{//icm_20948对应寄存器读出1个字节数据
	
	  ret_code_t err_code;
	
		uint8_t	get_buf[1];
		uint8_t	tx_buf[1];	
		tx_buf[0] = reg;
	  err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, 1, true);


			
	  err_code = nrf_drv_twi_rx(&m_twi, ICM_ADDRESS_H, get_buf, 1);

		return get_buf[0];
}

uint8_t icm_AK09916_Write_Byte(uint8_t DevAddr, uint8_t reg, uint8_t value)
{//罗盘仪内写入1个字节数据
	  ret_code_t err_code;
	  uint8_t tx_buf[2];	
	  tx_buf[0] = reg;
    tx_buf[1] = value;
		//写入数据
    err_code = nrf_drv_twi_tx(&m_twi, DevAddr, tx_buf, 2, false);

}

uint8_t icm_AK09916_ReadOne_Byte(uint8_t DevAddr, uint8_t reg)
{//罗盘仪内对应寄存器读取1个字节数据
	  ret_code_t err_code;
		uint8_t	get_buf[1];
		uint8_t	tx_buf[1];	
		tx_buf[0] = reg;
	  err_code = nrf_drv_twi_tx(&m_twi, DevAddr, tx_buf, 1, true);

	
	  err_code = nrf_drv_twi_rx(&m_twi, DevAddr, get_buf, 1);

		return get_buf[0];
}

/* IIC drive icm_20948 end*/



//icm_20948设置初始化
void icm_20948_init()
{
	bool init_OK = false;
	
	init_OK = icm_20948_Check();																//姿态仪自检
	while(init_OK == false)	{}
	

	/*user bank 0 register*/
	icm_20948_Write_Byte(REG_BANK_SEL, BANK_set_0);
	icm_20948_Write_Byte(PWR_MEMT_1,0x80);						//软复位，重置电源
	nrf_delay_ms(10);	
	icm_20948_Write_Byte(PWR_MEMT_1,0x01);						//开启温度传感器，自适应时钟，低功耗运行
	nrf_delay_ms(10);
	icm_20948_Write_Byte(INT_PIN_CFG,0x10);						
	nrf_delay_ms(10);	
	icm_20948_Write_Byte(INT_ENABLE_1,RAW_DATA_0_RDY_EN);		//开启INT1，数据准备中断（罗盘仪没有中断，只能开启一个对应频率的定时器实时读取）
	
	/*user bank 2 register*/
	icm_20948_Write_Byte(REG_BANK_SEL, BANK_set_2);	
	icm_20948_Write_Byte(GYRO_SMPLRT_DIV,0x07);				//陀螺仪采样率时钟分频（采样率）  分频系数0-7有效	ODR is follow 1.1Khz/(1+ODRseriel) = 140.625Hz
	icm_20948_Write_Byte(GYRO_CONFIG_1,								//设置详看宏定义
                  REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF);
	
	icm_20948_Write_Byte(ACCEL_SMPLRT_DIV_2,0x07);		//设置加速度时钟分频（采样率）  分频系数0-7有效	ODR is follow 1.1Khz/(1+ODRseriel) = 140.625Hz
	icm_20948_Write_Byte(ACCEL_CONFIG,								//设置详看宏定义
                  REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);
									

	/*user bank 0 register*/
	icm_20948_Write_Byte(REG_BANK_SEL, BANK_set_0);
	nrf_delay_ms(100);

	init_OK = false;
	init_OK = ak_09916MagCheck();																//罗盘仪自检
	while(init_OK == false)	{}

	icm_20948WriteSecondary(ICM_AK09916_ADD|ICM_AK09916_WRITE,
													MAG_CNLT2,
													REG_VAL_MAG_MODE_100HZ);
	}

//who_am_i id判断函数
bool icm_20948_Check()
{
	bool bRet = false;
	if(REG_VAL_WIA == icm_20948_ReadOne_Byte(REG_WHO_AM_I))
	{
		bRet = true;
	}
	return bRet;
}

//gyro 角速度原始数据读取
void	icm_20948GyroRead()
{
	uint8_t u8Buf[6];
	int16_t	s16Buf[3] = {0};
	//get gry Xdata
	u8Buf[0] = icm_20948_ReadOne_Byte(GYRO_XOUT_L);
	u8Buf[1] = icm_20948_ReadOne_Byte(GYRO_XOUT_H);
  s16Buf[0]=  (u8Buf[1]<<8)|u8Buf[0];	
	//get gry Ydata
	u8Buf[0] = icm_20948_ReadOne_Byte(GYRO_YOUT_L);
	u8Buf[1] = icm_20948_ReadOne_Byte(GYRO_YOUT_H);
	s16Buf[1]=  (u8Buf[1]<<8)|u8Buf[0];
	//get gry Zdata
	u8Buf[0] = icm_20948_ReadOne_Byte(GYRO_ZOUT_L);
	u8Buf[1] = icm_20948_ReadOne_Byte(GYRO_ZOUT_H);	
  s16Buf[2]=  (u8Buf[1]<<8)|u8Buf[0];	
	
	IMU_Data.D_gry[0] = s16Buf[0];
	IMU_Data.D_gry[1] = s16Buf[1];
	IMU_Data.D_gry[2] = s16Buf[2];		
}


//accel 加速度原始数据读取
void	icm_20948AccelRead()
{
	uint8_t u8Buf[6];
	int16_t	s16Buf[3] = {0};
	//get acc Xdata
	u8Buf[0] = icm_20948_ReadOne_Byte(ACCEL_XOUT_L);
	u8Buf[1] = icm_20948_ReadOne_Byte(ACCEL_XOUT_H);
  s16Buf[0]=  (u8Buf[1]<<8)|u8Buf[0];	
	//get acc Ydata
	u8Buf[0] = icm_20948_ReadOne_Byte(ACCEL_YOUT_L);
	u8Buf[1] = icm_20948_ReadOne_Byte(ACCEL_YOUT_H);
	s16Buf[1]=  (u8Buf[1]<<8)|u8Buf[0];
	//get acc Zdata
	u8Buf[0] = icm_20948_ReadOne_Byte(ACCEL_ZOUT_L);
	u8Buf[1] = icm_20948_ReadOne_Byte(ACCEL_ZOUT_H);	
  s16Buf[2]=  (u8Buf[1]<<8)|u8Buf[0];	
	
	IMU_Data.D_acc[0] = s16Buf[0];
	IMU_Data.D_acc[1] = s16Buf[1];
	IMU_Data.D_acc[2] = s16Buf[2];	
}

#define MAG_DATA_LEN    6				//罗盘仪输出长度6byte 3个方向
//mag 地磁仪原始数据度
void  ak_09916MagRead()
{
	uint8_t	counter = 20;
	uint8_t u8Data[MAG_DATA_LEN];
	int16_t s16Buf[3] = {0};
	uint8_t	i;
	int32_t s32OutBuf[3] = {0};
	
	while(counter>0)
	{
		nrf_delay_ms(10);
		icm20948ReadSecondary(ICM_AK09916_ADD|ICM_AK09916_READ,
													MAG_ST1, 1, u8Data);		//读取罗盘仪ST1寄存器判断DRDY
		if((u8Data[0] & 0x01) !=0 )	//“1”: Data is ready
			break;
		
		counter--;	
	}
	if(counter != 0)
	{	//如果10次自检内成功
		icm20948ReadSecondary(ICM_AK09916_ADD|ICM_AK09916_READ,
													MAG_DATA,
													MAG_DATA_LEN,
													u8Data);
		s16Buf[0] = ((int16_t)u8Data[1]<<8) | u8Data[0];
		s16Buf[1] = ((int16_t)u8Data[3]<<8) | u8Data[2];
		s16Buf[2] = ((int16_t)u8Data[5]<<8) | u8Data[4];
	}
	
	//P83图，y z轴与加速度/脱落仪一致的话，需要取反
	IMU_Data.D_mag[0] = s16Buf[0];
	IMU_Data.D_mag[1]  = -s16Buf[1];
	IMU_Data.D_mag[2]  = -s16Buf[2];	
}

/**********罗盘仪写入数据****************
* IICAddr	:	磁力计IIC的地址（读or写）
* RegAddr ：磁力计寄存器
*	data		：对应寄存器写入值
******************************************/
void icm_20948WriteSecondary(uint8_t IICAddr, uint8_t RegAddr, uint8_t data)
{
	uint8_t u8Temp;
	/*bank 3 register*/		
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_3);			
	icm_20948_Write_Byte(I2C_SLV1_ADDR,	IICAddr);				//从属设备1的物理地址
	icm_20948_Write_Byte(I2C_SLV1_REG,	RegAddr);				//从属设备1的寄存器值
	icm_20948_Write_Byte(I2C_SLV1_DO,	data);						//从属设备1的寄存器设置data值
	icm_20948_Write_Byte(I2C_SLV1_CTRL,	REG_VAL_BIT_SLV0_EN|1);		//使能写入1个字节			
	/*bank 0 register*/	
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_0);			
	u8Temp = icm_20948_ReadOne_Byte(USER_CTRL);					//读取USR_CTRL状态
	u8Temp |= REG_VAL_BIT_I2C_MST_EN;
	icm_20948_Write_Byte(USER_CTRL, u8Temp);
	nrf_delay_ms(5);
	u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
	icm_20948_Write_Byte(USER_CTRL, u8Temp);						//发送完成后在关闭
	/*bank 3 register*/	
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_3);				
	u8Temp = icm_20948_ReadOne_Byte(I2C_SLV0_CTRL);			//读取slv0状态
	u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));		//bit5 bit[0-3]置0 I2C_SLV恢复读写状态，下次读取数据长度置0
	icm_20948_Write_Byte(I2C_SLV0_CTRL, u8Temp);
	/*bank 0 register*/	
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_0);				
}

/**********罗盘仪内读取数据****************
* IICAddr	:	磁力计IIC的地址（读or写）
* RegAddr ：磁力计寄存器
*	Len			：读取数据长度
*	data		：读取指针缓存地址
******************************************/
void icm20948ReadSecondary(uint8_t IICAddr, uint8_t RegAddr, uint8_t Len, uint8_t *data)
{
	uint8_t i;
	uint8_t u8Temp;
	/*bank 3 register*/		
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_3);	
	icm_20948_Write_Byte(I2C_SLV0_ADDR,	IICAddr);				//从属设备0的物理地址
	icm_20948_Write_Byte(I2C_SLV0_REG,	RegAddr);				//从属设备0的寄存器值
	icm_20948_Write_Byte(I2C_SLV0_CTRL,	REG_VAL_BIT_SLV0_EN|Len);		//bit7置1：使能通过从属设备0读取数据  bit[3:0] = |LEN，读取数据长度
	/*bank 0 register*/		
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_0);	
	u8Temp = icm_20948_ReadOne_Byte(USER_CTRL);
	u8Temp |= REG_VAL_BIT_I2C_MST_EN;
	icm_20948_Write_Byte(USER_CTRL,u8Temp);
	nrf_delay_ms(5);
	u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;	
	icm_20948_Write_Byte(USER_CTRL,u8Temp);
	
	for(i=0; i<Len; i++)
	{
		*(data+i) = icm_20948_ReadOne_Byte(EXT_SLV_SENS_DATA_00+i);	//读取传感器数据
	}	
	/*bank 3 register*/		
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_3);
	u8Temp = icm_20948_ReadOne_Byte(I2C_SLV0_CTRL);					//依旧读取从属设备0状态
	u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));		//bit5 bit[0-3]置0 I2C_SLV恢复读写状态，下次读取数据长度置0
	icm_20948_Write_Byte(I2C_SLV0_CTRL, u8Temp);
	/*bank 0 register*/	
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_0);	
}

//罗盘仪芯片ID自检
bool ak_09916MagCheck()
{
    bool bRet = false;
    uint8_t u8Ret[2];
    icm20948ReadSecondary(ICM_AK09916_ADD|ICM_AK09916_READ,
													MAG_WIA2,
													1,
													u8Ret);
	if(u8Ret[0] == 0x09)	//P78->13.1
	{
		bRet = true;	
	}
	
	return bRet;
}





























