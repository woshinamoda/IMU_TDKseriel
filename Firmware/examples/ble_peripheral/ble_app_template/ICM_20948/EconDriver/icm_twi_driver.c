#include "icm_twi_driver.h"
#include "nrf_drv_twi.h"

extern Sensor_DataTypedef	IMU_Data;

//TWI��������ʵ��ID��ID�������Ŷ�Ӧ��0:TWI0		1:TWI1
#define		TWI_INSTANCE_ID		0

//����TWI������ɱ�־
static volatile bool	m_xfer_done = false;
//����TWI����ʵ��������m_twi
static	const	nrf_drv_twi_t	m_twi=	NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//TWI�¼�������
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
  //����twi�������  
	m_xfer_done = true;
}

/* IIC drive icm_20948 start*/
void Nordic_TWI_init(void)
{//TWI��ʼ��
    ret_code_t err_code;
	    //���岢��ʼ��TWI���ýṹ��
    const nrf_drv_twi_config_t twi_config = {
		   .scl                = TWI_SCL_M,  //����TWI SCL����
       .sda                = TWI_SDA_M,  //����TWI SDA����
       .frequency          = TWI_FREQUENCY_FREQUENCY_K100, //TWI����
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //TWI���ȼ�
       .clear_bus_init     = false//��ʼ���ڼ䲻����9��SCLʱ��
    };
    //��ʼ��TWI
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
		//��鷵�صĴ������
    APP_ERROR_CHECK(err_code);
    //ʹ��TWI
    nrf_drv_twi_enable(&m_twi);
}

uint8_t	icm_20948_Write_Byte(uint8_t reg, uint8_t value)
{//icm_20948��Ӧ�Ĵ���д������
	  ret_code_t err_code;
	  uint8_t tx_buf[2];	
	  tx_buf[0] = reg;
    tx_buf[1] = value;
		//д������
    err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, 2, false);


	
}

uint8_t icm_20948_ReadOne_Byte(uint8_t reg)
{//icm_20948��Ӧ�Ĵ�������1���ֽ�����
	
	  ret_code_t err_code;
	
		uint8_t	get_buf[1];
		uint8_t	tx_buf[1];	
		tx_buf[0] = reg;
	  err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, 1, true);


			
	  err_code = nrf_drv_twi_rx(&m_twi, ICM_ADDRESS_H, get_buf, 1);

		return get_buf[0];
}

uint8_t icm_AK09916_Write_Byte(uint8_t DevAddr, uint8_t reg, uint8_t value)
{//��������д��1���ֽ�����
	  ret_code_t err_code;
	  uint8_t tx_buf[2];	
	  tx_buf[0] = reg;
    tx_buf[1] = value;
		//д������
    err_code = nrf_drv_twi_tx(&m_twi, DevAddr, tx_buf, 2, false);

}

uint8_t icm_AK09916_ReadOne_Byte(uint8_t DevAddr, uint8_t reg)
{//�������ڶ�Ӧ�Ĵ�����ȡ1���ֽ�����
	  ret_code_t err_code;
		uint8_t	get_buf[1];
		uint8_t	tx_buf[1];	
		tx_buf[0] = reg;
	  err_code = nrf_drv_twi_tx(&m_twi, DevAddr, tx_buf, 1, true);

	
	  err_code = nrf_drv_twi_rx(&m_twi, DevAddr, get_buf, 1);

		return get_buf[0];
}

/* IIC drive icm_20948 end*/



//icm_20948���ó�ʼ��
void icm_20948_init()
{
	bool init_OK = false;
	
	init_OK = icm_20948_Check();																//��̬���Լ�
	while(init_OK == false)	{}
	

	/*user bank 0 register*/
	icm_20948_Write_Byte(REG_BANK_SEL, BANK_set_0);
	icm_20948_Write_Byte(PWR_MEMT_1,0x80);						//��λ�����õ�Դ
	nrf_delay_ms(10);	
	icm_20948_Write_Byte(PWR_MEMT_1,0x01);						//�����¶ȴ�����������Ӧʱ�ӣ��͹�������
	nrf_delay_ms(10);
	icm_20948_Write_Byte(INT_PIN_CFG,0x10);						
	nrf_delay_ms(10);	
	icm_20948_Write_Byte(INT_ENABLE_1,RAW_DATA_0_RDY_EN);		//����INT1������׼���жϣ�������û���жϣ�ֻ�ܿ���һ����ӦƵ�ʵĶ�ʱ��ʵʱ��ȡ��
	
	/*user bank 2 register*/
	icm_20948_Write_Byte(REG_BANK_SEL, BANK_set_2);	
	icm_20948_Write_Byte(GYRO_SMPLRT_DIV,0x07);				//�����ǲ�����ʱ�ӷ�Ƶ�������ʣ�  ��Ƶϵ��0-7��Ч	ODR is follow 1.1Khz/(1+ODRseriel) = 140.625Hz
	icm_20948_Write_Byte(GYRO_CONFIG_1,								//�����꿴�궨��
                  REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF);
	
	icm_20948_Write_Byte(ACCEL_SMPLRT_DIV_2,0x07);		//���ü��ٶ�ʱ�ӷ�Ƶ�������ʣ�  ��Ƶϵ��0-7��Ч	ODR is follow 1.1Khz/(1+ODRseriel) = 140.625Hz
	icm_20948_Write_Byte(ACCEL_CONFIG,								//�����꿴�궨��
                  REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);
									

	/*user bank 0 register*/
	icm_20948_Write_Byte(REG_BANK_SEL, BANK_set_0);
	nrf_delay_ms(100);

	init_OK = false;
	init_OK = ak_09916MagCheck();																//�������Լ�
	while(init_OK == false)	{}

	icm_20948WriteSecondary(ICM_AK09916_ADD|ICM_AK09916_WRITE,
													MAG_CNLT2,
													REG_VAL_MAG_MODE_100HZ);
	}

//who_am_i id�жϺ���
bool icm_20948_Check()
{
	bool bRet = false;
	if(REG_VAL_WIA == icm_20948_ReadOne_Byte(REG_WHO_AM_I))
	{
		bRet = true;
	}
	return bRet;
}

//gyro ���ٶ�ԭʼ���ݶ�ȡ
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


//accel ���ٶ�ԭʼ���ݶ�ȡ
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

#define MAG_DATA_LEN    6				//�������������6byte 3������
//mag �ش���ԭʼ���ݶ�
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
													MAG_ST1, 1, u8Data);		//��ȡ������ST1�Ĵ����ж�DRDY
		if((u8Data[0] & 0x01) !=0 )	//��1��: Data is ready
			break;
		
		counter--;	
	}
	if(counter != 0)
	{	//���10���Լ��ڳɹ�
		icm20948ReadSecondary(ICM_AK09916_ADD|ICM_AK09916_READ,
													MAG_DATA,
													MAG_DATA_LEN,
													u8Data);
		s16Buf[0] = ((int16_t)u8Data[1]<<8) | u8Data[0];
		s16Buf[1] = ((int16_t)u8Data[3]<<8) | u8Data[2];
		s16Buf[2] = ((int16_t)u8Data[5]<<8) | u8Data[4];
	}
	
	//P83ͼ��y z������ٶ�/������һ�µĻ�����Ҫȡ��
	IMU_Data.D_mag[0] = s16Buf[0];
	IMU_Data.D_mag[1]  = -s16Buf[1];
	IMU_Data.D_mag[2]  = -s16Buf[2];	
}

/**********������д������****************
* IICAddr	:	������IIC�ĵ�ַ����orд��
* RegAddr �������ƼĴ���
*	data		����Ӧ�Ĵ���д��ֵ
******************************************/
void icm_20948WriteSecondary(uint8_t IICAddr, uint8_t RegAddr, uint8_t data)
{
	uint8_t u8Temp;
	/*bank 3 register*/		
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_3);			
	icm_20948_Write_Byte(I2C_SLV1_ADDR,	IICAddr);				//�����豸1�������ַ
	icm_20948_Write_Byte(I2C_SLV1_REG,	RegAddr);				//�����豸1�ļĴ���ֵ
	icm_20948_Write_Byte(I2C_SLV1_DO,	data);						//�����豸1�ļĴ�������dataֵ
	icm_20948_Write_Byte(I2C_SLV1_CTRL,	REG_VAL_BIT_SLV0_EN|1);		//ʹ��д��1���ֽ�			
	/*bank 0 register*/	
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_0);			
	u8Temp = icm_20948_ReadOne_Byte(USER_CTRL);					//��ȡUSR_CTRL״̬
	u8Temp |= REG_VAL_BIT_I2C_MST_EN;
	icm_20948_Write_Byte(USER_CTRL, u8Temp);
	nrf_delay_ms(5);
	u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
	icm_20948_Write_Byte(USER_CTRL, u8Temp);						//������ɺ��ڹر�
	/*bank 3 register*/	
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_3);				
	u8Temp = icm_20948_ReadOne_Byte(I2C_SLV0_CTRL);			//��ȡslv0״̬
	u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));		//bit5 bit[0-3]��0 I2C_SLV�ָ���д״̬���´ζ�ȡ���ݳ�����0
	icm_20948_Write_Byte(I2C_SLV0_CTRL, u8Temp);
	/*bank 0 register*/	
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_0);				
}

/**********�������ڶ�ȡ����****************
* IICAddr	:	������IIC�ĵ�ַ����orд��
* RegAddr �������ƼĴ���
*	Len			����ȡ���ݳ���
*	data		����ȡָ�뻺���ַ
******************************************/
void icm20948ReadSecondary(uint8_t IICAddr, uint8_t RegAddr, uint8_t Len, uint8_t *data)
{
	uint8_t i;
	uint8_t u8Temp;
	/*bank 3 register*/		
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_3);	
	icm_20948_Write_Byte(I2C_SLV0_ADDR,	IICAddr);				//�����豸0�������ַ
	icm_20948_Write_Byte(I2C_SLV0_REG,	RegAddr);				//�����豸0�ļĴ���ֵ
	icm_20948_Write_Byte(I2C_SLV0_CTRL,	REG_VAL_BIT_SLV0_EN|Len);		//bit7��1��ʹ��ͨ�������豸0��ȡ����  bit[3:0] = |LEN����ȡ���ݳ���
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
		*(data+i) = icm_20948_ReadOne_Byte(EXT_SLV_SENS_DATA_00+i);	//��ȡ����������
	}	
	/*bank 3 register*/		
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_3);
	u8Temp = icm_20948_ReadOne_Byte(I2C_SLV0_CTRL);					//���ɶ�ȡ�����豸0״̬
	u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));		//bit5 bit[0-3]��0 I2C_SLV�ָ���д״̬���´ζ�ȡ���ݳ�����0
	icm_20948_Write_Byte(I2C_SLV0_CTRL, u8Temp);
	/*bank 0 register*/	
	icm_20948_Write_Byte(REG_BANK_SEL,BANK_set_0);	
}

//������оƬID�Լ�
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





























