#include "icm_twi_driver.h"
#include "nrf_drv_twi.h"

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

//TWI��ʼ��
void Nordic_TWI_init(void)
{
    ret_code_t err_code;
	    //���岢��ʼ��TWI���ýṹ��
    const nrf_drv_twi_config_t twi_config = {
		   .scl                = TWI_SCL_M,  //����TWI SCL����
       .sda                = TWI_SDA_M,  //����TWI SDA����
       .frequency          = NRF_DRV_TWI_FREQ_400K, //TWI����
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //TWI���ȼ�
       .clear_bus_init     = false//��ʼ���ڼ䲻����9��SCLʱ��
    };
    //��ʼ��TWI
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
		//��鷵�صĴ������
    APP_ERROR_CHECK(err_code);
    //ʹ��TWI
    nrf_drv_twi_enable(&m_twi);
}

uint8_t	icm_20948_Write_Byte(uint8_t reg, uint8_t value)
{
	  ret_code_t err_code;
	  uint8_t tx_buf[2];	
	  tx_buf[0] = reg;
    tx_buf[1] = value;
		//TWI�����־��false
		m_xfer_done = false;	
		//д������
    err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, 2, false);
		//�ȴ�TWI���ߴ������
		while (m_xfer_done == false){}	
}

uint8_t icm_20948_ReadOne_Byte(uint8_t reg)
{
	
	  ret_code_t err_code;
	
		uint8_t	get_buf[1];
		uint8_t	tx_buf[1];	
		tx_buf[0] = reg;
		m_xfer_done = false;	
	  err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, 1, true);
		while (m_xfer_done == false){}	
			
		m_xfer_done = false;	
	  err_code = nrf_drv_twi_rx(&m_twi, ICM_ADDRESS_H, get_buf, 1);
		while (m_xfer_done == false){}	
		return get_buf[0];
}

//����invn�˶����㴫�ݺ���
//9�ᴫ������̶�reg������������
int	icm_20948_inv_write_hook(uint8_t reg, uint8_t len, uint8_t *Bufp)
{
	ret_code_t err_code;
	int result = 0;
	uint8_t	tx_buf[len + 1];
	
	//׼��д�������
	tx_buf[0] = reg;
	for(uint8_t i=1; i<len+1; i++)
	{
		tx_buf[i] = Bufp[i-1];
	}
	
	//TWI�����־��false
	m_xfer_done = false;
	//д������
	err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, len+1, false);
	//�ȴ�TWI���ߴ������
	while (m_xfer_done == false){}	
	return result;	
}


//9�ᴫ������ȡ�̶�reg����������
int icm_20948_inv_read_hook(uint8_t reg, uint8_t len, uint8_t *Bufp)
{
	ret_code_t err_code;
	int result = 0;	
	uint8_t	tx_buf[1];

	tx_buf[0] = reg;
	//TWI������ɱ�־����Ϊfalse
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, 1, true);
	//�ȴ�TWI���ߴ������	
	while (m_xfer_done == false){}	
	
		
	//TWI������ɱ�־����Ϊfalse
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, ICM_ADDRESS_H, Bufp, len);
	//�ȴ�TWI���ߴ������
	while (m_xfer_done == false){}		

	return result;	
}





//int inv_serial_interface_read_hook(uint16_t reg, uint32_t length, uint8_t *data)




















