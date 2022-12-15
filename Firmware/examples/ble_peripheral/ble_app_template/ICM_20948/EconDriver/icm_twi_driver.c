#include "icm_twi_driver.h"
#include "nrf_drv_twi.h"

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

//TWI初始化
void Nordic_TWI_init(void)
{
    ret_code_t err_code;
	    //定义并初始化TWI配置结构体
    const nrf_drv_twi_config_t twi_config = {
		   .scl                = TWI_SCL_M,  //定义TWI SCL引脚
       .sda                = TWI_SDA_M,  //定义TWI SDA引脚
       .frequency          = NRF_DRV_TWI_FREQ_400K, //TWI速率
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //TWI优先级
       .clear_bus_init     = false//初始化期间不发送9个SCL时钟
    };
    //初始化TWI
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
		//检查返回的错误代码
    APP_ERROR_CHECK(err_code);
    //使能TWI
    nrf_drv_twi_enable(&m_twi);
}

uint8_t	icm_20948_Write_Byte(uint8_t reg, uint8_t value)
{
	  ret_code_t err_code;
	  uint8_t tx_buf[2];	
	  tx_buf[0] = reg;
    tx_buf[1] = value;
		//TWI传输标志置false
		m_xfer_done = false;	
		//写入数据
    err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, 2, false);
		//等待TWI总线传输完成
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

//用于invn运动计算传递函数
//9轴传感器向固定reg发送数组数据
int	icm_20948_inv_write_hook(uint8_t reg, uint8_t len, uint8_t *Bufp)
{
	ret_code_t err_code;
	int result = 0;
	uint8_t	tx_buf[len + 1];
	
	//准备写入的数据
	tx_buf[0] = reg;
	for(uint8_t i=1; i<len+1; i++)
	{
		tx_buf[i] = Bufp[i-1];
	}
	
	//TWI传输标志置false
	m_xfer_done = false;
	//写入数据
	err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, len+1, false);
	//等待TWI总线传输完成
	while (m_xfer_done == false){}	
	return result;	
}


//9轴传感器读取固定reg内所有数据
int icm_20948_inv_read_hook(uint8_t reg, uint8_t len, uint8_t *Bufp)
{
	ret_code_t err_code;
	int result = 0;	
	uint8_t	tx_buf[1];

	tx_buf[0] = reg;
	//TWI传输完成标志设置为false
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, ICM_ADDRESS_H, tx_buf, 1, true);
	//等待TWI总线传输完成	
	while (m_xfer_done == false){}	
	
		
	//TWI传输完成标志设置为false
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, ICM_ADDRESS_H, Bufp, len);
	//等待TWI总线传输完成
	while (m_xfer_done == false){}		

	return result;	
}





//int inv_serial_interface_read_hook(uint16_t reg, uint32_t length, uint8_t *data)




















