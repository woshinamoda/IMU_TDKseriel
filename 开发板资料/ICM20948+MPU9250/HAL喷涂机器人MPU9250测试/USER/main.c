#include "sys.h"
#include "delay.h"
#include "usart.h"

#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

/************************************************
 ALIENTEK ̽����STM32F407������ʵ��0-1
 Template����ģ��-�½������½�ʹ��-HAL��汾
 ����֧�֣�www.openedv.com
 �Ա����̣� http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


/***ע�⣺�����̺ͽ̳��е��½�����3.3С�ڶ�Ӧ***/


void Delay(__IO uint32_t nCount);

void Delay(__IO uint32_t nCount)
{
  while(nCount--){}
}

int main(void)
{
	u8 t=0,report=1;	            //Ĭ�Ͽ����ϱ�
	u8 key;
	float pitch,roll,yaw; 	        //ŷ����
	short aacx,aacy,aacz;	        //���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;        //������ԭʼ���� 
	short temp;		                //�¶�
	u8 rest=0;
	
	GPIO_InitTypeDef GPIO_Initure;
     
    HAL_Init();                    	 			//��ʼ��HAL��    
    Stm32_Clock_Init(336,8,2,7);   				//����ʱ��,168Mhz

		delay_init(180);                //��ʼ����ʱ����
	  uart_init(115200);              //��ʼ��USART   500000
	
//    __HAL_RCC_GPIOF_CLK_ENABLE();           	//����GPIOFʱ��
//	
//    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10; 	//PF9,10
//    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  	//�������
//    GPIO_Initure.Pull=GPIO_PULLUP;          	//����
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;    	 	//����
//    HAL_GPIO_Init(GPIOF,&GPIO_Initure);

	rest=mpu_dmp_init();
	printf("%d\n",rest);
	while(rest)
	{
		rest=mpu_dmp_init();
		printf("%d\n",rest);
	}
	
		
	
	while(1)
	{
		
		 if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
     {
        temp=MPU_Get_Temperature();	//�õ��¶�ֵ
//		    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//		    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			 
			  printf("pitch=%f roll=%f yaw=%f\n",pitch,roll,yaw);
		 }
		 
		
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_SET);		//PF9��1 
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);		//PF10��1  			
//		Delay(0x7FFFFF);
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET);		//PF9��0
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);	//PF10��0  
//		Delay(0x7FFFFF);
	}
}
