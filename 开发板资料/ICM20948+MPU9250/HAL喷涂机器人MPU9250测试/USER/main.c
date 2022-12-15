#include "sys.h"
#include "delay.h"
#include "usart.h"

#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

/************************************************
 ALIENTEK 探索者STM32F407开发板实验0-1
 Template工程模板-新建工程章节使用-HAL库版本
 技术支持：www.openedv.com
 淘宝店铺： http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/


/***注意：本工程和教程中的新建工程3.3小节对应***/


void Delay(__IO uint32_t nCount);

void Delay(__IO uint32_t nCount)
{
  while(nCount--){}
}

int main(void)
{
	u8 t=0,report=1;	            //默认开启上报
	u8 key;
	float pitch,roll,yaw; 	        //欧拉角
	short aacx,aacy,aacz;	        //加速度传感器原始数据
	short gyrox,gyroy,gyroz;        //陀螺仪原始数据 
	short temp;		                //温度
	u8 rest=0;
	
	GPIO_InitTypeDef GPIO_Initure;
     
    HAL_Init();                    	 			//初始化HAL库    
    Stm32_Clock_Init(336,8,2,7);   				//设置时钟,168Mhz

		delay_init(180);                //初始化延时函数
	  uart_init(115200);              //初始化USART   500000
	
//    __HAL_RCC_GPIOF_CLK_ENABLE();           	//开启GPIOF时钟
//	
//    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10; 	//PF9,10
//    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  	//推挽输出
//    GPIO_Initure.Pull=GPIO_PULLUP;          	//上拉
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;    	 	//高速
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
        temp=MPU_Get_Temperature();	//得到温度值
//		    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//		    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			 
			  printf("pitch=%f roll=%f yaw=%f\n",pitch,roll,yaw);
		 }
		 
		
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_SET);		//PF9置1 
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);		//PF10置1  			
//		Delay(0x7FFFFF);
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET);		//PF9置0
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);	//PF10置0  
//		Delay(0x7FFFFF);
	}
}
