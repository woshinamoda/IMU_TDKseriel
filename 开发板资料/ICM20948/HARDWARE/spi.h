#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//PB12 -- NCS
//PB13 -- SCK
//PB14 -- SDO
//PB15 -- SDI
#define	ICM20948_CS  PBout(12)   //Ƭѡ�ź�

void SPI2_Init(void);			 //��ʼ��SPI��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI�ٶ�
u8 SPI2_ReadWriteByte(u8 TxData,u8* RxData);//SPI���߶�дһ���ֽ�

#endif

