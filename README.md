# IMU_TDKseriel
TDK旗下6轴9轴芯片驱动代码


# 更新记录
***
2022-12-19<br>
新增STM32驱动代码<br>
- 内容1：通过外部中断PC3读取加速度/角速度/地磁仪数据，但是由于罗盘仪没有对应中断寄存器，所以每过15个姿态数据后读取一次罗盘仪的数据<br>
- 内容2：STM32的硬件IIC速度最好设置到400K，100K如果出现PE=0或者busy，需要用户自己复位。

新增文件注释
- icm_20948_F412Discovery：对应STM32硬件IIC驱动程序
- icm_20948_Nordic_NRF52832Example：对应带协议栈nrf52832的硬件TWI驱动程序

开发板资料注释
- 1.DK开发板官方资料：对应TDK官方开发板资料和程序和一些安装软件，需要用到Ateml的GCC
- 2.ICM20948：网上参考代码
- 3.ICM20948_eMD_nucleo_1.0：TDK对应STM32NUCLEO的驱动代码，需要用到IAR编译器
- 4.pdf1：icm20948与MPU9250对比参考
- 5.pdf2：icm20948数据手册，编写代码可以参考该文档，又寄存器详细介绍
***
2022-12-15<br>
文件夹firmware:icm—20948蓝牙工程<br>
文件夹开发板资料-官网以及CNSD下载下来的参考文档<br>

***
2022-12-15<br>
修改中断读出数据，但是只能读前两次<br>

![Image text]
(picutre_Config/NUCLEO_circuit_icm.png)
