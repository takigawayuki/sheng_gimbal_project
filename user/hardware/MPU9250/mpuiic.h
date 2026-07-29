#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"

//正点原子MPU9250通讯线驱动
//由正点原子MPU6050驱动修改



//IO方向设置
#define MPU_SDA_IN()  {GPIOC->MODER = (GPIOC->MODER & ~(3 << (2*5))) | (0 << (2*5));}
#define MPU_SDA_OUT() {GPIOC->MODER = (GPIOC->MODER & ~(3 << (2*5))) | (1 << (2*5));}

////IO操作函数	 
#define MPU_IIC_SCL_SET    HAL_GPIO_WritePin(MPU9250_SCL_GPIO_Port,MPU9250_SCL_Pin,GPIO_PIN_SET) 		//SCL
#define MPU_IIC_SCL_RESET    HAL_GPIO_WritePin(MPU9250_SCL_GPIO_Port,MPU9250_SCL_Pin,GPIO_PIN_RESET)
#define MPU_IIC_SDA_SET   HAL_GPIO_WritePin(MPU9250_SDA_GPIO_Port,MPU9250_SDA_Pin,GPIO_PIN_SET) 		//SDA	 
#define MPU_IIC_SDA_RESET    HAL_GPIO_WritePin(MPU9250_SDA_GPIO_Port,MPU9250_SDA_Pin,GPIO_PIN_RESET) 		//SDA
#define MPU_READ_SDA   HAL_GPIO_ReadPin(MPU9250_SDA_GPIO_Port,MPU9250_SDA_Pin) 		//输入SDA

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数	
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















