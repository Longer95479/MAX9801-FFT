#ifndef __RDA__H
#define __RDA__H

#include "common.h" 
#include "MK60_GPIO.h"

#define RDA_SCL_PIN  PTD8 //模拟IIC的SCL信号  1.修改引脚即可修改IIC接口
#define RDA_SDA_PIN  PTD9 //模拟IIC的SDA信号

#define RDA_SDA_IN()  GPIO_PinSetDir(RDA_SDA_PIN, 0);	//输入
#define RDA_SDA_OUT() GPIO_PinSetDir(RDA_SDA_PIN, 1);	//输出

	 
#define RDA_SCL    PTD8_OUT  //SCL            2.修改引脚即可修改IIC接口    
#define RDA_SDA    PTD9_OUT  //SDA	 
#define RDA_READ_SDA   PTD9_IN   //输入SDA 
/*  RDA5807寄存器  */
#define RDA_WRITE 0x22
#define RDA_READ  0x23
#define RDA_ID    0x00     //读出16个位的ID=0X5800
#define RDA_R02   0x02
#define RDA_R03   0x03
#define RDA_R04   0x04
#define RDA_R05   0x05
#define RDA_R06   0x06
#define RDA_R07   0x07

/*---------------------------------------------------------------
            IIC内部函数
----------------------------------------------------------------*/		 
void RDA_IIC_Start(void);			        //发送IIC开始信号
void RDA_IIC_Stop(void);	  	            //发送IIC停止信号
void RDA_IIC_Ack(void);					//IIC发送ACK信号
void RDA_IIC_NAck(void);				    //IIC不发送ACK信号
uint8_t RDA_IIC_WaitAck(void); 		        //IIC等待ACK信号
void RDA_IIC_SendByte(uint8_t data);        //IIC发送一个字节
uint8_t RDA_IIC_ReadByte(uint8_t ack);       //IIC读取一个字节


/*---------------------------------------------------------------
            IIC用户函数
----------------------------------------------------------------*/
void RDA_IIC_Init(void);                    //初始化IIC的IO口   
uint8_t RDA_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t RDA_IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t RDA_IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t RDA_IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
/*---------------------------------------------------------------
            RDA读写函数
----------------------------------------------------------------*/
uint16_t RDA_readreg(uint8_t reg_addr);
void RDA_writereg(uint8_t reg_addr,uint16_t value);
void RDA_Init(void);
void Set_Fre(uint32_t fre);
uint16_t Get_RSSI(void);
void Vol_Set(uint8_t vol);


#endif