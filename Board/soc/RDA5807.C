#include "include.h"
#include "RDA5807.h"
/******************************************************************************
*函  数：void RDA_delay_us(void)
*功　能：IIC延时
*参  数：无
*返回值：无
*备  注: 移植时只需要将RDA_delay_us()换成自己的延时即可
*******************************************************************************/	
void RDA_delay_us(uint8_t us)
{
    for(int i = 0; i < 100; i++)    
    {
        asm("NOP");//core bus 200M  情况下大概IIC速率 400K
    }
}

/******************************************************************************
*函  数：void IIC_Init(void)
*功　能：IIC初始化
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/	
void RDA_IIC_Init(void)
{			
    GPIO_PinInit(RDA_SCL_PIN,GPO,1);
    GPIO_PinInit(RDA_SDA_PIN,GPO,1);
    
    uint8_t ptx = PTX(RDA_SDA_PIN);
    uint8_t ptn = PTn(RDA_SDA_PIN);
    
    PORTX[ptx]->PCR[ptn] |= 0x01 << 1;
    PORTX[ptx]->PCR[ptn] |= 0x01 << 5;	
	
    RDA_SCL=1;
    RDA_SDA=1;
}
/******************************************************************************
*函  数：void IIC_Start(void)
*功　能：产生IIC起始信号
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/	
void RDA_IIC_Start(void)
{
	RDA_SDA_OUT(); //sda线输出 
	RDA_SDA=1;	
	RDA_SCL=1;
	delayus(4);
 	RDA_SDA=0; //START:when CLK is high,DATA change form high to low 
	delayus(4);
	RDA_SCL=0; //钳住I2C总线，准备发送或接收数据 
}

/******************************************************************************
*函  数：void IIC_Stop(void)
*功　能：产生IIC停止信号
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/	  
void RDA_IIC_Stop(void)
{
	RDA_SDA_OUT(); //sda线输出
	RDA_SCL=0;
	RDA_SDA=0; //STOP:when CLK is high DATA change form low to high
        delayus(4);
	RDA_SCL=1; 
	RDA_SDA=1; //发送I2C总线结束信号
        delayus(4);							   	
}

/******************************************************************************
*函  数: uint8_t IIC_WaitAck(void)
*功　能: 等待应答信号到来 （有效应答：从机第9个 SCL=0 时 SDA 被从机拉低,
并且 SCL = 1时 SDA依然为低）
*参  数：无
*返回值：1，接收应答失败
0，接收应答成功
*备  注：从机给主机的应答
*******************************************************************************/
uint8_t RDA_IIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	RDA_SDA_IN(); //SDA设置为输入  （从机给一个低电平做为应答） 
	RDA_SDA=1;delayus(1);	   
	RDA_SCL=1;delayus(1);;	 
	while(RDA_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			RDA_IIC_Stop();
			return 1;
		}
	}
	RDA_SCL=0; //时钟输出0 	   
	return 0;  
} 

/******************************************************************************
*函  数: void IIC_Ack(void)
*功　能: 产生ACK应答 （主机接收完一个字节数据后，主机产生的ACK通知从机一个
字节数据已正确接收）
*参  数：无
*返回值：无
*备  注：主机给从机的应答
*******************************************************************************/

void RDA_IIC_Ack(void)
{
	RDA_SCL=0;
	RDA_SDA_OUT();
	RDA_SDA=0;
	delayus(1);
	RDA_SCL=1;
	delayus(2);
	RDA_SCL=0;
}

/******************************************************************************
*函  数: void IIC_NAck(void)
*功　能: 产生NACK应答 （主机接收完最后一个字节数据后，主机产生的NACK通知从机
发送结束，释放SDA,以便主机产生停止信号）
*参  数：无
*返回值：无
*备  注：主机给从机的应答
*******************************************************************************/
void RDA_IIC_NAck(void)
{
	RDA_SCL=0;
	RDA_SDA_OUT();
	RDA_SDA=1;
	delayus(1);
	RDA_SCL=1;
	delayus(1);
	RDA_SCL=0;
}					 				     

/******************************************************************************
*函  数：void IIC_SendByte(uint8_t txd)
*功  能：IIC发送一个字节
*参  数：data 要写的数据
*返回值：无
*备  注：主机往从机发
*******************************************************************************/		  
void RDA_IIC_SendByte(uint8_t data)
{                        
    uint8_t t;   
    RDA_SDA_OUT(); 	    
    RDA_SCL=0; //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        RDA_SDA=(data&0x80)>>7;
        data<<=1;
        delayus(1);			
        RDA_SCL=1;
        delayus(1);
        RDA_SCL=0;	
        delayus(1);
    }	 
} 	 

/******************************************************************************
*函  数：uint8_t IIC_ReadByte(uint8_t ack)
*功  能：IIC读取一个字节
*参  数：ack=1 时，主机数据还没接收完 ack=0 时主机数据已全部接收完成
*返回值：无
*备  注：从机往主机发
*******************************************************************************/	
uint8_t RDA_IIC_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;
	RDA_SDA_IN(); //SDA设置为输入模式 等待接收从机返回数据
    for(i=0;i<8;i++ )
	{
        RDA_SCL=0; 
        delayus(1);
        RDA_SCL=1;
        receive<<=1;
        if(RDA_READ_SDA)receive++; //从机发送的电平
        delayus(1); 
    }					 
    if(ack)
        RDA_IIC_Ack(); //发送ACK 
    else
        RDA_IIC_NAck(); //发送nACK  
    return receive;
}

/******************************************************************************
*函  数：uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
*功　能：读取指定设备 指定寄存器的一个值
*参  数：I2C_Addr  目标设备地址
reg	     寄存器地址
*buf      读取数据要存储的地址    
*返回值：返回 1失败 0成功
*备  注：无
*******************************************************************************/ 
uint8_t RDA_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	RDA_IIC_Start();	
	RDA_IIC_SendByte(I2C_Addr);	 //发送从机地址
	if(RDA_IIC_WaitAck()) //如果从机未应答则数据发送失败
	{
		RDA_IIC_Stop();
		return 1;
	}
	RDA_IIC_SendByte(reg); //发送寄存器地址
	RDA_IIC_WaitAck();	  
	
	RDA_IIC_Start();
	RDA_IIC_SendByte(I2C_Addr+1); //进入接收模式			   
	RDA_IIC_WaitAck();
	*buf=RDA_IIC_ReadByte(0);	   
        RDA_IIC_Stop(); //产生一个停止条件
	return 0;
}

/******************************************************************************
*函  数：uint8_t IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr，uint8_t buf))
*功　能：写入指定设备 指定寄存器的一个值
*参  数：I2C_Addr  目标设备地址
reg	     寄存器地址
buf       要写入的数据
*返回值：1 失败 0成功
*备  注：无
*******************************************************************************/ 
uint8_t RDA_IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	RDA_IIC_Start();
	RDA_IIC_SendByte(I2C_Addr); //发送从机地址
	if(RDA_IIC_WaitAck())
	{
		RDA_IIC_Stop();
		return 1; //从机地址写入失败
	}
	RDA_IIC_SendByte(reg); //发送寄存器地址
        RDA_IIC_WaitAck();	  
	RDA_IIC_SendByte(data); 
	if(RDA_IIC_WaitAck())
	{
		RDA_IIC_Stop(); 
		return 1; //数据写入失败
	}
	RDA_IIC_Stop(); //产生一个停止条件
    
    //return 1; //status == 0;
	return 0;
}

/******************************************************************************
*函  数：uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*功　能：读取指定设备 指定寄存器的 length个值
*参  数：dev     目标设备地址
reg	   寄存器地址
length  要读的字节数
*data   读出的数据将要存放的指针
*返回值：1成功 0失败
*备  注：无
*******************************************************************************/ 
uint8_t RDA_IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
        uint8_t count = 0;
	uint8_t temp;
	RDA_IIC_Start();
	RDA_IIC_SendByte(dev); //发送从机地址
	if(RDA_IIC_WaitAck())
	{
		RDA_IIC_Stop(); 
		return 1; //从机地址写入失败
	}
	RDA_IIC_SendByte(reg); //发送寄存器地址
        RDA_IIC_WaitAck();	  
	RDA_IIC_Start();
	RDA_IIC_SendByte(dev+1); //进入接收模式	
	RDA_IIC_WaitAck();
        for(count=0;count<length;count++)
	{
		if(count!=(length-1))
            temp = RDA_IIC_ReadByte(1); //带ACK的读数据
		else  
            temp = RDA_IIC_ReadByte(0); //最后一个字节NACK
        
		data[count] = temp;
	}
    RDA_IIC_Stop(); //产生一个停止条件
    //return count;
    return 0;
}

/******************************************************************************
*函  数：uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*功　能：将多个字节写入指定设备 指定寄存器
*参  数：dev     目标设备地址
reg	   寄存器地址
length  要写的字节数
*data   要写入的数据将要存放的指针
*返回值：1成功 0失败
*备  注：无
*******************************************************************************/ 
uint8_t RDA_IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
    
 	uint8_t count = 0;
	RDA_IIC_Start();
	RDA_IIC_SendByte(dev); //发送从机地址
	if(RDA_IIC_WaitAck())
	{
		RDA_IIC_Stop();
		return 1; //从机地址写入失败
	}
	RDA_IIC_SendByte(reg); //发送寄存器地址
        RDA_IIC_WaitAck();	  
	for(count=0;count<length;count++)
	{
		RDA_IIC_SendByte(data[count]); 
		if(RDA_IIC_WaitAck()) //每一个字节都要等从机应答
		{
			RDA_IIC_Stop();
			return 1; //数据写入失败
		}
	}
	RDA_IIC_Stop(); //产生一个停止条件
    
	return 0;
}


/*读取指定寄存器*/
uint16_t RDA_readreg(uint8_t reg_addr)
{
	uint16_t buff;
	RDA_IIC_Start();
	RDA_IIC_SendByte(RDA_WRITE);
	if (RDA_IIC_WaitAck() != 0)
	{
		goto cmd_fail;	 /*RDA5807无应答 */
	}
	RDA_IIC_SendByte(reg_addr);
	if (RDA_IIC_WaitAck() != 0)
	{
		goto cmd_fail;	 /*RDA5807无应答 */
	}
	RDA_IIC_Start();
	RDA_IIC_SendByte(RDA_READ);
	if (RDA_IIC_WaitAck() != 0)
	{
		goto cmd_fail;	 /*RDA5807无应答 */
	}
	buff=RDA_IIC_ReadByte(1);
	buff=buff<<8;
	buff=buff|RDA_IIC_ReadByte(0);
	RDA_IIC_Stop();
	return buff;
	cmd_fail:
	RDA_IIC_Stop();
	printf("\r\n failed to read register\r\n");
	return 0;
}
/*写指定寄存器*/
void RDA_writereg(uint8_t reg_addr,uint16_t value)
{
	RDA_IIC_Start();
	RDA_IIC_SendByte(RDA_WRITE);
	if (RDA_IIC_WaitAck() != 0)
	{
		goto cmd_fail;	 /*RDA5807无应答 */
	}
	RDA_IIC_SendByte(reg_addr);
    if (RDA_IIC_WaitAck() != 0)
	{
		goto cmd_fail;	 /*RDA5807无应答 */
	}
	RDA_IIC_SendByte(value>>8);
	if (RDA_IIC_WaitAck() != 0)
	{
		goto cmd_fail;	 /*RDA5807无应答 */
	}
	RDA_IIC_SendByte(value&0xFF);
	RDA_IIC_NAck();
	RDA_IIC_Stop();
	cmd_fail:
	RDA_IIC_Stop();
	printf("\r\n failed to write register \r\n");
}

void RDA_Init(void)
{
	RDA_IIC_Init();
	RDA_writereg(RDA_R02,0x0002);  //软启动
	delayms(50);
	RDA_writereg(RDA_R02,0xc281);//使能芯片，晶振32.768KHZ
	delayms(600);
	RDA_writereg(RDA_R03,0x1A10);//初始频率设为97.4MHz，步进100Khz,频带为87M~108M
	RDA_writereg(RDA_R04,0x0400);
	RDA_writereg(RDA_R05,0x86ad);
	RDA_writereg(RDA_R06,0x8000);
	RDA_writereg(RDA_R07,0x5F1A);
}


void Set_Fre(uint32_t fre)  //88MHz~108MHz  200KHz步进  单位KHZ 例如:Set_Fre(110000)  设置接收频率为110Mhz
{
	uint16_t chan, temp=0;
	chan = (int)((fre-87000)/200);
	temp |= chan<<6;//chan[15:6]
	temp |= 1<<4;		//tune enable
	temp |= 1;      //设置步进200khz
	RDA_writereg(RDA_R03, temp);
	delayms(40);
}


uint16_t Get_RSSI(void)//获取信号强度RSSI值0~127
{
	uint16_t temp;
	temp=RDA_readreg(0x0B);
	temp=(temp>>9)&0x7f;
	return temp;
}


void Vol_Set(uint8_t vol)//音频输出电压大小（音量控制），0~15
{
	uint16_t temp=0;
	temp=RDA_readreg(RDA_R05);
	temp&=0xfff0;
	RDA_writereg(0x05,vol|temp);
}
