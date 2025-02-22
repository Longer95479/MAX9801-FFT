/*---------------------------------------------------------------
【平    台】龙邱K60核心板-智能车板
【编    写】LQ-005
【E-mail  】chiusir@163.com
【软件版本】V1.0，龙邱开源代码，仅供参考，后果自负
【最后更新】2019年04月02日
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【编译平台】IAR 8.2
【功    能】中断服务函数
【注意事项】中断服务函数的名称在startup_MK60D10.s 中
----------------------------------------------------------------*/
#include "include.h"


/////////////////////////////////////////////////////////////////
///////////////GPIO中断服务函数//////////////////////////////////
/////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------    
//全局变量  用于测试按键外部中断
//------------------------------------------------------------------------------------------------ 

/*---------------------------------------------------------------
【函    数】PORTA_Interrupt
【功    能】PORTA端口的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void PORTA_IRQHandler()
{
   
}

extern volatile uint8_t key_exti_flag;
/*---------------------------------------------------------------
【函    数】PORTB_Interrupt
【功    能】PORTB端口的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void PORTB_IRQHandler()
{
    
    /* 中断使用的引脚号，这里只写了管脚20和管脚21和22，使用其他管脚可以自行添加 */
    int n;  
    n=20;
    if((PORTB_ISFR & (1<<n)))
    {
        /* 用户自行添加中断内程序 */
        key_exti_flag = 0;
    } 
    n=21;
    if((PORTB_ISFR & (1<<n)))
    {
        /* 用户自行添加中断内程序 */
        key_exti_flag = 1;
    } 
    n=22;
    if((PORTB_ISFR & (1<<n)))
    {
        /* 用户自行添加中断内程序 */
        key_exti_flag = 2;
    }
    //清除中断标志
    PORTB_ISFR = 0xffffffff;
}


/////////////////////////////////////////////////////////////////
///////////////PIT中断服务函数///////////////////////////////////
/////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------    
//全局变量  用于测试PIT定时器
//------------------------------------------------------------------------------------------------ 
extern volatile uint8_t pit0_test_flag;
extern volatile uint8_t pit1_test_flag;
extern volatile uint8_t pit2_test_flag;
extern volatile uint8_t pit3_test_flag;

/*---------------------------------------------------------------
【函    数】PIT0_Interrupt
【功    能】PIT0的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void PIT0_IRQHandler()
{
    PIT_Flag_Clear(PIT0);       //清中断标志位
    
    uint16_t count = TPM1->CNT;
    TPM1->CNT = 0;
    printf("rps = %f\n", (float)(count * 50 / 520.0)); 
    
    pit0_test_flag = 1;
}
/*---------------------------------------------------------------
【函    数】PIT1_Interrupt
【功    能】PIT1的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void PIT1_IRQHandler()
{
    PIT_Flag_Clear(PIT1);       //清中断标志位
    /*用户添加所需代码*/
    pit1_test_flag = 1;
}
/*---------------------------------------------------------------
【函    数】PIT2_Interrupt
【功    能】PIT2的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void PIT2_IRQHandler()
{
    PIT_Flag_Clear(PIT2);       //清中断标志位
    /*用户添加所需代码*/
    pit2_test_flag = 1;
}
/*---------------------------------------------------------------
【函    数】PIT3_Interrupt
【功    能】PIT3的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void PIT3_IRQHandler()
{
    PIT_Flag_Clear(PIT3);       //清中断标志位
    /*用户添加所需代码*/
    pit3_test_flag = 1;
}

/////////////////////////////////////////////////////////////////
///////////////DMA中断服务函数///////////////////////////////////
/////////////////////////////////////////////////////////////////
/*---------------------------------------------------------------
【函    数】DMA_CH4_Handler
【功    能】DMA通道4的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void DMA4_IRQHandler(void)
{
    /* 清除通道传输中断标志位    (这样才能再次进入中断) */
    DMA_IRQ_CLEAN(DMA_CH4);          
    
    /* 采集完H个数据后进入这个DMA中断，停止DMA传输。行中断中打开DMA传输 */
    DMA_DIS(DMA_CH4);                                      
}




/////////////////////////////////////////////////////////////////
///////////////串口中断服务函数/////////////////////////////////
/////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------
【函    数】UART4_RX_TX_IRQHandler
【功    能】UART4的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void UART4_RX_TX_IRQHandler(void)
{
    if(UART4_S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
        //用户需要处理接收数据
        printf("接收到字符： %c \n", UART_GetChar(UART4));
    }
    
}

