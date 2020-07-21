/**
 * @Platform    龙邱K66核心板        IAR 8.32.1
 * @Fielname    ADC&DMA.c
 * @brief       ADC由 LPTMR 触发并利用DMA 传输到内存，并对数据类型进行转化的相关函数
 * @Author      Longer95479
 * @Email       371573369@qq.com
 * @Date        2020/7/17
 *
 */


/**
 * @brief       激活存储AD引脚编号的全局变量，利用 DMA 自动更改引脚（通过写入 ADC_SC1A[ADCH] 来实现）
 */
#define ADCDMA_GLOBALS   
   
#include "include.h"

/**
 * @brief 内部使用函数声明
 */
static void DMA_Init_for_ADC(DMA_CHn CHn, void *SADDR, void *DADDR, DMA_BYTEn byten, int16_t soff, int16_t doff, uint16_t count, int32_t slast, int32_t dlast, DMA_sources DMA_source);
static void ADC_Start_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit);
static void LPTMR_for_ADC_trigger(uint32_t us);





/**
 * @brief       初始化ADC转换，由 LPTMR 触发，并把数据通过DMA搬运到内存，然后通过DMA链接功能更换ADC0的引脚
 * @param       init_str: 初始化结构体
 * @return
 * @example
 * @note        源和目的的数据宽度均固定为 2bytes，主循环次数固定为2048次
 *
 */
void ADC_in_DMA_mode_Init(ADC_in_DMA_mode_Init_StrType *init_str)
{ 
  DMA_Init_for_ADC(init_str->CHn,                //选择DMA通道
                   init_str->SADDR,             //选择源地址
                   init_str->DADDR,             //选择目的地址
                   init_str->byten,             //源数据和目的数据位宽
                   init_str->soff,              //源地址偏移
                   init_str->doff,              //目的地址偏移
                   init_str->count,             //主循环次数
                   init_str->slast,             //主循环结束后源地址偏移
                   init_str->dlast,             //主循环结束后目的地址偏移
                   init_str->DMA_source);       //选择DMA触发源
  
  ADC_Start_for_DMA(init_str->adc_n, init_str->adc_ch, ADC_16bit);
  
  LPTMR_for_ADC_trigger(init_str->us);
  
}




/**
 * @brief       LPTMR定时触发ADC采集，周期为100us
 * @param       us：触发周期，单位为微秒
 * @return
 * @example
 * @note        LPTMR0_CMR = xus * 50 /32       xus最大为41942us
 *              按道理 CSR[TIE] 和 CSR[TCF] 都置位应产生中断，但实际却进不去中断函数。
 * @date        2020/7/19
 *
 */
static void LPTMR_for_ADC_trigger(uint32_t us)
{


    OSC_CR |= OSC_CR_ERCLKEN_MASK;                              //使能 OSCERCLK

    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;                          //使能LPT模块时钟

    LPTMR0_CSR = 0x00;                                          //先关了LPT，自动清计数器的值

    LPTMR0_CMR = ((us < 41943) ? us : 41942) * 50 / 32;       //设置比较值

    //选择时钟源
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(3)          //选择时钟源： 0 为 MCGIRCLK ，1为 LPO（1KHz） ，2为 ERCLK32K ，3为 OSCERCLK
                      //| LPTMR_PSR_PBYP_MASK     //旁路 预分频/干扰滤波器 ,即不用 预分频/干扰滤波器(注释了表示使用预分频/干扰滤波器)
                      | LPTMR_PSR_PRESCALE(4)     //预分频值 = 2^(n+1) ,n = 0~ 0xF
                    );

    //使能 LPT
    LPTMR0_CSR  =  (0
                    //| LPTMR_CSR_TPS(1)        // 选择输入管脚 选择
                    //| LPTMR_CSR_TMS_MASK      // 选择脉冲计数 (注释了表示时间计数模式)
                    //| ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //脉冲计数器触发方式选择：0为高电平有效，上升沿加1
                    | LPTMR_CSR_TEN_MASK        //使能LPT(注释了表示禁用)
                    | LPTMR_CSR_TIE_MASK      //中断使能
                    //| LPTMR_CSR_TFC_MASK      //0:计数值等于比较值就复位；1：溢出复位（注释表示0）
                   );
    
    SIM->SOPT7 = 0x8e8eU;       //设置 LPTMR 作为 ADC 的触发信号
}



/**
 * @brief       用于DMA传输的触发
 * @param       us: 触发周期
 * @return
 * @example
 * @note        开启了CH0 1 2 3，对应DMA的 CH0 1 2 3 
 * @date        2020/7/7
 *
 */
//void PIT_Init_for_DMA(uint32_t us)
//{
  //PIT 用的是 Bus Clock 总线频率

    /* 开启时钟*/
//    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                          

    /* 使能PIT定时器时钟 ，调试模式下继续运行 */
//    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );   
    
    /* 设置触发周期 */
//    PIT_LDVAL(PIT0)  = us * bus_clk;     
//    PIT_LDVAL(PIT1)  = us * bus_clk;     
//    PIT_LDVAL(PIT2)  = us * bus_clk;     
//    PIT_LDVAL(PIT3)  = us * bus_clk;  
    
    /* 使能 PITn定时器 */
//    PIT_TCTRL(PIT0) |= PIT_TCTRL_TEN_MASK;   
//    PIT_TCTRL(PIT1) |= PIT_TCTRL_TEN_MASK;   
//    PIT_TCTRL(PIT2) |= PIT_TCTRL_TEN_MASK;   
//    PIT_TCTRL(PIT3) |= PIT_TCTRL_TEN_MASK;   
//}



   
/**
 * @brief       DMA初始化，读取ADC数据到内存
 * @param       CHn: 通道号（DMA_CH0 ~ DMA_CH15）
 *               SADDR: 源地址                                      
 *               DADDR: 目的地址
 *               byten: 源地址和目的地址的数据宽度                 1(2 bytes)
 *               soff: 源地址偏移
 *               doff：目标地址偏移
 *               count: 主循环次数                     2048（但若设置了通道间链接的功能，最大只能为511次,因此需要分成四次进行，中间用中断更改源地址）
 *               slast：主循环结束后源地址的偏移
 *               dlast：主循环结束后目标地址的偏移
 *               DMA_source：DMA传输的触发源
 * @return
 * @example
 * @note        次循环一次传输16bit(NBYTES = 2)，源数据和目标数据宽度都是16bit，DOFF = 8 BYTES, 要进行2048次主循环
 * 
 * @date 2020/7/16
 *
 */                             
static void DMA_Init_for_ADC(DMA_CHn CHn, void *SADDR, void *DADDR, DMA_BYTEn byten, int16_t soff, int16_t doff, uint16_t count, int32_t slast, int32_t dlast, DMA_sources DMA_source)
{

    uint8_t BYTEs = (byten == DMA_BYTE1 ? 1 : (byten == DMA_BYTE2 ? 2 : (byten == DMA_BYTE4 ? 4 : 16 ) ) ); //计算传输字节数

    /* 开启时钟 */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //打开DMA模块时钟
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK; 
	
    /* 配置 DMA 通道 的 传输控制块 TCD ( Transfer Control Descriptor ) */
    DMA_SADDR(CHn) =    (uint32_t)SADDR;                         // 设置  源地址
    DMA_DADDR(CHn) =    (uint32_t)DADDR;                         // 设置目的地址
    DMA_SOFF(CHn)  =    soff;                              // 设置源地址偏移 
    DMA_DOFF(CHn)  =    doff;                            // 目的地址偏移

    DMA_ATTR(CHn)  =    (0
                         | DMA_ATTR_SMOD(0x0)                // 源地址模数禁止  Source address modulo feature is disabled
                         | DMA_ATTR_SSIZE(byten)             // 源数据位宽 ：DMA_BYTEn  。    SSIZE = 0 -> 8-bit ，SSIZE = 1 -> 16-bit ，SSIZE = 2 -> 32-bit ，SSIZE = 4 -> 16-byte
                         | DMA_ATTR_DMOD(0x0)                // 目标地址模数禁止
                         | DMA_ATTR_DSIZE(byten)             // 目标数据位宽 ：DMA_BYTEn  。  设置参考  SSIZE
                        );

    //**DMA_CITER_ELINKNO(CHn)  = DMA_CITER_ELINKNO_CITER(count); //当前主循环次数 
    //**DMA_BITER_ELINKNO(CHn)  = DMA_BITER_ELINKNO_BITER(count); //起始主循环次数
    
    DMA_CITER_ELINKYES(CHn)  |= DMA_CITER_ELINKYES_CITER(count); //当前主循环次数 
    DMA_BITER_ELINKYES(CHn)  |= DMA_BITER_ELINKYES_BITER(count); //起始主循环次数

    DMA_CR &= ~DMA_CR_EMLM_MASK;                            // CR[EMLM] = 0

    //当CR[EMLM] = 0 时:
    DMA_NBYTES_MLNO(CHn) =   DMA_NBYTES_MLNO_NBYTES(BYTEs); // 通道每次传输字节数，这里设置为BYTEs个字节。注：值为0表示传输4GB */


    /* 配置 DMA 传输结束后的操作 */
    DMA_SLAST(CHn)      =   slast;                              //调整  源地址的附加值,主循环结束后恢复  源地址
    DMA_DLAST_SGA(CHn)  =   dlast;/*(xOFF_mode == UP) ? (-count * 8) : (count * 8)*/ //调整目的地址的附加值,主循环结束后恢复目的地址或者保持地址
    DMA_CSR(CHn)        =   (0
                             | DMA_CSR_BWC(3)               //带宽控制,每读一次，eDMA 引擎停止 8 个周期（0不停止；1保留；2停止4周期；3停止8周期）
                             | DMA_CSR_DREQ_MASK            //主循环结束后停止硬件请求
                             | DMA_CSR_INTMAJOR_MASK        //主循环结束后产生中断
                            );

    /* 配置 DMA 触发源 */
    DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, CHn) = (0
            | DMAMUX_CHCFG_ENBL_MASK                        /* Enable routing of DMA request */
            //| DMAMUX_CHCFG_TRIG_MASK                        /* Trigger Mode: Periodic   PIT周期触发传输模式   通道1对应PIT1，必须使能PIT1，且配置相应的PIT定时触发 */
            | DMAMUX_CHCFG_SOURCE(DMA_source)              ); /* 通道触发传输源选择，具体编号可在头文件里查看*/



    /* 开启中断 */
    DMA_EN(CHn);                                    //使能通道CHn 硬件请求
    DMA_IRQ_EN(CHn);                                //允许DMA通道传输
}


/**
 * @brief       链接DMA通道，实现自动更换DMA通道传输
 * @param       to_link_CH：要链接其他通道的通道
 *               be_linked_CH：被链接的通道
 *               link_type：链接类次：主循环结尾链接 或 主循环结尾链接
 * @return
 * @example
 * @note
 * @date        2020/7/20
 *
 */
static void DMA_set_channel_link(DMA_CHn to_link_CH, DMA_CHn be_linked_CH, link_type_EnumType link_type)
{
  switch (link_type) {
     case DMA_minor_link: DMA_CITER_ELINKYES(to_link_CH) = (DMA_CITER_ELINKYES(to_link_CH)               //次循环结尾链接
                                                          | DMA_CITER_ELINKYES_ELINK(1)             //使能次循环结束的通道间链接
                                                          | DMA_CITER_ELINKYES_LINKCH(be_linked_CH)     //选择被链接的通道
                                                      );
                         break;
    
    case DMA_major_link: DMA_CSR(to_link_CH) = (DMA_CSR(to_link_CH)               //主循环结束链接
                                              | DMA_CSR_MAJORELINK(1)         //使能主循环结尾的通道间链接
                                              | DMA_CSR_MAJORLINKCH(be_linked_CH)      //选择被链接的通道
                                               );
                        break;
      
  }
  
}



/**
 * @brief       ADC初始化，由 LPTMR 触发采集，转换完成触发DMA传输
 * @param       adc_n ：  模块名ADC0或ADC1 
 *               adc_ch：  ADC通道编号 
 *                bit   ：  精度选择ADC_8bit，ADC_12bit，ADC_10bit，ADC_16bit
 * @return
 * @example
 * @note        时钟初始化也并入其中
 * @date        20207/19
 *
 */
static void ADC_Start_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit)
{
  
   ADC_Init(adc_n);
   
   if(adc_n==ADC0)
   {
     ADC0_CFG1 = (0  | ADC_CFG1_ADIV(2)              //时钟分频选择,分频系数为 2^n,2bit
                     | ADC_CFG1_ADLSMP_MASK          //采样时间配置，0为短采样时间，1 为长采样时间
                     | ADC_CFG1_MODE(bit)            //精确度选择
                     | ADC_CFG1_ADICLK(0)            //0为总线时钟,1为总线时钟/2,2为交替时钟（ALTCLK），3为 异步时钟（ADACK）。
                 );


    ADC0_CFG2  = (0  | ADC_CFG2_ADHSC_MASK          //高速配置,0为正常转换序列，1为高速转换序列
                     | ADC_CFG2_ADLSTS(0)           //长采样时间选择，ADCK为4+n个额外循环，额外循环，0为20，1为12，2为6，3为2
                  );
    
    ADC0_SC2 = (0 | ADC_SC2_DMAEN_MASK              //开启ADC的DMA 传输功能
                  |  ADC_SC2_ADTRG_MASK             //开启硬件触发
               );          
    
    //ADC0_SC3 = 0 | ADC_SC3_ADCO_MASK;           //开启连续转换模式

    //写入 SC1A 启动转换
    ADC0_SC1(0) = (0 | ADC_SC1_AIEN_MASK            // 转换完成中断,0为禁止，1为使能
                     //| ADC_SC1_ADCH( 0x0c )       
                     | ADC_SC1_ADCH( adc_ch )       //ADC0 通道选择
                  );
   }
   else 
   {
     ADC1_CFG1 = (0   | ADC_CFG1_ADIV(2)              //时钟分频选择,分频系数为 2^n,2bit
                     | ADC_CFG1_ADLSMP_MASK           //采样时间配置，0为短采样时间，1 为长采样时间
                     | ADC_CFG1_MODE(bit)             //精确度选择
                     | ADC_CFG1_ADICLK(0)             //0为总线时钟,1为总线时钟/2,2为交替时钟（ALTCLK），3为 异步时钟（ADACK）。
                 );


    ADC1_CFG2  = (0  | ADC_CFG2_ADHSC_MASK          //高速配置,0为正常转换序列，1为高速转换序列
                     | ADC_CFG2_ADLSTS(0)           //长采样时间选择，ADCK为4+n个额外循环，额外循环，0为20，1为12，2为6，3为2
                  );
    
    ADC0_SC2 = (0 | ADC_SC2_DMAEN_MASK           //开启ADC的DMA 传输功能
                |  ADC_SC2_ADTRG_MASK             //开启硬件触发
               );      
    
    //ADC0_SC3 = 0 | ADC_SC3_ADCO_MASK;           //开启连续转换模式
    
    //写入 SC1A 启动转换
    ADC1_SC1(0) = (0 | ADC_SC1_AIEN_MASK            // 转换完成中断,0为禁止，1为使能
                     //| ADC_SC1_ADCH( 0x0c )       
                     | ADC_SC1_ADCH( adc_ch )       //ADC1 通道选择
                  );
   }
}

