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
 * @brief       ADC 引脚复用的存储数组，用于被 DMA 传送
 */ 
uint8_t g_ADC0_mux[2] = {ADC0_SE8, ADC0_SE9};
uint8_t g_ADC1_mux[2] = {ADC1_SE6a, ADC1_SE7a};
   

/**
 * @brief 内部使用函数声明
 */
static void DMA_Init_for_ADC(DMA_CHn CHn, void *SADDR, void *DADDR, DMA_BYTEn byten, int16_t soff, int16_t doff, uint16_t count, int32_t slast, int32_t dlast, DMA_sources DMA_source);
static void ADC_conf_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit);
static void ADC_chan_select_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch);
static void LPTMR_for_ADC_trigger(uint32_t us);
static void DMA_set_channel_link(DMA_CHn to_link_CH, DMA_CHn be_linked_CH, link_type_EnumType link_type);



/**
 * @brief       ADC_in_DMA_mode 的实例配置
 * @param
 * @return
 * @example
 * @note
 * @date        2020/7/22
 *
 */
void ADC_in_DMA_mode_instance_conf(uint32_t us)
{
  // init_str0 配置了用于传输 ADC0 数据的 DMA0
  ADC_in_DMA_mode_Init_StrType init_str0;  
  init_str0.CHn = DMA_CH0;
  init_str0.SADDR = (void *)&ADC0_RA;
  init_str0.DADDR = sample_sx;
  init_str0.byten = DMA_BYTE2;
  init_str0.soff = 0x0;
  init_str0.doff = sizeof(uint32_t);
  init_str0.count = (COUNT_MAX + 1) / 2;        //256
  init_str0.slast = 0; 
  init_str0.dlast = 0;         
  init_str0.DMA_source = DMA_ADC0;
  
  init_str0.adc_n = ADC0;
  init_str0.adc_ch = ADC0_SE8;
  
  
  // init_str1 配置了用于更改 ADC引脚 的 DMA1
  ADC_in_DMA_mode_Init_StrType init_str1;  
  init_str1.CHn = DMA_CH1;
  init_str1.SADDR = g_ADC0_mux;
  init_str1.DADDR = (void *)&ADC0_SC1A;
  init_str1.byten = DMA_BYTE1;
  init_str1.soff = sizeof(uint8_t);
  init_str1.doff = 0x0;
  init_str1.count = 2;
  init_str1.slast = -2 /** (int)sizeof(uint8_t)*/;
  init_str1.dlast = 0x0;
  init_str1.DMA_source = DMA_Always_EN3;
  
  init_str1.adc_n = ADC0;
  init_str1.adc_ch = ADC0_SE8;
  
  
  // init_str2 配置了用于传输 ADC1 数据的 DMA2
  ADC_in_DMA_mode_Init_StrType init_str2;  
  init_str2.CHn = DMA_CH2;
  init_str2.SADDR = (void *)&ADC1_RA;
  init_str2.DADDR = sample_sy;
  init_str2.byten = DMA_BYTE2;
  init_str2.soff = 0x0;
  init_str2.doff = sizeof(uint32_t);
  init_str2.count = (COUNT_MAX + 1) / 2;
  init_str2.slast = 0x0;
  init_str2.dlast = 0/*-COUNT_MAX * (int)sizeof(uint32_t)*/;
  init_str2.DMA_source = DMA_ADC1;
  
  init_str2.adc_n = ADC1;
  init_str2.adc_ch = ADC1_SE6a; //第一次采集的AD 引脚
  
  
  // init_str3 配置了用于更改 ADC引脚 的 DMA3
  ADC_in_DMA_mode_Init_StrType init_str3;  
  init_str3.CHn = DMA_CH3;
  init_str3.SADDR = g_ADC1_mux;
  init_str3.DADDR = (void *)&ADC1_SC1A;
  init_str3.byten = DMA_BYTE1;
  init_str3.soff = sizeof(uint8_t);
  init_str3.doff = 0x0;
  init_str3.count = 2;
  init_str3.slast = -2 * sizeof(uint8_t);
  init_str3.dlast = 0x0;
  init_str3.DMA_source = DMA_Always_EN4;
  
  init_str2.adc_n = ADC1;
  init_str2.adc_ch = ADC1_SE6a; //第一次采集的AD 引脚


  
  // 传入句柄
  ADC_in_DMA_mode_Init(&init_str0);
  ADC_in_DMA_mode_Init(&init_str1);
  ADC_in_DMA_mode_Init(&init_str2);
  ADC_in_DMA_mode_Init(&init_str3);
  
  // 设置DMA 通道间链接
  DMA_set_channel_link(DMA_CH0, DMA_CH1, DMA_minor_link);
  DMA_set_channel_link(DMA_CH0, DMA_CH1, DMA_major_link);    
  DMA_set_channel_link(DMA_CH2, DMA_CH3, DMA_minor_link);
  DMA_set_channel_link(DMA_CH2, DMA_CH3, DMA_major_link);  
  
  // 硬件请求使能 和 中断使能
  DMA_IRQ_EN(DMA_CH0);                                //允许DMA通道传输
  DMA_EN(DMA_CH0);                                    //使能通道CHn 硬件请求  
  //DMA_IRQ_EN(DMA_CH1);                                //允许DMA通道传输
  //DMA_EN(DMA_CH1);                                    //使能通道CHn 硬件请求
  DMA_IRQ_EN(DMA_CH2);                                //允许DMA通道传输
  DMA_EN(DMA_CH2);                                    //使能通道CHn 硬件请求  
  //DMA_IRQ_EN(DMA_CH3);                                //允许DMA通道传输
  //DMA_EN(DMA_CH3);                                    //使能通道CHn 硬件请求
  
  //**ADC_chan_select_for_DMA(ADC0, ADC0_SE8);
  //**ADC_chan_select_for_DMA(ADC1, ADC1_SE6a);
  
  
  LPTMR_for_ADC_trigger(us);
  
  
}



/**
 * @brief       把数据在内存的存储性质从 uint16_t 转化成 type_complex
 * @param        sample：要转换数据类型的数组
 * @return
 * @example
 * @note
 *
 */
void data_type_trans(type_complex sample[])
{
  for (int i = 0; i < _N/2; i++) {
    sample[i].re = *((uint16_t *)(&(sample[i].re)));
    sample[i].im = *((uint16_t *)(&(sample[i].im)));
  }
}


/**
 * @brief       把采集的数据位置重新分配，用于FFT运算
 * @param       sample_s：数据全部存放在此数组
 *              sample_d：相应的数据应该转移到此数组的实部
 * @return
 * @example
 * @note
 *
 */
void data_reprocess(type_complex sample_s[], type_complex sample_d[])
{
  
  for (int i = 0; i < _N/2; i++) {
    sample_d[_N/2-i-1].re = sample_s[i].im;
    sample_d[i].im = 0;
    sample_s[i].im = 0;
  }
  
  for (int i = _N/2; i < _N; i++) {
    sample_s[i].re = 0;
    sample_s[i].im = 0;
    sample_d[i].re = 0;
    sample_d[i].im = 0;
  }
}





/**
 * @brief       初始化ADC转换，由 LPTMR 触发，并把数据通过DMA搬运到内存，然后通过DMA链接功能更换ADC0的引脚
 * @param       init_str: 初始化结构体
 * @return
 * @example
 * @note        源和目的的数据宽度均固定为 2bytes，主循环总次数为2048次，但由于开启通道间链接，count最大只能511，因此需要分成四次
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
  
  ADC_conf_for_DMA(init_str->adc_n, init_str->adc_ch, ADC_16bit);
  
  
}




/**
 * @brief       LPTMR定时触发ADC采集，周期为100us
 * @param       us：触发周期，单位为微秒
 * @return
 * @example
 * @note        LPTMR0_CMR = xus * 50 /32       xus最大为41942us
 *              按道理 CSR[TIE] 和 CSR[TCF] 都置位应产生中断，但实际却进不去中断函数。原因是中断向量表处没有使能
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
    
    DMA_CITER_ELINKYES(CHn) = 0 | DMA_CITER_ELINKYES_CITER(count); //当前主循环次数 
    DMA_BITER_ELINKYES(CHn)  = 0 | DMA_BITER_ELINKYES_BITER(count); //起始主循环次数

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
    //DMA_EN(CHn);                                    //使能通道CHn 硬件请求
    //DMA_IRQ_EN(CHn);                                //允许DMA通道传输
}


/**
 * @brief       链接DMA通道，实现自动更换DMA通道传输
 * @param       to_link_CH：要链接其他通道的通道
 *               be_linked_CH：被链接的通道
 *               link_type：链接类次：主循环结尾链接 或 主循环结尾链接
 * @return
 * @example
 * @note        CITER 和 BITER 应有相同的配置，之前只配置了 CITER,从而寄存器写入错误
 * @date        2020/7/20
 *
 */
static void DMA_set_channel_link(DMA_CHn to_link_CH, DMA_CHn be_linked_CH, link_type_EnumType link_type)
{
  switch (link_type) {
    //次循环结尾链接
    case DMA_minor_link: 
      DMA_CITER_ELINKYES(to_link_CH) |= DMA_CITER_ELINKYES_ELINK(1);             //使能次循环结束的通道间链接
      DMA_BITER_ELINKYES(to_link_CH) |= DMA_BITER_ELINKYES_ELINK(1);
      
      DMA_CITER_ELINKYES(to_link_CH) &= (~(uint16_t)DMA_CITER_ELINKYES_LINKCH_MASK);    //得先清除这个位域
      DMA_CITER_ELINKYES(to_link_CH) |= DMA_CITER_ELINKYES_LINKCH(be_linked_CH);     //选择被链接的通道
      DMA_BITER_ELINKYES(to_link_CH) &= (~(uint16_t)DMA_BITER_ELINKYES_LINKCH_MASK);    //同上
      DMA_BITER_ELINKYES(to_link_CH) |= DMA_BITER_ELINKYES_LINKCH(be_linked_CH);
      break;
    
   //主循环结束链接
    case DMA_major_link:
      DMA_CSR(to_link_CH) |= DMA_CSR_MAJORELINK(1);         //使能主循环结尾的通道间链接
      DMA_CSR(to_link_CH) &= (~(uint16_t)DMA_CSR_MAJORLINKCH(be_linked_CH));
      DMA_CSR(to_link_CH) |= DMA_CSR_MAJORLINKCH(be_linked_CH);      //选择被链接的通道
      break;
      
  }
  
}



/**
 * @brief       ADC初始化配置，由 LPTMR 触发采集，转换完成触发DMA传输
 * @param       adc_n ：  模块名ADC0或ADC1 
 *               adc_ch：  ADC通道编号 
 *                bit   ：  精度选择ADC_8bit，ADC_12bit，ADC_10bit，ADC_16bit
 * @return
 * @example
 * @note        时钟初始化也并入其中，且未启动转换
 * @date        20207/19
 *
 */
static void ADC_conf_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit)
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
    
    ADC1_SC2 = (0 | ADC_SC2_DMAEN_MASK           //开启ADC的DMA 传输功能
                |  ADC_SC2_ADTRG_MASK             //开启硬件触发
               );      
    
    //ADC0_SC3 = 0 | ADC_SC3_ADCO_MASK;           //开启连续转换模式
    
    //写入 SC1A 启动转换
    ADC1_SC1(0) = (0 | ADC_SC1_AIEN_MASK            // 转换完成中断,0为禁止，1为使能       
                      | ADC_SC1_ADCH( adc_ch )       //ADC1 通道选择
                  );
   }
}


/**
 * @brief       ADC转换通道选择，用于第一次转换
 * @param       adc_n ：  模块名ADC0或ADC1 
 *               adc_ch：  ADC通道编号 
 * @return
 * @example
 * @note
 * @date        2020/7/24
 *
 */
static void ADC_chan_select_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch)
{
  if(adc_n == ADC0) { 
    //写入 SC1A 选择引脚
    ADC0_SC1(0) &= (uint16_t)(~((uint16_t)ADC_SC1_ADCH( adc_ch )));       //ADC0 通道清零
    ADC0_SC1(0) |= ADC_SC1_ADCH( adc_ch );       //ADC0 通道选择
  }
  else {
    //写入 SC1A 选择引脚
    ADC1_SC1(0) &= (uint16_t)(~((uint16_t)ADC_SC1_ADCH( adc_ch )));       //ADC0 通道清零
    ADC1_SC1(0) |= ADC_SC1_ADCH( adc_ch );       //ADC1 通道选择
  }
}



