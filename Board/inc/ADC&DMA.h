#ifndef _ADC_AND_DMA_H
#define _ADC_AND_DMA_H

/**
 * @brief       DMA目的地址偏移模式：递增或递减
 */
typedef enum{
  UP, DOWN
} DMA_xOFF_mode_EnumType;

/**
 * @brief       初始化结构体
 */
typedef struct{
  
  //DMA初始化字段
  DMA_CHn CHn;
  void *SADDR;
  void *DADDR;
  DMA_xOFF_mode_EnumType xOFF_mode;
  DMA_sources DMA_source;
  
  //ADC初始化字段
  ADC_Type * adc_n;
  ADCn_Ch_e adc_ch;
  
} ADC_in_DMA_mode_Init_StrType;


/**
 * @brief       用于DMA传输的触发
 * @param       us: 触发周期
 * @return
 * @example
 * @note        开启了CH0 1 2 3，对应DMA的 CH0 1 2 3 
 * @date        2020/7/7
 *
 */
void PIT_Init_for_DMA(uint32_t us);



/**
 * @brief       LPTMR定时触发ADC采集，周期为100us
 * @param
 * @return
 * @example
 * @note        LPTMR0_CMR = xus * 50 /32
 * @date        2020/7/19
 *
 */
void LPTMR_for_ADC_trigger(void);

#endif