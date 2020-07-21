#ifndef _ADC_AND_DMA_H
#define _ADC_AND_DMA_H


/**
 * @brief       DMA 通道间链接的类型枚举
 */
typedef enum{
  DMA_minor_link,
  DMA_major_link
} link_type_EnumType;


/**
 * @brief       整体功能初始化结构体
 */
typedef struct{
  
  //DMA初始化字段
  DMA_CHn       CHn;
  void          *SADDR;
  void          *DADDR;
  DMA_BYTEn      byten;
  int16_t       soff;
  int16_t       doff;
  uint16_t      count;
  int32_t       slast;
  int32_t       dlast;
  DMA_sources   DMA_source;
  
  //ADC初始化字段
  ADC_Type      *adc_n;
  ADCn_Ch_e     adc_ch;
  
  //LPTMR 初始化字段
  uint32_t      us;
  
} ADC_in_DMA_mode_Init_StrType;


/**
 * @brief       初始化ADC转换，由 LPTMR 触发，并把数据通过DMA搬运到内存，然后通过DMA链接功能更换ADC0的引脚
 * @param       init_str: 初始化结构体
 * @return
 * @example
 * @note        源和目的的数据宽度均固定为 2bytes，主循环次数固定为2048次
 *
 */
void ADC_in_DMA_mode_Init(ADC_in_DMA_mode_Init_StrType *init_str);

#endif