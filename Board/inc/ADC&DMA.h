#ifndef _ADC_AND_DMA_H
#define _ADC_AND_DMA_H


/**
 * @brief       开启通道间链接的情况下主循环的最大次数
 */
#define COUNT_MAX       511


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


/**
 * @brief       ADC_in_DMA_mode 的实例配置
 * @param       us：LPTMR 触发 ADC 采集的周期
 * @return
 * @example
 * @note
 * @date        2020/7/22
 *
 */
void ADC_in_DMA_mode_instance_conf(uint32_t us);


/**
 * @brief       把数据在内存的存储性质从 uint16_t 转化成 type_complex
 * @param        sample：要转换数据类型的数组
 * @return
 * @example
 * @note
 *
 */
void data_type_trans(type_complex sample[]);


/**
 * @brief       把采集的数据位置重新分配，用于FFT运算
 * @param       sample_s：数据全部存放在此数组
 *              sample_d：相应的数据应该转移到此数组的实部
 * @return
 * @example
 * @note
 *
 */
void data_reprocess(type_complex sample_s[], type_complex sample_d[]);


/**
 * @brief       把采集的数据位置重新分配，用于FFT运算
 * @param       sample_s：数据全部存放在此数组
 *              sample_d：相应的数据应该转移到此数组的实部
 * @return
 * @example
 * @note        本次转换没有对移动的数据进行翻转
 *
 */
void data_reprocess_no_overturn(type_complex sample_s[], type_complex sample_d[]);


#endif