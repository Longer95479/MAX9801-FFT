#ifndef _ADC_AND_DMA_H
#define _ADC_AND_DMA_H


/**
 * @brief       DMA ͨ�������ӵ�����ö��
 */
typedef enum{
  DMA_minor_link,
  DMA_major_link
} link_type_EnumType;


/**
 * @brief       ���幦�ܳ�ʼ���ṹ��
 */
typedef struct{
  
  //DMA��ʼ���ֶ�
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
  
  //ADC��ʼ���ֶ�
  ADC_Type      *adc_n;
  ADCn_Ch_e     adc_ch;
  
  //LPTMR ��ʼ���ֶ�
  uint32_t      us;
  
} ADC_in_DMA_mode_Init_StrType;


/**
 * @brief       ��ʼ��ADCת������ LPTMR ��������������ͨ��DMA���˵��ڴ棬Ȼ��ͨ��DMA���ӹ��ܸ���ADC0������
 * @param       init_str: ��ʼ���ṹ��
 * @return
 * @example
 * @note        Դ��Ŀ�ĵ����ݿ�Ⱦ��̶�Ϊ 2bytes����ѭ�������̶�Ϊ2048��
 *
 */
void ADC_in_DMA_mode_Init(ADC_in_DMA_mode_Init_StrType *init_str);

#endif