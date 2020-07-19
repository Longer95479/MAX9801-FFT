#ifndef _ADC_AND_DMA_H
#define _ADC_AND_DMA_H

/**
 * @brief       DMAĿ�ĵ�ַƫ��ģʽ��������ݼ�
 */
typedef enum{
  UP, DOWN
} DMA_xOFF_mode_EnumType;

/**
 * @brief       ��ʼ���ṹ��
 */
typedef struct{
  
  //DMA��ʼ���ֶ�
  DMA_CHn CHn;
  void *SADDR;
  void *DADDR;
  DMA_xOFF_mode_EnumType xOFF_mode;
  DMA_sources DMA_source;
  
  //ADC��ʼ���ֶ�
  ADC_Type * adc_n;
  ADCn_Ch_e adc_ch;
  
} ADC_in_DMA_mode_Init_StrType;


/**
 * @brief       ����DMA����Ĵ���
 * @param       us: ��������
 * @return
 * @example
 * @note        ������CH0 1 2 3����ӦDMA�� CH0 1 2 3 
 * @date        2020/7/7
 *
 */
void PIT_Init_for_DMA(uint32_t us);



/**
 * @brief       LPTMR��ʱ����ADC�ɼ�������Ϊ100us
 * @param
 * @return
 * @example
 * @note        LPTMR0_CMR = xus * 50 /32
 * @date        2020/7/19
 *
 */
void LPTMR_for_ADC_trigger(void);

#endif