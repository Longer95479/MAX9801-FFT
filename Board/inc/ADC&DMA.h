#ifndef _ADC_AND_DMA_H
#define _ADC_AND_DMA_H


/**
 * @brief       ����ͨ�������ӵ��������ѭ����������
 */
#define COUNT_MAX       511


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


/**
 * @brief       ADC_in_DMA_mode ��ʵ������
 * @param       us��LPTMR ���� ADC �ɼ�������
 * @return
 * @example
 * @note
 * @date        2020/7/22
 *
 */
void ADC_in_DMA_mode_instance_conf(uint32_t us);


/**
 * @brief       ���������ڴ�Ĵ洢���ʴ� uint16_t ת���� type_complex
 * @param        sample��Ҫת���������͵�����
 * @return
 * @example
 * @note
 *
 */
void data_type_trans(type_complex sample[]);


/**
 * @brief       �Ѳɼ�������λ�����·��䣬����FFT����
 * @param       sample_s������ȫ������ڴ�����
 *              sample_d����Ӧ������Ӧ��ת�Ƶ��������ʵ��
 * @return
 * @example
 * @note
 *
 */
void data_reprocess(type_complex sample_s[], type_complex sample_d[]);


/**
 * @brief       �Ѳɼ�������λ�����·��䣬����FFT����
 * @param       sample_s������ȫ������ڴ�����
 *              sample_d����Ӧ������Ӧ��ת�Ƶ��������ʵ��
 * @return
 * @example
 * @note        ����ת��û�ж��ƶ������ݽ��з�ת
 *
 */
void data_reprocess_no_overturn(type_complex sample_s[], type_complex sample_d[]);


#endif