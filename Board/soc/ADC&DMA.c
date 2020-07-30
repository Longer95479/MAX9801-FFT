/**
 * @Platform    ����K66���İ�        IAR 8.32.1
 * @Fielname    ADC&DMA.c
 * @brief       ADC�� LPTMR ����������DMA ���䵽�ڴ棬�����������ͽ���ת������غ���
 * @Author      Longer95479
 * @Email       371573369@qq.com
 * @Date        2020/7/17
 *
 */


/**
 * @brief       ����洢AD���ű�ŵ�ȫ�ֱ��������� DMA �Զ��������ţ�ͨ��д�� ADC_SC1A[ADCH] ��ʵ�֣�
 */
#define ADCDMA_GLOBALS   
   
#include "include.h"
   
/**
 * @brief       ADC ���Ÿ��õĴ洢���飬���ڱ� DMA ����
 */ 
uint8_t g_ADC0_mux[2] = {ADC0_SE8, ADC0_SE9};
uint8_t g_ADC1_mux[2] = {ADC1_SE6a, ADC1_SE7a};
   

/**
 * @brief �ڲ�ʹ�ú�������
 */
static void DMA_Init_for_ADC(DMA_CHn CHn, void *SADDR, void *DADDR, DMA_BYTEn byten, int16_t soff, int16_t doff, uint16_t count, int32_t slast, int32_t dlast, DMA_sources DMA_source);
static void ADC_conf_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit);
static void ADC_chan_select_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch);
static void LPTMR_for_ADC_trigger(uint32_t us);
static void DMA_set_channel_link(DMA_CHn to_link_CH, DMA_CHn be_linked_CH, link_type_EnumType link_type);



/**
 * @brief       ADC_in_DMA_mode ��ʵ������
 * @param
 * @return
 * @example
 * @note
 * @date        2020/7/22
 *
 */
void ADC_in_DMA_mode_instance_conf(uint32_t us)
{
  // init_str0 ���������ڴ��� ADC0 ���ݵ� DMA0
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
  
  
  // init_str1 ���������ڸ��� ADC���� �� DMA1
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
  
  
  // init_str2 ���������ڴ��� ADC1 ���ݵ� DMA2
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
  init_str2.adc_ch = ADC1_SE6a; //��һ�βɼ���AD ����
  
  
  // init_str3 ���������ڸ��� ADC���� �� DMA3
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
  init_str2.adc_ch = ADC1_SE6a; //��һ�βɼ���AD ����


  
  // ������
  ADC_in_DMA_mode_Init(&init_str0);
  ADC_in_DMA_mode_Init(&init_str1);
  ADC_in_DMA_mode_Init(&init_str2);
  ADC_in_DMA_mode_Init(&init_str3);
  
  // ����DMA ͨ��������
  DMA_set_channel_link(DMA_CH0, DMA_CH1, DMA_minor_link);
  DMA_set_channel_link(DMA_CH0, DMA_CH1, DMA_major_link);    
  DMA_set_channel_link(DMA_CH2, DMA_CH3, DMA_minor_link);
  DMA_set_channel_link(DMA_CH2, DMA_CH3, DMA_major_link);  
  
  // Ӳ������ʹ�� �� �ж�ʹ��
  DMA_IRQ_EN(DMA_CH0);                                //����DMAͨ������
  DMA_EN(DMA_CH0);                                    //ʹ��ͨ��CHn Ӳ������  
  //DMA_IRQ_EN(DMA_CH1);                                //����DMAͨ������
  //DMA_EN(DMA_CH1);                                    //ʹ��ͨ��CHn Ӳ������
  DMA_IRQ_EN(DMA_CH2);                                //����DMAͨ������
  DMA_EN(DMA_CH2);                                    //ʹ��ͨ��CHn Ӳ������  
  //DMA_IRQ_EN(DMA_CH3);                                //����DMAͨ������
  //DMA_EN(DMA_CH3);                                    //ʹ��ͨ��CHn Ӳ������
  
  //**ADC_chan_select_for_DMA(ADC0, ADC0_SE8);
  //**ADC_chan_select_for_DMA(ADC1, ADC1_SE6a);
  
  
  LPTMR_for_ADC_trigger(us);
  
  
}



/**
 * @brief       ���������ڴ�Ĵ洢���ʴ� uint16_t ת���� type_complex
 * @param        sample��Ҫת���������͵�����
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
 * @brief       �Ѳɼ�������λ�����·��䣬����FFT����
 * @param       sample_s������ȫ������ڴ�����
 *              sample_d����Ӧ������Ӧ��ת�Ƶ��������ʵ��
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
 * @brief       ��ʼ��ADCת������ LPTMR ��������������ͨ��DMA���˵��ڴ棬Ȼ��ͨ��DMA���ӹ��ܸ���ADC0������
 * @param       init_str: ��ʼ���ṹ��
 * @return
 * @example
 * @note        Դ��Ŀ�ĵ����ݿ�Ⱦ��̶�Ϊ 2bytes����ѭ���ܴ���Ϊ2048�Σ������ڿ���ͨ�������ӣ�count���ֻ��511�������Ҫ�ֳ��Ĵ�
 *
 */
void ADC_in_DMA_mode_Init(ADC_in_DMA_mode_Init_StrType *init_str)
{ 
  DMA_Init_for_ADC(init_str->CHn,                //ѡ��DMAͨ��
                   init_str->SADDR,             //ѡ��Դ��ַ
                   init_str->DADDR,             //ѡ��Ŀ�ĵ�ַ
                   init_str->byten,             //Դ���ݺ�Ŀ������λ��
                   init_str->soff,              //Դ��ַƫ��
                   init_str->doff,              //Ŀ�ĵ�ַƫ��
                   init_str->count,             //��ѭ������
                   init_str->slast,             //��ѭ��������Դ��ַƫ��
                   init_str->dlast,             //��ѭ��������Ŀ�ĵ�ַƫ��
                   init_str->DMA_source);       //ѡ��DMA����Դ
  
  ADC_conf_for_DMA(init_str->adc_n, init_str->adc_ch, ADC_16bit);
  
  
}




/**
 * @brief       LPTMR��ʱ����ADC�ɼ�������Ϊ100us
 * @param       us���������ڣ���λΪ΢��
 * @return
 * @example
 * @note        LPTMR0_CMR = xus * 50 /32       xus���Ϊ41942us
 *              ������ CSR[TIE] �� CSR[TCF] ����λӦ�����жϣ���ʵ��ȴ����ȥ�жϺ�����ԭ�����ж�������û��ʹ��
 * @date        2020/7/19
 *
 */
static void LPTMR_for_ADC_trigger(uint32_t us)
{


    OSC_CR |= OSC_CR_ERCLKEN_MASK;                              //ʹ�� OSCERCLK

    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;                          //ʹ��LPTģ��ʱ��

    LPTMR0_CSR = 0x00;                                          //�ȹ���LPT���Զ����������ֵ

    LPTMR0_CMR = ((us < 41943) ? us : 41942) * 50 / 32;       //���ñȽ�ֵ

    //ѡ��ʱ��Դ
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(3)          //ѡ��ʱ��Դ�� 0 Ϊ MCGIRCLK ��1Ϊ LPO��1KHz�� ��2Ϊ ERCLK32K ��3Ϊ OSCERCLK
                      //| LPTMR_PSR_PBYP_MASK     //��· Ԥ��Ƶ/�����˲��� ,������ Ԥ��Ƶ/�����˲���(ע���˱�ʾʹ��Ԥ��Ƶ/�����˲���)
                      | LPTMR_PSR_PRESCALE(4)     //Ԥ��Ƶֵ = 2^(n+1) ,n = 0~ 0xF
                    );

    //ʹ�� LPT
    LPTMR0_CSR  =  (0
                    //| LPTMR_CSR_TPS(1)        // ѡ������ܽ� ѡ��
                    //| LPTMR_CSR_TMS_MASK      // ѡ��������� (ע���˱�ʾʱ�����ģʽ)
                    //| ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //���������������ʽѡ��0Ϊ�ߵ�ƽ��Ч�������ؼ�1
                    | LPTMR_CSR_TEN_MASK        //ʹ��LPT(ע���˱�ʾ����)
                    | LPTMR_CSR_TIE_MASK      //�ж�ʹ��
                    //| LPTMR_CSR_TFC_MASK      //0:����ֵ���ڱȽ�ֵ�͸�λ��1�������λ��ע�ͱ�ʾ0��
                   );
    
    SIM->SOPT7 = 0x8e8eU;       //���� LPTMR ��Ϊ ADC �Ĵ����ź�
}



/**
 * @brief       ����DMA����Ĵ���
 * @param       us: ��������
 * @return
 * @example
 * @note        ������CH0 1 2 3����ӦDMA�� CH0 1 2 3 
 * @date        2020/7/7
 *
 */
//void PIT_Init_for_DMA(uint32_t us)
//{
  //PIT �õ��� Bus Clock ����Ƶ��

    /* ����ʱ��*/
//    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                          

    /* ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼������� */
//    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );   
    
    /* ���ô������� */
//    PIT_LDVAL(PIT0)  = us * bus_clk;     
//    PIT_LDVAL(PIT1)  = us * bus_clk;     
//    PIT_LDVAL(PIT2)  = us * bus_clk;     
//    PIT_LDVAL(PIT3)  = us * bus_clk;  
    
    /* ʹ�� PITn��ʱ�� */
//    PIT_TCTRL(PIT0) |= PIT_TCTRL_TEN_MASK;   
//    PIT_TCTRL(PIT1) |= PIT_TCTRL_TEN_MASK;   
//    PIT_TCTRL(PIT2) |= PIT_TCTRL_TEN_MASK;   
//    PIT_TCTRL(PIT3) |= PIT_TCTRL_TEN_MASK;   
//}



   
/**
 * @brief       DMA��ʼ������ȡADC���ݵ��ڴ�
 * @param       CHn: ͨ���ţ�DMA_CH0 ~ DMA_CH15��
 *               SADDR: Դ��ַ                                      
 *               DADDR: Ŀ�ĵ�ַ
 *               byten: Դ��ַ��Ŀ�ĵ�ַ�����ݿ��                 1(2 bytes)
 *               soff: Դ��ַƫ��
 *               doff��Ŀ���ַƫ��
 *               count: ��ѭ������                     2048������������ͨ�������ӵĹ��ܣ����ֻ��Ϊ511��,�����Ҫ�ֳ��Ĵν��У��м����жϸ���Դ��ַ��
 *               slast����ѭ��������Դ��ַ��ƫ��
 *               dlast����ѭ��������Ŀ���ַ��ƫ��
 *               DMA_source��DMA����Ĵ���Դ
 * @return
 * @example
 * @note        ��ѭ��һ�δ���16bit(NBYTES = 2)��Դ���ݺ�Ŀ�����ݿ�ȶ���16bit��DOFF = 8 BYTES, Ҫ����2048����ѭ��
 * 
 * @date 2020/7/16
 *
 */                             
static void DMA_Init_for_ADC(DMA_CHn CHn, void *SADDR, void *DADDR, DMA_BYTEn byten, int16_t soff, int16_t doff, uint16_t count, int32_t slast, int32_t dlast, DMA_sources DMA_source)
{

    uint8_t BYTEs = (byten == DMA_BYTE1 ? 1 : (byten == DMA_BYTE2 ? 2 : (byten == DMA_BYTE4 ? 4 : 16 ) ) ); //���㴫���ֽ���

    /* ����ʱ�� */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //��DMAģ��ʱ��
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK; 
	
    /* ���� DMA ͨ�� �� ������ƿ� TCD ( Transfer Control Descriptor ) */
    DMA_SADDR(CHn) =    (uint32_t)SADDR;                         // ����  Դ��ַ
    DMA_DADDR(CHn) =    (uint32_t)DADDR;                         // ����Ŀ�ĵ�ַ
    DMA_SOFF(CHn)  =    soff;                              // ����Դ��ַƫ�� 
    DMA_DOFF(CHn)  =    doff;                            // Ŀ�ĵ�ַƫ��

    DMA_ATTR(CHn)  =    (0
                         | DMA_ATTR_SMOD(0x0)                // Դ��ַģ����ֹ  Source address modulo feature is disabled
                         | DMA_ATTR_SSIZE(byten)             // Դ����λ�� ��DMA_BYTEn  ��    SSIZE = 0 -> 8-bit ��SSIZE = 1 -> 16-bit ��SSIZE = 2 -> 32-bit ��SSIZE = 4 -> 16-byte
                         | DMA_ATTR_DMOD(0x0)                // Ŀ���ַģ����ֹ
                         | DMA_ATTR_DSIZE(byten)             // Ŀ������λ�� ��DMA_BYTEn  ��  ���òο�  SSIZE
                        );

    //**DMA_CITER_ELINKNO(CHn)  = DMA_CITER_ELINKNO_CITER(count); //��ǰ��ѭ������ 
    //**DMA_BITER_ELINKNO(CHn)  = DMA_BITER_ELINKNO_BITER(count); //��ʼ��ѭ������
    
    DMA_CITER_ELINKYES(CHn) = 0 | DMA_CITER_ELINKYES_CITER(count); //��ǰ��ѭ������ 
    DMA_BITER_ELINKYES(CHn)  = 0 | DMA_BITER_ELINKYES_BITER(count); //��ʼ��ѭ������

    DMA_CR &= ~DMA_CR_EMLM_MASK;                            // CR[EMLM] = 0

    //��CR[EMLM] = 0 ʱ:
    DMA_NBYTES_MLNO(CHn) =   DMA_NBYTES_MLNO_NBYTES(BYTEs); // ͨ��ÿ�δ����ֽ�������������ΪBYTEs���ֽڡ�ע��ֵΪ0��ʾ����4GB */


    /* ���� DMA ���������Ĳ��� */
    DMA_SLAST(CHn)      =   slast;                              //����  Դ��ַ�ĸ���ֵ,��ѭ��������ָ�  Դ��ַ
    DMA_DLAST_SGA(CHn)  =   dlast;/*(xOFF_mode == UP) ? (-count * 8) : (count * 8)*/ //����Ŀ�ĵ�ַ�ĸ���ֵ,��ѭ��������ָ�Ŀ�ĵ�ַ���߱��ֵ�ַ
    DMA_CSR(CHn)        =   (0
                             | DMA_CSR_BWC(3)               //�������,ÿ��һ�Σ�eDMA ����ֹͣ 8 �����ڣ�0��ֹͣ��1������2ֹͣ4���ڣ�3ֹͣ8���ڣ�
                             | DMA_CSR_DREQ_MASK            //��ѭ��������ֹͣӲ������
                             | DMA_CSR_INTMAJOR_MASK        //��ѭ������������ж�
                            );

    /* ���� DMA ����Դ */
    DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, CHn) = (0
            | DMAMUX_CHCFG_ENBL_MASK                        /* Enable routing of DMA request */
            //| DMAMUX_CHCFG_TRIG_MASK                        /* Trigger Mode: Periodic   PIT���ڴ�������ģʽ   ͨ��1��ӦPIT1������ʹ��PIT1����������Ӧ��PIT��ʱ���� */
            | DMAMUX_CHCFG_SOURCE(DMA_source)              ); /* ͨ����������Դѡ�񣬾����ſ���ͷ�ļ���鿴*/



    /* �����ж� */
    //DMA_EN(CHn);                                    //ʹ��ͨ��CHn Ӳ������
    //DMA_IRQ_EN(CHn);                                //����DMAͨ������
}


/**
 * @brief       ����DMAͨ����ʵ���Զ�����DMAͨ������
 * @param       to_link_CH��Ҫ��������ͨ����ͨ��
 *               be_linked_CH�������ӵ�ͨ��
 *               link_type��������Σ���ѭ����β���� �� ��ѭ����β����
 * @return
 * @example
 * @note        CITER �� BITER Ӧ����ͬ�����ã�֮ǰֻ������ CITER,�Ӷ��Ĵ���д�����
 * @date        2020/7/20
 *
 */
static void DMA_set_channel_link(DMA_CHn to_link_CH, DMA_CHn be_linked_CH, link_type_EnumType link_type)
{
  switch (link_type) {
    //��ѭ����β����
    case DMA_minor_link: 
      DMA_CITER_ELINKYES(to_link_CH) |= DMA_CITER_ELINKYES_ELINK(1);             //ʹ�ܴ�ѭ��������ͨ��������
      DMA_BITER_ELINKYES(to_link_CH) |= DMA_BITER_ELINKYES_ELINK(1);
      
      DMA_CITER_ELINKYES(to_link_CH) &= (~(uint16_t)DMA_CITER_ELINKYES_LINKCH_MASK);    //����������λ��
      DMA_CITER_ELINKYES(to_link_CH) |= DMA_CITER_ELINKYES_LINKCH(be_linked_CH);     //ѡ�����ӵ�ͨ��
      DMA_BITER_ELINKYES(to_link_CH) &= (~(uint16_t)DMA_BITER_ELINKYES_LINKCH_MASK);    //ͬ��
      DMA_BITER_ELINKYES(to_link_CH) |= DMA_BITER_ELINKYES_LINKCH(be_linked_CH);
      break;
    
   //��ѭ����������
    case DMA_major_link:
      DMA_CSR(to_link_CH) |= DMA_CSR_MAJORELINK(1);         //ʹ����ѭ����β��ͨ��������
      DMA_CSR(to_link_CH) &= (~(uint16_t)DMA_CSR_MAJORLINKCH(be_linked_CH));
      DMA_CSR(to_link_CH) |= DMA_CSR_MAJORLINKCH(be_linked_CH);      //ѡ�����ӵ�ͨ��
      break;
      
  }
  
}



/**
 * @brief       ADC��ʼ�����ã��� LPTMR �����ɼ���ת����ɴ���DMA����
 * @param       adc_n ��  ģ����ADC0��ADC1 
 *               adc_ch��  ADCͨ����� 
 *                bit   ��  ����ѡ��ADC_8bit��ADC_12bit��ADC_10bit��ADC_16bit
 * @return
 * @example
 * @note        ʱ�ӳ�ʼ��Ҳ�������У���δ����ת��
 * @date        20207/19
 *
 */
static void ADC_conf_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit)
{
  
   ADC_Init(adc_n);
   
   if(adc_n==ADC0)
   {
     ADC0_CFG1 = (0  | ADC_CFG1_ADIV(2)              //ʱ�ӷ�Ƶѡ��,��Ƶϵ��Ϊ 2^n,2bit
                     | ADC_CFG1_ADLSMP_MASK          //����ʱ�����ã�0Ϊ�̲���ʱ�䣬1 Ϊ������ʱ��
                     | ADC_CFG1_MODE(bit)            //��ȷ��ѡ��
                     | ADC_CFG1_ADICLK(0)            //0Ϊ����ʱ��,1Ϊ����ʱ��/2,2Ϊ����ʱ�ӣ�ALTCLK����3Ϊ �첽ʱ�ӣ�ADACK����
                 );


    ADC0_CFG2  = (0  | ADC_CFG2_ADHSC_MASK          //��������,0Ϊ����ת�����У�1Ϊ����ת������
                     | ADC_CFG2_ADLSTS(0)           //������ʱ��ѡ��ADCKΪ4+n������ѭ��������ѭ����0Ϊ20��1Ϊ12��2Ϊ6��3Ϊ2
                  );
    
    ADC0_SC2 = (0 | ADC_SC2_DMAEN_MASK              //����ADC��DMA ���书��
                  |  ADC_SC2_ADTRG_MASK             //����Ӳ������
               );          
    
    //ADC0_SC3 = 0 | ADC_SC3_ADCO_MASK;           //��������ת��ģʽ

    //д�� SC1A ����ת��
    ADC0_SC1(0) = (0 | ADC_SC1_AIEN_MASK            // ת������ж�,0Ϊ��ֹ��1Ϊʹ��
                      | ADC_SC1_ADCH( adc_ch )       //ADC0 ͨ��ѡ��
                   );
   }
   else 
   {
     ADC1_CFG1 = (0   | ADC_CFG1_ADIV(2)              //ʱ�ӷ�Ƶѡ��,��Ƶϵ��Ϊ 2^n,2bit
                     | ADC_CFG1_ADLSMP_MASK           //����ʱ�����ã�0Ϊ�̲���ʱ�䣬1 Ϊ������ʱ��
                     | ADC_CFG1_MODE(bit)             //��ȷ��ѡ��
                     | ADC_CFG1_ADICLK(0)             //0Ϊ����ʱ��,1Ϊ����ʱ��/2,2Ϊ����ʱ�ӣ�ALTCLK����3Ϊ �첽ʱ�ӣ�ADACK����
                 );


    ADC1_CFG2  = (0  | ADC_CFG2_ADHSC_MASK          //��������,0Ϊ����ת�����У�1Ϊ����ת������
                     | ADC_CFG2_ADLSTS(0)           //������ʱ��ѡ��ADCKΪ4+n������ѭ��������ѭ����0Ϊ20��1Ϊ12��2Ϊ6��3Ϊ2
                  );
    
    ADC1_SC2 = (0 | ADC_SC2_DMAEN_MASK           //����ADC��DMA ���书��
                |  ADC_SC2_ADTRG_MASK             //����Ӳ������
               );      
    
    //ADC0_SC3 = 0 | ADC_SC3_ADCO_MASK;           //��������ת��ģʽ
    
    //д�� SC1A ����ת��
    ADC1_SC1(0) = (0 | ADC_SC1_AIEN_MASK            // ת������ж�,0Ϊ��ֹ��1Ϊʹ��       
                      | ADC_SC1_ADCH( adc_ch )       //ADC1 ͨ��ѡ��
                  );
   }
}


/**
 * @brief       ADCת��ͨ��ѡ�����ڵ�һ��ת��
 * @param       adc_n ��  ģ����ADC0��ADC1 
 *               adc_ch��  ADCͨ����� 
 * @return
 * @example
 * @note
 * @date        2020/7/24
 *
 */
static void ADC_chan_select_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch)
{
  if(adc_n == ADC0) { 
    //д�� SC1A ѡ������
    ADC0_SC1(0) &= (uint16_t)(~((uint16_t)ADC_SC1_ADCH( adc_ch )));       //ADC0 ͨ������
    ADC0_SC1(0) |= ADC_SC1_ADCH( adc_ch );       //ADC0 ͨ��ѡ��
  }
  else {
    //д�� SC1A ѡ������
    ADC1_SC1(0) &= (uint16_t)(~((uint16_t)ADC_SC1_ADCH( adc_ch )));       //ADC0 ͨ������
    ADC1_SC1(0) |= ADC_SC1_ADCH( adc_ch );       //ADC1 ͨ��ѡ��
  }
}



