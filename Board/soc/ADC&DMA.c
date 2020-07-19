/**
 * @Platform    ����K66���İ�        IAR 8.32.1
 * @Fielname    ADC&DMA.c
 * @brief       ADC����ת��������DMA ���䵽�ڴ棬�����������ͽ���ת������غ���
 * @Author      Longer95479
 * @Email       371573369@qq.com
 * @Date        2020/7/17
 *
 */

#include "include.h"

/**
 * @brief �ڲ�ʹ�ú�������
 */
static void DMA_Init_for_ADC(DMA_CHn CHn, void *SADDR, void *DADDR, DMA_BYTEn byten, uint32_t count, DMA_xOFF_mode_EnumType xOFF_mode, DMA_sources);
static void ADC_Start_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit);



/**
 * @brief       ��ʼ��ADCת������ LPTMR ��������������ͨ��DMA���˵��ڴ棬Ȼ��ͨ��DMA���ӹ��ܸ���ADC0������
 * @param       init_str: ��ʼ���ṹ��
 * @return
 * @example
 * @note        Դ��Ŀ�ĵ����ݿ�Ⱦ��̶�Ϊ 2bytes����ѭ�������̶�Ϊ2048��
 *
 */
void ADC_in_DMA_mode_Init(ADC_in_DMA_mode_Init_StrType *init_str)
{ 
  DMA_Init_for_ADC(init_str->CHn, init_str->SADDR, init_str->DADDR, DMA_BYTE2, _N/2, init_str->xOFF_mode, init_str->DMA_source);
  ADC_Start_for_DMA(init_str->adc_n, init_str->adc_ch, ADC_16bit);
}




/**
 * @brief       LPTMR��ʱ����ADC�ɼ�������Ϊ100us
 * @param
 * @return
 * @example
 * @note        LPTMR0_CMR = xus * 50 /32
 * @date        2020/7/19
 *
 */
void LPTMR_for_ADC_trigger(void)
{


    OSC_CR |= OSC_CR_ERCLKEN_MASK;                              //ʹ�� OSCERCLK

    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;                          //ʹ��LPTģ��ʱ��

    LPTMR0_CSR = 0x00;                                          //�ȹ���LPT���Զ����������ֵ

    LPTMR0_CMR = 15625;                                            //���ñȽ�ֵ

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
    
    SIM->SOPT7 = 0x8e8eU;
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
void PIT_Init_for_DMA(uint32_t us)
{
  //PIT �õ��� Bus Clock ����Ƶ��

    /* ����ʱ��*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                          

    /* ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼������� */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );   
    
    /* ���ô������� */
    PIT_LDVAL(PIT0)  = us * bus_clk;     
    PIT_LDVAL(PIT1)  = us * bus_clk;     
    PIT_LDVAL(PIT2)  = us * bus_clk;     
    PIT_LDVAL(PIT3)  = us * bus_clk;  
    
    /* ʹ�� PITn��ʱ�� */
    PIT_TCTRL(PIT0) |= PIT_TCTRL_TEN_MASK;   
    PIT_TCTRL(PIT1) |= PIT_TCTRL_TEN_MASK;   
    PIT_TCTRL(PIT2) |= PIT_TCTRL_TEN_MASK;   
    PIT_TCTRL(PIT3) |= PIT_TCTRL_TEN_MASK;   
}



   
/**
 * @brief       DMA��ʼ������ȡADC���ݵ��ڴ�
 * @param       DMA_CHn: ͨ���ţ�DMA_CH0 ~ DMA_CH15��
 *               SADDR: Դ��ַ                                      ADC�Ĵ���
 *               DADDR: Ŀ�ĵ�ַ
 *               DMA_BYTEn: ÿ��DMA�����ֽ���                  1(2 bytes)
 *               count: һ����ѭ�������ֽ���                     2048
 *               xOFF_mode : ÿ�δ�����ַ�ǵ������ǵݼ���ģʽȷ��
 *              DMA_source��DMA����Ĵ���Դ
 * @return
 * @example
 * @note        ��ѭ��һ�δ���16bit(NBYTES = 2)��Դ���ݺ�Ŀ�����ݿ�ȶ���16bit��DOFF = 8 BYTES, Ҫ����2048����ѭ��
 * 
 * @date 2020/7/16
 *
 */                             
static void DMA_Init_for_ADC(DMA_CHn CHn, void *SADDR, void *DADDR, DMA_BYTEn byten, uint32_t count, DMA_xOFF_mode_EnumType xOFF_mode, DMA_sources DMA_source)
{

    uint8_t BYTEs = (byten == DMA_BYTE1 ? 1 : (byten == DMA_BYTE2 ? 2 : (byten == DMA_BYTE4 ? 4 : 16 ) ) ); //���㴫���ֽ���

    /* ����ʱ�� */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //��DMAģ��ʱ��
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK; 
	
    /* ���� DMA ͨ�� �� ������ƿ� TCD ( Transfer Control Descriptor ) */
    DMA_SADDR(CHn) =    (uint32_t)SADDR;                         // ����  Դ��ַ
    DMA_DADDR(CHn) =    (uint32_t)DADDR;                         // ����Ŀ�ĵ�ַ
    DMA_SOFF(CHn)  =    0x00u;                              // ����Դ��ַƫ�� = 0x0, ������
    DMA_DOFF(CHn)  =    (xOFF_mode == UP) ? (4 * BYTEs) : (-4 * BYTEs);                              // ÿ�δ����Ŀ�ĵ�ַ�� 4 * BYTEs  = 8 bytes

    DMA_ATTR(CHn)  =    (0
                         | DMA_ATTR_SMOD(0x0)                // Դ��ַģ����ֹ  Source address modulo feature is disabled
                         | DMA_ATTR_SSIZE(byten)             // Դ����λ�� ��DMA_BYTEn  ��    SSIZE = 0 -> 8-bit ��SSIZE = 1 -> 16-bit ��SSIZE = 2 -> 32-bit ��SSIZE = 4 -> 16-byte
                         | DMA_ATTR_DMOD(0x0)                // Ŀ���ַģ����ֹ
                         | DMA_ATTR_DSIZE(byten)             // Ŀ������λ�� ��DMA_BYTEn  ��  ���òο�  SSIZE
                        );

    DMA_CITER_ELINKNO(CHn)  = DMA_CITER_ELINKNO_CITER(count); //��ǰ��ѭ������ 
    DMA_BITER_ELINKNO(CHn)  = DMA_BITER_ELINKNO_BITER(count); //��ʼ��ѭ������

    DMA_CR &= ~DMA_CR_EMLM_MASK;                            // CR[EMLM] = 0

    //��CR[EMLM] = 0 ʱ:
    DMA_NBYTES_MLNO(CHn) =   DMA_NBYTES_MLNO_NBYTES(BYTEs); // ͨ��ÿ�δ����ֽ�������������ΪBYTEs���ֽڡ�ע��ֵΪ0��ʾ����4GB */


    /* ���� DMA ���������Ĳ��� */
    DMA_SLAST(CHn)      =   0;                              //����  Դ��ַ�ĸ���ֵ,��ѭ��������ָ�  Դ��ַ
    DMA_DLAST_SGA(CHn)  =   (xOFF_mode == UP) ? (-count * 8) : (count * 8); //����Ŀ�ĵ�ַ�ĸ���ֵ,��ѭ��������ָ�Ŀ�ĵ�ַ���߱��ֵ�ַ
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
    DMA_EN(CHn);                                    //ʹ��ͨ��CHn Ӳ������
    DMA_IRQ_EN(CHn);                                //����DMAͨ������
}


/**
 * @brief       ADC��ʼ������ LPTMR �����ɼ���ת����ɴ���DMA����
 * @param       adc_n ��  ģ����ADC0��ADC1 
 *               adc_ch��  ADCͨ����� 
 *                bit   ��  ����ѡ��ADC_8bit��ADC_12bit��ADC_10bit��ADC_16bit
 * @return
 * @example
 * @note        ʱ�ӳ�ʼ��Ҳ��������
 * @date        20207/19
 *
 */
static void ADC_Start_for_DMA(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit)
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
                     //| ADC_SC1_ADCH( 0x0c )       
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
    
    ADC0_SC2 = (0 | ADC_SC2_DMAEN_MASK           //����ADC��DMA ���书��
                |  ADC_SC2_ADTRG_MASK             //����Ӳ������
               );      
    
    //ADC0_SC3 = 0 | ADC_SC3_ADCO_MASK;           //��������ת��ģʽ
    
    //д�� SC1A ����ת��
    ADC1_SC1(0) = (0 | ADC_SC1_AIEN_MASK            // ת������ж�,0Ϊ��ֹ��1Ϊʹ��
                     //| ADC_SC1_ADCH( 0x0c )       
                     | ADC_SC1_ADCH( adc_ch )       //ADC1 ͨ��ѡ��
                  );
   }
}

