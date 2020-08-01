#include "include.h"
#include "LG_EncTPM.h"

//@brief ����TPM12���������빦��
void Enc_TPM12_Init(void)
{
  //���ų�ʼ��
  /* Port A Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortA);
  
  /* PORTA8 is configured as TPM1_CH0 */
  PORT_SetPinMux(PORTA, 8U, kPORT_MuxAlt6);
  /* PORTA9 is configured as TPM1_CH1 */
  PORT_SetPinMux(PORTA, 9U, kPORT_MuxAlt6);
  /* PORTA10 is configured as TPM2_CH0 */
  PORT_SetPinMux(PORTA, 10U, kPORT_MuxAlt6);
  /* PORTA11 is configured as TPM2_CH1 */
  PORT_SetPinMux(PORTA, 11U, kPORT_MuxAlt6);
  
  //ʹ��ʱ��
  CLOCK_EnableClock(kCLOCK_Tpm1);
  CLOCK_EnableClock(kCLOCK_Tpm2);
  
  //ʹ�� MCGPLLCLK
  MCG_C5 |= MCG_C5_PLLCLKEN(1U);
  
  //ѡ�� MCGPLLCLK Ϊ TPM ������ʱ��
  CLOCK_SetTpmClock(1U); //ѡ��PLLFLLSELCLK 180MHz(MCGFLLCLK , or MCGPLLCLK, or IRC48M, or USB1 PFD clock as selected by SOPT2[PLLFLLSEL], and then divided by the PLLFLLCLK fractional divider as configured by SIM_CLKDIV3[PLLFLLFRAC, PLLFLLDIV]. )
  CLOCK_SetPllFllSelClock(1U, 0U, 0U);//Selects MCGPLLCLK for various peripheral clocking options
  
  //TPM��ʼ��
  tpm_config_t tpm_config_struct;
  TPM_GetDefaultConfig(&tpm_config_struct);
  TPM_Init(TPM1, &tpm_config_struct);
  TPM_Init(TPM2, &tpm_config_struct);
  
  /* Set the timer to be in free-running mode */
  TPM1->MOD = 0xFFFF;
  TPM2->MOD = 0xFFFF;
  
  /*����������*/
  TPM1->CNT = 0;
  TPM2->CNT = 0;
  
  //������ģʽ��ʼ��
  tpm_phase_params_t phA;
  phA.phaseFilterVal = 0;
  phA.phasePolarity = kTPM_QuadPhaseNormal;
  tpm_phase_params_t phB;
  phB.phaseFilterVal = 0;
  phB.phasePolarity = kTPM_QuadPhaseNormal;
  
  //ʹ����������
  TPM_SetupQuadDecode(TPM1, &phA, &phB, kTPM_QuadPhaseEncode);
  TPM_SetupQuadDecode(TPM2, &phA, &phB, kTPM_QuadPhaseEncode);
  
  //ʹ�ܼ�����
  TPM_StartTimer(TPM1, kTPM_SystemClock);
  TPM_StartTimer(TPM2, kTPM_SystemClock);    
  
}

void PIT_Init_For_IT(PITn pitn, uint32_t ms)
{
  NVIC_SetPriority(PIT2_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
  NVIC_EnableIRQ(PIT2_IRQn);
  PIT_Init(pitn, ms);
}


/**
* @brief       ����TPM����������
* @param
* @return      tpm_flag: 1 ��ʾ 1��2����
*                       0 ��ʾ 3��4����
* @example
* @note
*
*/
int8_t pin_turn(void)
{
  static int8_t tpm_flag = 1;
  
  if (tpm_flag) {
    /* PORTA8 is configured as TPM1_CH0 */
    PORT_SetPinMux(PORTA, 8U, kPORT_MuxAlt6);
    /* PORTA9 is configured as TPM1_CH1 */
    PORT_SetPinMux(PORTA, 9U, kPORT_MuxAlt6);
    /* PORTA10 is configured as TPM2_CH0 */
    PORT_SetPinMux(PORTA, 10U, kPORT_MuxAlt6);
    /* PORTA11 is configured as TPM2_CH1 */
    PORT_SetPinMux(PORTA, 11U, kPORT_MuxAlt6);
    
    //cancel 
    PORT_SetPinMux(PORTA, 12U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTA, 13U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTB, 18U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTB, 19U, kPORT_MuxAsGpio);
    
    tpm_flag = 0;
  }
  else {
    /* PORTA12 is configured as TPM1_CH0 */
    PORT_SetPinMux(PORTA, 12U, kPORT_MuxAlt7);
    /* PORTA13 is configured as TPM1_CH1 */
    PORT_SetPinMux(PORTA, 13U, kPORT_MuxAlt7);
    /* PORTB18 is configured as TPM2_CH0 */
    PORT_SetPinMux(PORTB, 18U, kPORT_MuxAlt6);
    /* PORTB19 is configured as TPM2_CH1 */
    PORT_SetPinMux(PORTB, 19U, kPORT_MuxAlt6);
    
    PORT_SetPinMux(PORTA, 8U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTA, 9U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTA, 10U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTA, 11U, kPORT_MuxAsGpio);
    
    tpm_flag = 1;
  }
  
  return tpm_flag;
  
}


/**
* @brief       ��PIT�ж��вɼ��ٶȣ��͵����ٶȣ�
* @param
* @return
* @example
* @note        ���� PIT2 �ж���
* 
*/
void get_speed_and_control(void)
{
  static uint8_t dir_flag_1 = 1;    // Ĭ�ϼ���������
  static uint8_t dir_flag_2 = 1;    // Ĭ�ϼ���������
  
  static int8_t tpm_flag;               // ����ָʾ������������ֵ��ٶ�
  
  static uint16_t count_1;      // TPM1 ����ֵ�ݴ����
  static uint16_t count_2;      // TPM2 ����ֵ�ݴ����
  
  static float rps_1;
  static float rps_2;
  static float rps_3;
  static float rps_4;
  
  static float rps1_set;
  static float rps2_set;
  static float rps3_set;
  static float rps4_set;
  
  
  tpm_flag = pin_turn();        // ��ȡ�������ָʾ
  
  count_1 = TPM1->CNT;
  TPM1->CNT = 0;  
  count_2 = TPM2->CNT;
  TPM2->CNT = 0;
  
  
  // �����������Դ�ȷ��ת��
  // if (!(TPM1_QDCTRL & TPM_QDCTRL_QUADIR_MASK))  // ����־λ���ȶ�������ë�̳���
  if (count_1 > 8192) {
    count_1 = 65535 - count_1;
    dir_flag_1 = 0;     //�������ݼ�
  }
  if (count_2 > 8192) {
    count_2 = 65535 - count_2;
    dir_flag_2 = 0;     //�������ݼ�
  }
  
  switch(tpm_flag) {
    
  case 1:
    rps_1 = count_1 * 50 / 8192.0f;     
    rps_2 = count_2 * 50 / 8192.0f;
    
    //�����������������Դ�ȷ���Ƿ�Ӹ���
    if (dir_flag_1 == 1)
      rps_1 = -rps_1;
    else
      dir_flag_1 = 1;
    
    if (dir_flag_2 == 1)
      rps_2 = -rps_2;
    else
      dir_flag_2 = 1;
    
    pid[0].ActualSpeed = rps_1;
    rps1_set = PID_realize(pid[0].SetSpeed, &(pid[0]));   //�˴���ֵ�������    
    if (rps1_set > 0) {
      GPIO_PinInit(PTC3, GPO, 1);
      FTM_PwmDuty(FTM0, FTM_CH3, (uint16_t)((0.031188f * rps1_set + 0.032839f) * 12000));
    }
    else {
      GPIO_PinInit(PTC3, GPO, 0);
      FTM_PwmDuty(FTM0, FTM_CH3, (uint16_t)((0.031188f * (-rps1_set) + 0.032839f) * 12000));
    }
    
    
    pid[1].ActualSpeed = rps_2;
    rps2_set = PID_realize(pid[1].SetSpeed, &(pid[1]));   //�˴���ֵ�������
    if (rps2_set > 0) {
      GPIO_PinInit(PTD0, GPO, 0);
      FTM_PwmDuty(FTM0, FTM_CH5, (uint16_t)((0.031188f * rps2_set + 0.032839f) * 12000));
    }
    else {
      GPIO_PinInit(PTD0, GPO, 1);
      FTM_PwmDuty(FTM0, FTM_CH5, (uint16_t)((0.031188f * (-rps2_set) + 0.032839f) * 12000));
    }
    
    break;
    
  case 0:
    rps_3 = count_1 * 50 / 8192.0f;     
    rps_4 = count_2 * 50 / 8192.0f;
    
    //�����������������Դ�ȷ���Ƿ�Ӹ���
    if (dir_flag_1 == 0) {
      rps_3 = -rps_3;
      dir_flag_1 = 1;
    }
    if (dir_flag_2 == 0) {
      rps_4 = -rps_4;
      dir_flag_2 = 1;
    }
    
    pid[2].ActualSpeed = rps_3;
    rps3_set = PID_realize(pid[2].SetSpeed, &(pid[2]));   //�˴���ֵ�������
    if (rps3_set > 0) {
      GPIO_PinInit(PTD6, GPO, 1);
      FTM_PwmDuty(FTM0, FTM_CH7, (int)((0.031188f * rps3_set + 0.032839f) * 12000));
    }
    else {
      GPIO_PinInit(PTD6, GPO, 0);
      FTM_PwmDuty(FTM0, FTM_CH7, (int)((0.031188f * (-rps3_set) + 0.032839f) * 12000));
    }
    
    pid[3].ActualSpeed = rps_4;
    rps4_set = PID_realize(pid[3].SetSpeed, &(pid[3]));   //�˴���ֵ�������
    if (rps4_set > 0) {
      GPIO_PinInit(PTC1, GPO, 1);
      FTM_PwmDuty(FTM0, FTM_CH1, (int)((0.031188f * rps4_set + 0.032839f) * 12000));
    }
    else {
      GPIO_PinInit(PTC1, GPO, 0);
      FTM_PwmDuty(FTM0, FTM_CH1, (int)((0.031188f * (-rps4_set) + 0.032839f) * 12000));
    }
    
    break;
    
  }
  
#if 1   
  //��ʾ����
  ANO_DT_send_int16((short)(rps_1 * 100), (short)(rps_2 * 100), (short)(rps_3 * 100), (short)(rps_4 * 100), (short)(rps1_set * 100), (short)(rps2_set * 100), (short)(rps3_set * 100), (short)(rps4_set * 100));
#endif
  
}


