#ifndef _LG_EncTPM
#define _LG_EncTPM

void Enc_TPM12_Init(void);
void PIT_Init_For_IT(PITn pitn, uint32_t ms);

/**
* @brief       ����TPM����������
* @param
* @return      tpm_flag: 0 ��ʾ 1��2����
*                       1 ��ʾ 3��4����
* @example
* @note
*
*/
int8_t pin_turn(void);


/**
* @brief       ��PIT�ж��вɼ��ٶ� �� ����ƽ���������ٶ�
* @param
* @return
* @example
* @note        ���� PIT2 �ж���
* 
*/
void get_speed_and_set_translation_speed(void);

#endif