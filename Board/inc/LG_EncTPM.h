#ifndef _LG_EncTPM
#define _LG_EncTPM

void Enc_TPM12_Init(void);
void PIT_Init_For_IT(PITn pitn, uint32_t ms);

/**
* @brief       更换TPM的输入引脚
* @param
* @return      tpm_flag: 0 表示 1、2号轮
*                       1 表示 3、4号轮
* @example
* @note
*
*/
int8_t pin_turn(void);


/**
* @brief       在PIT中断中采集速度（和调整速度）
* @param
* @return
* @example
* @note        放在 PIT2 中断里
* 
*/
void get_speed_and_control(void);

#endif