/*-----------------------------------------------------------------------------------------------------
��ƽ    ̨������K66���İ�-���ܳ���
����    д��LQ-005
��E-mail  ��chiusir@163.com
������汾��V1.0������Դ���룬�����ο�������Ը�
�������¡�2019��04��02��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
������ƽ̨��IAR 8.2
����    �ܡ�
��ע�����
-------------------------------------------------------------------------------------------------------*/
#include "include.h"

type_complex sample_s[_N], sample_d[_N], z[_N];
int main(void) 
{
   PLL_Init(PLL180);                   //��ʼ��PLLΪ180M 
    
    /* �����ж����ȼ���  0: 0����ռ���ȼ�16λ�������ȼ� 
     * 1: 2����ռ���ȼ� 8�������ȼ� 2: 4����ռ���ȼ� 4�������ȼ� 
     * 3: 8����ռ���ȼ� 2�������ȼ� 4: 16����ռ���ȼ� 0�������ȼ�
     */
    /* �������ȼ��� 2: 4����ռ���ȼ� 4�������ȼ� */
    NVIC_SetPriorityGrouping(0x07 - 2);
    
    /* ���ȼ����� ��ռ���ȼ�0  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж� */
    NVIC_SetPriority(UART4_RX_TX_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
    
    LED_Init();
    
    LPTMR_PulseInit(LPT0_ALT1,32768,LPT_Rising);  
    systime.init(); 
    
    UART_Init(UART4, 115200);           //������ʾ����
    
    ADC_Init(ADC1);                    //ADC1��ʼ��
    
    int time;
    
    type_complex *Wnk_fft, *Wnk_ifft;
    Wnk_fft = init_Wnk(fft, _N);
    Wnk_ifft = init_Wnk(ifft, _N);
        
    float V_sound = V_sound_Identification(sample_d, sample_s, z, Wnk_fft, Wnk_ifft); //���ٱ�ʶ
    
    while(1) {
      
      float s = distance_difference(V_sound, sample_d, sample_s, z, Wnk_fft, Wnk_ifft);
      
      int max = (s / V_sound - 19e-6) / 5e-5 + _N/2 - 1;
            
      IFFT(sample_s, Wnk_ifft, _N);
      IFFT(sample_d, Wnk_ifft, _N);
      
      for(int i = 0; i < _N; i++) 
        ANO_DT_send_int16((int16)(100*sample_s[i].re), (int16)(100*sample_d[i].re), (int16)(s*1000), (int16)(max - _N/2 + 1), (int16)z[i].re, (int16)V_sound, 0, 0);  //��������ݴ�����λ��      
      
      //printf("%d, %f\n", time, s);
      
    }
}



