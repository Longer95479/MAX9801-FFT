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
    
    for(int i = 0; i < _N; i++) {
        sample_s[i].re = 0;
        sample_s[i].im = 0;
        sample_d[i].re = 0;
        sample_d[i].im = 0;
      }
    
    while(1) {
      
      LPTMR_TimeStartus();
      for(int i = 0; i < _N/2; i++) {
        sample_s[i].re = (uint16_t)(ADC_Get(0)*0.806);  //PTB4, ADC�ɼ�����λ�� mv�����д���ִ����Ҫ 19us
        sample_d[_N/2-i-1].re = (uint16_t)(ADC_Get(1)*0.806);  //PTB5
        systime.delay_us(158);                    //��ʱ�Կ��Ʋ���Ƶ�ʣ�Ŀǰ��200us�ɼ�һ������
      }
      
      
      FFT(sample_s, Wnk_fft, _N);      
      FFT(sample_d, Wnk_fft, _N);
      
      for (int i = 0; i < _N; i++) {
        z[i] = complex_mult(sample_s[i], sample_d[i]);
      }
      
      IFFT(z, Wnk_ifft, _N);
      
      int max = 0;
      for (int i = 1; i < _N; i++)
        if (z[i].re > z[max].re)
          max = i;
      
      float s = (max - _N + 1) * 340 * 0.000038;
      
      time = LPTMR_TimeGetus();
      
      /*
      for(int i = 0; i < _N; i++) 
        ANO_DT_send_int16(batv1[i], batv2[i], 0, 0, 0, 0, 0, 0);  //��������ݴ�����λ��
      */
      
      printf("%d, %f\n", time, s);
      
    }
}



