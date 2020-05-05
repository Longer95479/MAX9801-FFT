/*-----------------------------------------------------------------------------------------------------
【平    台】龙邱K66核心板-智能车板
【编    写】LQ-005
【E-mail  】chiusir@163.com
【软件版本】V1.0，龙邱开源代码，仅供参考，后果自负
【最后更新】2019年04月02日
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【编译平台】IAR 8.2
【功    能】
【注意事项】
-------------------------------------------------------------------------------------------------------*/
#include "include.h"

type_complex sample_s[_N], sample_d[_N], z[_N];
int main(void) 
{
   PLL_Init(PLL180);                   //初始化PLL为180M 
    
    /* 设置中断优先级组  0: 0个抢占优先级16位个子优先级 
     * 1: 2个抢占优先级 8个子优先级 2: 4个抢占优先级 4个子优先级 
     * 3: 8个抢占优先级 2个子优先级 4: 16个抢占优先级 0个子优先级
     */
    /* 配置优先级组 2: 4个抢占优先级 4个子优先级 */
    NVIC_SetPriorityGrouping(0x07 - 2);
    
    /* 优先级配置 抢占优先级0  子优先级2   越小优先级越高  抢占优先级可打断别的中断 */
    NVIC_SetPriority(UART4_RX_TX_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
    
    LED_Init();
    
    LPTMR_PulseInit(LPT0_ALT1,32768,LPT_Rising);  
    systime.init(); 
    
    UART_Init(UART4, 115200);           //用于显示波形
    
    ADC_Init(ADC1);                    //ADC1初始化
    
    int time;
    
    type_complex *Wnk_fft, *Wnk_ifft;
    Wnk_fft = init_Wnk(fft, _N);
    Wnk_ifft = init_Wnk(ifft, _N);
    
    
    
    while(1) {
      
      for(int i = 0; i < _N; i++) {
        sample_s[i].re = 0;
        sample_s[i].im = 0;
        sample_d[i].re = 0;
        sample_d[i].im = 0;
      }
      
      LPTMR_TimeStartms();
      for(int i = 0; i < _N/2; i++) {
        sample_s[i].re = (uint16_t)(ADC_Get(0)*0.806);  //PTB4, ADC采集，单位是 mv，这行代码执行需要 19us
        sample_d[_N/2-i-1].re = (uint16_t)(ADC_Get(1)*0.806);  //PTB5
        systime.delay_us(12); //经检测，这个其实是ms延时      //延时以控制采样频率，目前是200us采集一次数据
      }
      time = LPTMR_TimeGetms();
    
      //对波形标准化，寻找均值和最大幅值
      static int16 max_s[32] = {0}, min_s[32] = {0}, max_d[32] = {0}, min_d[32] = {0};
      for (int i = 0; i < 32; i++) {
        for (int j = i * _N/2/32; j < (i+1) * _N/2/32; j++) {
          
          if (sample_s[j].re > sample_s[max_s[i]].re)
            max_s[i] = j;
          else if (sample_s[j].re < sample_s[min_s[i]].re)
            min_s[i] = j;
          
          if (sample_d[j].re > sample_d[max_d[i]].re)
            max_d[i] = j;
          else if (sample_d[j].re < sample_d[min_d[i]].re)
            min_d[i] = j;
        }            
      }
      
      static float mean_max_s = 0, mean_min_s = 0, mean_max_d = 0, mean_min_d = 0;
      mean_max_s = 0, mean_min_s = 0, mean_max_d = 0, mean_min_d = 0;
      for (int i = 0; i < 32; i++) {
        mean_max_s = mean_max_s + sample_s[max_s[i]].re;
        mean_min_s = mean_min_s + sample_s[min_s[i]].re;
        mean_max_d = mean_max_d + sample_d[max_d[i]].re;
        mean_min_d = mean_min_d + sample_d[min_d[i]].re;     
      }
      
      mean_max_s = mean_max_s/32;
      mean_min_s = mean_min_s/32;
      mean_max_d = mean_max_d/32;
      mean_min_d = mean_min_d/32;
      
      static float amplitude_s, mid_s, amplitude_d, mid_d;
      amplitude_s = (mean_max_s - mean_min_s) / 2;
      amplitude_d = (mean_max_d - mean_min_d) / 2;
      mid_s = (mean_max_s + mean_min_s) / 2;
      mid_d = (mean_max_d + mean_min_d) / 2;
      
      
      for (int i = 0; i < _N/2; i++) {
        sample_s[i].re = (sample_s[i].re - mid_s)/amplitude_s;
        sample_d[i].re = (sample_d[i].re - mid_d)/amplitude_d;
        
      }
      
      
      FFT(sample_s, Wnk_fft, _N);      
      FFT(sample_d, Wnk_fft, _N);
      
      for (int i = 0; i < _N; i++) {
        z[i] = complex_mult(sample_s[i], sample_d[i]);
      }
      
      IFFT(z, Wnk_ifft, _N);
      
      int max = 0;
      for (int i = _N/2 - 1; i < _N; i++)
        if (z[i].re > z[max].re)
          max = i;
      
      float s = (max - _N/2 + 1) * 343 * 5e-5;
      
      
      IFFT(sample_s, Wnk_ifft, _N);
      IFFT(sample_d, Wnk_ifft, _N);
      
      for(int i = 0; i < _N; i++) 
        ANO_DT_send_int16((int16)(100*sample_s[i].re), (int16)(100*sample_d[i].re), (int16)(s*1000), (max - _N/2 + 1), (int16)z[i].re, 0, 0, 0);  //这里把数据传给上位机
      
      
      //printf("%d, %f\n", time, s);
      
    }
}



