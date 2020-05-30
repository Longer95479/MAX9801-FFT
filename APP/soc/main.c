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

type_complex sample_sx[_N], sample_dx[_N], sample_sy[_N], sample_dy[_N], z[_N];
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
    ADC_Init(ADC0);
    
    int time;
    
    type_complex *Wnk_fft, *Wnk_ifft;
    Wnk_fft = init_Wnk(fft, _N);
    Wnk_ifft = init_Wnk(ifft, _N);
        
    float V_sound = V_sound_Identification(sample_dx, sample_sx, z, Wnk_fft, Wnk_ifft, ADC1_SE8, ADC1_SE9, 1); //音速辨识
    
    V_sound = 352;
    
    while(1) {
      
      //LPTMR_TimeStartms();
      float sx = distance_difference(V_sound, sample_dx, sample_sx, z, Wnk_fft, Wnk_ifft, ADC1_SE8, ADC1_SE9, 1);
      //time = LPTMR_TimeGetms();
      
      float sy = distance_difference(V_sound, sample_dy, sample_sy, z, Wnk_fft, Wnk_ifft, ADC0_SE12, ADC0_SE13, 0);
      
      int max = (sy / V_sound - 19e-6) / DELTA_TIME + _N/2 - 1;
      
            
      IFFT(sample_sy, Wnk_ifft, _N);
      IFFT(sample_dy, Wnk_ifft, _N);
      
      //for(int i = 0; i < _N; i++) 
        //ANO_DT_send_int16((int16)(100*sample_sy[i].re), (int16)(100*sample_dy[i].re), (int16)(sy*1000), (int16)(max - _N/2 + 1), (int16)z[i].re, (int16)V_sound, 0, 0);  //这里把数据传给上位机      
      
      //printf("%d, %d\n", time, max - _N/2 + 1);
      ANO_DT_send_int16((int16)(max - _N/2 + 1), 0, 0, 0, 0, 0, 0, 0);
      
    }
}



