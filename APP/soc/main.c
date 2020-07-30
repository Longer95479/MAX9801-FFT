/**
 * @Platform    龙邱K66核心板        IAR 8.32.1
 * @Fielname    main.c
 * @brief       主函数，总体功能实现听声辨位
 * @Author      Longer95479
 * @Email       371573369@qq.com
 * @Date        2020/6/27
 *
 */

#include "include.h"

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
    NVIC_SetPriority(LPUART0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,1));
    
    NVIC_SetPriority(PIT0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,0));
    NVIC_SetPriority(DMA0_DMA16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,1));
    NVIC_SetPriority(DMA2_DMA18_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,2));
    
    LED_Init();
    
    systime.init(); 
    
    UART_Init(UART4, 115200);           //用于显示波形
    
    motor_init();
    
    ADC_Init(ADC0);                    //ADC0初始化
    ADC_Init(ADC1);                    //ADC1初始化
    ADC_in_DMA_mode_instance_conf(50);  //周期为50us
    
    task_rhythm_init(TASK_RHYTHM_T);    //任务调度器初始化
   
    
    //int time;
    
    //type_complex *Wnk_fft, *Wnk_ifft;
    //Wnk_fft = init_Wnk(fft, _N);
    //Wnk_ifft = init_Wnk(ifft, _N);
        
    //float V_sound = V_sound_Identification(sample_dx, sample_sx, z, Wnk_fft, Wnk_ifft, ADC1_SE8, ADC1_SE9, ADC1, ADC1); //音速辨识
    
    //float V_sound = 352;
    
    while(1) {
      
      task_process();
    
      //**sample_get();
      
      //LPTMR_TimeStartms();
      //** float sx = distance_difference(V_sound, sample_dx, sample_sx, z, kWnk_fft, kWnk_ifft/*, ADC0_SE9, ADC0_SE8, ADC0, ADC0*/);
      //time = LPTMR_TimeGetms();
      
      //** float sy = distance_difference(V_sound, sample_dy, sample_sy, z, kWnk_fft, kWnk_ifft/*, ADC1_SE7a, ADC1_SE6a, ADC1, ADC1*/);
      
      //** car_move(sx, sy);
           
      /*
      IFFT(sample_sx, kWnk_ifft, _N);
      IFFT(sample_dx, kWnk_ifft, _N);
      
      for (int i = 0; i < _N/4; i++) {
        float temp = sample_dx[i].re;
        sample_dx[i].re = sample_dx[_N/2-i-1].re;
        sample_dx[_N/2-i-1].re = temp;
      }
      
      
      IFFT(sample_sy, kWnk_ifft, _N);
      IFFT(sample_dy, kWnk_ifft, _N);
      
      for (int i = 0; i < _N/4; i++) {
        float temp = sample_dy[i].re;
        sample_dy[i].re = sample_dy[_N/2-i-1].re;
        sample_dy[_N/2-i-1].re = temp;
      }
        
      for(int i = 0; i < _N; i++) {
        //ANO_DT_send_int16((int16)(100*sample_sx[i].re), (int16)(100*sample_dx[i].re), (int16)(sx*1000), (int16)(max_x - _N/2 + 1), (int16)z[i].re, (int16)V_sound, 0, 0);  //这里把数据传给上位机 
        //ANO_DT_send_int16((int16)(100*sample_sy[i].re), (int16)(100*sample_dy[i].re), (int16)(sy*1000), (int16)(max_y - _N/2 + 1), (int16)z[i].re, (int16)V_sound, 0, 0);  //这里把数据传给上位机      
        ANO_DT_send_int16((int16)(100*sample_sx[i].re), (int16)(100*sample_dx[i].re), (int16)(100*sample_sy[i].re), (int16)(100*sample_dy[i].re), (int16)z[i].re, 0, 0, 0);  //这里把数据传给上位机 
      }
      */
      //printf("%d\n", time);
      /*
      int max_x = (sx / V_sound - 19e-6) / DELTA_TIME + _N/2 - 1;
      int max_y = (sy / V_sound - 19e-6) / DELTA_TIME + _N/2 - 1; 
      ANO_DT_send_int16((int16)(max_x - _N/2 + 1), (int16)(max_y - _N/2 + 1), 0, 0, 0, 0, 0, 0);
      */
    }
}



