/**
 * @Platform    ����K66���İ�        IAR 8.32.1
 * @Fielname    main.c
 * @brief       �����������幦��ʵ��������λ
 * @Author      Longer95479
 * @Email       371573369@qq.com
 * @Date        2020/6/27
 *
 */


#define MAIN_GLOBALS

#include "include.h"

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
    NVIC_SetPriority(PIT2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,0));
    NVIC_SetPriority(PIT0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,0));
    NVIC_SetPriority(DMA0_DMA16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,1));
    NVIC_SetPriority(DMA2_DMA18_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,2));
    
    LED_Init();
    
    systime.init(); 
    
    UART_Init(UART4, 115200);           //������ʾ����
    UART_Init(UART2, 115200);           //���ں� OpenMV ͨѶ
    
    MPU6050_Init();             //���ڴ��ڳ�ʼ��֮���ʼ������Ϊ��ʼ����Ϣ�������ڴ�ӡ
    
    //NVIC_EnableIRQ(UART4_RX_TX_IRQn);   //ʹ��UART4_RX_TX_IRQn���ж�
    NVIC_EnableIRQ(UART2_RX_TX_IRQn);   //ʹ��UART2_RX_TX_IRQn���ж�
    
    PID_init();
    Enc_TPM12_Init();
    PIT_Init_For_IT(PIT2, 5);          //�ж����� PID���ڣ�����ÿ���ֵĿ�������Ϊ�ж����ڵ�����
    motor_init();
    
    ADC_Init(ADC0);                    //ADC0��ʼ��
    ADC_Init(ADC1);                    //ADC1��ʼ��
    ADC_in_DMA_mode_instance_conf(50);  //����Ϊ50us
    
    task_rhythm_init(20);    //�����������ʼ�������ĵ�λΪms
   
    
    while(1) {
#if 1
      static float theta;
      
      static float theta_mean;
      static uint8_t theta_add_cnt = 0;
      static int8_t car_run_first_in = 1;           //��Ҫ������״̬��λ
      
      static int8_t max_x, max_y;
      static uint8_t x_ready_for_theta, y_ready_for_theta;      // �����ж��Ƿ�ʼ���� theta
      
      //**task_process();
    
      //**sample_get();
      
      // ADC�ɼ����������´�������� x �᷽��ļ����
      if (x_ready_for_fft == 1) {
        data_type_trans(sample_sx);
        data_reprocess(sample_sx, sample_dx);
        
        x_ready_for_fft = 0;
        
        max_x = distance_difference(340, sample_dx, sample_sx, z_x, kWnk_fft, kWnk_ifft);
        
        x_ready_for_theta = 1;
        
        DMA_EN(DMA_CH0);    //��ʱ�������̿�ʼ��һ�ֵĲɼ�        
      }
      
      // ADC�ɼ����������´�������� x �᷽��ļ����
      if (y_ready_for_fft == 1) {
        data_type_trans(sample_sy);
        data_reprocess(sample_sy, sample_dy);
         
        y_ready_for_fft = 0;
        
        max_y = distance_difference(340, sample_dy, sample_sy, z_y, kWnk_fft, kWnk_ifft);
        
        y_ready_for_theta = 1;
        
        DMA_EN(DMA_CH2);    //��ʱ�������̿�ʼ��һ�ֵĲɼ�
      }  
      
      // ����� theta�����趨�����ٶȵ�Ŀ��ֵ
      if (x_ready_for_theta && y_ready_for_theta) {
        theta = get_theta(max_x, max_y);
        
        for (int i = 0; i < 4; i ++) {
          amplitude[4] += amplitude[i];
        }        
        amplitude[4] /= 4;
        
        //ANO_DT_send_int16((int16)(amplitude[0]), (int16)(amplitude[1]), (int16)(amplitude[2]), (int16)(amplitude[3]), (int16)(amplitude[4]), 0, 0, 0);
        //ANO_DT_send_int16((int16_t)(100 * theta), (int16_t)max_x, (int16_t)max_y, 0, 0, 0, 0, 0);        //�����ڲ���
        //ANO_DT_send_int16((int16)(100 * theta), (int16)(amplitude[4]), (int16)max_x, (int16)max_y, 0, 0, 0, 0);
        
        x_ready_for_theta = 0;
        y_ready_for_theta = 0;
        
        
        // �˶�����
        //car_control(theta, amplitude[4]);
        
        static float vx, vy, wheel_v[4];
        
        vx = 4.5 * cos(theta);         //rps * cos(theta)
        vy = 4.5 * sin(theta);         //rps * sin(theta)
        
        wheel_v[0] = vx + vy;
        wheel_v[1] = -vx + vy;
        wheel_v[2] = vx + vy;
        wheel_v[3] = -vx + vy;
        
        for (int i = 0; i < 4; i++)
          pid[i].SetSpeed = wheel_v[i];
        
        
        
        /*
        if (car_status == CAR_RUN) {
          
          vx = 5 * cos(theta);         //rps * cos(theta)
          vy = 5 * sin(theta);         //rps * sin(theta)
          
          wheel_v[0] = vx + vy;
          wheel_v[1] = -vx + vy;
          wheel_v[2] = vx + vy;
          wheel_v[3] = -vx + vy;
          
          for (int i = 0; i < 4; i++)
            pid[i].SetSpeed = wheel_v[i];
          
        }
        
        else 
          car_stop();
                    
        */  
        
      
#else
      
      static float theta;
      static int8_t max_sx, max_dx, max_sy, max_dy, max_x, max_y;
      static uint8_t x_ready_for_theta, y_ready_for_theta;      // �����ж��Ƿ�ʼ���� theta
      
      // ADC�ɼ����������´�������� x �᷽��ļ����
      if (x_ready_for_fft == 1) {
        data_type_trans(sample_sx);
        data_reprocess_no_overturn(sample_sx, sample_dx);
        
        x_ready_for_fft = 0;
        
        max_sx = distance_difference2(sample_sx, z_x, kWnk_fft, kWnk_ifft);
        max_dx = distance_difference2(sample_dx, z_x, kWnk_fft, kWnk_ifft);
        /*
        for (int i = 0; i < _N; i++) {
        ANO_DT_send_int16((int16)(100 * z_x[i].re), 0, 0, 0, 0, 0, 0, 0);       //������
      }
       */ 
        max_x = max_sx - max_dx;
        
        x_ready_for_theta = 1;
        
        DMA_EN(DMA_CH0);    //��ʱ�������̿�ʼ��һ�ֵĲɼ�        
      }
      
      // ADC�ɼ����������´�������� x �᷽��ļ����
      if (y_ready_for_fft == 1) {
        data_type_trans(sample_sy);
        data_reprocess_no_overturn(sample_sy, sample_dy);
         
        y_ready_for_fft = 0;
        
        max_sy = distance_difference2(sample_sy, z_x, kWnk_fft, kWnk_ifft);
        max_dy = distance_difference2(sample_dy, z_x, kWnk_fft, kWnk_ifft);
        
        max_y = max_sy - max_dy;
        
        y_ready_for_theta = 1;
        
        DMA_EN(DMA_CH2);    //��ʱ�������̿�ʼ��һ�ֵĲɼ�
      }  
      
      // ����� theta�����趨�����ٶȵ�Ŀ��ֵ
      if (x_ready_for_theta && y_ready_for_theta) {
        theta = get_theta(max_x, max_y);
        
        //ANO_DT_send_int16((int16_t)(100 * theta), (int16_t)max_x, (int16_t)max_y, 0, 0, 0, 0, 0);        //�����ڲ���
        
        x_ready_for_theta = 0;
        y_ready_for_theta = 0;
        
        static float vx, vy, wheel_v[4];
        
        vx = 4 * cos(theta);         //rps * cos(theta)
        vy = 4 * sin(theta);         //rps * sin(theta)
        
        wheel_v[0] = vx + vy;
        wheel_v[1] = -vx + vy;
        wheel_v[2] = vx + vy;
        wheel_v[3] = -vx + vy;
        
        for (int i = 0; i < 4; i++)
          pid[i].SetSpeed = wheel_v[i];
      }
#endif
      /*
      for (int i = 0; i < _N; i++) {
        z_x[i].re = ref_chirp[i].re;
        z_x[i].im = ref_chirp[i].im;
      }
      
       IFFT(z_x, kWnk_ifft, _N);
      
      for (int i = 0; i < _N/4; i++) {
        float temp = z_x[i].re;
        z_x[i].re = z_x[_N/2-i-1].re;
        z_x[_N/2-i-1].re = temp;
      }
      
      for (int i = 0; i < _N; i++) {
        ANO_DT_send_int16((int16)(100 * z_x[i].re), (int16)(100 * z_x[i].im), 0, 0, 0, 0, 0, 0);
      }
      */
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
        //ANO_DT_send_int16((int16)(100*sample_sx[i].re), (int16)(100*sample_dx[i].re), (int16)(sx*1000), (int16)(max_x - _N/2 + 1), (int16)z[i].re, (int16)V_sound, 0, 0);  //��������ݴ�����λ�� 
        //ANO_DT_send_int16((int16)(100*sample_sy[i].re), (int16)(100*sample_dy[i].re), (int16)(sy*1000), (int16)(max_y - _N/2 + 1), (int16)z[i].re, (int16)V_sound, 0, 0);  //��������ݴ�����λ��      
        ANO_DT_send_int16((int16)(100*sample_sx[i].re), (int16)(100*sample_dx[i].re), (int16)(100*sample_sy[i].re), (int16)(100*sample_dy[i].re), (int16)z_x[i].re, 0, 0, 0);  //��������ݴ�����λ�� 
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
}
