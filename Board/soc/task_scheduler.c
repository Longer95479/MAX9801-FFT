/**
 * @Platform    ����K66���İ�        IAR 8.32.1
 * @Fielname    task_scheduler.c
 * @brief       ���������
 * @Author      Longer95479
 * @Email       371573369@qq.com
 * @Date        2020/7/27
 *
 */
   
/*!
 * @file_note    ��������ȫ��ִ��һ���ʱ�䲻�ܳ�������ʱ�ӽ�������
 *              ����������ʱ�䷽��ϴ������Ҫ���ڿ����λ��
 *              ����ʱ��Խ�̵��������Խ��ǰ
 * 
 */

#include "include.h"

   
/**********************************************************************/
/*************************����ʵ����ʼ����*****************************/
/**********************************************************************/   
/**
 * @brief       �������
 */
#define TASK_NUM        1

/**
 * @brief       �����Ծ��������Ϊ0ʱ�������������
 */
static uint8_t active_task_num = 0;
   
/**
 * @brief       �����ͨѶ�ṹ�壬1 ��ʾ��Դ�����ɣ�0 ��ʾ��Դ��δ����
 */
static task_commu_t t2wft1_can_xfft = {0};
static task_commu_t t3wft1_can_yfft = {0};
static task_commu_t t4wft2_can_xUART = {0};
static task_commu_t t4wft3_can_yUART = {0};


/**
 * @brief       ������ں�������
 */
void task0_entry(void *arg);
void task1_entry(void *arg);
void task2_entry(void *arg);
void task3_entry(void *arg);
void task4_entry(void *arg);

/**
 * @brief       ����ʵ����ʱ����ʼֵ
 */
typedef enum {
  TASK0_TIMER_INIT_VAL = 1,
  TASK1_TIMER_INIT_VAL = 1,
  TASK2_TIMER_INIT_VAL = 1,
  TASK3_TIMER_INIT_VAL = 1,
  TASK4_TIMER_INIT_VAL = 2,
} task_timer_init_val_instance;
   
   
/**
 * @brief       ������ƿ�ʵ���������ʼ��
 * @note        �е���ں���˳���������ִ�е�˳����Ҫ�������ȼ���������˳�򼴿�
 *
 */
static task_t tasks[TASK_NUM] = {
  {TASK_DELAY, TASK0_TIMER_INIT_VAL, TASK0_TIMER_INIT_VAL, NULL, task0_entry},
  //**{TASK_DELAY, TASK1_TIMER_INIT_VAL, TASK1_TIMER_INIT_VAL, NULL, task1_entry},
  //**{TASK_DELAY, TASK2_TIMER_INIT_VAL, TASK2_TIMER_INIT_VAL, NULL, task2_entry},
  //**{TASK_DELAY, TASK3_TIMER_INIT_VAL, TASK3_TIMER_INIT_VAL, NULL, task3_entry},
  //**{TASK_DELAY, TASK4_TIMER_INIT_VAL, TASK4_TIMER_INIT_VAL, NULL, task4_entry},
};   
/***********************************************************************/
/***********************************************************************/

/**
 * @brief       �������ʱ��������
 */
static void PIT1_start_count(void);
static uint32_t PIT1_get_time(void);

/**********************************************************************/
/*************************������ں���*********************************/
/**********************************************************************/
/**
 * @brief       ����0��ں�����һ�����ķ�תһ�εƣ���Ϊ����ֱ�ۼ��
 */
void task0_entry(void *arg)
{ 
  LED_Reverse(2);
  //printf("task0 is running\n");
}



/**
 * @brief       ����1��ں����������ش���
 */
void task1_entry(void *arg)
{
  PIT1_start_count();
  
  if (x_ready_for_fft == 1) {
    data_type_trans(sample_sx);
    data_reprocess(sample_sx, sample_dx);
    
    //DMA_EN(DMA_CH0); 
    x_ready_for_fft = 0;
    t2wft1_can_xfft.flag = 1;    //���ݴ����꣬������FFT
    
    //LED_Reverse(0);
  }
  
  if (y_ready_for_fft == 1) {
    data_type_trans(sample_sy);
    data_reprocess(sample_sy, sample_dy);
    
    //DMA_EN(DMA_CH2); 
    y_ready_for_fft = 0;
    t3wft1_can_yfft.flag = 1;    //���ݴ����꣬������FFT
    
    //LED_Reverse(1);
  }  
  
  
  printf("task1 running time: %u.\n", PIT1_get_time());
  
}


/**
 * @brief       ����2��ں�����x���FFT����
 */
void task2_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN


//! ���α�׼����sample_sx �� FFT
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t2wft1_can_xfft.flag == 1) {
    t2wft1_can_xfft.flag = 0;
    
    amplitude_and_mean_process(sample_sx);
    amplitude_and_mean_process(sample_dx);
    
    FFT(sample_sx, kWnk_fft, _N); 
    
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
  }
  
  printf("task2_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! sample_dx �� FFT��Ƶ���˲�
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  
  FFT(sample_dx, kWnk_fft, _N);
  low_pass_filter(sample_sx);
  low_pass_filter(sample_dx);
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
  
  printf("task2_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! ���廥��ش���
SUBTASK_CASE SUBTASK_STATUS2:
  PIT1_start_count();
  // ���廥���
  for (int i = 0; i < _N; i++) {
    z_x[i] = complex_mult(sample_sx[i], sample_dx[i]);
        
    static float A = 1;
    A =  100 * InvSqrt(pow(z_x[i].re, 2) + pow(z_x[i].im, 2));
    z_x[i].re = z_x[i].re * A;
    z_x[i].im = z_x[i].im * A;
  }
  
  DMA_EN(DMA_CH0);    //��ʱ���ɿ�ʼ��һ�ֵĲɼ�
  //printf("DMA_ERQ_ERQ0 = %d\n", (int8_t)(DMA_ERQ & (DMA_ERQ_ERQ0_MASK<<(DMA_CH0))));  //�����ڲ���
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS3;
  
  printf("task2_subtask2 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK  


//! z �� IFFT�����һ���ؽ�����ֵ������
SUBTASK_CASE SUBTASK_STATUS3:
  PIT1_start_count();
  
  IFFT(z_x, kWnk_ifft, _N);
  
  // max����λֵ�˲�
  static int16 max_now, max_queue[9] = {0}; //�������飬0��
  max_now = 0;
  for (int i = 0; i < _N; i++)
    if (z_x[i].re > z_x[max_now].re)
      max_now = i;
  
  t4wft2_can_xUART.int16_val = midst_filter(max_now, max_queue, 5) - _N/2 + 1;
  t4wft2_can_xUART.flag = 1;
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
  
  printf("task2_subtask3 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

  
SUBTASK_END  
}


/**
 * @brief       ����3��ں�����y���FFT����
 */
void task3_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN


//! ���α�׼����sample_sy �� FFT
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t3wft1_can_yfft.flag == 1) {
    t3wft1_can_yfft.flag = 0;
    
    amplitude_and_mean_process(sample_sy);
    amplitude_and_mean_process(sample_dy);
    
    FFT(sample_sy, kWnk_fft, _N); 
    
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
  }
  
  printf("task3_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! sample_dy �� FFT��Ƶ���˲�
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  
  FFT(sample_dy, kWnk_fft, _N);
  low_pass_filter(sample_sy);
  low_pass_filter(sample_dy);
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
  
  printf("task3_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! ���廥���
SUBTASK_CASE SUBTASK_STATUS2:
  PIT1_start_count();
  
  // ���廥���
  for (int i = 0; i < _N; i++) {
    z_y[i] = complex_mult(sample_sy[i], sample_dy[i]);
    
    static float A = 1;
    A =  100 * InvSqrt(pow(z_y[i].re, 2) + pow(z_y[i].im, 2));
    z_y[i].re = z_y[i].re * A;
    z_y[i].im = z_y[i].im * A;
  }
  
  DMA_EN(DMA_CH2);    //��ʱ�������̿�ʼ��һ�ֵĲɼ�
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS3;
  
  printf("task3_subtask2 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! z �� IFFT�����һ���ؽ�����ֵ������
SUBTASK_CASE SUBTASK_STATUS3:
  PIT1_start_count();
  
  IFFT(z_y, kWnk_ifft, _N);
  
  // max����λֵ�˲�
  static int16 max_now, max_queue[9] = {0}; //�������飬0��
  max_now = 0;
  for (int i = 0; i < _N; i++)
    if (z_y[i].re > z_y[max_now].re)
      max_now = i;
  
  t4wft3_can_yUART.int16_val = midst_filter(max_now, max_queue, 5) - _N/2 + 1;
  t4wft3_can_yUART.flag = 1;
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
  
  printf("task3_subtask3 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

  
SUBTASK_END    
}


/**
 * @brief       ����4��ں��������ڴ��ڷ�������
 */
void task4_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN

// ������ڴ���������Ƿ�����
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t4wft2_can_xUART.flag & t4wft3_can_yUART.flag == 1)
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;

  printf("task4_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

//�������ݣ�������ɺ�ʹ�� DMA����
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  /*
  static uint8 group = 0;
  
  for (int i = group * 32; i < (group + 1) * 32; i++)
      ANO_DT_send_int16((int16_t)(100 * sample_sx[i].re), (int16_t)(100 * sample_dx[i].re), (int16_t)(100 * sample_sy[i].re), (int16_t)(100 * sample_dy[i].re), 0, 0, 0, 0);
  
  group++;
  
  if (group == _N/32) {
    DMA_EN(DMA_CH0); 
    DMA_EN(DMA_CH2); 
    
    group = 0;
    t4wft2_can_xUART.flag = 0;
    t4wft3_can_yUART.flag = 0;
    
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
  }
  */
  
  ANO_DT_send_int16(t4wft2_can_xUART.int16_val, t4wft3_can_yUART.int16_val, 0, 0, 0, 0, 0, 0);
  
  t4wft2_can_xUART.flag = 0;
  t4wft3_can_yUART.flag = 0;
  
  printf("task4_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK  
  
  
SUBTASK_END  
    
}


/**
 * @brief       ����5��ں�����x���FFT���㣨��ʾ���ΰ棩
 */
void task5_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN


//! ���α�׼����sample_sx �� FFT
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t2wft1_can_xfft.flag == 1) {
    t2wft1_can_xfft.flag = 0;
    
    //**DMA_EN(DMA_CH0);    //��ʱ���ɿ�ʼ��һ�ֵĲɼ�
    
    amplitude_and_mean_process(sample_sx);
    amplitude_and_mean_process(sample_dx);
    
    FFT(sample_sx, kWnk_fft, _N); 
    
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
  }
  
  printf("task2_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! sample_dx �� FFT��Ƶ���˲�
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  
  FFT(sample_dx, kWnk_fft, _N);
  low_pass_filter(sample_sx);
  low_pass_filter(sample_dx);
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
  
  printf("task2_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! ���廥��ش���
SUBTASK_CASE SUBTASK_STATUS2:
  PIT1_start_count();
  // ���廥���
  for (int i = 0; i < _N; i++) {
    z_x[i] = complex_mult(sample_sx[i], sample_dx[i]);
        
    static float A = 1;
    A =  100 * InvSqrt(pow(z_x[i].re, 2) + pow(z_x[i].im, 2));
    z_x[i].re = z_x[i].re * A;
    z_x[i].im = z_x[i].im * A;
  }
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS3;
  
  printf("task2_subtask2 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK  


//! z �� IFFT�����һ���ؽ�����ֵ������
SUBTASK_CASE SUBTASK_STATUS3:
  PIT1_start_count();
/*  
  IFFT(z_x, kWnk_ifft, _N);
  
  // max����λֵ�˲�
  static int16 max_now, max_queue[9] = {0}; //�������飬0��
  max_now = 0;
  for (int i = 0; i < _N; i++)
    if (z_x[i].re > z_x[max_now].re)
      max_now = i;
  
  t4wft2_can_xUART.int16_val = midst_filter(max_now, max_queue, 5) - _N/2 + 1;
  t4wft2_can_xUART.flag = 1;
*/
  IFFT(sample_sx, kWnk_ifft, _N);
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS4;
  
  printf("task2_subtask3 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


SUBTASK_CASE SUBTASK_STATUS4:
  
  IFFT(sample_dx, kWnk_ifft, _N);
  
  t4wft2_can_xUART.flag = 1;
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
SUBTASK_BREAK  
  
SUBTASK_END  
}


/**
 * @brief       ����6��ں�����y���FFT���㣨��ʾ���ΰ棩
 */
void task6_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN

//! ���α�׼����sample_sx �� FFT
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t3wft1_can_yfft.flag == 1) {
    t3wft1_can_yfft.flag = 0;
    
    //**DMA_EN(DMA_CH2);    //��ʱ�������̿�ʼ��һ�ֵĲɼ�
    
    amplitude_and_mean_process(sample_sy);
    amplitude_and_mean_process(sample_dy);
    
    FFT(sample_sy, kWnk_fft, _N); 
    
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
  }
  
  printf("task3_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

//! sample_dy �� FFT��Ƶ���˲�
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  
  FFT(sample_dy, kWnk_fft, _N);
  low_pass_filter(sample_sy);
  low_pass_filter(sample_dy);
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
  
  printf("task3_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

//! ���廥���
SUBTASK_CASE SUBTASK_STATUS2:
  PIT1_start_count();
  // ���廥���
  for (int i = 0; i < _N; i++) {
    z_y[i] = complex_mult(sample_sy[i], sample_dy[i]);
        
    static float A = 1;
    A =  100 * InvSqrt(pow(z_y[i].re, 2) + pow(z_y[i].im, 2));
    z_y[i].re = z_y[i].re * A;
    z_y[i].im = z_y[i].im * A;
  }
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS3;
  
  printf("task3_subtask2 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! z �� IFFT�����һ���ؽ�����ֵ������
SUBTASK_CASE SUBTASK_STATUS3:
  PIT1_start_count();
  /*
  IFFT(z_y, kWnk_ifft, _N);
  
  // max����λֵ�˲�
  static int16 max_now, max_queue[9] = {0}; //�������飬0��
  max_now = 0;
  for (int i = 0; i < _N; i++)
    if (z_y[i].re > z_y[max_now].re)
      max_now = i;
  
  t4wft3_can_yUART.int16_val = midst_filter(max_now, max_queue, 9) - _N/2 + 1;
  t4wft3_can_yUART.flag = 1;
  */
  IFFT(sample_sy, kWnk_ifft, _N);
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS4;
  
  printf("task3_subtask3 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

SUBTASK_CASE SUBTASK_STATUS4:
  
  IFFT(sample_dy, kWnk_ifft, _N);
  
  t4wft3_can_yUART.flag = 1;
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
SUBTASK_BREAK
  
SUBTASK_END    
}


/**
 * @brief       ����7��ں��������ڴ��ڷ������ݣ���ʾ���ΰ棩
 */
void task7_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN

// ������ڴ���������Ƿ�����
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t4wft2_can_xUART.flag & t4wft3_can_yUART.flag == 1)
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;

  printf("task4_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

//�������ݣ�������ɺ�ʹ�� DMA����
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  
  static uint8 group = 0;
  
  for (int i = group * 32; i < (group + 1) * 32; i++)
      ANO_DT_send_int16((int16_t)(100 * sample_sx[i].re), (int16_t)(100 * sample_dx[i].re), (int16_t)(100 * sample_sy[i].re), (int16_t)(100 * sample_dy[i].re), 0, 0, 0, 0);
  
  group++;
  
  if (group == _N/32) {
    DMA_EN(DMA_CH0); 
    DMA_EN(DMA_CH2); 
    
    group = 0;
    t4wft2_can_xUART.flag = 0;
    t4wft3_can_yUART.flag = 0;
    
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
  }
  
  
  /*ANO_DT_send_int16(t4wft2_can_xUART.int16_val, t4wft3_can_yUART.int16_val, 0, 0, 0, 0, 0, 0);
  
  t4wft2_can_xUART.flag = 0;
  t4wft3_can_yUART.flag = 0;
  */
  printf("task4_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK  
  
  
SUBTASK_END  
    
}



/***********************************************************************/
/***********************************************************************/





/**
 * @brief       ����״̬���º���
 * @param       
 * @return
 * @example
 * @note        �ú����������жϺ������Ϊ������ĵĴ�������
 *
 */
void task_rhythm(void)
{
  for (int i = 0; i < TASK_NUM; i++) {
    
    if (tasks[i].task_timer != 0) {
      tasks[i].task_timer--;
      
      if(tasks[i].task_timer == 0) {
        tasks[i].task_status_flag = TASK_RUN;
        tasks[i].task_timer = tasks[i].task_timer_init_val;
        active_task_num++;
      }
      
    }    
  }  
  //printf("\n*********\n");      //only for debug
}


/**
 * @brief       ��������
 * @param
 * @return
 * @exanple
 * @note        ��������Ĳ����ڴ˴����Ƕ��壬����ʵ���������������䡣
 *
 */
void task_process(void)
{
  while(active_task_num != 0) {
    for (int i = 0; i < TASK_NUM; i++) {
      
      if (tasks[i].task_status_flag == TASK_RUN) {
        tasks[i].task_entry(tasks[i].arg);
        tasks[i].task_status_flag = TASK_DELAY;
        active_task_num--;
      }
      
    }
  }
}

/**
 * @brief       ��������жϵĳ�ʼ��
 * @param       ms����������
 * @return
 * @example
 * @note        ʹ�� PIT0
 *
 */
void task_rhythm_init(uint32_t ms)
{
  PIT_Init(PIT0, ms);
  NVIC_EnableIRQ(PIT0_IRQn);			          //ʹ��PIT0_IRQn���ж�
}


/**
 * @brief       PIT1 ��ʱ����������������ʱ���Ĳ���
 * @param      
 * @return
 * @example
 * @note        ʹ�� PIT1 
 * @date        2020/7/7
 *
 */
static void PIT1_start_count(void)
{
  //PIT �õ��� Bus Clock ����Ƶ�ʣ�36M

    /* ����ʱ��*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                          

    /* ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼������� */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );   
    
    /* �������¼�����ʼֵ��Ϊ��� 0xffffffff*/  
    PIT_LDVAL(PIT1)  = 0xffffffffu;     
     
    /* ʹ�� PITn��ʱ�� */
    PIT_TCTRL(PIT1) |= PIT_TCTRL_TEN_MASK;   

}

/**
 * @brief       ��ȡ����ʱ��
 * @param
 * @return      ����ʱ�䣬��λΪ ms��
 * @example
 * @note
 *
 */
static uint32_t PIT1_get_time(void)
{
    uint32_t time;

    if(PIT_TFLG1 & PIT_TFLG_TIF_MASK) {      //�Ѿ������
      time = ~((uint32_t)0);                          //���� 0xffffffff ��ʾ����
      PIT_TFLG1 |= PIT_TFLG_TIF_MASK;            //д 1 ��������־λ
    }
    else
      time = (uint32_t)((0xffffffffu - PIT_CVAL1) / (bus_clk * 1e3)); //���е�λ���㣬ms
    
    PIT_TCTRL(PIT1) &= ~PIT_TCTRL_TEN_MASK;     //�ر�PIT1 

    return time;
}


