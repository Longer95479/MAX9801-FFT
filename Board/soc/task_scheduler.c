/**
 * @Platform    龙邱K66核心板        IAR 8.32.1
 * @Fielname    task_scheduler.c
 * @brief       任务调度器
 * @Author      Longer95479
 * @Email       371573369@qq.com
 * @Date        2020/7/27
 *
 */
   
/*!
 * @file_note    各个任务全部执行一遍的时间不能超过任务时钟节拍周期
 *              子任务运行时间方差较大的任务要放在靠后的位置
 *              运行时间越短的任务放在越靠前
 * 
 */

#include "include.h"

   
/**********************************************************************/
/*************************任务实例初始化处*****************************/
/**********************************************************************/   
/**
 * @brief       任务个数
 */
#define TASK_NUM        1

/**
 * @brief       任务活跃数，当不为0时激活任务调度器
 */
static uint8_t active_task_num = 0;
   
/**
 * @brief       任务间通讯结构体，1 表示资源已生成，0 表示资源还未生成
 */
static task_commu_t t2wft1_can_xfft = {0};
static task_commu_t t3wft1_can_yfft = {0};
static task_commu_t t4wft2_can_xUART = {0};
static task_commu_t t4wft3_can_yUART = {0};


/**
 * @brief       任务入口函数声明
 */
void task0_entry(void *arg);
void task1_entry(void *arg);
void task2_entry(void *arg);
void task3_entry(void *arg);
void task4_entry(void *arg);

/**
 * @brief       任务实例定时器初始值
 */
typedef enum {
  TASK0_TIMER_INIT_VAL = 1,
  TASK1_TIMER_INIT_VAL = 1,
  TASK2_TIMER_INIT_VAL = 1,
  TASK3_TIMER_INIT_VAL = 1,
  TASK4_TIMER_INIT_VAL = 2,
} task_timer_init_val_instance;
   
   
/**
 * @brief       任务控制块实例，对其初始化
 * @note        列的入口函数顺序就是任务执行的顺序，若要调整优先级，调整列顺序即可
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
 * @brief       子任务计时函数声明
 */
static void PIT1_start_count(void);
static uint32_t PIT1_get_time(void);

/**********************************************************************/
/*************************任务入口函数*********************************/
/**********************************************************************/
/**
 * @brief       任务0入口函数，一个节拍翻转一次灯，作为节拍直观监测
 */
void task0_entry(void *arg)
{ 
  LED_Reverse(2);
  //printf("task0 is running\n");
}



/**
 * @brief       任务1入口函数，数据重处理
 */
void task1_entry(void *arg)
{
  PIT1_start_count();
  
  if (x_ready_for_fft == 1) {
    data_type_trans(sample_sx);
    data_reprocess(sample_sx, sample_dx);
    
    //DMA_EN(DMA_CH0); 
    x_ready_for_fft = 0;
    t2wft1_can_xfft.flag = 1;    //数据处理完，可用于FFT
    
    //LED_Reverse(0);
  }
  
  if (y_ready_for_fft == 1) {
    data_type_trans(sample_sy);
    data_reprocess(sample_sy, sample_dy);
    
    //DMA_EN(DMA_CH2); 
    y_ready_for_fft = 0;
    t3wft1_can_yfft.flag = 1;    //数据处理完，可用于FFT
    
    //LED_Reverse(1);
  }  
  
  
  printf("task1 running time: %u.\n", PIT1_get_time());
  
}


/**
 * @brief       任务2入口函数，x轴的FFT运算
 */
void task2_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN


//! 波形标准化，sample_sx 的 FFT
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


//! sample_dx 的 FFT，频域滤波
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  
  FFT(sample_dx, kWnk_fft, _N);
  low_pass_filter(sample_sx);
  low_pass_filter(sample_dx);
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
  
  printf("task2_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! 广义互相关处理
SUBTASK_CASE SUBTASK_STATUS2:
  PIT1_start_count();
  // 广义互相关
  for (int i = 0; i < _N; i++) {
    z_x[i] = complex_mult(sample_sx[i], sample_dx[i]);
        
    static float A = 1;
    A =  100 * InvSqrt(pow(z_x[i].re, 2) + pow(z_x[i].im, 2));
    z_x[i].re = z_x[i].re * A;
    z_x[i].im = z_x[i].im * A;
  }
  
  DMA_EN(DMA_CH0);    //此时即可开始下一轮的采集
  //printf("DMA_ERQ_ERQ0 = %d\n", (int8_t)(DMA_ERQ & (DMA_ERQ_ERQ0_MASK<<(DMA_CH0))));  //仅用于测试
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS3;
  
  printf("task2_subtask2 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK  


//! z 的 IFFT，查找互相关结果最大值的索引
SUBTASK_CASE SUBTASK_STATUS3:
  PIT1_start_count();
  
  IFFT(z_x, kWnk_ifft, _N);
  
  // max的中位值滤波
  static int16 max_now, max_queue[9] = {0}; //队列数组，0进
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
 * @brief       任务3入口函数，y轴的FFT运算
 */
void task3_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN


//! 波形标准化，sample_sy 的 FFT
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


//! sample_dy 的 FFT，频域滤波
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  
  FFT(sample_dy, kWnk_fft, _N);
  low_pass_filter(sample_sy);
  low_pass_filter(sample_dy);
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
  
  printf("task3_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! 广义互相关
SUBTASK_CASE SUBTASK_STATUS2:
  PIT1_start_count();
  
  // 广义互相关
  for (int i = 0; i < _N; i++) {
    z_y[i] = complex_mult(sample_sy[i], sample_dy[i]);
    
    static float A = 1;
    A =  100 * InvSqrt(pow(z_y[i].re, 2) + pow(z_y[i].im, 2));
    z_y[i].re = z_y[i].re * A;
    z_y[i].im = z_y[i].im * A;
  }
  
  DMA_EN(DMA_CH2);    //此时即可立刻开始下一轮的采集
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS3;
  
  printf("task3_subtask2 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! z 的 IFFT，查找互相关结果最大值的索引
SUBTASK_CASE SUBTASK_STATUS3:
  PIT1_start_count();
  
  IFFT(z_y, kWnk_ifft, _N);
  
  // max的中位值滤波
  static int16 max_now, max_queue[9] = {0}; //队列数组，0进
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
 * @brief       任务4入口函数，用于串口发送数据
 */
void task4_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN

// 检测用于传输的数据是否生成
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t4wft2_can_xUART.flag & t4wft3_can_yUART.flag == 1)
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;

  printf("task4_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

//传输数据，传输完成后使能 DMA传输
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
 * @brief       任务5入口函数，x轴的FFT运算（显示波形版）
 */
void task5_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN


//! 波形标准化，sample_sx 的 FFT
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t2wft1_can_xfft.flag == 1) {
    t2wft1_can_xfft.flag = 0;
    
    //**DMA_EN(DMA_CH0);    //此时即可开始下一轮的采集
    
    amplitude_and_mean_process(sample_sx);
    amplitude_and_mean_process(sample_dx);
    
    FFT(sample_sx, kWnk_fft, _N); 
    
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
  }
  
  printf("task2_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! sample_dx 的 FFT，频域滤波
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  
  FFT(sample_dx, kWnk_fft, _N);
  low_pass_filter(sample_sx);
  low_pass_filter(sample_dx);
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
  
  printf("task2_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK


//! 广义互相关处理
SUBTASK_CASE SUBTASK_STATUS2:
  PIT1_start_count();
  // 广义互相关
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


//! z 的 IFFT，查找互相关结果最大值的索引
SUBTASK_CASE SUBTASK_STATUS3:
  PIT1_start_count();
/*  
  IFFT(z_x, kWnk_ifft, _N);
  
  // max的中位值滤波
  static int16 max_now, max_queue[9] = {0}; //队列数组，0进
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
 * @brief       任务6入口函数，y轴的FFT运算（显示波形版）
 */
void task6_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN

//! 波形标准化，sample_sx 的 FFT
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t3wft1_can_yfft.flag == 1) {
    t3wft1_can_yfft.flag = 0;
    
    //**DMA_EN(DMA_CH2);    //此时即可立刻开始下一轮的采集
    
    amplitude_and_mean_process(sample_sy);
    amplitude_and_mean_process(sample_dy);
    
    FFT(sample_sy, kWnk_fft, _N); 
    
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
  }
  
  printf("task3_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

//! sample_dy 的 FFT，频域滤波
SUBTASK_CASE SUBTASK_STATUS1:
  PIT1_start_count();
  
  FFT(sample_dy, kWnk_fft, _N);
  low_pass_filter(sample_sy);
  low_pass_filter(sample_dy);
  
  SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
  
  printf("task3_subtask1 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

//! 广义互相关
SUBTASK_CASE SUBTASK_STATUS2:
  PIT1_start_count();
  // 广义互相关
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


//! z 的 IFFT，查找互相关结果最大值的索引
SUBTASK_CASE SUBTASK_STATUS3:
  PIT1_start_count();
  /*
  IFFT(z_y, kWnk_ifft, _N);
  
  // max的中位值滤波
  static int16 max_now, max_queue[9] = {0}; //队列数组，0进
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
 * @brief       任务7入口函数，用于串口发送数据（显示波形版）
 */
void task7_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN

// 检测用于传输的数据是否生成
SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (t4wft2_can_xUART.flag & t4wft3_can_yUART.flag == 1)
    SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;

  printf("task4_subtask0 running time: %u.\n", PIT1_get_time());
SUBTASK_BREAK

//传输数据，传输完成后使能 DMA传输
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
 * @brief       任务状态更新函数
 * @param       
 * @return
 * @example
 * @note        该函数放置在中断函数里，作为任务节拍的处理内容
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
 * @brief       任务处理函数
 * @param
 * @return
 * @exanple
 * @note        任务输入的参数在此处不是定义，而是实例，因此依需求而变。
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
 * @brief       任务节拍中断的初始化
 * @param       ms：节拍周期
 * @return
 * @example
 * @note        使用 PIT0
 *
 */
void task_rhythm_init(uint32_t ms)
{
  PIT_Init(PIT0, ms);
  NVIC_EnableIRQ(PIT0_IRQn);			          //使能PIT0_IRQn的中断
}


/**
 * @brief       PIT1 计时，用于子任务运行时长的测试
 * @param      
 * @return
 * @example
 * @note        使用 PIT1 
 * @date        2020/7/7
 *
 */
static void PIT1_start_count(void)
{
  //PIT 用的是 Bus Clock 总线频率，36M

    /* 开启时钟*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                          

    /* 使能PIT定时器时钟 ，调试模式下继续运行 */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );   
    
    /* 设置向下计数初始值，为最大 0xffffffff*/  
    PIT_LDVAL(PIT1)  = 0xffffffffu;     
     
    /* 使能 PITn定时器 */
    PIT_TCTRL(PIT1) |= PIT_TCTRL_TEN_MASK;   

}

/**
 * @brief       获取运行时间
 * @param
 * @return      运行时间，单位为 ms，
 * @example
 * @note
 *
 */
static uint32_t PIT1_get_time(void)
{
    uint32_t time;

    if(PIT_TFLG1 & PIT_TFLG_TIF_MASK) {      //已经溢出了
      time = ~((uint32_t)0);                          //返回 0xffffffff 表示错误
      PIT_TFLG1 |= PIT_TFLG_TIF_MASK;            //写 1 清除溢出标志位
    }
    else
      time = (uint32_t)((0xffffffffu - PIT_CVAL1) / (bus_clk * 1e3)); //进行单位换算，ms
    
    PIT_TCTRL(PIT1) &= ~PIT_TCTRL_TEN_MASK;     //关闭PIT1 

    return time;
}


