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

//#define TASK_TIME

/**********************************************************************/
/*************************任务实例初始化处*****************************/
/**********************************************************************/   
/**
* @brief       任务个数
*/
#define TASK_NUM        3

/**
* @brief       任务活跃数，当不为0时激活任务调度器
*/
static uint8_t active_task_num = 0;

/**
* @brief       任务间通讯结构体，1 表示资源已生成，0 表示资源还未生成
*/
#ifdef NO_BYE
static task_commu_t t2wft1_can_xfft = {0};
static task_commu_t t3wft1_can_yfft = {0};
static task_commu_t t4wft2_can_xUART = {0};
static task_commu_t t4wft3_can_yUART = {0};
static task_commu_t t8wft4_theta = {0};
#endif  //NO_BYE


/**
* @brief       任务入口函数声明
*/
void task0_entry(void *arg);
void task1_entry(void *arg);
void task2_entry(void *arg);
void task3_entry(void *arg);
void task4_entry(void *arg);
void task5_entry(void *arg);
void task6_entry(void *arg);

/**
* @brief       任务实例定时器初始值
*/
typedef enum {
  TASK0_TIMER_INIT_VAL = 25,
  TASK1_TIMER_INIT_VAL = 1,
  TASK2_TIMER_INIT_VAL = 1,
  TASK3_TIMER_INIT_VAL = 1,
  TASK4_TIMER_INIT_VAL = 6,
  TASK5_TIMER_INIT_VAL = 5,
  TASK6_TIMER_INIT_VAL = 5
    
} task_timer_init_val_instance;


/**
* @brief       任务控制块实例，对其初始化
* @note        列的入口函数顺序就是任务执行的顺序，若要调整优先级，调整列顺序即可
*
*/
static task_t tasks[TASK_NUM] = {
  {TASK_DELAY, TASK0_TIMER_INIT_VAL, TASK0_TIMER_INIT_VAL, NULL, task0_entry},
  {TASK_DELAY, TASK1_TIMER_INIT_VAL, TASK1_TIMER_INIT_VAL, NULL, task1_entry},
  {TASK_DELAY, TASK2_TIMER_INIT_VAL, TASK2_TIMER_INIT_VAL, NULL, task2_entry},
  /*{TASK_DELAY, TASK3_TIMER_INIT_VAL, TASK3_TIMER_INIT_VAL, NULL, task3_entry},
  {TASK_DELAY, TASK4_TIMER_INIT_VAL, TASK4_TIMER_INIT_VAL, NULL, task4_entry},
  {TASK_DELAY, TASK5_TIMER_INIT_VAL, TASK5_TIMER_INIT_VAL, NULL, task5_entry},
  {TASK_DELAY, TASK6_TIMER_INIT_VAL, TASK6_TIMER_INIT_VAL, NULL, task6_entry}*/
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
 * @brief       任务1入口函数，检测任务是否为停止状态，若是，则开始 theta 采样计时
 * @note        theta_sample_time = TASK_RYM * TASK2_TIMER_INIT_VAL * THETA_SAMPLE_TIME_CNT
 */
#define THETA_SAMPLE_TIME_CNT 50

void task1_entry(void *arg)
{
  static int16_t theta_sample_time_cnt = 0;
  
  if (car_status == CAR_STOP) {    
    theta_sample_time_cnt++;
    LED_ON(1);          //仅用于测试
    
    if (theta_sample_time_cnt == THETA_SAMPLE_TIME_CNT) {  
      // 如果幅值大于等于则进入校
      last_car_status_record();
      car_status = CAR_RUN;
      
      theta_sample_time_cnt = 0;
      LED_OFF(1);          //仅用于测试
    }
    
  }
}


/**
 * @brief       任务2入口函数，检测车状态是否为校准模式下运动，若是，则进行计时
 * @note        car_run_time =  TASK_RYM * TASK3_TIMER_INIT_VAL * CAR_RUN_TIME_CNT
 */
#define CAR_RUN_TIME_CNT 80

void task2_entry(void *arg)
{
  static int16_t car_run_time_cnt = 0;
  
  if (car_status == CAR_RUN) {
    car_run_time_cnt++;
    
    if (car_run_time_cnt == CAR_RUN_TIME_CNT) {
      last_car_status_record();
      car_status = CAR_STOP;
      
      car_run_time_cnt = 0;
    }
  }
}


/**
 * 
 */
void task3_entry(void *arg) {
  ;
}



#ifdef AMP_STOP

/**
 * @brief       任务1入口函数，检测信号幅值是否低于阈值，若低于阈值，则把车状态设置为 CAR_STOP
 */
#define THRESHOLD 4500.0f

void task1_entry(void *arg)
{
  if (car_status == CAR_NORMAL_RUN)
    if (amplitude[4] < THRESHOLD) {
      last_car_status_record();
      car_status = CAR_STOP;
    }
}


/**
 * @brief       任务2入口函数，检测任务是否为停止状态，若是，则开始 theta 采样计时
 * @note        theta_sample_time = TASK_RYM * TASK2_TIMER_INIT_VAL * THETA_SAMPLE_TIME_CNT
 */
#define THETA_SAMPLE_TIME_CNT 60

void task2_entry(void *arg)
{
  static int16_t theta_sample_time_cnt = 0;
  
  if (car_status == CAR_STOP) {    
    theta_sample_time_cnt++;
    LED_ON(1);          //仅用于测试
    
    if (theta_sample_time_cnt == THETA_SAMPLE_TIME_CNT) {  
      // 如果幅值大于等于则进入校
      if (amplitude[4] >= THRESHOLD) {
        last_car_status_record();
        car_status = CAR_NORMAL_RUN;
      }
      
      else {
        last_car_status_record();
        car_status = CAR_RUN;
      }
      
      theta_sample_time_cnt = 0;
      LED_OFF(1);          //仅用于测试
    }
    
  }
}


/**
 * @brief       任务3入口函数，检测车状态是否为校准模式下运动，若是，则进行计时
 * @note        car_run_time =  TASK_RYM * TASK3_TIMER_INIT_VAL * CAR_RUN_TIME_CNT
 */
#define CAR_RUN_TIME_CNT 85

void task3_entry(void *arg)
{
  static int16_t car_run_time_cnt = 0;
  
  if (car_status == CAR_RUN) {
    car_run_time_cnt++;
    
    if (car_run_time_cnt == CAR_RUN_TIME_CNT) {
      last_car_status_record();
      car_status = CAR_STOP;
      
      car_run_time_cnt = 0;
    }
  }
}


/**
 * @brief       任务4入口函数，检测幅值是否在一段时间内相等
 * @note        AMP_CHECK_TIME = AMP_CHECK_TIME_CNT * TASK_TIME * TASK4_TIMER_INIT_VAL
 */
#define AMP_CHECK_TIME_CNT 9
#define AMP_CHECK_THRESHOLD 20000.0f            //幅值小于一定值开始检测是否卡车
#define AMP_VAR_THRESHOLD  250000.0f           //阈值的选取根据实验结果确定，静止时的方差大致小于 

void task4_entry(void *arg)
{
  static float amp_queue[AMP_CHECK_TIME_CNT];
  static int8_t amp_order = 0;
  static float amp_mean, amp_var;       // 样本均值和方差
  
  //车状态在运行状态下，且未进入 CAR_BACK_ROUND 
  if (car_status == CAR_RUN || car_status == CAR_NORMAL_RUN) {
    
    // 如果幅值小于一定值状态，说明还未靠近信标
    if (amplitude[4] < AMP_CHECK_THRESHOLD) {
      
      // 判断进入数组的幅值个数是否到达了 AMP_CHECK_TIME_CNT 个
      if (amp_order < AMP_CHECK_TIME_CNT) {
        amp_queue[amp_order] = amplitude[4];
        amp_order++;
      }
      else {
        amp_order = 0;              // 次序清零
        
        // 计算幅值平均值
        for (int i = 0; i < AMP_CHECK_TIME_CNT; i++) {
          amp_mean += amp_queue[i];
        }      
        amp_mean /= AMP_CHECK_TIME_CNT;
        
        // 计算幅值方差
        static float amp_diff;
        for (int i = 0; i < AMP_CHECK_TIME_CNT; i++) {
          amp_diff = amp_queue[i] - amp_mean;
          amp_var += amp_diff * amp_diff;
        }
        amp_var /= AMP_CHECK_TIME_CNT - 1;
        
        // 如果方差小于一定值，说明幅值比较稳定，车极大可能卡住了
        if (amp_var < AMP_VAR_THRESHOLD) {
          last_car_status_record();
          car_status = CAR_BACK;
        }
      }   
      
    }
  }
  
}


/**
 * @brief       任务5入口函数，检测车是否处于后退状态，若是，则进行计时
 * @note        CAR_BACK_TIME = TASK_RYM * TASK5_TIMER_INIT_VAL * CAR_BACK_TIME_CNT
 */
#define CAR_BACK_TIME_CNT 4

void task5_entry(void *arg)
{
  static int16_t car_back_time_cnt = 0;
  if (car_status == CAR_BACK) {
    car_back_time_cnt++;
    
    LED_ON(3);          //仅用于测试
    
    if (car_back_time_cnt == CAR_BACK_TIME_CNT) {
      car_back_time_cnt = 0;
      
      last_car_status_record();
      car_status = CAR_OBLIQUE;
    }
    
  }
}


/**
 * @brief       任务6入口函数，检测车是否处于斜进状态，若是，开始计时
 */
#define CAR_OBLIQUE_TIME_CNT 4

void task6_entry(void *arg)
{
  static int16_t car_oblique_time_cnt = 0;
  
  if (car_status == CAR_OBLIQUE) {
    car_oblique_time_cnt++;
    
    if (car_oblique_time_cnt == CAR_OBLIQUE_TIME_CNT) {
      last_car_status_record();
      car_status = CAR_NORMAL_RUN;
      
      car_oblique_time_cnt = 0;
      
      LED_OFF(3);               //仅用于测试
    }
    
  }
}


/**
 * @brief       检测模式标志位
 */
void task7_entry(void *arg)
{
  ;
}


#endif


#ifdef NO_BYE
/**
* @brief       任务1入口函数，数据重处理
*/
void task1_entry(void *arg)
{
#ifdef TASK_TIME  
  PIT1_start_count();
#endif  //TASK_TIME
  
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
  
#ifdef TASK_TIME  
  printf("task1 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
  
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
    
#ifdef TASK_TIME
    PIT1_start_count();
#endif  //TASK_TIME
    
    if (t2wft1_can_xfft.flag == 1) {
      t2wft1_can_xfft.flag = 0;
      
      amplitude_and_mean_process(sample_sx);
      amplitude_and_mean_process(sample_dx);
      
      FFT(sample_sx, kWnk_fft, _N); 
      
      SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
    }
    
#ifdef    TASK_TIME
    printf("task2_subtask0 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
    
    SUBTASK_BREAK
      
      
      //! sample_dx 的 FFT，频域滤波
      SUBTASK_CASE SUBTASK_STATUS1:
      PIT1_start_count();
      
      FFT(sample_dx, kWnk_fft, _N);
      low_pass_filter(sample_sx);
      low_pass_filter(sample_dx);
      
      SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
      
#ifdef  TASK_TIME
      printf("task2_subtask1 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
      
      SUBTASK_BREAK
        
        
        //! 广义互相关处理
        SUBTASK_CASE SUBTASK_STATUS2:
        
#ifdef  TASK_TIME
        PIT1_start_count();
#endif  //TASK_TIME
        
        // 广义互相关
        for (int i = 0; i < _N; i++) {
          //**z_x[i] = complex_mult(sample_sx[i], sample_dx[i]);
          z_x[i].re = sample_sx[i].re * sample_dx[i].re - sample_sx[i].im * sample_dx[i].im;
          z_x[i].im = sample_sx[i].re * sample_dx[i].im + sample_sx[i].im * sample_dx[i].re;
          
          static float A = 1;
          //**A =  100 * InvSqrt(pow(z_x[i].re, 2) + pow(z_x[i].im, 2));
          A =  InvSqrt(z_x[i].re * z_x[i].re + z_x[i].im * z_x[i].im);
          z_x[i].re = z_x[i].re * A;
          z_x[i].im = z_x[i].im * A;
        }
        
        DMA_EN(DMA_CH0);    //此时即可开始下一轮的采集
        //printf("DMA_ERQ_ERQ0 = %d\n", (int8_t)(DMA_ERQ & (DMA_ERQ_ERQ0_MASK<<(DMA_CH0))));  //仅用于测试
        
        //逆傅里叶变换回时域
        IFFT(z_x, kWnk_ifft, _N);
        
        // max的中位值滤波
        static int16 max_now, max_queue[9] = {0}; //队列数组，0进
        max_now = 2038;               ////2038 - _N/2 + 1 = -9          2056 -_N/2 + 1 = 9
        for (int i = 2039; i < 2056; i++)
          if (z_x[i].re > z_x[max_now].re)
            max_now = i;
        
        t4wft2_can_xUART.int16_val = midst_filter(max_now, max_queue, 3) - _N/2 + 1;
        t4wft2_can_xUART.flag = 1;
        
        SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
        
#ifdef  TASK_TIME
        printf("task2_subtask2 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
        
        SUBTASK_BREAK  
          
          
          //! z 的 IFFT，查找互相关结果最大值的索引
          SUBTASK_CASE SUBTASK_STATUS3:
          
#ifdef  TASK_TIME
          PIT1_start_count();
#endif  //TASK_TIME
          /*
          IFFT(z_x, kWnk_ifft, _N);
          
          // max的中位值滤波
          static int16 max_now, max_queue[9] = {0}; //队列数组，0进
          max_now = 0;
          for (int i = 0; i < _N; i++)
          if (z_x[i].re > z_x[max_now].re)
          max_now = i;
          
          t4wft2_can_xUART.int16_val = midst_filter(max_now, max_queue, 3) - _N/2 + 1;
          t4wft2_can_xUART.flag = 1;
          */
          SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
          
#ifdef  TASK_TIME
          printf("task2_subtask3 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
          
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
    
#ifdef  TASK_TIME
    PIT1_start_count();
#endif  //TASK_TIME
    
    if (t3wft1_can_yfft.flag == 1) {
      t3wft1_can_yfft.flag = 0;
      
      amplitude_and_mean_process(sample_sy);
      amplitude_and_mean_process(sample_dy);
      
      FFT(sample_sy, kWnk_fft, _N); 
      
      SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
    }
    
#ifdef  TASK_TIME
    printf("task3_subtask0 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
    
    SUBTASK_BREAK
      
      
      //! sample_dy 的 FFT，频域滤波
      SUBTASK_CASE SUBTASK_STATUS1:
      
#ifdef  TASK_TIME
      PIT1_start_count();
#endif  //TASK_TIME
      
      FFT(sample_dy, kWnk_fft, _N);
      low_pass_filter(sample_sy);
      low_pass_filter(sample_dy);
      
      SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
      
#ifdef  TASK_TIME
      printf("task3_subtask1 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
      
      SUBTASK_BREAK
        
        
        //! 广义互相关
        SUBTASK_CASE SUBTASK_STATUS2:
        
#ifdef  TASK_TIME
        PIT1_start_count();
#endif  //TASK_TIME
        
        // 广义互相关
        for (int i = 0; i < _N; i++) {
          //**z_y[i] = complex_mult(sample_sy[i], sample_dy[i]);
          z_y[i].re = sample_sy[i].re * sample_dy[i].re - sample_sy[i].im * sample_dy[i].im;
          z_y[i].im = sample_sy[i].re * sample_dy[i].im + sample_sy[i].im * sample_dy[i].re;
          
          static float A = 1;
          //**A =  100 * InvSqrt(pow(z_y[i].re, 2) + pow(z_y[i].im, 2));
          A =  InvSqrt(z_y[i].re * z_y[i].re + z_y[i].im * z_y[i].im);
          z_y[i].re = z_y[i].re * A;
          z_y[i].im = z_y[i].im * A;
        }
        
        DMA_EN(DMA_CH2);    //此时即可立刻开始下一轮的采集
        
        //逆傅里叶变换回时域
        IFFT(z_y, kWnk_ifft, _N);
        
        // max的中位值滤波
        static int16 max_now, max_queue[9] = {0}; //队列数组，0进
        max_now = 2038;       //2038 - _N/2 + 1 = -9          2056 -_N/2 + 1 = 9
        for (int i = 2039; i < 2056; i++)
          if (z_y[i].re > z_y[max_now].re)
            max_now = i;
        
        t4wft3_can_yUART.int16_val = midst_filter(max_now, max_queue, 3) - _N/2 + 1;
        t4wft3_can_yUART.flag = 1;
        
        SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
        
#ifdef  TASK_TIME
        printf("task3_subtask2 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
        
        SUBTASK_BREAK
          
          
          //! z 的 IFFT，查找互相关结果最大值的索引
          SUBTASK_CASE SUBTASK_STATUS3:
#ifdef  TASK_TIME
          PIT1_start_count();
#endif  //TASK_TIME
          /*
          IFFT(z_y, kWnk_ifft, _N);
          
          // max的中位值滤波
          static int16 max_now, max_queue[9] = {0}; //队列数组，0进
          max_now = 0;
          for (int i = 0; i < _N; i++)
          if (z_y[i].re > z_y[max_now].re)
          max_now = i;
          
          t4wft3_can_yUART.int16_val = midst_filter(max_now, max_queue, 3) - _N/2 + 1;
          t4wft3_can_yUART.flag = 1;
          */
          SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
          
#ifdef  TASK_TIME
          printf("task3_subtask3 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
          
          SUBTASK_BREAK
            
            
            SUBTASK_END    
}


/**
* @brief       任务4入口函数，生成 theta
*/
void task4_entry(void *arg)
{
  CREAT_SUBTASK
    SUBTASK_BEGIN
      
      // 检测用于传输的数据是否生成
      SUBTASK_CASE SUBTASK_STATUS0:
    
#ifdef  TASK_TIME
    PIT1_start_count();
#endif  //TASK_TIME
    
    if (t4wft2_can_xUART.flag & t4wft3_can_yUART.flag == 1)
      SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
    
#ifdef  TASK_TIME
    printf("task4_subtask0 running time: %u.\n", PIT1_get_time());
#endif
    
    SUBTASK_BREAK
      
      //！计算theta
      SUBTASK_CASE SUBTASK_STATUS1:
      
#ifdef  TASK_TIME
      PIT1_start_count();
#endif  //TASK_TIME
      
      ;     //此句是为了消除 Warning[Pe1072]: a declaration cannot have a label     
      static float dx, dy, /*x, y,*/ theta, theta_queue[9] = {0}, theta_xhat, theta_yhat;
      
      if (t4wft2_can_xUART.int16_val == 0) {
        if (t4wft3_can_yUART.int16_val > 0)
          theta = PI/2;
        else
          theta = -PI/2;
      }
      
      else if (t4wft3_can_yUART.int16_val == 0) {
        if (t4wft2_can_xUART.int16_val > 0)
          theta = 0;
        else
          theta = PI;
      }
      
      else {
        // 对最大值索引进行限制，防止出现距离差大于麦克风距离的情况
        if (t4wft2_can_xUART.int16_val > 8)
          t4wft2_can_xUART.int16_val = 8;
        else if(t4wft2_can_xUART.int16_val < -8)
          t4wft2_can_xUART.int16_val = -8;
        
        if (t4wft3_can_yUART.int16_val > 8)
          t4wft3_can_yUART.int16_val = 8;
        else if(t4wft3_can_yUART.int16_val < -8)
          t4wft3_can_yUART.int16_val = -8;
        
        dx = 0.034f * t4wft2_can_xUART.int16_val;
        dy = 0.034f * t4wft3_can_yUART.int16_val;
        
        if (dy > 0)
          theta_xhat = acos(dx/_L);
        else
          theta_xhat = -acos(dx/_L);
        
        if (dx < 0)
          if (dy > 0)
            theta_yhat = PI/2 + acos(dy/_L);
          else
            theta_yhat = -3*PI/2 + acos(dy/_L);
        else
          theta_yhat =PI/2 - acos(dy/_L);
        
        theta = (theta_xhat + theta_yhat) / 2;
      }
      
      
      /*  
      x = dx / (2*_L) / InvSqrt(((_L*_L - dy*dy) * (_L*_L + dy*dy - dx*dx)) / (_L*_L - dx*dx -dy*dy));
      y = dy / (2*_L) / InvSqrt(((_L*_L - dx*dx) * (_L*_L + dx*dx - dy*dy)) / (_L*_L - dx*dx -dy*dy));
      
      theta = atan2(y, x);
      if (theta > 3.13f || theta < -3.13f)
      if (t4wft2_can_xUART.int16_val > 0)
      theta = 0;
    else
      theta = PI;
      */
      
      //滤波 + 通讯部分
      static uint8_t num_only_test = 3;
      t8wft4_theta.float_val = f_midst_filter(theta, theta_queue, num_only_test);
      //**t8wft4_theta.float_val = theta;
      if (car_status == CAR_STOP) {
        t8wft4_theta.flag = 1;          //生成标志
        t8wft4_theta.int16_val ++;    //theta生成计数
        
        LED_OFF(1);         //用于测试
      }
      
      //**ANO_DT_send_int16(t4wft2_can_xUART.int16_val, t4wft3_can_yUART.int16_val, (int16_t)(x * 100), (int16_t)(y * 100), (int16_t)(theta * 100), 0, 0, 0);              //仅用于测试
#if 0
      ANO_DT_send_int16(t4wft2_can_xUART.int16_val, t4wft3_can_yUART.int16_val, (int16_t)(theta * 100), 0, 0, 0, 0, 0);          
#endif
      
      t4wft2_can_xUART.flag = 0;
      t4wft3_can_yUART.flag = 0;
      
      SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
      
#ifdef  TASK_TIME
      printf("task4_subtask1 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
      
      SUBTASK_BREAK  
        
        
        SUBTASK_END  
          
}


/**
* @brief       任务8入口函数，速度控制
*/
void task8_entry(void *arg)
{
  static float vx, vy, wheel_v[4];
  
  static float theta_mean = 0;         //在进入停车状态前记得清零
  
  static const int8_t sum_time = 4;           //theta累加次数
  
  static const uint8_t k_car_run_last_time = 15;
  static uint8_t car_run_last_time;        //校准模式下的开车时间，car_run_last_time * TASK_RHYTHM_T * TIMER_INIT_VAL
  
  static const uint8_t k_car_normal_run_last_time = 3;         //正常模式持续时间，即使用单次计算得到的角度作为方位，时间计算同上
  static uint8_t car_normal_run_last_time = 3;
  
  
  CREAT_SUBTASK
    SUBTASK_BEGIN
      
      //检测是否到停车时间，若是则校准方位，若不是则直接使用单次计算的角度
      SUBTASK_CASE SUBTASK_STATUS0:
    
#ifdef  TASK_TIME
    PIT1_start_count();
#endif  //TASK_TIME
    
    if (car_status == CAR_STOP) {
      
      if (t8wft4_theta.flag == 1) {
        theta_mean += ((float)t8wft4_theta.int16_val / ((sum_time + 1) * sum_time / 2) * t8wft4_theta.float_val);      //越往后的数据越可靠，权重越大。记得进行类型提升（加float）！！
        t8wft4_theta.flag = 0;          // 此次theta已被消耗
      }
      
      if (t8wft4_theta.int16_val == sum_time) {
        t8wft4_theta.int16_val = 0;         //theta 计数清零
        
        car_status = CAR_RUN;                //即将进入运动状态
        
        //theta_mean /= sum_time;
        
        //ANO_DT_send_int16((int16_t)(100 * theta_mean), 0, 0, 0, 0, 0, 0, 0);        //仅用于测试
        
        SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
      }
    }
    else {
      
      vx = 4.5f * cos(t8wft4_theta.float_val);         //rps * cos(theta)
      vy = 4.5f * sin(t8wft4_theta.float_val);         //rps * sin(theta)
      
      wheel_v[0] = vx + vy;
      wheel_v[1] = -vx + vy;
      wheel_v[2] = vx + vy;
      wheel_v[3] = -vx + vy;
      
      for (int i = 0; i < 4; i++)
        pid[i].SetSpeed = wheel_v[i];
      
      car_normal_run_last_time--;
      if (car_normal_run_last_time == 0) {
        car_status = CAR_STOP;
        car_stop();
      }
    }
    
#ifdef  TASK_TIME
    printf("task8_subtask0 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
    
    SUBTASK_BREAK
      
      
      // 计算四轮速度
      SUBTASK_CASE SUBTASK_STATUS1: 
      
#ifdef  TASK_TIME
      PIT1_start_count();
#endif  //TASK_TIME
      
      ;     //此句是为了消除 Warning[Pe1072]: a declaration cannot have a label
      
      vx = 5 * cos(theta_mean);         //rps * cos(theta)
      vy = 5 * sin(theta_mean);         //rps * sin(theta)
      
      wheel_v[0] = vx + vy;
      wheel_v[1] = -vx + vy;
      wheel_v[2] = vx + vy;
      wheel_v[3] = -vx + vy;
      
      for (int i = 0; i < 4; i++)
        pid[i].SetSpeed = wheel_v[i];
      
      car_run_last_time = k_car_run_last_time;        //k_car_run_last_time * TASK_RHYTHM_T * TIMER_INIT_VAL
      
#ifdef  TASK_TIME
      printf("task8_subtask1 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
      
      SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS2;
      SUBTASK_BREAK
        
        
        //开车状态
        SUBTASK_CASE SUBTASK_STATUS2: 
        
#ifdef  TASK_TIME
        PIT1_start_count();
#endif  //TASK_TIME
        
        LED_ON(1);      //仅用于测试
        
        if (!(--car_run_last_time)) {
          //car_slow();
          
          car_status = CAR_NORMAL_RUN;
          car_normal_run_last_time = k_car_normal_run_last_time;
          
          theta_mean = 0;
          
          SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
        }
        
#ifdef  TASK_TIME
        printf("task8_subtask2 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
        
        SUBTASK_BREAK
          
          
          SUBTASK_END
            
}


/**
* @brief       任务8入口函数，速度控制第二版本
*/
void task8_2_entry(void *arg)
{
  
#ifdef  TASK_TIME
  PIT1_start_count();
#endif  //TASK_TIME
  
  static float vx, vy, wheel_v[4];
  
  vx = 4 * cos(t8wft4_theta.float_val);         //rps * cos(theta)
  vy = 4 * sin(t8wft4_theta.float_val);         //rps * sin(theta)
  
  wheel_v[0] = vx + vy;
  wheel_v[1] = -vx + vy;
  wheel_v[2] = vx + vy;
  wheel_v[3] = -vx + vy;
  
  for (int i = 0; i < 4; i++)
    pid[i].SetSpeed = wheel_v[i];
  
#ifdef  TASK_TIME
  printf("task8_2 running time: %u.\n", PIT1_get_time());
#endif  //TASK_TIME
}


/**
 * @brief       遇障检测
 */
void task9_entry(void *arg)
{
  static uint8_t unnormal_count;
  if (fabs(rps_1 - rps_3) > 1.5)
    ;
    
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

#endif  //NO_BYE

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


