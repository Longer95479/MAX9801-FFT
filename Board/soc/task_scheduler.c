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
 * 
 */

#include "include.h"

   
/**********************************************************************/
/*************************任务实例初始化处*****************************/
/**********************************************************************/   
/**
 * @brief       任务个数
 */
#define TASK_NUM        3
   
/**
 * @brief       任务间通讯标志，1 表示资源已生成，0 表示资源还未生成
 */
static task_commu_t task1_wf_task0;

/**
 * @brief       任务入口函数声明
 */
void task0_entry(void *arg);
void task1_entry(void *arg);
void task2_entry(void *arg);

/**
 * @brief       任务实例定时器初始值
 */
typedef enum {
  TASK0_TIMER_INIT_VAL = 1,
  TASK1_TIMER_INIT_VAL = 1,
  TASK2_TIMER_INIT_VAL = 50
} task_timer_init_val_instance;
   
   
/**
 * @brief       任务控制块实例，对其初始化
 *
 */
static task_t tasks[TASK_NUM] = {
  {TASK_DELAY, TASK0_TIMER_INIT_VAL, TASK0_TIMER_INIT_VAL, NULL, task0_entry},
  {TASK_DELAY, TASK1_TIMER_INIT_VAL, TASK1_TIMER_INIT_VAL, NULL, task1_entry},
  {TASK_DELAY, TASK2_TIMER_INIT_VAL, TASK2_TIMER_INIT_VAL, NULL, task2_entry},
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
 * @brief       任务1入口函数，数据重处理
 */
void task0_entry(void *arg)
{
CREAT_SUBTASK
SUBTASK_BEGIN

SUBTASK_CASE SUBTASK_STATUS0:
  PIT1_start_count();
  
  if (x_ready_for_fft == 1) {
    data_type_trans(sample_sx);
    data_reprocess(sample_sx, sample_dx);
    
    DMA_EN(DMA_CH0); 
    x_ready_for_fft = 0;
    
    LED_Reverse(0);
  }
  
  if (y_ready_for_fft == 1) {
    data_type_trans(sample_sy);
    data_reprocess(sample_sy, sample_dy);
    
    DMA_EN(DMA_CH2); 
    y_ready_for_fft = 0;
    
    LED_Reverse(0);
  }  
  
  printf("task0_SUBTASK_STATUS0 running time: %u.\n", PIT1_get_time());
  
SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS1;
SUBTASK_BREAK

SUBTASK_CASE SUBTASK_STATUS1:

  printf("task0_SUBTASK_STATUS1 is running.\n");
  
SUBTASK_STATUS_CHANGE_TO SUBTASK_STATUS0;
SUBTASK_BREAK

SUBTASK_END
}


/**
 * @brief       任务1入口函数，100ms 点一次灯
 */
void task1_entry(void *arg)
{
  LED_Reverse(1);
  printf("task1 is running.\n");
}


/**
 * @brief       任务2入口函数，400ms 点一次灯
 */
void task2_entry(void *arg)
{ 
  LED_Reverse(2);
  printf("task2 is running\n\n");
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
      }
      
    }    
  }  
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
  for (int i = 0; i < TASK_NUM; i++) {
    
    if (tasks[i].task_status_flag == TASK_RUN) {
      tasks[i].task_entry(tasks[i].arg);
      tasks[i].task_status_flag = TASK_DELAY;
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


