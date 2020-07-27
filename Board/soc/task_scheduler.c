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
 * @file_note    各个任务全部执行一遍的时间不能超过任务时钟节拍
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
 * @brief       任务入口函数声明
 */
void task0_entry(void *arg);
void task1_entry(void *arg);
void task2_entry(void *arg);

/**
 * @brief       任务实例定时器初始值
 */
typedef enum {
  TASK0_TIMER_INIT_VAL = 5,
  TASK1_TIMER_INIT_VAL = 10,
  TASK2_TIMER_INIT_VAL = 20
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

/**********************************************************************/
/*************************任务入口函数*********************************/
/**********************************************************************/
/**
 * @brief       任务1入口函数，100ms 点一次灯
 */
void task0_entry(void *arg)
{
  LED_Reverse(0);
}

/**
 * @brief       任务1入口函数，200ms 点一次灯
 */
void task1_entry(void *arg)
{
  LED_Reverse(1);
}

/**
 * @brief       任务2入口函数，400ms 点一次灯
 */
void task2_entry(void *arg)
{
  LED_Reverse(2);
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




