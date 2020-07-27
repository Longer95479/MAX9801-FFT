#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

/**
 * @brief       节拍中断周期
 */
#define TASK_RHYTHM_T   20      //20ms 进一次中断作为任务节拍

/**
 * @brief       任务状态枚举
 */
typedef enum {
  TASK_RUN,
  TASK_DELAY
} task_status;


/**
 * @brief       任务控制句柄
 */
typedef struct task{
  
  task_status task_status_flag; //任务状态
  
  uint32_t task_timer;  //任务计时器，为0时任务执行，单位为 一个时钟节拍
  uint32_t task_timer_init_val;  //任务计时器的初始值
  
  void *arg;    //任务入口函数的参数
  void (*task_entry) (void *arg);       //任务入口函数  
  
} task_t;



/**
 * @brief       任务状态更新函数
 * @param       
 * @return
 * @example
 * @note        该函数放置在中断函数里，作为任务节拍的处理内容
 *
 */
void task_rhythm(void);


/**
 * @brief       任务处理函数
 * @param
 * @return
 * @exanple
 * @note        任务输入的参数在此处不是定义，而是实例，因此依需求而变。
 *
 */
void task_process(void);


/**
 * @brief       任务节拍中断的初始化
 * @param       ms：节拍周期
 * @return
 * @example
 * @note        使用 PIT0
 *
 */
void task_rhythm_init(uint32_t ms);

#endif
