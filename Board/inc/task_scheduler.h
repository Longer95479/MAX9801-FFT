#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H


/**
 * @brief       子任务控制流程宏定义
 */
#define CREAT_SUBTASK                   static subtask_status_t subtask_status = SUBTASK_STATUS0;
#define SUBTASK_BEGIN                   switch(subtask_status) {
#define SUBTASK_CASE                    case                    //引用此句时记得添加冒号
#define SUBTASK_BREAK                   break;
#define SUBTASK_END                     }
#define SUBTASK_STATUS_CHANGE_TO         subtask_status =        //在引用此句时末尾记得添加逗号


/**
 * @brief       任务状态枚举
 */
typedef enum {
  TASK_RUN,
  TASK_DELAY
} task_status_t;

/**
 * @brief       任务间同步变量
 */
typedef struct {
  uint8_t       flag;
  int16_t       int16_val;
  uint16_t      uint16_val;
  int32_t       int32_val;
  uint32_t      uint32_val;
  float         float_val;
} task_commu_t;

/**
 * @brief       子任务状态枚举
 */
typedef enum {
  SUBTASK_STATUS0,
  SUBTASK_STATUS1,
  SUBTASK_STATUS2,
  SUBTASK_STATUS3,
  SUBTASK_STATUS4,
  SUBTASK_STATUS5,
  SUBTASK_STATUS6,
  SUBTASK_STATUS7  
} subtask_status_t;


/**
 * @brief       任务控制句柄
 */
typedef struct task{
  
  task_status_t task_status_flag; //任务状态
  
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
