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
 * @file_note    ��������ȫ��ִ��һ���ʱ�䲻�ܳ�������ʱ�ӽ���
 * 
 */

#include "include.h"

   
/**********************************************************************/
/*************************����ʵ����ʼ����*****************************/
/**********************************************************************/   
/**
 * @brief       �������
 */
#define TASK_NUM        3

/**
 * @brief       ������ں�������
 */
void task0_entry(void *arg);
void task1_entry(void *arg);
void task2_entry(void *arg);

/**
 * @brief       ����ʵ����ʱ����ʼֵ
 */
typedef enum {
  TASK0_TIMER_INIT_VAL = 5,
  TASK1_TIMER_INIT_VAL = 10,
  TASK2_TIMER_INIT_VAL = 20
} task_timer_init_val_instance;
   
   
/**
 * @brief       ������ƿ�ʵ���������ʼ��
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
/*************************������ں���*********************************/
/**********************************************************************/
/**
 * @brief       ����1��ں�����100ms ��һ�ε�
 */
void task0_entry(void *arg)
{
  LED_Reverse(0);
}

/**
 * @brief       ����1��ں�����200ms ��һ�ε�
 */
void task1_entry(void *arg)
{
  LED_Reverse(1);
}

/**
 * @brief       ����2��ں�����400ms ��һ�ε�
 */
void task2_entry(void *arg)
{
  LED_Reverse(2);
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
      }
      
    }    
  }  
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
  for (int i = 0; i < TASK_NUM; i++) {
    
    if (tasks[i].task_status_flag == TASK_RUN) {
      tasks[i].task_entry(tasks[i].arg);
      tasks[i].task_status_flag = TASK_DELAY;
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




