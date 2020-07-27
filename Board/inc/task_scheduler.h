#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

/**
 * @brief       �����ж�����
 */
#define TASK_RHYTHM_T   20      //20ms ��һ���ж���Ϊ�������

/**
 * @brief       ����״̬ö��
 */
typedef enum {
  TASK_RUN,
  TASK_DELAY
} task_status;


/**
 * @brief       ������ƾ��
 */
typedef struct task{
  
  task_status task_status_flag; //����״̬
  
  uint32_t task_timer;  //�����ʱ����Ϊ0ʱ����ִ�У���λΪ һ��ʱ�ӽ���
  uint32_t task_timer_init_val;  //�����ʱ���ĳ�ʼֵ
  
  void *arg;    //������ں����Ĳ���
  void (*task_entry) (void *arg);       //������ں���  
  
} task_t;



/**
 * @brief       ����״̬���º���
 * @param       
 * @return
 * @example
 * @note        �ú����������жϺ������Ϊ������ĵĴ�������
 *
 */
void task_rhythm(void);


/**
 * @brief       ��������
 * @param
 * @return
 * @exanple
 * @note        ��������Ĳ����ڴ˴����Ƕ��壬����ʵ���������������䡣
 *
 */
void task_process(void);


/**
 * @brief       ��������жϵĳ�ʼ��
 * @param       ms����������
 * @return
 * @example
 * @note        ʹ�� PIT0
 *
 */
void task_rhythm_init(uint32_t ms);

#endif
