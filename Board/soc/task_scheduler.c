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
 * @brief       �����ͨѶ��־��1 ��ʾ��Դ�����ɣ�0 ��ʾ��Դ��δ����
 */
static task_commu_t task1_wf_task0;

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
  TASK0_TIMER_INIT_VAL = 1,
  TASK1_TIMER_INIT_VAL = 1,
  TASK2_TIMER_INIT_VAL = 50
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

/**
 * @brief       �������ʱ��������
 */
static void PIT1_start_count(void);
static uint32_t PIT1_get_time(void);

/**********************************************************************/
/*************************������ں���*********************************/
/**********************************************************************/
/**
 * @brief       ����1��ں����������ش���
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
 * @brief       ����1��ں�����100ms ��һ�ε�
 */
void task1_entry(void *arg)
{
  LED_Reverse(1);
  printf("task1 is running.\n");
}


/**
 * @brief       ����2��ں�����400ms ��һ�ε�
 */
void task2_entry(void *arg)
{ 
  LED_Reverse(2);
  printf("task2 is running\n\n");
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


