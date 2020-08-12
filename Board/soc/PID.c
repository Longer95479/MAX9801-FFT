#define PID_GLOBALS

#include "include.h"
#include "PID.h"

/**
 * @brief       PID��ر�����ʼ�����趨��ز������ǲ�������Ϊ0
 * @param
 * @return
 * @example
 * @note        ֱ���ڳ����ڲ��޸�
 *
 */
void PID_init(void)
{
  
  pid[0].SetSpeed = 0; 
  pid[0].ActualSpeed = 0.0; 
  pid[0].err = 0.0; 
  pid[0].err_last = 0.0; 
  pid[0].err_next =0.0; 
  pid[0].Kp = 5; 
  pid[0].Ki = 9.2; 
  pid[0].Kd = 0;
  pid[0].epsilon = 0.5;
  
  pid[1].SetSpeed = 0; 
  pid[1].ActualSpeed = 0.0; 
  pid[1].err = 0.0; 
  pid[1].err_last = 0.0; 
  pid[1].err_next =0.0; 
  pid[1].Kp = 5; 
  pid[1].Ki = 10.5; 
  pid[1].Kd = 0; 
  pid[1].epsilon = 0.5;
  
  pid[2].SetSpeed = 0; 
  pid[2].ActualSpeed = 0.0; 
  pid[2].err = 0.0; 
  pid[2].err_last = 0.0; 
  pid[2].err_next =0.0; 
  pid[2].Kp = 5; 
  pid[2].Ki = 8; 
  pid[2].Kd = 0; 
  pid[2].epsilon = 0.5;
  
  pid[3].SetSpeed = 0; 
  pid[3].ActualSpeed = 0.0; 
  pid[3].err = 0.0; 
  pid[3].err_last = 0.0; 
  pid[3].err_next =0.0; 
  pid[3].Kp = 8.5; 
  pid[3].Ki = 8; 
  pid[3].Kd = 0; 
  pid[3].epsilon = 0.5;
  
  pid_omega.SetSpeed = 0; 
  pid_omega.ActualSpeed = 0.0; 
  pid_omega.err = 0.0; 
  pid_omega.err_last = 0.0; 
  pid_omega.err_next =0.0; 
  pid_omega.Kp = 0.0; 
  pid_omega.Ki = 13.0; 
  pid_omega.Kd = 0.0; 
  pid_omega.epsilon = 0.5;
  
  pid_yaw.SetSpeed = 0; 
  pid_yaw.ActualSpeed = 0.0; 
  pid_yaw.err = 0.0; 
  pid_yaw.err_last = 0.0; 
  pid_yaw.err_next =0.0; 
  pid_yaw.Kp = 0.0; 
  pid_yaw.Ki = 0.1; 
  pid_yaw.Kd = 0.0; 
  pid_yaw.epsilon = 0.5;
  
}


/**
 * @brief       ���ַ��룬����  beta
 * @param       error: ��ǰƫ��
 *               epsilon: ������ֵ
 *
 * @return      beta : ����ϵ��
 * @example
 * @note        �ڲ�ʹ��
 *
 */
static float beta_generation(float error, float epsilon)
{
  float beta = 0.4;
  
  if(fabs(error) <= epsilon)
    beta = 1;
    
  return beta;
  
}




/**
 * @brief       PIDʵ�ֺ���
 * @param       speed: Ŀ��ֵ
 * @return      pid.ActualSpeed����������ٶ�
 * @example
 * @note        ����ֵ���� �ٶ�PWM ���� ת��֮����Ϊ�����������������ֵ
 *
 */
float PID_realize(float speed, pid_t *pid)
{
  static float beta;
  pid->SetSpeed = speed; 
  pid->err = pid->SetSpeed - pid->ActualSpeed;
  
  if (fabs(pid->err) <= 0.1)
    pid->err = 0;
  
  beta = beta_generation(pid->err, pid->epsilon);
  
  float incrementSpeed = pid->Kp * (pid->err - pid->err_next) + beta * pid->Ki * (pid->err + pid->err_next) / 2 + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);
  
  pid->ActualSpeed += incrementSpeed; 
  pid->err_last = pid->err_next; 
  pid->err_next = pid->err; 
  
  return pid->ActualSpeed; 
}


/**
 * @brief       �ĸ��ֵ�pid����
 * @note        �����ƽ������ת
 * 
 */
void PID_four_wheels_speed_control(void)
{
  get_speed_and_set_translation_speed();
  get_yaw_and_set_omega();            //һ���˶�������yawƯ�����أ����Ŀǰ�Ȳ��Զ�ȡ���ٶȣ�����Ҫ�Ƕ�
  get_omega_and_set_rotation_speed();
}







/**
 * @brief       pid�������ڣ����ڲ���
 * @param       buffer[]: ������λ�������ʽ���ַ������ڸ���pid�������ٶ�Ŀ��ֵ
 * @return
 * @example
 * @note        ������pid����������
 *              �����ʽΪ�� 16          15~13                12~8           7~4               3~0
 *                          pid_num      SetSpeed * 100        Kd * 100       Ki * 100          Kp * 100
 *
 */
void pid_param_adjust(char buffer[])
{
  uint8_t pid_num = buffer[16] - '0';
  
  pid[pid_num].Kp = 0;
  pid[pid_num].Ki = 0;
  pid[pid_num].Kd = 0;
  pid[pid_num].SetSpeed = 0;
  
  pid_omega.Kp = 0;
  pid_omega.Ki = 0;
  pid_omega.SetSpeed = 0;
  
  pid_yaw.Ki = 0;
  
  for(int i = 0; i < 4; i++) {
    pid[pid_num].Kp += (buffer[i] - '0') * pow(10, 3 - i);
  }
  pid[pid_num].Kp /= 100;
  
  for(int i = 4; i < 8; i++) {
    pid[pid_num].Ki += (buffer[i] - '0') * pow(10, 7 - i);
  }
  pid[pid_num].Ki /= 100;
  
  for(int i = 8; i < 12; i++) {
    pid[pid_num].Kd += (buffer[i] - '0') * pow(10, 11 - i);
  }
  pid[pid_num].Kd /= 100;
  
  for(int i = 13; i < 16; i++) {
    pid[pid_num].SetSpeed += (buffer[i] - '0') * pow(10, 15 - i);
  }
  pid[pid_num].SetSpeed /= 100;
  
  for(int i = 17; i < 21; i++) {
    pid_omega.Kp += (buffer[i] - '0') * pow(10, 20 - i);
  }
  pid_omega.Kp /= 100;
  
  for(int i = 21; i < 25; i++) {
    pid_omega.Ki += (buffer[i] - '0') * pow(10, 24 - i);
  }
  pid_omega.Ki /= 100;
  
  for(int i = 25; i < 29; i++) {
    pid_omega.SetSpeed += (buffer[i] - '0') * pow(10, 28 - i);
  }
  pid_omega.SetSpeed /= 100;
  
  for(int i = 29; i < 33; i++) {
    pid_yaw.Kp += (buffer[i] - '0') * pow(10, 32 - i);
  }
  pid_yaw.Kp /= -100;
  
  if (buffer[33] - '0' == 0)
    pid[pid_num].SetSpeed = -pid[pid_num].SetSpeed;
  
  NVIC_DisableIRQ(UART4_RX_TX_IRQn);
  
  printf("PID%d: kp = %.2f, ki = %.2f, kd = %.2f, SetSpeed = %.2f \nPID_omega: kp = %.2f, ki = %.2f, SetSpeed = %.2f\nPID_yaw: kp = %.2f\n\n", pid_num, pid[pid_num].Kp, pid[pid_num].Ki, pid[pid_num].Kd, pid[pid_num].SetSpeed,
                                                                       pid_omega.Kp, pid_omega.Ki, pid_omega.SetSpeed,
                                                                       pid_yaw.Kp);
  NVIC_EnableIRQ(UART4_RX_TX_IRQn);
}
