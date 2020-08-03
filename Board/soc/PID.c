#define PID_GLOBALS

#include "include.h"
#include "PID.h"

/**
 * @brief       PID相关变量初始化，设定相关参数，非参数量设为0
 * @param
 * @return
 * @example
 * @note        直接在程序内部修改
 *
 */
void PID_init(void)
{
  
  pid[0].SetSpeed = 0; 
  pid[0].ActualSpeed = 0.0; 
  pid[0].err = 0.0; 
  pid[0].err_last = 0.0; 
  pid[0].err_next =0.0; 
  pid[0].Kp = 2; 
  pid[0].Ki = 4; 
  pid[0].Kd = 0;
  pid[0].epsilon = 0.5;
  
  pid[1].SetSpeed = 0; 
  pid[1].ActualSpeed = 0.0; 
  pid[1].err = 0.0; 
  pid[1].err_last = 0.0; 
  pid[1].err_next =0.0; 
  pid[1].Kp = 1; 
  pid[1].Ki = 4.2; 
  pid[1].Kd = 0; 
  pid[1].epsilon = 0.5;
  
  pid[2].SetSpeed = 0; 
  pid[2].ActualSpeed = 0.0; 
  pid[2].err = 0.0; 
  pid[2].err_last = 0.0; 
  pid[2].err_next =0.0; 
  pid[2].Kp = 2.5; 
  pid[2].Ki = 4; 
  pid[2].Kd = 0; 
  pid[2].epsilon = 0.5;
  
  pid[3].SetSpeed = 0; 
  pid[3].ActualSpeed = 0.0; 
  pid[3].err = 0.0; 
  pid[3].err_last = 0.0; 
  pid[3].err_next =0.0; 
  pid[3].Kp = 2; 
  pid[3].Ki = 4.3; 
  pid[3].Kd = 0; 
  pid[3].epsilon = 0.5;
  
}


/**
 * @brief       积分分离，计算  beta
 * @param       error: 当前偏差
 *               epsilon: 分离阈值
 *
 * @return      beta : 分离系数
 * @example
 * @note        内部使用
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
 * @brief       PID实现函数
 * @param       speed: 目标值
 * @return      pid.ActualSpeed：调整后的速度
 * @example
 * @note        返回值利用 速度PWM 曲线 转化之后作为电机驱动函数的输入值
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
 * @brief       pid参数调节，用于测试
 * @param       buffer[]: 来自上位机特殊格式的字符，用于更改pid参数和速度目标值
 * @return
 * @example
 * @note        仅用于pid参数整定。
 *              输入格式为： 16          15~13                12~8           7~4               3~0
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
  
  printf("PID%d: kp = %.2f, ki = %.2f, kd = %.2f, SetSpeed = %.2f \n", pid_num, pid[pid_num].Kp, pid[pid_num].Ki, pid[pid_num].Kd, pid[pid_num].SetSpeed);
}
