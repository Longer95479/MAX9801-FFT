#ifndef _PID_H
#define _PID_H


/**
 * @brief       PID相关变量初始化，设定相关参数，非参数量设为0
 * @param
 * @return
 * @example
 * @note        直接在程序内部修改
 *
 */
void PID_init(void);


/**
 * @brief       PID实现函数
 * @param       speed: 目标值
 * @return      pid.ActualSpeed：调整后的速度
 * @example
 * @note        返回值利用 速度PWM 曲线 转化之后作为电机驱动函数的输入值
 *
 */
float PID_realize(float speed, pid_t *pid);


/**
 * @brief       四个轮的pid控制
 * @note        耦合了平动和旋转
 * 
 */
void PID_four_wheels_speed_control(void);


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
void pid_param_adjust(char buffer[]);

#endif
