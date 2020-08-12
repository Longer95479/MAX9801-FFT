#ifndef _PID_H
#define _PID_H


/**
 * @brief       PID��ر�����ʼ�����趨��ز������ǲ�������Ϊ0
 * @param
 * @return
 * @example
 * @note        ֱ���ڳ����ڲ��޸�
 *
 */
void PID_init(void);


/**
 * @brief       PIDʵ�ֺ���
 * @param       speed: Ŀ��ֵ
 * @return      pid.ActualSpeed����������ٶ�
 * @example
 * @note        ����ֵ���� �ٶ�PWM ���� ת��֮����Ϊ�����������������ֵ
 *
 */
float PID_realize(float speed, pid_t *pid);


/**
 * @brief       �ĸ��ֵ�pid����
 * @note        �����ƽ������ת
 * 
 */
void PID_four_wheels_speed_control(void);


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
void pid_param_adjust(char buffer[]);

#endif
