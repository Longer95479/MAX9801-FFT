#ifndef LONGER_MOTOR
#define LONGER_MOTOR

void motor_init(void);
void car_go(void);
void car_stop(void);
void car_slow(void);
/**
 * @brief       ��¼���ϴε�״̬
 */
void last_car_status_record(void);

/**
* @brief       ���ݳ�״̬�Գ����п���
*/
void car_control(float theta, float amplitude);


#endif