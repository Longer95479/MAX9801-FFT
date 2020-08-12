#ifndef LONGER_MOTOR
#define LONGER_MOTOR

void motor_init(void);
void car_go(void);
void car_stop(void);
void car_slow(void);
/**
 * @brief       记录车上次的状态
 */
void last_car_status_record(void);

/**
* @brief       依据车状态对车进行控制
*/
void car_control(float theta, float amplitude);


#endif