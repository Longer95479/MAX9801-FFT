#ifndef _AHRS_H
#define _AHRS_H

/**
 * @breif       
 */
typedef struct {
 float q0;
 float q1;
 float q2;
 float q3;
} quaterInfo_t;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} accel_t;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} gyro_t;


/**
 * @brief       ����ŷ����
 * @param
 * @return
 * @example
 * @note        ����ӿں���������ȫ�ֱ���ŷ���ǽṹ�����
 *
 */
void EulerianAngles_update(void);

/**
 * @brief       �ǶȻ����ƣ�YAW�����ƣ�
 * @note        �˺������޸� pid_omega.SetSpeed
 */
void get_yaw_and_set_omega(void);

/**
 * @brief       ���ٶȻ�����
 * @note        �˺������޸� rpsx_set
 */
void get_omega_and_set_rotation_speed(void);


#endif
