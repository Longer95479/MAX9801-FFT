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
 * @brief       更新欧拉角
 * @param
 * @return
 * @example
 * @note        对外接口函数，更新全局变量欧拉角结构体变量
 *
 */
void EulerianAngles_update(void);

/**
 * @brief       角度环控制（YAW环控制）
 * @note        此函数会修改 pid_omega.SetSpeed
 */
void get_yaw_and_set_omega(void);

/**
 * @brief       角速度环控制
 * @note        此函数会修改 rpsx_set
 */
void get_omega_and_set_rotation_speed(void);


#endif
