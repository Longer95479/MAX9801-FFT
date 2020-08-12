#define MOTOR_GLOBALS

#include "include.h"
#include "Longer_motor.h"

void motor_init(void)
{   
    GPIO_PinInit(PTC1, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH1, 12000, 0);
    //FTM_PwmDuty(FTM0, FTM_CH1, 2400);
      
    GPIO_PinInit(PTD0, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH5, 12000, 0);
    //FTM_PwmDuty(FTM0, FTM_CH5, 2400);
      
    GPIO_PinInit(PTD6, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH7, 12000, 0);
    //FTM_PwmDuty(FTM0, FTM_CH7, 2400);
      
    GPIO_PinInit(PTC3, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH3, 12000, 0);
    //FTM_PwmDuty(FTM0, FTM_CH2, 2400);

}



/**
* @brief       依据车状态对车进行控制
*/
#define CAR_NORMAL_RUN_AMP_THRESHOLD
#define DELTA_THETA_THRESHOLD   20000.0f
#define CAR_BACK_VELOCITY       4
#define CAR_OBLIQUE_VELOCITY    4

void car_control(float theta, float amplitude)
{
  static float v, vx, vy, wheel_v[4], d;
  static float theta_mean[4] = {0}, delta_theta;
  static int16_t theta_section_cnt[4] = {0};
  static int8_t car_run_first_in = 1;           //需要在其他状态置位
  
  switch (car_status) {
    
  case CAR_NORMAL_RUN:
    car_run_first_in = 1;
#if 0
    if (amplitude < 18000.0f)
      v = 0.0007f * amplitude -4;               //！！！需要再调整
      //v = 0.001f * amplitude - 11;
      //v = 6.2e-8f * amplitude * amplitude;
    else
      v = 0.0007f * amplitude + 17;             //！！！需要再调整
      //v = -0.001f * amplitude + 25;
      //v = -1.2e-7f * amplitude * amplitude + 2.3e-3f * amplitude + 17.86;
    
    if (amplitude > DELTA_THETA_THRESHOLD) {
      //delta_theta = asin(0.20f * InvSqrt(44971.0f / amplitude));
      d = 6.4e-9f * amplitude * amplitude + 3.5e-4f * amplitude + 5.2f;          //根据幅值估算车到灯的距离，新拟合的表达式
      delta_theta = asin(0.20f / d); 
      
      vx = v * cos(theta + delta_theta);         //rps * cos(theta)
      vy = v * sin(theta + delta_theta);         //rps * sin(theta)
    }
    
    else {
      vx = v * cos(theta);         //rps * cos(theta)
      vy = v * sin(theta);         //rps * sin(theta)
    }
#else
    vx = 4.5 * cos(theta);         //rps * cos(theta)
    vy = 4.5 * sin(theta);         //rps * sin(theta)
    
#endif
    
    wheel_v[0] = vx + vy;
    wheel_v[1] = -vx + vy;
    wheel_v[2] = vx + vy;
    wheel_v[3] = -vx + vy;
    
    for (int i = 0; i < 4; i++)
      pid[i].SetSpeed = wheel_v[i];
    
    
  case CAR_STOP:
    car_run_first_in = 1;
    
    car_stop();                 //此处可优化，不必每次都运行，只需第一次运行
    
    // 对 theta 划分区间计数，取计数最多的区间的均值
    if (theta > 0 ) {
      if (theta < PI/2) {
        theta_mean[0] += theta;
        theta_section_cnt[0]++;
      }
    
      else {
        theta_mean[1] += theta;
        theta_section_cnt[1]++;
      }
    }
    
    else {
      if (theta > -PI/2) {
        theta_mean[2] += theta;
        theta_section_cnt[2]++;
      }
      
      else {
        theta_mean[3] += theta;
        theta_section_cnt[3]++;
      }
    }
    
    //**theta_mean += theta;
    //**theta_add_cnt++;
    
    
  case CAR_RUN:
    ;
    if (car_run_first_in == 1) {
      car_run_first_in = 0;
      
      //theta_mean /= theta_add_cnt;
      static uint8_t max = 0;     //用于记录计数多的区间
      for (int i = 0; i < 4; i++) {
        if (theta_section_cnt[i] > theta_section_cnt[max])
          max = i;        
      }
      
      theta_mean[max] /= theta_section_cnt[max];
      
      //v = 1.8e-8f * amplitude * amplitude;
      
      vx = 4 * cos(theta_mean[max]);         //rps * cos(theta)
      vy = 4 * sin(theta_mean[max]);         //rps * sin(theta)
      
      wheel_v[0] = vx + vy;
      wheel_v[1] = -vx + vy;
      wheel_v[2] = vx + vy;
      wheel_v[3] = -vx + vy;
      
      for (int i = 0; i < 4; i++)
        pid[i].SetSpeed = wheel_v[i];
      
      //theta_mean = 0;
      //theta_add_cnt= 0;
      for (int i = 0; i < 4; i++) {
        theta_mean[i] = 0;
        theta_section_cnt[i] = 0;
      }
    }
      
  case CAR_BACK:
    
    // v 待定，根据实际测试决定
    vx = CAR_BACK_VELOCITY * cos(theta + PI);         //rps * cos(theta)        
    vy = CAR_BACK_VELOCITY * sin(theta + PI);         //rps * sin(theta)
    
    wheel_v[0] = vx + vy;
    wheel_v[1] = -vx + vy;
    wheel_v[2] = vx + vy;
    wheel_v[3] = -vx + vy;
    
    for (int i = 0; i < 4; i++)
      pid[i].SetSpeed = wheel_v[i];
  
  case CAR_OBLIQUE:
    
    // v 待定，根据实际测试决定
    vx = CAR_OBLIQUE_VELOCITY * cos(theta + PI/2);         //rps * cos(theta)        
    vy = CAR_OBLIQUE_VELOCITY * sin(theta + PI/2);         //rps * sin(theta)
    
    wheel_v[0] = vx + vy;
    wheel_v[1] = -vx + vy;
    wheel_v[2] = vx + vy;
    wheel_v[3] = -vx + vy;
    
    for (int i = 0; i < 4; i++)
      pid[i].SetSpeed = wheel_v[i];
  
  }
}

/**
* @brief       记录车上次的状态
*/
void last_car_status_record(void)
{
  switch (car_status) {
  case CAR_NORMAL_RUN:
    last_car_status = CAR_NORMAL_RUN;
    
  case CAR_STOP:
    last_car_status = CAR_STOP;
    
  case CAR_RUN:
    last_car_status = CAR_RUN;
    
  case CAR_BACK:
    last_car_status = CAR_BACK;
    
  case CAR_OBLIQUE:
    last_car_status = CAR_OBLIQUE;
    
  }
}

    
/**
 * @brief       车运动
 * @note        每轮速度由全局变量 rpsx_set 决定
 */
void car_go(void)
{
  // 轮1
  if (rps1_set > 0) {
    GPIO_PinInit(PTC3, GPO, 0);
    FTM_PwmDuty(FTM0, FTM_CH3, (uint16_t)((0.031188f * rps1_set /*+ 0.032839f*/) * 12000));
  }
  else {
    GPIO_PinInit(PTC3, GPO, 1);
    FTM_PwmDuty(FTM0, FTM_CH3, (uint16_t)((0.031188f * (-rps1_set) /*+ 0.032839f*/) * 12000));
  }
  
  // 轮2
  if (rps2_set > 0) {
    GPIO_PinInit(PTD0, GPO, 1);
    FTM_PwmDuty(FTM0, FTM_CH5, (uint16_t)((0.031188f * rps2_set /*+ 0.032839f*/) * 12000));
  }
  else {
    GPIO_PinInit(PTD0, GPO, 0);
    FTM_PwmDuty(FTM0, FTM_CH5, (uint16_t)((0.031188f * (-rps2_set) /*+ 0.032839f*/) * 12000));
  }
  
  // 轮3
  if (rps3_set > 0) {
    GPIO_PinInit(PTD6, GPO, 0);
    FTM_PwmDuty(FTM0, FTM_CH7, (int)((0.031188f * rps3_set /*+ 0.032839f*/) * 12000));
  }
  else {
    GPIO_PinInit(PTD6, GPO, 1);
    FTM_PwmDuty(FTM0, FTM_CH7, (int)((0.031188f * (-rps3_set) /*+ 0.032839f*/) * 12000));
  }
  
  // 轮4
  if (rps4_set > 0) {
    GPIO_PinInit(PTC1, GPO, 0);
    FTM_PwmDuty(FTM0, FTM_CH1, (int)((0.031188f * rps4_set /*+ 0.032839f*/) * 12000));
  }
  else {
    GPIO_PinInit(PTC1, GPO, 1);
    FTM_PwmDuty(FTM0, FTM_CH1, (int)((0.031188f * (-rps4_set) /*+ 0.032839f*/) * 12000));
  }
}


/**
 * @brief       停车
 */
void car_stop(void)
{
  for (int i = 0; i < 4; i++) {
    pid[i].SetSpeed = 0;
  }
  
  FTM_PwmDuty(FTM0, FTM_CH1, 0);
  
  FTM_PwmDuty(FTM0, FTM_CH5, 0);
  
  FTM_PwmDuty(FTM0, FTM_CH7, 0);
  
  FTM_PwmDuty(FTM0, FTM_CH2, 0);

}


/**
 * @brief       减速
 */
void car_slow(void)
{
  for (int i = 0; i < 4; i++) {
    pid[i].SetSpeed /= 2;
  }
}
