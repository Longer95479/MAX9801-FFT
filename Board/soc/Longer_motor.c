#include "include.h"
#include "Longer_motor.h"

void motor_init(void)
{
    FTM_PwmInit(FTM0, FTM_CH0, 12000, 0);
    FTM_PwmInit(FTM0, FTM_CH1, 12000, 0);
    
    FTM_PwmInit(FTM0, FTM_CH2, 12000, 0);
    FTM_PwmInit(FTM0, FTM_CH3, 12000, 0);
    
    GPIO_PinInit(PTD0, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH5, 12000, 0);
    
    GPIO_PinInit(PTD6, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH7, 12000, 0);

}

void car_move(float sx, float sy)
{
  static int gain = 1500;
  static float vy = 0, w = 0;
  static float wheel[4];
  
  int max_x = (sx / 352 - 19e-6) / DELTA_TIME;
  int max_y = (sy / 352 - 19e-6) / DELTA_TIME;
  
  /*
  if (max_y < 0) {
    GPIO_PinInit(PTA17, GPO, 0);
  }
  else {
    GPIO_PinInit(PTA17, GPO, 1);
  }
  
  if (max_x > 2) {
    GPIO_PinInit(PTC0, GPO, 0);
    GPIO_PinInit(PTD15, GPO, 1);
  }
  else if (max_x < -2) {
    GPIO_PinInit(PTC0, GPO, 1);
    GPIO_PinInit(PTD15, GPO, 0);
  }
  else {
    GPIO_PinInit(PTC0, GPO, 1);
    GPIO_PinInit(PTD15, GPO, 1);
  }*/
  
  
  if (max_x < -3 && max_y <= 0.5) {
    wheel[0] = -1.7;//-1.5;
    wheel[1] = -1.7;//-1.5;
    wheel[2] = 1.7;//2.5;
    wheel[3] = 1.7;//2.5;
  }
  else if (max_x >= -3 && max_x <= 3 && max_y < -0.5) {
    wheel[0] = 2;
    wheel[1] = 2;
    wheel[2] = 2;
    wheel[3] = 2;
  }
  else if (max_x > 3 && max_y <= 0.5) {
    wheel[0] = 1.7;//2.5;
    wheel[1] = 1.7;//2.5;
    wheel[2] = -1.7;//-1.5;
    wheel[3] = -1.7;//-1.5;
  }
  else if (max_x > 3 && max_y > 0.5) {
    wheel[0] = -1.7;//-2.5;
    wheel[1] = -1.7;//-2.5;
    wheel[2] = 1.7;//1.5;
    wheel[3] = 1.7;//1.5;
  }
  else if (max_x >= -3 && max_x <= 3 && max_y > 0.5) {
    wheel[0] = -2;
    wheel[1] = -2;
    wheel[2] = -2;
    wheel[3] = -2;
  }
  else if (max_x < -3 && max_y > 0.5) {
    wheel[0] = 1.7;//1.5;
    wheel[1] = 1.7;//1.5;
    wheel[2] = -1.7;//-2.5;
    wheel[3] = -1.7;//-2.5;
  }
  else if (max_x >= -3 && max_x <= 3 && max_y >= -0.5 && max_y <= 0.5) {
    wheel[0] = 0;
    wheel[1] = 0;
    wheel[2] = 0;
    wheel[3] = 0;
  }
  
  /*
  if (max_x > 2)
    w = 10;
  else if (max_x < -2)
    w = -10;
  else
    w = 0;
  
  if (max_y > 2)
    vy = -0.5;
  else if (max_y < -2)
    vy = 0.5;
  else
   vy = 0;
  
  if (w == 0)
    GPIO_PinInit(PTC0, GPO, 1);
  else
    GPIO_PinInit(PTC0, GPO, 0);
  
  wheel[0] = vy + 0.18 * w;
  wheel[1] = vy + 0.18 * w;
  wheel[2] = vy - 0.18 * w;
  wheel[3] = vy - 0.18 * w;
  */
  if (wheel[0] > 0) {
    FTM_PwmDuty(FTM0, FTM_CH0, 0);
    FTM_PwmDuty(FTM0, FTM_CH1, (int)(gain * wheel[0]));
  }
  else {
    FTM_PwmDuty(FTM0, FTM_CH1, 0);
    FTM_PwmDuty(FTM0, FTM_CH0, (int)(-gain * wheel[0]));
  }
  
  if (wheel[1] > 0) {
    GPIO_PinInit(PTD0, GPO, 0);
    FTM_PwmDuty(FTM0, FTM_CH5, (int)(gain * wheel[1]));
  }
  else {
    GPIO_PinInit(PTD0, GPO, 1);
    FTM_PwmDuty(FTM0, FTM_CH5, (int)(-gain * wheel[1]));
  }
  
  if (wheel[2] > 0) {
    GPIO_PinInit(PTD6, GPO, 0);
    FTM_PwmDuty(FTM0, FTM_CH7, (int)(gain * wheel[2]));
  }
  else {
    GPIO_PinInit(PTD6, GPO, 1);
    FTM_PwmDuty(FTM0, FTM_CH7, (int)(-gain * wheel[2]));
  }
  
  if (wheel[3] > 0) {
    FTM_PwmDuty(FTM0, FTM_CH2, 0);
    FTM_PwmDuty(FTM0, FTM_CH3, (int)(gain * wheel[3]));
  }
  else {
    FTM_PwmDuty(FTM0, FTM_CH3, 0);
    FTM_PwmDuty(FTM0, FTM_CH2, (int)(-gain * wheel[3]));
  }
  
    
}

