#include "include.h"
#include "Longer_motor.h"

void motor_init(void)
{
    GPIO_PinInit(PTC1, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH1, 12000, 0);
    
    GPIO_PinInit(PTC3, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH3, 12000, 0);
    
    GPIO_PinInit(PTD0, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH5, 12000, 0);
    
    GPIO_PinInit(PTD6, GPO, 0);
    FTM_PwmInit(FTM0, FTM_CH7, 12000, 0);

}

void car_move(float sx, float sy)
{
  static int gain = 1000;
  static float vy = 0, w = 0;
  static float wheel[4];
  
  int max_x = (sx / 352 - 19e-6) / DELTA_TIME + _N/2 - 1;
  int max_y = (sy / 352 - 19e-6) / DELTA_TIME + _N/2 - 1;
  
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
  
  wheel[0] = vy + 0.18 * w;
  wheel[1] = vy + 0.18 * w;
  wheel[2] = vy - 0.18 * w;
  wheel[3] = vy - 0.18 * w;
  
  if (wheel[0] > 0) {
    GPIO_PinInit(PTC1, GPO, 0);
    FTM_PwmDuty(FTM0, FTM_CH1, (int)(gain * wheel[0]));
  }
  else {
    GPIO_PinInit(PTC1, GPO, 1);
    FTM_PwmDuty(FTM0, FTM_CH1, (int)(-gain * wheel[0]));
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
    GPIO_PinInit(PTC3, GPO, 0);
    FTM_PwmDuty(FTM0, FTM_CH3, (int)(gain * wheel[3]));
  }
  else {
    GPIO_PinInit(PTC3, GPO, 1);
    FTM_PwmDuty(FTM0, FTM_CH3, (int)(-gain * wheel[3]));
  }
   
    
}

