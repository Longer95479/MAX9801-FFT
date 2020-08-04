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
 * @brief       Í£³µ
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

