#ifndef _STEPPER_H
#define _STEPPER_H
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "ds18b20.h"
#define stepsperrev 4096
void stepper_set_rpm (int );  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
void stepper_half_drive (int );
void stepper_step_angle (float , int , int );
#endif