#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32h7xx_hal.h"

#define MOTOR_MAX_PWM_VALUE     1700.0f  
#define MOTOR_MIN_PWM_VALUE     850.0f     

void set_motor_pwm(float motor_pwm[]);
void set_motor_pwm_zero(float motor_pwm[]);

#endif 
