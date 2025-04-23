#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32h7xx_hal.h"

#define MOTOR_MAX_PWM_VALUE     600
#define MOTOR_MIN_PWM_VALUE     0

void set_motor_pwm(uint32_t motor_pwm[]);
void set_motor_pwm_zero(uint32_t motor_pwm[]);

#endif 
