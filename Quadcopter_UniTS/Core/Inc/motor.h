#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32h7xx_hal.h"

#define MOTOR_MAX_PWM     221
#define MOTOR_MIN_PWM     105

void set_motor_pwm(uint16_t motor_pwm[]);
void set_motor_pwm_zero(uint16_t motor_pwm[]);

#endif 
