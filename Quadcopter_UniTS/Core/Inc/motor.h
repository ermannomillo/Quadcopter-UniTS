#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "control.h"
#include "stm32h7xx_hal.h"


#define MOTOR_MAX_PWM     1800 //180 // 221
#define MOTOR_MIN_PWM     1000 //100

void set_motor_pwm(uint16_t motor_pwm[], uint16_t motor_pwm_lim[]);
void set_motor_pwm_zero(uint16_t motor_pwm[]);
void init_motors();
void mixing_formula(uint16_t motor_pwm[], uint16_t motor_throttle, PID_Out out_pid);

#endif 
