#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32h7xx_hal.h"


#define MOTOR_MAX_PWM_VALUE     1700.0f  
#define MOTOR_MIN_PWM_VALUE     850.0f     

typedef struct
{
  float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
}MotorControlTypeDef;

void set_motor_pwm(MotorControlTypeDef *motor_pwm);
void set_motor_pwm_zero(MotorControlTypeDef *motor_pwm);

#endif 
