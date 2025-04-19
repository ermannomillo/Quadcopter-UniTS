#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32h7xx_hal.h"

<<<<<<< HEAD
#define MOTOR_MAX_PWM_VALUE     1700.0f  
#define MOTOR_MIN_PWM_VALUE     850.0f     

void set_motor_pwm(uint32_t *motor_pwm[]);
void set_motor_pwm_zero(uint32_t *motor_pwm[]);
=======

#define MOTOR_MAX_PWM_VALUE     1700.0f  
#define MOTOR_MIN_PWM_VALUE     850.0f     

typedef struct
{
  float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
}MotorControlTypeDef;

void set_motor_pwm(MotorControlTypeDef *motor_pwm);
void set_motor_pwm_zero(MotorControlTypeDef *motor_pwm);
>>>>>>> branch 'RC_Motor_Power' of git@github.com:ermannomillo/Quadcopter-UniTS.git

#endif 
