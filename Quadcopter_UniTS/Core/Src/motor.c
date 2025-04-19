#include "motor.h"


extern TIM_HandleTypeDef htim1;

<<<<<<< HEAD
void set_motor_pwm(uint32_t *motor_pwm[])
{
  if (motor_pwm[0] >= MOTOR_MAX_PWM_VALUE)
	  htim1.Instance->CCR1 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm[0] <= MOTOR_MIN_PWM_VALUE)
	  htim1.Instance->CCR1 = MOTOR_MIN_PWM_VALUE;
  else
	  htim1.Instance->CCR1 = motor_pwm[0] ;
  
  if (motor_pwm[1] >= MOTOR_MAX_PWM_VALUE)
	  htim1.Instance->CCR2 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm[1] <= MOTOR_MIN_PWM_VALUE)
	  htim1.Instance->CCR2 = MOTOR_MIN_PWM_VALUE;
  else
	  htim1.Instance->CCR2 =  motor_pwm[1] ;
  
  if (motor_pwm[2] >= MOTOR_MAX_PWM_VALUE)
	  htim1.Instance->CCR3 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm[2] <= MOTOR_MIN_PWM_VALUE)
	  htim1.Instance->CCR3 = MOTOR_MIN_PWM_VALUE;
  else
	  htim1.Instance->CCR3 =  motor_pwm[2] ;
  
  if (motor_pwm[3] >= MOTOR_MAX_PWM_VALUE)
	  htim1.Instance->CCR4 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm[3]  <= MOTOR_MIN_PWM_VALUE)
	  htim1.Instance->CCR4 = MOTOR_MIN_PWM_VALUE;
  else
	  htim1.Instance->CCR4 = motor_pwm[3] ;
}


void set_motor_pwm_zero(uint32_t *motor_pwm[])
{
  motor_pwm[0] = 0;
  motor_pwm[1] = 0;
  motor_pwm[2] = 0;
  motor_pwm[3] = 0;
=======
void set_motor_pwm(MotorControlTypeDef *motor_pwm)
{
  if (motor_pwm->motor1_pwm >= MOTOR_MAX_PWM_VALUE)
    htim4.Instance->CCR1 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm->motor1_pwm <= MOTOR_MIN_PWM_VALUE)
    htim4.Instance->CCR1 = MOTOR_MIN_PWM_VALUE;
  else
    htim4.Instance->CCR1 = (uint32_t) motor_pwm->motor1_pwm; 
  
  if (motor_pwm->motor2_pwm >= MOTOR_MAX_PWM_VALUE)
    htim4.Instance->CCR2 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm->motor2_pwm <= MOTOR_MIN_PWM_VALUE)
    htim4.Instance->CCR2 = MOTOR_MIN_PWM_VALUE;
  else
    htim4.Instance->CCR2 = (uint32_t) motor_pwm->motor2_pwm;
  
  if (motor_pwm->motor3_pwm >= MOTOR_MAX_PWM_VALUE)
    htim4.Instance->CCR3 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm->motor3_pwm <= MOTOR_MIN_PWM_VALUE)
    htim4.Instance->CCR3 = MOTOR_MIN_PWM_VALUE;
  else
    htim4.Instance->CCR3 = (uint32_t) motor_pwm->motor3_pwm;
  
  if (motor_pwm->motor4_pwm >= MOTOR_MAX_PWM_VALUE)
    htim4.Instance->CCR4 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm->motor4_pwm <= MOTOR_MIN_PWM_VALUE)
    htim4.Instance->CCR4 = MOTOR_MIN_PWM_VALUE;
  else
    htim4.Instance->CCR4 = (uint32_t) motor_pwm->motor4_pwm;
}


void set_motor_pwm_zero(MotorControlTypeDef *motor_pwm)
{
  motor_pwm->motor1_pwm = 0;
  motor_pwm->motor2_pwm = 0;
  motor_pwm->motor3_pwm = 0;
  motor_pwm->motor4_pwm = 0;
>>>>>>> branch 'RC_Motor_Power' of git@github.com:ermannomillo/Quadcopter-UniTS.git
}

