#include "motor.h"


extern TIM_HandleTypeDef htim1;

void set_motor_pwm(uint16_t motor_pwm[])
{
  if (motor_pwm[0] >= MOTOR_MAX_PWM)
	  htim1.Instance->CCR1 = MOTOR_MAX_PWM;
  else if (motor_pwm[0] <= MOTOR_MIN_PWM)
	  htim1.Instance->CCR1 = MOTOR_MIN_PWM;
  else
	  htim1.Instance->CCR1 = motor_pwm[0] ;
  
  if (motor_pwm[1] >= MOTOR_MAX_PWM)
	  htim1.Instance->CCR2 = MOTOR_MAX_PWM;
  else if (motor_pwm[1] <= MOTOR_MIN_PWM)
	  htim1.Instance->CCR2 = MOTOR_MIN_PWM;
  else
	  htim1.Instance->CCR2 =  motor_pwm[1] ;
  
  if (motor_pwm[2] >= MOTOR_MAX_PWM)
	  htim1.Instance->CCR3 = MOTOR_MAX_PWM;
  else if (motor_pwm[2] <= MOTOR_MIN_PWM)
	  htim1.Instance->CCR3 = MOTOR_MIN_PWM;
  else
	  htim1.Instance->CCR3 =  motor_pwm[2] ;
  
  if (motor_pwm[3] >= MOTOR_MAX_PWM)
	  htim1.Instance->CCR4 = MOTOR_MAX_PWM;
  else if (motor_pwm[3]  <= MOTOR_MIN_PWM)
	  htim1.Instance->CCR4 = MOTOR_MIN_PWM;
  else
	  htim1.Instance->CCR4 = motor_pwm[3] ;
}


void set_motor_pwm_zero(uint16_t motor_pwm[])
{
  motor_pwm[0] = 0;
  motor_pwm[1] = 0;
  motor_pwm[2] = 0;
  motor_pwm[3] = 0;
}

