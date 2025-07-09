
#include <motor.h>
#include <control.h>


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
	motor_pwm[0] = MOTOR_MIN_PWM;
	motor_pwm[1] = MOTOR_MIN_PWM;
	motor_pwm[2] = MOTOR_MIN_PWM;
	motor_pwm[3] = MOTOR_MIN_PWM;
}

void init_motors()
{
	/*
	 * Initialise motors and set them to rest
	 */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);  // 1ms if period is 2000 ticks
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);

	HAL_Delay(3000);  // Wait for ESC to initialize

	// Optional: set mid-throttle (1.5 ms pulse width)
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 150);  // 1.5ms
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 150);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 150);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 150);

	uint16_t motor_pwm[4];

	set_motor_pwm_zero(motor_pwm);
	set_motor_pwm(motor_pwm);  /*checks for limits and writes in registers the values */

}

void mixing_formula(uint16_t motor_pwm[], uint16_t motor_throttle,  PID_Out out_pid)
{
	/*
	 * Drone mixing formula
	 */

	motor_pwm[2] = motor_throttle - out_pid.pitch - out_pid.roll + out_pid.yaw; //should be the back left motor
	motor_pwm[1] = 0.9965*(motor_throttle + out_pid.pitch - out_pid.roll - out_pid.yaw); //should be the front left motor
	motor_pwm[0] = 0.998*(motor_throttle + out_pid.pitch + out_pid.roll + out_pid.yaw); // should be the front right motor
	motor_pwm[3] = (motor_throttle - out_pid.pitch + out_pid.roll - out_pid.yaw); //should be the back right motor motor_throttle
}

