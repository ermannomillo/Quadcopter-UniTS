#ifndef _FLIGHT_CONTROL_H_
#define _FLIGHT_CONTROL_H_

#include "stm32h7xx_hal.h"
#include "orientation.h"



#define I_TERM_LIMIT 50000

typedef struct
{
	float p_error[3];
	float i_error[3];
	float d_error[3];

}PID_Error;

typedef struct
{
	float pitch;
	float roll;
	float yaw;

}PID_Out;


void pid_update(PID_Out *out_pid, Euler imu_est_euler, Euler rc_ref_euler, uint16_t motor_throttle);
void init_pid();

#endif
