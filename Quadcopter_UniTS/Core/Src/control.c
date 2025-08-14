#include <control.h>
#include <math.h>
#include <main.h>


// PID COEFFICIENTS
float Kp[3] = {0.05205, 0.05205, 0}; //{0.000205, 0.000205, 0}; //
float Ki[3] = {0, 0, 0}; // 2*0.000205*0.6*2
float Kd[3] = { 0.00205*0.6/8/2,  0.00205*0.6/8/2, 0}; // 0.000205*0.6/8/2

extern PID_Error control_error; //current
extern PID_Error former_error;  //previous
extern float dt_pid;
int prev_tick_ms_pid;


void pid_update(PID_Out *out_pid, Euler imu_est_euler, Euler rc_ref_euler, uint16_t motor_throttle) {

	/*
	 * Compute PID control, given the current error
	 */

	int now_pid = HAL_GetTick();
	dt_pid = (now_pid - prev_tick_ms_pid) / 1000.0f;
	prev_tick_ms_pid = now_pid;

	// Update proportional errors
	control_error.p_error[0] = rc_ref_euler.roll - imu_est_euler.roll ;
	control_error.p_error[1] = rc_ref_euler.pitch - imu_est_euler.pitch ;
	control_error.p_error[2] = rc_ref_euler.yaw - imu_est_euler.yaw;

	// Update derivative and integrative errors
	for (int i = 0; i < 3; i++) {
		control_error.d_error[i] = (control_error.p_error[i] - former_error.p_error[i])/dt_pid;
		control_error.i_error[i] += control_error.p_error[i] * dt_pid;
	}

	// Anti-windup
	for (int i = 0; i < 3; i++) {
		if (control_error.i_error[i] > I_TERM_LIMIT)
			control_error.i_error[i] = I_TERM_LIMIT;
		else if (control_error.i_error[i] < -I_TERM_LIMIT)
			control_error.i_error[i] = -I_TERM_LIMIT;
	}

	// Update former errors
	for (int i = 0; i < 3; i++) {
		former_error.p_error[i] = control_error.p_error[i];
		former_error.i_error[i] = control_error.i_error[i];
	}

	// PID control output
	out_pid->roll = Kp[0] * control_error.p_error[0] +
			Ki[0] * control_error.i_error[0] +
			Kd[0] * control_error.d_error[0];

	out_pid->pitch = Kp[1] * control_error.p_error[1] +
			Ki[1] * control_error.i_error[1] +
			Kd[1] * control_error.d_error[1];

	out_pid->yaw = Kp[2] * control_error.p_error[2] +
			Ki[2] * control_error.i_error[2] +
			Kd[2] * control_error.d_error[2];
}


void init_pid() {

	/*
	 * Initialise PID errors
	 */

	int prev_tick_ms_pid = HAL_GetTick();

	for (int i = 0; i < 3; i++) {
		control_error.p_error[i] = 0;
		control_error.i_error[i] = 0;
		control_error.d_error[i] = 0;
		former_error.p_error[i] = 0;
		former_error.i_error[i] = 0;
		former_error.d_error[i] = 0;
	}
}


