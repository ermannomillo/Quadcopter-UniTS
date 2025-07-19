#include "imu.h"
#include "filter.h"
#include "orientation.h"
#include "main.h"  // for HAL_GetTick()

static int euler_offset[2] = {-179303, 13};
static int gyro_offset[3] = {-80, 25, 206};
static int gyro_data[3];
static int gyro_delta[3];
static int prev_tick_ms = 0;
static float alpha = 0.01;

void orientation_init(void) {
	imu_calibrate(euler_offset, gyro_offset);
	prev_tick_ms = HAL_GetTick();
}

void orientation_update(Euler *imu_est_euler){
	short int imu_raw[7];
	int acc_euler[3];

	imu_read_all(imu_raw);
	filter_compute_gravity_angles(imu_raw[0], imu_raw[1], imu_raw[2], acc_euler);

	if (acc_euler[0] > 0) acc_euler[0] -= 360000;

	acc_euler[0] -= euler_offset[0];
	acc_euler[1] -= euler_offset[1];

	for (int i = 0; i < 3; i++) {
		gyro_delta[i] = imu_raw[4 + i] - gyro_offset[i] - gyro_data[i];
		gyro_data[i]  = imu_raw[4 + i] - gyro_offset[i];
	}

	int euler_est[3];

	euler_est[0] = imu_est_euler->roll;
	euler_est[1] = imu_est_euler->pitch;
	euler_est[2] = imu_est_euler->yaw;

	filter_integrate_gyro(gyro_data, gyro_delta, euler_est, &prev_tick_ms);
	filter_fuse_angles(acc_euler, euler_est, alpha);

	imu_est_euler->roll  = euler_est[0];
	imu_est_euler->pitch = euler_est[1];
	imu_est_euler->yaw   = euler_est[2];
}


