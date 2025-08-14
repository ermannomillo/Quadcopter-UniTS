#include "imu.h"
#include "filter.h"
#include "orientation.h"
#include "main.h"  // for HAL_GetTick()

// Calibration offsets for accelerometer-based euler angles (roll, pitch)
static int euler_offset[2] = {-178961, 575};

// Calibration offsets for gyroscope data
static int gyro_offset[3] = {-74, 27, 177};

// Stores current gyroscope data
static int gyro_data[3];

// Stores the difference in gyroscope readings between updates
static int gyro_delta[3];

// Stores previous tick count in milliseconds
static int prev_tick_ms = 0;

// Complementary filter coefficient
static float alpha = 0.01;

// Initialize orientation module
void orientation_init(void) {
	//imu_calibrate(euler_offset, gyro_offset); // Perform IMU calibration (not needed, already done)
	prev_tick_ms = HAL_GetTick(); // Record initial time tick
}

// Update orientation estimate
void orientation_update(Euler *imu_est_euler){
	short int imu_raw[7]; // Array to hold raw IMU data
	int acc_euler[3];     // Euler angles from accelerometer

	imu_read_all(imu_raw); // Read all sensor data from IMU
	filter_compute_gravity_angles(imu_raw[0], imu_raw[1], imu_raw[2], acc_euler); // Compute pitch and roll

	// Adjust pitch angle if needed
	if (acc_euler[0] > 0) acc_euler[0] -= 360000;

	// Subtract calibration offsets from pitch and roll
	acc_euler[0] -= euler_offset[0];
	acc_euler[1] -= euler_offset[1];

	// Compute gyroscope delta values and update gyro_data
	gyro_delta[0] = imu_raw[4 + 0] - gyro_offset[0] - gyro_data[0]; // Change since last reading
	gyro_data[0]  = imu_raw[4 + 0] - gyro_offset[0];                // Current corrected value

	gyro_delta[1] = imu_raw[4 + 1] - gyro_offset[1] - gyro_data[1]; // Change since last reading
	gyro_data[1]  = imu_raw[4 + 1] - gyro_offset[1];                // Current corrected value

	gyro_delta[2] = imu_raw[4 + 2] - gyro_offset[2] - gyro_data[2]; // Change since last reading
	gyro_data[2]  = imu_raw[4 + 2] - gyro_offset[2];                // Current corrected value

	int euler_est[3];

	// Initialize estimated euler angles from previous state
	euler_est[0] = imu_est_euler->roll;
	euler_est[1] = imu_est_euler->pitch;
	euler_est[2] = imu_est_euler->yaw;

	// Integrate gyroscope data
	filter_integrate_gyro(gyro_data, gyro_delta, euler_est, &prev_tick_ms);

	// Fuse accelerometer and gyroscope angles
	filter_fuse_angles(acc_euler, euler_est, alpha);

	// Store updated orientation estimates in output structure
	imu_est_euler->roll  = euler_est[0];
	imu_est_euler->pitch = euler_est[1];
	imu_est_euler->yaw   = euler_est[2];
}
