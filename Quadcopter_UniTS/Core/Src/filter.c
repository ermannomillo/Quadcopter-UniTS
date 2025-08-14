#include "filter.h"
#include "math.h"
#include "main.h"

// External variable for storing time delta between IMU readings
extern float dt_imu;

// Compute roll and pitch angles from accelerometer values
void filter_compute_gravity_angles(float gx, float gy, float gz, int euler[3]) {
	// Calculate roll angle from gyroscope Y and Z axis
	float roll  = atan2(gy, gz);
	// Calculate pitch angle from gyroscope X and magnitude of Y and Z
	float pitch = atan2(gx, sqrt(gy * gy + gz * gz));

	// Convert from radians to millidegrees and store in euler[0] and euler[1]
	euler[0] = (int)(roll * RAD_TO_MDEG);  // Roll in millidegrees
	euler[1] = (int)(pitch * RAD_TO_MDEG); // Pitch in millidegrees
}

// Integrate gyroscope values
void filter_integrate_gyro(int gyro[3], int gyro_delta[3], int euler_est[3], int* prev_tick_ms) {
	// Get current system tick in milliseconds
	int now = HAL_GetTick();

	// Compute time delta in seconds since last update
	dt_imu = (now - *prev_tick_ms) / 1000.0f;
	*prev_tick_ms = now; // Update previous tick

	// Perform numerical integration for each of the 3 axes (X, Y, Z)
	for (int i = 0; i < 3; i++) {
		float gyro_scaled = gyro[i] * SCALING;               // Scale gyro reading
		float gyro_delta_scaled = gyro_delta[i] * SCALING;   // Scale gyro delta

		// Update estimated euler angle using trapezoidal integration
		euler_est[i] += (int)((gyro_scaled + gyro_delta_scaled / 2.0f) * dt_imu);
		// euler_est[i] += (int)((gyro_scaled) * dt_imu);
	}
}

// Fuse accelerometer and gyroscope angles
void filter_fuse_angles(int acc_euler[3], int euler_est[3], float alpha) {
	// Apply complementary filter to pitch and roll
	for (int i = 0; i < 2; i++) {
		euler_est[i] = (int)(alpha * acc_euler[i] + (1.0f - alpha) * euler_est[i]);
	}
}




