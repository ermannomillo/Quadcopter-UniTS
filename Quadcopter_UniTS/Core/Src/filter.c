#include "filter.h"
#include "math.h"
#include "main.h"

extern float dt;

void filter_compute_gravity_angles(float gx, float gy, float gz, int euler[3]) {
    float roll  = atan2(gy, gz);
    float pitch = atan2(-gx, sqrt(gy * gy + gz * gz));

    euler[0] = (int)(roll * RAD_TO_MDEG);
    euler[1] = (int)(pitch * RAD_TO_MDEG);
}

void filter_integrate_gyro(int gyro[3], int gyro_delta[3], int euler_est[3], int* prev_tick_ms) {
    int now = HAL_GetTick();
    dt = (now - *prev_tick_ms) / 1000.0f;
    *prev_tick_ms = now;

    for (int i = 0; i < 3; i++) {
    	float gyro_scaled = gyro[i] * SCALING;
    	float gyro_delta_scaled = gyro_delta[i] * SCALING;
    	euler_est[i] += (int)((gyro_scaled + gyro_delta_scaled / 2.0f) * dt);
    }
}

void filter_fuse_angles(int acc_euler[3], int euler_est[3], float alpha) {
    for (int i = 0; i < 2; i++) {
        euler_est[i] = (int)(alpha * acc_euler[i] + (1.0f - alpha) * euler_est[i]);
    }
}



