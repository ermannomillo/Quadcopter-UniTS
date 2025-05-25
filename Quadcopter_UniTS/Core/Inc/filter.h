#ifndef _FILTER_H_
#define _FILTER_H_

#define RAD_TO_MDEG 57295.8f  // costante per la conversione rad → milligradi
#define SCALING (250000.0f / 32768.0f)

void filter_compute_gravity_angles(float gx, float gy, float gz, int euler[3]);
void filter_integrate_gyro(int gyro[3], int gyro_delta[3], int euler_est[3], int* prev_tick_ms);
void filter_fuse_angles(int acc_euler[3], int euler_est[3], float alpha);

#endif /* _FILTER_H_ */
