#ifndef ORIENTATION_H
#define ORIENTATION_H


typedef struct
{
	float pitch, roll, yaw;
}Euler;


void orientation_init(void);
void orientation_update(Euler *imu_est_euler);

#endif
