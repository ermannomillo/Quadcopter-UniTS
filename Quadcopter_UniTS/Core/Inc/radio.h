#ifndef _RC_H_
#define _RC_H_


#define RC_FULLSCALE        1800


#define PI  3.141592654f

#define PITCH_MAX_DEG   30
#define ROLL_MAX_DEG    30

//#define YAW_MAX_DEG     (180.0*SENSOR_SAMPLING_TIME)
#define YAW_MAX_DEG     2.0f //(60.0f*SENSOR_SAMPLING_TIME)
#define YAW_MIN_RAD     0.0872

#define EULER_Z_TH      600

void get_target_euler(float euler_rc[], float rc_comm[]);

#endif /* _RC_H_ */
