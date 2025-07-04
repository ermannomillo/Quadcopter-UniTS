#ifndef _RC_H_
#define _RC_H_

#include "orientation.h"
#include "main.h"

#define RC_FULLSCALE        1000

#define PI  3.141592654f

#define PITCH_MAX_DEG   30
#define ROLL_MAX_DEG    30
#define RADIO_SAMPLING_TIME    0.02 // 50 Hz RC frequency

#define YAW_MAX_DEG     (60.0f * RADIO_SAMPLING_TIME)
#define YAW_MIN_RAD     0.0872

#define YAW_DEAD_THR    300

#define NUM_ITERATIONS_RC_CAL 10

#define MAX_THROTTLE_RC 0.80f


typedef struct
{
	float AIL, ELE, THR, RUD;
} Radio;



void get_target_euler(Euler *euler_rc, Radio *rc_comm);
void calibrate_rc();


#endif /* _RC_H_ */
