#ifndef _RC_H_
#define _RC_H_

#include "orientation.h"
#include "main.h"

#define RC_FULLSCALE        10000 //1000
#define NUM_ITERATIONS_RC_CAL 10
#define MAX_THROTTLE_RC 0.80f

#define PI  3.141592654f

#define PITCH_MAX_DEG   30
#define ROLL_MAX_DEG    30
#define YAW_MAX_DEG     (60.0f * 0.00002)

#define YAW_DEAD_THRD    300


// UDS keeping commands
typedef struct
{
	float AIL, ELE, THR, RUD;
} Radio;


void get_target_euler(Euler *euler_rc, Radio *rc_comm);
void calibrate_rc();


#endif /* _RC_H_ */
