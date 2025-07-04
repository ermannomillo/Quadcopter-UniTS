
#include <radio.h>
#include "orientation.h"
#include "filter.h"
#include "main.h"

const float max_pitch_rad = PI*PITCH_MAX_DEG/180.0f;
const float max_roll_rad = PI*ROLL_MAX_DEG/180.0f;
const float max_yaw_rad = PI*YAW_MAX_DEG/180.0f;
int t1;

extern volatile uint32_t pulse_on_rc_0;
extern volatile uint32_t pulse_on_rc_1;
extern volatile uint32_t pulse_on_rc_2;
extern volatile uint32_t pulse_on_rc_3;

extern uint16_t offset_rc_0;
extern uint16_t offset_rc_1;
extern uint16_t offset_rc_2;
extern uint16_t offset_rc_3;


void get_target_euler(Euler *euler_rc, Radio *rc_comm)
{
    t1 = rc_comm->ELE;
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;
    euler_rc->pitch =  (int)( (-t1 * max_pitch_rad / RC_FULLSCALE) * RAD_TO_MDEG);

    t1 = rc_comm->AIL;
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;
    euler_rc->roll =  (int)((-t1 * max_roll_rad / RC_FULLSCALE) * RAD_TO_MDEG);

    t1 = rc_comm->RUD;
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;

    if(t1 > YAW_DEAD_THR)
    {
        euler_rc->yaw = euler_rc->yaw +  (int)(max_yaw_rad * RAD_TO_MDEG);
    }
    else if(t1 < -YAW_DEAD_THR)
    {
        euler_rc->yaw =  euler_rc->yaw -  (int)(max_yaw_rad * RAD_TO_MDEG);
    }
}

void calibrate_rc() {

	uint16_t sum_pulse_on_rc_0_cal = 0;
	uint16_t sum_pulse_on_rc_1_cal = 0;
	uint16_t sum_pulse_on_rc_2_cal = 0;
	uint16_t sum_pulse_on_rc_3_cal = 0;

	int i = 0;
	while ( i < NUM_ITERATIONS_RC_CAL) {
	      if (pulse_on_rc_0 > 3000 ) {
	    	  sum_pulse_on_rc_0_cal += pulse_on_rc_0;
	      	  i++;
	  	  }
	}

	offset_rc_0 = (int) sum_pulse_on_rc_0_cal / NUM_ITERATIONS_RC_CAL;

	i = 0;
	while ( i < NUM_ITERATIONS_RC_CAL) {
		if (pulse_on_rc_1 > 3000 ) {
			sum_pulse_on_rc_1_cal += pulse_on_rc_1;
	        i++;
		}
	}
	offset_rc_1 = (int) sum_pulse_on_rc_1_cal / NUM_ITERATIONS_RC_CAL;

	i = 0;
	while ( i < NUM_ITERATIONS_RC_CAL) {
		if (pulse_on_rc_2 > 3000 ) {
			sum_pulse_on_rc_2_cal += pulse_on_rc_2;
	        i++;
		}
	}
	offset_rc_2 = (int) sum_pulse_on_rc_2_cal / NUM_ITERATIONS_RC_CAL;

	i = 0;
	while ( i < NUM_ITERATIONS_RC_CAL) {
		if (pulse_on_rc_3 > 2000 ) {
			sum_pulse_on_rc_3_cal += pulse_on_rc_3;
			i++;
		}
	}
	offset_rc_3 = (int) sum_pulse_on_rc_3_cal / NUM_ITERATIONS_RC_CAL;

}
