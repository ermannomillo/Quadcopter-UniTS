
#include <radio.h>
#include "quaternion.h"
#include "filter.h"

const float max_pitch_rad = PI*PITCH_MAX_DEG/180.0f;
const float max_roll_rad = PI*ROLL_MAX_DEG/180.0f;
const float max_yaw_rad = PI*YAW_MAX_DEG/180.0f;
int t1;


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
