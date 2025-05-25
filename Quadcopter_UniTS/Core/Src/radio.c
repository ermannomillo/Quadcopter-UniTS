
#include <radio.h>
#include "quaternion.h"

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
    euler_rc->thx = -t1 * max_pitch_rad / RC_FULLSCALE;

    t1 = rc_comm->AIL;
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;
    euler_rc->thy = -t1 * max_roll_rad / RC_FULLSCALE;

    t1 = rc_comm->RUD;
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;

    if(t1 > YAW_DEAD_THR)
    {
        euler_rc->thz = euler_rc->thz + max_yaw_rad;
    }
    else if(t1 < -YAW_DEAD_THR)
    {
        euler_rc->thz = euler_rc->thz - max_yaw_rad;
    }
}
