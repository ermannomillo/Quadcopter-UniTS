
#include <radio.h>

const float max_pitch_rad = PI*PITCH_MAX_DEG/180.0f;
const float max_roll_rad = PI*ROLL_MAX_DEG/180.0f;
const float max_yaw_rad = PI*YAW_MAX_DEG/180.0f;
int t1;


void get_target_euler(float euler_rc[], float rc_comm[])
{
    t1 = rc_comm[0];
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;
    euler_rc[0] = -t1 * max_pitch_rad / RC_FULLSCALE;

    t1 = rc_comm[1];
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;
    euler_rc[1] = -t1 * max_roll_rad / RC_FULLSCALE;

    t1 = rc_comm[2];
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;

    if(t1 > EULER_Z_TH)
    {
        euler_rc[3] = euler_rc[3] + max_yaw_rad;
    }
    else if(t1 < -EULER_Z_TH)
    {
        euler_rc[3] = euler_rc[3] - max_yaw_rad;
    }
}
