#include <control.h>
#include <math.h>

float pid_x_integ1 = 0;
float pid_y_integ1 = 0;
float pid_z_integ1 = 0;
float pid_x_integ2 = 0;
float pid_y_integ2 = 0;
float pid_z_integ2 = 0;
float pid_x_pre_error2 = 0;
float pid_y_pre_error2 = 0;
float pid_z_pre_error2 = 0;
float pid_x_pre_deriv = 0;
float pid_y_pre_deriv = 0;

int16_t motor_thr;
float dt_recip;

void PID_init(Dual_PID_Control *pid)
{
  pid->ts = PID_SAMPLING_TIME;

  pid->x_kp1 = PITCH_PID_KP1;
  pid->x_ki1 = PITCH_PID_KI1;
  pid->x_i1_limit = PITCH_PID_I1_LIMIT;
  pid->x_kp2 = PITCH_PID_KP2;
  pid->x_ki2 = PITCH_PID_KI2;
  pid->x_kd2 = PITCH_PID_KD2;
  pid->x_i2_limit = PITCH_PID_I2_LIMIT;
  pid->x_s1 = 0;
  pid->x_s2 = 0;

  pid->y_kp1 = ROLL_PID_KP1;
  pid->y_ki1 = ROLL_PID_KI1;
  pid->y_i1_limit = ROLL_PID_I1_LIMIT;
  pid->y_kp2 = ROLL_PID_KP2;
  pid->y_ki2 = ROLL_PID_KI2;
  pid->y_kd2 = ROLL_PID_KD2;
  pid->y_i2_limit = ROLL_PID_I2_LIMIT;
  pid->y_s1 = 0;
  pid->y_s2 = 0;

  pid->z_kp1 = YAW_PID_KP1;
  pid->z_ki1 = YAW_PID_KI1;
  pid->z_i1_limit = YAW_PID_I1_LIMIT;
  pid->z_kp2 = YAW_PID_KP2;
  pid->z_ki2 = YAW_PID_KI2;
  pid->z_kd2 = YAW_PID_KD2;
  pid->z_i2_limit = YAW_PID_I2_LIMIT;
  pid->z_s1 = 0;
  pid->z_s2 = 0;
}


void PID_outer(Euler *euler_rc, Euler *euler_ahrs, Attitude *ahrs, Dual_PID_Control *pid, Radio *rc_comm)
{
  float error;

  if(rc_comm->THR<MIN_THR)
  {
    pid_x_integ1 = 0;
    pid_y_integ1 = 0;
    pid_z_integ1 = 0;
  }

  // x-axis PID
  error = euler_rc->thx - euler_ahrs->thx;
  pid_x_integ1 += error*pid->ts;
  if(pid_x_integ1 > pid->x_i1_limit)
    pid_x_integ1 = pid->x_i1_limit;
  else if(pid_x_integ1 < -pid->x_i1_limit)
    pid_x_integ1 = -pid->x_i1_limit;
  pid->x_s1 =  pid->x_kp1*error + pid->x_ki1*pid_x_integ1;

  // y-axis PID
  error = euler_rc->thy - euler_ahrs->thy;
  pid_y_integ1 += error*pid->ts;
  if(pid_y_integ1 > pid->y_i1_limit)
    pid_y_integ1 = pid->y_i1_limit;
  else if(pid_y_integ1 < -pid->y_i1_limit)
    pid_y_integ1 = -pid->y_i1_limit;
  pid->y_s1 =  pid->y_kp1*error + pid->y_ki1*pid_y_integ1;

  // z-axis PID
  error = euler_rc->thz - euler_ahrs->thz;
  pid_z_integ1 += error*pid->ts;
  if(pid_z_integ1 > pid->z_i1_limit)
    pid_z_integ1 = pid->z_i1_limit;
  else if(pid_z_integ1 < -pid->z_i1_limit)
    pid_z_integ1 = -pid->z_i1_limit;
  pid->z_s1 =  pid->z_kp1*error + pid->z_ki1*pid_z_integ1;
}

void PID_inner(Euler *euler_rc, Gyro *gyro_rad, Attitude *ahrs, Dual_PID_Control *pid, uint16_t motor_pwm[], Radio *rc_comm)
{
  float error, deriv;

  if(rc_comm->THR<MIN_THR)
  {
    pid_x_integ2 = 0;
    pid_y_integ2 = 0;
    pid_z_integ2 = 0;
  }
  
  dt_recip = 1/pid->ts;

  // X Axis
  error = pid->x_s1 - gyro_rad->gx;
  pid_x_integ2 += error*pid->ts;
  if(pid_x_integ2 > pid->x_i2_limit)
    pid_x_integ2 = pid->x_i2_limit;
  else if(pid_x_integ2 < -pid->x_i2_limit)
    pid_x_integ2 = -pid->x_i2_limit;
  deriv = (error - pid_x_pre_error2)*dt_recip;
  pid_x_pre_error2 = error;
  deriv = pid_x_pre_deriv + (deriv - pid_x_pre_deriv)*D_FILTER_COFF;
  pid_x_pre_deriv = deriv;
  pid->x_s2 = pid->x_kp2*error + pid->x_ki2*pid_x_integ2 + pid->x_kd2*deriv;
  
  if(pid->x_s2 > MAX_ADJ_AMOUNT)  pid->x_s2 = MAX_ADJ_AMOUNT;
  if(pid->x_s2 < -MAX_ADJ_AMOUNT)  pid->x_s2 = -MAX_ADJ_AMOUNT;

  // Y Axis
  error = pid->y_s1 - gyro_rad->gy;
  pid_y_integ2 += error*pid->ts;
  if(pid_y_integ2 > pid->y_i2_limit)
    pid_y_integ2 = pid->y_i2_limit;
  else if(pid_y_integ2 < -pid->y_i2_limit)
    pid_y_integ2 = -pid->y_i2_limit;
  deriv = (error - pid_y_pre_error2)*dt_recip;
  pid_y_pre_error2 = error;
  deriv = pid_y_pre_deriv + (deriv - pid_y_pre_deriv)*D_FILTER_COFF;
  pid_y_pre_deriv = deriv;
  pid->y_s2 = pid->y_kp2*error + pid->y_ki2*pid_y_integ2 + pid->y_kd2*deriv;

  if(pid->y_s2 > MAX_ADJ_AMOUNT)  pid->y_s2 = MAX_ADJ_AMOUNT;
  if(pid->y_s2 < -MAX_ADJ_AMOUNT)  pid->y_s2 = -MAX_ADJ_AMOUNT;

  // Z Axis
  error = pid->z_s1 - gyro_rad->gz;
  pid_z_integ2 += error*pid->ts;
  if(pid_z_integ2 > pid->z_i2_limit)
    pid_z_integ2 = pid->z_i2_limit;
  else if(pid_z_integ2 < -pid->z_i2_limit)
    pid_z_integ2 = -pid->z_i2_limit;
  deriv = (error - pid_z_pre_error2)*dt_recip;
  pid_z_pre_error2 = error;
  pid->z_s2 = pid->z_kp2*error + pid->z_ki2*pid_z_integ2 + pid->z_kd2*deriv;

  if(pid->z_s2 > MAX_ADJ_AMOUNT_YAW)  pid->z_s2 = MAX_ADJ_AMOUNT_YAW;
  if(pid->z_s2 < -MAX_ADJ_AMOUNT_YAW)  pid->z_s2 = -MAX_ADJ_AMOUNT_YAW;

  motor_thr = 0.28f*rc_comm->THR + 750.0f;

  motor_pwm[0] = motor_thr - pid->x_s2 - pid->y_s2 + pid->z_s2 + MOTOR_OFF1;
  motor_pwm[1] = motor_thr + pid->x_s2 - pid->y_s2 - pid->z_s2 + MOTOR_OFF2;
  motor_pwm[2] = motor_thr + pid->x_s2 + pid->y_s2 + pid->z_s2 + MOTOR_OFF3;
  motor_pwm[3] = motor_thr - pid->x_s2 + pid->y_s2 - pid->z_s2 + MOTOR_OFF4;

}

