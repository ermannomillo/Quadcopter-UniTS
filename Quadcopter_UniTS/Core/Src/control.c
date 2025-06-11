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

