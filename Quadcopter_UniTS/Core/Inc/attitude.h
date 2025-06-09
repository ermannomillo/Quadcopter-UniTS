#ifndef AHRS_H
#define AHRS_H

#include "stm32h7xx_hal.h"
#include "quaternion.h"
#include "radio.h"

#define AHRS_DEBUG          0
#define COE_MDPS_TO_RADPS   1.745329252e-5
#define COE_RADS_TO_MDPS    (1/(1.745329252e-5))
#define NORM_R              0.25f
#define BIG_R               0.25f
#define AHRS_P_1            1e-1
#define AHRS_P_2            1e-0
#define AHRS_Q              1e-4
#define ACC_OVER_U          (1.0/1250)
#define ACC_OVER_D          (1.0/750)

// Sampling time of sensors
#define SENSOR_SAMPLING_TIME    0.00625f
#define alpha_att        1.0f
#define BETA_NORM    0.8f            //0.15
#define BETA_ZERO    0
#define zeta         0.006f             //0.003
#define AHRS_KP_BIG  10.0f               //0.4 is tested, so slow in calibrated. 
#define AHRS_KP_NORM 0.4f
#define AHRS_KI      0.1f


typedef struct
{
  Quaternion q;    // Current altitude
  float gx,gy,gz;       // Current angle rate @ body frame
}Attitude;

typedef struct
{
  float gx, gy, gz;
}Gyro;

typedef struct {
    int32_t AXIS_X;
    int32_t AXIS_Y;
    int32_t AXIS_Z;
} Axes_int;

typedef struct {
    float AXIS_X;
    float AXIS_Y;
    float AXIS_Z;
} Axes_float;


void attitude_fusion(Axes_float *acc, Axes_float *gyro, Attitude *ahrs, Radio *rc_comm);


#endif // AHRS_H

