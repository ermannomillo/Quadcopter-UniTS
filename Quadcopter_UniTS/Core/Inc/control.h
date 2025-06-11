#ifndef _FLIGHT_CONTROL_H_
#define _FLIGHT_CONTROL_H_

#include "stm32h7xx_hal.h"
#include "quaternion.h"
#include "motor.h"


#define I_TERM_LIMIT 50000

#define MIN_THR              20                /* External ESC configuration */


#define MAX_ADJ_AMOUNT       800
#define MAX_ADJ_AMOUNT_YAW   800
#define MAX_ACC_DIFF         10
#define MAX_ACC_DATA         125

#define GYRO_FIL_K           0.05
#define ACC_FIL_K            0.05
#define MAG_FIL_K            1.0
#define EULER_FIL_K          0.001
#define EULER_OFFSET_LIMIT   0.015            //5 degree  0.09
#define EULER_NEAR_ZERO      0.005            //1 degree  0.01745

#define ROLL_PID_KP1         3
#define ROLL_PID_KI1         0
#define ROLL_PID_KP2         80
#define ROLL_PID_KI2         80
#define ROLL_PID_KD2         10           //(x/PID_SAMPLING_TIME)
#define ROLL_PID_I1_LIMIT    2.0         //5 degree
#define ROLL_PID_I2_LIMIT    20.0


#define PITCH_PID_KP1        ROLL_PID_KP1
#define PITCH_PID_KI1        ROLL_PID_KI1
#define PITCH_PID_KP2        ROLL_PID_KP2
#define PITCH_PID_KI2        ROLL_PID_KI2
#define PITCH_PID_KD2        ROLL_PID_KD2
#define PITCH_PID_I1_LIMIT   ROLL_PID_I1_LIMIT
#define PITCH_PID_I2_LIMIT   ROLL_PID_I2_LIMIT

#define YAW_PID_KP1         4
#define YAW_PID_KI1         0

#define YAW_PID_KP2         900
#define YAW_PID_KI2         3
#define YAW_PID_KD2         3
#define YAW_PID_I1_LIMIT    2.0         //5 degree
#define YAW_PID_I2_LIMIT    2

#define PID_SAMPLING_TIME   0.00125f

#define D_FILTER_COFF       0.025f

#define MOTOR_OFF1          0
#define MOTOR_OFF2          0
#define MOTOR_OFF3          0
#define MOTOR_OFF4          0


#define X_AXIS_OFFSET       -0.056
#define Y_AXIS_OFFSET       -0.029

#define GYRO_OFFSET_X       0.094
#define GYRO_OFFSET_Y       0.060
#define GYRO_OFFSET_Z      -0.064

#define FIFO_Order          5
#define MID_FIFO            (FIFO_Order>>1)
#define FIFO_Order_Recip    (1.0/FIFO_Order)

// Structure for P-PI type PID control
//first stage is angle stage, second stage is angle rate stage.
typedef struct
{
    float ts;                                  // sampling time
    float x_kp1, x_ki1, x_kp2, x_ki2, x_kd2;          // stage pid parameter
    float y_kp1, y_ki1, y_kp2, y_ki2, y_kd2;
    float z_kp1, z_ki1, z_kp2, z_ki2, z_kd2;
    float x_i1_limit, y_i1_limit, z_i1_limit;
    float x_i2_limit, y_i2_limit, z_i2_limit;
    float x_s1, x_s2;                   // stage output
    float y_s1, y_s2;
    float z_s1, z_s2;
}Dual_PID_Control;


typedef struct
{
    float p_error[3];
    float i_error[3];
    float d_error[3];

}PID_Error;

typedef struct
{
    float pitch;
    float roll;
    float yaw;

}PID_Out;


#endif
