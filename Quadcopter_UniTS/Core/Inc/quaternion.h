#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "stm32h7xx_hal.h"
#include "math_extension.h"


typedef struct
{
    float q0, q1, q2, q3;
} Quaternion;

typedef struct
{
    float pitch, roll, yaw;
}Euler;


void normalize_quat(Quaternion *q);
void multiply_quat(Quaternion *qa, Quaternion *qb, Quaternion *qo);
void rotate_quat(Quaternion *qr, Quaternion *qv, Quaternion *qo);
void conjugate_quat(Quaternion *qa, Quaternion *qo);
void quat_to_euler(Quaternion *qr, Euler *ea);


#define MAX_RAD  1.5




#endif /* _QUATERNION_H_ */
