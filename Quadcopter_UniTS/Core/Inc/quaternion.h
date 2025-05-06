#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "stm32h7xx_hal.h"
#include "math_extension.h"


void normalize_quat(float q[]);
void multiply_quat(float qa[], float qb[], float qo[]);
void rotate_quat(float qr[], float qv[], float qo[]);
void conjugate_quat(float qa[], float qo[]);
void quat_to_euler(float qr[], float ea[]);

#define MAX_RAD  1.5


#endif /* _QUATERNION_H_ */
