#include "quaternion.h"
#include <math.h>

void QuaternionNorm(float q[])
{
    float norm;

    norm = fast_inv_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= norm;
    q[1] *= norm;
    q[2] *= norm;
    q[3] *= norm;
}

/*
 * Quaternion Multiplay - qo = qa * qb
 * note - qo can be different from qa/qb, or the same as qa/qb
 */
void multiply_quat(float qa[], float qb[], float qo[])
{
    float q0, q1, q2, q3;

    q0 = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
    q1 = qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2];
    q2 = qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1];
    q3 = qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0];
    qo[0] = q0; qo[1] = q1; qo[2] = q2; qo[3] = q3;
}


/*
 * This function calculate the vector rotaton via quatornion
 * qr - rotation quaternion
 * qv - vector to rotate (qv[0] = 0)
 * qo - output vector (qo[0] = 0)
 * qo = qr' * qv * qr
 */
void rotate_quat(float qr[], float qv[], float qo[])
{
    float q0q0, q1q1, q2q2, q3q3;
    float dq0, dq1, dq2;
    float dq1q2, dq1q3, dq0q2, dq0q3;
    float dq0q1, dq2q3;

    q0q0 = qr[0]*qr[0];
    q1q1 = qr[1]*qr[1];
    q2q2 = qr[2]*qr[2];
    q3q3 = qr[3]*qr[3];
    dq0 = 2*qr[0];
    dq1 = 2*qr[1];
    dq2 = 2*qr[2];
    dq1q2 = dq1 * qr[2];
    dq1q3 = dq1 * qr[3];
    dq0q2 = dq0 * qr[2];
    dq0q3 = dq0 * qr[3];
    dq0q1 = dq0 * qr[1];
    dq2q3 = dq2 * qr[3];

    qo[0] = 0;
    qo[1] = (q0q0+q1q1-q2q2-q3q3)*qv[1] + (dq1q2+dq0q3)*qv[2] + (dq1q3-dq0q2)*qv[3];
    qo[2] = (dq1q2-dq0q3)*qv[1] + (q0q0+q2q2-q1q1-q3q3)*qv[2] + (dq0q1+dq2q3)*qv[3];
    qo[3] = (dq0q2+dq1q3)*qv[1] + (dq2q3-dq0q1)*qv[2] + (q0q0+q3q3-q1q1-q2q2)*qv[3];
}

void conjugate_quat(float qa[], float qo[])
{
    qo[0] = qa[0];
    qo[1] = -qa[1];
    qo[2] = -qa[2];
    qo[3] = -qa[3];
}

/*
 * Convert Quaternion to Euler Angle
 */
void quat_to_euler(float qr[], float ea[])
{
    float q0q0, q1q1, q2q2, q3q3;
    float dq0, dq1, dq2;
    float dq1q3, dq0q2/*, dq1q2*/;
    float dq0q1, dq2q3/*, dq0q3*/;

    q0q0 = qr[0]*qr[0];
    q1q1 = qr[1]*qr[1];
    q2q2 = qr[2]*qr[2];
    q3q3 = qr[3]*qr[3];
    dq0 = 2*qr[0];
    dq1 = 2*qr[1];
    dq2 = 2*qr[2];
    //dq1q2 = dq1 * qr[2];
    dq1q3 = dq1 * qr[3];
    dq0q2 = dq0 * qr[2];
    //dq0q3 = dq0 * qr[3];
    dq0q1 = dq0 * qr[1];
    dq2q3 = dq2 * qr[3];

    ea[0] = atan2(dq0q1+dq2q3, q0q0+q3q3-q1q1-q2q2);
    ea[1] = asin(dq0q2-dq1q3);

    /* This part is removed to manage angle >90deg */
//    if(ea->thx > MAX_RAD || ea->thx < -MAX_RAD)
//      ea->thx = ea_pre.thx;
//    if(ea->thy > MAX_RAD || ea->thy < -MAX_RAD)
//      ea->thy = ea_pre.thy;
//
//    ea_pre.thx = ea->thx;
//    ea_pre.thy = ea->thy;
    
    
    

    //ea->thz = atan2(dq1q2+dq0q3, q0q0+q1q1-q2q2-q3q3);

}
