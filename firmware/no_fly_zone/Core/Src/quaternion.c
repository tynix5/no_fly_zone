#include "quaternion.h"
#include "math.h"
#include "arm_math.h"

void quat_mult_scalar(Quaternion * q, float k)
{
    q->q1 = q->q1 * k;
    q->q2 = q->q2 * k;
    q->q3 = q->q3 * k;
    q->q4 = q->q4 * k;
}

void quat_mult(Quaternion q_1, Quaternion q_2, Quaternion * q_prod)
{
    // Hamiltonian product
    q_prod->q1 = q_1.q1 * q_2.q1 - q_1.q2 * q_2.q2 - q_1.q3 * q_2.q3 - q_1.q4 * q_2.q4;     // real
    q_prod->q2 = q_1.q1 * q_2.q2 + q_1.q2 * q_2.q1 + q_1.q3 * q_2.q4 - q_1.q4 * q_2.q3;     // imag, i
    q_prod->q3 = q_1.q1 * q_2.q3 - q_1.q2 * q_2.q4 + q_1.q3 * q_2.q1 + q_1.q4 * q_2.q2;     // imag, j
    q_prod->q4 = q_1.q1 * q_2.q4 + q_1.q2 * q_2.q3 - q_1.q3 * q_2.q2 + q_1.q4 * q_2.q1;     // imag, k
}

void quat_add(Quaternion q_1, Quaternion q_2, Quaternion * q_sum)
{
    // add likewise components
    q_sum->q1 = q_1.q1 + q_2.q1;
    q_sum->q2 = q_1.q2 + q_2.q2;
    q_sum->q3 = q_1.q3 + q_2.q3;
    q_sum->q4 = q_1.q4 + q_2.q4;
}

void quat_sub(Quaternion q_1, Quaternion q_2, Quaternion * q_diff)
{
    // subtract likewise components
    q_diff->q1 = q_1.q1 - q_2.q1;
    q_diff->q2 = q_1.q2 - q_2.q2;
    q_diff->q3 = q_1.q3 - q_2.q3;
    q_diff->q4 = q_1.q4 - q_2.q4;
}

void quat_normalize(Quaternion * q)
{
    // calculate magnitude
    float mag = sqrtf(q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3 + q->q4 * q->q4);

    // divide each compoonent by magnitude to get unit vector
    q->q1 = q->q1 / mag;
    q->q2 = q->q2 / mag;
    q->q3 = q->q3 / mag;
    q->q4 = q->q4 / mag;
}

void quat_to_euler(Quaternion q, float * pitch, float * roll, float * yaw)
{
    // normalize quaternion before conversion
    quat_normalize(&q);

    // test for singularities near gimbal lock
    float test = 2.0 * (q.q2 * q.q3 - q.q1 * q.q4);

    if (test > 0.499)       // north pole
    {
        *pitch = PI / 2.0;
        *yaw = 2.0 * atan2f(q.q2, q.q1);
        *roll = 0.0;

    }
    else if (test < -0.499)     // south pole
    {
        *pitch = -PI / 2.0;
        *yaw = -2 * atan2f(q.q2, q.q1);
        *roll = 0.0;
    }
    else        // compute normally
    {
        *pitch = asinf((2.0 * (q.q1 * q.q3 - q.q2 * q.q4)));
        // *pitch = asinf((2.0 * (q.q2 * q.q3 - q.q1 * q.q4)));
        *roll = atan2f((2.0 * (q.q1 * q.q2 + q.q3 * q.q4)), (1.0 - 2.0 * (q.q3 * q.q3 + q.q4 * q.q4)));
        *yaw = atan2f((2.0 * (q.q1 * q.q3 + q.q2 * q.q4)), (1.0 - 2.0 * (q.q2 * q.q2 + q.q3 * q.q3)));
    }
}