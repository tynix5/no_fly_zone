#ifndef QUATERNION__H_
#define QUATERNION__H_

typedef struct {

    float q1;           // real
    float q2;           // imag, i
    float q3;           // imag, j
    float q4;           // imag, k

} Quaternion;

/* Scale all elements in quaternion */
void quat_mult_scalar(Quaternion * q, float k);
/* Hamiltonian multiply two quaternions */
void quat_mult(Quaternion q1, Quaternion q2, Quaternion * q_prod);
/* Element-wise addition of two quaternions */
void quat_add(Quaternion q1, Quaternion q2, Quaternion * q_sum);
/* Element-wise subtraction of two quaternions */
void quat_sub(Quaternion q_1, Quaternion q_2, Quaternion * q_diff);
/* Normalize quaternion to unit vector */
void quat_normalize(Quaternion * q);
/* Convert from quaternion to Euler angle (rad) */
void quat_to_euler(Quaternion q, float * pitch, float * roll, float * yaw);


#endif