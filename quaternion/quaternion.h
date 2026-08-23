#ifndef QUATERNION__H_
#define QUATERNION__H_

typedef struct
{

    float q1; // real
    float q2; // imag, i
    float q3; // imag, j
    float q4; // imag, k
} quaternion_t;

/* Copy elements of quaternion */
void quat_copy(quaternion_t q_src, quaternion_t * q_dest);
/* Conjugate quaternion */
void quat_conjugate(quaternion_t * q);
/* Scale all elements in quaternion */
void quat_mult_scalar(quaternion_t * q, float k);
/* Hamiltonian multiply two quaternions */
void quat_mult(quaternion_t q1, quaternion_t q2, quaternion_t * q_prod);
/* Element-wise addition of two quaternions */
void quat_add(quaternion_t q1, quaternion_t q2, quaternion_t * q_sum);
/* Element-wise subtraction of two quaternions */
void quat_sub(quaternion_t q_1, quaternion_t q_2, quaternion_t * q_diff);
/* Normalize quaternion to unit vector */
void quat_normalize(quaternion_t * q);
/* Convert from quaternion to Euler angle (rad) */
void quat_to_euler(quaternion_t q, float * pitch, float * roll, float * yaw);
/* Convert from euler (rad) to quaternion */
void euler_to_quat(float pitch, float roll, float yaw, quaternion_t * q);

#endif