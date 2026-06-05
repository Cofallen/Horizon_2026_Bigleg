#include "Revolve.h"

/*四元数的逆：+q0,-q1,-q2,-q3*/
void quaternion_inverse(float *q_in , float *q_out) {

  float norm_sq = q_in[0] * q_in[0] + q_in[1] * q_in[1] + q_in[2] * q_in[2] + q_in[3] * q_in[3];
  if (norm_sq > 0.0) {
    q_out[0] = +q_in[0] / norm_sq;
    q_out[1] = -q_in[1] / norm_sq;
    q_out[2] = -q_in[2] / norm_sq;
    q_out[3] = -q_in[3] / norm_sq;
  } else {
    q_out[0] = 0.0f;
    q_out[1] = 0.0f;
    q_out[2] = 0.0f;
    q_out[3] = 0.0f;
  }
}

/*四元数乘法：q1*P2（P2*q1）
 =[+q0 -q1 -q2 -q3    [+p0
   +q1 -q0 -q3 +q2  *  +p1
   +q2 +q3 +q0 -q1     +p2
   +q3 -q2 +q1 +q0]    +p3]
 =[+p0 -p1 -p2 -p3    [+q0
   +p1 -p0 -p3 +p2  *  +q1
   +p2 +p3 +p0 -p1     +q2
   +p3 -p2 +p1 +p0]    +q3]
 */
void quaternion_multiply(float *q1, float *q2, float *q3) {

  q3[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
  q3[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
  q3[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
  q3[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

/*误差四元数结合杆长得出旋转矩阵*/
void Quaternion_to_T_Matrix(float *Q_dest , float *Q_src , T_Matrix_Data_typdef *T_Matrix , float L)
{
  float q_inv[4];
  float q_out[4];

  quaternion_inverse(Q_src , q_inv);
  quaternion_multiply(Q_dest , q_inv , q_out);

  T_Matrix->T_out[0] = 1 - 2*q_out[2]*q_out[2] - 2*q_out[3]*q_out[3];
  T_Matrix->T_out[1] = 2*q_out[1]*q_out[2] - 2*q_out[3]*q_out[0];
  T_Matrix->T_out[2] = 2*q_out[1]*q_out[3] + 2*q_out[2]*q_out[0];

  T_Matrix->T_out[4] = 2*q_out[1]*q_out[2] + 2*q_out[3]*q_out[0];
  T_Matrix->T_out[5] = 1 - 2*q_out[1]*q_out[1] - 2*q_out[3]*q_out[3];
  T_Matrix->T_out[6] = 2*q_out[2]*q_out[3] - 2*q_out[1]*q_out[0];

  T_Matrix->T_out[8] = 2*q_out[1]*q_out[3] - 2*q_out[2]*q_out[0];
  T_Matrix->T_out[9] = 2*q_out[2]*q_out[3] + 2*q_out[1]*q_out[0];
  T_Matrix->T_out[10] = 1 - 2*q_out[1]*q_out[1] - 2*q_out[2]*q_out[2];

  T_Matrix->T_out[3] = L * T_Matrix->T_out[0];
  T_Matrix->T_out[7] = L * T_Matrix->T_out[4];
  T_Matrix->T_out[11] = L * T_Matrix->T_out[8];

  T_Matrix->T_out[12] = 0;
  T_Matrix->T_out[13] = 0;
  T_Matrix->T_out[14] = 0;
  T_Matrix->T_out[15] = 1;
}
