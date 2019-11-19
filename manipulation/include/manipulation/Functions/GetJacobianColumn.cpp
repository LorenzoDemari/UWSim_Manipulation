/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * GetJacobianColumn.cpp
 *
 * Code generation for function 'GetJacobianColumn'
 *
 */

/* Include files */
//#include "../Headers/GetJacobianColumn.h"

/* Function Definitions */
void GetJacobianColumn(const double bTei[16], const double bTe[16], double
  jointType, double h[6])
{
  double re_idx_0;
  double re_idx_1;
  double re_idx_2;

  /*  Inverse Kinematic Function */
  /*  Function computing the end effector jacobian column for the input */
  /*  parameters. */
  /*  */
  /*  Inputs */
  /*  - bTei: transformation matrix from the base to the ith joint for the */
  /*    current configuration */
  /*  - bTe: tranformation matrix from the base to the end effector */
  /*  - jointType: 0 if the joint is revolute,1 if the joint is prismatic (is */
  /*    referred to the joint corresponding to bTe). */
  /*  */
  /*  Output */
  /*  - h: Jacobian column h(1:3) angular part, h(4:6) linear */
  if (jointType == 0.0) {
    /*  for rotational joint   */
    re_idx_0 = bTe[12] - bTei[12];
    h[0] = bTei[8];
    re_idx_1 = bTe[13] - bTei[13];
    h[1] = bTei[9];
    re_idx_2 = bTe[14] - bTei[14];
    h[2] = bTei[10];
    h[3] = bTei[9] * re_idx_2 - bTei[10] * re_idx_1;
    h[4] = bTei[10] * re_idx_0 - bTei[8] * re_idx_2;
    h[5] = bTei[8] * re_idx_1 - bTei[9] * re_idx_0;
  } else {
    /*  for prismatic joint */
    h[0] = 0.0;
    h[3] = bTei[8];
    h[1] = 0.0;
    h[4] = bTei[9];
    h[2] = 0.0;
    h[5] = bTei[10];
  }
}

/* End of code generation (GetJacobianColumn.cpp) */
