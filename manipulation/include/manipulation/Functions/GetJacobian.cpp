/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * GetJacobian.cpp
 *
 * Code generation for function 'GetJacobian'
 *
 */

/* Include files */
//#include "../Headers/GetJacobian.h"

/* Function Definitions */
void GetJacobian(const double bTei[96], const double bTe[16], const double jointType[6], double J[36])
{
  int i;
  double h[6];
  double re_idx_0;
  int h_tmp;
  double re_idx_1;
  double re_idx_2;
  double b_h_tmp;
  double c_h_tmp;

  /*  GetJacobian function */
  /*  Function returning the end effector jacobian for a manipulator which current */
  /*  configuration is described by bTei. */
  /*  */
  /*  Inputs: */
  /*  - bTei: vector of matrices containing the transformation matrices from */
  /*  base to joint i for the current configuration. */
  /*  - bTe: current transformation matrix from base to the end effector. */
  /*  - jointType: vector identifying the joint type, 0 for revolute, 1 for */
  /*  prismatic */
  /*  */
  /*  Output: */
  /*  - J: end-effector jacobian matrix */
  /*  TODO */
  for (i = 0; i < 6; i++) {
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
    if (jointType[i] == 0.0) {
      /*  for rotational joint   */
      re_idx_0 = bTe[12] - bTei[12 + (i << 4)];
      h[0] = bTei[8 + (i << 4)];
      re_idx_1 = bTe[13] - bTei[(i << 4) + 13];
      h[1] = bTei[(i << 4) + 9];
      re_idx_2 = bTe[14] - bTei[(i << 4) + 14];
      h[2] = bTei[(i << 4) + 10];
      b_h_tmp = bTei[10 + (i << 4)];
      c_h_tmp = bTei[9 + (i << 4)];
      h[3] = c_h_tmp * re_idx_2 - b_h_tmp * re_idx_1;
      h[4] = b_h_tmp * re_idx_0 - bTei[8 + (i << 4)] * re_idx_2;
      h[5] = bTei[8 + (i << 4)] * re_idx_1 - c_h_tmp * re_idx_0;
    } else {
      /*  for prismatic joint */
      h[0] = 0.0;
      h_tmp = i << 4;
      h[3] = bTei[8 + h_tmp];
      h[1] = 0.0;
      h[4] = bTei[h_tmp + 9];
      h[2] = 0.0;
      h[5] = bTei[h_tmp + 10];
    }

    for (h_tmp = 0; h_tmp < 6; h_tmp++) {
      J[h_tmp + 6 * i] = h[h_tmp];
    }
  }
}

/* End of code generation (GetJacobian.cpp) */
