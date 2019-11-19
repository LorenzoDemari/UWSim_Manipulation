/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * DirectGeometry.cpp
 *
 * Code generation for function 'DirectGeometry'
 *
 */

/* Include files */
#include <cmath>
//#include "../Headers/DirectGeometry.h"

/* Function Definitions */
void DirectGeometry(double q, const double biTri[16], double jointType, double biTei[16])
{
  double r;
  double t;
  double d0;
  double dv0[16];
  int i0;
  int i1;
  int biTei_tmp;
  int b_biTei_tmp;

  /*  DirectGeometry Function  */
  /*  inputs:  */
  /*  - q : current links position (scalar); */
  /*        qi  = theta   [rad] around z-axis  */
  /*        OR  = distance [m]  in z-adirection */
  /*  - biTri: transformation matrix from link i to link i+1 for qi=0;  */
  /*  - jointType: 0 for revolute, 1 for prismatic; */
  /*  output: */
  /*  biTei : transformation matrix from link i to link i+1 for the input qi. */
  /*  biTri(:,:,i) */
  r = 0.0;
  t = 0.0;
  if (jointType == 0.0) {
    /*  rotational joint */
    /*  no translation */
    r = q;

    /*  rotation around z-axis */
  }

  if (jointType == 1.0) {
    /*  translational joint: */
    t = q;

    /*  translation in z-direction */
    r = 0.0;
  }

  /*  combined rotation & translation-matrix  */
  d0 = std::cos(r);
  dv0[0] = d0;
  r = std::sin(r);
  dv0[4] = -r;
  dv0[8] = 0.0;
  dv0[12] = 0.0;
  dv0[1] = r;
  dv0[5] = d0;
  dv0[9] = 0.0;
  dv0[13] = 0.0;
  dv0[2] = 0.0;
  dv0[6] = 0.0;
  dv0[10] = 1.0;
  dv0[14] = t;
  dv0[3] = 0.0;
  dv0[7] = 0.0;
  dv0[11] = 0.0;
  dv0[15] = 1.0;
  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      biTei_tmp = i1 << 2;
      b_biTei_tmp = i0 + biTei_tmp;
      biTei[b_biTei_tmp] = 0.0;
      biTei[b_biTei_tmp] = ((biTri[i0] * dv0[biTei_tmp] + biTri[i0 + 4] * dv0[1
        + biTei_tmp]) + biTri[i0 + 8] * dv0[2 + biTei_tmp]) + biTri[i0 + 12] * dv0[3 + biTei_tmp];
    }
  }

  /*  multiply matrices     */
}

/* End of code generation (DirectGeometry.cpp) */
