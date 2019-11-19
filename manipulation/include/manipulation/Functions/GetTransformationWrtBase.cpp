/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * GetTransformationWrtBase.cpp
 *
 * Code generation for function 'GetTransformationWrtBase'
 *
 */

/* Include files */
#include <string.h>
//#include "../Headers/GetTransformationWrtBase.h"

/* Function Definitions */
void GetTransformationWrtBase(const double biTei[96], double linkNumber, double
  bTi[16])
{
  int i0;
  int i;
  int i1;
  double b_bTi[16];
  int i2;
  int bTi_tmp;
  int b_bTi_tmp;

  /*  GetTransformatioWrtBase function */
  /*  inputs: */
  /*  - biTei: vector of matrices containing the  */
  /*           transformation matrices from link i  */
  /*           to link i+1 for the current q. */
  /*  - linkNumber: for which computing the  */
  /*                transformation matrix; */
  /*  output: */
  /*  - bTi: transformation matrix from the manipulator */
  /*         base to the ith joint in the configuration  */
  /*         identified by biTei. */
  memset(&bTi[0], 0, sizeof(double) << 4);
  bTi[0] = 1.0;
  bTi[5] = 1.0;
  bTi[10] = 1.0;
  bTi[15] = 1.0;
  i0 = (int)linkNumber;
  for (i = 0; i < i0; i++) {
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < 4; i2++) {
        bTi_tmp = i2 << 2;
        b_bTi_tmp = i1 + bTi_tmp;
        b_bTi[b_bTi_tmp] = 0.0;
        bTi_tmp += i << 4;
        b_bTi[b_bTi_tmp] = ((bTi[i1] * biTei[bTi_tmp] + bTi[i1 + 4] * biTei[bTi_tmp + 1]) +
                bTi[i1 + 8] * biTei[bTi_tmp + 2]) + bTi[i1 + 12] * biTei[bTi_tmp + 3];
      }
    }

    memcpy(&bTi[0], &b_bTi[0], sizeof(double) << 4);
  }
}

/* End of code generation (GetTransformationWrtBase.cpp) */
