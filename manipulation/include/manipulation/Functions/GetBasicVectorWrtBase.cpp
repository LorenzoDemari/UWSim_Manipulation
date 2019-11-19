/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * GetBasicVectorWrtBase.cpp
 *
 * Code generation for function 'GetBasicVectorWrtBase'
 *
 */

/* Include files */
#include <string.h>
//#include "../Headers/GetBasicVectorWrtBase.h"

/* Function Definitions */
void GetBasicVectorWrtBase(const double biTei[96], double linkNumber, double r[3])
{
  double Tinterim[16];
  int i0;
  int i;
  int i1;
  double b_Tinterim[16];
  int i2;
  int Tinterim_tmp;
  int b_Tinterim_tmp;

  /*  GetBasicVectorWrtBase function  */
  /*  inputs : */
  /*  - biTei: vector of matrices containing the  */
  /*           transformation matrices from link i */
  /*           to link i +1. The */
  /*  size of biTri is equal to (4,4,numberOfLinks) */
  /*  - linkNumber: number of link for which computing  */
  /*                the basic vector from the base to the joint */
  /*  output: */
  /*  r : basic vector from the base to the input joint  */
  /*  initialize a vector with four rows */
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
  memset(&Tinterim[0], 0, sizeof(double) << 4);
  Tinterim[0] = 1.0;
  Tinterim[5] = 1.0;
  Tinterim[10] = 1.0;
  Tinterim[15] = 1.0;
  i0 = (int)linkNumber;
  for (i = 0; i < i0; i++) {
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < 4; i2++) {
        Tinterim_tmp = i2 << 2;
        b_Tinterim_tmp = i1 + Tinterim_tmp;
        b_Tinterim[b_Tinterim_tmp] = 0.0;
        Tinterim_tmp += i << 4;
        b_Tinterim[b_Tinterim_tmp] = ((Tinterim[i1] * biTei[Tinterim_tmp] +
          Tinterim[i1 + 4] * biTei[Tinterim_tmp + 1]) + Tinterim[i1 + 8] *
          biTei[Tinterim_tmp + 2]) + Tinterim[i1 + 12] * biTei[Tinterim_tmp + 3];
      }
    }

    memcpy(&Tinterim[0], &b_Tinterim[0], sizeof(double) << 4);
  }

  r[0] = Tinterim[12];
  r[1] = Tinterim[13];
  r[2] = Tinterim[14];
}

/* End of code generation (GetBasicVectorWrtBase.cpp) */
