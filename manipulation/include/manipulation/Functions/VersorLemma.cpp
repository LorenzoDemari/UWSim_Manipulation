/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * VersorLemma.cpp
 *
 * Code generation for function 'VersorLemma'
 *
 */

/* Include files */
#include <cmath>
//#include "../Headers/VersorLemma.h"

/* Function Definitions */
void VersorLemma(const double r1[9], const double r2[9], double c[3])
{
  double theta;

  /*  VersorLemma function */
  /*  inputs: */
  /*  - r1: the first rotation matrix */
  /*  - r2: the second rotation matrix */
  /*  output: */
  /*  - c: the Vect3 representing the axis around which r1 should rotate to reach r2, where its modulus is the angle */
  /* TODO */
  theta = std::acos((((((r1[0] * r2[0] + r1[1] * r2[1]) + r1[2] * r2[2]) + ((r1
    [3] * r2[3] + r1[4] * r2[4]) + r1[5] * r2[5])) + ((r1[6] * r2[6] + r1[7] *
    r2[7]) + r1[8] * r2[8])) - 1.0) / 2.0);
  if (theta == 0.0) {
    c[0] = 0.0;
    c[1] = 0.0;
    c[2] = 0.0;
  } else {
    c[0] = r1[1] * r2[2] - r1[2] * r2[1];
    c[1] = r1[2] * r2[0] - r1[0] * r2[2];
    c[2] = r1[0] * r2[1] - r1[1] * r2[0];
    theta = 2.0 * std::sin(theta);
    c[0] = ((c[0] + (r1[4] * r2[5] - r1[5] * r2[4])) + (r1[7] * r2[8] - r1[8] * r2[7])) / theta;
    c[1] = ((c[1] + (r1[5] * r2[3] - r1[3] * r2[5])) + (r1[8] * r2[6] - r1[6] * r2[8])) / theta;
    c[2] = ((c[2] + (r1[3] * r2[4] - r1[4] * r2[3])) + (r1[6] * r2[7] - r1[7] * r2[6])) / theta;

    /* has the module 1 */
  }
}

/* End of code generation (VersorLemma.cpp) */
