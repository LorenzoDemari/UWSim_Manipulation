/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * GetJacobian.h
 *
 * Code generation for function 'GetJacobian'
 *
 */

#ifndef GETJACOBIAN_H
#define GETJACOBIAN_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "GetJacobian_types.h"

/* Function Declarations */
extern void GetJacobian(const double bTei[96], const double bTe[16], const
  double jointType[6], double J[36]);

#endif

/* End of code generation (GetJacobian.h) */
