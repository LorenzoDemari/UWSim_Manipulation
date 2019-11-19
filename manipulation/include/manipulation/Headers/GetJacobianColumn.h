/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * GetJacobianColumn.h
 *
 * Code generation for function 'GetJacobianColumn'
 *
 */

#ifndef GETJACOBIANCOLUMN_H
#define GETJACOBIANCOLUMN_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "GetJacobianColumn_types.h"

/* Function Declarations */
extern void GetJacobianColumn(const double bTei[16], const double bTe[16],
  double jointType, double h[6]);

#endif

/* End of code generation (GetJacobianColumn.h) */
