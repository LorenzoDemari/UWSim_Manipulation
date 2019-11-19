/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * KinematicSimulation.h
 *
 * Code generation for function 'KinematicSimulation'
 *
 */

#ifndef KINEMATICSIMULATION_H
#define KINEMATICSIMULATION_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "KinematicSimulation_types.h"

/* Function Declarations */
extern void KinematicSimulation(double q[6], const double q_dot[6], double ts,
  const double q_min[6], const double q_max[6]);

#endif

/* End of code generation (KinematicSimulation.h) */
