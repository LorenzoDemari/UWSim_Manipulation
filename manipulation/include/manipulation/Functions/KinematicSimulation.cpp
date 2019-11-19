/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * KinematicSimulation.cpp
 *
 * Code generation for function 'KinematicSimulation'
 *
 */

/* Include files */
//#include "../Headers/KinematicSimulation.h"

/* Function Definitions */
void KinematicSimulation(double q[6], const double q_dot[6], double ts, const
  double q_min[6], const double q_max[6])
{
  int i;

  /*  Kinematic Simulation function */
  /*  */
  /*  Inputs */
  /*  - q current robot configuration */
  /*  - q_dot joints velocity */
  /*  - ts sample time */
  /*  - q_min lower joints bound */
  /*  - q_max upper joints bound */
  /*  */
  /*  Outputs */
  /*  - q new joint configuration */
  /*  updating q */
  /*  saturating */
  for (i = 0; i < 6; i++) {
    q[i] += q_dot[i] * ts;

    /*  upper bound */
    if (q[i] > q_max[i]) {
      q[i] = q_max[i];

      /*  lower bound */
    } else {
      if (q[i] < q_min[i]) {
        q[i] = q_min[i];
      }
    }
  }
}

/* End of code generation (KinematicSimulation.cpp) */
