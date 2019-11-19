/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * getT0forall.cpp
 *
 * Code generation for function 'getT0forall'
 *
 */

/* Include files */
//#include "../Headers/getT0forall.h"

/* Function Definitions */
void getT0forall(const double Tij[96], const double [6], double T0forall[96])
{
  int i;
  int i0;
  int T0forall_tmp;
  int b_i;
  int i1;
  int b_T0forall_tmp;
  double b_T0forall[16];
  int i2;

  /*    Input:  */
  /*    -   Transformation matrices j with respect to i */
  /*    -   jointType vector */
  /* T-matrices wRt= for each joint */
  for (i = 0; i < 6; i++) {
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
    for (i0 = 0; i0 < 4; i0++) {
      T0forall_tmp = (i0 << 2) + (i << 4);
      T0forall[T0forall_tmp] = 0.0;
      T0forall[T0forall_tmp + 1] = 0.0;
      T0forall[T0forall_tmp + 2] = 0.0;
      T0forall[T0forall_tmp + 3] = 0.0;
    }

    T0forall[i << 4] = 1.0;
    T0forall[5 + (i << 4)] = 1.0;
    T0forall[10 + (i << 4)] = 1.0;
    T0forall[15 + (i << 4)] = 1.0;
    for (b_i = 0; b_i <= i; b_i++) {
      for (i0 = 0; i0 < 4; i0++) {
        for (i1 = 0; i1 < 4; i1++) {
          T0forall_tmp = i1 << 2;
          b_T0forall_tmp = i0 + T0forall_tmp;
          b_T0forall[b_T0forall_tmp] = 0.0;
          i2 = i0 + (i << 4);
          T0forall_tmp += b_i << 4;
          b_T0forall[b_T0forall_tmp] = ((T0forall[i2] * Tij[T0forall_tmp] + T0forall[i2 + 4] * Tij[T0forall_tmp + 1]) +
                  T0forall[i2 + 8] * Tij[T0forall_tmp + 2]) + T0forall[i2 + 12] * Tij[T0forall_tmp + 3];
        }
      }

      for (i0 = 0; i0 < 4; i0++) {
        T0forall_tmp = i0 << 2;
        b_T0forall_tmp = T0forall_tmp + (i << 4);
        T0forall[b_T0forall_tmp] = b_T0forall[T0forall_tmp];
        T0forall[b_T0forall_tmp + 1] = b_T0forall[1 + T0forall_tmp];
        T0forall[b_T0forall_tmp + 2] = b_T0forall[2 + T0forall_tmp];
        T0forall[b_T0forall_tmp + 3] = b_T0forall[3 + T0forall_tmp];
      }
    }
  }

  /*  T0all=NaN(4,4,length(jointType)); %T-matrices wRt0 for each joint */
  /*  for i=1:length(jointType) */
  /*      X=GetTransformationWrtBase(Tstart, i); */
  /*      T0all(:,:,i)=X; */
  /*  end */
  /*  T0all */
}

/* End of code generation (getT0forall.cpp) */
