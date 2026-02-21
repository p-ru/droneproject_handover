/*
 * trajGen_with_yaw.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "trajGen_with_yaw".
 *
 * Model version              : 1.30
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C++ source code generated on : Wed Nov 26 20:40:17 2025
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "trajGen_with_yaw.h"
#include "rtwtypes.h"
#include "trajGen_with_yaw_types.h"
#include "coder_array.h"
#include "trajGen_with_yaw_private.h"
#include <cmath>
#include <cstring>
#include "cmath"
#include "rt_defines.h"

/* Block signals (default storage) */
B_trajGen_with_yaw_T trajGen_with_yaw_B;

/* Block states (default storage) */
DW_trajGen_with_yaw_T trajGen_with_yaw_DW;

/* Real-time model */
RT_MODEL_trajGen_with_yaw_T trajGen_with_yaw_M_{ };

RT_MODEL_trajGen_with_yaw_T *const trajGen_with_yaw_M{ &trajGen_with_yaw_M_ };

/* Forward declaration for local functions */
static void trajG_MinJerkPolyTraj_setupImpl(shared_uav_rst_sluav_internal_T *obj,
  const real_T waypoints[8], const real_T timePoints[2]);
static boolean_T trajGen_with_yaw_isequaln(const real_T varargin_1[8], const
  real_T varargin_2[8]);
static real_T trajGen_with_yaw_rt_powd_snf(real_T u0, real_T u1);
static void trajGen_with_yaw_eml_find(const boolean_T x[8], int32_T i_data[],
  int32_T i_size[1]);
static void trajGen_with_yaw_constructM(const real_T constraints[8], real_T M[64]);
static void trajGen_with_yaw_inv(const real_T x[16], real_T y[16]);
static void trajGen_with_yaw_mtimes(const coder::array<real_T, 2U> &A, const
  coder::array<real_T, 1U> &B, coder::array<real_T, 1U> &C);
static real_T trajGen_with_yaw_xnrm2(int32_T n, const coder::array<real_T, 2U>
  &x, int32_T ix0);
static real_T trajGen_with_yaw_rt_hypotd_snf(real_T u0, real_T u1);
static void trajGen_with_yaw_xgeqp3(const coder::array<real_T, 2U> &A, coder::
  array<real_T, 2U> &b_A, coder::array<real_T, 1U> &tau, coder::array<int32_T,
  2U> &jpvt);
static int32_T trajGen_with_yaw_rankFromQR(const coder::array<real_T, 2U> &A);
static void trajGen_with_yaw_xzgetrf(int32_T m, int32_T n, coder::array<real_T,
  2U> &A, int32_T lda, coder::array<int32_T, 2U> &ipiv, int32_T *info);
static void trajGen_with_yaw_mldivide(const coder::array<real_T, 2U> &A, const
  coder::array<real_T, 1U> &B, coder::array<real_T, 1U> &Y);
static void trajGen_with_yaw_xzgetrf_g(real_T A[64], int32_T ipiv[8], int32_T
  *info);
static void trajGen_with_yaw_mldivide_gw(const real_T A[64], real_T B[8]);
static real_T trajGen_with_yaw_rcond(const real_T A[64]);
static void MinJerkPolyTraj_computePPandTim(shared_uav_rst_sluav_internal_T *obj,
  const real_T waypoints[8], const real_T timePoints[2], real_T ppMatrix[32],
  coder::array<real_T, 2U> &toaZero, real_T *maxIterStatus, real_T
  *maxTimeStatus, real_T *singularityStatus);
static void trajGen_with_y_binary_expand_op(real_T in1[2], const
  shared_uav_rst_sluav_internal_T *in2);
real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaN);
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

/* Start for atomic system: */
void CoordinateTransformationC_Start(DW_CoordinateTransformationCo_T *localDW)
{
  /* Start for MATLABSystem: '<S5>/Coordinate Transformation Conversion' */
  localDW->objisempty = true;
  localDW->obj.isInitialized = 1;
}

/* Output and update for atomic system: */
void CoordinateTransformationConvers(const real_T rtu_0[4],
  B_CoordinateTransformationCon_T *localB)
{
  real_T b;
  real_T d_tmp;
  real_T f_idx_0;
  real_T y_idx_0;
  real_T y_idx_1;
  real_T y_idx_2;

  /* MATLABSystem: '<S5>/Coordinate Transformation Conversion' */
  b = 1.0 / std::sqrt(((rtu_0[0] * rtu_0[0] + rtu_0[1] * rtu_0[1]) + rtu_0[2] *
                       rtu_0[2]) + rtu_0[3] * rtu_0[3]);
  y_idx_0 = rtu_0[0] * b;
  y_idx_1 = rtu_0[1] * b;
  y_idx_2 = rtu_0[2] * b;
  b *= rtu_0[3];
  d_tmp = y_idx_1 * b * 2.0 - y_idx_0 * y_idx_2 * 2.0;
  if (d_tmp > 1.0) {
    d_tmp = 1.0;
  }

  f_idx_0 = d_tmp;
  if (d_tmp < -1.0) {
    f_idx_0 = -1.0;
  }

  if ((f_idx_0 < 0.0) && (std::abs(f_idx_0 + 1.0) < 2.2204460492503131E-15)) {
    /* MATLABSystem: '<S5>/Coordinate Transformation Conversion' */
    localB->CoordinateTransformationConve_p[0] = -2.0 * rt_atan2d_snf(y_idx_1,
      y_idx_0);
    localB->CoordinateTransformationConve_p[2] = 0.0;
    localB->CoordinateTransformationConve_p[1] = 1.5707963267948966;
  } else if ((f_idx_0 > 0.0) && (std::abs(f_idx_0 - 1.0) <
              2.2204460492503131E-15)) {
    /* MATLABSystem: '<S5>/Coordinate Transformation Conversion' */
    localB->CoordinateTransformationConve_p[0] = 2.0 * rt_atan2d_snf(y_idx_1,
      y_idx_0);
    localB->CoordinateTransformationConve_p[2] = 0.0;
    localB->CoordinateTransformationConve_p[1] = -1.5707963267948966;
  } else {
    d_tmp = y_idx_0 * y_idx_0 * 2.0 - 1.0;

    /* MATLABSystem: '<S5>/Coordinate Transformation Conversion' */
    localB->CoordinateTransformationConve_p[0] = rt_atan2d_snf(y_idx_0 * b * 2.0
      + y_idx_1 * y_idx_2 * 2.0, y_idx_1 * y_idx_1 * 2.0 + d_tmp);
    localB->CoordinateTransformationConve_p[2] = rt_atan2d_snf(y_idx_0 * y_idx_1
      * 2.0 + y_idx_2 * b * 2.0, b * b * 2.0 + d_tmp);
    localB->CoordinateTransformationConve_p[1] = -std::asin(f_idx_0);
  }

  /* End of MATLABSystem: '<S5>/Coordinate Transformation Conversion' */
}

static void trajG_MinJerkPolyTraj_setupImpl(shared_uav_rst_sluav_internal_T *obj,
  const real_T waypoints[8], const real_T timePoints[2])
{
  obj->PPMatrix.set_size(4, 8, 1);
  for (int32_T i{0}; i < 32; i++) {
    obj->PPMatrix[i] = 0.0;
  }

  obj->TimeOfArrZero.set_size(1, 2);
  obj->TimeOfArrZero[0] = 0.0;
  obj->TimeOfArrZero[1] = 0.0;
  obj->TimeOffset = 0.0;
  obj->MaxIterStatus = 0.0;
  obj->MaxTimeStatus = 0.0;
  obj->SingularityStatus = 0.0;
  obj->PrevTimePoint[0] = timePoints[0];
  obj->PrevTimePoint[1] = timePoints[1];
  for (int32_T i{0}; i < 8; i++) {
    obj->PrevWaypoint[i] = waypoints[i];

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    obj->PrevVelBC[i] = 0.0;
    obj->PrevAccelBC[i] = 0.0;
    obj->PrevJerkBC[i] = 0.0;
  }
}

static boolean_T trajGen_with_yaw_isequaln(const real_T varargin_1[8], const
  real_T varargin_2[8])
{
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  trajGen_with_yaw_B.b_k_o = 0;
  exitg1 = false;
  while ((!exitg1) && (trajGen_with_yaw_B.b_k_o < 8)) {
    if ((varargin_1[trajGen_with_yaw_B.b_k_o] ==
         varargin_2[trajGen_with_yaw_B.b_k_o]) || (std::isnan
         (varargin_1[trajGen_with_yaw_B.b_k_o]) && std::isnan
         (varargin_2[trajGen_with_yaw_B.b_k_o]))) {
      trajGen_with_yaw_B.b_k_o++;
    } else {
      p_0 = false;
      exitg1 = true;
    }
  }

  if (p_0) {
    p = true;
  }

  return p;
}

static real_T trajGen_with_yaw_rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaN);
  } else {
    trajGen_with_yaw_B.d = std::abs(u0);
    trajGen_with_yaw_B.d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (trajGen_with_yaw_B.d == 1.0) {
        y = 1.0;
      } else if (trajGen_with_yaw_B.d > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (trajGen_with_yaw_B.d1 == 0.0) {
      y = 1.0;
    } else if (trajGen_with_yaw_B.d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

static void trajGen_with_yaw_eml_find(const boolean_T x[8], int32_T i_data[],
  int32_T i_size[1])
{
  boolean_T exitg1;
  trajGen_with_yaw_B.idx = 0;
  trajGen_with_yaw_B.b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (trajGen_with_yaw_B.b_ii - 1 < 8)) {
    if (x[trajGen_with_yaw_B.b_ii - 1]) {
      trajGen_with_yaw_B.idx++;
      i_data[trajGen_with_yaw_B.idx - 1] = trajGen_with_yaw_B.b_ii;
      if (trajGen_with_yaw_B.idx >= 8) {
        exitg1 = true;
      } else {
        trajGen_with_yaw_B.b_ii++;
      }
    } else {
      trajGen_with_yaw_B.b_ii++;
    }
  }

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  if (trajGen_with_yaw_B.idx < 1) {
    i_size[0] = 0;
  } else {
    i_size[0] = trajGen_with_yaw_B.idx;
  }

  /* End of Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
}

static void trajGen_with_yaw_constructM(const real_T constraints[8], real_T M[64])
{
  boolean_T fixedBCIdx_tmp;
  std::memset(&trajGen_with_yaw_B.M1[0], 0, sizeof(int8_T) << 6U);

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  for (trajGen_with_yaw_B.i_p = 0; trajGen_with_yaw_B.i_p < 8;
       trajGen_with_yaw_B.i_p++) {
    fixedBCIdx_tmp = std::isnan(constraints[trajGen_with_yaw_B.i_p]);
    trajGen_with_yaw_B.fixedBCIdx_tmp[trajGen_with_yaw_B.i_p] = fixedBCIdx_tmp;
    trajGen_with_yaw_B.fixedBCIdx_tmp_j[trajGen_with_yaw_B.i_p] =
      !fixedBCIdx_tmp;
  }

  trajGen_with_yaw_eml_find(trajGen_with_yaw_B.fixedBCIdx_tmp_j,
    trajGen_with_yaw_B.tmp_data, trajGen_with_yaw_B.tmp_size);
  trajGen_with_yaw_B.loop_ub_p = trajGen_with_yaw_B.tmp_size[0];
  trajGen_with_yaw_B.fixedBCIdx_size_idx_0 = trajGen_with_yaw_B.tmp_size[0];
  if (trajGen_with_yaw_B.loop_ub_p - 1 >= 0) {
    std::memcpy(&trajGen_with_yaw_B.fixedBCIdx_data[0],
                &trajGen_with_yaw_B.tmp_data[0], static_cast<uint32_T>
                (trajGen_with_yaw_B.loop_ub_p) * sizeof(int32_T));
  }

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  trajGen_with_yaw_eml_find(trajGen_with_yaw_B.fixedBCIdx_tmp,
    trajGen_with_yaw_B.tmp_data, trajGen_with_yaw_B.tmp_size);
  trajGen_with_yaw_B.loop_ub_a = trajGen_with_yaw_B.tmp_size[0];
  if (trajGen_with_yaw_B.loop_ub_a - 1 >= 0) {
    std::memcpy(&trajGen_with_yaw_B.freeBCIdx_data[0],
                &trajGen_with_yaw_B.tmp_data[0], static_cast<uint32_T>
                (trajGen_with_yaw_B.loop_ub_a) * sizeof(int32_T));
  }

  for (trajGen_with_yaw_B.i_p = 0; trajGen_with_yaw_B.i_p <
       trajGen_with_yaw_B.loop_ub_p; trajGen_with_yaw_B.i_p++) {
    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.M1
      [(trajGen_with_yaw_B.fixedBCIdx_data[trajGen_with_yaw_B.i_p] +
        (trajGen_with_yaw_B.i_p << 3)) - 1] = 1;
  }

  trajGen_with_yaw_B.i_p = 7 - trajGen_with_yaw_B.fixedBCIdx_size_idx_0;
  for (trajGen_with_yaw_B.loop_ub_p = 0; trajGen_with_yaw_B.loop_ub_p <=
       trajGen_with_yaw_B.i_p; trajGen_with_yaw_B.loop_ub_p++) {
    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.M1
      [(trajGen_with_yaw_B.freeBCIdx_data[trajGen_with_yaw_B.loop_ub_p] +
        ((trajGen_with_yaw_B.fixedBCIdx_size_idx_0 +
          trajGen_with_yaw_B.loop_ub_p) << 3)) - 1] = 1;
  }

  std::memset(&trajGen_with_yaw_B.Mcontinuity[0], 0, sizeof(int8_T) << 6U);
  for (trajGen_with_yaw_B.i_p = 0; trajGen_with_yaw_B.i_p < 8;
       trajGen_with_yaw_B.i_p++) {
    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.Mcontinuity[trajGen_with_yaw_B.i_p +
      (trajGen_with_yaw_B.i_p << 3)] = 1;
  }

  for (trajGen_with_yaw_B.i_p = 0; trajGen_with_yaw_B.i_p < 8;
       trajGen_with_yaw_B.i_p++) {
    for (trajGen_with_yaw_B.loop_ub_p = 0; trajGen_with_yaw_B.loop_ub_p < 8;
         trajGen_with_yaw_B.loop_ub_p++) {
      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.M_b = 0.0;
      for (trajGen_with_yaw_B.loop_ub_a = 0; trajGen_with_yaw_B.loop_ub_a < 8;
           trajGen_with_yaw_B.loop_ub_a++) {
        /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
        trajGen_with_yaw_B.M_b += static_cast<real_T>
          (trajGen_with_yaw_B.Mcontinuity[(trajGen_with_yaw_B.loop_ub_a << 3) +
           trajGen_with_yaw_B.loop_ub_p] * trajGen_with_yaw_B.M1
           [(trajGen_with_yaw_B.i_p << 3) + trajGen_with_yaw_B.loop_ub_a]);
      }

      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      M[trajGen_with_yaw_B.i_p + (trajGen_with_yaw_B.loop_ub_p << 3)] =
        trajGen_with_yaw_B.M_b;
    }
  }
}

static void trajGen_with_yaw_inv(const real_T x[16], real_T y[16])
{
  for (trajGen_with_yaw_B.c_a = 0; trajGen_with_yaw_B.c_a < 16;
       trajGen_with_yaw_B.c_a++) {
    y[trajGen_with_yaw_B.c_a] = 0.0;

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.c_a] = x[trajGen_with_yaw_B.c_a];
  }

  trajGen_with_yaw_B.b_ipiv_e[0] = 1;
  trajGen_with_yaw_B.b_ipiv_e[1] = 2;
  trajGen_with_yaw_B.b_ipiv_e[2] = 3;
  for (trajGen_with_yaw_B.pipk = 0; trajGen_with_yaw_B.pipk < 3;
       trajGen_with_yaw_B.pipk++) {
    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.c_a = trajGen_with_yaw_B.pipk * 5 + 2;
    trajGen_with_yaw_B.jj_a = trajGen_with_yaw_B.pipk * 5;
    trajGen_with_yaw_B.jBcol = 4 - trajGen_with_yaw_B.pipk;
    trajGen_with_yaw_B.kAcol_e = 1;

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.smax_ln = std::abs
      (trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.jj_a]);
    for (trajGen_with_yaw_B.k_i = 2; trajGen_with_yaw_B.k_i <=
         trajGen_with_yaw_B.jBcol; trajGen_with_yaw_B.k_i++) {
      trajGen_with_yaw_B.s_h = std::abs(trajGen_with_yaw_B.c_A_g
        [(trajGen_with_yaw_B.c_a + trajGen_with_yaw_B.k_i) - 3]);
      if (trajGen_with_yaw_B.s_h > trajGen_with_yaw_B.smax_ln) {
        trajGen_with_yaw_B.kAcol_e = trajGen_with_yaw_B.k_i;
        trajGen_with_yaw_B.smax_ln = trajGen_with_yaw_B.s_h;
      }
    }

    if (trajGen_with_yaw_B.c_A_g[(trajGen_with_yaw_B.c_a +
         trajGen_with_yaw_B.kAcol_e) - 3] != 0.0) {
      if (trajGen_with_yaw_B.kAcol_e - 1 != 0) {
        trajGen_with_yaw_B.k_i = trajGen_with_yaw_B.pipk +
          trajGen_with_yaw_B.kAcol_e;
        trajGen_with_yaw_B.b_ipiv_e[trajGen_with_yaw_B.pipk] =
          static_cast<int8_T>(trajGen_with_yaw_B.k_i);
        trajGen_with_yaw_B.smax_ln =
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.pipk];
        trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.pipk] =
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.k_i - 1];
        trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.k_i - 1] =
          trajGen_with_yaw_B.smax_ln;
        trajGen_with_yaw_B.smax_ln =
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.pipk + 4];
        trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.pipk + 4] =
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.k_i + 3];
        trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.k_i + 3] =
          trajGen_with_yaw_B.smax_ln;
        trajGen_with_yaw_B.smax_ln =
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.pipk + 8];
        trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.pipk + 8] =
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.k_i + 7];
        trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.k_i + 7] =
          trajGen_with_yaw_B.smax_ln;
        trajGen_with_yaw_B.smax_ln =
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.pipk + 12];
        trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.pipk + 12] =
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.k_i + 11];
        trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.k_i + 11] =
          trajGen_with_yaw_B.smax_ln;
      }

      trajGen_with_yaw_B.kAcol_e = trajGen_with_yaw_B.c_a -
        trajGen_with_yaw_B.pipk;
      for (trajGen_with_yaw_B.jBcol = trajGen_with_yaw_B.c_a;
           trajGen_with_yaw_B.jBcol <= trajGen_with_yaw_B.kAcol_e + 2;
           trajGen_with_yaw_B.jBcol++) {
        trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.jBcol - 1] /=
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.jj_a];
      }
    }

    trajGen_with_yaw_B.jA_l = trajGen_with_yaw_B.jj_a;
    trajGen_with_yaw_B.kAcol_e = 3 - trajGen_with_yaw_B.pipk;
    for (trajGen_with_yaw_B.k_i = 0; trajGen_with_yaw_B.k_i <
         trajGen_with_yaw_B.kAcol_e; trajGen_with_yaw_B.k_i++) {
      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.smax_ln = trajGen_with_yaw_B.c_A_g
        [((trajGen_with_yaw_B.k_i << 2) + trajGen_with_yaw_B.jj_a) + 4];
      if (trajGen_with_yaw_B.smax_ln != 0.0) {
        /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
        trajGen_with_yaw_B.jBcol = trajGen_with_yaw_B.jA_l + 6;
        trajGen_with_yaw_B.d_o = (trajGen_with_yaw_B.jA_l -
          trajGen_with_yaw_B.pipk) + 8;
        for (trajGen_with_yaw_B.ijA = trajGen_with_yaw_B.jBcol;
             trajGen_with_yaw_B.ijA <= trajGen_with_yaw_B.d_o;
             trajGen_with_yaw_B.ijA++) {
          /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.ijA - 1] +=
            trajGen_with_yaw_B.c_A_g[((trajGen_with_yaw_B.c_a +
            trajGen_with_yaw_B.ijA) - trajGen_with_yaw_B.jA_l) - 7] *
            -trajGen_with_yaw_B.smax_ln;
        }
      }

      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.jA_l += 4;
    }
  }

  trajGen_with_yaw_B.p[0] = 1;
  trajGen_with_yaw_B.p[1] = 2;
  trajGen_with_yaw_B.p[2] = 3;
  trajGen_with_yaw_B.p[3] = 4;

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  if (trajGen_with_yaw_B.b_ipiv_e[0] > 1) {
    trajGen_with_yaw_B.pipk = trajGen_with_yaw_B.p[trajGen_with_yaw_B.b_ipiv_e[0]
      - 1];
    trajGen_with_yaw_B.p[trajGen_with_yaw_B.b_ipiv_e[0] - 1] = 1;
    trajGen_with_yaw_B.p[0] = static_cast<int8_T>(trajGen_with_yaw_B.pipk);
  }

  if (trajGen_with_yaw_B.b_ipiv_e[1] > 2) {
    trajGen_with_yaw_B.pipk = trajGen_with_yaw_B.p[trajGen_with_yaw_B.b_ipiv_e[1]
      - 1];
    trajGen_with_yaw_B.p[trajGen_with_yaw_B.b_ipiv_e[1] - 1] =
      trajGen_with_yaw_B.p[1];
    trajGen_with_yaw_B.p[1] = static_cast<int8_T>(trajGen_with_yaw_B.pipk);
  }

  if (trajGen_with_yaw_B.b_ipiv_e[2] > 3) {
    trajGen_with_yaw_B.pipk = trajGen_with_yaw_B.p[trajGen_with_yaw_B.b_ipiv_e[2]
      - 1];
    trajGen_with_yaw_B.p[trajGen_with_yaw_B.b_ipiv_e[2] - 1] =
      trajGen_with_yaw_B.p[2];
    trajGen_with_yaw_B.p[2] = static_cast<int8_T>(trajGen_with_yaw_B.pipk);
  }

  for (trajGen_with_yaw_B.pipk = 0; trajGen_with_yaw_B.pipk < 4;
       trajGen_with_yaw_B.pipk++) {
    trajGen_with_yaw_B.jj_a = (trajGen_with_yaw_B.p[trajGen_with_yaw_B.pipk] - 1)
      << 2;
    y[trajGen_with_yaw_B.pipk + trajGen_with_yaw_B.jj_a] = 1.0;

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    for (trajGen_with_yaw_B.k_i = trajGen_with_yaw_B.pipk + 1;
         trajGen_with_yaw_B.k_i < 5; trajGen_with_yaw_B.k_i++) {
      trajGen_with_yaw_B.c_a = (trajGen_with_yaw_B.jj_a + trajGen_with_yaw_B.k_i)
        - 1;
      if (y[trajGen_with_yaw_B.c_a] != 0.0) {
        for (trajGen_with_yaw_B.jBcol = trajGen_with_yaw_B.k_i + 1;
             trajGen_with_yaw_B.jBcol < 5; trajGen_with_yaw_B.jBcol++) {
          trajGen_with_yaw_B.kAcol_e = (trajGen_with_yaw_B.jj_a +
            trajGen_with_yaw_B.jBcol) - 1;
          y[trajGen_with_yaw_B.kAcol_e] -= trajGen_with_yaw_B.c_A_g
            [(((trajGen_with_yaw_B.k_i - 1) << 2) + trajGen_with_yaw_B.jBcol) -
            1] * y[trajGen_with_yaw_B.c_a];
        }
      }
    }
  }

  for (trajGen_with_yaw_B.pipk = 0; trajGen_with_yaw_B.pipk < 4;
       trajGen_with_yaw_B.pipk++) {
    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.jBcol = trajGen_with_yaw_B.pipk << 2;
    for (trajGen_with_yaw_B.k_i = 3; trajGen_with_yaw_B.k_i >= 0;
         trajGen_with_yaw_B.k_i--) {
      trajGen_with_yaw_B.kAcol_e = trajGen_with_yaw_B.k_i << 2;
      trajGen_with_yaw_B.c_a = trajGen_with_yaw_B.k_i + trajGen_with_yaw_B.jBcol;
      trajGen_with_yaw_B.smax_ln = y[trajGen_with_yaw_B.c_a];
      if (trajGen_with_yaw_B.smax_ln != 0.0) {
        y[trajGen_with_yaw_B.c_a] = trajGen_with_yaw_B.smax_ln /
          trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.k_i +
          trajGen_with_yaw_B.kAcol_e];
        for (trajGen_with_yaw_B.jA_l = 0; trajGen_with_yaw_B.jA_l <
             trajGen_with_yaw_B.k_i; trajGen_with_yaw_B.jA_l++) {
          trajGen_with_yaw_B.jj_a = trajGen_with_yaw_B.jA_l +
            trajGen_with_yaw_B.jBcol;
          y[trajGen_with_yaw_B.jj_a] -=
            trajGen_with_yaw_B.c_A_g[trajGen_with_yaw_B.jA_l +
            trajGen_with_yaw_B.kAcol_e] * y[trajGen_with_yaw_B.c_a];
        }
      }
    }
  }
}

static void trajGen_with_yaw_mtimes(const coder::array<real_T, 2U> &A, const
  coder::array<real_T, 1U> &B, coder::array<real_T, 1U> &C)
{
  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  trajGen_with_yaw_B.m_tmp = A.size(1);
  trajGen_with_yaw_B.inner = A.size(0);
  C.set_size(A.size(1));
  for (trajGen_with_yaw_B.b_i = 0; trajGen_with_yaw_B.b_i <
       trajGen_with_yaw_B.m_tmp; trajGen_with_yaw_B.b_i++) {
    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    C[trajGen_with_yaw_B.b_i] = 0.0;
  }

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  for (trajGen_with_yaw_B.b_i = 0; trajGen_with_yaw_B.b_i <
       trajGen_with_yaw_B.inner; trajGen_with_yaw_B.b_i++) {
    for (trajGen_with_yaw_B.c_i = 0; trajGen_with_yaw_B.c_i <
         trajGen_with_yaw_B.m_tmp; trajGen_with_yaw_B.c_i++) {
      C[trajGen_with_yaw_B.c_i] = A[trajGen_with_yaw_B.c_i * A.size(0) +
        trajGen_with_yaw_B.b_i] * B[trajGen_with_yaw_B.b_i] +
        C[trajGen_with_yaw_B.c_i];
    }
  }
}

static real_T trajGen_with_yaw_xnrm2(int32_T n, const coder::array<real_T, 2U>
  &x, int32_T ix0)
{
  real_T y;
  y = 0.0;

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      trajGen_with_yaw_B.scale_o = 3.3121686421112381E-170;
      trajGen_with_yaw_B.kend_c = ix0 + n;
      for (trajGen_with_yaw_B.k_ct = ix0; trajGen_with_yaw_B.k_ct <
           trajGen_with_yaw_B.kend_c; trajGen_with_yaw_B.k_ct++) {
        trajGen_with_yaw_B.absxk_b = std::abs(x[trajGen_with_yaw_B.k_ct - 1]);
        if (trajGen_with_yaw_B.absxk_b > trajGen_with_yaw_B.scale_o) {
          trajGen_with_yaw_B.t_n = trajGen_with_yaw_B.scale_o /
            trajGen_with_yaw_B.absxk_b;
          y = y * trajGen_with_yaw_B.t_n * trajGen_with_yaw_B.t_n + 1.0;
          trajGen_with_yaw_B.scale_o = trajGen_with_yaw_B.absxk_b;
        } else {
          trajGen_with_yaw_B.t_n = trajGen_with_yaw_B.absxk_b /
            trajGen_with_yaw_B.scale_o;
          y += trajGen_with_yaw_B.t_n * trajGen_with_yaw_B.t_n;
        }
      }

      y = trajGen_with_yaw_B.scale_o * std::sqrt(y);
    }
  }

  /* End of Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  return y;
}

static real_T trajGen_with_yaw_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  trajGen_with_yaw_B.a = std::abs(u0);
  trajGen_with_yaw_B.b = std::abs(u1);
  if (trajGen_with_yaw_B.a < trajGen_with_yaw_B.b) {
    trajGen_with_yaw_B.a /= trajGen_with_yaw_B.b;
    y = std::sqrt(trajGen_with_yaw_B.a * trajGen_with_yaw_B.a + 1.0) *
      trajGen_with_yaw_B.b;
  } else if (trajGen_with_yaw_B.a > trajGen_with_yaw_B.b) {
    trajGen_with_yaw_B.b /= trajGen_with_yaw_B.a;
    y = std::sqrt(trajGen_with_yaw_B.b * trajGen_with_yaw_B.b + 1.0) *
      trajGen_with_yaw_B.a;
  } else if (std::isnan(trajGen_with_yaw_B.b)) {
    y = (rtNaN);
  } else {
    y = trajGen_with_yaw_B.a * 1.4142135623730951;
  }

  return y;
}

static void trajGen_with_yaw_xgeqp3(const coder::array<real_T, 2U> &A, coder::
  array<real_T, 2U> &b_A, coder::array<real_T, 1U> &tau, coder::array<int32_T,
  2U> &jpvt)
{
  int32_T exitg1;
  boolean_T exitg2;

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  trajGen_with_yaw_B.ma_tmp = A.size(0);
  trajGen_with_yaw_B.na_tmp = A.size(1);
  b_A.set_size(A.size(0), A.size(1));
  trajGen_with_yaw_B.loop_ub_tmp = A.size(0) * A.size(1);
  for (trajGen_with_yaw_B.b_tmp = 0; trajGen_with_yaw_B.b_tmp <
       trajGen_with_yaw_B.loop_ub_tmp; trajGen_with_yaw_B.b_tmp++) {
    b_A[trajGen_with_yaw_B.b_tmp] = A[trajGen_with_yaw_B.b_tmp];
  }

  if (A.size(0) <= A.size(1)) {
    trajGen_with_yaw_B.minmana_tmp = A.size(0);
  } else {
    trajGen_with_yaw_B.minmana_tmp = A.size(1);
  }

  tau.set_size(trajGen_with_yaw_B.minmana_tmp);
  for (trajGen_with_yaw_B.b_tmp = 0; trajGen_with_yaw_B.b_tmp <
       trajGen_with_yaw_B.minmana_tmp; trajGen_with_yaw_B.b_tmp++) {
    tau[trajGen_with_yaw_B.b_tmp] = 0.0;
  }

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  if ((A.size(0) == 0) || (A.size(1) == 0) || (trajGen_with_yaw_B.minmana_tmp <
       1)) {
    jpvt.set_size(1, A.size(1));
    for (trajGen_with_yaw_B.loop_ub_tmp = 0; trajGen_with_yaw_B.loop_ub_tmp <
         trajGen_with_yaw_B.na_tmp; trajGen_with_yaw_B.loop_ub_tmp++) {
      jpvt[trajGen_with_yaw_B.loop_ub_tmp] = trajGen_with_yaw_B.loop_ub_tmp + 1;
    }
  } else {
    jpvt.set_size(1, A.size(1));
    for (trajGen_with_yaw_B.loop_ub_tmp = 0; trajGen_with_yaw_B.loop_ub_tmp <
         trajGen_with_yaw_B.na_tmp; trajGen_with_yaw_B.loop_ub_tmp++) {
      jpvt[trajGen_with_yaw_B.loop_ub_tmp] = trajGen_with_yaw_B.loop_ub_tmp + 1;
    }

    tau.set_size(trajGen_with_yaw_B.minmana_tmp);
    for (trajGen_with_yaw_B.b_tmp = 0; trajGen_with_yaw_B.b_tmp <
         trajGen_with_yaw_B.minmana_tmp; trajGen_with_yaw_B.b_tmp++) {
      tau[trajGen_with_yaw_B.b_tmp] = 0.0;
    }

    trajGen_with_yaw_B.work.set_size(A.size(1));
    trajGen_with_yaw_B.vn1.set_size(A.size(1));
    trajGen_with_yaw_B.vn2.set_size(A.size(1));
    for (trajGen_with_yaw_B.loop_ub_tmp = 0; trajGen_with_yaw_B.loop_ub_tmp <
         trajGen_with_yaw_B.na_tmp; trajGen_with_yaw_B.loop_ub_tmp++) {
      trajGen_with_yaw_B.work[trajGen_with_yaw_B.loop_ub_tmp] = 0.0;
      trajGen_with_yaw_B.vn1[trajGen_with_yaw_B.loop_ub_tmp] = 0.0;
      trajGen_with_yaw_B.vn2[trajGen_with_yaw_B.loop_ub_tmp] = 0.0;
      trajGen_with_yaw_B.jA_j = trajGen_with_yaw_B.loop_ub_tmp *
        trajGen_with_yaw_B.ma_tmp;
      trajGen_with_yaw_B.smax_l = 0.0;
      if (trajGen_with_yaw_B.ma_tmp >= 1) {
        if (trajGen_with_yaw_B.ma_tmp == 1) {
          trajGen_with_yaw_B.smax_l = std::abs(A[trajGen_with_yaw_B.jA_j]);
        } else {
          trajGen_with_yaw_B.scale = 3.3121686421112381E-170;
          trajGen_with_yaw_B.kend = trajGen_with_yaw_B.jA_j +
            trajGen_with_yaw_B.ma_tmp;
          for (trajGen_with_yaw_B.itemp = trajGen_with_yaw_B.jA_j + 1;
               trajGen_with_yaw_B.itemp <= trajGen_with_yaw_B.kend;
               trajGen_with_yaw_B.itemp++) {
            trajGen_with_yaw_B.absxk = std::abs(A[trajGen_with_yaw_B.itemp - 1]);
            if (trajGen_with_yaw_B.absxk > trajGen_with_yaw_B.scale) {
              trajGen_with_yaw_B.t = trajGen_with_yaw_B.scale /
                trajGen_with_yaw_B.absxk;
              trajGen_with_yaw_B.smax_l = trajGen_with_yaw_B.smax_l *
                trajGen_with_yaw_B.t * trajGen_with_yaw_B.t + 1.0;
              trajGen_with_yaw_B.scale = trajGen_with_yaw_B.absxk;
            } else {
              trajGen_with_yaw_B.t = trajGen_with_yaw_B.absxk /
                trajGen_with_yaw_B.scale;
              trajGen_with_yaw_B.smax_l += trajGen_with_yaw_B.t *
                trajGen_with_yaw_B.t;
            }
          }

          trajGen_with_yaw_B.smax_l = trajGen_with_yaw_B.scale * std::sqrt
            (trajGen_with_yaw_B.smax_l);
        }
      }

      trajGen_with_yaw_B.vn1[trajGen_with_yaw_B.loop_ub_tmp] =
        trajGen_with_yaw_B.smax_l;
      trajGen_with_yaw_B.vn2[trajGen_with_yaw_B.loop_ub_tmp] =
        trajGen_with_yaw_B.smax_l;
    }

    for (trajGen_with_yaw_B.loop_ub_tmp = 0; trajGen_with_yaw_B.loop_ub_tmp <
         trajGen_with_yaw_B.minmana_tmp; trajGen_with_yaw_B.loop_ub_tmp++) {
      trajGen_with_yaw_B.jA_j = trajGen_with_yaw_B.loop_ub_tmp *
        trajGen_with_yaw_B.ma_tmp;
      trajGen_with_yaw_B.ii = trajGen_with_yaw_B.jA_j +
        trajGen_with_yaw_B.loop_ub_tmp;
      trajGen_with_yaw_B.nmi = trajGen_with_yaw_B.na_tmp -
        trajGen_with_yaw_B.loop_ub_tmp;
      trajGen_with_yaw_B.kend = (trajGen_with_yaw_B.ma_tmp -
        trajGen_with_yaw_B.loop_ub_tmp) - 1;
      if (trajGen_with_yaw_B.nmi < 1) {
        trajGen_with_yaw_B.pvt = 0;
      } else {
        trajGen_with_yaw_B.pvt = 1;
        if (trajGen_with_yaw_B.nmi > 1) {
          trajGen_with_yaw_B.smax_l = std::abs
            (trajGen_with_yaw_B.vn1[trajGen_with_yaw_B.loop_ub_tmp]);
          for (trajGen_with_yaw_B.itemp = 2; trajGen_with_yaw_B.itemp <=
               trajGen_with_yaw_B.nmi; trajGen_with_yaw_B.itemp++) {
            trajGen_with_yaw_B.scale = std::abs(trajGen_with_yaw_B.vn1
              [(trajGen_with_yaw_B.loop_ub_tmp + trajGen_with_yaw_B.itemp) - 1]);
            if (trajGen_with_yaw_B.scale > trajGen_with_yaw_B.smax_l) {
              trajGen_with_yaw_B.pvt = trajGen_with_yaw_B.itemp;
              trajGen_with_yaw_B.smax_l = trajGen_with_yaw_B.scale;
            }
          }
        }
      }

      trajGen_with_yaw_B.pvt = (trajGen_with_yaw_B.loop_ub_tmp +
        trajGen_with_yaw_B.pvt) - 1;
      if (trajGen_with_yaw_B.pvt + 1 != trajGen_with_yaw_B.loop_ub_tmp + 1) {
        trajGen_with_yaw_B.ix_m = trajGen_with_yaw_B.pvt *
          trajGen_with_yaw_B.ma_tmp;
        for (trajGen_with_yaw_B.itemp = 0; trajGen_with_yaw_B.itemp <
             trajGen_with_yaw_B.ma_tmp; trajGen_with_yaw_B.itemp++) {
          trajGen_with_yaw_B.temp_tmp_h = trajGen_with_yaw_B.ix_m +
            trajGen_with_yaw_B.itemp;
          trajGen_with_yaw_B.scale = b_A[trajGen_with_yaw_B.temp_tmp_h];
          trajGen_with_yaw_B.b_tmp = trajGen_with_yaw_B.jA_j +
            trajGen_with_yaw_B.itemp;
          b_A[trajGen_with_yaw_B.temp_tmp_h] = b_A[trajGen_with_yaw_B.b_tmp];
          b_A[trajGen_with_yaw_B.b_tmp] = trajGen_with_yaw_B.scale;
        }

        trajGen_with_yaw_B.itemp = jpvt[trajGen_with_yaw_B.pvt];
        jpvt[trajGen_with_yaw_B.pvt] = jpvt[trajGen_with_yaw_B.loop_ub_tmp];
        jpvt[trajGen_with_yaw_B.loop_ub_tmp] = trajGen_with_yaw_B.itemp;
        trajGen_with_yaw_B.vn1[trajGen_with_yaw_B.pvt] =
          trajGen_with_yaw_B.vn1[trajGen_with_yaw_B.loop_ub_tmp];
        trajGen_with_yaw_B.vn2[trajGen_with_yaw_B.pvt] =
          trajGen_with_yaw_B.vn2[trajGen_with_yaw_B.loop_ub_tmp];
      }

      if (trajGen_with_yaw_B.loop_ub_tmp + 1 < trajGen_with_yaw_B.ma_tmp) {
        trajGen_with_yaw_B.jA_j = trajGen_with_yaw_B.ii + 2;
        trajGen_with_yaw_B.smax_l = b_A[trajGen_with_yaw_B.ii];
        tau[trajGen_with_yaw_B.loop_ub_tmp] = 0.0;
        if (trajGen_with_yaw_B.kend + 1 > 0) {
          trajGen_with_yaw_B.scale = trajGen_with_yaw_xnrm2
            (trajGen_with_yaw_B.kend, b_A, trajGen_with_yaw_B.ii + 2);
          if (trajGen_with_yaw_B.scale != 0.0) {
            trajGen_with_yaw_B.scale = trajGen_with_yaw_rt_hypotd_snf
              (b_A[trajGen_with_yaw_B.ii], trajGen_with_yaw_B.scale);
            if (b_A[trajGen_with_yaw_B.ii] >= 0.0) {
              trajGen_with_yaw_B.scale = -trajGen_with_yaw_B.scale;
            }

            if (std::abs(trajGen_with_yaw_B.scale) < 1.0020841800044864E-292) {
              trajGen_with_yaw_B.pvt = -1;
              do {
                trajGen_with_yaw_B.pvt++;
                trajGen_with_yaw_B.b_tmp = (trajGen_with_yaw_B.ii +
                  trajGen_with_yaw_B.kend) + 1;
                for (trajGen_with_yaw_B.itemp = trajGen_with_yaw_B.jA_j;
                     trajGen_with_yaw_B.itemp <= trajGen_with_yaw_B.b_tmp;
                     trajGen_with_yaw_B.itemp++) {
                  b_A[trajGen_with_yaw_B.itemp - 1] =
                    b_A[trajGen_with_yaw_B.itemp - 1] * 9.9792015476736E+291;
                }

                trajGen_with_yaw_B.scale *= 9.9792015476736E+291;
                trajGen_with_yaw_B.smax_l *= 9.9792015476736E+291;
              } while ((std::abs(trajGen_with_yaw_B.scale) <
                        1.0020841800044864E-292) && (trajGen_with_yaw_B.pvt + 1 <
                        20));

              trajGen_with_yaw_B.scale = trajGen_with_yaw_rt_hypotd_snf
                (trajGen_with_yaw_B.smax_l, trajGen_with_yaw_xnrm2
                 (trajGen_with_yaw_B.kend, b_A, trajGen_with_yaw_B.ii + 2));
              if (trajGen_with_yaw_B.smax_l >= 0.0) {
                trajGen_with_yaw_B.scale = -trajGen_with_yaw_B.scale;
              }

              tau[trajGen_with_yaw_B.loop_ub_tmp] = (trajGen_with_yaw_B.scale -
                trajGen_with_yaw_B.smax_l) / trajGen_with_yaw_B.scale;
              trajGen_with_yaw_B.smax_l = 1.0 / (trajGen_with_yaw_B.smax_l -
                trajGen_with_yaw_B.scale);
              for (trajGen_with_yaw_B.itemp = trajGen_with_yaw_B.jA_j;
                   trajGen_with_yaw_B.itemp <= trajGen_with_yaw_B.b_tmp;
                   trajGen_with_yaw_B.itemp++) {
                b_A[trajGen_with_yaw_B.itemp - 1] = b_A[trajGen_with_yaw_B.itemp
                  - 1] * trajGen_with_yaw_B.smax_l;
              }

              for (trajGen_with_yaw_B.itemp = 0; trajGen_with_yaw_B.itemp <=
                   trajGen_with_yaw_B.pvt; trajGen_with_yaw_B.itemp++) {
                trajGen_with_yaw_B.scale *= 1.0020841800044864E-292;
              }

              trajGen_with_yaw_B.smax_l = trajGen_with_yaw_B.scale;
            } else {
              tau[trajGen_with_yaw_B.loop_ub_tmp] = (trajGen_with_yaw_B.scale -
                b_A[trajGen_with_yaw_B.ii]) / trajGen_with_yaw_B.scale;
              trajGen_with_yaw_B.smax_l = 1.0 / (b_A[trajGen_with_yaw_B.ii] -
                trajGen_with_yaw_B.scale);
              trajGen_with_yaw_B.pvt = (trajGen_with_yaw_B.ii +
                trajGen_with_yaw_B.kend) + 1;
              for (trajGen_with_yaw_B.itemp = trajGen_with_yaw_B.jA_j;
                   trajGen_with_yaw_B.itemp <= trajGen_with_yaw_B.pvt;
                   trajGen_with_yaw_B.itemp++) {
                b_A[trajGen_with_yaw_B.itemp - 1] = b_A[trajGen_with_yaw_B.itemp
                  - 1] * trajGen_with_yaw_B.smax_l;
              }

              trajGen_with_yaw_B.smax_l = trajGen_with_yaw_B.scale;
            }
          }
        }

        b_A[trajGen_with_yaw_B.ii] = trajGen_with_yaw_B.smax_l;
      } else {
        tau[trajGen_with_yaw_B.loop_ub_tmp] = 0.0;
      }

      if (trajGen_with_yaw_B.loop_ub_tmp + 1 < trajGen_with_yaw_B.na_tmp) {
        trajGen_with_yaw_B.smax_l = b_A[trajGen_with_yaw_B.ii];
        b_A[trajGen_with_yaw_B.ii] = 1.0;
        trajGen_with_yaw_B.jA_j = (trajGen_with_yaw_B.ii +
          trajGen_with_yaw_B.ma_tmp) + 1;
        if (tau[trajGen_with_yaw_B.loop_ub_tmp] != 0.0) {
          trajGen_with_yaw_B.itemp = trajGen_with_yaw_B.kend;
          trajGen_with_yaw_B.pvt = trajGen_with_yaw_B.ii +
            trajGen_with_yaw_B.kend;
          while ((trajGen_with_yaw_B.itemp + 1 > 0) &&
                 (b_A[trajGen_with_yaw_B.pvt] == 0.0)) {
            trajGen_with_yaw_B.itemp--;
            trajGen_with_yaw_B.pvt--;
          }

          trajGen_with_yaw_B.nmi--;
          exitg2 = false;
          while ((!exitg2) && (trajGen_with_yaw_B.nmi > 0)) {
            trajGen_with_yaw_B.pvt = (trajGen_with_yaw_B.nmi - 1) *
              trajGen_with_yaw_B.ma_tmp + trajGen_with_yaw_B.jA_j;
            trajGen_with_yaw_B.b_tmp = trajGen_with_yaw_B.pvt;
            do {
              exitg1 = 0;
              if (trajGen_with_yaw_B.b_tmp <= trajGen_with_yaw_B.pvt +
                  trajGen_with_yaw_B.itemp) {
                if (b_A[trajGen_with_yaw_B.b_tmp - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  trajGen_with_yaw_B.b_tmp++;
                }
              } else {
                trajGen_with_yaw_B.nmi--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }

          trajGen_with_yaw_B.nmi--;
        } else {
          trajGen_with_yaw_B.itemp = -1;
          trajGen_with_yaw_B.nmi = -1;
        }

        if (trajGen_with_yaw_B.itemp + 1 > 0) {
          if (trajGen_with_yaw_B.nmi + 1 != 0) {
            for (trajGen_with_yaw_B.pvt = 0; trajGen_with_yaw_B.pvt <=
                 trajGen_with_yaw_B.nmi; trajGen_with_yaw_B.pvt++) {
              trajGen_with_yaw_B.work[trajGen_with_yaw_B.pvt] = 0.0;
            }

            trajGen_with_yaw_B.ix_m = 0;
            trajGen_with_yaw_B.pvt = trajGen_with_yaw_B.ma_tmp *
              trajGen_with_yaw_B.nmi + trajGen_with_yaw_B.jA_j;
            for (trajGen_with_yaw_B.temp_tmp_h = trajGen_with_yaw_B.jA_j;
                 trajGen_with_yaw_B.ma_tmp < 0 ? trajGen_with_yaw_B.temp_tmp_h >=
                 trajGen_with_yaw_B.pvt : trajGen_with_yaw_B.temp_tmp_h <=
                 trajGen_with_yaw_B.pvt; trajGen_with_yaw_B.temp_tmp_h +=
                 trajGen_with_yaw_B.ma_tmp) {
              trajGen_with_yaw_B.scale = 0.0;
              trajGen_with_yaw_B.d_m = trajGen_with_yaw_B.temp_tmp_h +
                trajGen_with_yaw_B.itemp;
              for (trajGen_with_yaw_B.b_tmp = trajGen_with_yaw_B.temp_tmp_h;
                   trajGen_with_yaw_B.b_tmp <= trajGen_with_yaw_B.d_m;
                   trajGen_with_yaw_B.b_tmp++) {
                trajGen_with_yaw_B.scale += b_A[(trajGen_with_yaw_B.ii +
                  trajGen_with_yaw_B.b_tmp) - trajGen_with_yaw_B.temp_tmp_h] *
                  b_A[trajGen_with_yaw_B.b_tmp - 1];
              }

              trajGen_with_yaw_B.work[trajGen_with_yaw_B.ix_m] =
                trajGen_with_yaw_B.work[trajGen_with_yaw_B.ix_m] +
                trajGen_with_yaw_B.scale;
              trajGen_with_yaw_B.ix_m++;
            }
          }

          if (!(-tau[trajGen_with_yaw_B.loop_ub_tmp] == 0.0)) {
            for (trajGen_with_yaw_B.pvt = 0; trajGen_with_yaw_B.pvt <=
                 trajGen_with_yaw_B.nmi; trajGen_with_yaw_B.pvt++) {
              trajGen_with_yaw_B.scale =
                trajGen_with_yaw_B.work[trajGen_with_yaw_B.pvt];
              if (trajGen_with_yaw_B.scale != 0.0) {
                trajGen_with_yaw_B.scale *= -tau[trajGen_with_yaw_B.loop_ub_tmp];
                trajGen_with_yaw_B.b_tmp = trajGen_with_yaw_B.itemp +
                  trajGen_with_yaw_B.jA_j;
                for (trajGen_with_yaw_B.ix_m = trajGen_with_yaw_B.jA_j;
                     trajGen_with_yaw_B.ix_m <= trajGen_with_yaw_B.b_tmp;
                     trajGen_with_yaw_B.ix_m++) {
                  b_A[trajGen_with_yaw_B.ix_m - 1] = b_A[(trajGen_with_yaw_B.ii
                    + trajGen_with_yaw_B.ix_m) - trajGen_with_yaw_B.jA_j] *
                    trajGen_with_yaw_B.scale + b_A[trajGen_with_yaw_B.ix_m - 1];
                }
              }

              trajGen_with_yaw_B.jA_j += trajGen_with_yaw_B.ma_tmp;
            }
          }
        }

        b_A[trajGen_with_yaw_B.ii] = trajGen_with_yaw_B.smax_l;
      }

      for (trajGen_with_yaw_B.ii = trajGen_with_yaw_B.loop_ub_tmp + 2;
           trajGen_with_yaw_B.ii <= trajGen_with_yaw_B.na_tmp;
           trajGen_with_yaw_B.ii++) {
        trajGen_with_yaw_B.nmi = (trajGen_with_yaw_B.ii - 1) *
          trajGen_with_yaw_B.ma_tmp + trajGen_with_yaw_B.loop_ub_tmp;
        trajGen_with_yaw_B.smax_l = trajGen_with_yaw_B.vn1[trajGen_with_yaw_B.ii
          - 1];
        if (trajGen_with_yaw_B.smax_l != 0.0) {
          trajGen_with_yaw_B.scale = std::abs(b_A[trajGen_with_yaw_B.nmi]) /
            trajGen_with_yaw_B.smax_l;
          trajGen_with_yaw_B.scale = 1.0 - trajGen_with_yaw_B.scale *
            trajGen_with_yaw_B.scale;
          if (trajGen_with_yaw_B.scale < 0.0) {
            trajGen_with_yaw_B.scale = 0.0;
          }

          trajGen_with_yaw_B.absxk = trajGen_with_yaw_B.smax_l /
            trajGen_with_yaw_B.vn2[trajGen_with_yaw_B.ii - 1];
          trajGen_with_yaw_B.absxk = trajGen_with_yaw_B.absxk *
            trajGen_with_yaw_B.absxk * trajGen_with_yaw_B.scale;
          if (trajGen_with_yaw_B.absxk <= 1.4901161193847656E-8) {
            if (trajGen_with_yaw_B.loop_ub_tmp + 1 < trajGen_with_yaw_B.ma_tmp)
            {
              trajGen_with_yaw_B.smax_l = trajGen_with_yaw_xnrm2
                (trajGen_with_yaw_B.kend, b_A, trajGen_with_yaw_B.nmi + 2);
              trajGen_with_yaw_B.vn1[trajGen_with_yaw_B.ii - 1] =
                trajGen_with_yaw_B.smax_l;
              trajGen_with_yaw_B.vn2[trajGen_with_yaw_B.ii - 1] =
                trajGen_with_yaw_B.smax_l;
            } else {
              trajGen_with_yaw_B.vn1[trajGen_with_yaw_B.ii - 1] = 0.0;
              trajGen_with_yaw_B.vn2[trajGen_with_yaw_B.ii - 1] = 0.0;
            }
          } else {
            trajGen_with_yaw_B.vn1[trajGen_with_yaw_B.ii - 1] =
              trajGen_with_yaw_B.smax_l * std::sqrt(trajGen_with_yaw_B.scale);
          }
        }
      }
    }
  }
}

static int32_T trajGen_with_yaw_rankFromQR(const coder::array<real_T, 2U> &A)
{
  int32_T r;
  r = 0;

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  if (A.size(0) < A.size(1)) {
    trajGen_with_yaw_B.minmn = A.size(0);
    trajGen_with_yaw_B.maxmn = A.size(1);
  } else {
    trajGen_with_yaw_B.minmn = A.size(1);
    trajGen_with_yaw_B.maxmn = A.size(0);
  }

  /* End of Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  if (trajGen_with_yaw_B.minmn > 0) {
    while ((r < trajGen_with_yaw_B.minmn) && (!(std::abs(A[A.size(0) * r + r]) <=
             std::fmin(1.4901161193847656E-8, 2.2204460492503131E-15 *
                       static_cast<real_T>(trajGen_with_yaw_B.maxmn)) * std::abs
             (A[0])))) {
      r++;
    }
  }

  return r;
}

static void trajGen_with_yaw_xzgetrf(int32_T m, int32_T n, coder::array<real_T,
  2U> &A, int32_T lda, coder::array<int32_T, 2U> &ipiv, int32_T *info)
{
  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  if (m <= n) {
    trajGen_with_yaw_B.n = m;
  } else {
    trajGen_with_yaw_B.n = n;
  }

  if (trajGen_with_yaw_B.n < 1) {
    trajGen_with_yaw_B.n = 0;
  }

  ipiv.set_size(1, trajGen_with_yaw_B.n);
  if (trajGen_with_yaw_B.n > 0) {
    ipiv[0] = 1;
    trajGen_with_yaw_B.yk = 1;
    for (trajGen_with_yaw_B.k_f = 2; trajGen_with_yaw_B.k_f <=
         trajGen_with_yaw_B.n; trajGen_with_yaw_B.k_f++) {
      trajGen_with_yaw_B.yk++;
      ipiv[trajGen_with_yaw_B.k_f - 1] = trajGen_with_yaw_B.yk;
    }
  }

  *info = 0;
  if ((m >= 1) && (n >= 1)) {
    if (m - 1 <= n) {
      trajGen_with_yaw_B.yk = m - 1;
    } else {
      trajGen_with_yaw_B.yk = n;
    }

    for (trajGen_with_yaw_B.n = 0; trajGen_with_yaw_B.n < trajGen_with_yaw_B.yk;
         trajGen_with_yaw_B.n++) {
      trajGen_with_yaw_B.mmj = m - trajGen_with_yaw_B.n;
      trajGen_with_yaw_B.c_tmp = (lda + 1) * trajGen_with_yaw_B.n;
      trajGen_with_yaw_B.c_o = trajGen_with_yaw_B.c_tmp + 2;
      if (trajGen_with_yaw_B.mmj < 1) {
        trajGen_with_yaw_B.a_i = 0;
      } else {
        trajGen_with_yaw_B.a_i = 1;
        if (trajGen_with_yaw_B.mmj > 1) {
          trajGen_with_yaw_B.smax_b = std::abs(A[trajGen_with_yaw_B.c_tmp]);
          for (trajGen_with_yaw_B.k_f = 2; trajGen_with_yaw_B.k_f <=
               trajGen_with_yaw_B.mmj; trajGen_with_yaw_B.k_f++) {
            trajGen_with_yaw_B.s_da = std::abs(A[(trajGen_with_yaw_B.c_tmp +
              trajGen_with_yaw_B.k_f) - 1]);
            if (trajGen_with_yaw_B.s_da > trajGen_with_yaw_B.smax_b) {
              trajGen_with_yaw_B.a_i = trajGen_with_yaw_B.k_f;
              trajGen_with_yaw_B.smax_b = trajGen_with_yaw_B.s_da;
            }
          }
        }
      }

      if (A[(trajGen_with_yaw_B.c_tmp + trajGen_with_yaw_B.a_i) - 1] != 0.0) {
        if (trajGen_with_yaw_B.a_i - 1 != 0) {
          trajGen_with_yaw_B.a_i += trajGen_with_yaw_B.n;
          ipiv[trajGen_with_yaw_B.n] = trajGen_with_yaw_B.a_i;
          for (trajGen_with_yaw_B.k_f = 0; trajGen_with_yaw_B.k_f < n;
               trajGen_with_yaw_B.k_f++) {
            trajGen_with_yaw_B.temp_tmp_g = trajGen_with_yaw_B.k_f * lda;
            trajGen_with_yaw_B.temp_tmp_c = trajGen_with_yaw_B.temp_tmp_g +
              trajGen_with_yaw_B.n;
            trajGen_with_yaw_B.smax_b = A[trajGen_with_yaw_B.temp_tmp_c];
            trajGen_with_yaw_B.temp_tmp_g = (trajGen_with_yaw_B.temp_tmp_g +
              trajGen_with_yaw_B.a_i) - 1;
            A[trajGen_with_yaw_B.temp_tmp_c] = A[trajGen_with_yaw_B.temp_tmp_g];
            A[trajGen_with_yaw_B.temp_tmp_g] = trajGen_with_yaw_B.smax_b;
          }
        }

        trajGen_with_yaw_B.k_f = (trajGen_with_yaw_B.c_tmp +
          trajGen_with_yaw_B.mmj) + 2;
        for (trajGen_with_yaw_B.a_i = trajGen_with_yaw_B.c_o;
             trajGen_with_yaw_B.a_i <= trajGen_with_yaw_B.k_f - 2;
             trajGen_with_yaw_B.a_i++) {
          A[trajGen_with_yaw_B.a_i - 1] = A[trajGen_with_yaw_B.a_i - 1] /
            A[trajGen_with_yaw_B.c_tmp];
        }
      } else {
        *info = trajGen_with_yaw_B.n + 1;
      }

      trajGen_with_yaw_B.c_o = (n - trajGen_with_yaw_B.n) - 2;
      trajGen_with_yaw_B.temp_tmp_g = (trajGen_with_yaw_B.c_tmp + lda) + 1;
      trajGen_with_yaw_B.jA_i = trajGen_with_yaw_B.temp_tmp_g;
      for (trajGen_with_yaw_B.a_i = 0; trajGen_with_yaw_B.a_i <=
           trajGen_with_yaw_B.c_o; trajGen_with_yaw_B.a_i++) {
        trajGen_with_yaw_B.smax_b = A[(trajGen_with_yaw_B.a_i * lda +
          trajGen_with_yaw_B.temp_tmp_g) - 1];
        if (trajGen_with_yaw_B.smax_b != 0.0) {
          trajGen_with_yaw_B.b_f = trajGen_with_yaw_B.jA_i + 1;
          trajGen_with_yaw_B.k_f = (trajGen_with_yaw_B.mmj +
            trajGen_with_yaw_B.jA_i) - 1;
          for (trajGen_with_yaw_B.temp_tmp_c = trajGen_with_yaw_B.b_f;
               trajGen_with_yaw_B.temp_tmp_c <= trajGen_with_yaw_B.k_f;
               trajGen_with_yaw_B.temp_tmp_c++) {
            A[trajGen_with_yaw_B.temp_tmp_c - 1] = A[(trajGen_with_yaw_B.c_tmp +
              trajGen_with_yaw_B.temp_tmp_c) - trajGen_with_yaw_B.jA_i] *
              -trajGen_with_yaw_B.smax_b + A[trajGen_with_yaw_B.temp_tmp_c - 1];
          }
        }

        trajGen_with_yaw_B.jA_i += lda;
      }
    }

    if ((*info == 0) && (m <= n) && (!(A[((m - 1) * A.size(0) + m) - 1] != 0.0)))
    {
      *info = m;
    }
  }

  /* End of Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
}

static void trajGen_with_yaw_mldivide(const coder::array<real_T, 2U> &A, const
  coder::array<real_T, 1U> &B, coder::array<real_T, 1U> &Y)
{
  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  if ((A.size(0) == 0) || (A.size(1) == 0) || (B.size(0) == 0)) {
    trajGen_with_yaw_B.mn = A.size(1);
    Y.set_size(A.size(1));
    for (trajGen_with_yaw_B.i_n = 0; trajGen_with_yaw_B.i_n <
         trajGen_with_yaw_B.mn; trajGen_with_yaw_B.i_n++) {
      Y[trajGen_with_yaw_B.i_n] = 0.0;
    }
  } else if (A.size(0) == A.size(1)) {
    if (A.size(0) <= A.size(1)) {
      trajGen_with_yaw_B.rankA = A.size(0);
    } else {
      trajGen_with_yaw_B.rankA = A.size(1);
    }

    if (B.size(0) <= trajGen_with_yaw_B.rankA) {
      trajGen_with_yaw_B.rankA = B.size(0);
    }

    trajGen_with_yaw_B.b_A.set_size(A.size(0), A.size(1));
    trajGen_with_yaw_B.mn = A.size(0) * A.size(1);
    for (trajGen_with_yaw_B.i_n = 0; trajGen_with_yaw_B.i_n <
         trajGen_with_yaw_B.mn; trajGen_with_yaw_B.i_n++) {
      trajGen_with_yaw_B.b_A[trajGen_with_yaw_B.i_n] = A[trajGen_with_yaw_B.i_n];
    }

    trajGen_with_yaw_xzgetrf(trajGen_with_yaw_B.rankA, trajGen_with_yaw_B.rankA,
      trajGen_with_yaw_B.b_A, A.size(0), trajGen_with_yaw_B.jpvt,
      &trajGen_with_yaw_B.i_n);
    trajGen_with_yaw_B.mn = B.size(0);
    Y.set_size(B.size(0));
    for (trajGen_with_yaw_B.i_n = 0; trajGen_with_yaw_B.i_n <
         trajGen_with_yaw_B.mn; trajGen_with_yaw_B.i_n++) {
      Y[trajGen_with_yaw_B.i_n] = B[trajGen_with_yaw_B.i_n];
    }

    trajGen_with_yaw_B.mn = trajGen_with_yaw_B.b_A.size(0);
    trajGen_with_yaw_B.m = trajGen_with_yaw_B.rankA - 2;
    for (trajGen_with_yaw_B.i_n = 0; trajGen_with_yaw_B.i_n <=
         trajGen_with_yaw_B.m; trajGen_with_yaw_B.i_n++) {
      trajGen_with_yaw_B.kAcol_m =
        trajGen_with_yaw_B.jpvt[trajGen_with_yaw_B.i_n];
      if (trajGen_with_yaw_B.i_n + 1 != trajGen_with_yaw_B.kAcol_m) {
        trajGen_with_yaw_B.wj = Y[trajGen_with_yaw_B.i_n];
        Y[trajGen_with_yaw_B.i_n] = Y[trajGen_with_yaw_B.kAcol_m - 1];
        Y[trajGen_with_yaw_B.kAcol_m - 1] = trajGen_with_yaw_B.wj;
      }
    }

    for (trajGen_with_yaw_B.m = 0; trajGen_with_yaw_B.m <
         trajGen_with_yaw_B.rankA; trajGen_with_yaw_B.m++) {
      trajGen_with_yaw_B.kAcol_m = trajGen_with_yaw_B.mn * trajGen_with_yaw_B.m
        - 1;
      if (Y[trajGen_with_yaw_B.m] != 0.0) {
        for (trajGen_with_yaw_B.i_n = trajGen_with_yaw_B.m + 2;
             trajGen_with_yaw_B.i_n <= trajGen_with_yaw_B.rankA;
             trajGen_with_yaw_B.i_n++) {
          Y[trajGen_with_yaw_B.i_n - 1] = Y[trajGen_with_yaw_B.i_n - 1] -
            trajGen_with_yaw_B.b_A[trajGen_with_yaw_B.i_n +
            trajGen_with_yaw_B.kAcol_m] * Y[trajGen_with_yaw_B.m];
        }
      }
    }

    for (trajGen_with_yaw_B.k_c = trajGen_with_yaw_B.rankA;
         trajGen_with_yaw_B.k_c >= 1; trajGen_with_yaw_B.k_c--) {
      trajGen_with_yaw_B.kAcol_m = (trajGen_with_yaw_B.k_c - 1) *
        trajGen_with_yaw_B.mn - 1;
      trajGen_with_yaw_B.wj = Y[trajGen_with_yaw_B.k_c - 1];
      if (trajGen_with_yaw_B.wj != 0.0) {
        Y[trajGen_with_yaw_B.k_c - 1] = trajGen_with_yaw_B.wj /
          trajGen_with_yaw_B.b_A[trajGen_with_yaw_B.k_c +
          trajGen_with_yaw_B.kAcol_m];
        trajGen_with_yaw_B.m = trajGen_with_yaw_B.k_c - 2;
        for (trajGen_with_yaw_B.i_n = 0; trajGen_with_yaw_B.i_n <=
             trajGen_with_yaw_B.m; trajGen_with_yaw_B.i_n++) {
          Y[trajGen_with_yaw_B.i_n] = Y[trajGen_with_yaw_B.i_n] -
            trajGen_with_yaw_B.b_A[(trajGen_with_yaw_B.i_n +
            trajGen_with_yaw_B.kAcol_m) + 1] * Y[trajGen_with_yaw_B.k_c - 1];
        }
      }
    }
  } else {
    trajGen_with_yaw_xgeqp3(A, trajGen_with_yaw_B.b_A, trajGen_with_yaw_B.tau,
      trajGen_with_yaw_B.jpvt);
    trajGen_with_yaw_B.rankA = trajGen_with_yaw_rankFromQR
      (trajGen_with_yaw_B.b_A);
    trajGen_with_yaw_B.mn = trajGen_with_yaw_B.b_A.size(1);
    Y.set_size(trajGen_with_yaw_B.b_A.size(1));
    for (trajGen_with_yaw_B.i_n = 0; trajGen_with_yaw_B.i_n <
         trajGen_with_yaw_B.mn; trajGen_with_yaw_B.i_n++) {
      Y[trajGen_with_yaw_B.i_n] = 0.0;
    }

    trajGen_with_yaw_B.mn = B.size(0);
    trajGen_with_yaw_B.B.set_size(B.size(0));
    for (trajGen_with_yaw_B.i_n = 0; trajGen_with_yaw_B.i_n <
         trajGen_with_yaw_B.mn; trajGen_with_yaw_B.i_n++) {
      trajGen_with_yaw_B.B[trajGen_with_yaw_B.i_n] = B[trajGen_with_yaw_B.i_n];
    }

    if (trajGen_with_yaw_B.b_A.size(0) <= trajGen_with_yaw_B.b_A.size(1)) {
      trajGen_with_yaw_B.mn = trajGen_with_yaw_B.b_A.size(0);
    } else {
      trajGen_with_yaw_B.mn = trajGen_with_yaw_B.b_A.size(1);
    }

    for (trajGen_with_yaw_B.kAcol_m = 0; trajGen_with_yaw_B.kAcol_m <
         trajGen_with_yaw_B.mn; trajGen_with_yaw_B.kAcol_m++) {
      trajGen_with_yaw_B.m = trajGen_with_yaw_B.b_A.size(0);
      if (trajGen_with_yaw_B.tau[trajGen_with_yaw_B.kAcol_m] != 0.0) {
        trajGen_with_yaw_B.wj = trajGen_with_yaw_B.B[trajGen_with_yaw_B.kAcol_m];
        for (trajGen_with_yaw_B.i_n = trajGen_with_yaw_B.kAcol_m + 2;
             trajGen_with_yaw_B.i_n <= trajGen_with_yaw_B.m;
             trajGen_with_yaw_B.i_n++) {
          trajGen_with_yaw_B.wj += trajGen_with_yaw_B.b_A
            [(trajGen_with_yaw_B.b_A.size(0) * trajGen_with_yaw_B.kAcol_m +
              trajGen_with_yaw_B.i_n) - 1] *
            trajGen_with_yaw_B.B[trajGen_with_yaw_B.i_n - 1];
        }

        trajGen_with_yaw_B.wj *=
          trajGen_with_yaw_B.tau[trajGen_with_yaw_B.kAcol_m];
        if (trajGen_with_yaw_B.wj != 0.0) {
          trajGen_with_yaw_B.B[trajGen_with_yaw_B.kAcol_m] =
            trajGen_with_yaw_B.B[trajGen_with_yaw_B.kAcol_m] -
            trajGen_with_yaw_B.wj;
          for (trajGen_with_yaw_B.i_n = trajGen_with_yaw_B.kAcol_m + 2;
               trajGen_with_yaw_B.i_n <= trajGen_with_yaw_B.m;
               trajGen_with_yaw_B.i_n++) {
            trajGen_with_yaw_B.B[trajGen_with_yaw_B.i_n - 1] =
              trajGen_with_yaw_B.B[trajGen_with_yaw_B.i_n - 1] -
              trajGen_with_yaw_B.b_A[(trajGen_with_yaw_B.b_A.size(0) *
              trajGen_with_yaw_B.kAcol_m + trajGen_with_yaw_B.i_n) - 1] *
              trajGen_with_yaw_B.wj;
          }
        }
      }
    }

    for (trajGen_with_yaw_B.i_n = 0; trajGen_with_yaw_B.i_n <
         trajGen_with_yaw_B.rankA; trajGen_with_yaw_B.i_n++) {
      Y[trajGen_with_yaw_B.jpvt[trajGen_with_yaw_B.i_n] - 1] =
        trajGen_with_yaw_B.B[trajGen_with_yaw_B.i_n];
    }

    for (trajGen_with_yaw_B.i_n = trajGen_with_yaw_B.rankA;
         trajGen_with_yaw_B.i_n >= 1; trajGen_with_yaw_B.i_n--) {
      trajGen_with_yaw_B.mn = trajGen_with_yaw_B.jpvt[trajGen_with_yaw_B.i_n - 1];
      Y[trajGen_with_yaw_B.mn - 1] = Y[trajGen_with_yaw_B.mn - 1] /
        trajGen_with_yaw_B.b_A[((trajGen_with_yaw_B.i_n - 1) *
        trajGen_with_yaw_B.b_A.size(0) + trajGen_with_yaw_B.i_n) - 1];
      trajGen_with_yaw_B.kAcol_m = trajGen_with_yaw_B.i_n - 2;
      for (trajGen_with_yaw_B.m = 0; trajGen_with_yaw_B.m <=
           trajGen_with_yaw_B.kAcol_m; trajGen_with_yaw_B.m++) {
        Y[trajGen_with_yaw_B.jpvt[trajGen_with_yaw_B.m] - 1] =
          Y[trajGen_with_yaw_B.jpvt[trajGen_with_yaw_B.m] - 1] -
          trajGen_with_yaw_B.b_A[(trajGen_with_yaw_B.i_n - 1) *
          trajGen_with_yaw_B.b_A.size(0) + trajGen_with_yaw_B.m] *
          Y[trajGen_with_yaw_B.mn - 1];
      }
    }
  }

  /* End of Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
}

static void trajGen_with_yaw_xzgetrf_g(real_T A[64], int32_T ipiv[8], int32_T
  *info)
{
  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  for (trajGen_with_yaw_B.c_jz = 0; trajGen_with_yaw_B.c_jz < 8;
       trajGen_with_yaw_B.c_jz++) {
    ipiv[trajGen_with_yaw_B.c_jz] = trajGen_with_yaw_B.c_jz + 1;
  }

  *info = 0;
  for (trajGen_with_yaw_B.b_j_a = 0; trajGen_with_yaw_B.b_j_a < 7;
       trajGen_with_yaw_B.b_j_a++) {
    trajGen_with_yaw_B.c_j = trajGen_with_yaw_B.b_j_a * 9 + 2;
    trajGen_with_yaw_B.jj = trajGen_with_yaw_B.b_j_a * 9;
    trajGen_with_yaw_B.c_jz = 8 - trajGen_with_yaw_B.b_j_a;
    trajGen_with_yaw_B.a_o = 1;
    trajGen_with_yaw_B.smax = std::abs(A[trajGen_with_yaw_B.jj]);
    for (trajGen_with_yaw_B.k = 2; trajGen_with_yaw_B.k <=
         trajGen_with_yaw_B.c_jz; trajGen_with_yaw_B.k++) {
      trajGen_with_yaw_B.s_d = std::abs(A[(trajGen_with_yaw_B.c_j +
        trajGen_with_yaw_B.k) - 3]);
      if (trajGen_with_yaw_B.s_d > trajGen_with_yaw_B.smax) {
        trajGen_with_yaw_B.a_o = trajGen_with_yaw_B.k;
        trajGen_with_yaw_B.smax = trajGen_with_yaw_B.s_d;
      }
    }

    if (A[(trajGen_with_yaw_B.c_j + trajGen_with_yaw_B.a_o) - 3] != 0.0) {
      if (trajGen_with_yaw_B.a_o - 1 != 0) {
        trajGen_with_yaw_B.a_o += trajGen_with_yaw_B.b_j_a;
        ipiv[trajGen_with_yaw_B.b_j_a] = trajGen_with_yaw_B.a_o;
        for (trajGen_with_yaw_B.k = 0; trajGen_with_yaw_B.k < 8;
             trajGen_with_yaw_B.k++) {
          trajGen_with_yaw_B.c_jz = trajGen_with_yaw_B.k << 3;
          trajGen_with_yaw_B.temp_tmp = trajGen_with_yaw_B.c_jz +
            trajGen_with_yaw_B.b_j_a;
          trajGen_with_yaw_B.smax = A[trajGen_with_yaw_B.temp_tmp];
          trajGen_with_yaw_B.c_jz = (trajGen_with_yaw_B.c_jz +
            trajGen_with_yaw_B.a_o) - 1;
          A[trajGen_with_yaw_B.temp_tmp] = A[trajGen_with_yaw_B.c_jz];
          A[trajGen_with_yaw_B.c_jz] = trajGen_with_yaw_B.smax;
        }
      }

      trajGen_with_yaw_B.k = trajGen_with_yaw_B.c_j - trajGen_with_yaw_B.b_j_a;
      for (trajGen_with_yaw_B.c_jz = trajGen_with_yaw_B.c_j;
           trajGen_with_yaw_B.c_jz <= trajGen_with_yaw_B.k + 6;
           trajGen_with_yaw_B.c_jz++) {
        A[trajGen_with_yaw_B.c_jz - 1] /= A[trajGen_with_yaw_B.jj];
      }
    } else {
      *info = trajGen_with_yaw_B.b_j_a + 1;
    }

    trajGen_with_yaw_B.jA_n = trajGen_with_yaw_B.jj;
    trajGen_with_yaw_B.k = 7 - trajGen_with_yaw_B.b_j_a;
    for (trajGen_with_yaw_B.a_o = 0; trajGen_with_yaw_B.a_o <
         trajGen_with_yaw_B.k; trajGen_with_yaw_B.a_o++) {
      trajGen_with_yaw_B.smax = A[((trajGen_with_yaw_B.a_o << 3) +
        trajGen_with_yaw_B.jj) + 8];
      if (trajGen_with_yaw_B.smax != 0.0) {
        trajGen_with_yaw_B.c_jz = trajGen_with_yaw_B.jA_n + 10;
        trajGen_with_yaw_B.d_i = (trajGen_with_yaw_B.jA_n -
          trajGen_with_yaw_B.b_j_a) + 16;
        for (trajGen_with_yaw_B.temp_tmp = trajGen_with_yaw_B.c_jz;
             trajGen_with_yaw_B.temp_tmp <= trajGen_with_yaw_B.d_i;
             trajGen_with_yaw_B.temp_tmp++) {
          A[trajGen_with_yaw_B.temp_tmp - 1] += A[((trajGen_with_yaw_B.c_j +
            trajGen_with_yaw_B.temp_tmp) - trajGen_with_yaw_B.jA_n) - 11] *
            -trajGen_with_yaw_B.smax;
        }
      }

      trajGen_with_yaw_B.jA_n += 8;
    }
  }

  if ((*info == 0) && (!(A[63] != 0.0))) {
    *info = 8;
  }

  /* End of Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
}

static void trajGen_with_yaw_mldivide_gw(const real_T A[64], real_T B[8])
{
  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  std::memcpy(&trajGen_with_yaw_B.c_A_b[0], &A[0], sizeof(real_T) << 6U);
  trajGen_with_yaw_xzgetrf_g(trajGen_with_yaw_B.c_A_b, trajGen_with_yaw_B.b_ipiv,
    &trajGen_with_yaw_B.b_info);
  for (trajGen_with_yaw_B.b_info = 0; trajGen_with_yaw_B.b_info < 7;
       trajGen_with_yaw_B.b_info++) {
    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.kAcol =
      trajGen_with_yaw_B.b_ipiv[trajGen_with_yaw_B.b_info];
    if (trajGen_with_yaw_B.b_info + 1 != trajGen_with_yaw_B.kAcol) {
      trajGen_with_yaw_B.temp_d = B[trajGen_with_yaw_B.b_info];
      B[trajGen_with_yaw_B.b_info] = B[trajGen_with_yaw_B.kAcol - 1];
      B[trajGen_with_yaw_B.kAcol - 1] = trajGen_with_yaw_B.temp_d;
    }
  }

  for (trajGen_with_yaw_B.b_info = 0; trajGen_with_yaw_B.b_info < 8;
       trajGen_with_yaw_B.b_info++) {
    trajGen_with_yaw_B.kAcol = (trajGen_with_yaw_B.b_info << 3) - 1;
    if (B[trajGen_with_yaw_B.b_info] != 0.0) {
      for (trajGen_with_yaw_B.i_o = trajGen_with_yaw_B.b_info + 2;
           trajGen_with_yaw_B.i_o < 9; trajGen_with_yaw_B.i_o++) {
        /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
        B[trajGen_with_yaw_B.i_o - 1] -=
          trajGen_with_yaw_B.c_A_b[trajGen_with_yaw_B.i_o +
          trajGen_with_yaw_B.kAcol] * B[trajGen_with_yaw_B.b_info];
      }
    }
  }

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  for (trajGen_with_yaw_B.i_o = 7; trajGen_with_yaw_B.i_o >= 0;
       trajGen_with_yaw_B.i_o--) {
    trajGen_with_yaw_B.kAcol = trajGen_with_yaw_B.i_o << 3;
    trajGen_with_yaw_B.temp_d = B[trajGen_with_yaw_B.i_o];
    if (trajGen_with_yaw_B.temp_d != 0.0) {
      B[trajGen_with_yaw_B.i_o] = trajGen_with_yaw_B.temp_d /
        trajGen_with_yaw_B.c_A_b[trajGen_with_yaw_B.i_o +
        trajGen_with_yaw_B.kAcol];
      for (trajGen_with_yaw_B.b_info = 0; trajGen_with_yaw_B.b_info <
           trajGen_with_yaw_B.i_o; trajGen_with_yaw_B.b_info++) {
        B[trajGen_with_yaw_B.b_info] -=
          trajGen_with_yaw_B.c_A_b[trajGen_with_yaw_B.b_info +
          trajGen_with_yaw_B.kAcol] * B[trajGen_with_yaw_B.i_o];
      }
    }
  }
}

static real_T trajGen_with_yaw_rcond(const real_T A[64])
{
  real_T result;
  int32_T exitg2;
  boolean_T exitg1;
  boolean_T p;
  result = 0.0;
  trajGen_with_yaw_B.normA = 0.0;
  trajGen_with_yaw_B.b_j = 0;
  exitg1 = false;
  while ((!exitg1) && (trajGen_with_yaw_B.b_j < 8)) {
    trajGen_with_yaw_B.s = 0.0;
    for (trajGen_with_yaw_B.i = 0; trajGen_with_yaw_B.i < 8;
         trajGen_with_yaw_B.i++) {
      trajGen_with_yaw_B.s += std::abs(A[(trajGen_with_yaw_B.b_j << 3) +
        trajGen_with_yaw_B.i]);
    }

    if (std::isnan(trajGen_with_yaw_B.s)) {
      trajGen_with_yaw_B.normA = (rtNaN);
      exitg1 = true;
    } else {
      if (trajGen_with_yaw_B.s > trajGen_with_yaw_B.normA) {
        trajGen_with_yaw_B.normA = trajGen_with_yaw_B.s;
      }

      trajGen_with_yaw_B.b_j++;
    }
  }

  if (!(trajGen_with_yaw_B.normA == 0.0)) {
    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    std::memcpy(&trajGen_with_yaw_B.c_A[0], &A[0], sizeof(real_T) << 6U);
    trajGen_with_yaw_xzgetrf_g(trajGen_with_yaw_B.c_A, trajGen_with_yaw_B.ipiv,
      &trajGen_with_yaw_B.iter);
    trajGen_with_yaw_B.iter = 7;
    do {
      exitg2 = 0;
      if (trajGen_with_yaw_B.iter + 1 > 0) {
        if (trajGen_with_yaw_B.c_A[(trajGen_with_yaw_B.iter << 3) +
            trajGen_with_yaw_B.iter] == 0.0) {
          exitg2 = 1;
        } else {
          trajGen_with_yaw_B.iter--;
        }
      } else {
        trajGen_with_yaw_B.s = 0.0;
        trajGen_with_yaw_B.iter = 2;
        trajGen_with_yaw_B.kase = 1;
        trajGen_with_yaw_B.jump = 1;
        trajGen_with_yaw_B.j = 1;
        for (trajGen_with_yaw_B.i = 0; trajGen_with_yaw_B.i < 8;
             trajGen_with_yaw_B.i++) {
          trajGen_with_yaw_B.x[trajGen_with_yaw_B.i] = 0.125;
        }

        while (trajGen_with_yaw_B.kase != 0) {
          if (trajGen_with_yaw_B.kase == 1) {
            for (trajGen_with_yaw_B.b_j = 0; trajGen_with_yaw_B.b_j < 8;
                 trajGen_with_yaw_B.b_j++) {
              trajGen_with_yaw_B.jA = (trajGen_with_yaw_B.b_j << 3) +
                trajGen_with_yaw_B.b_j;
              trajGen_with_yaw_B.c_f = 7 - trajGen_with_yaw_B.b_j;
              for (trajGen_with_yaw_B.i = 0; trajGen_with_yaw_B.i <
                   trajGen_with_yaw_B.c_f; trajGen_with_yaw_B.i++) {
                trajGen_with_yaw_B.ix = (trajGen_with_yaw_B.i +
                  trajGen_with_yaw_B.b_j) + 1;
                trajGen_with_yaw_B.x[trajGen_with_yaw_B.ix] -=
                  trajGen_with_yaw_B.c_A[(trajGen_with_yaw_B.i +
                  trajGen_with_yaw_B.jA) + 1] *
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.b_j];
              }
            }

            for (trajGen_with_yaw_B.b_j = 7; trajGen_with_yaw_B.b_j >= 0;
                 trajGen_with_yaw_B.b_j--) {
              trajGen_with_yaw_B.jA = (trajGen_with_yaw_B.b_j << 3) +
                trajGen_with_yaw_B.b_j;
              trajGen_with_yaw_B.x[trajGen_with_yaw_B.b_j] /=
                trajGen_with_yaw_B.c_A[trajGen_with_yaw_B.jA];
              for (trajGen_with_yaw_B.i = 0; trajGen_with_yaw_B.i <
                   trajGen_with_yaw_B.b_j; trajGen_with_yaw_B.i++) {
                trajGen_with_yaw_B.ix = (trajGen_with_yaw_B.b_j -
                  trajGen_with_yaw_B.i) - 1;
                trajGen_with_yaw_B.x[trajGen_with_yaw_B.ix] -=
                  trajGen_with_yaw_B.c_A[(trajGen_with_yaw_B.jA -
                  trajGen_with_yaw_B.i) - 1] *
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.b_j];
              }
            }
          } else {
            for (trajGen_with_yaw_B.b_j = 0; trajGen_with_yaw_B.b_j < 8;
                 trajGen_with_yaw_B.b_j++) {
              trajGen_with_yaw_B.jA = trajGen_with_yaw_B.b_j << 3;
              trajGen_with_yaw_B.temp =
                trajGen_with_yaw_B.x[trajGen_with_yaw_B.b_j];
              for (trajGen_with_yaw_B.i = 0; trajGen_with_yaw_B.i <
                   trajGen_with_yaw_B.b_j; trajGen_with_yaw_B.i++) {
                trajGen_with_yaw_B.temp -=
                  trajGen_with_yaw_B.c_A[trajGen_with_yaw_B.jA +
                  trajGen_with_yaw_B.i] *
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.i];
              }

              trajGen_with_yaw_B.x[trajGen_with_yaw_B.b_j] =
                trajGen_with_yaw_B.temp /
                trajGen_with_yaw_B.c_A[trajGen_with_yaw_B.jA +
                trajGen_with_yaw_B.b_j];
            }

            for (trajGen_with_yaw_B.b_j = 7; trajGen_with_yaw_B.b_j >= 0;
                 trajGen_with_yaw_B.b_j--) {
              trajGen_with_yaw_B.jA = (trajGen_with_yaw_B.b_j << 3) - 1;
              trajGen_with_yaw_B.temp =
                trajGen_with_yaw_B.x[trajGen_with_yaw_B.b_j];
              for (trajGen_with_yaw_B.i = 8; trajGen_with_yaw_B.i >=
                   trajGen_with_yaw_B.b_j + 2; trajGen_with_yaw_B.i--) {
                trajGen_with_yaw_B.temp -=
                  trajGen_with_yaw_B.c_A[trajGen_with_yaw_B.jA +
                  trajGen_with_yaw_B.i] *
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.i - 1];
              }

              trajGen_with_yaw_B.x[trajGen_with_yaw_B.b_j] =
                trajGen_with_yaw_B.temp;
            }
          }

          if (trajGen_with_yaw_B.jump == 1) {
            trajGen_with_yaw_B.s = 0.0;
            for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 8;
                 trajGen_with_yaw_B.kase++) {
              trajGen_with_yaw_B.s += std::abs
                (trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase]);
            }

            if ((!std::isinf(trajGen_with_yaw_B.s)) && (!std::isnan
                 (trajGen_with_yaw_B.s))) {
              for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 8;
                   trajGen_with_yaw_B.kase++) {
                trajGen_with_yaw_B.temp =
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase];
                trajGen_with_yaw_B.absrexk = std::abs(trajGen_with_yaw_B.temp);
                if (trajGen_with_yaw_B.absrexk > 2.2250738585072014E-308) {
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase] =
                    trajGen_with_yaw_B.temp / trajGen_with_yaw_B.absrexk;
                } else {
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase] = 1.0;
                }
              }

              trajGen_with_yaw_B.kase = 2;
              trajGen_with_yaw_B.jump = 2;
            } else {
              trajGen_with_yaw_B.kase = 0;
            }
          } else if (trajGen_with_yaw_B.jump == 2) {
            trajGen_with_yaw_B.j = 1;
            trajGen_with_yaw_B.temp = std::abs(trajGen_with_yaw_B.x[0]);
            for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 7;
                 trajGen_with_yaw_B.kase++) {
              trajGen_with_yaw_B.absrexk = std::abs
                (trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase + 1]);
              if (!(trajGen_with_yaw_B.absrexk <= trajGen_with_yaw_B.temp)) {
                trajGen_with_yaw_B.j = trajGen_with_yaw_B.kase + 2;
                trajGen_with_yaw_B.temp = trajGen_with_yaw_B.absrexk;
              }
            }

            trajGen_with_yaw_B.iter = 2;
            std::memset(&trajGen_with_yaw_B.x[0], 0, sizeof(real_T) << 3U);
            trajGen_with_yaw_B.x[trajGen_with_yaw_B.j - 1] = 1.0;
            trajGen_with_yaw_B.kase = 1;
            trajGen_with_yaw_B.jump = 3;
          } else if (trajGen_with_yaw_B.jump == 3) {
            trajGen_with_yaw_B.s = 0.0;
            for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 8;
                 trajGen_with_yaw_B.kase++) {
              trajGen_with_yaw_B.s += std::abs
                (trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase]);
            }

            if (trajGen_with_yaw_B.s <= trajGen_with_yaw_B.x[0]) {
              trajGen_with_yaw_B.temp = 1.0;
              for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 8;
                   trajGen_with_yaw_B.kase++) {
                trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase] =
                  (((static_cast<real_T>(trajGen_with_yaw_B.kase) + 1.0) - 1.0) /
                   7.0 + 1.0) * trajGen_with_yaw_B.temp;
                trajGen_with_yaw_B.temp = -trajGen_with_yaw_B.temp;
              }

              trajGen_with_yaw_B.kase = 1;
              trajGen_with_yaw_B.jump = 5;
            } else {
              for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 8;
                   trajGen_with_yaw_B.kase++) {
                trajGen_with_yaw_B.temp =
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase];
                trajGen_with_yaw_B.absrexk = std::abs(trajGen_with_yaw_B.temp);
                if (trajGen_with_yaw_B.absrexk > 2.2250738585072014E-308) {
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase] =
                    trajGen_with_yaw_B.temp / trajGen_with_yaw_B.absrexk;
                } else {
                  trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase] = 1.0;
                }
              }

              trajGen_with_yaw_B.kase = 2;
              trajGen_with_yaw_B.jump = 4;
            }
          } else if (trajGen_with_yaw_B.jump == 4) {
            trajGen_with_yaw_B.jump = trajGen_with_yaw_B.j;
            trajGen_with_yaw_B.j = 1;
            trajGen_with_yaw_B.temp = std::abs(trajGen_with_yaw_B.x[0]);
            for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 7;
                 trajGen_with_yaw_B.kase++) {
              trajGen_with_yaw_B.absrexk = std::abs
                (trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase + 1]);
              if (!(trajGen_with_yaw_B.absrexk <= trajGen_with_yaw_B.temp)) {
                trajGen_with_yaw_B.j = trajGen_with_yaw_B.kase + 2;
                trajGen_with_yaw_B.temp = trajGen_with_yaw_B.absrexk;
              }
            }

            if ((std::abs(trajGen_with_yaw_B.x[trajGen_with_yaw_B.jump - 1]) !=
                 std::abs(trajGen_with_yaw_B.x[trajGen_with_yaw_B.j - 1])) &&
                (trajGen_with_yaw_B.iter <= 5)) {
              trajGen_with_yaw_B.iter++;
              std::memset(&trajGen_with_yaw_B.x[0], 0, sizeof(real_T) << 3U);
              trajGen_with_yaw_B.x[trajGen_with_yaw_B.j - 1] = 1.0;
              trajGen_with_yaw_B.kase = 1;
              trajGen_with_yaw_B.jump = 3;
            } else {
              trajGen_with_yaw_B.temp = 1.0;
              for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 8;
                   trajGen_with_yaw_B.kase++) {
                trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase] =
                  (((static_cast<real_T>(trajGen_with_yaw_B.kase) + 1.0) - 1.0) /
                   7.0 + 1.0) * trajGen_with_yaw_B.temp;
                trajGen_with_yaw_B.temp = -trajGen_with_yaw_B.temp;
              }

              trajGen_with_yaw_B.kase = 1;
              trajGen_with_yaw_B.jump = 5;
            }
          } else if (trajGen_with_yaw_B.jump == 5) {
            trajGen_with_yaw_B.temp = 0.0;
            for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 8;
                 trajGen_with_yaw_B.kase++) {
              trajGen_with_yaw_B.temp += std::abs
                (trajGen_with_yaw_B.x[trajGen_with_yaw_B.kase]);
            }

            trajGen_with_yaw_B.temp = 2.0 * trajGen_with_yaw_B.temp / 3.0 / 8.0;
            if (trajGen_with_yaw_B.temp > trajGen_with_yaw_B.s) {
              trajGen_with_yaw_B.s = trajGen_with_yaw_B.temp;
            }

            trajGen_with_yaw_B.kase = 0;
          }
        }

        if (trajGen_with_yaw_B.s != 0.0) {
          result = 1.0 / trajGen_with_yaw_B.s / trajGen_with_yaw_B.normA;
        }

        if (std::isnan(result)) {
          p = false;
          for (trajGen_with_yaw_B.kase = 0; trajGen_with_yaw_B.kase < 64;
               trajGen_with_yaw_B.kase++) {
            if (p || std::isnan(trajGen_with_yaw_B.c_A[trajGen_with_yaw_B.kase]))
            {
              p = true;
            }
          }

          if (!p) {
            result = 0.0;
          }
        }

        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }

  return result;
}

static void MinJerkPolyTraj_computePPandTim(shared_uav_rst_sluav_internal_T *obj,
  const real_T waypoints[8], const real_T timePoints[2], real_T ppMatrix[32],
  coder::array<real_T, 2U> &toaZero, real_T *maxIterStatus, real_T
  *maxTimeStatus, real_T *singularityStatus)
{
  int8_T d;
  static const int8_T tmp[8]{ 0, 0, 0, 1, 0, 0, 0, 0 };

  obj->TimeOffset = 0.0;

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 4; trajGen_with_yaw_B.c
       ++) {
    trajGen_with_yaw_B.b_p_tmp = trajGen_with_yaw_B.c << 1;
    trajGen_with_yaw_B.b_p[trajGen_with_yaw_B.b_p_tmp] =
      waypoints[trajGen_with_yaw_B.c];
    trajGen_with_yaw_B.b_p[trajGen_with_yaw_B.b_p_tmp + 1] =
      waypoints[trajGen_with_yaw_B.c + 4];
  }

  std::memset(&trajGen_with_yaw_B.constraints[0], 0, sizeof(real_T) << 5U);

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  trajGen_with_yaw_B.upperleft[1] = 0.0;
  trajGen_with_yaw_B.upperleft[2] = 0.0;
  trajGen_with_yaw_B.upperleft[3] = 0.0;
  trajGen_with_yaw_B.upperleft[5] = 0.0;
  trajGen_with_yaw_B.upperleft[6] = 0.0;
  trajGen_with_yaw_B.upperleft[7] = 0.0;
  trajGen_with_yaw_B.upperleft[9] = 0.0;
  trajGen_with_yaw_B.upperleft[10] = 0.0;
  trajGen_with_yaw_B.upperleft[11] = 0.0;
  trajGen_with_yaw_B.upperleft[13] = 0.0;
  trajGen_with_yaw_B.upperleft[14] = 0.0;
  trajGen_with_yaw_B.upperleft[15] = 0.0;
  for (trajGen_with_yaw_B.b_p_tmp = 0; trajGen_with_yaw_B.b_p_tmp < 2;
       trajGen_with_yaw_B.b_p_tmp++) {
    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.b_b = (trajGen_with_yaw_B.b_p_tmp << 2) + 1;
    trajGen_with_yaw_B.c = (trajGen_with_yaw_B.b_p_tmp + 1) << 2;
    if (trajGen_with_yaw_B.b_b > trajGen_with_yaw_B.c) {
      trajGen_with_yaw_B.e = 0;
      trajGen_with_yaw_B.c = 0;
    } else {
      trajGen_with_yaw_B.e = trajGen_with_yaw_B.b_b - 1;
    }

    trajGen_with_yaw_B.upperleft[0] =
      trajGen_with_yaw_B.b_p[trajGen_with_yaw_B.b_p_tmp];
    trajGen_with_yaw_B.upperleft[4] =
      trajGen_with_yaw_B.b_p[trajGen_with_yaw_B.b_p_tmp + 2];
    trajGen_with_yaw_B.upperleft[8] =
      trajGen_with_yaw_B.b_p[trajGen_with_yaw_B.b_p_tmp + 4];
    trajGen_with_yaw_B.upperleft[12] =
      trajGen_with_yaw_B.b_p[trajGen_with_yaw_B.b_p_tmp + 6];
    d = static_cast<int8_T>(trajGen_with_yaw_B.c - trajGen_with_yaw_B.e);
    trajGen_with_yaw_B.d_l[0] = d;
    trajGen_with_yaw_B.loop_ub_j = d;
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 4;
         trajGen_with_yaw_B.c++) {
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm <
           trajGen_with_yaw_B.loop_ub_j; trajGen_with_yaw_B.powerTerm++) {
        /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
        trajGen_with_yaw_B.constraints[(trajGen_with_yaw_B.e +
          trajGen_with_yaw_B.powerTerm) + (trajGen_with_yaw_B.c << 3)] =
          trajGen_with_yaw_B.upperleft[trajGen_with_yaw_B.d_l[0] *
          trajGen_with_yaw_B.c + trajGen_with_yaw_B.powerTerm];
      }
    }
  }

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  trajGen_with_yaw_B.tmp1 = timePoints[1] - timePoints[0];
  for (trajGen_with_yaw_B.b_p_tmp = 0; trajGen_with_yaw_B.b_p_tmp < 4;
       trajGen_with_yaw_B.b_p_tmp++) {
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 8;
         trajGen_with_yaw_B.c++) {
      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.x_tmp[trajGen_with_yaw_B.c] = !std::isnan
        (trajGen_with_yaw_B.constraints[(trajGen_with_yaw_B.b_p_tmp << 3) +
         trajGen_with_yaw_B.c]);
    }

    trajGen_with_yaw_B.nz = trajGen_with_yaw_B.x_tmp[0];

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    for (trajGen_with_yaw_B.loop_ub_j = 0; trajGen_with_yaw_B.loop_ub_j < 7;
         trajGen_with_yaw_B.loop_ub_j++) {
      trajGen_with_yaw_B.nz +=
        trajGen_with_yaw_B.x_tmp[trajGen_with_yaw_B.loop_ub_j + 1];
    }

    trajGen_with_yaw_B.prod = 1.0;
    std::memset(&trajGen_with_yaw_B.Q[0], 0, sizeof(real_T) << 6U);
    std::memset(&trajGen_with_yaw_B.Qsum[0], 0, sizeof(real_T) << 6U);
    for (trajGen_with_yaw_B.loop_ub_j = 0; trajGen_with_yaw_B.loop_ub_j < 8;
         trajGen_with_yaw_B.loop_ub_j++) {
      trajGen_with_yaw_B.e = 8 - trajGen_with_yaw_B.loop_ub_j;
      for (trajGen_with_yaw_B.b_b = 0; trajGen_with_yaw_B.b_b <
           trajGen_with_yaw_B.e; trajGen_with_yaw_B.b_b++) {
        trajGen_with_yaw_B.row = trajGen_with_yaw_B.loop_ub_j +
          trajGen_with_yaw_B.b_b;
        for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c <
             trajGen_with_yaw_B.e; trajGen_with_yaw_B.c++) {
          trajGen_with_yaw_B.col = trajGen_with_yaw_B.loop_ub_j +
            trajGen_with_yaw_B.c;
          for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm <
               trajGen_with_yaw_B.loop_ub_j; trajGen_with_yaw_B.powerTerm++) {
            /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
            trajGen_with_yaw_B.prod = static_cast<real_T>(trajGen_with_yaw_B.row
              - trajGen_with_yaw_B.powerTerm) * trajGen_with_yaw_B.prod *
              static_cast<real_T>(trajGen_with_yaw_B.col -
                                  trajGen_with_yaw_B.powerTerm);
          }

          trajGen_with_yaw_B.powerTerm = ((trajGen_with_yaw_B.row +
            trajGen_with_yaw_B.col) - (trajGen_with_yaw_B.loop_ub_j << 1)) + 1;

          /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
          trajGen_with_yaw_B.Q[trajGen_with_yaw_B.row + (trajGen_with_yaw_B.col <<
            3)] = trajGen_with_yaw_B.prod * trajGen_with_yaw_rt_powd_snf
            (trajGen_with_yaw_B.tmp1, static_cast<real_T>
             (trajGen_with_yaw_B.powerTerm)) / static_cast<real_T>
            (trajGen_with_yaw_B.powerTerm);
          trajGen_with_yaw_B.prod = 1.0;
        }
      }

      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.c = tmp[trajGen_with_yaw_B.loop_ub_j];
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm < 64;
           trajGen_with_yaw_B.powerTerm++) {
        /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
        trajGen_with_yaw_B.Qsum[trajGen_with_yaw_B.powerTerm] +=
          static_cast<real_T>(trajGen_with_yaw_B.c) *
          trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm];
        trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm] = 0.0;
      }
    }

    std::memset(&trajGen_with_yaw_B.A0[0], 0, sizeof(real_T) << 5U);
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 4;
         trajGen_with_yaw_B.c++) {
      trajGen_with_yaw_B.prod = 1.0;

      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm <
           trajGen_with_yaw_B.c; trajGen_with_yaw_B.powerTerm++) {
        trajGen_with_yaw_B.prod *= static_cast<real_T>(trajGen_with_yaw_B.c -
          trajGen_with_yaw_B.powerTerm);
      }

      trajGen_with_yaw_B.A0[trajGen_with_yaw_B.c + (trajGen_with_yaw_B.c << 2)] =
        trajGen_with_yaw_B.prod;
    }

    for (trajGen_with_yaw_B.b_b = 0; trajGen_with_yaw_B.b_b < 4;
         trajGen_with_yaw_B.b_b++) {
      for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 8;
           trajGen_with_yaw_B.c++) {
        trajGen_with_yaw_B.prod = 1.0;

        /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
        for (trajGen_with_yaw_B.loop_ub_j = 0; trajGen_with_yaw_B.loop_ub_j <
             trajGen_with_yaw_B.b_b; trajGen_with_yaw_B.loop_ub_j++) {
          trajGen_with_yaw_B.prod *= static_cast<real_T>(trajGen_with_yaw_B.c -
            trajGen_with_yaw_B.loop_ub_j);
        }

        trajGen_with_yaw_B.AT[trajGen_with_yaw_B.b_b + (trajGen_with_yaw_B.c <<
          2)] = trajGen_with_yaw_rt_powd_snf(trajGen_with_yaw_B.tmp1,
          static_cast<real_T>(trajGen_with_yaw_B.c - trajGen_with_yaw_B.b_b)) *
          trajGen_with_yaw_B.prod;
      }
    }

    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 8;
         trajGen_with_yaw_B.c++) {
      trajGen_with_yaw_B.loop_ub_j = trajGen_with_yaw_B.c << 2;
      trajGen_with_yaw_B.powerTerm = trajGen_with_yaw_B.c << 3;
      trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm] =
        trajGen_with_yaw_B.A0[trajGen_with_yaw_B.loop_ub_j];
      trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm + 4] =
        trajGen_with_yaw_B.AT[trajGen_with_yaw_B.loop_ub_j];
      trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm + 1] =
        trajGen_with_yaw_B.A0[trajGen_with_yaw_B.loop_ub_j + 1];
      trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm + 5] =
        trajGen_with_yaw_B.AT[trajGen_with_yaw_B.loop_ub_j + 1];
      trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm + 2] =
        trajGen_with_yaw_B.A0[trajGen_with_yaw_B.loop_ub_j + 2];
      trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm + 6] =
        trajGen_with_yaw_B.AT[trajGen_with_yaw_B.loop_ub_j + 2];
      trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm + 3] =
        trajGen_with_yaw_B.A0[trajGen_with_yaw_B.loop_ub_j + 3];
      trajGen_with_yaw_B.Q[trajGen_with_yaw_B.powerTerm + 7] =
        trajGen_with_yaw_B.AT[trajGen_with_yaw_B.loop_ub_j + 3];
    }

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.b_b = trajGen_with_yaw_B.b_p_tmp << 3;
    trajGen_with_yaw_constructM
      (&trajGen_with_yaw_B.constraints[trajGen_with_yaw_B.b_b],
       trajGen_with_yaw_B.M);
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 4;
         trajGen_with_yaw_B.c++) {
      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.loop_ub_j = trajGen_with_yaw_B.c << 3;
      trajGen_with_yaw_B.powerTerm = trajGen_with_yaw_B.c << 2;

      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.powerTerm] =
        trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j];
      trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.powerTerm + 1] =
        trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 1];
      trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.powerTerm + 2] =
        trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 2];
      trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.powerTerm + 3] =
        trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 3];
    }

    trajGen_with_yaw_inv(trajGen_with_yaw_B.Q_p, trajGen_with_yaw_B.upperleft);
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 4;
         trajGen_with_yaw_B.c++) {
      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.loop_ub_j = (trajGen_with_yaw_B.c + 4) << 3;
      trajGen_with_yaw_B.powerTerm = trajGen_with_yaw_B.c << 2;

      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.powerTerm] =
        trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 4];
      trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.powerTerm + 1] =
        trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 5];
      trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.powerTerm + 2] =
        trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 6];
      trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.powerTerm + 3] =
        trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 7];
    }

    trajGen_with_yaw_inv(trajGen_with_yaw_B.Q_p, trajGen_with_yaw_B.lowerright);

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 16;
         trajGen_with_yaw_B.c++) {
      trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.c] =
        -trajGen_with_yaw_B.lowerright[trajGen_with_yaw_B.c];
    }

    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 4;
         trajGen_with_yaw_B.c++) {
      trajGen_with_yaw_B.prod = trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.c];
      trajGen_with_yaw_B.lowerright_d =
        trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.c + 4];
      trajGen_with_yaw_B.lowerright_g =
        trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.c + 8];
      trajGen_with_yaw_B.lowerright_l =
        trajGen_with_yaw_B.Q_p[trajGen_with_yaw_B.c + 12];
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm < 4;
           trajGen_with_yaw_B.powerTerm++) {
        trajGen_with_yaw_B.loop_ub_j = trajGen_with_yaw_B.powerTerm << 3;
        trajGen_with_yaw_B.lowerright_c[trajGen_with_yaw_B.c +
          (trajGen_with_yaw_B.powerTerm << 2)] =
          ((trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 4] *
            trajGen_with_yaw_B.prod +
            trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 5] *
            trajGen_with_yaw_B.lowerright_d) +
           trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 6] *
           trajGen_with_yaw_B.lowerright_g) +
          trajGen_with_yaw_B.Q[trajGen_with_yaw_B.loop_ub_j + 7] *
          trajGen_with_yaw_B.lowerright_l;
      }

      trajGen_with_yaw_B.prod =
        trajGen_with_yaw_B.lowerright_c[trajGen_with_yaw_B.c + 4];
      trajGen_with_yaw_B.lowerright_d =
        trajGen_with_yaw_B.lowerright_c[trajGen_with_yaw_B.c];
      trajGen_with_yaw_B.lowerright_g =
        trajGen_with_yaw_B.lowerright_c[trajGen_with_yaw_B.c + 8];
      trajGen_with_yaw_B.lowerright_l =
        trajGen_with_yaw_B.lowerright_c[trajGen_with_yaw_B.c + 12];
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm < 4;
           trajGen_with_yaw_B.powerTerm++) {
        trajGen_with_yaw_B.loop_ub_j = trajGen_with_yaw_B.powerTerm << 2;
        trajGen_with_yaw_B.lowerright_f[trajGen_with_yaw_B.c +
          trajGen_with_yaw_B.loop_ub_j] =
          ((trajGen_with_yaw_B.upperleft[trajGen_with_yaw_B.loop_ub_j + 1] *
            trajGen_with_yaw_B.prod +
            trajGen_with_yaw_B.upperleft[trajGen_with_yaw_B.loop_ub_j] *
            trajGen_with_yaw_B.lowerright_d) +
           trajGen_with_yaw_B.upperleft[trajGen_with_yaw_B.loop_ub_j + 2] *
           trajGen_with_yaw_B.lowerright_g) +
          trajGen_with_yaw_B.upperleft[trajGen_with_yaw_B.loop_ub_j + 3] *
          trajGen_with_yaw_B.lowerright_l;
        trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.powerTerm +
          (trajGen_with_yaw_B.c << 3)] = trajGen_with_yaw_B.upperleft
          [(trajGen_with_yaw_B.c << 2) + trajGen_with_yaw_B.powerTerm];
        trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.powerTerm +
          ((trajGen_with_yaw_B.c + 4) << 3)] = 0.0;
      }
    }

    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 4;
         trajGen_with_yaw_B.c++) {
      trajGen_with_yaw_B.loop_ub_j = trajGen_with_yaw_B.c << 2;
      trajGen_with_yaw_B.powerTerm = trajGen_with_yaw_B.c << 3;
      trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.powerTerm + 4] =
        trajGen_with_yaw_B.lowerright_f[trajGen_with_yaw_B.loop_ub_j];
      trajGen_with_yaw_B.e = (trajGen_with_yaw_B.c + 4) << 3;
      trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.e + 4] =
        trajGen_with_yaw_B.lowerright[trajGen_with_yaw_B.loop_ub_j];
      trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.powerTerm + 5] =
        trajGen_with_yaw_B.lowerright_f[trajGen_with_yaw_B.loop_ub_j + 1];
      trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.e + 5] =
        trajGen_with_yaw_B.lowerright[trajGen_with_yaw_B.loop_ub_j + 1];
      trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.powerTerm + 6] =
        trajGen_with_yaw_B.lowerright_f[trajGen_with_yaw_B.loop_ub_j + 2];
      trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.e + 6] =
        trajGen_with_yaw_B.lowerright[trajGen_with_yaw_B.loop_ub_j + 2];
      trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.powerTerm + 7] =
        trajGen_with_yaw_B.lowerright_f[trajGen_with_yaw_B.loop_ub_j + 3];
      trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.e + 7] =
        trajGen_with_yaw_B.lowerright[trajGen_with_yaw_B.loop_ub_j + 3];
    }

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 8;
         trajGen_with_yaw_B.c++) {
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm < 8;
           trajGen_with_yaw_B.powerTerm++) {
        trajGen_with_yaw_B.e = trajGen_with_yaw_B.c << 3;
        trajGen_with_yaw_B.row = trajGen_with_yaw_B.powerTerm << 3;
        trajGen_with_yaw_B.col = trajGen_with_yaw_B.row + trajGen_with_yaw_B.c;
        trajGen_with_yaw_B.R_tmp[trajGen_with_yaw_B.powerTerm +
          trajGen_with_yaw_B.e] = trajGen_with_yaw_B.M[trajGen_with_yaw_B.col];
        trajGen_with_yaw_B.prod = 0.0;
        for (trajGen_with_yaw_B.loop_ub_j = 0; trajGen_with_yaw_B.loop_ub_j < 8;
             trajGen_with_yaw_B.loop_ub_j++) {
          trajGen_with_yaw_B.prod +=
            trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.e +
            trajGen_with_yaw_B.loop_ub_j] *
            trajGen_with_yaw_B.Qsum[trajGen_with_yaw_B.row +
            trajGen_with_yaw_B.loop_ub_j];
        }

        trajGen_with_yaw_B.AInv_k[trajGen_with_yaw_B.col] =
          trajGen_with_yaw_B.prod;
      }

      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm < 8;
           trajGen_with_yaw_B.powerTerm++) {
        trajGen_with_yaw_B.prod = 0.0;
        for (trajGen_with_yaw_B.loop_ub_j = 0; trajGen_with_yaw_B.loop_ub_j < 8;
             trajGen_with_yaw_B.loop_ub_j++) {
          trajGen_with_yaw_B.prod += trajGen_with_yaw_B.AInv_k
            [(trajGen_with_yaw_B.loop_ub_j << 3) + trajGen_with_yaw_B.c] *
            trajGen_with_yaw_B.AInv[(trajGen_with_yaw_B.powerTerm << 3) +
            trajGen_with_yaw_B.loop_ub_j];
        }

        trajGen_with_yaw_B.AInv_c[trajGen_with_yaw_B.c +
          (trajGen_with_yaw_B.powerTerm << 3)] = trajGen_with_yaw_B.prod;
      }
    }

    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 8;
         trajGen_with_yaw_B.c++) {
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm < 8;
           trajGen_with_yaw_B.powerTerm++) {
        trajGen_with_yaw_B.prod = 0.0;
        for (trajGen_with_yaw_B.loop_ub_j = 0; trajGen_with_yaw_B.loop_ub_j < 8;
             trajGen_with_yaw_B.loop_ub_j++) {
          trajGen_with_yaw_B.prod += trajGen_with_yaw_B.M
            [(trajGen_with_yaw_B.loop_ub_j << 3) + trajGen_with_yaw_B.c] *
            trajGen_with_yaw_B.AInv_c[(trajGen_with_yaw_B.powerTerm << 3) +
            trajGen_with_yaw_B.loop_ub_j];
        }

        trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.c +
          (trajGen_with_yaw_B.powerTerm << 3)] = trajGen_with_yaw_B.prod;
      }

      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm < 8;
           trajGen_with_yaw_B.powerTerm++) {
        trajGen_with_yaw_B.prod = 0.0;
        for (trajGen_with_yaw_B.loop_ub_j = 0; trajGen_with_yaw_B.loop_ub_j < 8;
             trajGen_with_yaw_B.loop_ub_j++) {
          trajGen_with_yaw_B.prod += trajGen_with_yaw_B.AInv
            [(trajGen_with_yaw_B.loop_ub_j << 3) + trajGen_with_yaw_B.c] *
            trajGen_with_yaw_B.R_tmp[(trajGen_with_yaw_B.powerTerm << 3) +
            trajGen_with_yaw_B.loop_ub_j];
        }

        trajGen_with_yaw_B.Qsum[trajGen_with_yaw_B.c +
          (trajGen_with_yaw_B.powerTerm << 3)] = trajGen_with_yaw_B.prod;
      }
    }

    if (static_cast<real_T>(trajGen_with_yaw_B.nz) + 1.0 > 8.0) {
      trajGen_with_yaw_B.e = 0;
      trajGen_with_yaw_B.row = 0;
      trajGen_with_yaw_B.col = 0;
      trajGen_with_yaw_B.loop_ub_j = -1;
    } else {
      trajGen_with_yaw_B.e = trajGen_with_yaw_B.nz;
      trajGen_with_yaw_B.row = trajGen_with_yaw_B.nz;
      trajGen_with_yaw_B.col = trajGen_with_yaw_B.nz;
      trajGen_with_yaw_B.loop_ub_j = 7;
    }

    trajGen_with_yaw_B.powerTerm = 0;
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 8;
         trajGen_with_yaw_B.c++) {
      if (trajGen_with_yaw_B.x_tmp[trajGen_with_yaw_B.c]) {
        trajGen_with_yaw_B.powerTerm++;
      }
    }

    trajGen_with_yaw_B.tmp_size_idx_0 = trajGen_with_yaw_B.powerTerm;
    trajGen_with_yaw_B.powerTerm = 0;
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 8;
         trajGen_with_yaw_B.c++) {
      if (trajGen_with_yaw_B.x_tmp[trajGen_with_yaw_B.c]) {
        trajGen_with_yaw_B.tmp_data_p[trajGen_with_yaw_B.powerTerm] =
          static_cast<int8_T>(trajGen_with_yaw_B.c);
        trajGen_with_yaw_B.powerTerm++;
      }
    }

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    if (trajGen_with_yaw_B.nz < 1) {
      trajGen_with_yaw_B.nz = 0;
    }

    trajGen_with_yaw_B.R_size[0] = trajGen_with_yaw_B.nz;
    trajGen_with_yaw_B.loop_ub_j -= trajGen_with_yaw_B.col;
    trajGen_with_yaw_B.R_size[1] = trajGen_with_yaw_B.loop_ub_j + 1;
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c <=
         trajGen_with_yaw_B.loop_ub_j; trajGen_with_yaw_B.c++) {
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm <
           trajGen_with_yaw_B.nz; trajGen_with_yaw_B.powerTerm++) {
        trajGen_with_yaw_B.M[trajGen_with_yaw_B.powerTerm +
          trajGen_with_yaw_B.nz * trajGen_with_yaw_B.c] =
          trajGen_with_yaw_B.Qsum[((trajGen_with_yaw_B.col +
          trajGen_with_yaw_B.c) << 3) + trajGen_with_yaw_B.powerTerm];
      }
    }

    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c <
         trajGen_with_yaw_B.tmp_size_idx_0; trajGen_with_yaw_B.c++) {
      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.b_p[trajGen_with_yaw_B.c] =
        trajGen_with_yaw_B.constraints[trajGen_with_yaw_B.b_b +
        trajGen_with_yaw_B.tmp_data_p[trajGen_with_yaw_B.c]];
    }

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.R_size_l[0] = trajGen_with_yaw_B.loop_ub_j + 1;
    trajGen_with_yaw_B.R_size_l[1] = trajGen_with_yaw_B.loop_ub_j + 1;
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c <=
         trajGen_with_yaw_B.loop_ub_j; trajGen_with_yaw_B.c++) {
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm <=
           trajGen_with_yaw_B.loop_ub_j; trajGen_with_yaw_B.powerTerm++) {
        /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
        trajGen_with_yaw_B.AInv[trajGen_with_yaw_B.powerTerm +
          (trajGen_with_yaw_B.loop_ub_j + 1) * trajGen_with_yaw_B.c] =
          -trajGen_with_yaw_B.Qsum[(((trajGen_with_yaw_B.row +
          trajGen_with_yaw_B.c) << 3) + trajGen_with_yaw_B.e) +
          trajGen_with_yaw_B.powerTerm];
      }
    }

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.R_data.set(&trajGen_with_yaw_B.M[0],
      trajGen_with_yaw_B.R_size[0], trajGen_with_yaw_B.R_size[1]);
    trajGen_with_yaw_B.constraints_data.set(&trajGen_with_yaw_B.b_p[0],
      trajGen_with_yaw_B.tmp_size_idx_0);
    trajGen_with_yaw_mtimes(trajGen_with_yaw_B.R_data,
      trajGen_with_yaw_B.constraints_data, trajGen_with_yaw_B.r2);
    trajGen_with_yaw_B.R_data_m.set(&trajGen_with_yaw_B.AInv[0],
      trajGen_with_yaw_B.R_size_l[0], trajGen_with_yaw_B.R_size_l[1]);

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_mldivide(trajGen_with_yaw_B.R_data_m, trajGen_with_yaw_B.r2,
      trajGen_with_yaw_B.r1);
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c <
         trajGen_with_yaw_B.tmp_size_idx_0; trajGen_with_yaw_B.c++) {
      trajGen_with_yaw_B.upperleft[trajGen_with_yaw_B.c] =
        trajGen_with_yaw_B.constraints[trajGen_with_yaw_B.b_b +
        trajGen_with_yaw_B.tmp_data_p[trajGen_with_yaw_B.c]];
    }

    trajGen_with_yaw_B.loop_ub_j = trajGen_with_yaw_B.r1.size(0);
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c <
         trajGen_with_yaw_B.loop_ub_j; trajGen_with_yaw_B.c++) {
      trajGen_with_yaw_B.upperleft[trajGen_with_yaw_B.c +
        trajGen_with_yaw_B.tmp_size_idx_0] =
        trajGen_with_yaw_B.r1[trajGen_with_yaw_B.c];
    }

    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 8;
         trajGen_with_yaw_B.c++) {
      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.prod = 0.0;
      for (trajGen_with_yaw_B.powerTerm = 0; trajGen_with_yaw_B.powerTerm < 8;
           trajGen_with_yaw_B.powerTerm++) {
        /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
        trajGen_with_yaw_B.prod += trajGen_with_yaw_B.R_tmp
          [(trajGen_with_yaw_B.powerTerm << 3) + trajGen_with_yaw_B.c] *
          trajGen_with_yaw_B.upperleft[trajGen_with_yaw_B.powerTerm];
      }

      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      trajGen_with_yaw_B.b_p[trajGen_with_yaw_B.c] = trajGen_with_yaw_B.prod;
    }

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_mldivide_gw(trajGen_with_yaw_B.Q, trajGen_with_yaw_B.b_p);
    for (trajGen_with_yaw_B.c = 0; trajGen_with_yaw_B.c < 8;
         trajGen_with_yaw_B.c++) {
      /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
      ppMatrix[trajGen_with_yaw_B.c + trajGen_with_yaw_B.b_b] =
        trajGen_with_yaw_B.b_p[7 - trajGen_with_yaw_B.c];
    }

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_B.nz = (trajGen_with_yaw_rcond(trajGen_with_yaw_B.Q) <
      1.0E-10);
  }

  toaZero.set_size(1, 2);

  /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  toaZero[0] = timePoints[0];
  toaZero[1] = timePoints[1];
  *maxIterStatus = 0.0;
  *maxTimeStatus = 0.0;
  *singularityStatus = trajGen_with_yaw_B.nz;
}

static void trajGen_with_y_binary_expand_op(real_T in1[2], const
  shared_uav_rst_sluav_internal_T *in2)
{
  /* Outputs for Enabled SubSystem: '<Root>/Subsystem' incorporates:
   *  EnablePort: '<S5>/Enable'
   */
  /* MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  in1[0] = in2->TimeOfArrZero[0] + in2->TimeOffset;
  in1[1] = in2->TimeOfArrZero[static_cast<int32_T>(in2->TimeOfArrZero.size(1) !=
    1)] + in2->TimeOffset;

  /* End of Outputs for SubSystem: '<Root>/Subsystem' */
}

/* Model step function */
void trajGen_with_yaw_step(void)
{
  SL_Bus_trajGen_with_yaw_std_msgs_Bool rtb_SourceBlock_o2_i_0;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T p;
  boolean_T p_0;
  boolean_T rEQ0;
  boolean_T rtb_FixPtRelationalOperator;

  /* Outputs for Atomic SubSystem: '<Root>/Subscribe1' */
  /* MATLABSystem: '<S4>/SourceBlock' */
  rEQ0 = Sub_trajGen_with_yaw_322.getLatestMessage(&rtb_SourceBlock_o2_i_0);

  /* Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S10>/Enable'
   */
  /* Start for MATLABSystem: '<S4>/SourceBlock' */
  if (rEQ0) {
    /* SignalConversion generated from: '<S10>/In1' */
    trajGen_with_yaw_B.In1_c = rtb_SourceBlock_o2_i_0;
  }

  /* End of Start for MATLABSystem: '<S4>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S4>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Subscribe1' */

  /* RelationalOperator: '<S1>/FixPt Relational Operator' incorporates:
   *  Constant: '<S8>/Constant'
   *  RelationalOperator: '<S8>/Compare'
   *  UnitDelay: '<S1>/Delay Input1'
   */
  rtb_FixPtRelationalOperator = (static_cast<int32_T>
    (trajGen_with_yaw_B.In1_c.Data) > static_cast<int32_T>
    (trajGen_with_yaw_DW.DelayInput1_DSTATE));

  /* Outputs for Atomic SubSystem: '<Root>/Subscribe' */
  /* MATLABSystem: '<S3>/SourceBlock' */
  rEQ0 = Sub_trajGen_with_yaw_321.getLatestMessage
    (&trajGen_with_yaw_B.rtb_SourceBlock_o2_p_g);

  /* Outputs for Enabled SubSystem: '<S3>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S9>/Enable'
   */
  /* Start for MATLABSystem: '<S3>/SourceBlock' */
  if (rEQ0) {
    /* SignalConversion generated from: '<S9>/In1' */
    trajGen_with_yaw_B.In1_h = trajGen_with_yaw_B.rtb_SourceBlock_o2_p_g;
  }

  /* End of Start for MATLABSystem: '<S3>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S3>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Subscribe' */

  /* Outputs for Atomic SubSystem: '<Root>/position reading' */
  /* MATLABSystem: '<S7>/SourceBlock' */
  rEQ0 = Sub_trajGen_with_yaw_377.getLatestMessage
    (&trajGen_with_yaw_B.rtb_SourceBlock_o2_c);

  /* Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S228>/Enable'
   */
  /* Start for MATLABSystem: '<S7>/SourceBlock' */
  if (rEQ0) {
    /* SignalConversion generated from: '<S228>/In1' */
    trajGen_with_yaw_B.In1 = trajGen_with_yaw_B.rtb_SourceBlock_o2_c;
  }

  /* End of Start for MATLABSystem: '<S7>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S7>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/position reading' */

  /* MATLABSystem: '<Root>/Get Parameter test' */
  ParamGet_trajGen_with_yaw_317.get_parameter(&trajGen_with_yaw_B.delT);

  /* Outputs for Enabled SubSystem: '<Root>/Subsystem' incorporates:
   *  EnablePort: '<S5>/Enable'
   */
  if (trajGen_with_yaw_B.In1_c.Data) {
    trajGen_with_yaw_DW.Subsystem_MODE = true;

    /* DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */
    if (rtb_FixPtRelationalOperator &&
        (trajGen_with_yaw_DW.DiscreteTimeIntegrator_PrevRese <= 0)) {
      trajGen_with_yaw_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
    }

    /* SignalConversion generated from: '<S5>/Vector Concatenate2' */
    trajGen_with_yaw_B.VectorConcatenate3[0] =
      trajGen_with_yaw_B.In1.Pose.Pose.Orientation.W;

    /* SignalConversion generated from: '<S5>/Vector Concatenate2' */
    trajGen_with_yaw_B.VectorConcatenate3[1] =
      trajGen_with_yaw_B.In1.Pose.Pose.Orientation.X;
    trajGen_with_yaw_B.VectorConcatenate3[2] =
      trajGen_with_yaw_B.In1.Pose.Pose.Orientation.Y;
    trajGen_with_yaw_B.VectorConcatenate3[3] =
      trajGen_with_yaw_B.In1.Pose.Pose.Orientation.Z;
    CoordinateTransformationConvers(trajGen_with_yaw_B.VectorConcatenate3,
      &trajGen_with_yaw_B.CoordinateTransformationConv_pn);

    /* SignalConversion generated from: '<S5>/Vector Concatenate' */
    trajGen_with_yaw_B.VectorConcatenate3[0] =
      trajGen_with_yaw_B.In1_h.Orientation.W;

    /* SignalConversion generated from: '<S5>/Vector Concatenate' */
    trajGen_with_yaw_B.VectorConcatenate3[1] =
      trajGen_with_yaw_B.In1_h.Orientation.X;
    trajGen_with_yaw_B.VectorConcatenate3[2] =
      trajGen_with_yaw_B.In1_h.Orientation.Y;
    trajGen_with_yaw_B.VectorConcatenate3[3] =
      trajGen_with_yaw_B.In1_h.Orientation.Z;
    CoordinateTransformationConvers(trajGen_with_yaw_B.VectorConcatenate3,
      &trajGen_with_yaw_B.CoordinateTransformationConve_p);

    /* MATLAB Function: '<S5>/MATLAB Function1' incorporates:
     *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
     *  MATLABSystem: '<S5>/Coordinate Transformation Conversion1'
     *  SignalConversion generated from: '<S5>/Vector Concatenate3'
     *  SignalConversion generated from: '<S5>/Coordinate Transformation Conversion1'
     */
    if (!trajGen_with_yaw_DW.startPosition_not_empty) {
      trajGen_with_yaw_DW.startPosition[0] =
        trajGen_with_yaw_B.In1.Pose.Pose.Position.X;
      trajGen_with_yaw_DW.startPosition[1] =
        trajGen_with_yaw_B.In1.Pose.Pose.Position.Y;
      trajGen_with_yaw_DW.startPosition[2] =
        trajGen_with_yaw_B.In1.Pose.Pose.Position.Z;
      trajGen_with_yaw_DW.startPosition[3] =
        trajGen_with_yaw_B.CoordinateTransformationConv_pn.CoordinateTransformationConve_p
        [0];
      trajGen_with_yaw_DW.startPosition_not_empty = true;
    }

    if (trajGen_with_yaw_DW.DiscreteTimeIntegrator_DSTATE == 0.0) {
      trajGen_with_yaw_DW.startPosition[0] =
        trajGen_with_yaw_B.In1.Pose.Pose.Position.X;
      trajGen_with_yaw_DW.startPosition[1] =
        trajGen_with_yaw_B.In1.Pose.Pose.Position.Y;
      trajGen_with_yaw_DW.startPosition[2] =
        trajGen_with_yaw_B.In1.Pose.Pose.Position.Z;
      trajGen_with_yaw_DW.startPosition[3] =
        trajGen_with_yaw_B.CoordinateTransformationConv_pn.CoordinateTransformationConve_p
        [0];
    }

    /* Math: '<S5>/Transpose' incorporates:
     *  MATLAB Function: '<S5>/MATLAB Function'
     *  MATLAB Function: '<S5>/MATLAB Function1'
     *  MATLABSystem: '<S5>/Coordinate Transformation Conversion'
     *  SignalConversion generated from: '<S5>/Vector Concatenate1'
     *  SignalConversion generated from: '<S5>/Coordinate Transformation Conversion'
     */
    trajGen_with_yaw_B.rtb_Transpose_tmp[0] = trajGen_with_yaw_DW.startPosition
      [0];
    trajGen_with_yaw_B.rtb_Transpose_tmp[4] =
      trajGen_with_yaw_B.In1_h.Position.X;
    trajGen_with_yaw_B.rtb_Transpose_tmp[1] = trajGen_with_yaw_DW.startPosition
      [1];
    trajGen_with_yaw_B.rtb_Transpose_tmp[5] =
      trajGen_with_yaw_B.In1_h.Position.Y;
    trajGen_with_yaw_B.rtb_Transpose_tmp[2] = trajGen_with_yaw_DW.startPosition
      [2];
    trajGen_with_yaw_B.rtb_Transpose_tmp[6] =
      trajGen_with_yaw_B.In1_h.Position.Z;
    trajGen_with_yaw_B.rtb_Transpose_tmp[3] = trajGen_with_yaw_DW.startPosition
      [3];
    trajGen_with_yaw_B.rtb_Transpose_tmp[7] =
      trajGen_with_yaw_B.CoordinateTransformationConve_p.CoordinateTransformationConve_p
      [0];

    /* MATLAB Function: '<S5>/MATLAB Function' incorporates:
     *  MATLAB Function: '<S5>/MATLAB Function1'
     *  MATLABSystem: '<Root>/Get Parameter test'
     *  MATLABSystem: '<S5>/Coordinate Transformation Conversion'
     *  SignalConversion generated from: '<S5>/Vector Concatenate1'
     *  SignalConversion generated from: '<S5>/Coordinate Transformation Conversion'
     * */
    trajGen_with_yaw_B.VectorConcatenate3[0] =
      trajGen_with_yaw_B.In1_h.Position.X - trajGen_with_yaw_DW.startPosition[0];
    trajGen_with_yaw_B.VectorConcatenate3[1] =
      trajGen_with_yaw_B.In1_h.Position.Y - trajGen_with_yaw_DW.startPosition[1];
    trajGen_with_yaw_B.VectorConcatenate3[2] =
      trajGen_with_yaw_B.In1_h.Position.Z - trajGen_with_yaw_DW.startPosition[2];
    trajGen_with_yaw_B.VectorConcatenate3[3] =
      trajGen_with_yaw_B.CoordinateTransformationConve_p.CoordinateTransformationConve_p
      [0] - trajGen_with_yaw_DW.startPosition[3];
    trajGen_with_yaw_B.timePoints[0] = 0.0 / trajGen_with_yaw_B.delT;
    trajGen_with_yaw_B.timePoints[1] = std::sqrt
      (((trajGen_with_yaw_B.VectorConcatenate3[0] *
         trajGen_with_yaw_B.VectorConcatenate3[0] +
         trajGen_with_yaw_B.VectorConcatenate3[1] *
         trajGen_with_yaw_B.VectorConcatenate3[1]) +
        trajGen_with_yaw_B.VectorConcatenate3[2] *
        trajGen_with_yaw_B.VectorConcatenate3[2]) +
       trajGen_with_yaw_B.VectorConcatenate3[3] *
       trajGen_with_yaw_B.VectorConcatenate3[3]) / trajGen_with_yaw_B.delT;
    trajGen_with_yaw_B.timePoints[1] += trajGen_with_yaw_B.timePoints[0];

    /* MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' incorporates:
     *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
     *  MATLAB Function: '<S5>/MATLAB Function1'
     *  MATLABSystem: '<S5>/Coordinate Transformation Conversion'
     *  Math: '<S5>/Transpose'
     *  SignalConversion generated from: '<S5>/Vector Concatenate1'
     *  SignalConversion generated from: '<S5>/Coordinate Transformation Conversion'
     */
    guard1 = false;
    if (trajGen_with_yaw_DW.obj.StartFlag) {
      guard1 = true;
    } else {
      rEQ0 = false;
      if (!trajGen_with_yaw_isequaln(trajGen_with_yaw_B.rtb_Transpose_tmp,
           trajGen_with_yaw_DW.obj.PrevWaypoint)) {
        rEQ0 = true;
      } else {
        p = false;
        p_0 = true;
        trajGen_with_yaw_B.b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (trajGen_with_yaw_B.b_k < 2)) {
          if ((trajGen_with_yaw_B.timePoints[trajGen_with_yaw_B.b_k] ==
               trajGen_with_yaw_DW.obj.PrevTimePoint[trajGen_with_yaw_B.b_k]) ||
              (std::isnan(trajGen_with_yaw_B.timePoints[trajGen_with_yaw_B.b_k])
               && std::isnan
               (trajGen_with_yaw_DW.obj.PrevTimePoint[trajGen_with_yaw_B.b_k])))
          {
            trajGen_with_yaw_B.b_k++;
          } else {
            p_0 = false;
            exitg1 = true;
          }
        }

        if (p_0) {
          p = true;
        }

        if (!p) {
          rEQ0 = true;
        } else {
          std::memset(&trajGen_with_yaw_B.dv[0], 0, sizeof(real_T) << 3U);
          if (!trajGen_with_yaw_isequaln(trajGen_with_yaw_B.dv,
               trajGen_with_yaw_DW.obj.PrevVelBC)) {
            rEQ0 = true;
          } else {
            std::memset(&trajGen_with_yaw_B.dv[0], 0, sizeof(real_T) << 3U);
            if (!trajGen_with_yaw_isequaln(trajGen_with_yaw_B.dv,
                 trajGen_with_yaw_DW.obj.PrevAccelBC)) {
              rEQ0 = true;
            } else {
              std::memset(&trajGen_with_yaw_B.dv[0], 0, sizeof(real_T) << 3U);
              if (!trajGen_with_yaw_isequaln(trajGen_with_yaw_B.dv,
                   trajGen_with_yaw_DW.obj.PrevJerkBC)) {
                rEQ0 = true;
              }
            }
          }
        }
      }

      if (rEQ0) {
        guard1 = true;
      }
    }

    if (guard1) {
      MinJerkPolyTraj_computePPandTim(&trajGen_with_yaw_DW.obj,
        trajGen_with_yaw_B.rtb_Transpose_tmp, trajGen_with_yaw_B.timePoints,
        trajGen_with_yaw_B.ppMatrix, trajGen_with_yaw_B.r,
        &trajGen_with_yaw_B.delT, &trajGen_with_yaw_B.error_x_body,
        &trajGen_with_yaw_B.Sum1);
      trajGen_with_yaw_DW.obj.SingularityStatus = trajGen_with_yaw_B.Sum1;
      trajGen_with_yaw_DW.obj.MaxTimeStatus = trajGen_with_yaw_B.error_x_body;
      trajGen_with_yaw_DW.obj.MaxIterStatus = trajGen_with_yaw_B.delT;
      trajGen_with_yaw_DW.obj.TimeOfArrZero.set_size(1,
        trajGen_with_yaw_B.r.size(1));
      trajGen_with_yaw_B.loop_ub = trajGen_with_yaw_B.r.size(0) *
        trajGen_with_yaw_B.r.size(1) - 1;
      for (trajGen_with_yaw_B.b_k = 0; trajGen_with_yaw_B.b_k <=
           trajGen_with_yaw_B.loop_ub; trajGen_with_yaw_B.b_k++) {
        trajGen_with_yaw_DW.obj.TimeOfArrZero[trajGen_with_yaw_B.b_k] =
          trajGen_with_yaw_B.r[trajGen_with_yaw_B.b_k];
      }

      trajGen_with_yaw_DW.obj.PPMatrix.set_size(1, 8, 4);
      for (trajGen_with_yaw_B.b_k = 0; trajGen_with_yaw_B.b_k < 32;
           trajGen_with_yaw_B.b_k++) {
        trajGen_with_yaw_DW.obj.PPMatrix[trajGen_with_yaw_B.b_k] =
          trajGen_with_yaw_B.ppMatrix[trajGen_with_yaw_B.b_k];
      }

      trajGen_with_yaw_DW.obj.StartFlag = false;
    }

    if (trajGen_with_yaw_DW.obj.TimeOfArrZero.size(1) == 2) {
      trajGen_with_yaw_B.loop_ub = trajGen_with_yaw_DW.obj.TimeOfArrZero.size(1);
      for (trajGen_with_yaw_B.b_k = 0; trajGen_with_yaw_B.b_k <
           trajGen_with_yaw_B.loop_ub; trajGen_with_yaw_B.b_k++) {
        trajGen_with_yaw_B.dv2[trajGen_with_yaw_B.b_k] =
          trajGen_with_yaw_DW.obj.TimeOfArrZero[trajGen_with_yaw_B.b_k] +
          trajGen_with_yaw_DW.obj.TimeOffset;
      }
    } else {
      trajGen_with_y_binary_expand_op(trajGen_with_yaw_B.dv2,
        &trajGen_with_yaw_DW.obj);
    }

    if (trajGen_with_yaw_DW.DiscreteTimeIntegrator_DSTATE <
        trajGen_with_yaw_B.dv2[0]) {
      trajGen_with_yaw_B.VectorConcatenate3[0] =
        trajGen_with_yaw_DW.startPosition[0];
      trajGen_with_yaw_B.VectorConcatenate3[1] =
        trajGen_with_yaw_DW.startPosition[1];
      trajGen_with_yaw_B.VectorConcatenate3[2] =
        trajGen_with_yaw_DW.startPosition[2];
      trajGen_with_yaw_B.VectorConcatenate3[3] =
        trajGen_with_yaw_DW.startPosition[3];
    } else if (trajGen_with_yaw_DW.DiscreteTimeIntegrator_DSTATE >
               trajGen_with_yaw_B.dv2[1]) {
      trajGen_with_yaw_B.VectorConcatenate3[0] =
        trajGen_with_yaw_B.In1_h.Position.X;
      trajGen_with_yaw_B.VectorConcatenate3[1] =
        trajGen_with_yaw_B.In1_h.Position.Y;
      trajGen_with_yaw_B.VectorConcatenate3[2] =
        trajGen_with_yaw_B.In1_h.Position.Z;
      trajGen_with_yaw_B.VectorConcatenate3[3] =
        trajGen_with_yaw_B.CoordinateTransformationConve_p.CoordinateTransformationConve_p
        [0];
    } else {
      trajGen_with_yaw_B.delT =
        trajGen_with_yaw_DW.DiscreteTimeIntegrator_DSTATE -
        trajGen_with_yaw_B.dv2[0];
      for (trajGen_with_yaw_B.b_k = 0; trajGen_with_yaw_B.b_k < 4;
           trajGen_with_yaw_B.b_k++) {
        if (std::isnan(trajGen_with_yaw_B.delT)) {
          trajGen_with_yaw_B.error_x_body = (rtNaN);
        } else {
          trajGen_with_yaw_B.error_x_body = trajGen_with_yaw_DW.obj.PPMatrix
            [(trajGen_with_yaw_DW.obj.PPMatrix.size(0) << 3) *
            trajGen_with_yaw_B.b_k];
          for (trajGen_with_yaw_B.loop_ub = 0; trajGen_with_yaw_B.loop_ub < 7;
               trajGen_with_yaw_B.loop_ub++) {
            trajGen_with_yaw_B.error_x_body = trajGen_with_yaw_DW.obj.PPMatrix
              [(trajGen_with_yaw_B.loop_ub + 1) *
              trajGen_with_yaw_DW.obj.PPMatrix.size(0) +
              (trajGen_with_yaw_DW.obj.PPMatrix.size(0) << 3) *
              trajGen_with_yaw_B.b_k] + trajGen_with_yaw_B.delT *
              trajGen_with_yaw_B.error_x_body;
          }
        }

        trajGen_with_yaw_B.VectorConcatenate3[trajGen_with_yaw_B.b_k] =
          trajGen_with_yaw_B.error_x_body;
      }
    }

    trajGen_with_yaw_DW.obj.PrevTimePoint[0] = trajGen_with_yaw_B.timePoints[0];
    trajGen_with_yaw_DW.obj.PrevTimePoint[1] = trajGen_with_yaw_B.timePoints[1];
    for (trajGen_with_yaw_B.b_k = 0; trajGen_with_yaw_B.b_k < 8;
         trajGen_with_yaw_B.b_k++) {
      trajGen_with_yaw_DW.obj.PrevWaypoint[trajGen_with_yaw_B.b_k] =
        trajGen_with_yaw_B.rtb_Transpose_tmp[trajGen_with_yaw_B.b_k];
      trajGen_with_yaw_DW.obj.PrevVelBC[trajGen_with_yaw_B.b_k] = 0.0;
      trajGen_with_yaw_DW.obj.PrevAccelBC[trajGen_with_yaw_B.b_k] = 0.0;
      trajGen_with_yaw_DW.obj.PrevJerkBC[trajGen_with_yaw_B.b_k] = 0.0;
    }

    /* Sum: '<S5>/Sum' incorporates:
     *  MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory'
     */
    trajGen_with_yaw_B.delT = trajGen_with_yaw_B.VectorConcatenate3[0] -
      trajGen_with_yaw_B.In1.Pose.Pose.Position.X;

    /* Sum: '<S5>/Sum1' incorporates:
     *  MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory'
     */
    trajGen_with_yaw_B.Sum1 = trajGen_with_yaw_B.VectorConcatenate3[1] -
      trajGen_with_yaw_B.In1.Pose.Pose.Position.Y;

    /* MATLAB Function: '<S5>/ convert to body frame' incorporates:
     *  MATLABSystem: '<S5>/Coordinate Transformation Conversion1'
     *  SignalConversion generated from: '<S5>/Coordinate Transformation Conversion1'
     */
    trajGen_with_yaw_B.Filter_j = std::sin
      (-trajGen_with_yaw_B.CoordinateTransformationConv_pn.CoordinateTransformationConve_p
       [0]);
    trajGen_with_yaw_B.Sum3 = std::cos
      (-trajGen_with_yaw_B.CoordinateTransformationConv_pn.CoordinateTransformationConve_p
       [0]);
    trajGen_with_yaw_B.error_x_body = trajGen_with_yaw_B.Sum3 *
      trajGen_with_yaw_B.delT - trajGen_with_yaw_B.Filter_j *
      trajGen_with_yaw_B.Sum1;
    trajGen_with_yaw_B.Sum1 = trajGen_with_yaw_B.Filter_j *
      trajGen_with_yaw_B.delT + trajGen_with_yaw_B.Sum3 *
      trajGen_with_yaw_B.Sum1;

    /* Gain: '<S109>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S101>/Filter'
     *  Gain: '<S99>/Derivative Gain'
     *  Sum: '<S101>/SumD'
     */
    trajGen_with_yaw_B.delT = (0.08 * trajGen_with_yaw_B.Sum1 -
      trajGen_with_yaw_DW.Filter_DSTATE) * 3.0;

    /* Gain: '<S5>/Gain3' incorporates:
     *  Gain: '<S111>/Proportional Gain'
     *  Sum: '<S115>/Sum'
     */
    trajGen_with_yaw_B.Gain3 = -(0.7 * trajGen_with_yaw_B.Sum1 +
      trajGen_with_yaw_B.delT);

    /* Gain: '<S57>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S49>/Filter'
     *  Gain: '<S47>/Derivative Gain'
     *  Sum: '<S49>/SumD'
     */
    trajGen_with_yaw_B.Filter_j = (0.08 * trajGen_with_yaw_B.error_x_body -
      trajGen_with_yaw_DW.Filter_DSTATE_j) * 3.0;

    /* Sum: '<S63>/Sum' incorporates:
     *  Gain: '<S59>/Proportional Gain'
     */
    trajGen_with_yaw_B.Sum = 0.7 * trajGen_with_yaw_B.error_x_body +
      trajGen_with_yaw_B.Filter_j;

    /* Product: '<S68>/Ungain*Ts Prod Out' */
    trajGen_with_yaw_B.error_x_body = trajGen_with_yaw_B.Filter_j * 0.0;

    /* Sum: '<S5>/Sum2' incorporates:
     *  MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory'
     */
    trajGen_with_yaw_B.Filter_j = trajGen_with_yaw_B.VectorConcatenate3[2] -
      trajGen_with_yaw_B.In1.Pose.Pose.Position.Z;

    /* Gain: '<S213>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S205>/Filter'
     *  Gain: '<S203>/Derivative Gain'
     *  Sum: '<S205>/SumD'
     */
    trajGen_with_yaw_B.Sum1 = (0.08 * trajGen_with_yaw_B.Filter_j -
      trajGen_with_yaw_DW.Filter_DSTATE_b) * 3.0;

    /* Gain: '<S5>/Gain1' incorporates:
     *  Gain: '<S215>/Proportional Gain'
     *  Sum: '<S219>/Sum'
     */
    trajGen_with_yaw_B.Gain1 = -(0.7 * trajGen_with_yaw_B.Filter_j +
      trajGen_with_yaw_B.Sum1);

    /* Sum: '<S5>/Sum3' incorporates:
     *  MATLABSystem: '<S5>/Coordinate Transformation Conversion1'
     *  MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory'
     *  SignalConversion generated from: '<S5>/Coordinate Transformation Conversion1'
     */
    trajGen_with_yaw_B.Sum3 = trajGen_with_yaw_B.VectorConcatenate3[3] -
      trajGen_with_yaw_B.CoordinateTransformationConv_pn.CoordinateTransformationConve_p
      [0];

    /* MATLAB Function: '<S5>/MATLAB Function3' */
    if (std::isnan(trajGen_with_yaw_B.Sum3 + 3.1415926535897931) || std::isinf
        (trajGen_with_yaw_B.Sum3 + 3.1415926535897931)) {
      trajGen_with_yaw_B.Filter_j = (rtNaN);
    } else if (trajGen_with_yaw_B.Sum3 + 3.1415926535897931 == 0.0) {
      trajGen_with_yaw_B.Filter_j = 0.0;
    } else {
      trajGen_with_yaw_B.Filter_j = std::fmod(trajGen_with_yaw_B.Sum3 +
        3.1415926535897931, 6.2831853071795862);
      rEQ0 = (trajGen_with_yaw_B.Filter_j == 0.0);
      if (!rEQ0) {
        trajGen_with_yaw_B.Sum3 = std::abs((trajGen_with_yaw_B.Sum3 +
          3.1415926535897931) / 6.2831853071795862);
        rEQ0 = !(std::abs(trajGen_with_yaw_B.Sum3 - std::floor
                          (trajGen_with_yaw_B.Sum3 + 0.5)) >
                 2.2204460492503131E-16 * trajGen_with_yaw_B.Sum3);
      }

      if (rEQ0) {
        trajGen_with_yaw_B.Filter_j = 0.0;
      } else if (trajGen_with_yaw_B.Filter_j < 0.0) {
        trajGen_with_yaw_B.Filter_j += 6.2831853071795862;
      }
    }

    /* Gain: '<S161>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S153>/Filter'
     *  Gain: '<S151>/Derivative Gain'
     *  MATLAB Function: '<S5>/MATLAB Function3'
     *  Sum: '<S153>/SumD'
     */
    trajGen_with_yaw_B.Sum3 = ((trajGen_with_yaw_B.Filter_j - 3.1415926535897931)
      * 0.08 - trajGen_with_yaw_DW.Filter_DSTATE_d) * 3.0;

    /* Gain: '<S5>/Gain2' incorporates:
     *  Gain: '<S15>/Gain'
     *  Gain: '<S163>/Proportional Gain'
     *  MATLAB Function: '<S5>/MATLAB Function3'
     *  Sum: '<S167>/Sum'
     */
    trajGen_with_yaw_B.Gain2 = -(((trajGen_with_yaw_B.Filter_j -
      3.1415926535897931) * 0.8 + trajGen_with_yaw_B.Sum3) * 57.295779513082323);

    /* Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S5>/Constant1'
     */
    trajGen_with_yaw_DW.DiscreteTimeIntegrator_DSTATE += 0.01;
    trajGen_with_yaw_DW.DiscreteTimeIntegrator_PrevRese = static_cast<int8_T>
      (rtb_FixPtRelationalOperator);

    /* Update for DiscreteIntegrator: '<S101>/Filter' incorporates:
     *  Product: '<S120>/Ungain*Ts Prod Out'
     */
    trajGen_with_yaw_DW.Filter_DSTATE += trajGen_with_yaw_B.delT * 0.0;

    /* Update for DiscreteIntegrator: '<S49>/Filter' */
    trajGen_with_yaw_DW.Filter_DSTATE_j += trajGen_with_yaw_B.error_x_body;

    /* Update for DiscreteIntegrator: '<S205>/Filter' incorporates:
     *  Product: '<S224>/Ungain*Ts Prod Out'
     */
    trajGen_with_yaw_DW.Filter_DSTATE_b += trajGen_with_yaw_B.Sum1 * 0.0;

    /* Update for DiscreteIntegrator: '<S153>/Filter' incorporates:
     *  Product: '<S172>/Ungain*Ts Prod Out'
     */
    trajGen_with_yaw_DW.Filter_DSTATE_d += trajGen_with_yaw_B.Sum3 * 0.0;
  } else if (trajGen_with_yaw_DW.Subsystem_MODE) {
    /* Disable for Sum: '<S63>/Sum' incorporates:
     *  Outport: '<S5>/x'
     */
    trajGen_with_yaw_B.Sum = 0.0;

    /* Disable for Gain: '<S5>/Gain3' incorporates:
     *  Outport: '<S5>/y'
     */
    trajGen_with_yaw_B.Gain3 = 0.0;

    /* Disable for Gain: '<S5>/Gain1' incorporates:
     *  Outport: '<S5>/z'
     */
    trajGen_with_yaw_B.Gain1 = 0.0;

    /* Disable for Gain: '<S5>/Gain2' incorporates:
     *  Outport: '<S5>/yaw'
     */
    trajGen_with_yaw_B.Gain2 = 0.0;
    trajGen_with_yaw_DW.Subsystem_MODE = false;
  }

  /* End of Outputs for SubSystem: '<Root>/Subsystem' */

  /* BusAssignment: '<Root>/Bus Assignment' */
  std::memset(&trajGen_with_yaw_B.BusAssignment, 0, sizeof
              (SL_Bus_trajGen_with_yaw_geometry_msgs_Twist));
  trajGen_with_yaw_B.BusAssignment.Linear.X = trajGen_with_yaw_B.Sum;
  trajGen_with_yaw_B.BusAssignment.Linear.Y = trajGen_with_yaw_B.Gain3;
  trajGen_with_yaw_B.BusAssignment.Linear.Z = trajGen_with_yaw_B.Gain1;
  trajGen_with_yaw_B.BusAssignment.Angular.Z = trajGen_with_yaw_B.Gain2;

  /* Outputs for Atomic SubSystem: '<Root>/Publish' */
  /* MATLABSystem: '<S2>/SinkBlock' */
  Pub_trajGen_with_yaw_319.publish(&trajGen_with_yaw_B.BusAssignment);

  /* End of Outputs for SubSystem: '<Root>/Publish' */

  /* Update for UnitDelay: '<S1>/Delay Input1' incorporates:
   *  Constant: '<S8>/Constant'
   *  RelationalOperator: '<S8>/Compare'
   */
  trajGen_with_yaw_DW.DelayInput1_DSTATE = trajGen_with_yaw_B.In1_c.Data;
}

/* Model initialize function */
void trajGen_with_yaw_initialize(void)
{
  /* Registration code */

  /* block I/O */
  (void) std::memset((static_cast<void *>(&trajGen_with_yaw_B)), 0,
                     sizeof(B_trajGen_with_yaw_T));

  /* states (dwork) */
  (void) std::memset(static_cast<void *>(&trajGen_with_yaw_DW), 0,
                     sizeof(DW_trajGen_with_yaw_T));

  {
    static const char_T b_zeroDelimTopic_2[23]{ "/tagslam/odom/body_rig" };

    static const char_T b_zeroDelimTopic_3[17]{ "/command/cmd_vel" };

    static const char_T b_zeroDelimTopic_0[16]{ "/trajGen/enable" };

    static const char_T b_zeroDelimName_0[14]{ "/trajGen/velo" };

    static const char_T b_zeroDelimTopic_1[12]{ "/chase_pose" };

    real_T tmp[2];
    int32_T i;
    char_T b_zeroDelimName[14];
    char_T b_zeroDelimTopic[12];

    /* Start for Atomic SubSystem: '<Root>/Subscribe1' */
    /* Start for MATLABSystem: '<S4>/SourceBlock' */
    trajGen_with_yaw_DW.obj_m.matlabCodegenIsDeleted = false;
    trajGen_with_yaw_DW.objisempty_c = true;
    trajGen_with_yaw_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      trajGen_with_yaw_B.b_zeroDelimTopic_n[i] = b_zeroDelimTopic_0[i];
    }

    Sub_trajGen_with_yaw_322.createSubscriber
      (&trajGen_with_yaw_B.b_zeroDelimTopic_n[0], 1);
    trajGen_with_yaw_DW.obj_m.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S4>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/Subscribe1' */

    /* Start for Atomic SubSystem: '<Root>/Subscribe' */
    /* Start for MATLABSystem: '<S3>/SourceBlock' */
    trajGen_with_yaw_DW.obj_kx.matlabCodegenIsDeleted = false;
    trajGen_with_yaw_DW.objisempty_lg = true;
    trajGen_with_yaw_DW.obj_kx.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      b_zeroDelimTopic[i] = b_zeroDelimTopic_1[i];
    }

    Sub_trajGen_with_yaw_321.createSubscriber(&b_zeroDelimTopic[0], 1);
    trajGen_with_yaw_DW.obj_kx.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S3>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/Subscribe' */

    /* Start for Atomic SubSystem: '<Root>/position reading' */
    /* Start for MATLABSystem: '<S7>/SourceBlock' */
    trajGen_with_yaw_DW.obj_k.matlabCodegenIsDeleted = false;
    trajGen_with_yaw_DW.objisempty = true;
    trajGen_with_yaw_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      trajGen_with_yaw_B.b_zeroDelimTopic[i] = b_zeroDelimTopic_2[i];
    }

    Sub_trajGen_with_yaw_377.createSubscriber
      (&trajGen_with_yaw_B.b_zeroDelimTopic[0], 1);
    trajGen_with_yaw_DW.obj_k.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S7>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/position reading' */

    /* Start for MATLABSystem: '<Root>/Get Parameter test' */
    trajGen_with_yaw_DW.obj_o.matlabCodegenIsDeleted = false;
    trajGen_with_yaw_DW.objisempty_f = true;
    trajGen_with_yaw_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      b_zeroDelimName[i] = b_zeroDelimName_0[i];
    }

    ParamGet_trajGen_with_yaw_317.initialize(&b_zeroDelimName[0]);
    ParamGet_trajGen_with_yaw_317.initialize_error_codes(0, 1, 2, 3);
    ParamGet_trajGen_with_yaw_317.set_initial_value(0.0);
    trajGen_with_yaw_DW.obj_o.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<Root>/Get Parameter test' */

    /* Start for Enabled SubSystem: '<Root>/Subsystem' */
    trajGen_with_yaw_DW.Subsystem_MODE = false;
    CoordinateTransformationC_Start
      (&trajGen_with_yaw_DW.CoordinateTransformationConv_pn);
    CoordinateTransformationC_Start
      (&trajGen_with_yaw_DW.CoordinateTransformationConve_p);

    /* Start for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
    trajGen_with_yaw_DW.objisempty_l = true;
    trajGen_with_yaw_DW.obj.isInitialized = 1;
    std::memset(&trajGen_with_yaw_B.dv1[0], 0, sizeof(real_T) << 3U);
    tmp[0] = 0.0;
    tmp[1] = 0.0;
    trajG_MinJerkPolyTraj_setupImpl(&trajGen_with_yaw_DW.obj,
      trajGen_with_yaw_B.dv1, tmp);

    /* End of Start for SubSystem: '<Root>/Subsystem' */

    /* Start for Atomic SubSystem: '<Root>/Publish' */
    /* Start for MATLABSystem: '<S2>/SinkBlock' */
    trajGen_with_yaw_DW.obj_b.matlabCodegenIsDeleted = false;
    trajGen_with_yaw_DW.objisempty_a = true;
    trajGen_with_yaw_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      trajGen_with_yaw_B.b_zeroDelimTopic_m[i] = b_zeroDelimTopic_3[i];
    }

    Pub_trajGen_with_yaw_319.createPublisher
      (&trajGen_with_yaw_B.b_zeroDelimTopic_m[0], 1);
    trajGen_with_yaw_DW.obj_b.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S2>/SinkBlock' */
    /* End of Start for SubSystem: '<Root>/Publish' */
  }

  /* InitializeConditions for UnitDelay: '<S1>/Delay Input1' */
  trajGen_with_yaw_DW.DelayInput1_DSTATE = false;

  /* SystemInitialize for SignalConversion generated from: '<S10>/In1' */
  std::memset(&trajGen_with_yaw_B.In1_c, 0, sizeof
              (SL_Bus_trajGen_with_yaw_std_msgs_Bool));

  /* SystemInitialize for SignalConversion generated from: '<S9>/In1' */
  std::memset(&trajGen_with_yaw_B.In1_h, 0, sizeof
              (SL_Bus_trajGen_with_yaw_geometry_msgs_Pose));

  /* SystemInitialize for Enabled SubSystem: '<Root>/Subsystem' */
  /* InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */
  trajGen_with_yaw_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
  trajGen_with_yaw_DW.DiscreteTimeIntegrator_PrevRese = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S101>/Filter' */
  trajGen_with_yaw_DW.Filter_DSTATE = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S49>/Filter' */
  trajGen_with_yaw_DW.Filter_DSTATE_j = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S205>/Filter' */
  trajGen_with_yaw_DW.Filter_DSTATE_b = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S153>/Filter' */
  trajGen_with_yaw_DW.Filter_DSTATE_d = 0.0;

  /* SystemInitialize for MATLAB Function: '<S5>/MATLAB Function1' */
  trajGen_with_yaw_DW.startPosition_not_empty = false;

  /* InitializeConditions for MATLABSystem: '<S5>/Minimum Jerk Polynomial Trajectory' */
  trajGen_with_yaw_DW.obj.StartFlag = true;

  /* SystemInitialize for Sum: '<S63>/Sum' incorporates:
   *  Outport: '<S5>/x'
   */
  trajGen_with_yaw_B.Sum = 0.0;

  /* SystemInitialize for Gain: '<S5>/Gain3' incorporates:
   *  Outport: '<S5>/y'
   */
  trajGen_with_yaw_B.Gain3 = 0.0;

  /* SystemInitialize for Gain: '<S5>/Gain1' incorporates:
   *  Outport: '<S5>/z'
   */
  trajGen_with_yaw_B.Gain1 = 0.0;

  /* SystemInitialize for Gain: '<S5>/Gain2' incorporates:
   *  Outport: '<S5>/yaw'
   */
  trajGen_with_yaw_B.Gain2 = 0.0;

  /* End of SystemInitialize for SubSystem: '<Root>/Subsystem' */

  /* SystemInitialize for SignalConversion generated from: '<S228>/In1' */
  std::memset(&trajGen_with_yaw_B.In1, 0, sizeof
              (SL_Bus_trajGen_with_yaw_nav_msgs_Odometry));
}

/* Model terminate function */
void trajGen_with_yaw_terminate(void)
{
  /* Terminate for Atomic SubSystem: '<Root>/Subscribe1' */
  /* Terminate for MATLABSystem: '<S4>/SourceBlock' */
  if (!trajGen_with_yaw_DW.obj_m.matlabCodegenIsDeleted) {
    trajGen_with_yaw_DW.obj_m.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S4>/SourceBlock' */
  /* End of Terminate for SubSystem: '<Root>/Subscribe1' */

  /* Terminate for Atomic SubSystem: '<Root>/Subscribe' */
  /* Terminate for MATLABSystem: '<S3>/SourceBlock' */
  if (!trajGen_with_yaw_DW.obj_kx.matlabCodegenIsDeleted) {
    trajGen_with_yaw_DW.obj_kx.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S3>/SourceBlock' */
  /* End of Terminate for SubSystem: '<Root>/Subscribe' */

  /* Terminate for Atomic SubSystem: '<Root>/position reading' */
  /* Terminate for MATLABSystem: '<S7>/SourceBlock' */
  if (!trajGen_with_yaw_DW.obj_k.matlabCodegenIsDeleted) {
    trajGen_with_yaw_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S7>/SourceBlock' */
  /* End of Terminate for SubSystem: '<Root>/position reading' */

  /* Terminate for MATLABSystem: '<Root>/Get Parameter test' */
  if (!trajGen_with_yaw_DW.obj_o.matlabCodegenIsDeleted) {
    trajGen_with_yaw_DW.obj_o.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<Root>/Get Parameter test' */

  /* Terminate for Atomic SubSystem: '<Root>/Publish' */
  /* Terminate for MATLABSystem: '<S2>/SinkBlock' */
  if (!trajGen_with_yaw_DW.obj_b.matlabCodegenIsDeleted) {
    trajGen_with_yaw_DW.obj_b.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S2>/SinkBlock' */
  /* End of Terminate for SubSystem: '<Root>/Publish' */
}
