/*
 * trajGen.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "trajGen".
 *
 * Model version              : 1.23
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C++ source code generated on : Mon Nov 24 01:30:18 2025
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "trajGen.h"
#include "trajGen_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstring>
#include <cmath>
#include "trajGen_private.h"
#include "cmath"
#include "rt_defines.h"

/* Block signals (default storage) */
B_trajGen_T trajGen_B;

/* Block states (default storage) */
DW_trajGen_T trajGen_DW;

/* Real-time model */
RT_MODEL_trajGen_T trajGen_M_{ };

RT_MODEL_trajGen_T *const trajGen_M{ &trajGen_M_ };

/* Forward declaration for local functions */
static void trajG_MinJerkPolyTraj_setupImpl(shared_uav_rst_sluav_internal_T *obj,
  const real_T waypoints[6], const real_T timePoints[2]);
static boolean_T trajGen_isequaln(const real_T varargin_1[6], const real_T
  varargin_2[6]);
static void trajGen_eml_find(const boolean_T x[8], int32_T i_data[], int32_T
  i_size[1]);
static void trajGen_constructM(const real_T constraints[8], real_T M[64]);
static void trajGen_inv(const real_T x[16], real_T y[16]);
static void trajGen_mtimes(const coder::array<real_T, 2U> &A, const coder::array<
  real_T, 1U> &B, coder::array<real_T, 1U> &C);
static real_T trajGen_xnrm2(int32_T n, const coder::array<real_T, 2U> &x,
  int32_T ix0);
static real_T trajGen_rt_hypotd_snf(real_T u0, real_T u1);
static void trajGen_xgeqp3(const coder::array<real_T, 2U> &A, coder::array<
  real_T, 2U> &b_A, coder::array<real_T, 1U> &tau, coder::array<int32_T, 2U>
  &jpvt);
static int32_T trajGen_rankFromQR(const coder::array<real_T, 2U> &A);
static void trajGen_xzgetrf(int32_T m, int32_T n, coder::array<real_T, 2U> &A,
  int32_T lda, coder::array<int32_T, 2U> &ipiv, int32_T *info);
static void trajGen_mldivide(const coder::array<real_T, 2U> &A, const coder::
  array<real_T, 1U> &B, coder::array<real_T, 1U> &Y);
static void trajGen_xzgetrf_p(real_T A[64], int32_T ipiv[8], int32_T *info);
static void trajGen_mldivide_pv(const real_T A[64], real_T B[8]);
static real_T trajGen_rcond(const real_T A[64]);
static void MinJerkPolyTraj_computePPandTim(shared_uav_rst_sluav_internal_T *obj,
  const real_T waypoints[6], const real_T timePoints[2], real_T ppMatrix[24],
  coder::array<real_T, 2U> &toaZero, real_T *maxIterStatus, real_T
  *maxTimeStatus, real_T *singularityStatus);
static void trajGen_binary_expand_op(real_T in1[2], const
  shared_uav_rst_sluav_internal_T *in2);
static void trajG_MinJerkPolyTraj_setupImpl(shared_uav_rst_sluav_internal_T *obj,
  const real_T waypoints[6], const real_T timePoints[2])
{
  obj->PPMatrix.set_size(3, 8, 1);
  for (int32_T i{0}; i < 24; i++) {
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
  for (int32_T i{0}; i < 6; i++) {
    obj->PrevWaypoint[i] = waypoints[i];

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    obj->PrevVelBC[i] = 0.0;
    obj->PrevAccelBC[i] = 0.0;
    obj->PrevJerkBC[i] = 0.0;
  }
}

static boolean_T trajGen_isequaln(const real_T varargin_1[6], const real_T
  varargin_2[6])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 6)) {
    if ((varargin_1[b_k] == varargin_2[b_k]) || (std::isnan(varargin_1[b_k]) &&
         std::isnan(varargin_2[b_k]))) {
      b_k++;
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

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (std::isinf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
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
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
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

static void trajGen_eml_find(const boolean_T x[8], int32_T i_data[], int32_T
  i_size[1])
{
  int32_T b_ii;
  int32_T idx;
  boolean_T exitg1;
  idx = 0;
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 < 8)) {
    if (x[b_ii - 1]) {
      idx++;
      i_data[idx - 1] = b_ii;
      if (idx >= 8) {
        exitg1 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  if (idx < 1) {
    i_size[0] = 0;
  } else {
    i_size[0] = idx;
  }

  /* End of Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
}

static void trajGen_constructM(const real_T constraints[8], real_T M[64])
{
  real_T M_0;
  int32_T tmp_size[1];
  int32_T fixedBCIdx_size_idx_0;
  int32_T i;
  int32_T loop_ub;
  int32_T loop_ub_0;
  boolean_T fixedBCIdx_tmp[8];
  boolean_T fixedBCIdx_tmp_0[8];
  boolean_T fixedBCIdx_tmp_1;
  std::memset(&trajGen_B.M1[0], 0, sizeof(int8_T) << 6U);

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  for (i = 0; i < 8; i++) {
    fixedBCIdx_tmp_1 = std::isnan(constraints[i]);
    fixedBCIdx_tmp[i] = fixedBCIdx_tmp_1;
    fixedBCIdx_tmp_0[i] = !fixedBCIdx_tmp_1;
  }

  trajGen_eml_find(fixedBCIdx_tmp_0, trajGen_B.tmp_data, tmp_size);
  loop_ub = tmp_size[0];
  fixedBCIdx_size_idx_0 = tmp_size[0];
  if (loop_ub - 1 >= 0) {
    std::memcpy(&trajGen_B.fixedBCIdx_data[0], &trajGen_B.tmp_data[0],
                static_cast<uint32_T>(loop_ub) * sizeof(int32_T));
  }

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  trajGen_eml_find(fixedBCIdx_tmp, trajGen_B.tmp_data, tmp_size);
  loop_ub_0 = tmp_size[0];
  if (loop_ub_0 - 1 >= 0) {
    std::memcpy(&trajGen_B.freeBCIdx_data[0], &trajGen_B.tmp_data[0],
                static_cast<uint32_T>(loop_ub_0) * sizeof(int32_T));
  }

  for (i = 0; i < loop_ub; i++) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.M1[(trajGen_B.fixedBCIdx_data[i] + (i << 3)) - 1] = 1;
  }

  i = 7 - fixedBCIdx_size_idx_0;
  for (loop_ub = 0; loop_ub <= i; loop_ub++) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.M1[(trajGen_B.freeBCIdx_data[loop_ub] + ((fixedBCIdx_size_idx_0 +
      loop_ub) << 3)) - 1] = 1;
  }

  std::memset(&trajGen_B.Mcontinuity[0], 0, sizeof(int8_T) << 6U);
  for (i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.Mcontinuity[i + (i << 3)] = 1;
  }

  for (i = 0; i < 8; i++) {
    for (loop_ub = 0; loop_ub < 8; loop_ub++) {
      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      M_0 = 0.0;
      for (loop_ub_0 = 0; loop_ub_0 < 8; loop_ub_0++) {
        /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
        M_0 += static_cast<real_T>(trajGen_B.Mcontinuity[(loop_ub_0 << 3) +
          loop_ub] * trajGen_B.M1[(i << 3) + loop_ub_0]);
      }

      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      M[i + (loop_ub << 3)] = M_0;
    }
  }
}

static void trajGen_inv(const real_T x[16], real_T y[16])
{
  int32_T c;
  int32_T jA;
  int32_T jBcol;
  int32_T jj;
  int32_T k;
  int32_T kAcol;
  int32_T pipk;
  int8_T b_ipiv[4];
  int8_T p[4];
  for (c = 0; c < 16; c++) {
    y[c] = 0.0;

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.c_A_g[c] = x[c];
  }

  b_ipiv[0] = 1;
  b_ipiv[1] = 2;
  b_ipiv[2] = 3;
  for (pipk = 0; pipk < 3; pipk++) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    c = pipk * 5 + 2;
    jj = pipk * 5;
    jBcol = 4 - pipk;
    kAcol = 1;

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.smax_h = std::abs(trajGen_B.c_A_g[jj]);
    for (k = 2; k <= jBcol; k++) {
      real_T s;
      s = std::abs(trajGen_B.c_A_g[(c + k) - 3]);
      if (s > trajGen_B.smax_h) {
        kAcol = k;
        trajGen_B.smax_h = s;
      }
    }

    if (trajGen_B.c_A_g[(c + kAcol) - 3] != 0.0) {
      if (kAcol - 1 != 0) {
        k = pipk + kAcol;
        b_ipiv[pipk] = static_cast<int8_T>(k);
        trajGen_B.smax_h = trajGen_B.c_A_g[pipk];
        trajGen_B.c_A_g[pipk] = trajGen_B.c_A_g[k - 1];
        trajGen_B.c_A_g[k - 1] = trajGen_B.smax_h;
        trajGen_B.smax_h = trajGen_B.c_A_g[pipk + 4];
        trajGen_B.c_A_g[pipk + 4] = trajGen_B.c_A_g[k + 3];
        trajGen_B.c_A_g[k + 3] = trajGen_B.smax_h;
        trajGen_B.smax_h = trajGen_B.c_A_g[pipk + 8];
        trajGen_B.c_A_g[pipk + 8] = trajGen_B.c_A_g[k + 7];
        trajGen_B.c_A_g[k + 7] = trajGen_B.smax_h;
        trajGen_B.smax_h = trajGen_B.c_A_g[pipk + 12];
        trajGen_B.c_A_g[pipk + 12] = trajGen_B.c_A_g[k + 11];
        trajGen_B.c_A_g[k + 11] = trajGen_B.smax_h;
      }

      kAcol = c - pipk;
      for (jBcol = c; jBcol <= kAcol + 2; jBcol++) {
        trajGen_B.c_A_g[jBcol - 1] /= trajGen_B.c_A_g[jj];
      }
    }

    jA = jj;
    kAcol = 3 - pipk;
    for (k = 0; k < kAcol; k++) {
      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.smax_h = trajGen_B.c_A_g[((k << 2) + jj) + 4];
      if (trajGen_B.smax_h != 0.0) {
        int32_T d;

        /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
        jBcol = jA + 6;
        d = (jA - pipk) + 8;
        for (int32_T ijA{jBcol}; ijA <= d; ijA++) {
          /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
          trajGen_B.c_A_g[ijA - 1] += trajGen_B.c_A_g[((c + ijA) - jA) - 7] *
            -trajGen_B.smax_h;
        }
      }

      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      jA += 4;
    }
  }

  p[0] = 1;
  p[1] = 2;
  p[2] = 3;
  p[3] = 4;

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  if (b_ipiv[0] > 1) {
    pipk = p[b_ipiv[0] - 1];
    p[b_ipiv[0] - 1] = 1;
    p[0] = static_cast<int8_T>(pipk);
  }

  if (b_ipiv[1] > 2) {
    pipk = p[b_ipiv[1] - 1];
    p[b_ipiv[1] - 1] = p[1];
    p[1] = static_cast<int8_T>(pipk);
  }

  if (b_ipiv[2] > 3) {
    pipk = p[b_ipiv[2] - 1];
    p[b_ipiv[2] - 1] = p[2];
    p[2] = static_cast<int8_T>(pipk);
  }

  for (pipk = 0; pipk < 4; pipk++) {
    jj = (p[pipk] - 1) << 2;
    y[pipk + jj] = 1.0;

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    for (k = pipk + 1; k < 5; k++) {
      c = (jj + k) - 1;
      if (y[c] != 0.0) {
        for (jBcol = k + 1; jBcol < 5; jBcol++) {
          kAcol = (jj + jBcol) - 1;
          y[kAcol] -= trajGen_B.c_A_g[(((k - 1) << 2) + jBcol) - 1] * y[c];
        }
      }
    }
  }

  for (pipk = 0; pipk < 4; pipk++) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    jBcol = pipk << 2;
    for (k = 3; k >= 0; k--) {
      kAcol = k << 2;
      c = k + jBcol;
      trajGen_B.smax_h = y[c];
      if (trajGen_B.smax_h != 0.0) {
        y[c] = trajGen_B.smax_h / trajGen_B.c_A_g[k + kAcol];
        for (jA = 0; jA < k; jA++) {
          jj = jA + jBcol;
          y[jj] -= trajGen_B.c_A_g[jA + kAcol] * y[c];
        }
      }
    }
  }
}

static void trajGen_mtimes(const coder::array<real_T, 2U> &A, const coder::array<
  real_T, 1U> &B, coder::array<real_T, 1U> &C)
{
  int32_T inner;
  int32_T m_tmp;

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  m_tmp = A.size(1);
  inner = A.size(0);
  C.set_size(A.size(1));
  for (int32_T b_i{0}; b_i < m_tmp; b_i++) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    C[b_i] = 0.0;
  }

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  for (int32_T b_i{0}; b_i < inner; b_i++) {
    for (int32_T c_i{0}; c_i < m_tmp; c_i++) {
      C[c_i] = A[c_i * A.size(0) + b_i] * B[b_i] + C[c_i];
    }
  }
}

static real_T trajGen_xnrm2(int32_T n, const coder::array<real_T, 2U> &x,
  int32_T ix0)
{
  real_T y;
  y = 0.0;

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      int32_T kend;
      trajGen_B.scale_n = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int32_T k{ix0}; k < kend; k++) {
        trajGen_B.absxk_b = std::abs(x[k - 1]);
        if (trajGen_B.absxk_b > trajGen_B.scale_n) {
          trajGen_B.t_l = trajGen_B.scale_n / trajGen_B.absxk_b;
          y = y * trajGen_B.t_l * trajGen_B.t_l + 1.0;
          trajGen_B.scale_n = trajGen_B.absxk_b;
        } else {
          trajGen_B.t_l = trajGen_B.absxk_b / trajGen_B.scale_n;
          y += trajGen_B.t_l * trajGen_B.t_l;
        }
      }

      y = trajGen_B.scale_n * std::sqrt(y);
    }
  }

  /* End of Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  return y;
}

static real_T trajGen_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T b;
  real_T y;
  trajGen_B.a = std::abs(u0);
  b = std::abs(u1);
  if (trajGen_B.a < b) {
    trajGen_B.a /= b;
    y = std::sqrt(trajGen_B.a * trajGen_B.a + 1.0) * b;
  } else if (trajGen_B.a > b) {
    b /= trajGen_B.a;
    y = std::sqrt(b * b + 1.0) * trajGen_B.a;
  } else if (std::isnan(b)) {
    y = (rtNaN);
  } else {
    y = trajGen_B.a * 1.4142135623730951;
  }

  return y;
}

static void trajGen_xgeqp3(const coder::array<real_T, 2U> &A, coder::array<
  real_T, 2U> &b_A, coder::array<real_T, 1U> &tau, coder::array<int32_T, 2U>
  &jpvt)
{
  int32_T b_tmp;
  int32_T d;
  int32_T exitg1;
  int32_T itemp;
  int32_T ix;
  int32_T jA;
  int32_T kend;
  int32_T loop_ub_tmp;
  int32_T ma_tmp;
  int32_T minmana_tmp;
  int32_T na_tmp;
  int32_T pvt;
  int32_T temp_tmp;
  boolean_T exitg2;

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  ma_tmp = A.size(0);
  na_tmp = A.size(1);
  b_A.set_size(A.size(0), A.size(1));
  loop_ub_tmp = A.size(0) * A.size(1);
  for (b_tmp = 0; b_tmp < loop_ub_tmp; b_tmp++) {
    b_A[b_tmp] = A[b_tmp];
  }

  if (A.size(0) <= A.size(1)) {
    minmana_tmp = A.size(0);
  } else {
    minmana_tmp = A.size(1);
  }

  tau.set_size(minmana_tmp);
  for (b_tmp = 0; b_tmp < minmana_tmp; b_tmp++) {
    tau[b_tmp] = 0.0;
  }

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  if ((A.size(0) == 0) || (A.size(1) == 0) || (minmana_tmp < 1)) {
    jpvt.set_size(1, A.size(1));
    for (loop_ub_tmp = 0; loop_ub_tmp < na_tmp; loop_ub_tmp++) {
      jpvt[loop_ub_tmp] = loop_ub_tmp + 1;
    }
  } else {
    jpvt.set_size(1, A.size(1));
    for (loop_ub_tmp = 0; loop_ub_tmp < na_tmp; loop_ub_tmp++) {
      jpvt[loop_ub_tmp] = loop_ub_tmp + 1;
    }

    tau.set_size(minmana_tmp);
    for (b_tmp = 0; b_tmp < minmana_tmp; b_tmp++) {
      tau[b_tmp] = 0.0;
    }

    trajGen_B.work.set_size(A.size(1));
    trajGen_B.vn1.set_size(A.size(1));
    trajGen_B.vn2.set_size(A.size(1));
    for (loop_ub_tmp = 0; loop_ub_tmp < na_tmp; loop_ub_tmp++) {
      trajGen_B.work[loop_ub_tmp] = 0.0;
      trajGen_B.vn1[loop_ub_tmp] = 0.0;
      trajGen_B.vn2[loop_ub_tmp] = 0.0;
      jA = loop_ub_tmp * ma_tmp;
      trajGen_B.smax_b = 0.0;
      if (ma_tmp >= 1) {
        if (ma_tmp == 1) {
          trajGen_B.smax_b = std::abs(A[jA]);
        } else {
          trajGen_B.scale_d = 3.3121686421112381E-170;
          kend = jA + ma_tmp;
          for (itemp = jA + 1; itemp <= kend; itemp++) {
            trajGen_B.absxk_l = std::abs(A[itemp - 1]);
            if (trajGen_B.absxk_l > trajGen_B.scale_d) {
              trajGen_B.t_o = trajGen_B.scale_d / trajGen_B.absxk_l;
              trajGen_B.smax_b = trajGen_B.smax_b * trajGen_B.t_o *
                trajGen_B.t_o + 1.0;
              trajGen_B.scale_d = trajGen_B.absxk_l;
            } else {
              trajGen_B.t_o = trajGen_B.absxk_l / trajGen_B.scale_d;
              trajGen_B.smax_b += trajGen_B.t_o * trajGen_B.t_o;
            }
          }

          trajGen_B.smax_b = trajGen_B.scale_d * std::sqrt(trajGen_B.smax_b);
        }
      }

      trajGen_B.vn1[loop_ub_tmp] = trajGen_B.smax_b;
      trajGen_B.vn2[loop_ub_tmp] = trajGen_B.smax_b;
    }

    for (loop_ub_tmp = 0; loop_ub_tmp < minmana_tmp; loop_ub_tmp++) {
      jA = loop_ub_tmp * ma_tmp;
      trajGen_B.ii = jA + loop_ub_tmp;
      trajGen_B.nmi = na_tmp - loop_ub_tmp;
      kend = (ma_tmp - loop_ub_tmp) - 1;
      if (trajGen_B.nmi < 1) {
        pvt = 0;
      } else {
        pvt = 1;
        if (trajGen_B.nmi > 1) {
          trajGen_B.smax_b = std::abs(trajGen_B.vn1[loop_ub_tmp]);
          for (itemp = 2; itemp <= trajGen_B.nmi; itemp++) {
            trajGen_B.scale_d = std::abs(trajGen_B.vn1[(loop_ub_tmp + itemp) - 1]);
            if (trajGen_B.scale_d > trajGen_B.smax_b) {
              pvt = itemp;
              trajGen_B.smax_b = trajGen_B.scale_d;
            }
          }
        }
      }

      pvt = (loop_ub_tmp + pvt) - 1;
      if (pvt + 1 != loop_ub_tmp + 1) {
        ix = pvt * ma_tmp;
        for (itemp = 0; itemp < ma_tmp; itemp++) {
          temp_tmp = ix + itemp;
          trajGen_B.scale_d = b_A[temp_tmp];
          b_tmp = jA + itemp;
          b_A[temp_tmp] = b_A[b_tmp];
          b_A[b_tmp] = trajGen_B.scale_d;
        }

        itemp = jpvt[pvt];
        jpvt[pvt] = jpvt[loop_ub_tmp];
        jpvt[loop_ub_tmp] = itemp;
        trajGen_B.vn1[pvt] = trajGen_B.vn1[loop_ub_tmp];
        trajGen_B.vn2[pvt] = trajGen_B.vn2[loop_ub_tmp];
      }

      if (loop_ub_tmp + 1 < ma_tmp) {
        jA = trajGen_B.ii + 2;
        trajGen_B.smax_b = b_A[trajGen_B.ii];
        tau[loop_ub_tmp] = 0.0;
        if (kend + 1 > 0) {
          trajGen_B.scale_d = trajGen_xnrm2(kend, b_A, trajGen_B.ii + 2);
          if (trajGen_B.scale_d != 0.0) {
            trajGen_B.scale_d = trajGen_rt_hypotd_snf(b_A[trajGen_B.ii],
              trajGen_B.scale_d);
            if (b_A[trajGen_B.ii] >= 0.0) {
              trajGen_B.scale_d = -trajGen_B.scale_d;
            }

            if (std::abs(trajGen_B.scale_d) < 1.0020841800044864E-292) {
              pvt = -1;
              do {
                pvt++;
                b_tmp = (trajGen_B.ii + kend) + 1;
                for (itemp = jA; itemp <= b_tmp; itemp++) {
                  b_A[itemp - 1] = b_A[itemp - 1] * 9.9792015476736E+291;
                }

                trajGen_B.scale_d *= 9.9792015476736E+291;
                trajGen_B.smax_b *= 9.9792015476736E+291;
              } while ((std::abs(trajGen_B.scale_d) < 1.0020841800044864E-292) &&
                       (pvt + 1 < 20));

              trajGen_B.scale_d = trajGen_rt_hypotd_snf(trajGen_B.smax_b,
                trajGen_xnrm2(kend, b_A, trajGen_B.ii + 2));
              if (trajGen_B.smax_b >= 0.0) {
                trajGen_B.scale_d = -trajGen_B.scale_d;
              }

              tau[loop_ub_tmp] = (trajGen_B.scale_d - trajGen_B.smax_b) /
                trajGen_B.scale_d;
              trajGen_B.smax_b = 1.0 / (trajGen_B.smax_b - trajGen_B.scale_d);
              for (itemp = jA; itemp <= b_tmp; itemp++) {
                b_A[itemp - 1] = b_A[itemp - 1] * trajGen_B.smax_b;
              }

              for (itemp = 0; itemp <= pvt; itemp++) {
                trajGen_B.scale_d *= 1.0020841800044864E-292;
              }

              trajGen_B.smax_b = trajGen_B.scale_d;
            } else {
              tau[loop_ub_tmp] = (trajGen_B.scale_d - b_A[trajGen_B.ii]) /
                trajGen_B.scale_d;
              trajGen_B.smax_b = 1.0 / (b_A[trajGen_B.ii] - trajGen_B.scale_d);
              pvt = (trajGen_B.ii + kend) + 1;
              for (itemp = jA; itemp <= pvt; itemp++) {
                b_A[itemp - 1] = b_A[itemp - 1] * trajGen_B.smax_b;
              }

              trajGen_B.smax_b = trajGen_B.scale_d;
            }
          }
        }

        b_A[trajGen_B.ii] = trajGen_B.smax_b;
      } else {
        tau[loop_ub_tmp] = 0.0;
      }

      if (loop_ub_tmp + 1 < na_tmp) {
        trajGen_B.smax_b = b_A[trajGen_B.ii];
        b_A[trajGen_B.ii] = 1.0;
        jA = (trajGen_B.ii + ma_tmp) + 1;
        if (tau[loop_ub_tmp] != 0.0) {
          itemp = kend;
          pvt = trajGen_B.ii + kend;
          while ((itemp + 1 > 0) && (b_A[pvt] == 0.0)) {
            itemp--;
            pvt--;
          }

          trajGen_B.nmi--;
          exitg2 = false;
          while ((!exitg2) && (trajGen_B.nmi > 0)) {
            pvt = (trajGen_B.nmi - 1) * ma_tmp + jA;
            b_tmp = pvt;
            do {
              exitg1 = 0;
              if (b_tmp <= pvt + itemp) {
                if (b_A[b_tmp - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  b_tmp++;
                }
              } else {
                trajGen_B.nmi--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }

          trajGen_B.nmi--;
        } else {
          itemp = -1;
          trajGen_B.nmi = -1;
        }

        if (itemp + 1 > 0) {
          if (trajGen_B.nmi + 1 != 0) {
            for (pvt = 0; pvt <= trajGen_B.nmi; pvt++) {
              trajGen_B.work[pvt] = 0.0;
            }

            ix = 0;
            pvt = ma_tmp * trajGen_B.nmi + jA;
            for (temp_tmp = jA; ma_tmp < 0 ? temp_tmp >= pvt : temp_tmp <= pvt;
                 temp_tmp += ma_tmp) {
              trajGen_B.scale_d = 0.0;
              d = temp_tmp + itemp;
              for (b_tmp = temp_tmp; b_tmp <= d; b_tmp++) {
                trajGen_B.scale_d += b_A[(trajGen_B.ii + b_tmp) - temp_tmp] *
                  b_A[b_tmp - 1];
              }

              trajGen_B.work[ix] = trajGen_B.work[ix] + trajGen_B.scale_d;
              ix++;
            }
          }

          if (!(-tau[loop_ub_tmp] == 0.0)) {
            for (pvt = 0; pvt <= trajGen_B.nmi; pvt++) {
              trajGen_B.scale_d = trajGen_B.work[pvt];
              if (trajGen_B.scale_d != 0.0) {
                trajGen_B.scale_d *= -tau[loop_ub_tmp];
                b_tmp = itemp + jA;
                for (ix = jA; ix <= b_tmp; ix++) {
                  b_A[ix - 1] = b_A[(trajGen_B.ii + ix) - jA] *
                    trajGen_B.scale_d + b_A[ix - 1];
                }
              }

              jA += ma_tmp;
            }
          }
        }

        b_A[trajGen_B.ii] = trajGen_B.smax_b;
      }

      for (trajGen_B.ii = loop_ub_tmp + 2; trajGen_B.ii <= na_tmp; trajGen_B.ii
           ++) {
        trajGen_B.nmi = (trajGen_B.ii - 1) * ma_tmp + loop_ub_tmp;
        trajGen_B.smax_b = trajGen_B.vn1[trajGen_B.ii - 1];
        if (trajGen_B.smax_b != 0.0) {
          trajGen_B.scale_d = std::abs(b_A[trajGen_B.nmi]) / trajGen_B.smax_b;
          trajGen_B.scale_d = 1.0 - trajGen_B.scale_d * trajGen_B.scale_d;
          if (trajGen_B.scale_d < 0.0) {
            trajGen_B.scale_d = 0.0;
          }

          trajGen_B.absxk_l = trajGen_B.smax_b / trajGen_B.vn2[trajGen_B.ii - 1];
          trajGen_B.absxk_l = trajGen_B.absxk_l * trajGen_B.absxk_l *
            trajGen_B.scale_d;
          if (trajGen_B.absxk_l <= 1.4901161193847656E-8) {
            if (loop_ub_tmp + 1 < ma_tmp) {
              trajGen_B.smax_b = trajGen_xnrm2(kend, b_A, trajGen_B.nmi + 2);
              trajGen_B.vn1[trajGen_B.ii - 1] = trajGen_B.smax_b;
              trajGen_B.vn2[trajGen_B.ii - 1] = trajGen_B.smax_b;
            } else {
              trajGen_B.vn1[trajGen_B.ii - 1] = 0.0;
              trajGen_B.vn2[trajGen_B.ii - 1] = 0.0;
            }
          } else {
            trajGen_B.vn1[trajGen_B.ii - 1] = trajGen_B.smax_b * std::sqrt
              (trajGen_B.scale_d);
          }
        }
      }
    }
  }
}

static int32_T trajGen_rankFromQR(const coder::array<real_T, 2U> &A)
{
  int32_T maxmn;
  int32_T minmn;
  int32_T r;
  r = 0;

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  if (A.size(0) < A.size(1)) {
    minmn = A.size(0);
    maxmn = A.size(1);
  } else {
    minmn = A.size(1);
    maxmn = A.size(0);
  }

  /* End of Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  if (minmn > 0) {
    while ((r < minmn) && (!(std::abs(A[A.size(0) * r + r]) <= std::fmin
             (1.4901161193847656E-8, 2.2204460492503131E-15 * static_cast<real_T>
              (maxmn)) * std::abs(A[0])))) {
      r++;
    }
  }

  return r;
}

static void trajGen_xzgetrf(int32_T m, int32_T n, coder::array<real_T, 2U> &A,
  int32_T lda, coder::array<int32_T, 2U> &ipiv, int32_T *info)
{
  int32_T k;
  int32_T n_0;
  int32_T yk;

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  if (m <= n) {
    n_0 = m;
  } else {
    n_0 = n;
  }

  if (n_0 < 1) {
    n_0 = 0;
  }

  ipiv.set_size(1, n_0);
  if (n_0 > 0) {
    ipiv[0] = 1;
    yk = 1;
    for (k = 2; k <= n_0; k++) {
      yk++;
      ipiv[k - 1] = yk;
    }
  }

  *info = 0;
  if ((m >= 1) && (n >= 1)) {
    if (m - 1 <= n) {
      yk = m - 1;
    } else {
      yk = n;
    }

    for (n_0 = 0; n_0 < yk; n_0++) {
      int32_T a;
      int32_T c;
      int32_T c_tmp;
      int32_T jA;
      int32_T mmj;
      int32_T temp_tmp;
      int32_T temp_tmp_0;
      mmj = m - n_0;
      c_tmp = (lda + 1) * n_0;
      c = c_tmp + 2;
      if (mmj < 1) {
        a = 0;
      } else {
        a = 1;
        if (mmj > 1) {
          trajGen_B.smax_bn = std::abs(A[c_tmp]);
          for (k = 2; k <= mmj; k++) {
            real_T s;
            s = std::abs(A[(c_tmp + k) - 1]);
            if (s > trajGen_B.smax_bn) {
              a = k;
              trajGen_B.smax_bn = s;
            }
          }
        }
      }

      if (A[(c_tmp + a) - 1] != 0.0) {
        if (a - 1 != 0) {
          a += n_0;
          ipiv[n_0] = a;
          for (k = 0; k < n; k++) {
            temp_tmp = k * lda;
            temp_tmp_0 = temp_tmp + n_0;
            trajGen_B.smax_bn = A[temp_tmp_0];
            temp_tmp = (temp_tmp + a) - 1;
            A[temp_tmp_0] = A[temp_tmp];
            A[temp_tmp] = trajGen_B.smax_bn;
          }
        }

        k = (c_tmp + mmj) + 2;
        for (a = c; a <= k - 2; a++) {
          A[a - 1] = A[a - 1] / A[c_tmp];
        }
      } else {
        *info = n_0 + 1;
      }

      c = (n - n_0) - 2;
      temp_tmp = (c_tmp + lda) + 1;
      jA = temp_tmp;
      for (a = 0; a <= c; a++) {
        trajGen_B.smax_bn = A[(a * lda + temp_tmp) - 1];
        if (trajGen_B.smax_bn != 0.0) {
          int32_T b;
          b = jA + 1;
          k = (mmj + jA) - 1;
          for (temp_tmp_0 = b; temp_tmp_0 <= k; temp_tmp_0++) {
            A[temp_tmp_0 - 1] = A[(c_tmp + temp_tmp_0) - jA] *
              -trajGen_B.smax_bn + A[temp_tmp_0 - 1];
          }
        }

        jA += lda;
      }
    }

    if ((*info == 0) && (m <= n) && (!(A[((m - 1) * A.size(0) + m) - 1] != 0.0)))
    {
      *info = m;
    }
  }

  /* End of Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
}

static void trajGen_mldivide(const coder::array<real_T, 2U> &A, const coder::
  array<real_T, 1U> &B, coder::array<real_T, 1U> &Y)
{
  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  if ((A.size(0) == 0) || (A.size(1) == 0) || (B.size(0) == 0)) {
    trajGen_B.mn = A.size(1);
    Y.set_size(A.size(1));
    for (trajGen_B.i = 0; trajGen_B.i < trajGen_B.mn; trajGen_B.i++) {
      Y[trajGen_B.i] = 0.0;
    }
  } else if (A.size(0) == A.size(1)) {
    if (A.size(0) <= A.size(1)) {
      trajGen_B.rankA = A.size(0);
    } else {
      trajGen_B.rankA = A.size(1);
    }

    if (B.size(0) <= trajGen_B.rankA) {
      trajGen_B.rankA = B.size(0);
    }

    trajGen_B.b_A.set_size(A.size(0), A.size(1));
    trajGen_B.mn = A.size(0) * A.size(1);
    for (trajGen_B.i = 0; trajGen_B.i < trajGen_B.mn; trajGen_B.i++) {
      trajGen_B.b_A[trajGen_B.i] = A[trajGen_B.i];
    }

    trajGen_xzgetrf(trajGen_B.rankA, trajGen_B.rankA, trajGen_B.b_A, A.size(0),
                    trajGen_B.jpvt, &trajGen_B.i);
    trajGen_B.mn = B.size(0);
    Y.set_size(B.size(0));
    for (trajGen_B.i = 0; trajGen_B.i < trajGen_B.mn; trajGen_B.i++) {
      Y[trajGen_B.i] = B[trajGen_B.i];
    }

    trajGen_B.mn = trajGen_B.b_A.size(0);
    trajGen_B.m = trajGen_B.rankA - 2;
    for (trajGen_B.i = 0; trajGen_B.i <= trajGen_B.m; trajGen_B.i++) {
      trajGen_B.kAcol = trajGen_B.jpvt[trajGen_B.i];
      if (trajGen_B.i + 1 != trajGen_B.kAcol) {
        trajGen_B.wj = Y[trajGen_B.i];
        Y[trajGen_B.i] = Y[trajGen_B.kAcol - 1];
        Y[trajGen_B.kAcol - 1] = trajGen_B.wj;
      }
    }

    for (trajGen_B.m = 0; trajGen_B.m < trajGen_B.rankA; trajGen_B.m++) {
      trajGen_B.kAcol = trajGen_B.mn * trajGen_B.m - 1;
      if (Y[trajGen_B.m] != 0.0) {
        for (trajGen_B.i = trajGen_B.m + 2; trajGen_B.i <= trajGen_B.rankA;
             trajGen_B.i++) {
          Y[trajGen_B.i - 1] = Y[trajGen_B.i - 1] - trajGen_B.b_A[trajGen_B.i +
            trajGen_B.kAcol] * Y[trajGen_B.m];
        }
      }
    }

    for (trajGen_B.k = trajGen_B.rankA; trajGen_B.k >= 1; trajGen_B.k--) {
      trajGen_B.kAcol = (trajGen_B.k - 1) * trajGen_B.mn - 1;
      trajGen_B.wj = Y[trajGen_B.k - 1];
      if (trajGen_B.wj != 0.0) {
        Y[trajGen_B.k - 1] = trajGen_B.wj / trajGen_B.b_A[trajGen_B.k +
          trajGen_B.kAcol];
        trajGen_B.m = trajGen_B.k - 2;
        for (trajGen_B.i = 0; trajGen_B.i <= trajGen_B.m; trajGen_B.i++) {
          Y[trajGen_B.i] = Y[trajGen_B.i] - trajGen_B.b_A[(trajGen_B.i +
            trajGen_B.kAcol) + 1] * Y[trajGen_B.k - 1];
        }
      }
    }
  } else {
    trajGen_xgeqp3(A, trajGen_B.b_A, trajGen_B.tau, trajGen_B.jpvt);
    trajGen_B.rankA = trajGen_rankFromQR(trajGen_B.b_A);
    trajGen_B.mn = trajGen_B.b_A.size(1);
    Y.set_size(trajGen_B.b_A.size(1));
    for (trajGen_B.i = 0; trajGen_B.i < trajGen_B.mn; trajGen_B.i++) {
      Y[trajGen_B.i] = 0.0;
    }

    trajGen_B.mn = B.size(0);
    trajGen_B.B.set_size(B.size(0));
    for (trajGen_B.i = 0; trajGen_B.i < trajGen_B.mn; trajGen_B.i++) {
      trajGen_B.B[trajGen_B.i] = B[trajGen_B.i];
    }

    if (trajGen_B.b_A.size(0) <= trajGen_B.b_A.size(1)) {
      trajGen_B.mn = trajGen_B.b_A.size(0);
    } else {
      trajGen_B.mn = trajGen_B.b_A.size(1);
    }

    for (trajGen_B.kAcol = 0; trajGen_B.kAcol < trajGen_B.mn; trajGen_B.kAcol++)
    {
      trajGen_B.m = trajGen_B.b_A.size(0);
      if (trajGen_B.tau[trajGen_B.kAcol] != 0.0) {
        trajGen_B.wj = trajGen_B.B[trajGen_B.kAcol];
        for (trajGen_B.i = trajGen_B.kAcol + 2; trajGen_B.i <= trajGen_B.m;
             trajGen_B.i++) {
          trajGen_B.wj += trajGen_B.b_A[(trajGen_B.b_A.size(0) * trajGen_B.kAcol
            + trajGen_B.i) - 1] * trajGen_B.B[trajGen_B.i - 1];
        }

        trajGen_B.wj *= trajGen_B.tau[trajGen_B.kAcol];
        if (trajGen_B.wj != 0.0) {
          trajGen_B.B[trajGen_B.kAcol] = trajGen_B.B[trajGen_B.kAcol] -
            trajGen_B.wj;
          for (trajGen_B.i = trajGen_B.kAcol + 2; trajGen_B.i <= trajGen_B.m;
               trajGen_B.i++) {
            trajGen_B.B[trajGen_B.i - 1] = trajGen_B.B[trajGen_B.i - 1] -
              trajGen_B.b_A[(trajGen_B.b_A.size(0) * trajGen_B.kAcol +
                             trajGen_B.i) - 1] * trajGen_B.wj;
          }
        }
      }
    }

    for (trajGen_B.i = 0; trajGen_B.i < trajGen_B.rankA; trajGen_B.i++) {
      Y[trajGen_B.jpvt[trajGen_B.i] - 1] = trajGen_B.B[trajGen_B.i];
    }

    for (trajGen_B.i = trajGen_B.rankA; trajGen_B.i >= 1; trajGen_B.i--) {
      trajGen_B.mn = trajGen_B.jpvt[trajGen_B.i - 1];
      Y[trajGen_B.mn - 1] = Y[trajGen_B.mn - 1] / trajGen_B.b_A[((trajGen_B.i -
        1) * trajGen_B.b_A.size(0) + trajGen_B.i) - 1];
      trajGen_B.kAcol = trajGen_B.i - 2;
      for (trajGen_B.m = 0; trajGen_B.m <= trajGen_B.kAcol; trajGen_B.m++) {
        Y[trajGen_B.jpvt[trajGen_B.m] - 1] = Y[trajGen_B.jpvt[trajGen_B.m] - 1]
          - trajGen_B.b_A[(trajGen_B.i - 1) * trajGen_B.b_A.size(0) +
          trajGen_B.m] * Y[trajGen_B.mn - 1];
      }
    }
  }

  /* End of Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
}

static void trajGen_xzgetrf_p(real_T A[64], int32_T ipiv[8], int32_T *info)
{
  int32_T c_0;

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  for (c_0 = 0; c_0 < 8; c_0++) {
    ipiv[c_0] = c_0 + 1;
  }

  *info = 0;
  for (int32_T b_j{0}; b_j < 7; b_j++) {
    int32_T a;
    int32_T c;
    int32_T jA;
    int32_T jj;
    int32_T k;
    int32_T temp_tmp;
    c = b_j * 9 + 2;
    jj = b_j * 9;
    c_0 = 8 - b_j;
    a = 1;
    trajGen_B.smax = std::abs(A[jj]);
    for (k = 2; k <= c_0; k++) {
      trajGen_B.s_d = std::abs(A[(c + k) - 3]);
      if (trajGen_B.s_d > trajGen_B.smax) {
        a = k;
        trajGen_B.smax = trajGen_B.s_d;
      }
    }

    if (A[(c + a) - 3] != 0.0) {
      if (a - 1 != 0) {
        a += b_j;
        ipiv[b_j] = a;
        for (k = 0; k < 8; k++) {
          c_0 = k << 3;
          temp_tmp = c_0 + b_j;
          trajGen_B.smax = A[temp_tmp];
          c_0 = (c_0 + a) - 1;
          A[temp_tmp] = A[c_0];
          A[c_0] = trajGen_B.smax;
        }
      }

      k = c - b_j;
      for (c_0 = c; c_0 <= k + 6; c_0++) {
        A[c_0 - 1] /= A[jj];
      }
    } else {
      *info = b_j + 1;
    }

    jA = jj;
    k = 7 - b_j;
    for (a = 0; a < k; a++) {
      trajGen_B.smax = A[((a << 3) + jj) + 8];
      if (trajGen_B.smax != 0.0) {
        int32_T d;
        c_0 = jA + 10;
        d = (jA - b_j) + 16;
        for (temp_tmp = c_0; temp_tmp <= d; temp_tmp++) {
          A[temp_tmp - 1] += A[((c + temp_tmp) - jA) - 11] * -trajGen_B.smax;
        }
      }

      jA += 8;
    }
  }

  if ((*info == 0) && (!(A[63] != 0.0))) {
    *info = 8;
  }

  /* End of Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
}

static void trajGen_mldivide_pv(const real_T A[64], real_T B[8])
{
  real_T temp;
  int32_T b_info;
  int32_T i;
  int32_T kAcol;

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  std::memcpy(&trajGen_B.c_A_b[0], &A[0], sizeof(real_T) << 6U);
  trajGen_xzgetrf_p(trajGen_B.c_A_b, trajGen_B.b_ipiv, &b_info);
  for (b_info = 0; b_info < 7; b_info++) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    kAcol = trajGen_B.b_ipiv[b_info];
    if (b_info + 1 != kAcol) {
      temp = B[b_info];
      B[b_info] = B[kAcol - 1];
      B[kAcol - 1] = temp;
    }
  }

  for (b_info = 0; b_info < 8; b_info++) {
    kAcol = (b_info << 3) - 1;
    if (B[b_info] != 0.0) {
      for (i = b_info + 2; i < 9; i++) {
        /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
        B[i - 1] -= trajGen_B.c_A_b[i + kAcol] * B[b_info];
      }
    }
  }

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  for (i = 7; i >= 0; i--) {
    kAcol = i << 3;
    temp = B[i];
    if (temp != 0.0) {
      B[i] = temp / trajGen_B.c_A_b[i + kAcol];
      for (b_info = 0; b_info < i; b_info++) {
        B[b_info] -= trajGen_B.c_A_b[b_info + kAcol] * B[i];
      }
    }
  }
}

static real_T trajGen_rcond(const real_T A[64])
{
  real_T result;
  int32_T b_j;
  int32_T c;
  int32_T exitg2;
  int32_T i;
  int32_T ix;
  boolean_T exitg1;
  boolean_T p;
  result = 0.0;
  trajGen_B.normA = 0.0;
  b_j = 0;
  exitg1 = false;
  while ((!exitg1) && (b_j < 8)) {
    trajGen_B.s = 0.0;
    for (i = 0; i < 8; i++) {
      trajGen_B.s += std::abs(A[(b_j << 3) + i]);
    }

    if (std::isnan(trajGen_B.s)) {
      trajGen_B.normA = (rtNaN);
      exitg1 = true;
    } else {
      if (trajGen_B.s > trajGen_B.normA) {
        trajGen_B.normA = trajGen_B.s;
      }

      b_j++;
    }
  }

  if (!(trajGen_B.normA == 0.0)) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    std::memcpy(&trajGen_B.c_A[0], &A[0], sizeof(real_T) << 6U);
    trajGen_xzgetrf_p(trajGen_B.c_A, trajGen_B.ipiv, &trajGen_B.iter);
    trajGen_B.iter = 7;
    do {
      exitg2 = 0;
      if (trajGen_B.iter + 1 > 0) {
        if (trajGen_B.c_A[(trajGen_B.iter << 3) + trajGen_B.iter] == 0.0) {
          exitg2 = 1;
        } else {
          trajGen_B.iter--;
        }
      } else {
        trajGen_B.s = 0.0;
        trajGen_B.iter = 2;
        trajGen_B.kase = 1;
        trajGen_B.jump = 1;
        trajGen_B.j = 1;
        for (i = 0; i < 8; i++) {
          trajGen_B.x[i] = 0.125;
        }

        while (trajGen_B.kase != 0) {
          if (trajGen_B.kase == 1) {
            for (b_j = 0; b_j < 8; b_j++) {
              trajGen_B.jA = (b_j << 3) + b_j;
              c = 7 - b_j;
              for (i = 0; i < c; i++) {
                ix = (i + b_j) + 1;
                trajGen_B.x[ix] -= trajGen_B.c_A[(i + trajGen_B.jA) + 1] *
                  trajGen_B.x[b_j];
              }
            }

            for (b_j = 7; b_j >= 0; b_j--) {
              trajGen_B.jA = (b_j << 3) + b_j;
              trajGen_B.x[b_j] /= trajGen_B.c_A[trajGen_B.jA];
              for (i = 0; i < b_j; i++) {
                ix = (b_j - i) - 1;
                trajGen_B.x[ix] -= trajGen_B.c_A[(trajGen_B.jA - i) - 1] *
                  trajGen_B.x[b_j];
              }
            }
          } else {
            for (b_j = 0; b_j < 8; b_j++) {
              trajGen_B.jA = b_j << 3;
              trajGen_B.temp = trajGen_B.x[b_j];
              for (i = 0; i < b_j; i++) {
                trajGen_B.temp -= trajGen_B.c_A[trajGen_B.jA + i] *
                  trajGen_B.x[i];
              }

              trajGen_B.x[b_j] = trajGen_B.temp / trajGen_B.c_A[trajGen_B.jA +
                b_j];
            }

            for (b_j = 7; b_j >= 0; b_j--) {
              trajGen_B.jA = (b_j << 3) - 1;
              trajGen_B.temp = trajGen_B.x[b_j];
              for (i = 8; i >= b_j + 2; i--) {
                trajGen_B.temp -= trajGen_B.c_A[trajGen_B.jA + i] *
                  trajGen_B.x[i - 1];
              }

              trajGen_B.x[b_j] = trajGen_B.temp;
            }
          }

          if (trajGen_B.jump == 1) {
            trajGen_B.s = 0.0;
            for (trajGen_B.kase = 0; trajGen_B.kase < 8; trajGen_B.kase++) {
              trajGen_B.s += std::abs(trajGen_B.x[trajGen_B.kase]);
            }

            if ((!std::isinf(trajGen_B.s)) && (!std::isnan(trajGen_B.s))) {
              for (trajGen_B.kase = 0; trajGen_B.kase < 8; trajGen_B.kase++) {
                trajGen_B.temp = trajGen_B.x[trajGen_B.kase];
                trajGen_B.absrexk = std::abs(trajGen_B.temp);
                if (trajGen_B.absrexk > 2.2250738585072014E-308) {
                  trajGen_B.x[trajGen_B.kase] = trajGen_B.temp /
                    trajGen_B.absrexk;
                } else {
                  trajGen_B.x[trajGen_B.kase] = 1.0;
                }
              }

              trajGen_B.kase = 2;
              trajGen_B.jump = 2;
            } else {
              trajGen_B.kase = 0;
            }
          } else if (trajGen_B.jump == 2) {
            trajGen_B.j = 1;
            trajGen_B.temp = std::abs(trajGen_B.x[0]);
            for (trajGen_B.kase = 0; trajGen_B.kase < 7; trajGen_B.kase++) {
              trajGen_B.absrexk = std::abs(trajGen_B.x[trajGen_B.kase + 1]);
              if (!(trajGen_B.absrexk <= trajGen_B.temp)) {
                trajGen_B.j = trajGen_B.kase + 2;
                trajGen_B.temp = trajGen_B.absrexk;
              }
            }

            trajGen_B.iter = 2;
            std::memset(&trajGen_B.x[0], 0, sizeof(real_T) << 3U);
            trajGen_B.x[trajGen_B.j - 1] = 1.0;
            trajGen_B.kase = 1;
            trajGen_B.jump = 3;
          } else if (trajGen_B.jump == 3) {
            trajGen_B.s = 0.0;
            for (trajGen_B.kase = 0; trajGen_B.kase < 8; trajGen_B.kase++) {
              trajGen_B.s += std::abs(trajGen_B.x[trajGen_B.kase]);
            }

            if (trajGen_B.s <= trajGen_B.x[0]) {
              trajGen_B.temp = 1.0;
              for (trajGen_B.kase = 0; trajGen_B.kase < 8; trajGen_B.kase++) {
                trajGen_B.x[trajGen_B.kase] = (((static_cast<real_T>
                  (trajGen_B.kase) + 1.0) - 1.0) / 7.0 + 1.0) * trajGen_B.temp;
                trajGen_B.temp = -trajGen_B.temp;
              }

              trajGen_B.kase = 1;
              trajGen_B.jump = 5;
            } else {
              for (trajGen_B.kase = 0; trajGen_B.kase < 8; trajGen_B.kase++) {
                trajGen_B.temp = trajGen_B.x[trajGen_B.kase];
                trajGen_B.absrexk = std::abs(trajGen_B.temp);
                if (trajGen_B.absrexk > 2.2250738585072014E-308) {
                  trajGen_B.x[trajGen_B.kase] = trajGen_B.temp /
                    trajGen_B.absrexk;
                } else {
                  trajGen_B.x[trajGen_B.kase] = 1.0;
                }
              }

              trajGen_B.kase = 2;
              trajGen_B.jump = 4;
            }
          } else if (trajGen_B.jump == 4) {
            trajGen_B.jump = trajGen_B.j;
            trajGen_B.j = 1;
            trajGen_B.temp = std::abs(trajGen_B.x[0]);
            for (trajGen_B.kase = 0; trajGen_B.kase < 7; trajGen_B.kase++) {
              trajGen_B.absrexk = std::abs(trajGen_B.x[trajGen_B.kase + 1]);
              if (!(trajGen_B.absrexk <= trajGen_B.temp)) {
                trajGen_B.j = trajGen_B.kase + 2;
                trajGen_B.temp = trajGen_B.absrexk;
              }
            }

            if ((std::abs(trajGen_B.x[trajGen_B.jump - 1]) != std::abs
                 (trajGen_B.x[trajGen_B.j - 1])) && (trajGen_B.iter <= 5)) {
              trajGen_B.iter++;
              std::memset(&trajGen_B.x[0], 0, sizeof(real_T) << 3U);
              trajGen_B.x[trajGen_B.j - 1] = 1.0;
              trajGen_B.kase = 1;
              trajGen_B.jump = 3;
            } else {
              trajGen_B.temp = 1.0;
              for (trajGen_B.kase = 0; trajGen_B.kase < 8; trajGen_B.kase++) {
                trajGen_B.x[trajGen_B.kase] = (((static_cast<real_T>
                  (trajGen_B.kase) + 1.0) - 1.0) / 7.0 + 1.0) * trajGen_B.temp;
                trajGen_B.temp = -trajGen_B.temp;
              }

              trajGen_B.kase = 1;
              trajGen_B.jump = 5;
            }
          } else if (trajGen_B.jump == 5) {
            trajGen_B.temp = 0.0;
            for (trajGen_B.kase = 0; trajGen_B.kase < 8; trajGen_B.kase++) {
              trajGen_B.temp += std::abs(trajGen_B.x[trajGen_B.kase]);
            }

            trajGen_B.temp = 2.0 * trajGen_B.temp / 3.0 / 8.0;
            if (trajGen_B.temp > trajGen_B.s) {
              trajGen_B.s = trajGen_B.temp;
            }

            trajGen_B.kase = 0;
          }
        }

        if (trajGen_B.s != 0.0) {
          result = 1.0 / trajGen_B.s / trajGen_B.normA;
        }

        if (std::isnan(result)) {
          p = false;
          for (trajGen_B.kase = 0; trajGen_B.kase < 64; trajGen_B.kase++) {
            if (p || std::isnan(trajGen_B.c_A[trajGen_B.kase])) {
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
  const real_T waypoints[6], const real_T timePoints[2], real_T ppMatrix[24],
  coder::array<real_T, 2U> &toaZero, real_T *maxIterStatus, real_T
  *maxTimeStatus, real_T *singularityStatus)
{
  int8_T d[2];
  int8_T d_0;
  static const int8_T tmp[8]{ 0, 0, 0, 1, 0, 0, 0, 0 };

  obj->TimeOffset = 0.0;
  for (trajGen_B.c = 0; trajGen_B.c < 3; trajGen_B.c++) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.waypoints_tmp = trajGen_B.c << 1;
    trajGen_B.waypoints_g[trajGen_B.waypoints_tmp] = waypoints[trajGen_B.c];
    trajGen_B.waypoints_g[trajGen_B.waypoints_tmp + 1] = waypoints[trajGen_B.c +
      3];
  }

  std::memset(&trajGen_B.constraints[0], 0, 24U * sizeof(real_T));

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  trajGen_B.waypoints[1] = 0.0;
  trajGen_B.waypoints[2] = 0.0;
  trajGen_B.waypoints[3] = 0.0;
  trajGen_B.waypoints[5] = 0.0;
  trajGen_B.waypoints[6] = 0.0;
  trajGen_B.waypoints[7] = 0.0;
  trajGen_B.waypoints[9] = 0.0;
  trajGen_B.waypoints[10] = 0.0;
  trajGen_B.waypoints[11] = 0.0;
  for (trajGen_B.waypoints_tmp = 0; trajGen_B.waypoints_tmp < 2;
       trajGen_B.waypoints_tmp++) {
    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.b = (trajGen_B.waypoints_tmp << 2) + 1;
    trajGen_B.c = (trajGen_B.waypoints_tmp + 1) << 2;
    if (trajGen_B.b > trajGen_B.c) {
      trajGen_B.e = 0;
      trajGen_B.c = 0;
    } else {
      trajGen_B.e = trajGen_B.b - 1;
    }

    trajGen_B.waypoints[0] = trajGen_B.waypoints_g[trajGen_B.waypoints_tmp];
    trajGen_B.waypoints[4] = trajGen_B.waypoints_g[trajGen_B.waypoints_tmp + 2];
    trajGen_B.waypoints[8] = trajGen_B.waypoints_g[trajGen_B.waypoints_tmp + 4];
    d_0 = static_cast<int8_T>(trajGen_B.c - trajGen_B.e);
    d[0] = d_0;
    trajGen_B.loop_ub_d = d_0;
    for (trajGen_B.c = 0; trajGen_B.c < 3; trajGen_B.c++) {
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < trajGen_B.loop_ub_d;
           trajGen_B.powerTerm++) {
        /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
        trajGen_B.constraints[(trajGen_B.e + trajGen_B.powerTerm) + (trajGen_B.c
          << 3)] = trajGen_B.waypoints[d[0] * trajGen_B.c + trajGen_B.powerTerm];
      }
    }
  }

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  trajGen_B.tmp1 = timePoints[1] - timePoints[0];
  for (trajGen_B.waypoints_tmp = 0; trajGen_B.waypoints_tmp < 3;
       trajGen_B.waypoints_tmp++) {
    for (trajGen_B.c = 0; trajGen_B.c < 8; trajGen_B.c++) {
      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.x_tmp[trajGen_B.c] = !std::isnan(trajGen_B.constraints
        [(trajGen_B.waypoints_tmp << 3) + trajGen_B.c]);
    }

    trajGen_B.nz = trajGen_B.x_tmp[0];

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    for (trajGen_B.loop_ub_d = 0; trajGen_B.loop_ub_d < 7; trajGen_B.loop_ub_d++)
    {
      trajGen_B.nz += trajGen_B.x_tmp[trajGen_B.loop_ub_d + 1];
    }

    trajGen_B.prod = 1.0;
    std::memset(&trajGen_B.Q[0], 0, sizeof(real_T) << 6U);
    std::memset(&trajGen_B.Qsum[0], 0, sizeof(real_T) << 6U);
    for (trajGen_B.loop_ub_d = 0; trajGen_B.loop_ub_d < 8; trajGen_B.loop_ub_d++)
    {
      trajGen_B.e = 8 - trajGen_B.loop_ub_d;
      for (trajGen_B.b = 0; trajGen_B.b < trajGen_B.e; trajGen_B.b++) {
        trajGen_B.row = trajGen_B.loop_ub_d + trajGen_B.b;
        for (trajGen_B.c = 0; trajGen_B.c < trajGen_B.e; trajGen_B.c++) {
          trajGen_B.col = trajGen_B.loop_ub_d + trajGen_B.c;
          for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm <
               trajGen_B.loop_ub_d; trajGen_B.powerTerm++) {
            /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
            trajGen_B.prod = static_cast<real_T>(trajGen_B.row -
              trajGen_B.powerTerm) * trajGen_B.prod * static_cast<real_T>
              (trajGen_B.col - trajGen_B.powerTerm);
          }

          trajGen_B.powerTerm = ((trajGen_B.row + trajGen_B.col) -
            (trajGen_B.loop_ub_d << 1)) + 1;

          /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
          trajGen_B.Q[trajGen_B.row + (trajGen_B.col << 3)] = trajGen_B.prod *
            rt_powd_snf(trajGen_B.tmp1, static_cast<real_T>(trajGen_B.powerTerm))
            / static_cast<real_T>(trajGen_B.powerTerm);
          trajGen_B.prod = 1.0;
        }
      }

      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.c = tmp[trajGen_B.loop_ub_d];
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < 64;
           trajGen_B.powerTerm++) {
        /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
        trajGen_B.Qsum[trajGen_B.powerTerm] += static_cast<real_T>(trajGen_B.c) *
          trajGen_B.Q[trajGen_B.powerTerm];
        trajGen_B.Q[trajGen_B.powerTerm] = 0.0;
      }
    }

    std::memset(&trajGen_B.A0[0], 0, sizeof(real_T) << 5U);
    for (trajGen_B.c = 0; trajGen_B.c < 4; trajGen_B.c++) {
      trajGen_B.prod = 1.0;

      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < trajGen_B.c;
           trajGen_B.powerTerm++) {
        trajGen_B.prod *= static_cast<real_T>(trajGen_B.c - trajGen_B.powerTerm);
      }

      trajGen_B.A0[trajGen_B.c + (trajGen_B.c << 2)] = trajGen_B.prod;
    }

    for (trajGen_B.b = 0; trajGen_B.b < 4; trajGen_B.b++) {
      for (trajGen_B.c = 0; trajGen_B.c < 8; trajGen_B.c++) {
        trajGen_B.prod = 1.0;

        /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
        for (trajGen_B.loop_ub_d = 0; trajGen_B.loop_ub_d < trajGen_B.b;
             trajGen_B.loop_ub_d++) {
          trajGen_B.prod *= static_cast<real_T>(trajGen_B.c -
            trajGen_B.loop_ub_d);
        }

        trajGen_B.AT[trajGen_B.b + (trajGen_B.c << 2)] = rt_powd_snf
          (trajGen_B.tmp1, static_cast<real_T>(trajGen_B.c - trajGen_B.b)) *
          trajGen_B.prod;
      }
    }

    for (trajGen_B.c = 0; trajGen_B.c < 8; trajGen_B.c++) {
      trajGen_B.loop_ub_d = trajGen_B.c << 2;
      trajGen_B.powerTerm = trajGen_B.c << 3;
      trajGen_B.Q[trajGen_B.powerTerm] = trajGen_B.A0[trajGen_B.loop_ub_d];
      trajGen_B.Q[trajGen_B.powerTerm + 4] = trajGen_B.AT[trajGen_B.loop_ub_d];
      trajGen_B.Q[trajGen_B.powerTerm + 1] = trajGen_B.A0[trajGen_B.loop_ub_d +
        1];
      trajGen_B.Q[trajGen_B.powerTerm + 5] = trajGen_B.AT[trajGen_B.loop_ub_d +
        1];
      trajGen_B.Q[trajGen_B.powerTerm + 2] = trajGen_B.A0[trajGen_B.loop_ub_d +
        2];
      trajGen_B.Q[trajGen_B.powerTerm + 6] = trajGen_B.AT[trajGen_B.loop_ub_d +
        2];
      trajGen_B.Q[trajGen_B.powerTerm + 3] = trajGen_B.A0[trajGen_B.loop_ub_d +
        3];
      trajGen_B.Q[trajGen_B.powerTerm + 7] = trajGen_B.AT[trajGen_B.loop_ub_d +
        3];
    }

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.b = trajGen_B.waypoints_tmp << 3;
    trajGen_constructM(&trajGen_B.constraints[trajGen_B.b], trajGen_B.M);
    for (trajGen_B.c = 0; trajGen_B.c < 4; trajGen_B.c++) {
      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.loop_ub_d = trajGen_B.c << 3;
      trajGen_B.powerTerm = trajGen_B.c << 2;

      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.Q_p[trajGen_B.powerTerm] = trajGen_B.Q[trajGen_B.loop_ub_d];
      trajGen_B.Q_p[trajGen_B.powerTerm + 1] = trajGen_B.Q[trajGen_B.loop_ub_d +
        1];
      trajGen_B.Q_p[trajGen_B.powerTerm + 2] = trajGen_B.Q[trajGen_B.loop_ub_d +
        2];
      trajGen_B.Q_p[trajGen_B.powerTerm + 3] = trajGen_B.Q[trajGen_B.loop_ub_d +
        3];
    }

    trajGen_inv(trajGen_B.Q_p, trajGen_B.upperleft);
    for (trajGen_B.c = 0; trajGen_B.c < 4; trajGen_B.c++) {
      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.loop_ub_d = (trajGen_B.c + 4) << 3;
      trajGen_B.powerTerm = trajGen_B.c << 2;

      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.Q_p[trajGen_B.powerTerm] = trajGen_B.Q[trajGen_B.loop_ub_d + 4];
      trajGen_B.Q_p[trajGen_B.powerTerm + 1] = trajGen_B.Q[trajGen_B.loop_ub_d +
        5];
      trajGen_B.Q_p[trajGen_B.powerTerm + 2] = trajGen_B.Q[trajGen_B.loop_ub_d +
        6];
      trajGen_B.Q_p[trajGen_B.powerTerm + 3] = trajGen_B.Q[trajGen_B.loop_ub_d +
        7];
    }

    trajGen_inv(trajGen_B.Q_p, trajGen_B.lowerright);

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    for (trajGen_B.c = 0; trajGen_B.c < 16; trajGen_B.c++) {
      trajGen_B.Q_p[trajGen_B.c] = -trajGen_B.lowerright[trajGen_B.c];
    }

    for (trajGen_B.c = 0; trajGen_B.c < 4; trajGen_B.c++) {
      trajGen_B.prod = trajGen_B.Q_p[trajGen_B.c];
      trajGen_B.lowerright_d = trajGen_B.Q_p[trajGen_B.c + 4];
      trajGen_B.lowerright_g = trajGen_B.Q_p[trajGen_B.c + 8];
      trajGen_B.lowerright_l = trajGen_B.Q_p[trajGen_B.c + 12];
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < 4; trajGen_B.powerTerm
           ++) {
        trajGen_B.loop_ub_d = trajGen_B.powerTerm << 3;
        trajGen_B.lowerright_c[trajGen_B.c + (trajGen_B.powerTerm << 2)] =
          ((trajGen_B.Q[trajGen_B.loop_ub_d + 4] * trajGen_B.prod +
            trajGen_B.Q[trajGen_B.loop_ub_d + 5] * trajGen_B.lowerright_d) +
           trajGen_B.Q[trajGen_B.loop_ub_d + 6] * trajGen_B.lowerright_g) +
          trajGen_B.Q[trajGen_B.loop_ub_d + 7] * trajGen_B.lowerright_l;
      }

      trajGen_B.prod = trajGen_B.lowerright_c[trajGen_B.c + 4];
      trajGen_B.lowerright_d = trajGen_B.lowerright_c[trajGen_B.c];
      trajGen_B.lowerright_g = trajGen_B.lowerright_c[trajGen_B.c + 8];
      trajGen_B.lowerright_l = trajGen_B.lowerright_c[trajGen_B.c + 12];
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < 4; trajGen_B.powerTerm
           ++) {
        trajGen_B.loop_ub_d = trajGen_B.powerTerm << 2;
        trajGen_B.lowerright_f[trajGen_B.c + trajGen_B.loop_ub_d] =
          ((trajGen_B.upperleft[trajGen_B.loop_ub_d + 1] * trajGen_B.prod +
            trajGen_B.upperleft[trajGen_B.loop_ub_d] * trajGen_B.lowerright_d) +
           trajGen_B.upperleft[trajGen_B.loop_ub_d + 2] * trajGen_B.lowerright_g)
          + trajGen_B.upperleft[trajGen_B.loop_ub_d + 3] *
          trajGen_B.lowerright_l;
        trajGen_B.AInv[trajGen_B.powerTerm + (trajGen_B.c << 3)] =
          trajGen_B.upperleft[(trajGen_B.c << 2) + trajGen_B.powerTerm];
        trajGen_B.AInv[trajGen_B.powerTerm + ((trajGen_B.c + 4) << 3)] = 0.0;
      }
    }

    for (trajGen_B.c = 0; trajGen_B.c < 4; trajGen_B.c++) {
      trajGen_B.loop_ub_d = trajGen_B.c << 2;
      trajGen_B.powerTerm = trajGen_B.c << 3;
      trajGen_B.AInv[trajGen_B.powerTerm + 4] =
        trajGen_B.lowerright_f[trajGen_B.loop_ub_d];
      trajGen_B.e = (trajGen_B.c + 4) << 3;
      trajGen_B.AInv[trajGen_B.e + 4] = trajGen_B.lowerright[trajGen_B.loop_ub_d];
      trajGen_B.AInv[trajGen_B.powerTerm + 5] =
        trajGen_B.lowerright_f[trajGen_B.loop_ub_d + 1];
      trajGen_B.AInv[trajGen_B.e + 5] = trajGen_B.lowerright[trajGen_B.loop_ub_d
        + 1];
      trajGen_B.AInv[trajGen_B.powerTerm + 6] =
        trajGen_B.lowerright_f[trajGen_B.loop_ub_d + 2];
      trajGen_B.AInv[trajGen_B.e + 6] = trajGen_B.lowerright[trajGen_B.loop_ub_d
        + 2];
      trajGen_B.AInv[trajGen_B.powerTerm + 7] =
        trajGen_B.lowerright_f[trajGen_B.loop_ub_d + 3];
      trajGen_B.AInv[trajGen_B.e + 7] = trajGen_B.lowerright[trajGen_B.loop_ub_d
        + 3];
    }

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    for (trajGen_B.c = 0; trajGen_B.c < 8; trajGen_B.c++) {
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < 8; trajGen_B.powerTerm
           ++) {
        trajGen_B.e = trajGen_B.c << 3;
        trajGen_B.row = trajGen_B.powerTerm << 3;
        trajGen_B.col = trajGen_B.row + trajGen_B.c;
        trajGen_B.R_tmp[trajGen_B.powerTerm + trajGen_B.e] =
          trajGen_B.M[trajGen_B.col];
        trajGen_B.prod = 0.0;
        for (trajGen_B.loop_ub_d = 0; trajGen_B.loop_ub_d < 8;
             trajGen_B.loop_ub_d++) {
          trajGen_B.prod += trajGen_B.AInv[trajGen_B.e + trajGen_B.loop_ub_d] *
            trajGen_B.Qsum[trajGen_B.row + trajGen_B.loop_ub_d];
        }

        trajGen_B.AInv_k[trajGen_B.col] = trajGen_B.prod;
      }

      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < 8; trajGen_B.powerTerm
           ++) {
        trajGen_B.prod = 0.0;
        for (trajGen_B.loop_ub_d = 0; trajGen_B.loop_ub_d < 8;
             trajGen_B.loop_ub_d++) {
          trajGen_B.prod += trajGen_B.AInv_k[(trajGen_B.loop_ub_d << 3) +
            trajGen_B.c] * trajGen_B.AInv[(trajGen_B.powerTerm << 3) +
            trajGen_B.loop_ub_d];
        }

        trajGen_B.AInv_c[trajGen_B.c + (trajGen_B.powerTerm << 3)] =
          trajGen_B.prod;
      }
    }

    for (trajGen_B.c = 0; trajGen_B.c < 8; trajGen_B.c++) {
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < 8; trajGen_B.powerTerm
           ++) {
        trajGen_B.prod = 0.0;
        for (trajGen_B.loop_ub_d = 0; trajGen_B.loop_ub_d < 8;
             trajGen_B.loop_ub_d++) {
          trajGen_B.prod += trajGen_B.M[(trajGen_B.loop_ub_d << 3) + trajGen_B.c]
            * trajGen_B.AInv_c[(trajGen_B.powerTerm << 3) + trajGen_B.loop_ub_d];
        }

        trajGen_B.AInv[trajGen_B.c + (trajGen_B.powerTerm << 3)] =
          trajGen_B.prod;
      }

      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < 8; trajGen_B.powerTerm
           ++) {
        trajGen_B.prod = 0.0;
        for (trajGen_B.loop_ub_d = 0; trajGen_B.loop_ub_d < 8;
             trajGen_B.loop_ub_d++) {
          trajGen_B.prod += trajGen_B.AInv[(trajGen_B.loop_ub_d << 3) +
            trajGen_B.c] * trajGen_B.R_tmp[(trajGen_B.powerTerm << 3) +
            trajGen_B.loop_ub_d];
        }

        trajGen_B.Qsum[trajGen_B.c + (trajGen_B.powerTerm << 3)] =
          trajGen_B.prod;
      }
    }

    if (static_cast<real_T>(trajGen_B.nz) + 1.0 > 8.0) {
      trajGen_B.e = 0;
      trajGen_B.row = 0;
      trajGen_B.col = 0;
      trajGen_B.loop_ub_d = -1;
    } else {
      trajGen_B.e = trajGen_B.nz;
      trajGen_B.row = trajGen_B.nz;
      trajGen_B.col = trajGen_B.nz;
      trajGen_B.loop_ub_d = 7;
    }

    trajGen_B.powerTerm = 0;
    for (trajGen_B.c = 0; trajGen_B.c < 8; trajGen_B.c++) {
      if (trajGen_B.x_tmp[trajGen_B.c]) {
        trajGen_B.powerTerm++;
      }
    }

    trajGen_B.tmp_size_idx_0 = trajGen_B.powerTerm;
    trajGen_B.powerTerm = 0;
    for (trajGen_B.c = 0; trajGen_B.c < 8; trajGen_B.c++) {
      if (trajGen_B.x_tmp[trajGen_B.c]) {
        trajGen_B.tmp_data_l[trajGen_B.powerTerm] = static_cast<int8_T>
          (trajGen_B.c);
        trajGen_B.powerTerm++;
      }
    }

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    if (trajGen_B.nz < 1) {
      trajGen_B.nz = 0;
    }

    trajGen_B.R_size[0] = trajGen_B.nz;
    trajGen_B.loop_ub_d -= trajGen_B.col;
    trajGen_B.R_size[1] = trajGen_B.loop_ub_d + 1;
    for (trajGen_B.c = 0; trajGen_B.c <= trajGen_B.loop_ub_d; trajGen_B.c++) {
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < trajGen_B.nz;
           trajGen_B.powerTerm++) {
        trajGen_B.M[trajGen_B.powerTerm + trajGen_B.nz * trajGen_B.c] =
          trajGen_B.Qsum[((trajGen_B.col + trajGen_B.c) << 3) +
          trajGen_B.powerTerm];
      }
    }

    for (trajGen_B.c = 0; trajGen_B.c < trajGen_B.tmp_size_idx_0; trajGen_B.c++)
    {
      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.b_p[trajGen_B.c] = trajGen_B.constraints[trajGen_B.b +
        trajGen_B.tmp_data_l[trajGen_B.c]];
    }

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.R_size_j[0] = trajGen_B.loop_ub_d + 1;
    trajGen_B.R_size_j[1] = trajGen_B.loop_ub_d + 1;
    for (trajGen_B.c = 0; trajGen_B.c <= trajGen_B.loop_ub_d; trajGen_B.c++) {
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm <= trajGen_B.loop_ub_d;
           trajGen_B.powerTerm++) {
        /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
        trajGen_B.AInv[trajGen_B.powerTerm + (trajGen_B.loop_ub_d + 1) *
          trajGen_B.c] = -trajGen_B.Qsum[(((trajGen_B.row + trajGen_B.c) << 3) +
          trajGen_B.e) + trajGen_B.powerTerm];
      }
    }

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.R_data.set(&trajGen_B.M[0], trajGen_B.R_size[0], trajGen_B.R_size
                         [1]);
    trajGen_B.constraints_data.set(&trajGen_B.b_p[0], trajGen_B.tmp_size_idx_0);
    trajGen_mtimes(trajGen_B.R_data, trajGen_B.constraints_data, trajGen_B.r2);
    trajGen_B.R_data_m.set(&trajGen_B.AInv[0], trajGen_B.R_size_j[0],
      trajGen_B.R_size_j[1]);

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_mldivide(trajGen_B.R_data_m, trajGen_B.r2, trajGen_B.r1);
    for (trajGen_B.c = 0; trajGen_B.c < trajGen_B.tmp_size_idx_0; trajGen_B.c++)
    {
      trajGen_B.upperleft[trajGen_B.c] = trajGen_B.constraints[trajGen_B.b +
        trajGen_B.tmp_data_l[trajGen_B.c]];
    }

    trajGen_B.loop_ub_d = trajGen_B.r1.size(0);
    for (trajGen_B.c = 0; trajGen_B.c < trajGen_B.loop_ub_d; trajGen_B.c++) {
      trajGen_B.upperleft[trajGen_B.c + trajGen_B.tmp_size_idx_0] =
        trajGen_B.r1[trajGen_B.c];
    }

    for (trajGen_B.c = 0; trajGen_B.c < 8; trajGen_B.c++) {
      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.prod = 0.0;
      for (trajGen_B.powerTerm = 0; trajGen_B.powerTerm < 8; trajGen_B.powerTerm
           ++) {
        /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
        trajGen_B.prod += trajGen_B.R_tmp[(trajGen_B.powerTerm << 3) +
          trajGen_B.c] * trajGen_B.upperleft[trajGen_B.powerTerm];
      }

      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      trajGen_B.b_p[trajGen_B.c] = trajGen_B.prod;
    }

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_mldivide_pv(trajGen_B.Q, trajGen_B.b_p);
    for (trajGen_B.c = 0; trajGen_B.c < 8; trajGen_B.c++) {
      /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
      ppMatrix[trajGen_B.c + trajGen_B.b] = trajGen_B.b_p[7 - trajGen_B.c];
    }

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_B.nz = (trajGen_rcond(trajGen_B.Q) < 1.0E-10);
  }

  toaZero.set_size(1, 2);

  /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  toaZero[0] = timePoints[0];
  toaZero[1] = timePoints[1];
  *maxIterStatus = 0.0;
  *maxTimeStatus = 0.0;
  *singularityStatus = trajGen_B.nz;
}

static void trajGen_binary_expand_op(real_T in1[2], const
  shared_uav_rst_sluav_internal_T *in2)
{
  /* Outputs for Enabled SubSystem: '<Root>/Subsystem' incorporates:
   *  EnablePort: '<S7>/Enable'
   */
  /* MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  in1[0] = in2->TimeOfArrZero[0] + in2->TimeOffset;
  in1[1] = in2->TimeOfArrZero[static_cast<int32_T>(in2->TimeOfArrZero.size(1) !=
    1)] + in2->TimeOffset;

  /* End of Outputs for SubSystem: '<Root>/Subsystem' */
}

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

/* Model step function */
void trajGen_step(void)
{
  SL_Bus_trajGen_std_msgs_Bool rtb_SourceBlock_o2_i_0;
  boolean_T b_varargout_1;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T p;
  boolean_T p_0;
  boolean_T rtb_FixPtRelationalOperator;

  /* BusAssignment: '<Root>/Bus Assignment' */
  std::memset(&trajGen_B.BusAssignment, 0, sizeof
              (SL_Bus_trajGen_geometry_msgs_Twist));

  /* Outputs for Atomic SubSystem: '<Root>/Subscribe1' */
  /* MATLABSystem: '<S6>/SourceBlock' */
  b_varargout_1 = Sub_trajGen_322.getLatestMessage(&rtb_SourceBlock_o2_i_0);

  /* Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S12>/Enable'
   */
  /* Start for MATLABSystem: '<S6>/SourceBlock' */
  if (b_varargout_1) {
    /* SignalConversion generated from: '<S12>/In1' */
    trajGen_B.In1_c = rtb_SourceBlock_o2_i_0;
  }

  /* End of Start for MATLABSystem: '<S6>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S6>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Subscribe1' */

  /* RelationalOperator: '<S2>/FixPt Relational Operator' incorporates:
   *  Constant: '<S10>/Constant'
   *  RelationalOperator: '<S10>/Compare'
   *  UnitDelay: '<S2>/Delay Input1'
   */
  rtb_FixPtRelationalOperator = (static_cast<int32_T>(trajGen_B.In1_c.Data) >
    static_cast<int32_T>(trajGen_DW.DelayInput1_DSTATE));

  /* Outputs for Atomic SubSystem: '<Root>/Subscribe' */
  /* MATLABSystem: '<S5>/SourceBlock' */
  b_varargout_1 = Sub_trajGen_321.getLatestMessage
    (&trajGen_B.rtb_SourceBlock_o2_p_m);

  /* Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S11>/Enable'
   */
  /* Start for MATLABSystem: '<S5>/SourceBlock' */
  if (b_varargout_1) {
    /* SignalConversion generated from: '<S11>/In1' */
    trajGen_B.In1_h = trajGen_B.rtb_SourceBlock_o2_p_m;
  }

  /* End of Start for MATLABSystem: '<S5>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S5>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Subscribe' */

  /* Outputs for Atomic SubSystem: '<Root>/position reading' */
  /* MATLABSystem: '<S9>/SourceBlock' */
  b_varargout_1 = Sub_trajGen_377.getLatestMessage
    (&trajGen_B.rtb_SourceBlock_o2_c);

  /* Outputs for Enabled SubSystem: '<S9>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S175>/Enable'
   */
  /* Start for MATLABSystem: '<S9>/SourceBlock' */
  if (b_varargout_1) {
    /* SignalConversion generated from: '<S175>/In1' */
    trajGen_B.In1 = trajGen_B.rtb_SourceBlock_o2_c;
  }

  /* End of Start for MATLABSystem: '<S9>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S9>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/position reading' */

  /* MATLABSystem: '<Root>/Get Parameter' */
  ParamGet_trajGen_317.get_parameter(&trajGen_B.delT);

  /* Outputs for Enabled SubSystem: '<Root>/Subsystem' incorporates:
   *  EnablePort: '<S7>/Enable'
   */
  if (trajGen_B.In1_c.Data) {
    trajGen_DW.Subsystem_MODE = true;

    /* DiscreteIntegrator: '<S7>/Discrete-Time Integrator' */
    if (rtb_FixPtRelationalOperator &&
        (trajGen_DW.DiscreteTimeIntegrator_PrevRese <= 0)) {
      trajGen_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
    }

    /* MATLAB Function: '<S7>/MATLAB Function1' incorporates:
     *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
     *  SignalConversion generated from: '<S7>/Transpose1'
     *  SignalConversion generated from: '<S7>/Transpose2'
     */
    trajGen_B.scale = 3.3121686421112381E-170;
    trajGen_B.absxk = std::abs(trajGen_B.In1_h.X -
      trajGen_B.In1.Pose.Pose.Position.X);
    if (trajGen_B.absxk > 3.3121686421112381E-170) {
      trajGen_B.distanceToGoal = 1.0;
      trajGen_B.scale = trajGen_B.absxk;
    } else {
      trajGen_B.t = trajGen_B.absxk / 3.3121686421112381E-170;
      trajGen_B.distanceToGoal = trajGen_B.t * trajGen_B.t;
    }

    trajGen_B.absxk = std::abs(trajGen_B.In1_h.Y -
      trajGen_B.In1.Pose.Pose.Position.Y);
    if (trajGen_B.absxk > trajGen_B.scale) {
      trajGen_B.t = trajGen_B.scale / trajGen_B.absxk;
      trajGen_B.distanceToGoal = trajGen_B.distanceToGoal * trajGen_B.t *
        trajGen_B.t + 1.0;
      trajGen_B.scale = trajGen_B.absxk;
    } else {
      trajGen_B.t = trajGen_B.absxk / trajGen_B.scale;
      trajGen_B.distanceToGoal += trajGen_B.t * trajGen_B.t;
    }

    trajGen_B.absxk = std::abs(trajGen_B.In1_h.Z -
      trajGen_B.In1.Pose.Pose.Position.Z);
    if (trajGen_B.absxk > trajGen_B.scale) {
      trajGen_B.t = trajGen_B.scale / trajGen_B.absxk;
      trajGen_B.distanceToGoal = trajGen_B.distanceToGoal * trajGen_B.t *
        trajGen_B.t + 1.0;
      trajGen_B.scale = trajGen_B.absxk;
    } else {
      trajGen_B.t = trajGen_B.absxk / trajGen_B.scale;
      trajGen_B.distanceToGoal += trajGen_B.t * trajGen_B.t;
    }

    trajGen_B.distanceToGoal = trajGen_B.scale * std::sqrt
      (trajGen_B.distanceToGoal);
    if (!trajGen_DW.startPosition_not_empty) {
      trajGen_DW.startPosition[0] = trajGen_B.In1.Pose.Pose.Position.X;
      trajGen_DW.startPosition[1] = trajGen_B.In1.Pose.Pose.Position.Y;
      trajGen_DW.startPosition[2] = trajGen_B.In1.Pose.Pose.Position.Z;
      trajGen_DW.startPosition_not_empty = true;
    }

    if (trajGen_DW.DiscreteTimeIntegrator_DSTATE == 0.0) {
      trajGen_DW.startPosition[0] = trajGen_B.In1.Pose.Pose.Position.X;
      trajGen_DW.startPosition[1] = trajGen_B.In1.Pose.Pose.Position.Y;
      trajGen_DW.startPosition[2] = trajGen_B.In1.Pose.Pose.Position.Z;
    }

    /* Math: '<S7>/Transpose' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function'
     *  MATLAB Function: '<S7>/MATLAB Function1'
     *  SignalConversion generated from: '<S7>/Transpose2'
     */
    trajGen_B.rtb_Transpose_tmp[0] = trajGen_DW.startPosition[0];
    trajGen_B.rtb_Transpose_tmp[3] = trajGen_B.In1_h.X;
    trajGen_B.rtb_Transpose_tmp[1] = trajGen_DW.startPosition[1];
    trajGen_B.rtb_Transpose_tmp[4] = trajGen_B.In1_h.Y;
    trajGen_B.rtb_Transpose_tmp[2] = trajGen_DW.startPosition[2];
    trajGen_B.rtb_Transpose_tmp[5] = trajGen_B.In1_h.Z;

    /* MATLAB Function: '<S7>/MATLAB Function' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function1'
     *  MATLABSystem: '<Root>/Get Parameter'
     *  SignalConversion generated from: '<S7>/Transpose2'
     * */
    trajGen_B.Transpose1[0] = trajGen_B.In1_h.X - trajGen_DW.startPosition[0];
    trajGen_B.Transpose1[1] = trajGen_B.In1_h.Y - trajGen_DW.startPosition[1];
    trajGen_B.Transpose1[2] = trajGen_B.In1_h.Z - trajGen_DW.startPosition[2];
    trajGen_B.timePoints[0] = 0.0 / trajGen_B.delT;
    trajGen_B.timePoints[1] = std::sqrt((trajGen_B.Transpose1[0] *
      trajGen_B.Transpose1[0] + trajGen_B.Transpose1[1] * trajGen_B.Transpose1[1])
      + trajGen_B.Transpose1[2] * trajGen_B.Transpose1[2]) / trajGen_B.delT;
    trajGen_B.timePoints[1] += trajGen_B.timePoints[0];

    /* MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' incorporates:
     *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
     *  MATLAB Function: '<S7>/MATLAB Function1'
     *  Math: '<S7>/Transpose'
     *  SignalConversion generated from: '<S7>/Transpose2'
     */
    guard1 = false;
    if (trajGen_DW.obj.StartFlag) {
      guard1 = true;
    } else {
      b_varargout_1 = false;
      if (!trajGen_isequaln(trajGen_B.rtb_Transpose_tmp,
                            trajGen_DW.obj.PrevWaypoint)) {
        b_varargout_1 = true;
      } else {
        p = false;
        p_0 = true;
        trajGen_B.b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (trajGen_B.b_k < 2)) {
          if ((trajGen_B.timePoints[trajGen_B.b_k] ==
               trajGen_DW.obj.PrevTimePoint[trajGen_B.b_k]) || (std::isnan
               (trajGen_B.timePoints[trajGen_B.b_k]) && std::isnan
               (trajGen_DW.obj.PrevTimePoint[trajGen_B.b_k]))) {
            trajGen_B.b_k++;
          } else {
            p_0 = false;
            exitg1 = true;
          }
        }

        if (p_0) {
          p = true;
        }

        if (!p) {
          b_varargout_1 = true;
        } else {
          for (trajGen_B.b_k = 0; trajGen_B.b_k < 6; trajGen_B.b_k++) {
            trajGen_B.dv[trajGen_B.b_k] = 0.0;
          }

          if (!trajGen_isequaln(trajGen_B.dv, trajGen_DW.obj.PrevVelBC)) {
            b_varargout_1 = true;
          } else {
            for (trajGen_B.b_k = 0; trajGen_B.b_k < 6; trajGen_B.b_k++) {
              trajGen_B.dv[trajGen_B.b_k] = 0.0;
            }

            if (!trajGen_isequaln(trajGen_B.dv, trajGen_DW.obj.PrevAccelBC)) {
              b_varargout_1 = true;
            } else {
              for (trajGen_B.b_k = 0; trajGen_B.b_k < 6; trajGen_B.b_k++) {
                trajGen_B.dv[trajGen_B.b_k] = 0.0;
              }

              if (!trajGen_isequaln(trajGen_B.dv, trajGen_DW.obj.PrevJerkBC)) {
                b_varargout_1 = true;
              }
            }
          }
        }
      }

      if (b_varargout_1) {
        guard1 = true;
      }
    }

    if (guard1) {
      MinJerkPolyTraj_computePPandTim(&trajGen_DW.obj,
        trajGen_B.rtb_Transpose_tmp, trajGen_B.timePoints, trajGen_B.ppMatrix,
        trajGen_B.r, &trajGen_B.delT, &trajGen_B.scale, &trajGen_B.absxk);
      trajGen_DW.obj.SingularityStatus = trajGen_B.absxk;
      trajGen_DW.obj.MaxTimeStatus = trajGen_B.scale;
      trajGen_DW.obj.MaxIterStatus = trajGen_B.delT;
      trajGen_DW.obj.TimeOfArrZero.set_size(1, trajGen_B.r.size(1));
      trajGen_B.loop_ub = trajGen_B.r.size(0) * trajGen_B.r.size(1) - 1;
      for (trajGen_B.b_k = 0; trajGen_B.b_k <= trajGen_B.loop_ub; trajGen_B.b_k
           ++) {
        trajGen_DW.obj.TimeOfArrZero[trajGen_B.b_k] = trajGen_B.r[trajGen_B.b_k];
      }

      trajGen_DW.obj.PPMatrix.set_size(1, 8, 3);
      for (trajGen_B.b_k = 0; trajGen_B.b_k < 24; trajGen_B.b_k++) {
        trajGen_DW.obj.PPMatrix[trajGen_B.b_k] =
          trajGen_B.ppMatrix[trajGen_B.b_k];
      }

      trajGen_DW.obj.StartFlag = false;
    }

    if (trajGen_DW.obj.TimeOfArrZero.size(1) == 2) {
      trajGen_B.loop_ub = trajGen_DW.obj.TimeOfArrZero.size(1);
      for (trajGen_B.b_k = 0; trajGen_B.b_k < trajGen_B.loop_ub; trajGen_B.b_k++)
      {
        trajGen_B.dv2[trajGen_B.b_k] =
          trajGen_DW.obj.TimeOfArrZero[trajGen_B.b_k] +
          trajGen_DW.obj.TimeOffset;
      }
    } else {
      trajGen_binary_expand_op(trajGen_B.dv2, &trajGen_DW.obj);
    }

    if (trajGen_DW.DiscreteTimeIntegrator_DSTATE < trajGen_B.dv2[0]) {
      trajGen_B.Transpose1[0] = trajGen_DW.startPosition[0];
      trajGen_B.Transpose1[1] = trajGen_DW.startPosition[1];
      trajGen_B.Transpose1[2] = trajGen_DW.startPosition[2];
    } else if (trajGen_DW.DiscreteTimeIntegrator_DSTATE > trajGen_B.dv2[1]) {
      trajGen_B.Transpose1[0] = trajGen_B.In1_h.X;
      trajGen_B.Transpose1[1] = trajGen_B.In1_h.Y;
      trajGen_B.Transpose1[2] = trajGen_B.In1_h.Z;
    } else {
      trajGen_B.delT = trajGen_DW.DiscreteTimeIntegrator_DSTATE - trajGen_B.dv2
        [0];
      for (trajGen_B.b_k = 0; trajGen_B.b_k < 3; trajGen_B.b_k++) {
        if (std::isnan(trajGen_B.delT)) {
          trajGen_B.scale = (rtNaN);
        } else {
          trajGen_B.scale = trajGen_DW.obj.PPMatrix
            [(trajGen_DW.obj.PPMatrix.size(0) << 3) * trajGen_B.b_k];
          for (trajGen_B.loop_ub = 0; trajGen_B.loop_ub < 7; trajGen_B.loop_ub++)
          {
            trajGen_B.scale = trajGen_DW.obj.PPMatrix[(trajGen_B.loop_ub + 1) *
              trajGen_DW.obj.PPMatrix.size(0) + (trajGen_DW.obj.PPMatrix.size(0)
              << 3) * trajGen_B.b_k] + trajGen_B.delT * trajGen_B.scale;
          }
        }

        trajGen_B.Transpose1[trajGen_B.b_k] = trajGen_B.scale;
      }
    }

    trajGen_DW.obj.PrevTimePoint[0] = trajGen_B.timePoints[0];
    trajGen_DW.obj.PrevTimePoint[1] = trajGen_B.timePoints[1];
    for (trajGen_B.b_k = 0; trajGen_B.b_k < 6; trajGen_B.b_k++) {
      trajGen_DW.obj.PrevWaypoint[trajGen_B.b_k] =
        trajGen_B.rtb_Transpose_tmp[trajGen_B.b_k];
      trajGen_DW.obj.PrevVelBC[trajGen_B.b_k] = 0.0;
      trajGen_DW.obj.PrevAccelBC[trajGen_B.b_k] = 0.0;
      trajGen_DW.obj.PrevJerkBC[trajGen_B.b_k] = 0.0;
    }

    /* MATLABSystem: '<S7>/Coordinate Transformation Conversion' incorporates:
     *  SignalConversion generated from: '<S7>/Vector Concatenate'
     * */
    trajGen_B.t = 1.0 / std::sqrt(((trajGen_B.In1.Pose.Pose.Orientation.W *
      trajGen_B.In1.Pose.Pose.Orientation.W +
      trajGen_B.In1.Pose.Pose.Orientation.X *
      trajGen_B.In1.Pose.Pose.Orientation.X) +
      trajGen_B.In1.Pose.Pose.Orientation.Y *
      trajGen_B.In1.Pose.Pose.Orientation.Y) +
      trajGen_B.In1.Pose.Pose.Orientation.Z *
      trajGen_B.In1.Pose.Pose.Orientation.Z);
    trajGen_B.delT = trajGen_B.In1.Pose.Pose.Orientation.W * trajGen_B.t;
    trajGen_B.scale = trajGen_B.In1.Pose.Pose.Orientation.X * trajGen_B.t;
    trajGen_B.absxk = trajGen_B.In1.Pose.Pose.Orientation.Y * trajGen_B.t;
    trajGen_B.t *= trajGen_B.In1.Pose.Pose.Orientation.Z;
    trajGen_B.UngainTsProdOut_h = trajGen_B.scale * trajGen_B.t * 2.0 -
      trajGen_B.delT * trajGen_B.absxk * 2.0;
    if (trajGen_B.UngainTsProdOut_h > 1.0) {
      trajGen_B.UngainTsProdOut_h = 1.0;
    }

    if (trajGen_B.UngainTsProdOut_h < -1.0) {
      trajGen_B.UngainTsProdOut_h = -1.0;
    }

    if ((trajGen_B.UngainTsProdOut_h < 0.0) && (std::abs
         (trajGen_B.UngainTsProdOut_h + 1.0) < 2.2204460492503131E-15)) {
      trajGen_B.delT = -2.0 * rt_atan2d_snf(trajGen_B.scale, trajGen_B.delT);
    } else if ((trajGen_B.UngainTsProdOut_h > 0.0) && (std::abs
                (trajGen_B.UngainTsProdOut_h - 1.0) < 2.2204460492503131E-15)) {
      trajGen_B.delT = 2.0 * rt_atan2d_snf(trajGen_B.scale, trajGen_B.delT);
    } else {
      trajGen_B.delT = rt_atan2d_snf(trajGen_B.delT * trajGen_B.t * 2.0 +
        trajGen_B.scale * trajGen_B.absxk * 2.0, (trajGen_B.delT *
        trajGen_B.delT * 2.0 - 1.0) + trajGen_B.scale * trajGen_B.scale * 2.0);
    }

    /* Sum: '<S7>/Sum' incorporates:
     *  MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory'
     */
    trajGen_B.scale = trajGen_B.Transpose1[0] -
      trajGen_B.In1.Pose.Pose.Position.X;

    /* Sum: '<S7>/Sum1' incorporates:
     *  MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory'
     */
    trajGen_B.absxk = trajGen_B.Transpose1[1] -
      trajGen_B.In1.Pose.Pose.Position.Y;

    /* MATLAB Function: '<S7>/MATLAB Function2' incorporates:
     *  MATLABSystem: '<S7>/Coordinate Transformation Conversion'
     * */
    trajGen_B.t = std::sin(-trajGen_B.delT);
    trajGen_B.UngainTsProdOut_h = std::cos(-trajGen_B.delT);
    trajGen_B.delT = trajGen_B.UngainTsProdOut_h * trajGen_B.scale - trajGen_B.t
      * trajGen_B.absxk;
    trajGen_B.scale = trajGen_B.t * trajGen_B.scale +
      trajGen_B.UngainTsProdOut_h * trajGen_B.absxk;

    /* Gain: '<S108>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S100>/Filter'
     *  Gain: '<S98>/Derivative Gain'
     *  Sum: '<S100>/SumD'
     */
    trajGen_B.absxk = (0.08 * trajGen_B.scale - trajGen_DW.Filter_DSTATE) * 3.0;

    /* Gain: '<S7>/Gain1' incorporates:
     *  DiscreteIntegrator: '<S105>/Integrator'
     *  Gain: '<S110>/Proportional Gain'
     *  Sum: '<S114>/Sum'
     */
    trajGen_B.Gain1 = -((0.7 * trajGen_B.scale + trajGen_DW.Integrator_DSTATE) +
                        trajGen_B.absxk);

    /* Gain: '<S56>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S48>/Filter'
     *  Gain: '<S46>/Derivative Gain'
     *  Sum: '<S48>/SumD'
     */
    trajGen_B.t = (0.08 * trajGen_B.delT - trajGen_DW.Filter_DSTATE_j) * 3.0;

    /* Sum: '<S62>/Sum' incorporates:
     *  DiscreteIntegrator: '<S53>/Integrator'
     *  Gain: '<S58>/Proportional Gain'
     */
    trajGen_B.Sum = (0.7 * trajGen_B.delT + trajGen_DW.Integrator_DSTATE_h) +
      trajGen_B.t;

    /* Product: '<S67>/Ungain*Ts Prod Out' */
    trajGen_B.UngainTsProdOut_h = trajGen_B.t * 0.0;

    /* Sum: '<S7>/Sum2' incorporates:
     *  MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory'
     */
    trajGen_B.t = trajGen_B.Transpose1[2] - trajGen_B.In1.Pose.Pose.Position.Z;

    /* Gain: '<S160>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S152>/Filter'
     *  Gain: '<S150>/Derivative Gain'
     *  Sum: '<S152>/SumD'
     */
    trajGen_B.FilterCoefficient = (0.08 * trajGen_B.t -
      trajGen_DW.Filter_DSTATE_b) * 3.0;

    /* Gain: '<S7>/Gain2' incorporates:
     *  DiscreteIntegrator: '<S157>/Integrator'
     *  Gain: '<S162>/Proportional Gain'
     *  Sum: '<S166>/Sum'
     */
    trajGen_B.Gain2 = -((0.7 * trajGen_B.t + trajGen_DW.Integrator_DSTATE_hi) +
                        trajGen_B.FilterCoefficient);

    /* Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S7>/Constant1'
     */
    trajGen_DW.DiscreteTimeIntegrator_DSTATE += 0.01;
    trajGen_DW.DiscreteTimeIntegrator_PrevRese = static_cast<int8_T>
      (rtb_FixPtRelationalOperator);

    /* Update for DiscreteIntegrator: '<S105>/Integrator' incorporates:
     *  Gain: '<S102>/Integral Gain'
     *  Product: '<S118>/Uintegral*Ts Prod Out'
     */
    trajGen_DW.Integrator_DSTATE += 0.0 * trajGen_B.scale * 0.0;

    /* Update for DiscreteIntegrator: '<S100>/Filter' incorporates:
     *  Product: '<S119>/Ungain*Ts Prod Out'
     */
    trajGen_DW.Filter_DSTATE += trajGen_B.absxk * 0.0;

    /* Update for DiscreteIntegrator: '<S48>/Filter' */
    trajGen_DW.Filter_DSTATE_j += trajGen_B.UngainTsProdOut_h;

    /* Update for DiscreteIntegrator: '<S53>/Integrator' incorporates:
     *  Gain: '<S50>/Integral Gain'
     *  Product: '<S66>/Uintegral*Ts Prod Out'
     */
    trajGen_DW.Integrator_DSTATE_h += 0.0 * trajGen_B.delT * 0.0;

    /* Update for DiscreteIntegrator: '<S157>/Integrator' incorporates:
     *  Gain: '<S154>/Integral Gain'
     *  Product: '<S170>/Uintegral*Ts Prod Out'
     */
    trajGen_DW.Integrator_DSTATE_hi += 0.0 * trajGen_B.t * 0.0;

    /* Update for DiscreteIntegrator: '<S152>/Filter' incorporates:
     *  Product: '<S171>/Ungain*Ts Prod Out'
     */
    trajGen_DW.Filter_DSTATE_b += trajGen_B.FilterCoefficient * 0.0;
  } else if (trajGen_DW.Subsystem_MODE) {
    /* Disable for Sum: '<S62>/Sum' incorporates:
     *  Outport: '<S7>/x'
     */
    trajGen_B.Sum = 0.0;

    /* Disable for Gain: '<S7>/Gain1' incorporates:
     *  Outport: '<S7>/y'
     */
    trajGen_B.Gain1 = 0.0;

    /* Disable for Gain: '<S7>/Gain2' incorporates:
     *  Outport: '<S7>/z'
     */
    trajGen_B.Gain2 = 0.0;
    trajGen_DW.Subsystem_MODE = false;
  }

  /* End of Outputs for SubSystem: '<Root>/Subsystem' */

  /* BusAssignment: '<Root>/Bus Assignment2' */
  trajGen_B.BusAssignment2.Data = trajGen_B.distanceToGoal;

  /* Outputs for Atomic SubSystem: '<Root>/Publish1' */
  /* MATLABSystem: '<S4>/SinkBlock' */
  Pub_trajGen_378.publish(&trajGen_B.BusAssignment2);

  /* End of Outputs for SubSystem: '<Root>/Publish1' */

  /* BusAssignment: '<Root>/Bus Assignment' */
  trajGen_B.BusAssignment.Linear.X = trajGen_B.Sum;
  trajGen_B.BusAssignment.Linear.Y = trajGen_B.Gain1;
  trajGen_B.BusAssignment.Linear.Z = trajGen_B.Gain2;

  /* Outputs for Atomic SubSystem: '<Root>/Publish' */
  /* MATLABSystem: '<S3>/SinkBlock' */
  Pub_trajGen_319.publish(&trajGen_B.BusAssignment);

  /* End of Outputs for SubSystem: '<Root>/Publish' */

  /* Update for UnitDelay: '<S2>/Delay Input1' incorporates:
   *  Constant: '<S10>/Constant'
   *  RelationalOperator: '<S10>/Compare'
   */
  trajGen_DW.DelayInput1_DSTATE = trajGen_B.In1_c.Data;
}

/* Model initialize function */
void trajGen_initialize(void)
{
  /* Registration code */

  /* block I/O */
  (void) std::memset((static_cast<void *>(&trajGen_B)), 0,
                     sizeof(B_trajGen_T));

  /* states (dwork) */
  (void) std::memset(static_cast<void *>(&trajGen_DW), 0,
                     sizeof(DW_trajGen_T));

  {
    static const char_T b_zeroDelimTopic_2[23]{ "/tagslam/odom/body_rig" };

    static const char_T b_zeroDelimTopic_4[17]{ "/command/cmd_vel" };

    static const char_T b_zeroDelimTopic_0[16]{ "/trajGen/enable" };

    static const char_T b_zeroDelimTopic_3[16]{ "/distanceToGoal" };

    static const char_T b_zeroDelimName_0[14]{ "/trajGen/velo" };

    static const char_T b_zeroDelimTopic_1[13]{ "/chase_point" };

    real_T tmp[2];
    int32_T i;
    char_T b_zeroDelimName[14];
    char_T b_zeroDelimTopic[13];

    /* Start for Atomic SubSystem: '<Root>/Subscribe1' */
    /* Start for MATLABSystem: '<S6>/SourceBlock' */
    trajGen_DW.obj_m.matlabCodegenIsDeleted = false;
    trajGen_DW.objisempty_c = true;
    trajGen_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      trajGen_B.b_zeroDelimTopic_p[i] = b_zeroDelimTopic_0[i];
    }

    Sub_trajGen_322.createSubscriber(&trajGen_B.b_zeroDelimTopic_p[0], 1);
    trajGen_DW.obj_m.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S6>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/Subscribe1' */

    /* Start for Atomic SubSystem: '<Root>/Subscribe' */
    /* Start for MATLABSystem: '<S5>/SourceBlock' */
    trajGen_DW.obj_kx.matlabCodegenIsDeleted = false;
    trajGen_DW.objisempty_lg = true;
    trajGen_DW.obj_kx.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      b_zeroDelimTopic[i] = b_zeroDelimTopic_1[i];
    }

    Sub_trajGen_321.createSubscriber(&b_zeroDelimTopic[0], 1);
    trajGen_DW.obj_kx.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S5>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/Subscribe' */

    /* Start for Atomic SubSystem: '<Root>/position reading' */
    /* Start for MATLABSystem: '<S9>/SourceBlock' */
    trajGen_DW.obj_k.matlabCodegenIsDeleted = false;
    trajGen_DW.objisempty = true;
    trajGen_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      trajGen_B.b_zeroDelimTopic[i] = b_zeroDelimTopic_2[i];
    }

    Sub_trajGen_377.createSubscriber(&trajGen_B.b_zeroDelimTopic[0], 1);
    trajGen_DW.obj_k.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S9>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/position reading' */

    /* Start for MATLABSystem: '<Root>/Get Parameter' */
    trajGen_DW.obj_o.matlabCodegenIsDeleted = false;
    trajGen_DW.objisempty_f = true;
    trajGen_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      b_zeroDelimName[i] = b_zeroDelimName_0[i];
    }

    ParamGet_trajGen_317.initialize(&b_zeroDelimName[0]);
    ParamGet_trajGen_317.initialize_error_codes(0, 1, 2, 3);
    ParamGet_trajGen_317.set_initial_value(0.0);
    trajGen_DW.obj_o.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<Root>/Get Parameter' */

    /* Start for Enabled SubSystem: '<Root>/Subsystem' */
    trajGen_DW.Subsystem_MODE = false;

    /* Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
    trajGen_DW.objisempty_l = true;
    trajGen_DW.obj.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      trajGen_B.dv1[i] = 0.0;
    }

    tmp[0] = 0.0;
    tmp[1] = 0.0;
    trajG_MinJerkPolyTraj_setupImpl(&trajGen_DW.obj, trajGen_B.dv1, tmp);

    /* End of Start for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */

    /* Start for MATLABSystem: '<S7>/Coordinate Transformation Conversion' */
    trajGen_DW.objisempty_b = true;
    trajGen_DW.obj_f.isInitialized = 1;

    /* End of Start for SubSystem: '<Root>/Subsystem' */

    /* Start for Atomic SubSystem: '<Root>/Publish1' */
    /* Start for MATLABSystem: '<S4>/SinkBlock' */
    trajGen_DW.obj_b.matlabCodegenIsDeleted = false;
    trajGen_DW.objisempty_bt = true;
    trajGen_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      trajGen_B.b_zeroDelimTopic_p[i] = b_zeroDelimTopic_3[i];
    }

    Pub_trajGen_378.createPublisher(&trajGen_B.b_zeroDelimTopic_p[0], 1);
    trajGen_DW.obj_b.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S4>/SinkBlock' */
    /* End of Start for SubSystem: '<Root>/Publish1' */

    /* Start for Atomic SubSystem: '<Root>/Publish' */
    /* Start for MATLABSystem: '<S3>/SinkBlock' */
    trajGen_DW.obj_bu.matlabCodegenIsDeleted = false;
    trajGen_DW.objisempty_a = true;
    trajGen_DW.obj_bu.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      trajGen_B.b_zeroDelimTopic_n[i] = b_zeroDelimTopic_4[i];
    }

    Pub_trajGen_319.createPublisher(&trajGen_B.b_zeroDelimTopic_n[0], 1);
    trajGen_DW.obj_bu.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S3>/SinkBlock' */
    /* End of Start for SubSystem: '<Root>/Publish' */
  }

  /* InitializeConditions for UnitDelay: '<S2>/Delay Input1' */
  trajGen_DW.DelayInput1_DSTATE = false;

  /* SystemInitialize for SignalConversion generated from: '<S12>/In1' */
  std::memset(&trajGen_B.In1_c, 0, sizeof(SL_Bus_trajGen_std_msgs_Bool));

  /* SystemInitialize for SignalConversion generated from: '<S11>/In1' */
  std::memset(&trajGen_B.In1_h, 0, sizeof(SL_Bus_trajGen_geometry_msgs_Point));

  /* SystemInitialize for Enabled SubSystem: '<Root>/Subsystem' */
  /* InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' */
  trajGen_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
  trajGen_DW.DiscreteTimeIntegrator_PrevRese = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S105>/Integrator' */
  trajGen_DW.Integrator_DSTATE = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S100>/Filter' */
  trajGen_DW.Filter_DSTATE = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S48>/Filter' */
  trajGen_DW.Filter_DSTATE_j = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S53>/Integrator' */
  trajGen_DW.Integrator_DSTATE_h = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S157>/Integrator' */
  trajGen_DW.Integrator_DSTATE_hi = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S152>/Filter' */
  trajGen_DW.Filter_DSTATE_b = 0.0;

  /* SystemInitialize for MATLAB Function: '<S7>/MATLAB Function1' */
  trajGen_DW.startPosition_not_empty = false;

  /* InitializeConditions for MATLABSystem: '<S7>/Minimum Jerk Polynomial Trajectory' */
  trajGen_DW.obj.StartFlag = true;

  /* SystemInitialize for Sum: '<S62>/Sum' incorporates:
   *  Outport: '<S7>/x'
   */
  trajGen_B.Sum = 0.0;

  /* SystemInitialize for Gain: '<S7>/Gain1' incorporates:
   *  Outport: '<S7>/y'
   */
  trajGen_B.Gain1 = 0.0;

  /* SystemInitialize for Gain: '<S7>/Gain2' incorporates:
   *  Outport: '<S7>/z'
   */
  trajGen_B.Gain2 = 0.0;

  /* SystemInitialize for Outport: '<S7>/distance' */
  trajGen_B.distanceToGoal = 0.0;

  /* End of SystemInitialize for SubSystem: '<Root>/Subsystem' */

  /* SystemInitialize for SignalConversion generated from: '<S175>/In1' */
  std::memset(&trajGen_B.In1, 0, sizeof(SL_Bus_trajGen_nav_msgs_Odometry));
}

/* Model terminate function */
void trajGen_terminate(void)
{
  /* Terminate for Atomic SubSystem: '<Root>/Subscribe1' */
  /* Terminate for MATLABSystem: '<S6>/SourceBlock' */
  if (!trajGen_DW.obj_m.matlabCodegenIsDeleted) {
    trajGen_DW.obj_m.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S6>/SourceBlock' */
  /* End of Terminate for SubSystem: '<Root>/Subscribe1' */

  /* Terminate for Atomic SubSystem: '<Root>/Subscribe' */
  /* Terminate for MATLABSystem: '<S5>/SourceBlock' */
  if (!trajGen_DW.obj_kx.matlabCodegenIsDeleted) {
    trajGen_DW.obj_kx.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S5>/SourceBlock' */
  /* End of Terminate for SubSystem: '<Root>/Subscribe' */

  /* Terminate for Atomic SubSystem: '<Root>/position reading' */
  /* Terminate for MATLABSystem: '<S9>/SourceBlock' */
  if (!trajGen_DW.obj_k.matlabCodegenIsDeleted) {
    trajGen_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S9>/SourceBlock' */
  /* End of Terminate for SubSystem: '<Root>/position reading' */

  /* Terminate for MATLABSystem: '<Root>/Get Parameter' */
  if (!trajGen_DW.obj_o.matlabCodegenIsDeleted) {
    trajGen_DW.obj_o.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<Root>/Get Parameter' */

  /* Terminate for Atomic SubSystem: '<Root>/Publish1' */
  /* Terminate for MATLABSystem: '<S4>/SinkBlock' */
  if (!trajGen_DW.obj_b.matlabCodegenIsDeleted) {
    trajGen_DW.obj_b.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S4>/SinkBlock' */
  /* End of Terminate for SubSystem: '<Root>/Publish1' */

  /* Terminate for Atomic SubSystem: '<Root>/Publish' */
  /* Terminate for MATLABSystem: '<S3>/SinkBlock' */
  if (!trajGen_DW.obj_bu.matlabCodegenIsDeleted) {
    trajGen_DW.obj_bu.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S3>/SinkBlock' */
  /* End of Terminate for SubSystem: '<Root>/Publish' */
}
