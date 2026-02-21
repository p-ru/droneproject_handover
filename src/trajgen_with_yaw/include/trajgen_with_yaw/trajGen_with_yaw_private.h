/*
 * trajGen_with_yaw_private.h
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

#ifndef trajGen_with_yaw_private_h_
#define trajGen_with_yaw_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "trajGen_with_yaw.h"
#include "trajGen_with_yaw_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void CoordinateTransformationC_Start(DW_CoordinateTransformationCo_T
  *localDW);
extern void CoordinateTransformationConvers(const real_T rtu_0[4],
  B_CoordinateTransformationCon_T *localB);

#endif                                 /* trajGen_with_yaw_private_h_ */
