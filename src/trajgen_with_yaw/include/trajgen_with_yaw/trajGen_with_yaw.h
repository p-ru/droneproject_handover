/*
 * trajGen_with_yaw.h
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

#ifndef trajGen_with_yaw_h_
#define trajGen_with_yaw_h_
#include <cmath>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "slros_initialize.h"
#include "coder_posix_time.h"
#include "trajGen_with_yaw_types.h"
#include "coder_array.h"

extern "C"
{

#include "rtGetNaN.h"

}

extern "C"
{

#include "rtGetInf.h"

}

#include <cstring>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals for system '<S5>/Coordinate Transformation Conversion' */
struct B_CoordinateTransformationCon_T {
  real_T CoordinateTransformationConve_p[3];
                               /* '<S5>/Coordinate Transformation Conversion' */
};

/* Block states (default storage) for system '<S5>/Coordinate Transformation Conversion' */
struct DW_CoordinateTransformationCo_T {
  robotics_slcore_internal_bloc_T obj;
                               /* '<S5>/Coordinate Transformation Conversion' */
  boolean_T objisempty;        /* '<S5>/Coordinate Transformation Conversion' */
};

/* Block signals (default storage) */
struct B_trajGen_with_yaw_T {
  coder::array<real_T,2> r;
  coder::array<real_T,2> R_data;
  coder::array<real_T,2> R_data_m;
  coder::array<real_T,2> b_A;
  coder::array<real_T,1> r1;
  coder::array<real_T,1> r2;
  coder::array<real_T,1> constraints_data;
  coder::array<real_T,1> tau;
  coder::array<real_T,1> B;
  coder::array<real_T,1> work;
  coder::array<real_T,1> vn1;
  coder::array<real_T,1> vn2;
  coder::array<int32_T,2> jpvt;
  SL_Bus_trajGen_with_yaw_nav_msgs_Odometry In1;/* '<S228>/In1' */
  SL_Bus_trajGen_with_yaw_nav_msgs_Odometry rtb_SourceBlock_o2_c;
  real_T M[64];
  real_T AInv[64];
  real_T Q[64];
  real_T Qsum[64];
  real_T R_tmp[64];
  real_T AInv_k[64];
  real_T AInv_c[64];
  real_T c_A[64];
  real_T c_A_b[64];
  real_T ppMatrix[32];
  real_T constraints[32];
  real_T A0[32];
  real_T AT[32];
  real_T upperleft[16];
  real_T lowerright[16];
  real_T Q_p[16];
  real_T lowerright_c[16];
  real_T lowerright_f[16];
  real_T c_A_g[16];
  int8_T M1[64];
  int8_T Mcontinuity[64];
  real_T rtb_Transpose_tmp[8];
  real_T dv[8];
  real_T b_p[8];
  real_T x[8];
  real_T dv1[8];
  SL_Bus_trajGen_with_yaw_geometry_msgs_Pose In1_h;/* '<S9>/In1' */
  SL_Bus_trajGen_with_yaw_geometry_msgs_Pose rtb_SourceBlock_o2_p_g;
  SL_Bus_trajGen_with_yaw_geometry_msgs_Twist BusAssignment;/* '<Root>/Bus Assignment' */
  real_T VectorConcatenate3[4];        /* '<S5>/Vector Concatenate3' */
  int32_T ipiv[8];
  int32_T b_ipiv[8];
  int32_T fixedBCIdx_data[8];
  int32_T freeBCIdx_data[8];
  int32_T tmp_data[8];
  char_T b_zeroDelimTopic[23];
  char_T b_zeroDelimTopic_m[17];
  char_T b_zeroDelimTopic_n[16];
  real_T dv2[2];
  real_T timePoints[2];                /* '<S5>/MATLAB Function' */
  int8_T tmp_data_p[8];
  boolean_T x_tmp[8];
  boolean_T fixedBCIdx_tmp[8];
  boolean_T fixedBCIdx_tmp_j[8];
  real_T Gain3;                        /* '<S5>/Gain3' */
  real_T Sum;                          /* '<S63>/Sum' */
  real_T Gain1;                        /* '<S5>/Gain1' */
  real_T Gain2;                        /* '<S5>/Gain2' */
  real_T delT;
  real_T Sum1;                         /* '<S5>/Sum1' */
  real_T error_x_body;                 /* '<S5>/ convert to body frame' */
  real_T Filter_j;                     /* '<S205>/Filter' */
  real_T Sum3;                         /* '<S5>/Sum3' */
  real_T tmp1;
  real_T prod;
  real_T lowerright_d;
  real_T lowerright_g;
  real_T lowerright_l;
  real_T normA;
  real_T absrexk;
  real_T temp;
  real_T s;
  real_T smax;
  real_T s_d;
  real_T temp_d;
  real_T wj;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T smax_l;
  real_T scale_o;
  real_T absxk_b;
  real_T t_n;
  real_T M_b;
  real_T smax_ln;
  real_T s_h;
  real_T smax_b;
  real_T s_da;
  real_T a;
  real_T b;
  real_T d;
  real_T d1;
  SL_Bus_trajGen_with_yaw_std_msgs_Bool In1_c;/* '<S10>/In1' */
  int32_T R_size[2];
  int32_T R_size_l[2];
  int8_T p[4];
  int8_T b_ipiv_e[4];
  int32_T b_k;
  int32_T loop_ub;
  int32_T b_b;
  int32_T c;
  int32_T e;
  int32_T nz;
  int32_T row;
  int32_T col;
  int32_T powerTerm;
  int32_T loop_ub_j;
  int32_T b_p_tmp;
  int32_T tmp_size_idx_0;
  int32_T iter;
  int32_T kase;
  int32_T jump;
  int32_T j;
  int32_T jA;
  int32_T ix;
  int32_T c_f;
  int32_T b_j;
  int32_T i;
  int32_T jj;
  int32_T b_j_a;
  int32_T c_j;
  int32_T c_jz;
  int32_T a_o;
  int32_T k;
  int32_T jA_n;
  int32_T d_i;
  int32_T temp_tmp;
  int32_T b_info;
  int32_T kAcol;
  int32_T i_o;
  int32_T rankA;
  int32_T mn;
  int32_T m;
  int32_T i_n;
  int32_T kAcol_m;
  int32_T k_c;
  int32_T ii;
  int32_T nmi;
  int32_T pvt;
  int32_T itemp;
  int32_T kend;
  int32_T ix_m;
  int32_T d_m;
  int32_T jA_j;
  int32_T minmana_tmp;
  int32_T ma_tmp;
  int32_T na_tmp;
  int32_T loop_ub_tmp;
  int32_T temp_tmp_h;
  int32_T b_tmp;
  int32_T kend_c;
  int32_T k_ct;
  int32_T i_p;
  int32_T loop_ub_p;
  int32_T loop_ub_a;
  int32_T fixedBCIdx_size_idx_0;
  int32_T idx;
  int32_T b_ii;
  int32_T pipk;
  int32_T jBcol;
  int32_T kAcol_e;
  int32_T jj_a;
  int32_T c_a;
  int32_T k_i;
  int32_T jA_l;
  int32_T d_o;
  int32_T ijA;
  int32_T mmj;
  int32_T c_o;
  int32_T a_i;
  int32_T n;
  int32_T yk;
  int32_T k_f;
  int32_T jA_i;
  int32_T b_f;
  int32_T c_tmp;
  int32_T temp_tmp_g;
  int32_T temp_tmp_c;
  int32_T inner;
  int32_T b_i;
  int32_T c_i;
  int32_T m_tmp;
  int32_T minmn;
  int32_T maxmn;
  int32_T b_k_o;
  int32_T tmp_size[1];
  int8_T d_l[2];
  B_CoordinateTransformationCon_T CoordinateTransformationConv_pn;
                               /* '<S5>/Coordinate Transformation Conversion' */
  B_CoordinateTransformationCon_T CoordinateTransformationConve_p;
                               /* '<S5>/Coordinate Transformation Conversion' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_trajGen_with_yaw_T {
  shared_uav_rst_sluav_internal_T obj;
                                 /* '<S5>/Minimum Jerk Polynomial Trajectory' */
  ros_slros_internal_block_GetP_T obj_o;/* '<Root>/Get Parameter test' */
  ros_slroscpp_internal_block_P_T obj_b;/* '<S2>/SinkBlock' */
  ros_slroscpp_internal_block_S_T obj_k;/* '<S7>/SourceBlock' */
  ros_slroscpp_internal_block_S_T obj_m;/* '<S4>/SourceBlock' */
  ros_slroscpp_internal_block_S_T obj_kx;/* '<S3>/SourceBlock' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S5>/Discrete-Time Integrator' */
  real_T Filter_DSTATE;                /* '<S101>/Filter' */
  real_T Filter_DSTATE_j;              /* '<S49>/Filter' */
  real_T Filter_DSTATE_b;              /* '<S205>/Filter' */
  real_T Filter_DSTATE_d;              /* '<S153>/Filter' */
  real_T freq;                   /* '<S5>/Minimum Jerk Polynomial Trajectory' */
  real_T startPosition[4];             /* '<S5>/MATLAB Function1' */
  boolean_T DelayInput1_DSTATE;        /* '<S1>/Delay Input1' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S5>/Discrete-Time Integrator' */
  boolean_T objisempty;                /* '<S7>/SourceBlock' */
  boolean_T objisempty_l;        /* '<S5>/Minimum Jerk Polynomial Trajectory' */
  boolean_T freq_not_empty;      /* '<S5>/Minimum Jerk Polynomial Trajectory' */
  boolean_T startPosition_not_empty;   /* '<S5>/MATLAB Function1' */
  boolean_T objisempty_c;              /* '<S4>/SourceBlock' */
  boolean_T objisempty_lg;             /* '<S3>/SourceBlock' */
  boolean_T objisempty_a;              /* '<S2>/SinkBlock' */
  boolean_T objisempty_f;              /* '<Root>/Get Parameter test' */
  boolean_T Subsystem_MODE;            /* '<Root>/Subsystem' */
  DW_CoordinateTransformationCo_T CoordinateTransformationConv_pn;
                               /* '<S5>/Coordinate Transformation Conversion' */
  DW_CoordinateTransformationCo_T CoordinateTransformationConve_p;
                               /* '<S5>/Coordinate Transformation Conversion' */
};

/* Real-time Model Data Structure */
struct tag_RTM_trajGen_with_yaw_T {
  const char_T *errorStatus;
};

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_trajGen_with_yaw_T trajGen_with_yaw_B;

#ifdef __cplusplus

}

#endif

/* Block states (default storage) */
extern struct DW_trajGen_with_yaw_T trajGen_with_yaw_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  /* Model entry point functions */
  extern void trajGen_with_yaw_initialize(void);
  extern void trajGen_with_yaw_step(void);
  extern void trajGen_with_yaw_terminate(void);

#ifdef __cplusplus

}

#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_trajGen_with_yaw_T *const trajGen_with_yaw_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'trajGen_with_yaw'
 * '<S1>'   : 'trajGen_with_yaw/Detect Rise Positive'
 * '<S2>'   : 'trajGen_with_yaw/Publish'
 * '<S3>'   : 'trajGen_with_yaw/Subscribe'
 * '<S4>'   : 'trajGen_with_yaw/Subscribe1'
 * '<S5>'   : 'trajGen_with_yaw/Subsystem'
 * '<S6>'   : 'trajGen_with_yaw/control output'
 * '<S7>'   : 'trajGen_with_yaw/position reading'
 * '<S8>'   : 'trajGen_with_yaw/Detect Rise Positive/Positive'
 * '<S9>'   : 'trajGen_with_yaw/Subscribe/Enabled Subsystem'
 * '<S10>'  : 'trajGen_with_yaw/Subscribe1/Enabled Subsystem'
 * '<S11>'  : 'trajGen_with_yaw/Subsystem/ convert to body frame'
 * '<S12>'  : 'trajGen_with_yaw/Subsystem/MATLAB Function'
 * '<S13>'  : 'trajGen_with_yaw/Subsystem/MATLAB Function1'
 * '<S14>'  : 'trajGen_with_yaw/Subsystem/MATLAB Function3'
 * '<S15>'  : 'trajGen_with_yaw/Subsystem/Radians to Degrees1'
 * '<S16>'  : 'trajGen_with_yaw/Subsystem/X'
 * '<S17>'  : 'trajGen_with_yaw/Subsystem/Y'
 * '<S18>'  : 'trajGen_with_yaw/Subsystem/YAW'
 * '<S19>'  : 'trajGen_with_yaw/Subsystem/Z'
 * '<S20>'  : 'trajGen_with_yaw/Subsystem/X/Anti-windup'
 * '<S21>'  : 'trajGen_with_yaw/Subsystem/X/D Gain'
 * '<S22>'  : 'trajGen_with_yaw/Subsystem/X/External Derivative'
 * '<S23>'  : 'trajGen_with_yaw/Subsystem/X/Filter'
 * '<S24>'  : 'trajGen_with_yaw/Subsystem/X/Filter ICs'
 * '<S25>'  : 'trajGen_with_yaw/Subsystem/X/I Gain'
 * '<S26>'  : 'trajGen_with_yaw/Subsystem/X/Ideal P Gain'
 * '<S27>'  : 'trajGen_with_yaw/Subsystem/X/Ideal P Gain Fdbk'
 * '<S28>'  : 'trajGen_with_yaw/Subsystem/X/Integrator'
 * '<S29>'  : 'trajGen_with_yaw/Subsystem/X/Integrator ICs'
 * '<S30>'  : 'trajGen_with_yaw/Subsystem/X/N Copy'
 * '<S31>'  : 'trajGen_with_yaw/Subsystem/X/N Gain'
 * '<S32>'  : 'trajGen_with_yaw/Subsystem/X/P Copy'
 * '<S33>'  : 'trajGen_with_yaw/Subsystem/X/Parallel P Gain'
 * '<S34>'  : 'trajGen_with_yaw/Subsystem/X/Reset Signal'
 * '<S35>'  : 'trajGen_with_yaw/Subsystem/X/Saturation'
 * '<S36>'  : 'trajGen_with_yaw/Subsystem/X/Saturation Fdbk'
 * '<S37>'  : 'trajGen_with_yaw/Subsystem/X/Sum'
 * '<S38>'  : 'trajGen_with_yaw/Subsystem/X/Sum Fdbk'
 * '<S39>'  : 'trajGen_with_yaw/Subsystem/X/Tracking Mode'
 * '<S40>'  : 'trajGen_with_yaw/Subsystem/X/Tracking Mode Sum'
 * '<S41>'  : 'trajGen_with_yaw/Subsystem/X/Tsamp - Integral'
 * '<S42>'  : 'trajGen_with_yaw/Subsystem/X/Tsamp - Ngain'
 * '<S43>'  : 'trajGen_with_yaw/Subsystem/X/postSat Signal'
 * '<S44>'  : 'trajGen_with_yaw/Subsystem/X/preInt Signal'
 * '<S45>'  : 'trajGen_with_yaw/Subsystem/X/preSat Signal'
 * '<S46>'  : 'trajGen_with_yaw/Subsystem/X/Anti-windup/Disabled'
 * '<S47>'  : 'trajGen_with_yaw/Subsystem/X/D Gain/Internal Parameters'
 * '<S48>'  : 'trajGen_with_yaw/Subsystem/X/External Derivative/Error'
 * '<S49>'  : 'trajGen_with_yaw/Subsystem/X/Filter/Disc. Forward Euler Filter'
 * '<S50>'  : 'trajGen_with_yaw/Subsystem/X/Filter ICs/Internal IC - Filter'
 * '<S51>'  : 'trajGen_with_yaw/Subsystem/X/I Gain/Disabled'
 * '<S52>'  : 'trajGen_with_yaw/Subsystem/X/Ideal P Gain/Passthrough'
 * '<S53>'  : 'trajGen_with_yaw/Subsystem/X/Ideal P Gain Fdbk/Disabled'
 * '<S54>'  : 'trajGen_with_yaw/Subsystem/X/Integrator/Disabled'
 * '<S55>'  : 'trajGen_with_yaw/Subsystem/X/Integrator ICs/Disabled'
 * '<S56>'  : 'trajGen_with_yaw/Subsystem/X/N Copy/Disabled'
 * '<S57>'  : 'trajGen_with_yaw/Subsystem/X/N Gain/Internal Parameters'
 * '<S58>'  : 'trajGen_with_yaw/Subsystem/X/P Copy/Disabled'
 * '<S59>'  : 'trajGen_with_yaw/Subsystem/X/Parallel P Gain/Internal Parameters'
 * '<S60>'  : 'trajGen_with_yaw/Subsystem/X/Reset Signal/Disabled'
 * '<S61>'  : 'trajGen_with_yaw/Subsystem/X/Saturation/Passthrough'
 * '<S62>'  : 'trajGen_with_yaw/Subsystem/X/Saturation Fdbk/Disabled'
 * '<S63>'  : 'trajGen_with_yaw/Subsystem/X/Sum/Sum_PD'
 * '<S64>'  : 'trajGen_with_yaw/Subsystem/X/Sum Fdbk/Disabled'
 * '<S65>'  : 'trajGen_with_yaw/Subsystem/X/Tracking Mode/Disabled'
 * '<S66>'  : 'trajGen_with_yaw/Subsystem/X/Tracking Mode Sum/Passthrough'
 * '<S67>'  : 'trajGen_with_yaw/Subsystem/X/Tsamp - Integral/Disabled'
 * '<S68>'  : 'trajGen_with_yaw/Subsystem/X/Tsamp - Ngain/External Ts'
 * '<S69>'  : 'trajGen_with_yaw/Subsystem/X/postSat Signal/Forward_Path'
 * '<S70>'  : 'trajGen_with_yaw/Subsystem/X/preInt Signal/Internal PreInt'
 * '<S71>'  : 'trajGen_with_yaw/Subsystem/X/preSat Signal/Forward_Path'
 * '<S72>'  : 'trajGen_with_yaw/Subsystem/Y/Anti-windup'
 * '<S73>'  : 'trajGen_with_yaw/Subsystem/Y/D Gain'
 * '<S74>'  : 'trajGen_with_yaw/Subsystem/Y/External Derivative'
 * '<S75>'  : 'trajGen_with_yaw/Subsystem/Y/Filter'
 * '<S76>'  : 'trajGen_with_yaw/Subsystem/Y/Filter ICs'
 * '<S77>'  : 'trajGen_with_yaw/Subsystem/Y/I Gain'
 * '<S78>'  : 'trajGen_with_yaw/Subsystem/Y/Ideal P Gain'
 * '<S79>'  : 'trajGen_with_yaw/Subsystem/Y/Ideal P Gain Fdbk'
 * '<S80>'  : 'trajGen_with_yaw/Subsystem/Y/Integrator'
 * '<S81>'  : 'trajGen_with_yaw/Subsystem/Y/Integrator ICs'
 * '<S82>'  : 'trajGen_with_yaw/Subsystem/Y/N Copy'
 * '<S83>'  : 'trajGen_with_yaw/Subsystem/Y/N Gain'
 * '<S84>'  : 'trajGen_with_yaw/Subsystem/Y/P Copy'
 * '<S85>'  : 'trajGen_with_yaw/Subsystem/Y/Parallel P Gain'
 * '<S86>'  : 'trajGen_with_yaw/Subsystem/Y/Reset Signal'
 * '<S87>'  : 'trajGen_with_yaw/Subsystem/Y/Saturation'
 * '<S88>'  : 'trajGen_with_yaw/Subsystem/Y/Saturation Fdbk'
 * '<S89>'  : 'trajGen_with_yaw/Subsystem/Y/Sum'
 * '<S90>'  : 'trajGen_with_yaw/Subsystem/Y/Sum Fdbk'
 * '<S91>'  : 'trajGen_with_yaw/Subsystem/Y/Tracking Mode'
 * '<S92>'  : 'trajGen_with_yaw/Subsystem/Y/Tracking Mode Sum'
 * '<S93>'  : 'trajGen_with_yaw/Subsystem/Y/Tsamp - Integral'
 * '<S94>'  : 'trajGen_with_yaw/Subsystem/Y/Tsamp - Ngain'
 * '<S95>'  : 'trajGen_with_yaw/Subsystem/Y/postSat Signal'
 * '<S96>'  : 'trajGen_with_yaw/Subsystem/Y/preInt Signal'
 * '<S97>'  : 'trajGen_with_yaw/Subsystem/Y/preSat Signal'
 * '<S98>'  : 'trajGen_with_yaw/Subsystem/Y/Anti-windup/Disabled'
 * '<S99>'  : 'trajGen_with_yaw/Subsystem/Y/D Gain/Internal Parameters'
 * '<S100>' : 'trajGen_with_yaw/Subsystem/Y/External Derivative/Error'
 * '<S101>' : 'trajGen_with_yaw/Subsystem/Y/Filter/Disc. Forward Euler Filter'
 * '<S102>' : 'trajGen_with_yaw/Subsystem/Y/Filter ICs/Internal IC - Filter'
 * '<S103>' : 'trajGen_with_yaw/Subsystem/Y/I Gain/Disabled'
 * '<S104>' : 'trajGen_with_yaw/Subsystem/Y/Ideal P Gain/Passthrough'
 * '<S105>' : 'trajGen_with_yaw/Subsystem/Y/Ideal P Gain Fdbk/Disabled'
 * '<S106>' : 'trajGen_with_yaw/Subsystem/Y/Integrator/Disabled'
 * '<S107>' : 'trajGen_with_yaw/Subsystem/Y/Integrator ICs/Disabled'
 * '<S108>' : 'trajGen_with_yaw/Subsystem/Y/N Copy/Disabled'
 * '<S109>' : 'trajGen_with_yaw/Subsystem/Y/N Gain/Internal Parameters'
 * '<S110>' : 'trajGen_with_yaw/Subsystem/Y/P Copy/Disabled'
 * '<S111>' : 'trajGen_with_yaw/Subsystem/Y/Parallel P Gain/Internal Parameters'
 * '<S112>' : 'trajGen_with_yaw/Subsystem/Y/Reset Signal/Disabled'
 * '<S113>' : 'trajGen_with_yaw/Subsystem/Y/Saturation/Passthrough'
 * '<S114>' : 'trajGen_with_yaw/Subsystem/Y/Saturation Fdbk/Disabled'
 * '<S115>' : 'trajGen_with_yaw/Subsystem/Y/Sum/Sum_PD'
 * '<S116>' : 'trajGen_with_yaw/Subsystem/Y/Sum Fdbk/Disabled'
 * '<S117>' : 'trajGen_with_yaw/Subsystem/Y/Tracking Mode/Disabled'
 * '<S118>' : 'trajGen_with_yaw/Subsystem/Y/Tracking Mode Sum/Passthrough'
 * '<S119>' : 'trajGen_with_yaw/Subsystem/Y/Tsamp - Integral/Disabled'
 * '<S120>' : 'trajGen_with_yaw/Subsystem/Y/Tsamp - Ngain/External Ts'
 * '<S121>' : 'trajGen_with_yaw/Subsystem/Y/postSat Signal/Forward_Path'
 * '<S122>' : 'trajGen_with_yaw/Subsystem/Y/preInt Signal/Internal PreInt'
 * '<S123>' : 'trajGen_with_yaw/Subsystem/Y/preSat Signal/Forward_Path'
 * '<S124>' : 'trajGen_with_yaw/Subsystem/YAW/Anti-windup'
 * '<S125>' : 'trajGen_with_yaw/Subsystem/YAW/D Gain'
 * '<S126>' : 'trajGen_with_yaw/Subsystem/YAW/External Derivative'
 * '<S127>' : 'trajGen_with_yaw/Subsystem/YAW/Filter'
 * '<S128>' : 'trajGen_with_yaw/Subsystem/YAW/Filter ICs'
 * '<S129>' : 'trajGen_with_yaw/Subsystem/YAW/I Gain'
 * '<S130>' : 'trajGen_with_yaw/Subsystem/YAW/Ideal P Gain'
 * '<S131>' : 'trajGen_with_yaw/Subsystem/YAW/Ideal P Gain Fdbk'
 * '<S132>' : 'trajGen_with_yaw/Subsystem/YAW/Integrator'
 * '<S133>' : 'trajGen_with_yaw/Subsystem/YAW/Integrator ICs'
 * '<S134>' : 'trajGen_with_yaw/Subsystem/YAW/N Copy'
 * '<S135>' : 'trajGen_with_yaw/Subsystem/YAW/N Gain'
 * '<S136>' : 'trajGen_with_yaw/Subsystem/YAW/P Copy'
 * '<S137>' : 'trajGen_with_yaw/Subsystem/YAW/Parallel P Gain'
 * '<S138>' : 'trajGen_with_yaw/Subsystem/YAW/Reset Signal'
 * '<S139>' : 'trajGen_with_yaw/Subsystem/YAW/Saturation'
 * '<S140>' : 'trajGen_with_yaw/Subsystem/YAW/Saturation Fdbk'
 * '<S141>' : 'trajGen_with_yaw/Subsystem/YAW/Sum'
 * '<S142>' : 'trajGen_with_yaw/Subsystem/YAW/Sum Fdbk'
 * '<S143>' : 'trajGen_with_yaw/Subsystem/YAW/Tracking Mode'
 * '<S144>' : 'trajGen_with_yaw/Subsystem/YAW/Tracking Mode Sum'
 * '<S145>' : 'trajGen_with_yaw/Subsystem/YAW/Tsamp - Integral'
 * '<S146>' : 'trajGen_with_yaw/Subsystem/YAW/Tsamp - Ngain'
 * '<S147>' : 'trajGen_with_yaw/Subsystem/YAW/postSat Signal'
 * '<S148>' : 'trajGen_with_yaw/Subsystem/YAW/preInt Signal'
 * '<S149>' : 'trajGen_with_yaw/Subsystem/YAW/preSat Signal'
 * '<S150>' : 'trajGen_with_yaw/Subsystem/YAW/Anti-windup/Disabled'
 * '<S151>' : 'trajGen_with_yaw/Subsystem/YAW/D Gain/Internal Parameters'
 * '<S152>' : 'trajGen_with_yaw/Subsystem/YAW/External Derivative/Error'
 * '<S153>' : 'trajGen_with_yaw/Subsystem/YAW/Filter/Disc. Forward Euler Filter'
 * '<S154>' : 'trajGen_with_yaw/Subsystem/YAW/Filter ICs/Internal IC - Filter'
 * '<S155>' : 'trajGen_with_yaw/Subsystem/YAW/I Gain/Disabled'
 * '<S156>' : 'trajGen_with_yaw/Subsystem/YAW/Ideal P Gain/Passthrough'
 * '<S157>' : 'trajGen_with_yaw/Subsystem/YAW/Ideal P Gain Fdbk/Disabled'
 * '<S158>' : 'trajGen_with_yaw/Subsystem/YAW/Integrator/Disabled'
 * '<S159>' : 'trajGen_with_yaw/Subsystem/YAW/Integrator ICs/Disabled'
 * '<S160>' : 'trajGen_with_yaw/Subsystem/YAW/N Copy/Disabled'
 * '<S161>' : 'trajGen_with_yaw/Subsystem/YAW/N Gain/Internal Parameters'
 * '<S162>' : 'trajGen_with_yaw/Subsystem/YAW/P Copy/Disabled'
 * '<S163>' : 'trajGen_with_yaw/Subsystem/YAW/Parallel P Gain/Internal Parameters'
 * '<S164>' : 'trajGen_with_yaw/Subsystem/YAW/Reset Signal/Disabled'
 * '<S165>' : 'trajGen_with_yaw/Subsystem/YAW/Saturation/Passthrough'
 * '<S166>' : 'trajGen_with_yaw/Subsystem/YAW/Saturation Fdbk/Disabled'
 * '<S167>' : 'trajGen_with_yaw/Subsystem/YAW/Sum/Sum_PD'
 * '<S168>' : 'trajGen_with_yaw/Subsystem/YAW/Sum Fdbk/Disabled'
 * '<S169>' : 'trajGen_with_yaw/Subsystem/YAW/Tracking Mode/Disabled'
 * '<S170>' : 'trajGen_with_yaw/Subsystem/YAW/Tracking Mode Sum/Passthrough'
 * '<S171>' : 'trajGen_with_yaw/Subsystem/YAW/Tsamp - Integral/Disabled'
 * '<S172>' : 'trajGen_with_yaw/Subsystem/YAW/Tsamp - Ngain/External Ts'
 * '<S173>' : 'trajGen_with_yaw/Subsystem/YAW/postSat Signal/Forward_Path'
 * '<S174>' : 'trajGen_with_yaw/Subsystem/YAW/preInt Signal/Internal PreInt'
 * '<S175>' : 'trajGen_with_yaw/Subsystem/YAW/preSat Signal/Forward_Path'
 * '<S176>' : 'trajGen_with_yaw/Subsystem/Z/Anti-windup'
 * '<S177>' : 'trajGen_with_yaw/Subsystem/Z/D Gain'
 * '<S178>' : 'trajGen_with_yaw/Subsystem/Z/External Derivative'
 * '<S179>' : 'trajGen_with_yaw/Subsystem/Z/Filter'
 * '<S180>' : 'trajGen_with_yaw/Subsystem/Z/Filter ICs'
 * '<S181>' : 'trajGen_with_yaw/Subsystem/Z/I Gain'
 * '<S182>' : 'trajGen_with_yaw/Subsystem/Z/Ideal P Gain'
 * '<S183>' : 'trajGen_with_yaw/Subsystem/Z/Ideal P Gain Fdbk'
 * '<S184>' : 'trajGen_with_yaw/Subsystem/Z/Integrator'
 * '<S185>' : 'trajGen_with_yaw/Subsystem/Z/Integrator ICs'
 * '<S186>' : 'trajGen_with_yaw/Subsystem/Z/N Copy'
 * '<S187>' : 'trajGen_with_yaw/Subsystem/Z/N Gain'
 * '<S188>' : 'trajGen_with_yaw/Subsystem/Z/P Copy'
 * '<S189>' : 'trajGen_with_yaw/Subsystem/Z/Parallel P Gain'
 * '<S190>' : 'trajGen_with_yaw/Subsystem/Z/Reset Signal'
 * '<S191>' : 'trajGen_with_yaw/Subsystem/Z/Saturation'
 * '<S192>' : 'trajGen_with_yaw/Subsystem/Z/Saturation Fdbk'
 * '<S193>' : 'trajGen_with_yaw/Subsystem/Z/Sum'
 * '<S194>' : 'trajGen_with_yaw/Subsystem/Z/Sum Fdbk'
 * '<S195>' : 'trajGen_with_yaw/Subsystem/Z/Tracking Mode'
 * '<S196>' : 'trajGen_with_yaw/Subsystem/Z/Tracking Mode Sum'
 * '<S197>' : 'trajGen_with_yaw/Subsystem/Z/Tsamp - Integral'
 * '<S198>' : 'trajGen_with_yaw/Subsystem/Z/Tsamp - Ngain'
 * '<S199>' : 'trajGen_with_yaw/Subsystem/Z/postSat Signal'
 * '<S200>' : 'trajGen_with_yaw/Subsystem/Z/preInt Signal'
 * '<S201>' : 'trajGen_with_yaw/Subsystem/Z/preSat Signal'
 * '<S202>' : 'trajGen_with_yaw/Subsystem/Z/Anti-windup/Disabled'
 * '<S203>' : 'trajGen_with_yaw/Subsystem/Z/D Gain/Internal Parameters'
 * '<S204>' : 'trajGen_with_yaw/Subsystem/Z/External Derivative/Error'
 * '<S205>' : 'trajGen_with_yaw/Subsystem/Z/Filter/Disc. Forward Euler Filter'
 * '<S206>' : 'trajGen_with_yaw/Subsystem/Z/Filter ICs/Internal IC - Filter'
 * '<S207>' : 'trajGen_with_yaw/Subsystem/Z/I Gain/Disabled'
 * '<S208>' : 'trajGen_with_yaw/Subsystem/Z/Ideal P Gain/Passthrough'
 * '<S209>' : 'trajGen_with_yaw/Subsystem/Z/Ideal P Gain Fdbk/Disabled'
 * '<S210>' : 'trajGen_with_yaw/Subsystem/Z/Integrator/Disabled'
 * '<S211>' : 'trajGen_with_yaw/Subsystem/Z/Integrator ICs/Disabled'
 * '<S212>' : 'trajGen_with_yaw/Subsystem/Z/N Copy/Disabled'
 * '<S213>' : 'trajGen_with_yaw/Subsystem/Z/N Gain/Internal Parameters'
 * '<S214>' : 'trajGen_with_yaw/Subsystem/Z/P Copy/Disabled'
 * '<S215>' : 'trajGen_with_yaw/Subsystem/Z/Parallel P Gain/Internal Parameters'
 * '<S216>' : 'trajGen_with_yaw/Subsystem/Z/Reset Signal/Disabled'
 * '<S217>' : 'trajGen_with_yaw/Subsystem/Z/Saturation/Passthrough'
 * '<S218>' : 'trajGen_with_yaw/Subsystem/Z/Saturation Fdbk/Disabled'
 * '<S219>' : 'trajGen_with_yaw/Subsystem/Z/Sum/Sum_PD'
 * '<S220>' : 'trajGen_with_yaw/Subsystem/Z/Sum Fdbk/Disabled'
 * '<S221>' : 'trajGen_with_yaw/Subsystem/Z/Tracking Mode/Disabled'
 * '<S222>' : 'trajGen_with_yaw/Subsystem/Z/Tracking Mode Sum/Passthrough'
 * '<S223>' : 'trajGen_with_yaw/Subsystem/Z/Tsamp - Integral/Disabled'
 * '<S224>' : 'trajGen_with_yaw/Subsystem/Z/Tsamp - Ngain/External Ts'
 * '<S225>' : 'trajGen_with_yaw/Subsystem/Z/postSat Signal/Forward_Path'
 * '<S226>' : 'trajGen_with_yaw/Subsystem/Z/preInt Signal/Internal PreInt'
 * '<S227>' : 'trajGen_with_yaw/Subsystem/Z/preSat Signal/Forward_Path'
 * '<S228>' : 'trajGen_with_yaw/position reading/Enabled Subsystem'
 */
#endif                                 /* trajGen_with_yaw_h_ */
