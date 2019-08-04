/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: minimal_one.h
 *
 * Code generated for Simulink model 'minimal_one'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
 * C/C++ source code generated on : Sun Aug  4 16:51:01 2019
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM 10
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_minimal_one_h_
#define RTW_HEADER_minimal_one_h_
#ifndef minimal_one_COMMON_INCLUDES_
# define minimal_one_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* minimal_one_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE[2];              /* '<Root>/Delay' */
} DW;

/* Block signals and states (default storage) */
extern DW rtDW;

/* Model entry point functions */
extern void minimal_one_initialize(void);

/* Customized model step function */
extern void minimal_one_step(real_T arg_Input, real_T *arg_Output);

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
 * '<Root>' : 'minimal_one'
 */
#endif                                 /* RTW_HEADER_minimal_one_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
