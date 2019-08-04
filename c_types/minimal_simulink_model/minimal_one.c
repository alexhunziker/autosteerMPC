/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: minimal_one.c
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

#include "minimal_one.h"
#include<stdio.h>

/* Block signals and states (default storage) */
DW rtDW;

/* Model step function */
void minimal_one_step(real_T arg_Input, real_T *arg_Output)
{
  real_T rtb_Add;

  /* Sum: '<Root>/Add' incorporates:
   *  Delay: '<Root>/Delay'
   *  Inport: '<Root>/Input'
   */
  rtb_Add = arg_Input + rtDW.Delay_DSTATE[0];

  /* Outport: '<Root>/Output' incorporates:
   *  Constant: '<Root>/Constant'
   *  Sum: '<Root>/Add1'
   */
  *arg_Output = rtb_Add + 1.0;

  /* Update for Delay: '<Root>/Delay' */
  rtDW.Delay_DSTATE[0] = rtDW.Delay_DSTATE[1];
  rtDW.Delay_DSTATE[1] = rtb_Add;
}

/* Model initialize function */
void minimal_one_initialize(void)
{
  printf("HELLO WORLD\n");
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
