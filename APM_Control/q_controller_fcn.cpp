//
// File: q_controller_fcn.cpp
//
// Code generated for Simulink model 'q_controller_fcn'.
//
// Model version                  : 1.62
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun Oct 11 17:39:48 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "q_controller_fcn.h"
#include "rtwtypes.h"

// Model step function
void q_controller_fcn::step()
{
  real32_T rtb_DeadZone;
  real32_T rtb_FilterCoefficient;
  real32_T rtb_IntegralGain;
  int8_T tmp;
  int8_T tmp_0;

  // Outputs for Atomic SubSystem: '<Root>/q_controller_fcn'
  // Gain: '<S36>/Integral Gain' incorporates:
  //   Inport: '<Root>/q_error'

  rtb_IntegralGain = 0.07F * q_controller_fcn_U.q_error;

  // Gain: '<S42>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S34>/Filter'
  //   Gain: '<S32>/Derivative Gain'
  //   Inport: '<Root>/q_error'
  //   Sum: '<S34>/SumD'

  rtb_FilterCoefficient = (0.0F * q_controller_fcn_U.q_error -
    q_controller_fcn_DW.Filter_DSTATE) * 100.0F;

  // Sum: '<S48>/Sum' incorporates:
  //   DiscreteIntegrator: '<S39>/Integrator'
  //   Gain: '<S44>/Proportional Gain'
  //   Inport: '<Root>/q_error'

  q_controller_fcn_Y.pitch_cmd = (0.005F * q_controller_fcn_U.q_error +
    q_controller_fcn_DW.Integrator_DSTATE) + rtb_FilterCoefficient;

  // DeadZone: '<S31>/DeadZone'
  if (q_controller_fcn_Y.pitch_cmd > 15.0F) {
    rtb_DeadZone = q_controller_fcn_Y.pitch_cmd - 15.0F;

    // Switch: '<S29>/Switch1' incorporates:
    //   Constant: '<S29>/Constant'

    tmp = 1;
  } else {
    if (q_controller_fcn_Y.pitch_cmd >= -15.0F) {
      rtb_DeadZone = 0.0F;
    } else {
      rtb_DeadZone = q_controller_fcn_Y.pitch_cmd - -15.0F;
    }

    // Switch: '<S29>/Switch1' incorporates:
    //   Constant: '<S29>/Constant2'

    tmp = -1;
  }

  // End of DeadZone: '<S31>/DeadZone'

  // Switch: '<S29>/Switch2' incorporates:
  //   Constant: '<S29>/Clamping_zero'
  //   Constant: '<S29>/Constant3'
  //   Constant: '<S29>/Constant4'
  //   RelationalOperator: '<S29>/fix for DT propagation issue1'

  if (rtb_IntegralGain > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  // Switch: '<S29>/Switch' incorporates:
  //   Constant: '<S29>/Clamping_zero'
  //   Constant: '<S29>/Constant1'
  //   Logic: '<S29>/AND3'
  //   RelationalOperator: '<S29>/Equal1'
  //   RelationalOperator: '<S29>/Relational Operator'
  //   Switch: '<S29>/Switch1'
  //   Switch: '<S29>/Switch2'

  if ((!iszero(rtb_DeadZone)) && (tmp == tmp_0)) {
    rtb_IntegralGain = 0.0F;
  }

  // Update for DiscreteIntegrator: '<S39>/Integrator' incorporates:
  //   Switch: '<S29>/Switch'

  q_controller_fcn_DW.Integrator_DSTATE += 0.0025F * rtb_IntegralGain;

  // Update for DiscreteIntegrator: '<S34>/Filter'
  q_controller_fcn_DW.Filter_DSTATE += 0.0025F * rtb_FilterCoefficient;

  // Saturate: '<S46>/Saturation'
  if (q_controller_fcn_Y.pitch_cmd > 15.0F) {
    // Sum: '<S48>/Sum' incorporates:
    //   Outport: '<Root>/pitch_cmd'

    q_controller_fcn_Y.pitch_cmd = 15.0F;
  } else if (q_controller_fcn_Y.pitch_cmd < -15.0F) {
    // Sum: '<S48>/Sum' incorporates:
    //   Outport: '<Root>/pitch_cmd'

    q_controller_fcn_Y.pitch_cmd = -15.0F;
  }

  // End of Saturate: '<S46>/Saturation'
  // End of Outputs for SubSystem: '<Root>/q_controller_fcn'
}

// Model initialize function
void q_controller_fcn::initialize()
{
  // (no initialization code required)
}

// Model terminate function
void q_controller_fcn::terminate()
{
  // (no terminate code required)
}

const char_T* q_controller_fcn::RT_MODEL_q_controller_fcn_T::getErrorStatus()
  const
{
  return (errorStatus);
}

void q_controller_fcn::RT_MODEL_q_controller_fcn_T::setErrorStatus(const char_T*
  const volatile aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

// Constructor
q_controller_fcn::q_controller_fcn() :
  q_controller_fcn_U(),
  q_controller_fcn_Y(),
  q_controller_fcn_DW(),
  q_controller_fcn_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
q_controller_fcn::~q_controller_fcn() = default;

// Real-Time Model get method
q_controller_fcn::RT_MODEL_q_controller_fcn_T * q_controller_fcn::getRTM()
{
  return (&q_controller_fcn_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
