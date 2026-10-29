//
// File: q_controller_fcn.cpp
//
// Code generated for Simulink model 'q_controller_fcn'.
//
// Model version                  : 1.64
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue Oct 27 12:30:41 2026
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
  // local block i/o variables
  real32_T rtb_DiscreteStateSpace;
  real32_T rtb_IntegralGain;
  int8_T rtb_Switch1;
  int8_T tmp;
  boolean_T rtb_RelationalOperator;

  // Outputs for Atomic SubSystem: '<Root>/q_controller_fcn'
  // DiscreteStateSpace: '<S1>/Discrete State-Space' incorporates:
  //   Inport: '<Root>/q_error'

  {
    {
      static const int_T colCidxRow0[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pCidx{ &colCidxRow0[0] };

      const real32_T *pC0{ q_controller_fcn_ConstP.DiscreteStateSpace_C };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *y0{ &rtb_DiscreteStateSpace };

      int_T numNonZero{ 5 };

      *y0 = (*pC0++) * xd[*pCidx++];
      while (numNonZero--) {
        *y0 += (*pC0++) * xd[*pCidx++];
      }
    }
  }

  // Saturate: '<S46>/Saturation' incorporates:
  //   DiscreteIntegrator: '<S39>/Integrator'

  if (q_controller_fcn_DW.Integrator_DSTATE > 15.0F) {
    rtb_IntegralGain = 15.0F;
  } else if (q_controller_fcn_DW.Integrator_DSTATE < -15.0F) {
    rtb_IntegralGain = -15.0F;
  } else {
    rtb_IntegralGain = q_controller_fcn_DW.Integrator_DSTATE;
  }

  // Outport: '<Root>/pitch_cmd' incorporates:
  //   Saturate: '<S46>/Saturation'
  //   Sum: '<S1>/Sum'

  q_controller_fcn_Y.pitch_cmd = rtb_DiscreteStateSpace + rtb_IntegralGain;

  // DeadZone: '<S31>/DeadZone' incorporates:
  //   DiscreteIntegrator: '<S39>/Integrator'

  if (q_controller_fcn_DW.Integrator_DSTATE > 15.0F) {
    rtb_IntegralGain = q_controller_fcn_DW.Integrator_DSTATE - 15.0F;
  } else if (q_controller_fcn_DW.Integrator_DSTATE >= -15.0F) {
    rtb_IntegralGain = 0.0F;
  } else {
    rtb_IntegralGain = q_controller_fcn_DW.Integrator_DSTATE - -15.0F;
  }

  // End of DeadZone: '<S31>/DeadZone'

  // RelationalOperator: '<S29>/Relational Operator' incorporates:
  //   Constant: '<S29>/Clamping_zero'

  rtb_RelationalOperator = (!iszero(rtb_IntegralGain));

  // Switch: '<S29>/Switch1' incorporates:
  //   Constant: '<S29>/Clamping_zero'
  //   Constant: '<S29>/Constant'
  //   Constant: '<S29>/Constant2'
  //   RelationalOperator: '<S29>/fix for DT propagation issue'

  if (rtb_IntegralGain > 0.0F) {
    rtb_Switch1 = 1;
  } else {
    rtb_Switch1 = -1;
  }

  // End of Switch: '<S29>/Switch1'

  // Gain: '<S36>/Integral Gain' incorporates:
  //   Inport: '<Root>/q_error'

  rtb_IntegralGain = 0.008F * q_controller_fcn_U.q_error;

  // Update for DiscreteStateSpace: '<S1>/Discrete State-Space' incorporates:
  //   Inport: '<Root>/q_error'

  {
    real32_T xnew[6];

    {
      static const int_T colAidxRow0[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow0[0] };

      const real32_T *pA0{ q_controller_fcn_ConstP.DiscreteStateSpace_A };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew0{ &xnew[0] };

      int_T numNonZero{ 5 };

      *pxnew0 = (*pA0++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew0 += (*pA0++) * xd[*pAidx++];
      }
    }

    xnew[0] += (2.46027684F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow1[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow1[0] };

      const real32_T *pA6{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[6] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew1{ &xnew[1] };

      int_T numNonZero{ 5 };

      *pxnew1 = (*pA6++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew1 += (*pA6++) * xd[*pAidx++];
      }
    }

    xnew[1] += (4.03473086E-6F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow2[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow2[0] };

      const real32_T *pA12{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[12] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew2{ &xnew[2] };

      int_T numNonZero{ 5 };

      *pxnew2 = (*pA12++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew2 += (*pA12++) * xd[*pAidx++];
      }
    }

    xnew[2] += (-1.67834714E-5F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow3[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow3[0] };

      const real32_T *pA18{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[18] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew3{ &xnew[3] };

      int_T numNonZero{ 5 };

      *pxnew3 = (*pA18++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew3 += (*pA18++) * xd[*pAidx++];
      }
    }

    xnew[3] += (-3.92206039E-6F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow4[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow4[0] };

      const real32_T *pA24{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[24] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew4{ &xnew[4] };

      int_T numNonZero{ 5 };

      *pxnew4 = (*pA24++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew4 += (*pA24++) * xd[*pAidx++];
      }
    }

    xnew[4] += (-8.26237567E-8F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow5[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow5[0] };

      const real32_T *pA30{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[30] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew5{ &xnew[5] };

      int_T numNonZero{ 5 };

      *pxnew5 = (*pA30++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew5 += (*pA30++) * xd[*pAidx++];
      }
    }

    xnew[5] += (-6.83593127E-10F)*q_controller_fcn_U.q_error;
    (void) std::memcpy(&q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0], xnew,
                       sizeof(real32_T)*6);
  }

  // Switch: '<S29>/Switch2' incorporates:
  //   Constant: '<S29>/Clamping_zero'
  //   Constant: '<S29>/Constant3'
  //   Constant: '<S29>/Constant4'
  //   RelationalOperator: '<S29>/fix for DT propagation issue1'

  if (rtb_IntegralGain > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  // Switch: '<S29>/Switch' incorporates:
  //   Constant: '<S29>/Constant1'
  //   Logic: '<S29>/AND3'
  //   RelationalOperator: '<S29>/Equal1'
  //   Switch: '<S29>/Switch2'

  if (rtb_RelationalOperator && (rtb_Switch1 == tmp)) {
    rtb_IntegralGain = 0.0F;
  }

  // Update for DiscreteIntegrator: '<S39>/Integrator' incorporates:
  //   Switch: '<S29>/Switch'

  q_controller_fcn_DW.Integrator_DSTATE += rtb_IntegralGain;

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
