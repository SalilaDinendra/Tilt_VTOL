//
// File: theta_controller_fcn.cpp
//
// Code generated for Simulink model 'theta_controller_fcn'.
//
// Model version                  : 1.63
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Oct 29 13:07:39 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "theta_controller_fcn.h"
#include "rtwtypes.h"

// Model step function
void theta_controller_fcn::step()
{
  // local block i/o variables
  real32_T rtb_DiscreteStateSpace;
  real32_T rtb_IntegralGain;
  int8_T rtb_Switch1;
  int8_T tmp;
  boolean_T rtb_RelationalOperator;

  // Outputs for Atomic SubSystem: '<Root>/theta_controller_fcn'
  // DiscreteStateSpace: '<S1>/Discrete State-Space' incorporates:
  //   Inport: '<Root>/theta_error'

  {
    {
      static const int_T colCidxRow0[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pCidx{ &colCidxRow0[0] };

      const real32_T *pC0{ theta_controller_fcn_ConstP.DiscreteStateSpace_C };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *y0{ &rtb_DiscreteStateSpace };

      int_T numNonZero{ 11 };

      *y0 = (*pC0++) * xd[*pCidx++];
      while (numNonZero--) {
        *y0 += (*pC0++) * xd[*pCidx++];
      }
    }
  }

  // Saturate: '<S46>/Saturation' incorporates:
  //   DiscreteIntegrator: '<S39>/Integrator'

  if (theta_controller_fcn_DW.Integrator_DSTATE > 15.0F) {
    rtb_IntegralGain = 15.0F;
  } else if (theta_controller_fcn_DW.Integrator_DSTATE < -15.0F) {
    rtb_IntegralGain = -15.0F;
  } else {
    rtb_IntegralGain = theta_controller_fcn_DW.Integrator_DSTATE;
  }

  // Outport: '<Root>/q_command' incorporates:
  //   Saturate: '<S46>/Saturation'
  //   Sum: '<S1>/Sum'

  theta_controller_fcn_Y.q_command = rtb_DiscreteStateSpace + rtb_IntegralGain;

  // DeadZone: '<S31>/DeadZone' incorporates:
  //   DiscreteIntegrator: '<S39>/Integrator'

  if (theta_controller_fcn_DW.Integrator_DSTATE > 15.0F) {
    rtb_IntegralGain = theta_controller_fcn_DW.Integrator_DSTATE - 15.0F;
  } else if (theta_controller_fcn_DW.Integrator_DSTATE >= -15.0F) {
    rtb_IntegralGain = 0.0F;
  } else {
    rtb_IntegralGain = theta_controller_fcn_DW.Integrator_DSTATE - -15.0F;
  }

  // End of DeadZone: '<S31>/DeadZone'

  // RelationalOperator: '<S29>/Relational Operator' incorporates:
  //   Constant: '<S29>/Clamping_zero'

  rtb_RelationalOperator = (rtb_IntegralGain != 0.0F);

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
  //   Inport: '<Root>/theta_error'

  rtb_IntegralGain = 0.08F * theta_controller_fcn_U.theta_error;

  // Update for DiscreteStateSpace: '<S1>/Discrete State-Space' incorporates:
  //   Inport: '<Root>/theta_error'

  {
    real32_T xnew[12];

    {
      static const int_T colAidxRow0[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow0[0] };

      const real32_T *pA0{ theta_controller_fcn_ConstP.DiscreteStateSpace_A };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew0{ &xnew[0] };

      int_T numNonZero{ 11 };

      *pxnew0 = (*pA0++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew0 += (*pA0++) * xd[*pAidx++];
      }
    }

    xnew[0] += (0.31840533F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow1[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow1[0] };

      const real32_T *pA12{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A[12]
      };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew1{ &xnew[1] };

      int_T numNonZero{ 11 };

      *pxnew1 = (*pA12++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew1 += (*pA12++) * xd[*pAidx++];
      }
    }

    xnew[1] += (7.32178693E-11F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow2[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow2[0] };

      const real32_T *pA24{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A[24]
      };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew2{ &xnew[2] };

      int_T numNonZero{ 11 };

      *pxnew2 = (*pA24++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew2 += (*pA24++) * xd[*pAidx++];
      }
    }

    xnew[2] += (-0.0024897526F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow3[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow3[0] };

      const real32_T *pA36{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A[36]
      };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew3{ &xnew[3] };

      int_T numNonZero{ 11 };

      *pxnew3 = (*pA36++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew3 += (*pA36++) * xd[*pAidx++];
      }
    }

    xnew[3] += (-2.44329529E-10F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow4[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow4[0] };

      const real32_T *pA48{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A[48]
      };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew4{ &xnew[4] };

      int_T numNonZero{ 11 };

      *pxnew4 = (*pA48++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew4 += (*pA48++) * xd[*pAidx++];
      }
    }

    xnew[4] += (2.86960539E-10F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow5[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow5[0] };

      const real32_T *pA60{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A[60]
      };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew5{ &xnew[5] };

      int_T numNonZero{ 11 };

      *pxnew5 = (*pA60++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew5 += (*pA60++) * xd[*pAidx++];
      }
    }

    xnew[5] += (4.62234374E-12F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow6[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow6[0] };

      const real32_T *pA72{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A[72]
      };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew6{ &xnew[6] };

      int_T numNonZero{ 11 };

      *pxnew6 = (*pA72++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew6 += (*pA72++) * xd[*pAidx++];
      }
    }

    xnew[6] += (7.96140158E-15F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow7[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow7[0] };

      const real32_T *pA84{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A[84]
      };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew7{ &xnew[7] };

      int_T numNonZero{ 11 };

      *pxnew7 = (*pA84++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew7 += (*pA84++) * xd[*pAidx++];
      }
    }

    xnew[7] += (-2.24184528E-16F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow8[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow8[0] };

      const real32_T *pA96{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A[96]
      };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew8{ &xnew[8] };

      int_T numNonZero{ 11 };

      *pxnew8 = (*pA96++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew8 += (*pA96++) * xd[*pAidx++];
      }
    }

    xnew[8] += (0.0194823444F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow9[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

      const int_T *pAidx{ &colAidxRow9[0] };

      const real32_T *pA108{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A
        [108] };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew9{ &xnew[9] };

      int_T numNonZero{ 11 };

      *pxnew9 = (*pA108++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew9 += (*pA108++) * xd[*pAidx++];
      }
    }

    xnew[9] += (0.000524623669F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow10[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
      };

      const int_T *pAidx{ &colAidxRow10[0] };

      const real32_T *pA120{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A
        [120] };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew10{ &xnew[10] };

      int_T numNonZero{ 11 };

      *pxnew10 = (*pA120++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew10 += (*pA120++) * xd[*pAidx++];
      }
    }

    xnew[10] += (1.31920922E-6F)*theta_controller_fcn_U.theta_error;

    {
      static const int_T colAidxRow11[12]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
      };

      const int_T *pAidx{ &colAidxRow11[0] };

      const real32_T *pA132{ &theta_controller_fcn_ConstP.DiscreteStateSpace_A
        [132] };

      const real32_T *xd{ &theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0]
      };

      real32_T *pxnew11{ &xnew[11] };

      int_T numNonZero{ 11 };

      *pxnew11 = (*pA132++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew11 += (*pA132++) * xd[*pAidx++];
      }
    }

    xnew[11] += (1.32431188E-9F)*theta_controller_fcn_U.theta_error;
    (void) std::memcpy(&theta_controller_fcn_DW.DiscreteStateSpace_DSTATE[0],
                       xnew,
                       sizeof(real32_T)*12);
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

  theta_controller_fcn_DW.Integrator_DSTATE += rtb_IntegralGain;

  // End of Outputs for SubSystem: '<Root>/theta_controller_fcn'
}

// Model initialize function
void theta_controller_fcn::initialize()
{
  // (no initialization code required)
}

// Model terminate function
void theta_controller_fcn::terminate()
{
  // (no terminate code required)
}

const char_T* theta_controller_fcn::RT_MODEL_theta_controller_fcn_T::
  getErrorStatus() const
{
  return (errorStatus);
}

void theta_controller_fcn::RT_MODEL_theta_controller_fcn_T::setErrorStatus(const
  char_T* const volatile aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

// Constructor
theta_controller_fcn::theta_controller_fcn() :
  theta_controller_fcn_U(),
  theta_controller_fcn_Y(),
  theta_controller_fcn_DW(),
  theta_controller_fcn_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
theta_controller_fcn::~theta_controller_fcn() = default;

// Real-Time Model get method
theta_controller_fcn::RT_MODEL_theta_controller_fcn_T * theta_controller_fcn::
  getRTM()
{
  return (&theta_controller_fcn_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
