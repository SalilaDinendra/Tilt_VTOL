//
// File: q_controller_fcn.cpp
//
// Code generated for Simulink model 'q_controller_fcn'.
//
// Model version                  : 1.57
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Oct  5 06:08:05 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "q_controller_fcn.h"

// Model step function
void q_controller_fcn::step()
{
  // Outputs for Atomic SubSystem: '<Root>/q_controller_fcn'

  // DiscreteStateSpace: '<S1>/Discrete State-Space' incorporates:
  //   Inport: '<Root>/q_error'
  //   Outport: '<Root>/pitch_command'

  {
    {
      static const int_T colCidxRow0[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pCidx{ &colCidxRow0[0] };

      const real32_T *pC0{ q_controller_fcn_ConstP.DiscreteStateSpace_C };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *y0{ &q_controller_fcn_Y.pitch_command };

      int_T numNonZero{ 5 };

      *y0 = (*pC0++) * xd[*pCidx++];
      while (numNonZero--) {
        *y0 += (*pC0++) * xd[*pCidx++];
      }
    }
  }

  // Update for DiscreteStateSpace: '<S1>/Discrete State-Space' incorporates:
  //   Inport: '<Root>/q_error'
  //   Outport: '<Root>/pitch_command'

  {
    real32_T xnew[6];
    xnew[0] = (0.923116326F)*q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0];
    xnew[0] += (2.46027684F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow1[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow1[0] };

      const real32_T *pA1{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[1] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew1{ &xnew[1] };

      int_T numNonZero{ 5 };

      *pxnew1 = (*pA1++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew1 += (*pA1++) * xd[*pAidx++];
      }
    }

    xnew[1] += (3.76759175E-7F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow2[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow2[0] };

      const real32_T *pA7{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[7] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew2{ &xnew[2] };

      int_T numNonZero{ 5 };

      *pxnew2 = (*pA7++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew2 += (*pA7++) * xd[*pAidx++];
      }
    }

    xnew[2] += (-3.60369398E-7F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow3[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow3[0] };

      const real32_T *pA13{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[13] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew3{ &xnew[3] };

      int_T numNonZero{ 5 };

      *pxnew3 = (*pA13++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew3 += (*pA13++) * xd[*pAidx++];
      }
    }

    xnew[3] += (-8.42096171E-8F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow4[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow4[0] };

      const real32_T *pA19{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[19] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew4{ &xnew[4] };

      int_T numNonZero{ 5 };

      *pxnew4 = (*pA19++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew4 += (*pA19++) * xd[*pAidx++];
      }
    }

    xnew[4] += (-1.7739521E-9F)*q_controller_fcn_U.q_error;

    {
      static const int_T colAidxRow5[6]{ 0, 1, 2, 3, 4, 5 };

      const int_T *pAidx{ &colAidxRow5[0] };

      const real32_T *pA25{ &q_controller_fcn_ConstP.DiscreteStateSpace_A[25] };

      const real32_T *xd{ &q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0] };

      real32_T *pxnew5{ &xnew[5] };

      int_T numNonZero{ 5 };

      *pxnew5 = (*pA25++) * xd[*pAidx++];
      while (numNonZero--) {
        *pxnew5 += (*pA25++) * xd[*pAidx++];
      }
    }

    xnew[5] += (-1.467668E-11F)*q_controller_fcn_U.q_error;
    (void) std::memcpy(&q_controller_fcn_DW.DiscreteStateSpace_DSTATE[0], xnew,
                       sizeof(real32_T)*6);
  }

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
