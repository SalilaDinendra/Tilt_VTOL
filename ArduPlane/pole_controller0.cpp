//
// File: pole_controller0.cpp
//
// Code generated for Simulink model 'pole_controller0'.
//
// Model version                  : 1.43
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Sep 16 13:45:10 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "pole_controller0.h"

// Model step function
void pole_controller0::step()
{
  // Outputs for Atomic SubSystem: '<Root>/pole_controller'
  pole_controller();

  // End of Outputs for SubSystem: '<Root>/pole_controller'

  // Outport: '<Root>/Aileron_Cmd'
  pole_controller0_Y.Aileron_Cmd = pole_controller0_B.RateLimiter[0];
}

// Model initialize function
void pole_controller0::initialize()
{
  // (no initialization code required)
}

// Model terminate function
void pole_controller0::terminate()
{
  // (no terminate code required)
}

const char_T* pole_controller0::RT_MODEL_pole_controller0_T::getErrorStatus()
  const
{
  return (errorStatus);
}

void pole_controller0::RT_MODEL_pole_controller0_T::setErrorStatus(const char_T*
  const volatile aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

// Constructor
pole_controller0::pole_controller0() :
  pole_controller0_U(),
  pole_controller0_Y(),
  pole_controller0_B(),
  pole_controller0_DW(),
  pole_controller0_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
pole_controller0::~pole_controller0() = default;

// Real-Time Model get method
pole_controller0::RT_MODEL_pole_controller0_T * pole_controller0::getRTM()
{
  return (&pole_controller0_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
