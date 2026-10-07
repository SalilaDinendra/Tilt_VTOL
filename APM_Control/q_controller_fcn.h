//
// File: q_controller_fcn.h
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
#ifndef q_controller_fcn_h_
#define q_controller_fcn_h_
#include <cmath>
#include "rtwtypes.h"
#include "q_controller_fcn_types.h"
#include <cstring>

// Class declaration for model q_controller_fcn
class q_controller_fcn final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_q_controller_fcn_T {
    real32_T DiscreteStateSpace_DSTATE[6];// '<S1>/Discrete State-Space'
  };

  // Constant parameters (default storage)
  struct ConstP_q_controller_fcn_T {
    // Computed Parameter: DiscreteStateSpace_A
    //  Referenced by: '<S1>/Discrete State-Space'

    real32_T DiscreteStateSpace_A[31];

    // Computed Parameter: DiscreteStateSpace_C
    //  Referenced by: '<S1>/Discrete State-Space'

    real32_T DiscreteStateSpace_C[6];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_q_controller_fcn_T {
    real32_T q_error;                  // '<Root>/q_error'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_q_controller_fcn_T {
    real32_T pitch_command;            // '<Root>/pitch_command'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_q_controller_fcn_T {
    const char_T * volatile errorStatus;
    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const volatile aErrorStatus);
  };

  // Copy Constructor
  q_controller_fcn(q_controller_fcn const&) = delete;

  // Assignment Operator
  q_controller_fcn& operator= (q_controller_fcn const&) & = delete;

  // Move Constructor
  q_controller_fcn(q_controller_fcn &&) = delete;

  // Move Assignment Operator
  q_controller_fcn& operator= (q_controller_fcn &&) = delete;

  // Real-Time Model get method
  q_controller_fcn::RT_MODEL_q_controller_fcn_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_q_controller_fcn_T *pExtU_q_controller_fcn_T)
  {
    q_controller_fcn_U = *pExtU_q_controller_fcn_T;
  }

  // Root outports get method
  const ExtY_q_controller_fcn_T &getExternalOutputs() const
  {
    return q_controller_fcn_Y;
  }

  // model initialize function
  static void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  q_controller_fcn();

  // Destructor
  ~q_controller_fcn();

  // private data and function members
 private:
  // External inputs
  ExtU_q_controller_fcn_T q_controller_fcn_U;

  // External outputs
  ExtY_q_controller_fcn_T q_controller_fcn_Y;

  // Block states
  DW_q_controller_fcn_T q_controller_fcn_DW;

  // Real-Time Model
  RT_MODEL_q_controller_fcn_T q_controller_fcn_M;
};

// Constant parameters (default storage)
extern const q_controller_fcn::ConstP_q_controller_fcn_T q_controller_fcn_ConstP;

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('q_controller/Subsystem')    - opens subsystem q_controller/Subsystem
//  hilite_system('q_controller/Subsystem/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'q_controller'

#endif                                 // q_controller_fcn_h_

//
// File trailer for generated code.
//
// [EOF]
//
