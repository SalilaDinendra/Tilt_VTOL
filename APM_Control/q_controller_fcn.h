//
// File: q_controller_fcn.h
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
#ifndef q_controller_fcn_h_
#define q_controller_fcn_h_
#include <cmath>
#include "rtwtypes.h"
#include "q_controller_fcn_types.h"

// Class declaration for model q_controller_fcn
class q_controller_fcn final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_q_controller_fcn_T {
    real32_T Integrator_DSTATE;        // '<S39>/Integrator'
    real32_T Filter_DSTATE;            // '<S34>/Filter'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_q_controller_fcn_T {
    real32_T q_error;                  // '<Root>/q_error'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_q_controller_fcn_T {
    real32_T pitch_cmd;                // '<Root>/pitch_cmd'
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
//  '<S2>'   : 'q_controller/q_controller_fcn/PID Controller'
//  '<S3>'   : 'q_controller/q_controller_fcn/PID Controller/Anti-windup'
//  '<S4>'   : 'q_controller/q_controller_fcn/PID Controller/D Gain'
//  '<S5>'   : 'q_controller/q_controller_fcn/PID Controller/External Derivative'
//  '<S6>'   : 'q_controller/q_controller_fcn/PID Controller/Filter'
//  '<S7>'   : 'q_controller/q_controller_fcn/PID Controller/Filter ICs'
//  '<S8>'   : 'q_controller/q_controller_fcn/PID Controller/I Gain'
//  '<S9>'   : 'q_controller/q_controller_fcn/PID Controller/Ideal P Gain'
//  '<S10>'  : 'q_controller/q_controller_fcn/PID Controller/Ideal P Gain Fdbk'
//  '<S11>'  : 'q_controller/q_controller_fcn/PID Controller/Integrator'
//  '<S12>'  : 'q_controller/q_controller_fcn/PID Controller/Integrator ICs'
//  '<S13>'  : 'q_controller/q_controller_fcn/PID Controller/N Copy'
//  '<S14>'  : 'q_controller/q_controller_fcn/PID Controller/N Gain'
//  '<S15>'  : 'q_controller/q_controller_fcn/PID Controller/P Copy'
//  '<S16>'  : 'q_controller/q_controller_fcn/PID Controller/Parallel P Gain'
//  '<S17>'  : 'q_controller/q_controller_fcn/PID Controller/Reset Signal'
//  '<S18>'  : 'q_controller/q_controller_fcn/PID Controller/Saturation'
//  '<S19>'  : 'q_controller/q_controller_fcn/PID Controller/Saturation Fdbk'
//  '<S20>'  : 'q_controller/q_controller_fcn/PID Controller/Sum'
//  '<S21>'  : 'q_controller/q_controller_fcn/PID Controller/Sum Fdbk'
//  '<S22>'  : 'q_controller/q_controller_fcn/PID Controller/Tracking Mode'
//  '<S23>'  : 'q_controller/q_controller_fcn/PID Controller/Tracking Mode Sum'
//  '<S24>'  : 'q_controller/q_controller_fcn/PID Controller/Tsamp - Integral'
//  '<S25>'  : 'q_controller/q_controller_fcn/PID Controller/Tsamp - Ngain'
//  '<S26>'  : 'q_controller/q_controller_fcn/PID Controller/postSat Signal'
//  '<S27>'  : 'q_controller/q_controller_fcn/PID Controller/preInt Signal'
//  '<S28>'  : 'q_controller/q_controller_fcn/PID Controller/preSat Signal'
//  '<S29>'  : 'q_controller/q_controller_fcn/PID Controller/Anti-windup/Disc. Clamping Parallel'
//  '<S30>'  : 'q_controller/q_controller_fcn/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
//  '<S31>'  : 'q_controller/q_controller_fcn/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
//  '<S32>'  : 'q_controller/q_controller_fcn/PID Controller/D Gain/Internal Parameters'
//  '<S33>'  : 'q_controller/q_controller_fcn/PID Controller/External Derivative/Error'
//  '<S34>'  : 'q_controller/q_controller_fcn/PID Controller/Filter/Disc. Forward Euler Filter'
//  '<S35>'  : 'q_controller/q_controller_fcn/PID Controller/Filter ICs/Internal IC - Filter'
//  '<S36>'  : 'q_controller/q_controller_fcn/PID Controller/I Gain/Internal Parameters'
//  '<S37>'  : 'q_controller/q_controller_fcn/PID Controller/Ideal P Gain/Passthrough'
//  '<S38>'  : 'q_controller/q_controller_fcn/PID Controller/Ideal P Gain Fdbk/Disabled'
//  '<S39>'  : 'q_controller/q_controller_fcn/PID Controller/Integrator/Discrete'
//  '<S40>'  : 'q_controller/q_controller_fcn/PID Controller/Integrator ICs/Internal IC'
//  '<S41>'  : 'q_controller/q_controller_fcn/PID Controller/N Copy/Disabled'
//  '<S42>'  : 'q_controller/q_controller_fcn/PID Controller/N Gain/Internal Parameters'
//  '<S43>'  : 'q_controller/q_controller_fcn/PID Controller/P Copy/Disabled'
//  '<S44>'  : 'q_controller/q_controller_fcn/PID Controller/Parallel P Gain/Internal Parameters'
//  '<S45>'  : 'q_controller/q_controller_fcn/PID Controller/Reset Signal/Disabled'
//  '<S46>'  : 'q_controller/q_controller_fcn/PID Controller/Saturation/Enabled'
//  '<S47>'  : 'q_controller/q_controller_fcn/PID Controller/Saturation Fdbk/Disabled'
//  '<S48>'  : 'q_controller/q_controller_fcn/PID Controller/Sum/Sum_PID'
//  '<S49>'  : 'q_controller/q_controller_fcn/PID Controller/Sum Fdbk/Disabled'
//  '<S50>'  : 'q_controller/q_controller_fcn/PID Controller/Tracking Mode/Disabled'
//  '<S51>'  : 'q_controller/q_controller_fcn/PID Controller/Tracking Mode Sum/Passthrough'
//  '<S52>'  : 'q_controller/q_controller_fcn/PID Controller/Tsamp - Integral/TsSignalSpecification'
//  '<S53>'  : 'q_controller/q_controller_fcn/PID Controller/Tsamp - Ngain/Passthrough'
//  '<S54>'  : 'q_controller/q_controller_fcn/PID Controller/postSat Signal/Forward_Path'
//  '<S55>'  : 'q_controller/q_controller_fcn/PID Controller/preInt Signal/Internal PreInt'
//  '<S56>'  : 'q_controller/q_controller_fcn/PID Controller/preSat Signal/Forward_Path'

#endif                                 // q_controller_fcn_h_

//
// File trailer for generated code.
//
// [EOF]
//
