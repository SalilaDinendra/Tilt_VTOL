//
// File: theta_controller_fcn.h
//
// Code generated for Simulink model 'theta_controller_fcn'.
//
// Model version                  : 1.60
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Oct  5 16:45:46 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef theta_controller_fcn_h_
#define theta_controller_fcn_h_
#include <cmath>
#include "rtwtypes.h"
#include "theta_controller_fcn_types.h"
#include <cstring>

// Class declaration for model theta_controller_fcn
class theta_controller_fcn final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_theta_controller_fcn_T {
    real32_T DiscreteStateSpace_DSTATE[6];// '<S1>/Discrete State-Space'
    real32_T Integrator_DSTATE;        // '<S39>/Integrator'
  };

  // Constant parameters (default storage)
  struct ConstP_theta_controller_fcn_T {
    // Computed Parameter: DiscreteStateSpace_A
    //  Referenced by: '<S1>/Discrete State-Space'

    real32_T DiscreteStateSpace_A[36];

    // Computed Parameter: DiscreteStateSpace_C
    //  Referenced by: '<S1>/Discrete State-Space'

    real32_T DiscreteStateSpace_C[6];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_theta_controller_fcn_T {
    real32_T theta_error;              // '<Root>/theta_error'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_theta_controller_fcn_T {
    real32_T q_command;                // '<Root>/q_command'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_theta_controller_fcn_T {
    const char_T * volatile errorStatus;
    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const volatile aErrorStatus);
  };

  // Copy Constructor
  theta_controller_fcn(theta_controller_fcn const&) = delete;

  // Assignment Operator
  theta_controller_fcn& operator= (theta_controller_fcn const&) & = delete;

  // Move Constructor
  theta_controller_fcn(theta_controller_fcn &&) = delete;

  // Move Assignment Operator
  theta_controller_fcn& operator= (theta_controller_fcn &&) = delete;

  // Real-Time Model get method
  theta_controller_fcn::RT_MODEL_theta_controller_fcn_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_theta_controller_fcn_T
    *pExtU_theta_controller_fcn_T)
  {
    theta_controller_fcn_U = *pExtU_theta_controller_fcn_T;
  }

  // Root outports get method
  const ExtY_theta_controller_fcn_T &getExternalOutputs() const
  {
    return theta_controller_fcn_Y;
  }

  // model initialize function
  static void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  theta_controller_fcn();

  // Destructor
  ~theta_controller_fcn();

  // private data and function members
 private:
  // External inputs
  ExtU_theta_controller_fcn_T theta_controller_fcn_U;

  // External outputs
  ExtY_theta_controller_fcn_T theta_controller_fcn_Y;

  // Block states
  DW_theta_controller_fcn_T theta_controller_fcn_DW;

  // Real-Time Model
  RT_MODEL_theta_controller_fcn_T theta_controller_fcn_M;
};

// Constant parameters (default storage)
extern const theta_controller_fcn::ConstP_theta_controller_fcn_T
  theta_controller_fcn_ConstP;

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
//  hilite_system('theta_controller/Subsystem')    - opens subsystem theta_controller/Subsystem
//  hilite_system('theta_controller/Subsystem/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'theta_controller'
//  '<S2>'   : 'theta_controller/theta_controller_fcn/PID Controller'
//  '<S3>'   : 'theta_controller/theta_controller_fcn/PID Controller/Anti-windup'
//  '<S4>'   : 'theta_controller/theta_controller_fcn/PID Controller/D Gain'
//  '<S5>'   : 'theta_controller/theta_controller_fcn/PID Controller/External Derivative'
//  '<S6>'   : 'theta_controller/theta_controller_fcn/PID Controller/Filter'
//  '<S7>'   : 'theta_controller/theta_controller_fcn/PID Controller/Filter ICs'
//  '<S8>'   : 'theta_controller/theta_controller_fcn/PID Controller/I Gain'
//  '<S9>'   : 'theta_controller/theta_controller_fcn/PID Controller/Ideal P Gain'
//  '<S10>'  : 'theta_controller/theta_controller_fcn/PID Controller/Ideal P Gain Fdbk'
//  '<S11>'  : 'theta_controller/theta_controller_fcn/PID Controller/Integrator'
//  '<S12>'  : 'theta_controller/theta_controller_fcn/PID Controller/Integrator ICs'
//  '<S13>'  : 'theta_controller/theta_controller_fcn/PID Controller/N Copy'
//  '<S14>'  : 'theta_controller/theta_controller_fcn/PID Controller/N Gain'
//  '<S15>'  : 'theta_controller/theta_controller_fcn/PID Controller/P Copy'
//  '<S16>'  : 'theta_controller/theta_controller_fcn/PID Controller/Parallel P Gain'
//  '<S17>'  : 'theta_controller/theta_controller_fcn/PID Controller/Reset Signal'
//  '<S18>'  : 'theta_controller/theta_controller_fcn/PID Controller/Saturation'
//  '<S19>'  : 'theta_controller/theta_controller_fcn/PID Controller/Saturation Fdbk'
//  '<S20>'  : 'theta_controller/theta_controller_fcn/PID Controller/Sum'
//  '<S21>'  : 'theta_controller/theta_controller_fcn/PID Controller/Sum Fdbk'
//  '<S22>'  : 'theta_controller/theta_controller_fcn/PID Controller/Tracking Mode'
//  '<S23>'  : 'theta_controller/theta_controller_fcn/PID Controller/Tracking Mode Sum'
//  '<S24>'  : 'theta_controller/theta_controller_fcn/PID Controller/Tsamp - Integral'
//  '<S25>'  : 'theta_controller/theta_controller_fcn/PID Controller/Tsamp - Ngain'
//  '<S26>'  : 'theta_controller/theta_controller_fcn/PID Controller/postSat Signal'
//  '<S27>'  : 'theta_controller/theta_controller_fcn/PID Controller/preInt Signal'
//  '<S28>'  : 'theta_controller/theta_controller_fcn/PID Controller/preSat Signal'
//  '<S29>'  : 'theta_controller/theta_controller_fcn/PID Controller/Anti-windup/Disc. Clamping Parallel'
//  '<S30>'  : 'theta_controller/theta_controller_fcn/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
//  '<S31>'  : 'theta_controller/theta_controller_fcn/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
//  '<S32>'  : 'theta_controller/theta_controller_fcn/PID Controller/D Gain/Disabled'
//  '<S33>'  : 'theta_controller/theta_controller_fcn/PID Controller/External Derivative/Disabled'
//  '<S34>'  : 'theta_controller/theta_controller_fcn/PID Controller/Filter/Disabled'
//  '<S35>'  : 'theta_controller/theta_controller_fcn/PID Controller/Filter ICs/Disabled'
//  '<S36>'  : 'theta_controller/theta_controller_fcn/PID Controller/I Gain/Internal Parameters'
//  '<S37>'  : 'theta_controller/theta_controller_fcn/PID Controller/Ideal P Gain/Passthrough'
//  '<S38>'  : 'theta_controller/theta_controller_fcn/PID Controller/Ideal P Gain Fdbk/Disabled'
//  '<S39>'  : 'theta_controller/theta_controller_fcn/PID Controller/Integrator/Discrete'
//  '<S40>'  : 'theta_controller/theta_controller_fcn/PID Controller/Integrator ICs/Internal IC'
//  '<S41>'  : 'theta_controller/theta_controller_fcn/PID Controller/N Copy/Disabled wSignal Specification'
//  '<S42>'  : 'theta_controller/theta_controller_fcn/PID Controller/N Gain/Disabled'
//  '<S43>'  : 'theta_controller/theta_controller_fcn/PID Controller/P Copy/Disabled'
//  '<S44>'  : 'theta_controller/theta_controller_fcn/PID Controller/Parallel P Gain/Disabled'
//  '<S45>'  : 'theta_controller/theta_controller_fcn/PID Controller/Reset Signal/Disabled'
//  '<S46>'  : 'theta_controller/theta_controller_fcn/PID Controller/Saturation/Enabled'
//  '<S47>'  : 'theta_controller/theta_controller_fcn/PID Controller/Saturation Fdbk/Disabled'
//  '<S48>'  : 'theta_controller/theta_controller_fcn/PID Controller/Sum/Passthrough_I'
//  '<S49>'  : 'theta_controller/theta_controller_fcn/PID Controller/Sum Fdbk/Disabled'
//  '<S50>'  : 'theta_controller/theta_controller_fcn/PID Controller/Tracking Mode/Disabled'
//  '<S51>'  : 'theta_controller/theta_controller_fcn/PID Controller/Tracking Mode Sum/Passthrough'
//  '<S52>'  : 'theta_controller/theta_controller_fcn/PID Controller/Tsamp - Integral/TsSignalSpecification'
//  '<S53>'  : 'theta_controller/theta_controller_fcn/PID Controller/Tsamp - Ngain/Passthrough'
//  '<S54>'  : 'theta_controller/theta_controller_fcn/PID Controller/postSat Signal/Forward_Path'
//  '<S55>'  : 'theta_controller/theta_controller_fcn/PID Controller/preInt Signal/Internal PreInt'
//  '<S56>'  : 'theta_controller/theta_controller_fcn/PID Controller/preSat Signal/Forward_Path'

#endif                                 // theta_controller_fcn_h_

//
// File trailer for generated code.
//
// [EOF]
//
