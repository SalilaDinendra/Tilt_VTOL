//
// File: pole_controller0.h
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
#ifndef pole_controller0_h_
#define pole_controller0_h_
#include <cmath>
#include "rtwtypes.h"
#include "pole_controller0_types.h"

// Class declaration for model pole_controller0
class pole_controller0 final
{
  // public data and function members
 public:
  // Block signals (default storage)
  struct B_pole_controller0_T {
    real32_T RateLimiter[3];           // '<S1>/Rate Limiter'
  };

  // Block states (default storage) for system '<Root>'
  struct DW_pole_controller0_T {
    real32_T PrevY[3];                 // '<S1>/Rate Limiter'
  };

  // Constant parameters (default storage)
  struct ConstP_pole_controller0_T {
    // Expression: -K_pole_single
    //  Referenced by: '<S1>/Gain'

    real32_T Gain_Gain[18];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_pole_controller0_T {
    real32_T p_ref;                    // '<Root>/p_ref'
    real32_T q_ref;                    // '<Root>/q_ref'
    real32_T r_ref;                    // '<Root>/r_ref'
    real32_T phi_ref;                  // '<Root>/phi_ref'
    real32_T theta_ref;                // '<Root>/theta_ref'
    real32_T psi_ref;                  // '<Root>/psi_ref'
    real32_T p;                        // '<Root>/p'
    real32_T q;                        // '<Root>/q'
    real32_T r;                        // '<Root>/r'
    real32_T phi;                      // '<Root>/phi'
    real32_T theta;                    // '<Root>/theta'
    real32_T psi;                      // '<Root>/psi'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_pole_controller0_T {
    real32_T Aileron_Cmd;              // '<Root>/Aileron_Cmd'
    real32_T RV_Left_Cmd;              // '<Root>/RV_Left_Cmd'
    real32_T RV_Right_Cmd;             // '<Root>/RV_Right_Cmd'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_pole_controller0_T {
    const char_T * volatile errorStatus;
    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const volatile aErrorStatus);
  };

  // Copy Constructor
  pole_controller0(pole_controller0 const&) = delete;

  // Assignment Operator
  pole_controller0& operator= (pole_controller0 const&) & = delete;

  // Move Constructor
  pole_controller0(pole_controller0 &&) = delete;

  // Move Assignment Operator
  pole_controller0& operator= (pole_controller0 &&) = delete;

  // Real-Time Model get method
  pole_controller0::RT_MODEL_pole_controller0_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_pole_controller0_T *pExtU_pole_controller0_T)
  {
    pole_controller0_U = *pExtU_pole_controller0_T;
  }

  // Root outports get method
  const ExtY_pole_controller0_T &getExternalOutputs() const
  {
    return pole_controller0_Y;
  }

  // model initialize function
  static void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  pole_controller0();

  // Destructor
  ~pole_controller0();

  // private data and function members
 private:
  // External inputs
  ExtU_pole_controller0_T pole_controller0_U;

  // External outputs
  ExtY_pole_controller0_T pole_controller0_Y;

  // Block signals
  B_pole_controller0_T pole_controller0_B;

  // Block states
  DW_pole_controller0_T pole_controller0_DW;

  // private member function(s) for subsystem '<Root>/pole_controller'
  void pole_controller();

  // Real-Time Model
  RT_MODEL_pole_controller0_T pole_controller0_M;
};

// Constant parameters (default storage)
extern const pole_controller0::ConstP_pole_controller0_T pole_controller0_ConstP;

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
//  hilite_system('pole_controller/Subsystem')    - opens subsystem pole_controller/Subsystem
//  hilite_system('pole_controller/Subsystem/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'pole_controller'

#endif                                 // pole_controller0_h_

//
// File trailer for generated code.
//
// [EOF]
//
