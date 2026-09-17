//
// File: pole_controller.cpp
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
#include <xmmintrin.h>
#include "rtwtypes.h"
#include "pole_controller.h"

// Output and update for atomic system: '<Root>/pole_controller'
void pole_controller0::pole_controller()
{
  __m128 tmp_2;
  __m128 tmp_3;
  int32_T i;
  int32_T i_0;
  real32_T tmp[6];
  real32_T tmp_0[6];
  real32_T tmp_1[6];
  real32_T PrevY;
  real32_T RateLimiter;
  real32_T rtb_Saturation_0;

  // Sum: '<S1>/Sum' incorporates:
  //   Inport: '<Root>/p'
  //   Inport: '<Root>/p_ref'
  //   Inport: '<Root>/phi'
  //   Inport: '<Root>/phi_ref'
  //   Inport: '<Root>/psi'
  //   Inport: '<Root>/psi_ref'
  //   Inport: '<Root>/q'
  //   Inport: '<Root>/q_ref'
  //   Inport: '<Root>/r'
  //   Inport: '<Root>/r_ref'
  //   Inport: '<Root>/theta'
  //   Inport: '<Root>/theta_ref'

  tmp[0] = pole_controller0_U.p_ref;
  tmp[1] = pole_controller0_U.q_ref;
  tmp[2] = pole_controller0_U.r_ref;
  tmp[3] = pole_controller0_U.phi_ref;
  tmp[4] = pole_controller0_U.theta_ref;
  tmp[5] = pole_controller0_U.psi_ref;
  tmp_0[0] = pole_controller0_U.p;
  tmp_0[1] = pole_controller0_U.q;
  tmp_0[2] = pole_controller0_U.r;
  tmp_0[3] = pole_controller0_U.phi;
  tmp_0[4] = pole_controller0_U.theta;
  tmp_0[5] = pole_controller0_U.psi;
  for (i = 0; i <= 0; i += 4) {
    tmp_2 = _mm_loadu_ps(&tmp[i]);
    tmp_3 = _mm_loadu_ps(&tmp_0[i]);
    _mm_storeu_ps(&tmp_1[i], _mm_sub_ps(tmp_2, tmp_3));
  }

  for (i = 4; i < 6; i++) {
    tmp_1[i] = tmp[i] - tmp_0[i];
  }

  // End of Sum: '<S1>/Sum'
  for (i_0 = 0; i_0 < 3; i_0++) {
    // Gain: '<S1>/Gain'
    rtb_Saturation_0 = 0.0F;
    for (i = 0; i < 6; i++) {
      rtb_Saturation_0 += pole_controller0_ConstP.Gain_Gain[3 * i + i_0] *
        tmp_1[i];
    }

    // Saturate: '<S1>/Saturation' incorporates:
    //   Gain: '<S1>/Gain'

    if (rtb_Saturation_0 > 15.0F) {
      rtb_Saturation_0 = 15.0F;
    } else if (rtb_Saturation_0 < -15.0F) {
      rtb_Saturation_0 = -15.0F;
    }

    // End of Saturate: '<S1>/Saturation'

    // RateLimiter: '<S1>/Rate Limiter'
    PrevY = pole_controller0_DW.PrevY[i_0];
    RateLimiter = rtb_Saturation_0 - PrevY;
    if (RateLimiter > 6.0F) {
      // RateLimiter: '<S1>/Rate Limiter'
      RateLimiter = PrevY + 6.0F;
    } else if (RateLimiter < -6.0F) {
      // RateLimiter: '<S1>/Rate Limiter'
      RateLimiter = PrevY - 6.0F;
    } else {
      // RateLimiter: '<S1>/Rate Limiter'
      RateLimiter = rtb_Saturation_0;
    }

    // RateLimiter: '<S1>/Rate Limiter'
    pole_controller0_B.RateLimiter[i_0] = RateLimiter;

    // RateLimiter: '<S1>/Rate Limiter'
    pole_controller0_DW.PrevY[i_0] = RateLimiter;
  }

  // Outport: '<Root>/RV_Left_Cmd' incorporates:
  //   Sum: '<S1>/Add'

  pole_controller0_Y.RV_Left_Cmd = pole_controller0_B.RateLimiter[1] - 5.803F;

  // Outport: '<Root>/RV_Right_Cmd' incorporates:
  //   Sum: '<S1>/Add1'

  pole_controller0_Y.RV_Right_Cmd = pole_controller0_B.RateLimiter[2] - 5.803F;
}

//
// File trailer for generated code.
//
// [EOF]
//
