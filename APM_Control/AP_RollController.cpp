/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Code by Jon Challinger
//  Modified by Paul Riseborough
//

#include <AP_HAL/AP_HAL.h>
#include "AP_RollController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] = {
    AP_GROUPINFO("2SRV_TCONST",      0, AP_RollController, gains.tau,       0.5f),

    AP_GROUPINFO("2SRV_RMAX",   4, AP_RollController, gains.rmax_pos,       0),

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_RollController, AC_PID),

    AP_GROUPINFO("2SRV_ACCEL", 10, AP_RollController, accel_limit, 500),

    AP_GROUPINFO("_ANGLE_P", 11, AP_RollController, angle_p, 0.0),

    AP_GROUPEND
};

// constructor
AP_RollController::AP_RollController(const AP_FixedWing &parms)
    : AP_FW_Controller(parms,
      AC_PID::Defaults{
        .p         = 0.08,
        .i         = 0.15,
        .d         = 0.0,
        .ff        = 0.345,
        .imax      = 0.666,
        .filt_T_hz = 3.0,
        .filt_E_hz = 0.0,
        .filt_D_hz = 12.0,
        .srmax     = 150.0,
        .srtau     = 1.0
    },
    AP_AutoTune::ATType::AUTOTUNE_ROLL)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Return the measured roll angle in degrees
float AP_RollController::get_measured_angle_deg() const
{
    return AP::ahrs().get_roll_deg();
}

// Return the measured roll rate in radians per second
float AP_RollController::get_measured_rate_rads() const
{
    return AP::ahrs().get_gyro().x;
}

// Return true if the airspeed should be considered as under speed
bool AP_RollController::is_underspeed() const
{
    return get_airspeed() <= float(aparm.airspeed_min);
}

// Return positive rate limit in deg per second, zero if disabled
float AP_RollController::get_positive_rate_limit_degs() const
{
    return MAX(gains.rmax_pos.get(), 0.0);
}

// Return negative rate limit in deg per second (as a positive number) zero if disabled
float AP_RollController::get_negative_rate_limit_degs() const
{
    return get_positive_rate_limit_degs();
}

// Return true if rate limits should be applied
bool AP_RollController::should_apply_rate_limits() const
{
    return !in_recovery;
}

// ################## CUSTOM ROLL CONTROL ###################
// ##########################################################

// Run angle controller
float AP_RollController::run_custom_angle_control(int32_t desired_angle_cd, float scaler, bool disable_integrator, bool ground_mode)
{
    // ###################### H Infinity Angle Controller #################################
    // ##############################################################################

// Ensure tau is valid (Retained for legacy ArduPilot checks, though unused by H-inf)
    if (gains.tau < 0.05f) {
        gains.tau.set(0.05f);
    }
    
    // Fetch loop time for the integrator
    const float dt = AP::scheduler().get_loop_period_s();

    // Calculate the shortest path angle error
    const float desired_angle_deg = wrap_180(desired_angle_cd * 0.01f);
    angle_err_deg = wrap_180(desired_angle_deg - get_measured_angle_deg());
    
    // Convert to radians if your H-infinity model requires it
    float e_k = angle_err_deg * DEG_TO_RAD; 

    // ############## START OF OUTER LOOP H-INFINITY + I CONTROL ####################

    // 1. Calculate H-infinity Control Output: u[k] = Ck * x[k] + Dk * e[k]
    float hinf_out = Outer_Dk * e_k;
    for (uint8_t i = 0; i < OUTER_HINF_ORDER; i++) {
        hinf_out += Outer_Ck[i] * _outer_hinf_x[i];
    }

    // 2. Define Integral Parameters 
    // (You can replace these with AP_Param variables later to tune via Mission Planner)
    const float Ki_outer = 0.1f;   // Integral gain ##########   Successful Ki = 0.1f ###############
    const float I_max = 15.0f;     // Maximum authority of the integrator (anti-windup limit)

    // 3. Update Controller States
    if (ground_mode) {
        // Zero internal states and the integrator on the ground to prevent windup
        for (uint8_t i = 0; i < OUTER_HINF_ORDER; i++) {
            _outer_hinf_x[i] = 0.0f;
        }
        _outer_integrator = 0.0f;
    } else {
        // Update H-infinity states
        float x_next[OUTER_HINF_ORDER] = {0.0f};
        
        for (uint8_t i = 0; i < OUTER_HINF_ORDER; i++) {
            x_next[i] = Outer_Bk[i] * e_k;
            for (uint8_t j = 0; j < OUTER_HINF_ORDER; j++) {
                x_next[i] += Outer_Ak[i][j] * _outer_hinf_x[j];
            }
        }
        
        for (uint8_t i = 0; i < OUTER_HINF_ORDER; i++) {
            _outer_hinf_x[i] = x_next[i];
        }

        // Update Parallel Integrator (Euler integration)
        _outer_integrator += e_k * Ki_outer * dt;
        
        // Apply Anti-Windup Clamp
        _outer_integrator = constrain_float(_outer_integrator, -I_max, I_max);
    }

    // 4. Sum Outputs 
    // Ensure both hinf_out and _outer_integrator are in degrees/sec. 
    // If hinf_out is in radians/sec, multiply it by RAD_TO_DEG before adding.
    float desired_rate_degs = hinf_out + _outer_integrator;

    // ############## END OF OUTER LOOP H-INFINITY + I CONTROL ####################

    // Add coordination offset for fixed-wing coordinated turns
    desired_rate_degs += get_rate_target_offset_degs();

    // Apply ArduPilot's physical rate limits to protect the airframe
    if (should_apply_rate_limits()) {
        desired_rate_degs = rate_limit_degs(desired_rate_degs);
    }

    // Run your inner loop H-infinity rate controller
    return run_custom_axis_rate_control(desired_rate_degs, scaler, disable_integrator, ground_mode);
}

// ################# Custom Roll Axis Controller #################
// ###############################################################

float AP_RollController::run_custom_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode)
{
    /*
      prevent indecision in the roll controller when target roll is
      close to 180 degrees from the current roll
     */
    const float indecision_threshold_deg = 160;
    const float last_desired_rate = _pid_info.target;
    const float abs_angle_err_deg = fabsf(angle_err_deg);
    if (abs_angle_err_deg > indecision_threshold_deg &&
        angle_err_deg <= 180) {
        if (desired_rate_degs * last_desired_rate < 0) {
            desired_rate_degs = -desired_rate_degs;
            // increase the desired rate in proportion to the extra
            // angle we are requesting
            const float new_angle_err_deg = abs_angle_err_deg + (180 - abs_angle_err_deg)*2;
            desired_rate_degs *= new_angle_err_deg / abs_angle_err_deg;
        }
    }

    // in_recovery flag is only valid for single loop, clear it
    in_recovery = false;

    // The code part from run_rate_control() should come here. [AP_FW_Controller.cpp file run_rate_control() function]

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // Check that the controller is called once per loop and no more
    const uint32_t ticks = AP::scheduler().ticks32();
    if (last_run_ticks != 0) {
        if (last_run_ticks == ticks) {
            AP_HAL::panic("FW rate control must be run only once per loop");
        }
        if ((last_run_ticks + 1) != ticks) {
            AP_HAL::panic("FW rate control must be run or reset every loop");
        }
    }
    last_run_ticks = ticks;
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

    const float dt = AP::scheduler().get_loop_period_s();

    const float eas2tas = AP::ahrs().get_EAS2TAS();
    bool limit_I = fabsf(_last_out) >= 45;
    const float rate_rads = get_measured_rate_rads();
    //const float old_I = rate_pid.get_i();

    const bool underspeed = is_underspeed();
    if (underspeed) {
        limit_I = true;
    }

    float target = radians(desired_rate_degs) * scaler * scaler;

    const float measurement = rate_rads * scaler * scaler;

    // Return zero if input is invalid (NaN or infinite)
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // Flag used for logging to indicate a filter reset occurred
    _pid_info.reset = _custom_reset_filter;

    // Reset filters to match the current input (first sample or after reset)
    if (_custom_reset_filter) {
        // Reset filters to match the current inputs
        _custom_reset_filter = false;

        // Reset target filter
        _custom_target = target;
#if AP_FILTER_ENABLED
        if (_target_notch != nullptr) {
            _target_notch->reset();
            _custom_target = _target_notch->apply(_custom_target);
        }
#endif

        // Calculate error and reset error filter
        _custom_error = _custom_target - measurement;
#if AP_FILTER_ENABLED
        if (_error_notch != nullptr) {
            _error_notch->reset();
            _custom_error = _error_notch->apply(_custom_error);
        }
#endif
        // Clear derivative history to avoid spikes after reset
        _custom_derivative = 0.0f;
        _custom_target_derivative = 0.0f;

    } else {

        // Apply target filters
        const float target_last = _custom_target;
#if AP_FILTER_ENABLED
        if (_target_notch != nullptr) {
            // Allocate and set up target notch filter
            target = _target_notch->apply(target);
        }
#endif
        // Apply first-order low-pass filter to target value
        _custom_target += rate_pid.get_filt_T_alpha(dt) * (target - _custom_target);

        // Calculate error and apply error filter
        const float error_last = _custom_error;

        float error = _custom_target - measurement;
#if AP_FILTER_ENABLED
        if (_error_notch != nullptr) {
            // Allocate and set up error notch filter
            error = _error_notch->apply(error);
        }
#endif
        // apply notch filters before FTLD/FLTE to minimize shot noise
        _custom_error += rate_pid.get_filt_E_alpha(dt) * (error - _custom_error);

        if (is_positive(dt)) {
            // Compute and low-pass filter the error derivative (D term)
            float derivative = (_custom_error - error_last) / dt;
            _custom_derivative += rate_pid.get_filt_D_alpha(dt) * (derivative - _custom_derivative);
            // Calculate target derivative for D_FF contribution
            _custom_target_derivative = (_custom_target - target_last) / dt;
        }
    }

    const float Kff = rate_pid.ff().get();
    const float Kdff = rate_pid.kDff().get();

    const float ff = degrees(ff_scale *  _pid_info.FF / (scaler * eas2tas));
    const float dff = degrees(ff_scale * _pid_info.DFF / (scaler * eas2tas));
    
    update_i(dt, limit_I, 1.0f);

    if (disable_integrator) {
    _custom_integrator = 0.0f;
    _custom_I_set = true;
    }

    float I_out = _custom_integrator;

    _pid_info.I = I_out;
    _pid_info.limit = limit_I;

    // Set I set flag for logging and clear
    _pid_info.I_term_set = _custom_I_set;
    _custom_I_set = false;

    _pid_info.FF = _custom_target * Kff;
    _pid_info.DFF = _custom_target_derivative * Kdff;

    auto &pinfo = _pid_info;
    const float deg_scale = degrees(1.0f);
    pinfo.I *= deg_scale;

    // ############## START OF H-INFINITY CONTROL ####################
    
    // The input to our controller is the filtered error in radians/sec
    float e_k = _custom_error;

    // 1. Calculate Control Output: u[k] = Ck * x[k] + Dk * e[k]
    float hinf_out = Dk * e_k;
    for (uint8_t i = 0; i < HINF_ORDER; i++) {
        hinf_out += Ck[i] * _hinf_x[i];
    }

    // Apply the scaling modifiers that ArduPilot uses for airspeed (dynamic pressure)
    hinf_out *= scaler * scaler;

    // 2. Update Controller States: x[k+1] = Ak * x[k] + Bk * e[k]
    if (disable_integrator || !is_positive(dt)) {
        // Zero all states
        for (uint8_t i = 0; i < HINF_ORDER; i++) {
            _hinf_x[i] = 0.0f;
        }
        _custom_I_set = true;
    } else {
        // Temporary array for x[k+1] so we don't overwrite x[k] during calculation
        float x_next[HINF_ORDER] = {0.0f};
        
        for (uint8_t i = 0; i < HINF_ORDER; i++) {
            x_next[i] = Bk[i] * e_k;
            for (uint8_t j = 0; j < HINF_ORDER; j++) {
                x_next[i] += Ak[i][j] * _hinf_x[j];
            }
        }
        
        // Push next state to current state
        for (uint8_t i = 0; i < HINF_ORDER; i++) {
            _hinf_x[i] = x_next[i];
        }
    }

    // 3. Map to AP_Logger structures
    // Since H-infinity doesn't separate P, I, and D, we log the total effort under P
    // and zero out the rest to prevent logger garbage data.
    _pid_info.target = _custom_target;
    _pid_info.actual = measurement;
    _pid_info.error = _custom_error;

    _pid_info.P = hinf_out;

    // ###################### Update_all ends ########################

    
    // Scale the output for ArduPilot's expected degree format
    pinfo.P *= deg_scale;

    // Fix the logged target and actual values to not have the scalers applied
    pinfo.target = desired_rate_degs;
    pinfo.actual = degrees(rate_rads);

    // Sum components (Only P contains our H-inf output)
    float out = pinfo.P + ff + dff + pinfo.I;

    if (ground_mode) {
        // When on ground, heavily suppress output to prevent oscillations
        out *= 0.5f; 
    }

    // Remember the last output to trigger the limit flag next loop
    _last_out = out;

    // Note: Autotune is bypassed here because H-infinity relies on 
    // offline nominal plant synthesis, not runtime Ziegler-Nichols probing.
    
    // Output is scaled to notional centidegrees of deflection
    return constrain_float(out * 100, -4500, 4500);
}

void AP_RollController::update_i(float dt, bool limit, float i_scale)
{   
    const float Ki =   0.2f;              //rate_pid.kI().get(); // Successful Ki = 0.2f
    const float Imax = rate_pid.imax();

    if (!is_zero(Ki) && is_positive(dt)) {
        // Allow integrator growth only if not limited, or if error opposes the integrator direction
        if (!limit || ((is_positive(_custom_integrator) && is_negative(_custom_error)) || (is_negative(_custom_integrator) && is_positive(_custom_error)))) {
            _custom_integrator += ((float)_custom_error * Ki) * i_scale * dt;
            _custom_integrator = constrain_float(_custom_integrator, -Imax, Imax);
        }
    } else {
        _custom_integrator = 0.0f;
    }
}



/*
 Function returns an equivalent aileron deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
*/
float AP_RollController::run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode)
{
    /*
      prevent indecision in the roll controller when target roll is
      close to 180 degrees from the current roll
     */
    const float indecision_threshold_deg = 160;
    const float last_desired_rate = _pid_info.target;
    const float abs_angle_err_deg = fabsf(angle_err_deg);
    if (abs_angle_err_deg > indecision_threshold_deg &&
        angle_err_deg <= 180) {
        if (desired_rate_degs * last_desired_rate < 0) {
            desired_rate_degs = -desired_rate_degs;
            // increase the desired rate in proportion to the extra
            // angle we are requesting
            const float new_angle_err_deg = abs_angle_err_deg + (180 - abs_angle_err_deg)*2;
            desired_rate_degs *= new_angle_err_deg / abs_angle_err_deg;
        }
    }

    // in_recovery flag is only valid for single loop, clear it
    in_recovery = false;

    return run_rate_control(desired_rate_degs, scaler, disable_integrator, ground_mode);
}


/*
  convert from old to new PIDs
  this is a temporary conversion function during development
 */
void AP_RollController::convert_pid()
{
    AP_Float &ff = rate_pid.ff();
    if (ff.configured()) {
        return;
    }
    float old_ff=0, old_p=1.0, old_i=0.3, old_d=0.08;
    int16_t old_imax=3000;
    bool have_old = AP_Param::get_param_by_index(this, 1, AP_PARAM_FLOAT, &old_p);
    have_old |= AP_Param::get_param_by_index(this, 3, AP_PARAM_FLOAT, &old_i);
    have_old |= AP_Param::get_param_by_index(this, 2, AP_PARAM_FLOAT, &old_d);
    have_old |= AP_Param::get_param_by_index(this, 6, AP_PARAM_FLOAT, &old_ff);
    have_old |= AP_Param::get_param_by_index(this, 5, AP_PARAM_INT16, &old_imax);
    if (!have_old) {
        // none of the old gains were set
        return;
    }

    const float kp_ff = MAX((old_p - old_i * gains.tau) * gains.tau  - old_d, 0);
    rate_pid.ff().set_and_save(old_ff + kp_ff);
    rate_pid.kI().set_and_save_ifchanged(old_i * gains.tau);
    rate_pid.kP().set_and_save_ifchanged(old_d);
    rate_pid.kD().set_and_save_ifchanged(0);
    rate_pid.kIMAX().set_and_save_ifchanged(old_imax/4500.0);
}
