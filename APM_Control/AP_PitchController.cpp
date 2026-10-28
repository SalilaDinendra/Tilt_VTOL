
#include <AP_HAL/AP_HAL.h>
#include "AP_PitchController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PitchController::var_info[] = {

    AP_GROUPINFO("2SRV_TCONST",      0, AP_PitchController, gains.tau,       0.5f),

    AP_GROUPINFO("2SRV_RMAX_UP",     4, AP_PitchController, gains.rmax_pos,   0.0f),

    AP_GROUPINFO("2SRV_RMAX_DN",     5, AP_PitchController, gains.rmax_neg,   0.0f),

    AP_GROUPINFO("2SRV_RLL",      6, AP_PitchController, _roll_ff,        1.0f),

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 11, AP_PitchController, AC_PID),
    
    AP_GROUPINFO("2SRV_ACCEL", 12, AP_PitchController, accel_limit, 500),

    AP_GROUPINFO("_ANGLE_P", 13, AP_PitchController, angle_p, 0.0),

    AP_GROUPEND
};

AP_PitchController::AP_PitchController(const AP_FixedWing &parms)
    : AP_FW_Controller(parms,
      AC_PID::Defaults{
        .p         = 0.04,
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
    AP_AutoTune::ATType::AUTOTUNE_PITCH)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Return the measured pitch angle in degrees
float AP_PitchController::get_measured_angle_deg() const
{
    return AP::ahrs().get_pitch_deg();
}

// Return the measured pitch rate in radians per second
float AP_PitchController::get_measured_rate_rads() const
{
    return AP::ahrs().get_gyro().y;
}

// Return true if the airspeed should be considered as under speed
bool AP_PitchController::is_underspeed() const
{
    return get_airspeed() <= 0.5*float(aparm.airspeed_min);
}

// Return true if the vehicle is inverted
bool AP_PitchController::is_inverted() const
{
    return fabsf(AP::ahrs().get_roll_deg()) >= 90.0;
}

// Return positive rate limit in deg per second, zero if disabled
float AP_PitchController::get_positive_rate_limit_degs() const
{
    return MAX(gains.rmax_pos.get(), 0.0);
}

// Return negative rate limit in deg per second (as a positive number) zero if disabled
float AP_PitchController::get_negative_rate_limit_degs() const
{
    return MAX(gains.rmax_neg.get(), 0.0);
}

// Return true if rate limits should be applied
bool AP_PitchController::should_apply_rate_limits() const
{
    return !is_inverted();
}

// get the rate offset in degrees/second needed for pitch in body frame to maintain height in a coordinated turn.
float AP_PitchController::get_rate_target_offset_degs() const
{
    const AP_AHRS &_ahrs = AP::ahrs();

    float bank_angle = _ahrs.get_roll_rad();

    // limit bank angle between +- 80 deg if right way up and between 100 and 260 if inverted
    if (!is_inverted()) {
        bank_angle = constrain_float(bank_angle,-radians(80),radians(80));
    } else {
        // Note that the wrap means we have a different range here, we could wrap it back but its only used in trigonometric functions so we don't need to.
        bank_angle = constrain_float(wrap_2PI(bank_angle), radians(100), radians(260));
    }
    if (abs(_ahrs.pitch_sensor) > 7000) {
        // don't do turn coordination handling when at very high pitch angles
        return 0.0;
    }

    // Assume true airspeed is at least min airspeed, protect against zeros.
    const float true_airspeed = MAX((get_airspeed() * _ahrs.get_EAS2TAS()), MAX(aparm.airspeed_min, 1));

    // Lateral acceleration
    const float lateral_accel = tanf(bank_angle) * GRAVITY_MSS * cosf(_ahrs.get_pitch_rad());

    // Resultant turn rate in the pitch axis
    const float turn_rate = (lateral_accel / true_airspeed) * sinf(bank_angle);

    // Apply gain
    float rate_offset = fabsf(degrees(turn_rate)) * _roll_ff;

    return rate_offset;
}


/*
  convert from old to new PIDs
  this is a temporary conversion function during development
 */
void AP_PitchController::convert_pid()
{
    AP_Float &ff = rate_pid.ff();
    if (ff.configured()) {
        return;
    }

    float old_ff=0, old_p=1.0, old_i=0.3, old_d=0.08;
    int16_t old_imax = 3000;
    bool have_old = AP_Param::get_param_by_index(this, 1, AP_PARAM_FLOAT, &old_p);
    have_old |= AP_Param::get_param_by_index(this, 3, AP_PARAM_FLOAT, &old_i);
    have_old |= AP_Param::get_param_by_index(this, 2, AP_PARAM_FLOAT, &old_d);
    have_old |= AP_Param::get_param_by_index(this, 8, AP_PARAM_FLOAT, &old_ff);
    have_old |= AP_Param::get_param_by_index(this, 7, AP_PARAM_FLOAT, &old_imax);
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


float AP_PitchController::run_custom_angle_control(
    int32_t desired_angle_cd,
    float scaler,
    bool disable_integrator,
    bool ground_mode)
{

    /// NATIVE PART ///
/*
    // Ensure tau is valid
    if (gains.tau < 0.05f) {
        gains.tau.set(0.05f);
    }
    const float desired_angle_deg = wrap_180(desired_angle_cd * 0.01);

    if (!should_apply_input_shaping()) {
        // Calculate rate directly from angle error with no input shaping
        angle_err_deg = wrap_180(desired_angle_deg - get_measured_angle_deg());
        float desired_rate_degs = angle_err_deg / gains.tau;

        // Reset input shaping set points
        reset_input_shaping_deg(desired_angle_deg, desired_rate_degs);

        // Add coordination offset
        desired_rate_degs += get_rate_target_offset_degs();

        // Apply rate limits if enabled
        if (should_apply_rate_limits()) {
            desired_rate_degs = rate_limit_degs(desired_rate_degs);
        }

        // Run rate controller
        return run_custom_axis_rate_control(desired_rate_degs, scaler, disable_integrator, ground_mode);
    }

    // Apply input shaping to desired angle
    const float dt = AP::scheduler().get_loop_period_s();

    const float accel_max = accel_limit.get();
    const float jerk_limit = accel_max / MAX(gains.tau.get(), 0.1);

    // Ensure the shortest path is taken
    const float angle_error = wrap_180(desired_angle_deg - angle_target_deg);

    // Apply input shaping updating the accel target
    shape_pos_vel_accel(
        angle_error, 0.0, 0.0, // desired pos, vel and accel
        0.0, rate_target_degs, accel_target_degss, // current shaped target
        -get_negative_rate_limit_degs(), get_positive_rate_limit_degs(), // velocity limits
        -accel_max, accel_max, // accel limits
        jerk_limit, // jerk limit
        dt, true
    );

    // Integrate pos and vel from updated accel target
    angle_target_deg += rate_target_degs * dt + accel_target_degss * 0.5 * sq(dt);
    rate_target_degs += accel_target_degss * dt;

    // Make sure target remains in the range +-180
    angle_target_deg = wrap_180(angle_target_deg);

    // Calculate angle error
    angle_err_deg = wrap_180(angle_target_deg - get_measured_angle_deg());

    // Use 1 / tau if angle gain is not set
    float angle_gain = 1.0 / gains.tau.get();
    if (is_positive(angle_p.get())) {
        angle_gain = angle_p.get();
    }

    // Apply gain using sqrt controller
    float desired_rate_degs = sqrt_controller(angle_err_deg, angle_gain, accel_max * 0.5, dt);

    // Add feed forward rate demand and offset then constrain to rate limit
    desired_rate_degs = rate_limit_degs(desired_rate_degs + rate_target_degs + get_rate_target_offset_degs()); 

*/
    // ###################### H Infinity controller #################################
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
    const float Ki_outer = 0.001f;   // Integral gain
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

    
    return run_custom_axis_rate_control(
        desired_rate_degs,
        scaler,
        disable_integrator,
        ground_mode);
}

// Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
// A positive demand is up
float AP_PitchController::run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode)
{
    
    // Invert desired if vehicle is inverted.
    if (is_inverted()) {
        desired_rate_degs *= -1.0;
    }

    const AP_AHRS &_ahrs = AP::ahrs();
    float roll_wrapped = labs(_ahrs.roll_sensor);
    if (roll_wrapped > 9000) {
        roll_wrapped = 18000 - roll_wrapped;
    }
    const float roll_limit_margin = MIN(aparm.roll_limit*100 + 500.0, 8500.0);
    if (roll_wrapped > roll_limit_margin && labs(_ahrs.pitch_sensor) < 7000) {
        float roll_prop = (roll_wrapped - roll_limit_margin) / (float)(9000 - roll_limit_margin);
        desired_rate_degs *= (1 - roll_prop);
    }

    return run_rate_control(desired_rate_degs, scaler, disable_integrator, ground_mode);
}

// ############# CUSTOM AXIS RATE CONTROL ##################### //
// ############################################################ //
float AP_PitchController::run_custom_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode)
{
    // Invert desired if vehicle is inverted.
    if (is_inverted()) {
        desired_rate_degs *= -1.0;
    }

    const AP_AHRS &_ahrs = AP::ahrs();
    float roll_wrapped = labs(_ahrs.roll_sensor);
    if (roll_wrapped > 9000) {
        roll_wrapped = 18000 - roll_wrapped;
    }
    const float roll_limit_margin = MIN(aparm.roll_limit*100 + 500.0, 8500.0);
    if (roll_wrapped > roll_limit_margin && labs(_ahrs.pitch_sensor) < 7000) {
        float roll_prop = (roll_wrapped - roll_limit_margin) / (float)(9000 - roll_limit_margin);
        desired_rate_degs *= (1 - roll_prop);
    }


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
/*
    // ############## START OF PID CONTROL ####################
    update_i(dt, limit_I, 1.0f);

    // Get native PID gains from ArduPilot parameters
    const float Kp = 0.3f;              //rate_pid.kP().get();
    const float Kd = 0.0001f;                  //rate_pid.kD().get();
    


    float P_out = (_custom_error * Kp);
    float D_out = (_custom_derivative * Kd);
    float I_out = _custom_integrator;

    // This modifier is used to reduce control effort under fast transients
    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    // scale pd output if required
    P_out *= 1.0f;
    D_out *= 1.0f;

    const float Kpdmax = rate_pid.pdmax();


    _pid_info.PD_limit = false;
    // Apply PD sum limit if enabled
    if (is_positive(Kpdmax)) {
        const float PD_sum_abs = fabsf(P_out + D_out);
        if (PD_sum_abs > Kpdmax) {
            const float pd_limit_scale = Kpdmax / PD_sum_abs;
            P_out *= pd_limit_scale;
            D_out *= pd_limit_scale;
            _pid_info.PD_limit = true;
        }
    }

    const float Kff = rate_pid.ff().get();
    const float Kdff = rate_pid.kDff().get();

    _pid_info.target = _custom_target;
    _pid_info.actual = measurement;
    _pid_info.error = _custom_error;

    _pid_info.P = P_out;
    _pid_info.D = D_out;
    _pid_info.I = I_out;

    _pid_info.limit = limit_I;

    // Set I set flag for logging and clear
    _pid_info.I_term_set = _custom_I_set;
    _custom_I_set = false;

    _pid_info.FF = _custom_target * Kff;
    _pid_info.DFF = _custom_target_derivative * Kdff;


    // ###################### Update_all ends ########################

    const float ff = degrees(ff_scale *  _pid_info.FF / (scaler * eas2tas));
    const float dff = degrees(ff_scale * _pid_info.DFF / (scaler * eas2tas));

    ff_scale = 1.0;

    if (disable_integrator) {
    _custom_integrator = 0.0f;
    _custom_I_set = true;
    }

    
    auto &pinfo = _pid_info;

    const float deg_scale = degrees(1.0f);
    pinfo.FF = ff;
    pinfo.P *= deg_scale;
    pinfo.I *= deg_scale;
    pinfo.D *= deg_scale;
    pinfo.DFF = dff;

    // fix the logged target and actual values to not have the scalers applied
    pinfo.target = desired_rate_degs;
    pinfo.actual = degrees(rate_rads);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D + pinfo.DFF;
    
    if (ground_mode) {
        // when on ground suppress D and half P term to prevent oscillations
        out -= pinfo.D + 0.5*pinfo.P;
    }

    // remember the last output to trigger the I limit
    _last_out = out;

    if (autotune != nullptr && autotune->running && get_airspeed() > aparm.airspeed_min) {
        // let autotune have a go at the values
        autotune->update(pinfo, scaler, angle_err_deg);
    }

    // output is scaled to notional centidegrees of deflection
    return constrain_float(out * 100, -4500, 4500);
*/

    // ############## END OF PID CONTROL ####################

    const float Kff = rate_pid.ff().get();
    const float Kdff = rate_pid.kDff().get();

    const float ff = degrees(ff_scale *  _pid_info.FF / (scaler * eas2tas));
    const float dff = degrees(ff_scale * _pid_info.DFF / (scaler * eas2tas));
    
    update_i(dt, limit_I, 1.0f);

    float I_out = _custom_integrator;

    _pid_info.I = I_out;
    _pid_info.limit = limit_I;

    // Set I set flag for logging and clear
    _pid_info.I_term_set = _custom_I_set;
    _custom_I_set = false;

    _pid_info.FF = _custom_target * Kff;
    _pid_info.DFF = _custom_target_derivative * Kdff;

    if (disable_integrator) {
    _custom_integrator = 0.0f;
    _custom_I_set = true;
    }

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


void AP_PitchController::update_i(float dt, bool limit, float i_scale)
{   
    const float Ki =   0.8f;              //rate_pid.kI().get();
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


