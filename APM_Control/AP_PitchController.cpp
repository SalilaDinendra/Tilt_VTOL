
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
        return run_axis_rate_control(desired_rate_degs, scaler, disable_integrator, ground_mode);
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
    desired_rate_degs = rate_limit_degs(desired_rate_degs + rate_target_degs + get_rate_target_offset_degs()); */

    /// CUSTOM PART ///
    
    // Convert desired pitch cd to degrees
    const float desired_angle_deg = wrap_180(desired_angle_cd * 0.01f);

    // Measured pitch angle in degrees
    const float measured_angle_deg = get_measured_angle_deg();

    // Pitch angle error in degrees
    const float angle_error_deg = wrap_180(desired_angle_deg - measured_angle_deg);

    const float angle_error_rad = angle_error_deg*DEG_TO_RAD;

    theta_controller_fcn::ExtU_theta_controller_fcn_T hinf_input{};

    hinf_input.theta_error = angle_error_rad;

    _theta_hinf_controller.setExternalInputs(&hinf_input);

    _theta_hinf_controller.step();

    const auto &hinf_output = _theta_hinf_controller.getExternalOutputs();

    const float desired_rate_rads = hinf_output.q_command;
    float desired_rate_degs = degrees(desired_rate_rads);

    // Ardupilot coordinated-turn pitch compensation
    desired_rate_degs += get_rate_target_offset_degs();

    // Ardupilot pitch rate limits
    if (should_apply_rate_limits()){
        desired_rate_degs = rate_limit_degs(desired_rate_degs);
    }

    return run_axis_rate_control(
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

    //const float eas2tas = AP::ahrs().get_EAS2TAS();
    bool limit_I = fabsf(_last_out) >= 45;
    const float rate_rads = get_measured_rate_rads();
    //const float old_I = rate_pid.get_i();

    const bool underspeed = is_underspeed();
    if (underspeed) {
        limit_I = true;
    }

    rate_pid.update_all(radians(desired_rate_degs) * scaler * scaler, rate_rads * scaler * scaler, dt, limit_I);

    const auto &native_pid_info = rate_pid.get_pid_info();
    const float filtered_rate_error_rads = native_pid_info.error;


    // From Simulink Controller
    q_controller_fcn::ExtU_q_controller_fcn_T hinf_input{};
    
    hinf_input.q_error = filtered_rate_error_rads;

    _q_hinf_controller.setExternalInputs(&hinf_input);

    _q_hinf_controller.step();

    const auto &hinf_output = _q_hinf_controller.getExternalOutputs();
    const float hinf_pitch_command = hinf_output.pitch_command;

    return hinf_pitch_command*100.0f;
}


