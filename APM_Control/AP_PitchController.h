#pragma once

#include "AP_FW_Controller.h"
#include "q_controller_fcn.h"
#include "theta_controller_fcn.h"

class AP_PitchController : public AP_FW_Controller
{
public:
    AP_PitchController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PitchController);

    static const struct AP_Param::GroupInfo var_info[];

    void convert_pid();


    float run_custom_angle_control(
        int32_t desired_angle_cd,
        float scaler,
        bool disable_integrator,
        bool ground_mode);




private:
    AP_Float _roll_ff;

    float run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode) override;

    float run_custom_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode);

    uint32_t last_run_ticks;

    q_controller_fcn _q_hinf_controller;
    theta_controller_fcn _theta_hinf_controller;

    bool _use_custom_angle_controller = false;
    bool _use_custom_rate_controller  = true;
    

    // Return true if the airspeed should be considered as under speed
    bool is_underspeed() const override;

    // Return the measured pitch angle in degrees
    float get_measured_angle_deg() const override;

    // Return the measured pitch rate in radians per second
    float get_measured_rate_rads() const override;

    // Return true if rate limits should be applied
    bool should_apply_rate_limits() const override;

    // Return rate target offset in deg per second, this is used in angle control
    float get_rate_target_offset_degs() const override;

    // Return positive rate limit in deg per second, zero if disabled
    float get_positive_rate_limit_degs() const override;

    // Return negative rate limit in deg per second (as a positive number) zero if disabled
    float get_negative_rate_limit_degs() const override;

    // Return true if the vehicle is inverted
    bool is_inverted() const;

};
