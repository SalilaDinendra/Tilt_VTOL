#include "mode.h"
#include "Plane.h"

void ModeCustom::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();


    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }

    // Converting pilot inputs to radians for the pole-placement controller
    phi_ref   = plane.nav_roll_cd * 0.01f * DEG_TO_RAD;
    theta_ref = plane.nav_pitch_cd * 0.01f * DEG_TO_RAD;
    psi_ref   = ahrs.get_yaw_rad();

    // Rate references initially zero
    p_ref = 0.0f;
    q_ref = 0.0f;
    r_ref = 0.0f;


    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}

void ModeCustom::run()
{
    // Run base class function and then output throttle
    //Mode::run();


    // 2. Read actual aircraft states from ArduPilot AHRS
    const Vector3f &gyro = ahrs.get_gyro();

    // 3. Build the input structure expected by the Simulink
    pole_controller0::ExtU_pole_controller0_T controller_input{};

    // Reference angular rates [rad/s]
    controller_input.p_ref = p_ref;
    controller_input.q_ref = q_ref;
    controller_input.r_ref = r_ref;

    // Reference attitude angles [rad]
    controller_input.phi_ref   = phi_ref;
    controller_input.theta_ref = theta_ref;
    controller_input.psi_ref   = psi_ref;

    // Measured angular rates [rad/s]
    controller_input.p = gyro.x;
    controller_input.q = gyro.y;
    controller_input.r = gyro.z;

    // Measured attitude angles [rad]
    controller_input.phi   = ahrs.get_roll_rad();
    controller_input.theta = ahrs.get_pitch_rad();
    controller_input.psi   = ahrs.get_yaw_rad();

    //------------------------------------------------------------//
    // 4. Send inputs to generated Simulink controller
    pole_controller_model.setExternalInputs(&controller_input);

    // 5. Execute one controller step
    pole_controller_model.step();

    // 6. Read generated controller outputs
    const auto &controller_output = pole_controller_model.getExternalOutputs();
    const float aileron_cmd_deg = controller_output.Aileron_Cmd;
    const float rv_left_cmd_deg = controller_output.RV_Left_Cmd;
    const float rv_right_cmd_deg = controller_output.RV_Right_Cmd;

    
    // 7. Convert controller outputs from degrees to ardupilot units (centidegrees)
    const int16_t aileron_cmd_cd = static_cast<int16_t>(aileron_cmd_deg * 100.0f);
    const int16_t rv_left_cmd_cd = static_cast<int16_t>(rv_left_cmd_deg * 100.0f);
    const int16_t rv_right_cmd_cd = static_cast<int16_t>(rv_right_cmd_deg * 100.0f);

    // 8. Output the commands to the servos
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron_cmd_cd);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rv_left_cmd_cd);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rv_right_cmd_cd);


    output_pilot_throttle();
}
