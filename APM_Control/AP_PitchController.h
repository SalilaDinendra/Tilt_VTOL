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

    void update_i(float dt, bool limit, float i_scale = 1.0f);

    bool _custom_reset_filter = true;
    bool _custom_I_set = false;
    float _custom_target = 0.0f;
    float _custom_error = 0.0f;
    float _custom_derivative = 0.0f;
    float _custom_target_derivative = 0.0f;
    float _custom_integrator = 0.0f;

#if AP_FILTER_ENABLED
    NotchFilterFloat* _target_notch;
    NotchFilterFloat* _error_notch;
#endif


    // Successfull : Folder 9g (Stallion), Folder 10(Comet)
    // Define the order of the H-infinity controller (e.g., 3 for a 3rd-order system)
    static constexpr uint8_t HINF_ORDER = 6;

    // State vector
    float _hinf_x[HINF_ORDER] = {0.0f};

    // Matrix dimensions scale automatically based on HINF_ORDER
    const float Ak[HINF_ORDER][HINF_ORDER] = {
        {  0.990049834F,    -1.07475755E-24F,  7.46920762E-17F,  2.79637705E-17F, -5.100999E-17F,   -1.84634697E-17F },
        {  1.47333868E-06F,  4.54120709E-05F, -0.764986799F,   -0.39251099F,    -0.0603416974F,    0.00372404263F  },
        { -2.08029922E-06F, -1.52961971E-08F,  0.961326442F,   -0.0202531592F,  -0.00656615788F,  -0.0052606513F  },
        { -4.19501848E-08F, -5.60479795E-10F,  0.0392241985F,   0.99959233F,    -0.000132096956F, -0.0001059046F  },
        { -3.51022546E-11F, -6.42068601E-13F,  4.93525283E-05F,  0.00249965919F,   0.99999989F,    -8.85425107E-08F },
        { -4.39856009E-14F, -9.86877494E-16F,  8.25232661E-08F,  6.24957317E-06F,  0.00499999986F,   1.0F            }
    };

    const float Bk[HINF_ORDER] = {
         0.63681064F,
         4.31374882E-07F,
        -6.71202957E-07F,
        -8.98617717E-09F,
        -5.63018358E-12F,
        -5.65572695E-15F
    };

    const float Ck[HINF_ORDER] = {
        -0.000426427938F,
        -3.16980133E-05F,
        -0.119569269F,
        -0.110325607F,
        -0.900257393F,
        -0.0248149697F
    };

    const float Dk = 0.0f;

    // Successfull : Folder 9g (Stallion), Folder 10(Comet)
    // Outer loop (Roll Angle to Roll Rate) H-infinity controller definitions
    static constexpr uint8_t OUTER_HINF_ORDER = 12;

    // State vector initialized to zero
    float _outer_hinf_x[OUTER_HINF_ORDER] = {0.0f};

    // Parallel integrator state
    float _outer_integrator = 0.0f;

    // Outer loop state-space matrices (Discretized at loop dt, 12x12)
    const float Outer_Ak[OUTER_HINF_ORDER][OUTER_HINF_ORDER] = {
        {  0.990049834F,     0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F              },
        {  8.89938824E-10F,  0.999500125F,    -2.650409E-07F,   -1.9790791E-09F,  -7.46142438E-05F, -6.88552349E-05F, -0.000562247701F, -1.52254239E-05F,  6.3877643E-08F,   3.3042761E-08F,   3.401078E-09F,   -2.40765726E-10F },
        { -0.00997775866F,  -2.97610925E-10F,  0.990019708F,    -7.68875629E-09F, -0.0586825938F,   -0.0695657369F,   -0.677726368F,    -0.895528925F,    -0.47626208F,     -0.244998981F,    -0.0180647405F,    0.00270166616F },
        { -4.16240952E-09F, -1.24033548E-16F,  1.47332611E-06F,   4.54120708E-05F, -0.764986823F,    -0.392511019F,    -0.0603419796F,    0.00372366941F, -3.24554714E-07F, -1.68032814E-07F, -1.80640699E-08F,  1.12589987E-09F },
        {  7.05206725E-09F,  2.10178527E-16F, -2.08027792E-06F, -1.52961895E-08F,  0.961326484F,    -0.0202531101F,   -0.00656567952F,  -0.00526001887F,   5.04566394E-07F,   2.60993996E-07F,   2.68159403E-08F, -1.90788869E-09F },
        {  7.08393343E-11F,  2.11027173E-18F, -4.19499708E-08F, -5.60479699E-10F,  0.039224199F,     0.99959233F,    -0.000132092155F, -0.000105898251F,  6.77273336E-09F,   3.51300048E-09F,   4.11868644E-10F, -1.91554391E-11F },
        {  3.55259963E-14F,  1.0579698E-21F,  -3.51021473E-11F, -6.42068545E-13F,  4.93525285E-05F,  0.00249965919F,   0.99999989F,    -8.85393272E-08F,  4.24994175E-12F,   2.20807257E-12F,   2.77894581E-13F, -9.60328394E-15F },
        {  2.97807286E-17F,  8.92202209E-25F, -4.39855147E-14F, -9.86877462E-16F,  8.25232663E-08F,  6.24957317E-06F,   0.00499999986F,   1.0F,              4.27349153E-15F,   2.22265683E-15F,   2.91947258E-16F, -8.10038152E-18F },
        {  0.0404831028F,    1.2094557E-09F,   0.000122142286F,  1.53657893E-08F, 0.238839557F,     0.283064303F,     2.7572542F,       3.6394154F,        0.949115124F,    -0.0324961119F,    -0.09085717F,    -0.0109801852F  },
        {  0.000818093658F,  2.44017042E-11F,  2.470027E-06F,    6.29465832E-10F, 0.00481151418F,   0.00570384435F,   0.0555682267F,    0.0734261787F,     0.0389780951F,     0.999345305F,    -0.00183275347F, -0.000221514948F },
        {  6.85268715E-07F,  2.04235549E-14F,  2.06971791E-09F,  7.46185249E-13F,  4.02409147E-06F,   4.77097282E-06F,  4.64835363E-05F,  6.14549243E-05F,   4.9146659E-05F,    0.00249945243F,    0.999998466F,    -1.85394117E-07F },
        {  8.59234316E-10F,  2.55961556E-17F,  2.59568726E-12F,  1.16620099E-15F,  5.04100078E-09F,   5.97706121E-09F,  5.82371166E-08F,  7.70188182E-08F,   8.22653417E-08F,   6.24931404E-06F,   0.00499999808F,   1.0F            }
    };

    const float Outer_Bk[OUTER_HINF_ORDER] = {
         0.31840532F,
         7.13801279E-11F,
        -0.00106881842F,
        -3.09402383E-10F,
         5.66714673E-10F,
         4.54734088E-12F,
         1.90276446E-15F,
         7.97200351E-18F,
         0.00654474926F,
         8.77143955E-05F,
         5.49909962E-08F,
         5.51041184E-11F
    };

    const float Outer_Ck[OUTER_HINF_ORDER] = {
        8.35087012F,
        2.48295504E-07F,
        0.0248218585F,
        7.66700473E-06F,
       48.6934491F,
       57.7836767F,
      563.2894F,
      747.073923F,
       -2.61825667F,
       -2.62413342F,
      -18.2053941F,
       -1.20548646F
    };
    const float Outer_Dk = 0.0f;


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
