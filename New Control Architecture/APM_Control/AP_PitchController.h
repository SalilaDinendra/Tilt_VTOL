#pragma once

#include "AP_FW_Controller.h"

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


    // Successfull : Folder 1 (Stallion, Controller New), Folder 22(Comet)
   // Outer loop (Roll Angle to Roll Rate) H-infinity controller definitions
    static constexpr uint8_t OUTER_HINF_ORDER = 12;

    // State vector initialized to zero
    float _outer_hinf_x[OUTER_HINF_ORDER] = {0.0f};

    // Parallel integrator state
    float _outer_integrator = 0.0f;

    // Outer loop state-space matrices (Discretized at loop dt, 12x12)
    const float Outer_Ak[OUTER_HINF_ORDER][OUTER_HINF_ORDER] = {
        {  0.990049834F,     3.01579973E-28F, -4.93891951E-20F, -2.14852085E-22F, -1.29089025E-17F, -1.18672077E-17F, -9.05579989E-17F,  4.78749301E-17F,  4.610656E-20F,   -1.28300838E-19F, -1.19791803E-18F, -1.4127358E-19F  },
        {  1.11255599E-10F,  0.999900005F,    -3.31367505E-08F, -2.4747399E-10F,  -9.32864668E-06F, -8.60862648E-06F, -7.0295022E-05F,  -1.90356156E-06F,  7.98577387E-09F,  4.13090005E-09F,  4.25206007E-10F, -3.00982155E-11F },
        { -0.00997794647F,  -5.95696054E-12F,  0.990019704F,    -7.68878663E-09F, -0.058682858F,    -0.0695656152F,   -0.67772288F,     -0.895505863F,    -0.476262096F,    -0.244999051F,    -0.0180653007F,    0.00270161985F   },
        { -4.16248785E-09F, -2.48254568E-18F,  1.4733261E-06F,   4.54120708E-05F, -0.764986823F,    -0.392511019F,    -0.0603419796F,    0.00372366942F,  -3.2455472E-07F,  -1.68032843E-07F, -1.80643034E-08F,  1.12588056E-09F  },
        {  7.05219996E-09F,  4.206779E-18F,   -2.08027792E-06F, -1.52961895E-08F,  0.961326484F,    -0.0202531101F,   -0.00656567952F,  -0.00526001888F,   5.04566405E-07F,  2.60994046E-07F,  2.68163359E-08F, -1.90785598E-09F },
        {  7.08406671E-11F,  4.22367959E-20F, -4.19499708E-08F, -5.60479699E-10F,  0.039224199F,     0.99959233F,     -0.000132092155F, -0.000105898251F,  6.77273347E-09F,  3.51300098E-09F,  4.11872617E-10F, -1.91551106E-11F },
        {  3.55267646E-14F,  2.1174817E-23F,  -3.51021473E-11F, -6.42068545E-13F,  4.93525285E-05F,  0.00249965919F,   0.99999989F,     -8.85393273E-08F,  4.2499418E-12F,   2.20807283E-12F,  2.77896651E-13F, -9.60311002E-15F },
        {  2.99276558E-17F,  1.78313377E-26F, -4.39855104E-14F, -9.8687744E-16F,   8.25232663E-08F,  6.24957317E-06F,  0.00499999986F,   1.0F,             4.27348728E-15F,  2.22266933E-15F,  2.92063244E-16F, -8.08676328E-18F },
        {  0.0404838653F,    2.42100304E-11F,  0.00012215898F,   1.53658497E-08F,  0.238840623F,     0.283063802F,     2.75723998F,      3.63932171F,      0.949115187F,    -0.0324958267F,   -0.0908548942F,   -0.0109799972F   },
        {  0.000818109057F,  4.88422954E-13F,  2.47036434E-06F,  6.29468316E-10F,  0.00481153584F,   0.00570383437F,   0.0555679407F,    0.0734242878F,    0.0389780964F,    0.99934531F,     -0.00183270754F,  -0.000221511151F },
        {  6.8528161E-07F,   4.08782813E-16F,  2.07000046E-09F,  7.46188199E-13F,  4.02410966E-06F,  4.77096452E-06F,  4.64832973E-05F,  6.14533414E-05F,  4.91466601E-05F,  0.00249945243F,   0.999998466F,    -1.85390938E-07F },
        {  8.59250477E-10F,  5.12303467E-19F,  2.59604139E-12F,  1.16620485E-15F,  5.04102359E-09F,  5.97705082E-09F,  5.82368172E-08F,  7.70168344E-08F,  8.2265343E-08F,   6.24931404E-06F,  0.00499999808F,   1.0F             }
    };

    const float Outer_Bk[OUTER_HINF_ORDER] = {
         0.31840532F,
         8.9234047E-12F,
        -0.00106883854F,
        -3.09408205E-10F,
         5.66725336E-10F,
         4.54742856E-12F,
         1.89820762E-15F,
         1.2495749E-18F,
         0.00654487246F,
         8.77160461E-05F,
         5.49920308E-08F,
         5.51053901E-11F
    };

    const float Outer_Ck[OUTER_HINF_ORDER] = {
         8.35102713F,
         4.96920141E-09F,
         0.0248253015F,
         7.66716091E-06F,
        48.6936726F,
        57.7835779F,
       563.286511F,
       747.05467F,
        -2.6182438F,
        -2.62407482F,
       -18.2049265F,
        -1.20544778F
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
