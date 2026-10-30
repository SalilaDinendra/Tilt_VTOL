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


    // Successfull : Folder 9g
    // Define the order of the H-infinity controller (e.g., 3 for a 3rd-order system)
    static constexpr uint8_t HINF_ORDER = 6;

    // State vector
    float _hinf_x[HINF_ORDER] = {0.0f};

    // Matrix dimensions scale automatically based on HINF_ORDER
    const float Ak[HINF_ORDER][HINF_ORDER] = {
        { 0.990049839F,    -8.40007096E-25F,  6.81606949E-17F,  9.41181156E-18F, -5.82217568E-18F, -2.5150776E-18F },
        { 4.90276534E-7F,   4.54154142E-5F,  -1.12833238F,     -0.293970913F,    -0.0119801369F,    0.00625127042F  },
        {-4.42297676E-7F,  -1.23757644E-8F,   0.898580372F,    -0.121947937F,    -0.00323902769F,  -0.00564222736F  },
        {-1.80380919E-8F,  -9.35286182E-10F,  0.0759395361F,    0.99503541F,     -0.000131670284F, -0.000229708603F },
        {-6.07048092E-11F, -4.32827862E-12F,  0.000386460102F,  0.00998330675F,   0.999999583F,    -7.72398209E-7F  },
        {-7.6314958E-14F,  -6.68526344E-15F,  6.49738581E-7F,   2.49790246E-5F,   0.00499999942F,   1.0F            }
    };

    const float Bk[HINF_ORDER] = {
         0.31840533F,
         7.24015337E-8F,
        -7.21523676E-8F,
        -1.9425539E-9F,
        -4.884143E-12F,
        -4.89677582E-15F
    };

    const float Ck[HINF_ORDER] = {
        -9.36645E-5F, 
        -2.72398884E-5F, 
        -0.00252690585F, 
        -0.00273351395F, 
        -0.0294189714F, 
        -0.000697141048F
    };

    const float Dk = 0.0f;

    // Successfull : Folder 9g
    // Outer loop (Angle to Rate) H-infinity controller definitions
    static constexpr uint8_t OUTER_HINF_ORDER = 12; 
    
    // State vector initialized to zero
    float _outer_hinf_x[OUTER_HINF_ORDER] = {0.0f};

    // Parallel integrator state
    float _outer_integrator = 0.0f;

    // Outer loop matrices (Discretized at loop dt, 12x12)
    const float Outer_Ak[OUTER_HINF_ORDER][OUTER_HINF_ORDER] = {
        { 0.990049839F,     1.2693936E-26F, -1.32018397E-20F, -2.15041135E-22F, -3.87673523E-19F, -4.49512486E-19F, -5.3670092E-18F, 1.94511107E-18F, 3.42468929E-21F, -1.89906306E-20F, -4.14870594E-20F, -7.79695276E-21F },
        { 9.10711395E-10F,  0.99900049F,    -1.16437562E-7F,  -3.40173E-9F,     -3.09752545E-6F,  -3.38580799E-6F,  -3.67480607E-5F, -8.52590745E-7F, 2.14827036E-8F,  6.87381707E-9F,   6.5173679E-11F, -5.30950839E-11F },
        {-0.0231506322F,   -3.46293261E-9F,  0.990035772F,    -7.97173616E-9F,  -0.00639685197F,  -0.00922386814F,  -0.0936440378F,  -0.233058453F,   -0.361664623F,   -0.10934031F,     0.000720965501F, 0.00135083951F  },
        {-3.27148175E-9F,  -4.88906147E-16F, 4.90274545E-7F,   4.54154142E-5F,  -1.12833238F,     -0.293970913F,    -0.0119801499F,   0.00625123736F, -8.34993088E-8F, -2.69363412E-8F,  -3.1569E-10F,    1.90695612E-10F },
        { 3.55079566E-9F,   5.30739463E-16F,-4.42295516E-7F,  -1.23757626E-8F,   0.898580372F,    -0.121947937F,    -0.00323901325F, -0.00564219151F,  8.30288158E-8F,  2.65413966E-8F,   2.44693543E-10F, -2.0701646E-10F },
        { 7.17401347E-11F,  1.07181109E-17F,-1.80380475E-8F, -9.35286071E-10F,  0.0759395361F,    0.99503541F,     -0.00013167F,    -0.00022970789F,  2.25031149E-9F,  7.39369843E-10F,  1.23239804E-11F, -4.18041643E-12F },
        { 1.44442184E-13F,  2.15742565E-20F,-6.0704719E-11F, -4.32827862E-12F,  0.000386460102F,  0.00998330675F,   0.999999583F,    -7.72396731E-7F,  5.6802436E-12F, 1.89630104E-12F,  3.96173369E-14F, -8.4143948E-15F },
        { 1.26446375E-16F,  1.79554129E-23F,-7.63148496E-14F,-6.68526259E-15F,  6.49738581E-7F,   2.49790246E-5F,   0.00499999942F,  1.0F,             5.70993084E-15F, 1.92648792E-15F,  4.56898704E-17F, -6.99350685E-18F },
        { 0.119380653F,     1.78851245E-8F,  7.26425496E-5F,   2.19078409E-8F,   0.0331224762F,    0.0477498285F,    0.484754026F,    1.20385933F,     0.896420836F,   -0.124367043F,   -0.0287358947F,  -0.00697793532F  },
        { 0.0048705861F,    7.28558547E-10F, 2.96383701E-6F,   1.6732612E-9F,   0.00134582946F,   0.0019406022F,    0.0197016913F,   0.0490326583F,   0.0758516F,      0.994936824F,    -0.00117011927F, -0.000284200185F },
        { 1.63944896E-5F,   2.45046973E-12F, 9.97651473E-9F,  7.76941272E-12F,  4.52096E-6F,     6.51966229E-6F,   6.61912491E-5F,  0.000164907135F,  0.000386164349F, 0.0099829752F,   0.999996066F,   -9.55811743E-7F  },
        { 2.0612644E-8F,    3.07955176E-15F, 1.25435166E-11F, 1.20201446E-14F,  5.67733416E-9F,   8.18780332E-9F,   8.31281071E-8F,  2.07233143E-7F,   6.49366939E-7F,  2.49786081E-5F,   0.00499999523F,  1.0F            }
    };//[cite: 68]

    const float Outer_Bk[OUTER_HINF_ORDER] = {
         0.31840533F,
         7.32178693E-11F,
        -0.0024897526F,
        -2.44329529E-10F,
         2.86960539E-10F,
         4.62234374E-12F,
         7.96140158E-15F,
        -2.24184528E-16F,
         0.0194823444F,
         0.000524623669F,
         1.31920922E-6F,
         1.32431188E-9F
    };//[cite: 66]

    const float Outer_Ck[OUTER_HINF_ORDER] = {
        25.3103657F, 
        3.77457741E-6F, 
        0.0153092351F, 
        2.19839694E-5F, 
        6.93555641F, 
        10.0058889F, 
        101.589355F, 
        253.960327F, 
       -0.457834393F, 
       -0.514361858F, 
       -5.40771389F, 
       -0.283827245F
    };//[cite: 68]
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
