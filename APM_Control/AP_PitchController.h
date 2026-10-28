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



    // Define the order of the H-infinity controller (e.g., 3 for a 3rd-order system) (from 12 folder)
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


    // Outer loop (Angle to Rate) H-infinity controller definitions (From 11 folder)
    static constexpr uint8_t OUTER_HINF_ORDER = 12; 
    
    // State vector initialized to zero
    float _outer_hinf_x[OUTER_HINF_ORDER] = {0.0f};

    // Parallel integrator state
    float _outer_integrator = 0.0f;

    // Outer loop matrices (Discretized at loop dt, 12x12)
    const float Outer_Ak[OUTER_HINF_ORDER][OUTER_HINF_ORDER] = {
        { 0.995012462F,    -3.27714386E-28F, 2.80804937E-21F,  5.39834557E-23F,  1.07143606E-19F,  1.15320868E-19F,  1.04141781E-18F, -3.21517866E-19F, -5.49735531E-22F,  2.79117234E-21F,  7.0734993E-21F,   1.29256311E-21F },
        { 9.85701104E-11F,  0.999500096F,   -5.82336455E-8F,  -1.70163061E-9F,  -1.5491994E-6F,   -1.69340353E-6F,  -1.83793945E-5F,  -4.29438217E-7F,   1.07487823E-8F,   3.44336937E-9F,   9.68735786E-11F, -2.22441475E-11F },
        {-0.00501353154F,  -4.92073493E-10F, 0.990050197F,    -4.79026818E-9F,  -0.00394356716F,  -0.00537824351F,  -0.0544264466F,   -0.0786405131F,  -0.361950904F,    -0.109638669F,    -0.00254906714F,  0.00113187381F  },
        {-7.08000769E-10F, -6.9457595E-17F,  4.90276591E-7F,   4.54154142E-5F,  -1.12833238F,     -0.293970913F,    -0.0119801443F,    0.00625125924F,  -8.35397174E-8F,  -2.69784941E-8F,  -7.77349518E-10F, 1.59759081E-10F },
        { 7.68545061E-10F,  7.54037134E-17F, -4.42297733E-7F, -1.23757635E-8F,   0.898580372F,    -0.121947937F,    -0.00323901931F,  -0.00564221526F,   8.30726918E-8F,   2.65871503E-8F,   7.45857487E-10F, -1.73437376E-10F },
        { 1.55224514E-11F,  1.52259321E-18F, -1.80380919E-8F, -9.35286182E-10F,  0.0759395361F,    0.99503541F,     -0.000131670124F, -0.00022970837F,   2.25119745E-9F,   7.40294159E-10F,  2.24446867E-11F, -3.50205303E-12F },
        { 3.12465304E-14F,  3.06457654E-21F, -6.07048092E-11F, -4.32827862E-12F,  0.000386460102F,  0.00998330675F,   0.999999583F,    -7.72397698E-7F,   5.68202646E-12F,  1.89816197E-12F,  5.99889875E-14F, -7.04858636E-15F },
        { 3.20605544E-17F,  2.55212181E-24F, -7.63149309E-14F, -6.68526302E-15F,  6.49738581E-7F,   2.49790246E-5F,   0.00499999942F,   1.0F,             5.71142246E-15F,  1.92801407E-15F,  6.26378218E-17F, -5.86301358E-18F },
        { 0.0258826222F,    2.54233545E-9F,  -2.02132446E-6F,  1.31715918E-8F,   0.0204029363F,    0.0278232601F,    0.281562686F,     0.406316459F,     0.897899687F,    -0.122828007F,    -0.0118462751F,  -0.00584842F    },
        { 0.00105478417F,   1.03526333E-10F, -8.2378115E-8F,  1.00547504E-9F,   0.000829682453F,  0.001131522F,     0.0114507135F,    0.0165450182F,    0.0759118274F,    0.994999588F,    -0.000482144445F, -0.000238132678F },
        { 3.54844815E-6F,   3.48144663E-13F, -2.77138562E-10F, 4.66792403E-12F,  2.78821358E-6F,   3.80272331E-6F,   3.84826926E-5F,   5.56376399E-5F,   0.000386366912F,  0.00998318661F,   0.999998391F,   -8.00773E-7F    },
        { 4.45995196E-9F,   4.37474385E-16F, -3.4833326E-13F, 7.22120829E-15F,  3.50221918E-9F,   4.77663908E-9F,   4.8338574E-8F,    6.99128719E-8F,   6.49621484E-7F,   2.49788736E-5F,   0.00499999803F,   1.0F            }
    }; //[cite: 11]

    const float Outer_Bk[OUTER_HINF_ORDER] = {
         0.15960066F,
         3.96098302E-12F,
        -0.000269442506F,
        -2.64304516E-11F,
         3.10449166E-11F,
         4.99970562E-13F,
         8.67267294E-16F,
        -2.23959323E-16F,
         0.00210956833F,
         5.67751704E-5F,
         1.42718463E-7F,
         1.43239101E-10F
    }; //[cite: 11]

    const float Outer_Ck[OUTER_HINF_ORDER] = {
        5.46919727F, 
        5.35986601E-7F, 
       -0.000520847447F, 
        2.32462298E-6F, 
        4.28147364F, 
        5.84070826F, 
        59.1052628F, 
        85.6519623F, 
       -0.145914212F, 
       -0.188389793F, 
       -1.84371352F, 
       -0.0445904434F
    }; //[cite: 23]

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
