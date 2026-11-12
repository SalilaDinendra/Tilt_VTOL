#pragma once

#include "AP_FW_Controller.h"

class AP_RollController : public AP_FW_Controller
{
public:
    AP_RollController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RollController);

    static const struct AP_Param::GroupInfo var_info[];

    void convert_pid();

    void set_in_recovery(void) {
        in_recovery = true;
    }

    float run_custom_angle_control(
        int32_t desired_angle_cd,
        float scaler,
        bool disable_integrator,
        bool ground_mode);

 
private:
    float run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode) override;
    float run_custom_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode);

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

    uint32_t last_run_ticks;

    // Successfull : Folder 24p (Stallion), Folder 16 (Comet, Don't change the inner loop)
    // Don't change the inner loop.
    // Define the order of the H-infinity controller
    static constexpr uint8_t HINF_ORDER = 6;

    // State vector
    float _hinf_x[HINF_ORDER] = {0.0f};

    // Matrix dimensions scale automatically based on HINF_ORDER
    const float Ak[HINF_ORDER][HINF_ORDER] = {
        {  0.990049834F,     0.0F,              0.0F,              0.0F,              0.0F,              0.0F              },
        {  1.5262232E-07F,   4.5425602E-05F,   -1.74036819F,       0.0148193884F,    -0.0526486558F,    0.00497668518F },
        { -9.4620447E-08F,  -1.43125698E-08F,   0.969858841F,      -0.0129667487F,    -0.0098652998F,   -0.00152817881F },
        { -9.52631982E-10F, -2.61178901E-10F,   0.0196974897F,      0.999870005F,     -9.91529658E-05F,  -1.53597345E-05F },
        { -6.37236169E-12F, -2.39046443E-12F,   0.000197979593F,    0.0199991323F,     0.999999337F,    -1.02658619E-07F },
        { -1.99539632E-15F, -9.17954159E-16F,   8.27012663E-08F,    1.24997286E-05F,   0.00124999979F,   1.0F            }
    };

    const float Bk[HINF_ORDER] = {
         0.31840532F,
         2.23836545E-08F,
        -1.52421117E-08F,
        -1.01957787E-10F,
        -5.10822278E-13F,
        -1.27935093E-16F
    };

    const float Ck[HINF_ORDER] = {
        -3.86228964E-05F,
        -5.88509592E-05F,
        -0.00892262754F,
        -0.0123357819F,
        -8.74313401E-05F,
        -0.00159668238F
    };

    const float Dk = 0.0f;

    // Successfull: Folder 24p (Stallion), Folder 20(Comet)
    // Outer loop (Roll Angle to Roll Rate) H-infinity controller definitions
    static constexpr uint8_t OUTER_HINF_ORDER = 12;

    // State vector initialized to zero
    float _outer_hinf_x[OUTER_HINF_ORDER] = {0.0f};

    // Parallel integrator state
    float _outer_integrator = 0.0f;

    // Outer loop state-space matrices (Discretized at loop dt, 12x12)
    const float Outer_Ak[OUTER_HINF_ORDER][OUTER_HINF_ORDER] = {
        {  0.997503125F,    -2.21861946E-10F,  6.43868964E-08F, -2.82064748E-08F,  9.50196893E-08F,  9.42108723E-11F,  3.52458346E-09F,  1.13461282E-10F, -4.31384501E-09F, -3.53441293E-11F, -1.8149044E-10F, -6.41350738E-12F },
        {  0.435827354F,     0.960194252F,    -0.74541142F,     0.191516391F,    -1.34786517F,    -0.0053264994F,  -0.0529179802F,   -0.00210733327F,  -0.0167645428F,   -0.00061799282F, -0.000301784845F,  2.68174132E-05F },
        { -0.439875117F,     0.0351228377F,    1.26458255F,    -0.403223729F,    -0.0564408844F,    0.0262661777F,    0.0169103581F,    0.00316489659F,  -0.0109694288F,    0.000547992425F, -0.000764089823F, -6.62167616E-05F },
        { -0.334946286F,     0.0267457655F,    1.0014997F,     -0.319278916F,    -0.0432986574F,    0.0199424072F,    0.012855317F,     0.00240793026F,  -0.00865775691F,    0.000416523079F, -0.000593492539F, -5.08477608E-05F },
        {  0.206482844F,    -0.0164856629F,   -0.562013191F,     0.17986145F,      0.0187383311F,   -0.0158174274F,   -0.0465679058F,   -0.00290268227F,   0.00519903223F,   -0.000257021641F,  0.000360599714F,  3.11545744E-05F },
        {  0.00174307476F, -0.000138755266F, -0.0214043097F,    0.00845653223F,  -0.0136006144F,    0.99979833F,     -0.0013629824F,   -5.99984583E-05F,  0.000730026627F,  -3.49932857E-07F,  2.93321652E-05F,  1.22604118E-06F },
        {  0.00563510125F, -0.000456834384F,  0.1320738F,     -0.0578638998F,    0.195106743F,     0.0201874154F,    1.00722744F,      0.000232592398F, -0.00885619016F,   -3.14962531E-05F, -0.000334959613F, -1.17819642E-05F },
        {  0.000663141248F, -5.3758173E-05F,  0.0156003092F,   -0.00683382449F,   0.0230241305F,    3.47970215E-05F,  0.00210387714F,   1.00002749F,     -0.00104504415F,   -3.71442984E-06F, -3.95270296E-05F, -1.39049859E-06F },
        {  0.794326014F,    -0.0633443007F,    0.0974636251F,   -0.0281170436F,    0.0895279262F,   -0.0486831195F,   -0.0316067639F,   -0.00583564556F,   0.000682554006F,  -0.00450725708F, -0.0376759878F,   -0.00131180576F },
        {  0.0162207621F,   -0.00129568277F, -0.000340953655F,  0.0103653957F,    0.0161696579F,   -0.000930533529F, -8.34843272E-05F, -9.68883565E-05F, -0.000245607381F,  0.999916271F,    -0.000763222906F, -2.65646414E-05F },
        { -0.00546622439F,  0.000443344959F, -0.132416116F,     0.0581963034F,   -0.195104111F,    -0.000198059284F, -0.00724229168F,  -0.00023410796F,   0.00886574328F,    0.0200307232F,    1.00032795F,      1.15396389E-05F },
        { -0.000662004608F, 5.36661022E-05F, -0.0155753411F,   0.00682299501F,  -0.0229870117F,   -2.22653575E-05F, -0.000852506663F, -2.7450808E-05F,   0.00104336456F,    1.62081837E-05F,  0.00128946081F,    1.00000139F     }
    };

    const float Outer_Bk[OUTER_HINF_ORDER] = {
         0.0799000832F,
         0.0171373016F,
        -0.0172934338F,
        -0.0126861325F,
         0.00859435308F,
         6.34000656E-05F,
        -6.47601235E-05F,
        -7.63691515E-06F,
         0.0633655248F,
         0.000615997722F,
         6.91099853E-05F,
         7.62585374E-06F
    };

    const float Outer_Ck[OUTER_HINF_ORDER] = {
       17869.9252F,
       -1428.63292F,
      397016.402F,
     -156751.878F,
      616605.512F,
         605.070706F,
       22872.8842F,
         737.221371F,
      -28263.0797F,
         -95.1436289F,
       -1065.6971F,
         -37.0387725F
    };
    const float Outer_Dk = 0.0f;

    // Return true if the airspeed should be considered as under speed
    bool is_underspeed() const override;

    // Return the measured roll angle in degrees
    float get_measured_angle_deg() const override;

    // Return the measured roll rate in radians per second
    float get_measured_rate_rads() const override;

    // Return true if rate limits should be applied
    bool should_apply_rate_limits() const override;

    // Return positive rate limit in deg per second, zero if disabled
    float get_positive_rate_limit_degs() const override;

    // Return negative rate limit in deg per second (as a positive number) zero if disabled
    float get_negative_rate_limit_degs() const override;

    bool in_recovery;
};
