#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_PID.h>
#include "AP_AutoTune.h"

class AP_YawController
{
public:
    AP_YawController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_YawController);

    // return true if rate control or damping is enabled
    bool enabled() const { return rate_control_enabled() || (_K_D > 0.0); } 

    // return true if rate control is enabled
    bool rate_control_enabled(void) const { return _rate_enable != 0; }

    // get actuator output for sideslip and yaw damping control
    int32_t get_servo_out(float scaler, bool disable_integrator);

    // get actuator output for direct rate control
    // desired_rate is in deg/sec. scaler is the surface speed scaler
    float get_rate_out(float desired_rate, float scaler, bool disable_integrator);

    void reset_I();

    void reset_rate_PID();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }

    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) { rate_pid.set_notch_sample_rate(sample_rate); }

    // start/stop auto tuner
    void autotune_start(void);
    void autotune_restore(void);
    

    static const struct AP_Param::GroupInfo var_info[];

    int32_t get_custom_servo_out(
        float scaler,
        bool disable_integrator);

private:
    const AP_FixedWing &aparm;
    AP_Float _K_A;
    AP_Float _K_I;
    AP_Float _K_D;
    AP_Float _K_FF;
    AP_Int16 _imax;
    AP_Int8  _rate_enable;
    AC_PID rate_pid{0.04, 0.15, 0, 0.15, 0.666, 3, 0, 12, 150, 1};

    uint32_t _last_t;
    float _last_out;
    float _last_rate_hp_out;
    float _last_rate_hp_in;
    float _K_D_last;

    float _integrator;

    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune;
    bool failed_autotune_alloc;
    
    AP_PIDInfo _pid_info;

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


    float _custom_error = 0.0f;
    bool _custom_I_set = false;
    float _custom_target = 0.0f;
};
