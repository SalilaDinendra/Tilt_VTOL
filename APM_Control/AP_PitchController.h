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


        // Successfull : Folder 6g
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

    // Successfull : Folder 6g
    // Outer loop (Angle to Rate) H-infinity controller definitions
    static constexpr uint8_t OUTER_HINF_ORDER = 12; 
    
    // State vector initialized to zero
    float _outer_hinf_x[OUTER_HINF_ORDER] = {0.0f};

    // Parallel integrator state
    float _outer_integrator = 0.0f;

    // Outer loop matrices (Discretized at loop dt, 12x12)
    const float Outer_Ak[OUTER_HINF_ORDER][OUTER_HINF_ORDER] = {
        { 0.990049839F,     0.0F,            0.0F,             0.0F,             0.0F,             0.0F,             0.0F,            0.0F,             0.0F,            0.0F,             0.0F,             0.0F            },
        { 6.96141256E-10F,  0.99900049F,    -1.16437775E-7F,  -3.40173E-9F,     -3.09759889E-6F,  -3.38592031E-6F,  -3.67492066E-5F, -8.56602412E-7F,  2.14900524E-8F,  6.8822823E-9F,   1.50295068E-10F, -4.77189191E-11F },
        {-0.0176977124F,   -3.94954114E-9F,  0.990041F,       -5.77059289E-9F,  -0.00452306215F,  -0.00636311807F,  -0.0644882619F,  -0.131005853F,    -0.361851573F,   -0.10955552F,    -0.00144445151F,  0.00121416431F  },
        {-2.50065257E-9F,  -5.57548638E-16F, 4.90275283E-7F,   4.54154142E-5F,  -1.12833238F,     -0.293970913F,    -0.0119801462F,   0.00625125179F,  -8.35257E-8F,    -2.69667488E-8F, -6.21417806E-10F, 1.71383532E-10F },
        { 2.7142073E-9F,    6.05267139E-16F,-4.4229634E-7F,  -1.23757635E-8F,   0.898580372F,    -0.121947937F,    -0.00323901768F, -0.00564220734F,  8.3057472E-8F,   2.65744013E-8F,   5.76579284E-10F, -1.86055171E-10F },
        { 5.48348866E-11F,  1.22225365E-17F,-1.80380635E-8F, -9.35286071E-10F,  0.0759395361F,    0.99503541F,     -0.000131670095F,-0.00022970821F,  2.25089014E-9F,  7.40036643E-10F,  1.90263968E-11F, -3.75693573E-12F },
        { 1.10414411E-13F,  2.46015453E-20F,-6.07047537E-11F,-4.32827862E-12F,  0.000386460102F,  0.00998330675F,   0.999999583F,    -7.72397357E-7F,  5.68140804E-12F, 1.8976435E-12F,  5.31088099E-14F, -7.56170368E-15F },
        { 2.17389378E-16F,  1.99105369E-23F,-7.63145311E-14F,-6.68525709E-15F,  6.49738581E-7F,   2.49790246E-5F,   0.00499999942F,  1.0F,             5.71079862E-15F, 1.92820571E-15F,  5.81483817E-17F, -5.99952113E-18F },
        { 0.091277793F,     2.0401929E-8F,   4.53655E-5F,     1.58702296E-8F,   0.0234128423F,    0.0329324938F,    0.333753705F,    0.676822901F,    0.897386611F,   -0.123257078F,   -0.0175526161F,  -0.00627302891F  },
        { 0.00372336619F,   8.30935432E-10F, 1.85060787E-6F,  1.21124533E-9F,   0.000951603171F,  0.00133873033F,   0.0135676228F,   0.0275620483F,   0.0758909285F,   0.994982123F,    -0.00071454217F, -0.000255445542F },
        { 1.25318375E-5F,   2.79456731E-12F, 6.22877039E-9F,  5.62286459E-12F,  3.197149E-6F,     4.49813888E-6F,   4.55877307E-5F,  9.26892681E-5F,  0.000386296655F, 0.00998312794F,  0.999997616F,   -8.5903082E-7F   },
        { 1.57553526E-8F,   3.51180453E-15F, 7.83106652E-12F, 8.69820336E-15F,  4.01528366E-9F,   5.64944402E-9F,   5.72563259E-8F,  1.16473672E-7F,  6.49533149E-7F,  2.4978799E-5F,   0.00499999709F,  1.0F             }
    };//[cite: 61, 63]

    const float Outer_Bk[OUTER_HINF_ORDER] = {
         0.159202665F,
         2.7988183E-11F,
        -0.000951574824F,
        -9.33761332E-11F,
         1.09670065E-10F,
         1.76636819E-12F,
         2.79000752E-15F,
        -2.37542131E-15F,
         0.00744673237F,
         0.000200509399F,
         5.04171283E-7F,
         5.06109821E-10F
    };//[cite: 61]

    const float Outer_Ck[OUTER_HINF_ORDER] = {
        19.3421192F, 
        4.30350565E-6F, 
        0.0095206378F, 
        8.3682653E-6F, 
        4.90619802F, 
        6.90497732F, 
        69.9808426F, 
        142.708023F, 
       -0.254105359F, 
       -0.279167086F, 
       -3.04720855F, 
       -0.13444975F
    };//[cite: 63]
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
