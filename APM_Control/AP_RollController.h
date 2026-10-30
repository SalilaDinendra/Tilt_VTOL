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

    /*
      set the in_recovery flag, which is used during a VTOL upset recovery
      this flag only lasts one loop
    */
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

    // Successfull : Folder 2_p roll rate controller
    // Define the order of the H-infinity controller (e.g., 3 for a 3rd-order system)
    static constexpr uint8_t HINF_ORDER = 6;

    // State vector
    float _hinf_x[HINF_ORDER] = {0.0f};

    // Matrix dimensions scale automatically based on HINF_ORDER
    const float Ak[HINF_ORDER][HINF_ORDER] = {
        {  0.990049834F,     0.0F,             0.0F,             0.0F,             0.0F,             0.0F            },
        {  4.1856826E-06F,   4.54268845E-05F, -1.68963132F,    -0.462525736F,    -0.572701249F,     0.00470913977F },
        { -2.61111184E-06F, -1.50681371E-08F,  0.943568103F,    -0.0211091742F,   -0.0205139968F,   -0.002326653F    },
        { -2.64086769E-08F, -2.78369408E-10F,  0.0194309422F,    0.999787569F,     -0.00020711589F,  -2.34917576E-05F },
        { -1.77054966E-10F, -2.55818239E-12F,  0.000196190402F,  0.0199985793F,    0.999998613F,     -1.57366259E-07F },
        { -1.11034595E-13F, -1.96872826E-15F,  1.64279693E-07F,  2.49991104E-05F,  0.00249999913F,   1.0F            }
    };

    const float Bk[HINF_ORDER] = {
         0.31840532F,
         6.15869775E-07F,
        -4.2253883E-07F,
        -2.83287945E-09F,
        -1.42124267E-11F,
        -7.12221948E-15F
    };
    const float Ck[HINF_ORDER] = {
        -0.000270117234F,
        -1.58770315E-05F,
        -0.0041171456F,
        -0.0039269029F,
        -0.00180874603F,
        -0.0114322853F
    };

    const float Dk = 0.0f;

    
    // Successfull : Folder 2_p roll angle controller
    // Outer loop (Roll Angle to Roll Rate) H-infinity controller definitions
    static constexpr uint8_t OUTER_HINF_ORDER = 12;

    // State vector initialized to zero
    float _outer_hinf_x[OUTER_HINF_ORDER] = {0.0f};

    // Parallel integrator state
    float _outer_integrator = 0.0f;

    // Outer loop state-space matrices (Discretized at loop dt, 12x12)
    const float Outer_Ak[OUTER_HINF_ORDER][OUTER_HINF_ORDER] = {
        {  0.990049834F,     -1.59442694E-25F, 1.7303337E-20F,   1.07438962E-22F,  2.93949332E-19F,  3.4345366E-19F,   9.17132502E-20F,  7.55570379E-19F, -4.55270901E-21F,  2.47128616E-21F, -5.3637337E-22F,  1.01772922E-20F  },
        {  1.25197789E-06F,   0.9990005F,     -3.34312353E-07F, -1.98252578E-09F, -5.02669624E-06F, -4.93333684E-06F, -2.23724758E-06F, -1.4468255E-05F,   8.54050294E-08F,  1.85477151E-08F,  2.7867427E-08F,  -1.33418484E-08F },
        { -10.8677226F,      -2.00435091E-06F, 0.977232568F,    -9.73448082E-07F,  0.10315723F,      0.590029451F,     0.206249558F,     1.66001375F,     -0.483177634F,    -0.0902087034F,   -0.153620447F,    0.115771172F    },
        { -1.32856006E-05F,  -2.44795007E-12F, 4.17001375E-06F,  4.54268828E-05F, -1.68963119F,     -0.462525015F,    -0.572700998F,     0.00471116699F,  -9.83795183E-07F, -2.18073032E-07F, -3.22221757E-07F,  1.41596499E-07F  },
        {  9.87297204E-06F,   1.81946677E-12F,-2.59946779E-06F, -1.50669118E-08F,  0.94356801F,     -0.0211097096F,   -0.0205141839F,   -0.00232815978F,   6.7016728E-07F,   1.45352993E-07F,  2.1862173E-07F,  -1.0521141E-07F  },
        {  5.0210712E-08F,    9.2488932E-15F, -2.63494591E-08F, -2.78361709E-10F,  0.0194309417F,    0.999787566F,    -0.000207116841F, -2.34994167E-05F,   4.59343825E-09F,  1.06413611E-09F,  1.51708492E-09F, -5.35450658E-10F  },
        {  2.03117801E-10F,   3.74031117E-17F,-1.76815412E-10F, -2.5581462E-12F,   0.0001961904F,    0.0199985793F,    0.999998613F,    -1.57397232E-07F,   2.33486042E-11F,  5.61141816E-12F,  7.76690832E-12F, -2.16796614E-12F  },
        {  8.53123364E-14F,   1.57064082E-20F,-1.10933979E-13F, -1.96871124E-15F,  1.64279692E-07F,  2.49991104E-05F,  0.00249999913F,   1.0F,              1.18024358E-14F,  2.90390286E-15F,  3.94456585E-15F, -9.11462435E-16F  },
        {  37.5110979F,       6.9295795E-06F,  0.044240329F,     1.7246458E-06F,  -0.360354858F,    -2.04145522F,     -0.714421455F,    -5.74000246F,      0.75577048F,     -0.218205278F,    -0.121649259F,    -0.399267533F    },
        {  0.39327686F,       7.25325736E-08F, 0.000463826148F,  3.5247577E-08F,  -0.00373298022F,  -0.0213517156F,   -0.00746365833F,  -0.0600718446F,    0.017499172F,     0.997733893F,    -0.00125519101F,  -0.00417843527F  },
        {  0.00268330023F,    4.94495465E-10F, 3.164647E-06F,    3.34256591E-10F, -2.53219553E-05F,  -0.000145512883F, -5.083705E-05F,   -0.000409513166F,  0.000183132106F,  0.019984609F,     0.999991502F,    -2.84843424E-05F  },
        {  1.70033409E-06F,   3.13203047E-13F, 2.00534816E-09F,  2.61244587E-13F, -1.59904834E-08F,  -9.21446712E-08F, -3.21815342E-08F,  -2.59365658E-07F,  1.56050702E-07F,  2.49902734E-05F,  0.00249999464F,   0.999999982F    }
    };

    const float Outer_Bk[OUTER_HINF_ORDER] = {
         0.31840532F,
         1.01576917E-07F,
        -1.1868584F,
        -9.99427845E-07F,
         8.03371392E-07F,
         3.24988548E-09F,
         1.09201694E-11F,
         3.92411012E-15F,
         6.29242977F,
         0.0429328037F,
         0.000217642764F,
         1.09821199E-07F
    };

    const float Outer_Ck[OUTER_HINF_ORDER] = {
        4309.6595F,
        0.000792403909F,
        5.08245471F,
        0.0021920903F,
        -39.9800189F,
        -232.929077F,
        -81.2447735F,
        -656.093843F,
        -20.4046317F,
        -22.2458789F,
        -11.232742F,
        -45.4055002F
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
