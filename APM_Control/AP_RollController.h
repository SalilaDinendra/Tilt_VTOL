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

    // Successfull : Folder 11p
    // Don't change the inner loop.
    // Define the order of the H-infinity controller
    static constexpr uint8_t HINF_ORDER = 6;

    // State vector
    float _hinf_x[HINF_ORDER] = {0.0f};

    // Matrix dimensions scale automatically based on HINF_ORDER
    const float Ak[HINF_ORDER][HINF_ORDER] = {
        {  0.990049834F,     2.08804572E-24F,  -1.51282684E-16F, -2.11069195E-17F, -3.32022072E-17F,  3.04976279E-17F  },
        {  6.87113583E-07F,  4.54256274E-05F, -1.68968177F,     -0.462571158F,    -0.572722458F,     0.00457564433F   },
        { -4.28635717E-07F, -1.43654083E-08F,  0.943599535F,    -0.0210808446F,   -0.0205007813F,   -0.00224334115F  },
        { -4.33517996E-09F, -2.65383198E-10F,  0.0194312617F,    0.999787855F,    -0.000206981671F, -2.26504498E-05F  },
        { -2.90648048E-11F, -2.43882865E-12F,  0.00019619255F,   0.0199985812F,    0.999998614F,    -1.51730103E-07F  },
        { -1.82270705E-14F, -1.87687129E-15F,  1.64281041E-07F,  2.49991116E-05F,  0.00249999913F,   1.0F             }
    };

    const float Bk[HINF_ORDER] = {
         0.31840532F,
         1.01099497E-07F,
        -6.93628794E-08F,
        -4.65036876E-10F,
        -2.33306499E-12F,
        -1.16919131E-15F
    };

    const float Ck[HINF_ORDER] = {
        -4.43412593E-05F,
        -1.51361254E-05F,
        -0.000815226436F,
        -0.000989133667F,
        -0.00042421191F,
        -0.00285331645F
    };

    const float Dk = 0.0f;

    // Successfull: Folder 11p
    // Outer loop (Roll Angle to Roll Rate) H-infinity controller definitions
    static constexpr uint8_t OUTER_HINF_ORDER = 12;

    // State vector initialized to zero
    float _outer_hinf_x[OUTER_HINF_ORDER] = {0.0f};

    // Parallel integrator state
    float _outer_integrator = 0.0f;

    // Outer loop state-space matrices (Discretized at loop dt, 12x12)
    const float Outer_Ak[OUTER_HINF_ORDER][OUTER_HINF_ORDER] = {
        {  0.997503122F,    0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F,              0.0F             },
        {  1.2430637E-07F,  0.975309913F,     -8.71412088E-07F,  -2.95972289E-08F,  -1.53731957E-05F,  -1.93317465E-05F,  -8.12612464E-06F,  -5.63701788E-05F,   2.37970536E-07F,   6.4933584E-08F,    8.10908576E-08F,  -1.37227486E-09F },
        { -0.420685751F,   -0.00160665931F,    0.989936883F,     -1.65401674E-08F,   0.0117382486F,     0.0345883902F,     0.0122503217F,     0.0980692628F,    -0.535106105F,    -0.145685514F,    -0.182033475F,     0.00449531942F  },
        { -8.2629675E-08F,  -3.16300224E-10F,   6.8709138E-07F,    4.54256274E-05F, -1.68968177F,     -0.462571151F,    -0.572722455F,      0.00457566359F,  -1.71804288E-07F,  -4.68956008E-08F, -5.85596261E-08F,  9.19693303E-10F },
        {  6.16452654E-08F,  2.35874291E-10F,  -4.28619156E-07F,  -1.4365405E-08F,   0.943599533F,    -0.0210808496F,   -0.0205007831F,   -0.00224335552F,   1.17687842E-07F,   3.21123801E-08F,   4.0102946E-08F,   -6.80350838E-10F },
        {  3.10207755E-10F,  1.18830478E-12F,  -4.33509659E-09F,  -2.65383177E-10F,  0.0194312617F,    0.999787855F,    -0.00020698168F,   -2.26505221E-05F,   7.92865379E-10F,   2.16583817E-10F,   2.70404492E-10F,  -3.53437675E-12F },
        {  1.24613596E-12F,  4.77714876E-15F,  -2.90644698E-11F,  -2.43882855E-12F,  0.00019619255F,   0.0199985812F,    0.999998614F,    -1.51730393E-07F,   3.98930538E-12F,   1.09046634E-12F,   1.36123004E-12F,  -1.46424405E-14F },
        {  5.20854748E-16F,  1.9977494E-18F,   -1.82269305E-14F,  -1.87687124E-15F,  1.64281041E-07F,  2.49991116E-05F,   0.00249999913F,   1.0F,              2.00305472E-15F,   5.47770957E-16F,   6.83711395E-16F,  -6.3054852E-18F  },
        {  1.50570824F,      0.00572860026F,    0.00040375838F,   3.25460852E-08F, -0.0420389547F,   -0.12384619F,     -0.043863178F,    -0.351139927F,     0.93835815F,     -0.0256699488F,    -0.0224812324F,    -0.0150038987F  },
        {  0.0152232044F,    5.81397683E-05F,   4.08731561E-06F,   5.98862226E-10F, -0.0004247676F,   -0.00125163731F,  -0.000443297868F, -0.00354879625F,   0.0193783133F,     0.999741458F,    -0.000226999992F, -0.000151630029F },
        {  0.00010204464F,   3.9046197E-07F,    2.74154442E-08F,   5.49629293E-12F, -2.84644917E-06F, -8.38839121E-06F, -2.97095152E-06F, -2.37839381E-05F,  0.000195837771F,   0.0199982702F,     0.999998479F,    -1.01619753E-06F },
        {  6.39873495E-08F,  2.45116974E-10F,   1.71973812E-11F,   4.22708751E-15F, -1.784548E-09F,   -5.25935421E-09F, -1.86272656E-09F, -1.4912108E-08F,   1.64058632E-07F,   2.49989166E-05F,   0.00249999905F,   0.999999999F    }
    };

    const float Outer_Bk[OUTER_HINF_ORDER] = {
         0.0799000833F,
         2.49799143E-09F,
        -0.0112840494F,
        -1.53923171E-09F,
         1.24083102E-09F,
         4.98454517E-12F,
         1.66701607E-14F,
         4.95121364E-18F,
         0.0608928176F,
         0.000408178561F,
         2.04759518E-06F,
         1.02603723E-09F
    };

    const float Outer_Ck[OUTER_HINF_ORDER] = {
        155.597767F,
        0.598738899F,
        0.0418370334F,
        1.88856738E-05F,
        -4.33710823F,
        -12.7842476F,
        -4.52790388F,
        -36.2484022F,
        -0.541083102F,
        -0.475220173F,
        -0.204923688F,
        -1.32048433F
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
