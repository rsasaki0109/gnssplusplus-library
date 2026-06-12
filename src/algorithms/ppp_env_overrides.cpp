#include <libgnss++/algorithms/ppp_env_overrides.hpp>

#include <cstdlib>

namespace libgnss {
namespace {

const char* envValue(const char* name) {
    return std::getenv(name);
}

bool envPresent(const char* name) {
    return envValue(name) != nullptr;
}

bool envExactOne(const char* name) {
    const char* value = envValue(name);
    return value != nullptr && value[0] == '1' && value[1] == '\0';
}

bool envFirstCharNotZero(const char* name) {
    const char* value = envValue(name);
    return value != nullptr && value[0] != '0';
}

bool envExactZero(const char* name) {
    const char* value = envValue(name);
    return value != nullptr && value[0] == '0' && value[1] == '\0';
}

double envDoubleOr(const char* name, double fallback) {
    const char* value = envValue(name);
    return value != nullptr ? std::atof(value) : fallback;
}

std::string envStringOrEmpty(const char* name) {
    const char* value = envValue(name);
    return value != nullptr ? std::string(value) : std::string();
}

}  // namespace

PPPEnvOverrides PPPEnvOverrides::fromEnvironment() {
    PPPEnvOverrides overrides;

    overrides.madoca_bias_subtract = envPresent("GNSS_PPP_MADOCA_BIAS_SUBTRACT");
    overrides.madoca_allow_partial_ssr = envExactOne("GNSS_PPP_MADOCA_ALLOW_PARTIAL_SSR");
    overrides.madoca_ssr_replay = envExactOne("GNSS_PPP_MADOCA_SSR_REPLAY");
    overrides.require_ssr_orbit = envFirstCharNotZero("GNSS_PPP_REQUIRE_SSR_ORBIT");
    overrides.disable_madoca_static_anchor =
        envExactOne("GNSS_PPP_DISABLE_MADOCA_STATIC_ANCHOR");
    overrides.static_anchor_blend = envDoubleOr("GNSS_PPP_STATIC_ANCHOR_BLEND", -1.0);
    overrides.madoca_early_window = !envExactZero("GNSS_PPP_MADOCA_EARLY_WINDOW");
    overrides.madoca_spike_guard = !envExactZero("GNSS_PPP_MADOCA_SPIKE_GUARD");
    overrides.madoca_boundary_guard = !envExactZero("GNSS_PPP_MADOCA_BOUNDARY_GUARD");
    overrides.madoca_postfit_commit =
        envExactOne("GNSS_PPP_MADOCA_POSTFIT_COMMIT");
    overrides.madoca_postfit_shadow_path =
        envStringOrEmpty("GNSS_PPP_MADOCA_POSTFIT_SHADOW");

    const char* isb = envValue("GNSS_PPP_ESTIMATE_ISB");
    const std::string isb_spec = isb != nullptr ? std::string(isb) : std::string();
    overrides.estimate_isb_all =
        isb_spec == "1" || isb_spec.find("all") != std::string::npos;
    overrides.estimate_isb_gal =
        overrides.estimate_isb_all || isb_spec.find("gal") != std::string::npos;
    overrides.estimate_isb_qzs =
        overrides.estimate_isb_all || isb_spec.find("qzs") != std::string::npos;
    overrides.estimate_isb_bds =
        overrides.estimate_isb_all || isb_spec.find("bds") != std::string::npos;

    overrides.isb_process_noise = envDoubleOr("GNSS_PPP_ISB_PROCESS_NOISE", 1e-6);
    overrides.init_iono_var = envDoubleOr("GNSS_PPP_INIT_IONO_VAR", 0.0);
    overrides.qzss_code_only = envFirstCharNotZero("GNSS_PPP_QZSS_CODE_ONLY");
    overrides.qzss_prefer_l1l_present = envPresent("GNSS_PPP_QZSS_PREFER_L1L");
    overrides.qzss_prefer_l1l = envFirstCharNotZero("GNSS_PPP_QZSS_PREFER_L1L");
    overrides.qzss_ssr_prn_fix =
        !envExactZero("GNSS_PPP_QZSS_SSR_PRN_FIX") &&
        !envExactOne("GNSS_PPP_DISABLE_QZSS_SSR_PRN_FIX");
    overrides.madoca_qzss_clock = !envExactZero("GNSS_PPP_MADOCA_QZSS_CLOCK");
    overrides.madoca_qzss_phase = envExactOne("GNSS_PPP_MADOCA_QZSS_PHASE");
    overrides.madoca_qzss_l5 = envExactOne("GNSS_PPP_MADOCA_QZSS_L5");
    overrides.madoca_glonass = !envExactZero("GNSS_PPP_MADOCA_GLONASS");
    overrides.madoca_glonass_phase = envExactOne("GNSS_PPP_MADOCA_GLONASS_PHASE");
    overrides.madoca_low_elev = !envExactZero("GNSS_PPP_MADOCA_LOW_ELEV");
    overrides.madoca_galileo_gate =
        envExactOne("GNSS_PPP_MADOCA_GALILEO_GATE") ||
        overrides.madoca_early_window;
    overrides.madoca_bias_identity = envExactOne("GNSS_PPP_MADOCA_BIAS_IDENTITY");
    overrides.pb_add = envPresent("GNSS_PPP_PB_ADD");
    overrides.no_phase_bias = envPresent("GNSS_PPP_NO_PHASE_BIAS");
    overrides.l2_reset_fix = envFirstCharNotZero("GNSS_PPP_L2_RESET_FIX");
    overrides.gal_inav = envFirstCharNotZero("GNSS_PPP_GAL_INAV");
    overrides.ssr_discnt_slip = envPresent("GNSS_PPP_SSR_DISCNT_SLIP");
    overrides.no_solid_tide = envPresent("GNSS_PPP_NO_SOLID_TIDE");
    overrides.tide_itrs_sun_moon = envPresent("GNSS_PPP_TIDE_ITRS_SUN_MOON");
    overrides.pf_code_var_scale = envDoubleOr("GNSS_PPP_PF_CODE_VAR_SCALE", 9.0);
    overrides.pf_rx_antenna = envPresent("GNSS_PPP_PF_RX_ANTENNA");
    overrides.pf_wet_trop = envPresent("GNSS_PPP_PF_WET_TROP");

    overrides.res_dump = envPresent("GNSS_PPP_RES_DUMP");
    overrides.satpos_dump = envPresent("GNSS_PPP_SATPOS_DUMP");
    overrides.pfdump = envPresent("GNSS_PPP_PFDUMP");
    overrides.pb_apply_dump = envPresent("GNSS_PPP_PB_APPLY_DUMP");
    overrides.madoca_cbias_dump = envPresent("GNSS_PPP_MADOCA_CBIAS_DUMP");
    overrides.ar_dddump = envPresent("GNSS_PPP_AR_DDDUMP");
    overrides.debug = envPresent("GNSS_PPP_DEBUG");

    return overrides;
}

const PPPEnvOverrides& pppEnvOverrides() {
    static const PPPEnvOverrides overrides = PPPEnvOverrides::fromEnvironment();
    return overrides;
}

}  // namespace libgnss
