#include <libgnss++/algorithms/ppp_env_overrides.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <initializer_list>
#include <sstream>
#include <string>

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

bool envBoolOr(const char* name, bool fallback) {
    const char* value = envValue(name);
    if (value == nullptr) {
        return fallback;
    }
    std::string text(value);
    std::transform(
        text.begin(),
        text.end(),
        text.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (text == "1" || text == "true" || text == "on" || text == "yes") {
        return true;
    }
    if (text == "0" || text == "false" || text == "off" || text == "no") {
        return false;
    }
    return fallback;
}

double envDoubleOr(const char* name, double fallback) {
    const char* value = envValue(name);
    return value != nullptr ? std::atof(value) : fallback;
}

std::string envStringOrEmpty(const char* name) {
    const char* value = envValue(name);
    return value != nullptr ? std::string(value) : std::string();
}

std::string normalizedTokenSpec(const std::string& value) {
    std::string normalized = value;
    std::transform(
        normalized.begin(),
        normalized.end(),
        normalized.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    for (char& ch : normalized) {
        if (ch == '_') {
            ch = '-';
        } else if (ch == ',' || ch == ';' || ch == ':' || ch == '+' || ch == '|') {
            ch = ' ';
        }
    }
    return normalized;
}

bool tokenSpecContains(const std::string& normalized_spec, const std::string& token) {
    std::string spaced = normalized_spec;
    for (char& ch : spaced) {
        if (ch == '-') {
            continue;
        }
        if (!std::isalnum(static_cast<unsigned char>(ch))) {
            ch = ' ';
        }
    }
    std::istringstream input(spaced);
    std::string candidate;
    while (input >> candidate) {
        if (candidate == token) {
            return true;
        }
    }
    return false;
}

bool tokenSpecHasAny(const std::string& normalized_spec,
                     std::initializer_list<const char*> tokens) {
    for (const char* token : tokens) {
        if (tokenSpecContains(normalized_spec, token)) {
            return true;
        }
    }
    return false;
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
    overrides.clas_nl_debug_path =
        envStringOrEmpty("GNSS_PPP_CLAS_NL_DEBUG");
    overrides.clas_fix_debug_path =
        envStringOrEmpty("GNSS_PPP_CLAS_FIX_DEBUG");
    overrides.clas_fix_require_osr =
        envExactOne("GNSS_PPP_CLAS_FIX_REQUIRE_OSR");
    overrides.clas_vertical_fix =
        !envExactZero("GNSS_PPP_CLAS_VERTICAL_FIX");
    overrides.clas_bridge_convention =
        !envExactZero("GNSS_PPP_CLAS_BRIDGE_CONVENTION");
    overrides.clas_fixed_state_output =
        !envExactZero("GNSS_PPP_CLAS_FIXED_STATE_OUTPUT");
    overrides.clas_resamb =
        envExactOne("GNSS_PPP_CLAS_RESAMB");
    overrides.clas_dd_filter =
        envExactOne("GNSS_PPP_CLAS_DD_FILTER");
    overrides.clas_amb_datum_residual_phase_trop =
        !envExactZero("GNSS_PPP_CLAS_AMB_DATUM_RESIDUAL_PHASE_TROP");
    overrides.clas_amb_datum =
        envBoolOr(
            "GNSS_PPP_CLAS_AMB_DATUM",
            overrides.clas_amb_datum_residual_phase_trop);
    overrides.clas_amb_datum_dump_path =
        envStringOrEmpty("GNSS_PPP_CLAS_AMB_DATUM_DUMP");
    overrides.clas_trop_prior_variance =
        envDoubleOr("GNSS_PPP_CLAS_TROP_PRIOR_VARIANCE", 0.0);
    overrides.clas_trop_initial_variance =
        envDoubleOr(
            "GNSS_PPP_CLAS_TROP_INITIAL_VARIANCE",
            overrides.clas_amb_datum_residual_phase_trop ? 1e-4 : 0.0);
    overrides.clas_trop_process_noise =
        envDoubleOr("GNSS_PPP_CLAS_TROP_PROCESS_NOISE", 0.0);
    overrides.clas_tx_time_sign_fix =
        envExactOne("GNSS_PPP_CLAS_TX_TIME_SIGN_FIX");
    overrides.clas_geom_dump_path =
        envStringOrEmpty("GNSS_PPP_CLAS_GEOM_DUMP");
    overrides.clas_geom_dump_rx_xyz =
        envStringOrEmpty("GNSS_PPP_CLAS_GEOM_DUMP_RX_XYZ");
    overrides.clas_code_dump_path =
        envStringOrEmpty("GNSS_PPP_CLAS_CODE_DUMP");
    overrides.clas_float_dump_path =
        envStringOrEmpty("GNSS_PPP_CLAS_FLOAT_DUMP");
    const std::string clas_code_row_parity =
        normalizedTokenSpec(envStringOrEmpty("GNSS_PPP_CLAS_CODE_ROW_PARITY"));
    const bool clas_code_row_all =
        tokenSpecHasAny(
            clas_code_row_parity,
            {"1", "true", "on", "yes", "all", "both", "claslib", "parity"});
    const bool clas_code_row_off =
        tokenSpecHasAny(
            clas_code_row_parity,
            {"0", "false", "off", "no", "none"});
    if (!clas_code_row_off) {
        overrides.clas_code_row_full_prc =
            clas_code_row_all ||
            tokenSpecHasAny(clas_code_row_parity, {"full-prc", "code-prc", "prc"});
        overrides.clas_code_row_sd =
            clas_code_row_all ||
            tokenSpecHasAny(clas_code_row_parity, {"sd", "code-sd", "single-diff"});
        overrides.clas_code_row_bias_identity =
            clas_code_row_all ||
            tokenSpecHasAny(
                clas_code_row_parity,
                {"bias", "bias-id", "bias-identity", "l2-bias", "l2"});
        overrides.clas_code_row_qzss =
            clas_code_row_all ||
            tokenSpecHasAny(clas_code_row_parity, {"qzss", "qzs", "j01", "qzss-prn"});
    }
    overrides.clas_atmos_grid_matrix =
        envExactOne("GNSS_PPP_CLAS_ATMOS_GRID_MATRIX");
    overrides.clas_atmos_lifecycle =
        envExactOne("GNSS_PPP_CLAS_ATMOS_LIFECYCLE");
    overrides.clas_stec_constraint =
        envExactOne("GNSS_PPP_CLAS_STEC_CONSTRAINT");
    overrides.clas_code_sd =
        envExactOne("GNSS_PPP_CLAS_CODE_SD") ||
        overrides.clas_code_row_sd;
    overrides.clas_qzss_s_prn_fix =
        envExactOne("GNSS_PPP_CLAS_QZSS_S_PRN_FIX") ||
        overrides.clas_code_row_qzss;
    const std::string clas_nl_datum_fix =
        envStringOrEmpty("GNSS_PPP_CLAS_NL_DATUM_FIX");
    if (clas_nl_datum_fix.empty()) {
        overrides.clas_nl_datum_reset = true;
        overrides.clas_nl_cpc_unified = !overrides.clas_bridge_convention;
    }
    std::string clas_nl_datum_fix_lower = clas_nl_datum_fix;
    std::transform(
        clas_nl_datum_fix_lower.begin(),
        clas_nl_datum_fix_lower.end(),
        clas_nl_datum_fix_lower.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (clas_nl_datum_fix_lower == "0" ||
        clas_nl_datum_fix_lower == "false" ||
        clas_nl_datum_fix_lower == "off" ||
        clas_nl_datum_fix_lower == "none") {
        overrides.clas_nl_datum_reset = false;
        overrides.clas_nl_cpc_unified = false;
    } else if (clas_nl_datum_fix_lower == "1" ||
        clas_nl_datum_fix_lower == "both" ||
        clas_nl_datum_fix_lower == "all" ||
        clas_nl_datum_fix_lower == "true" ||
        clas_nl_datum_fix_lower == "on") {
        overrides.clas_nl_datum_reset = true;
        overrides.clas_nl_cpc_unified = true;
    } else if (clas_nl_datum_fix_lower == "datum" ||
               clas_nl_datum_fix_lower == "reset") {
        overrides.clas_nl_datum_reset = true;
        overrides.clas_nl_cpc_unified = false;
    } else if (clas_nl_datum_fix_lower == "cpc") {
        overrides.clas_nl_datum_reset = false;
        overrides.clas_nl_cpc_unified = true;
    }
    overrides.debug = envPresent("GNSS_PPP_DEBUG");

    return overrides;
}

const PPPEnvOverrides& pppEnvOverrides() {
    static const PPPEnvOverrides overrides = PPPEnvOverrides::fromEnvironment();
    return overrides;
}

}  // namespace libgnss
