#pragma once

#include <string>

namespace libgnss {

struct PPPEnvOverrides {
    // GNSS_PPP_MADOCA_BIAS_SUBTRACT: subtract MADOCA SSR code/phase biases
    // instead of adding them. Default false.
    bool madoca_bias_subtract = false;
    // GNSS_PPP_MADOCA_ALLOW_PARTIAL_SSR: accept incomplete MADOCA SSR epochs
    // when set exactly to "1". Default false.
    bool madoca_allow_partial_ssr = false;
    // GNSS_PPP_REQUIRE_SSR_ORBIT: drop satellites missing SSR orbit
    // corrections when set to a value whose first char is not '0'. Default false.
    bool require_ssr_orbit = false;
    // GNSS_PPP_DISABLE_MADOCA_STATIC_ANCHOR: disable the MADOCA static anchor
    // blend when set exactly to "1". Default false.
    bool disable_madoca_static_anchor = false;
    // GNSS_PPP_STATIC_ANCHOR_BLEND: override static anchor blend fraction;
    // values outside [0,1] are ignored at use sites. Default -1.0 (inactive).
    double static_anchor_blend = -1.0;
    // GNSS_PPP_ESTIMATE_ISB: comma/space-style spec parsed by substring
    // checks for "gal", "qzs", "bds", "all", or exact "1". Defaults false.
    bool estimate_isb_all = false;
    bool estimate_isb_gal = false;
    bool estimate_isb_qzs = false;
    bool estimate_isb_bds = false;
    // GNSS_PPP_ISB_PROCESS_NOISE: ISB random-walk process noise in m^2/s.
    // Default 1e-6.
    double isb_process_noise = 1e-6;
    // GNSS_PPP_INIT_IONO_VAR: initial ionosphere variance override; used only
    // when > 0. Default 0.0 (inactive).
    double init_iono_var = 0.0;
    // GNSS_PPP_QZSS_CODE_ONLY: force QZSS code-only rows when set to a value
    // whose first char is not '0'. Default false.
    bool qzss_code_only = false;
    // GNSS_PPP_QZSS_PREFER_L1L: prefer QZSS L1L/L1X RINEX observations when
    // set to a value whose first char is not '0'. Default false.
    bool qzss_prefer_l1l = false;
    // GNSS_PPP_QZSS_PREFER_L1L presence bit so apps can preserve "env exists"
    // semantics when applying explicit reader defaults. Default false.
    bool qzss_prefer_l1l_present = false;
    // GNSS_PPP_QZSS_SSR_PRN_FIX / GNSS_PPP_DISABLE_QZSS_SSR_PRN_FIX: normalize
    // QZSS SSR PRNs unless the legacy var is exactly "0" or disable is exactly
    // "1". Default true.
    bool qzss_ssr_prn_fix = true;
    // GNSS_PPP_MADOCA_QZSS_CLOCK: allow default QZSS ISB clock in coherent
    // MADOCA unless set exactly to "0". Default true.
    bool madoca_qzss_clock = true;
    // GNSS_PPP_MADOCA_QZSS_PHASE: allow QZSS phase rows in MADOCA when set
    // exactly to "1". Default false.
    bool madoca_qzss_phase = false;
    // GNSS_PPP_MADOCA_GLONASS: include GLONASS in coherent MADOCA unless set
    // exactly to "0". Default true.
    bool madoca_glonass = true;
    // GNSS_PPP_PB_ADD: add non-MADOCA SSR phase biases instead of subtracting.
    // Default false.
    bool pb_add = false;
    // GNSS_PPP_NO_PHASE_BIAS: suppress per-frequency phase-bias application
    // when present. Default false.
    bool no_phase_bias = false;
    // GNSS_PPP_L2_RESET_FIX: reset L2 ambiguity on slip when set to a value
    // whose first char is not '0'. Default false.
    bool l2_reset_fix = false;
    // GNSS_PPP_GAL_INAV: apply Galileo I/NAV ephemeris selection when set to a
    // value whose first char is not '0'. Default false.
    bool gal_inav = false;
    // GNSS_PPP_SSR_DISCNT_SLIP: detect SSR phase-bias discontinuity slips when
    // present. Default false.
    bool ssr_discnt_slip = false;
    // GNSS_PPP_NO_SOLID_TIDE: skip solid-earth tides when present.
    // Default false.
    bool no_solid_tide = false;
    // GNSS_PPP_TIDE_ITRS_SUN_MOON: rotate sun/moon vectors to ITRS before IERS
    // solid tide when present. Default false.
    bool tide_itrs_sun_moon = false;
    // GNSS_PPP_PF_CODE_VAR_SCALE: per-frequency code variance scale.
    // Default 9.0.
    double pf_code_var_scale = 9.0;
    // GNSS_PPP_PF_RX_ANTENNA: enable per-frequency receiver antenna correction
    // when present. Default false.
    bool pf_rx_antenna = false;
    // GNSS_PPP_PF_WET_TROP: enable per-frequency wet trop split when present.
    // Default false.
    bool pf_wet_trop = false;

    // GNSS_PPP_RES_DUMP: PPP residual diagnostic dump. Default false.
    bool res_dump = false;
    // GNSS_PPP_SATPOS_DUMP: satellite position diagnostic dump. Default false.
    bool satpos_dump = false;
    // GNSS_PPP_PFDUMP: per-frequency AR diagnostic dump. Default false.
    bool pfdump = false;
    // GNSS_PPP_PB_APPLY_DUMP: phase-bias application diagnostic dump.
    // Default false.
    bool pb_apply_dump = false;
    // GNSS_PPP_MADOCA_CBIAS_DUMP: MADOCA code/phase-bias diagnostic dump.
    // Default false.
    bool madoca_cbias_dump = false;
    // GNSS_PPP_AR_DDDUMP: double-difference AR diagnostic dump. Default false.
    bool ar_dddump = false;
    // GNSS_PPP_DEBUG: general PPP debug logging. Default false.
    bool debug = false;

    static PPPEnvOverrides fromEnvironment();
};

const PPPEnvOverrides& pppEnvOverrides();

}  // namespace libgnss
