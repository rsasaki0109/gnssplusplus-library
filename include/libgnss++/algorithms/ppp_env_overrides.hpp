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
    // GNSS_PPP_MADOCA_SSR_REPLAY: select native MADOCA L6 SSR corrections from
    // per-PRN replay snapshots when set exactly to "1". Default false.
    bool madoca_ssr_replay = false;
    // GNSS_PPP_REQUIRE_SSR_ORBIT: drop satellites missing SSR orbit
    // corrections when set to a value whose first char is not '0'. Default false.
    bool require_ssr_orbit = false;
    // GNSS_PPP_DISABLE_MADOCA_STATIC_ANCHOR: disable the MADOCA static anchor
    // blend when set exactly to "1". Default false.
    bool disable_madoca_static_anchor = false;
    // GNSS_PPP_STATIC_ANCHOR_BLEND: override static anchor blend fraction;
    // values outside [0,1] are ignored at use sites. Default -1.0 (inactive).
    double static_anchor_blend = -1.0;
    // GNSS_PPP_MADOCA_EARLY_WINDOW: enable coherent MADOCA bridge-convergence
    // parity defaults unless set exactly to "0". Default true.
    bool madoca_early_window = true;
    // GNSS_PPP_MADOCA_SPIKE_GUARD: reject implausible converged static update
    // jumps in coherent MADOCA unless set exactly to "0". Default true.
    bool madoca_spike_guard = true;
    // GNSS_PPP_MADOCA_BOUNDARY_GUARD: reject smaller coherent-MADOCA
    // file-boundary update jumps unless set exactly to "0". Default true.
    bool madoca_boundary_guard = true;
    // GNSS_PPP_MADOCA_POSTFIT_COMMIT: preview RTKLIB/MADOCALIB-style
    // postfit validation with commit-on-success semantics for coherent MADOCA
    // when set exactly to "1". Default false.
    bool madoca_postfit_commit = false;
    // GNSS_PPP_MADOCA_POSTFIT_SHADOW: path for native MADOCA postfit
    // residual/statistic shadow dump. Empty disables it.
    std::string madoca_postfit_shadow_path;
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
    // GNSS_PPP_MADOCA_QZSS_L5: prefer QZSS L1/L5 over L1/L2 in coherent
    // MADOCA when set exactly to "1". Default false while measured.
    bool madoca_qzss_l5 = false;
    // GNSS_PPP_MADOCA_GLONASS: include GLONASS in coherent MADOCA unless set
    // exactly to "0". Default true.
    bool madoca_glonass = true;
    // GNSS_PPP_MADOCA_GLONASS_PHASE: allow GLONASS phase rows in MADOCA when
    // set exactly to "1". Default false.
    bool madoca_glonass_phase = false;
    // GNSS_PPP_MADOCA_LOW_ELEV: admit coherent MADOCA observations down to the
    // MADOCALIB 10 degree mask unless set exactly to "0". Default true.
    bool madoca_low_elev = true;
    // GNSS_PPP_MADOCA_GALILEO_GATE: apply MADOCALIB Galileo broadcast
    // ephemeris admission semantics in coherent MADOCA when set exactly to "1",
    // or when GNSS_PPP_MADOCA_EARLY_WINDOW is enabled. Default follows
    // GNSS_PPP_MADOCA_EARLY_WINDOW.
    bool madoca_galileo_gate = false;
    // GNSS_PPP_MADOCA_BIAS_IDENTITY: preserve MADOCA SSR code/phase-bias
    // signal identity instead of collapsed RTCM band ids when set exactly to
    // "1". Default false while the parity impact is measured.
    bool madoca_bias_identity = false;
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
    // GNSS_PPP_CLAS_NL_DEBUG: path for CLAS DD-WLNL narrow-lane component
    // diagnostic CSV. Empty disables it.
    std::string clas_nl_debug_path;
    // GNSS_PPP_CLAS_FIX_DEBUG: path for CLAS DD-WLNL fixed-position WLS
    // diagnostic CSV. Empty disables it.
    std::string clas_fix_debug_path;
    // GNSS_PPP_CLAS_FIX_REQUIRE_OSR: require a current CLAS OSR correction for
    // every DD-WLNL fixed-position observation when set exactly to "1".
    // Default false while measured.
    bool clas_fix_require_osr = false;
    // GNSS_PPP_CLAS_VERTICAL_FIX: make CLAS DD-WLNL NL prediction consistent
    // with full-CPC-corrected phase before fixed-position WLS. Default true;
    // set exactly "0" for bit-exact opt-out.
    bool clas_vertical_fix = true;
    // GNSS_PPP_CLAS_BRIDGE_CONVENTION: enable the coherent CLAS bridge
    // convention set unless set exactly to "0". Default true; the opt-out
    // restores current develop behavior for comparison.
    bool clas_bridge_convention = true;
    // GNSS_PPP_CLAS_NL_DATUM_FIX: CLAS DD-WLNL NL datum preview.
    // Under GNSS_PPP_CLAS_BRIDGE_CONVENTION default, datum reset is enabled
    // without CPC unification. With GNSS_PPP_CLAS_BRIDGE_CONVENTION=0, the
    // default restores the current develop datum reset + CPC unification
    // behavior. Values "datum" or "cpc" enable one component for diagnostics;
    // "0", "false", or "off" disable both.
    bool clas_nl_datum_reset = true;
    bool clas_nl_cpc_unified = true;
    // GNSS_PPP_CLAS_FIXED_STATE_OUTPUT: keep the accepted CLAS filter-state
    // position instead of replacing it with the standalone DD-WLNL fixed-WLS
    // position. Default true; set exactly "0" for bit-exact opt-out.
    bool clas_fixed_state_output = true;
    // GNSS_PPP_CLAS_RESAMB: publish the CLAS DD-WLNL fixed output from a
    // CLASLIB-style filter-state DD ambiguity conditioning copy when set
    // exactly to "1". Default false while measured.
    bool clas_resamb = false;
    // GNSS_PPP_CLAS_AMB_DATUM: align CLAS OSR carrier phase ambiguity states
    // with CLASLIB by subtracting the full CPC before ambiguity estimation.
    // Default follows the residual-phase-trop surface; explicit boolean values
    // still force diagnostics.
    bool clas_amb_datum = false;
    // GNSS_PPP_CLAS_AMB_DATUM_DUMP: path for native CLAS phase-row ambiguity
    // convention diagnostics. Empty disables it.
    std::string clas_amb_datum_dump_path;
    // GNSS_PPP_CLAS_AMB_DATUM_RESIDUAL_PHASE_TROP: with AMB_DATUM full-CPC
    // phase rows, keep only the residual trop model
    // (native mapped ZTD - CLAS CPC trop) and its Jacobian. Default true after
    // the S31 all-epoch CLASLIB-oracle gate; set exactly "0" for bit-exact
    // opt-out.
    bool clas_amb_datum_residual_phase_trop = true;
    // GNSS_PPP_CLAS_TROP_PRIOR_VARIANCE /
    // GNSS_PPP_CLAS_TROP_INITIAL_VARIANCE /
    // GNSS_PPP_CLAS_TROP_PROCESS_NOISE: CLAS trop state tuning overrides.
    // The residual-phase-trop surface defaults initial variance to 1e-4.
    // Values <= 0 leave PPPConfig defaults.
    double clas_trop_prior_variance = 0.0;
    double clas_trop_initial_variance = 1e-4;
    double clas_trop_process_noise = 0.0;
    // GNSS_PPP_CLAS_TX_TIME_SIGN_FIX: preview RTKLIB/CLASLIB transmit-time
    // iteration for CLAS SSR satellite positions. Default false; set exactly
    // "1" to enable. The default and exact "0" preserve legacy bit-exact
    // behavior because the S28 rollout rule rejected this term as default-on.
    bool clas_tx_time_sign_fix = false;
    // GNSS_PPP_CLAS_GEOM_DUMP: path for CLAS geometry-row forensic dump.
    // Empty disables it.
    std::string clas_geom_dump_path;
    // GNSS_PPP_CLAS_GEOM_DUMP_RX_XYZ: optional forced receiver ECEF
    // "x,y,z" for GNSS_PPP_CLAS_GEOM_DUMP.
    std::string clas_geom_dump_rx_xyz;
    // GNSS_PPP_CLAS_CODE_DUMP: path for CLAS float code-row diagnostics.
    // Empty disables it.
    std::string clas_code_dump_path;
    // GNSS_PPP_CLAS_FLOAT_DUMP: path for CLAS float-position diagnostics.
    // Empty disables it.
    std::string clas_float_dump_path;
    // GNSS_PPP_CLAS_CODE_SD: form CLAS OSR code rows as same-system
    // single differences when set exactly to "1". Default false while
    // float-datum parity is measured.
    bool clas_code_sd = false;
    // GNSS_PPP_CLAS_CODE_ROW_PARITY: comma/space separated preview gate for
    // CLASLIB-style float code rows. Tokens: full-prc, sd, bias, qzss, or
    // all/claslib. Default empty preserves legacy behavior.
    bool clas_code_row_full_prc = false;
    bool clas_code_row_sd = false;
    bool clas_code_row_bias_identity = false;
    bool clas_code_row_qzss = false;
    // GNSS_PPP_CLAS_QZSS_S_PRN_FIX: map compact CLAS legacy S120..S122
    // correction labels to QZSS J01..J03 when set exactly to "1".
    // Default false while row-set parity is measured.
    bool clas_qzss_s_prn_fix = false;
    // GNSS_PPP_DEBUG: general PPP debug logging. Default false.
    bool debug = false;

    static PPPEnvOverrides fromEnvironment();
};

const PPPEnvOverrides& pppEnvOverrides();

}  // namespace libgnss
