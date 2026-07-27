#pragma once

#include "../core/processor.hpp"
#include "../core/observation.hpp"
#include "../core/navigation.hpp"
#include "../core/solution.hpp"
#include "lambda.hpp"
#include "ppp_ar.hpp"
#include "ppp_clas_dd.hpp"
#include "ppp_shared.hpp"
#include "ppp_clas_sd.hpp"
#include "ppp_osr_types.hpp"
#include "../io/madoca_l6d.hpp"
#include "spp.hpp"
#include "../iers/eop_table.hpp"
#include "../iers/tides.hpp"
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <mutex>
#include <string>

namespace libgnss {

/// Per-SVN ANTEX entry used by PPPProcessor to translate satellite
/// centre-of-mass positions (as delivered by SP3) to the antenna phase
/// centre. Body frame uses ANTEX semantics: x = north-of-antenna,
/// y = east-of-antenna, z = up-of-antenna ≡ boresight toward Earth.
struct SatelliteAntexEntry {
    SatelliteId satellite;
    GNSSTime valid_from;
    GNSSTime valid_until;
    std::map<SignalType, Vector3d> body_offsets_m;
};

// Receiver antenna phase-centre variation (NOAZI elevation grid) for one
// signal, used by the est-stec per-frequency path.
struct ReceiverPcvGrid {
    double zen1_deg = 0.0;
    double zen2_deg = 90.0;
    double dzen_deg = 5.0;
    std::vector<double> noazi_m;  // PCV at each zenith node (metres)
};

struct MadocaL6dShadowStatus {
    bool snapshot_available = false;
    bool stale = false;
    double age_s = 0.0;
    int region_id = -1;
    int area_number = 0;
    int matched_satellites = 0;
};

struct PPPConvergenceTelemetry {
    size_t evaluated_epochs = 0;
    size_t insufficient_history_epochs = 0;
    size_t unstable_position_epochs = 0;
    size_t unstable_horizontal_epochs = 0;
    size_t unstable_vertical_epochs = 0;
    size_t window_epochs = 0;
    size_t required_window_epochs = 0;
    double max_position_deviation_m = 0.0;
    double max_horizontal_position_deviation_m = 0.0;
    double max_vertical_position_deviation_m = 0.0;
    double position_deviation_threshold_m = 0.0;
    double horizontal_position_deviation_threshold_m = 0.0;
    double vertical_position_deviation_threshold_m = 0.0;
    std::string policy = "legacy-3d";
    std::string gate_reason = "not_evaluated";
};

struct PPPConvergenceWindowMetrics {
    double max_ecef_3d_m = 0.0;
    double max_horizontal_m = 0.0;
    double max_vertical_m = 0.0;
};

struct PPPARStageTelemetry {
    size_t per_frequency_attempts = 0;
    size_t insufficient_satellite_epochs = 0;
    size_t no_wide_lane_epochs = 0;
    size_t wide_lane_only_epochs = 0;
    size_t n1_lambda_failure_epochs = 0;
    size_t n1_ratio_rejection_epochs = 0;
    size_t n1_fixed_epochs = 0;
    int last_satellite_candidates = 0;
    int last_wide_lane_pairs = 0;
    int last_n1_candidates = 0;
    double last_n1_ratio = 0.0;
    std::string last_stage = "not_attempted";
};

PPPConvergenceWindowMetrics evaluatePPPConvergenceWindow(
    const std::vector<Vector3d>& positions_ecef);

/**
 * @brief Precise Point Positioning (PPP) processor
 *
 * Implements PPP using precise orbits and clocks for decimeter to
 * centimeter level positioning without base stations.
 */
class PPPProcessor : public ProcessorBase {
public:
    using PPPConfig = ppp_shared::PPPConfig;
    using PPPState = ppp_shared::PPPState;
    using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;
    
    PPPProcessor();
    explicit PPPProcessor(const PPPConfig& ppp_config);
    ~PPPProcessor() override = default;
    
    // ProcessorBase interface
    bool initialize(const ProcessorConfig& config) override;
    PositionSolution processEpoch(const ObservationData& obs, const NavigationData& nav) override;
    ProcessorStats getStats() const override;
    void reset() override;
    
    /**
     * @brief Set PPP configuration
     */
    void setPPPConfig(const PPPConfig& config) {
        ppp_config_ = config;
        applyEnvironmentOverridesToPPPConfig();
    }
    
    /**
     * @brief Get PPP configuration
     */
    const PPPConfig& getPPPConfig() const { return ppp_config_; }
    
    /**
     * @brief Load precise products
     */
    bool loadPreciseProducts(const std::string& orbit_file, const std::string& clock_file);

    /**
     * @brief Load SSR orbit/clock corrections.
     */
    bool loadSSRProducts(const std::string& ssr_file);
    bool loadL6Products(const std::string& l6_file);

    /**
     * @brief Load native MADOCA L6E SSR products from one or more L6E channel
     * files (e.g. PRN 204 and 206). Decodes the MADOCA Compact SSR stream into
     * the SSR product time series (RAC orbit deltas, clock c0, code/phase
     * biases) for the standard PPP path. The GPS week comes from
     * @c l6_gps_week or the latest observation. Returns true if corrections
     * were produced. Distinct from loadL6Products (CLAS-oriented decoder).
     */
    bool loadMadocaL6Products(const std::vector<std::string>& l6_files);

    // Load receiver-specific L6D ionosphere snapshots for diagnostic shadow
    // lookup. This does not change PPP measurements or filter state.
    bool loadMadocaL6dProducts(const std::vector<std::string>& l6d_files,
                               const double reference_epoch[6],
                               const double receiver_ecef[3]);
    void setMadocaIonoProductsForShadow(const io::MadocaIonoProducts& products) {
        madoca_iono_products_ = products;
    }
    MadocaL6dShadowStatus inspectMadocaL6dShadow(
        const std::vector<SatelliteId>& satellites,
        const GNSSTime& time,
        double max_age_s = 300.0) const;
    const MadocaL6dShadowStatus& getLastMadocaL6dShadowStatus() const {
        return last_madoca_l6d_shadow_status_;
    }
    bool hasLoadedMadocaL6dProducts() const { return !madoca_iono_products_.empty(); }

    /**
     * @brief Load IONEX ionosphere products for future PPP hooks.
     */
    bool loadIONEXProducts(const std::string& ionex_file);

    /**
     * @brief Load DCB / Bias-SINEX products for future PPP hooks.
     */
    bool loadDCBProducts(const std::string& dcb_file);

    /**
     * @brief Load an IERS 20 C04 Earth Orientation Parameter file.
     *
     * The series is stored in-memory and consumed by per-epoch
     * earth-rotation / pole-tide / sub-daily-EOP code paths in
     * subsequent IERS phases. Phase D-0 wires the loader and
     * lookup; consumers arrive in Phase D-1+.
     */
    bool loadEopC04(const std::string& path);

    /**
     * @brief Whether an EOP series has been loaded successfully.
     */
    bool hasEopTable() const { return static_cast<bool>(eop_table_); }

    /**
     * @brief Look up EOP at a GNSS epoch.
     *
     * Returns the zero EarthOrientationParams (i.e. rigid-body
     * approximation) when no EOP series has been loaded. Throws
     * std::out_of_range when the requested epoch falls outside the
     * loaded series — callers should keep their EOP file current.
     */
    iers::EarthOrientationParams getEarthOrientationParams(const GNSSTime& time) const;

    /**
     * @brief Load RTCM SSR orbit/clock corrections and convert them to sampled PPP corrections.
     */
    bool loadRTCMSSRProducts(const std::string& rtcm_file,
                             const NavigationData& nav,
                             double sample_step_seconds = 1.0);

    /**
     * @brief Interpolate a loaded SSR correction for inspection/debugging.
     */
    bool interpolateLoadedSSRCorrection(const SatelliteId& sat,
                                        const GNSSTime& time,
                                        Vector3d& orbit_correction_ecef,
                                        double& clock_correction_m,
                                        double* ura_sigma_m = nullptr,
                                        std::map<uint8_t, double>* code_bias_m = nullptr,
                                        std::map<uint8_t, double>* phase_bias_m = nullptr,
                                        std::map<std::string, std::string>* atmos_tokens = nullptr) const;

    /**
     * @brief Get the number of tropospheric atmospheric corrections applied in the last epoch.
     */
    int getLastAppliedAtmosphericTroposphereCorrections() const {
        return last_applied_atmos_trop_corrections_;
    }

    /**
     * @brief Get the number of ionospheric atmospheric corrections applied in the last epoch.
     */
    int getLastAppliedAtmosphericIonosphereCorrections() const {
        return last_applied_atmos_iono_corrections_;
    }

    /**
     * @brief Get the summed absolute atmospheric troposphere correction applied in the last epoch.
     */
    double getLastAppliedAtmosphericTroposphereMeters() const {
        return last_applied_atmos_trop_m_;
    }

    /**
     * @brief Get the summed absolute atmospheric ionosphere correction applied in the last epoch.
     */
    double getLastAppliedAtmosphericIonosphereMeters() const {
        return last_applied_atmos_iono_m_;
    }

    int getLastAppliedIonexCorrections() const {
        return last_applied_ionex_corrections_;
    }

    int getLastAppliedDcbCorrections() const {
        return last_applied_dcb_corrections_;
    }

    double getLastAppliedIonexMeters() const {
        return last_applied_ionex_m_;
    }

    double getLastAppliedDcbMeters() const {
        return last_applied_dcb_m_;
    }
    
    /**
     * @brief Check convergence status
     */
    bool hasConverged() const { return converged_; }

    /**
     * @brief Check whether any SSR products are loaded.
     */
    bool hasLoadedSSRProducts() const { return ssr_products_loaded_; }
    bool hasLoadedIONEXProducts() const { return ionex_products_loaded_; }
    bool hasLoadedDCBProducts() const { return dcb_products_loaded_; }
    size_t getLoadedIONEXMapCount() const { return ionex_products_.tec_maps.size(); }
    size_t getLoadedDCBEntryCount() const { return dcb_products_.entries.size(); }
    bool getLastClasHybridFallbackUsed() const { return last_clas_hybrid_fallback_used_; }
    const std::string& getLastClasHybridFallbackReason() const {
        return last_clas_hybrid_fallback_reason_;
    }
    
    /**
     * @brief Get convergence time
     */
    double getConvergenceTime() const { return convergence_time_; }
    const PPPConvergenceTelemetry& getConvergenceTelemetry() const {
        return convergence_telemetry_;
    }
    const PPPARStageTelemetry& getARStageTelemetry() const {
        return ar_stage_telemetry_;
    }

private:
    void applyEnvironmentOverridesToPPPConfig();

    PPPConfig ppp_config_;
    PPPEnvOverrides env_overrides_;
    SPPProcessor spp_processor_;  ///< Fallback SPP processor
    PreciseProducts precise_products_;
    SSRProducts ssr_products_;
    IONEXProducts ionex_products_;
    DCBProducts dcb_products_;
    io::MadocaIonoProducts madoca_iono_products_;
    
    PPPState filter_state_;
    bool filter_initialized_ = false;
    
    // Convergence monitoring
    bool converged_ = false;
    double convergence_time_ = 0.0;
    GNSSTime convergence_start_time_;
    std::vector<Vector3d> recent_positions_;
    GNSSTime last_processed_time_;
    bool has_last_processed_time_ = false;
    int last_clas_atmos_network_id_ = -1;
    bool has_last_clas_atmos_network_id_ = false;
    bool precise_products_loaded_ = false;
    bool ssr_products_loaded_ = false;
    bool require_coherent_ssr_ = false;
    bool ionex_products_loaded_ = false;
    bool dcb_products_loaded_ = false;
    struct OceanLoadingCoefficients {
        std::array<double, 11> up_amplitudes_m{};
        std::array<double, 11> west_amplitudes_m{};
        std::array<double, 11> south_amplitudes_m{};
        std::array<double, 11> up_phases_deg{};
        std::array<double, 11> west_phases_deg{};
        std::array<double, 11> south_phases_deg{};
    };
    OceanLoadingCoefficients ocean_loading_coefficients_{};
    bool ocean_loading_loaded_ = false;
    std::unique_ptr<iers::EopTable> eop_table_;
    iers::AtmosphericTidalLoadingCoefficients atm_tidal_loading_coefficients_{};
    bool atm_tidal_loading_loaded_ = false;
    std::map<std::string, std::map<SignalType, Vector3d>> receiver_antex_offsets_;
    // Receiver PCV (NOAZI elevation grid) per antenna type and signal, used by
    // the est-stec per-frequency path.
    std::map<std::string, std::map<SignalType, ReceiverPcvGrid>> receiver_antex_pcv_;
    bool receiver_antex_loaded_ = false;
    std::vector<SatelliteAntexEntry> satellite_antex_offsets_;
    bool satellite_antex_loaded_ = false;
    Vector3d static_anchor_position_ = Vector3d::Zero();
    bool has_static_anchor_position_ = false;
    double last_ar_ratio_ = 0.0;
    int last_fixed_ambiguities_ = 0;
    bool last_ar_wide_lane_only_ = false;
    PPPConvergenceTelemetry convergence_telemetry_;
    PPPARStageTelemetry ar_stage_telemetry_;
    int last_applied_atmos_trop_corrections_ = 0;
    int last_applied_atmos_iono_corrections_ = 0;
    double last_applied_atmos_trop_m_ = 0.0;
    double last_applied_atmos_iono_m_ = 0.0;
    int last_applied_ionex_corrections_ = 0;
    int last_applied_dcb_corrections_ = 0;
    double last_applied_ionex_m_ = 0.0;
    double last_applied_dcb_m_ = 0.0;
    MadocaL6dShadowStatus last_madoca_l6d_shadow_status_;
    bool last_clas_hybrid_fallback_used_ = false;
    std::string last_clas_hybrid_fallback_reason_;
    
public:
    using ARState = PPPState;
    using ARAmbiguityInfo = PPPAmbiguityInfo;

private:
    std::map<SatelliteId, PPPAmbiguityInfo> ambiguity_states_;
    std::map<SatelliteId, CLASDispersionCompensationInfo> clas_dispersion_compensation_;
    std::map<SatelliteId, CLASSisContinuityInfo> clas_sis_continuity_;

    // Pre-anchor covariance saved for DD-AR position correction
    Eigen::MatrixXd pre_anchor_covariance_;
    bool had_fixed_last_epoch_ = false;  ///< AR succeeded in previous epoch
    int clas_kinematic_fix_candidate_streak_ = 0;
    int clas_kinematic_spp_divergence_count_ = 0;
    // Diagnostic-only (CLAS-HOLDCONT-DBG): consecutive epochs in a row with
    // clas_baseline_seed_maxdiff_this_epoch true, independent of/simpler than
    // clas_kinematic_spp_divergence_count_ above (different seed-selection
    // basis; see the maxdiff-only hold-continuation carve-out). Not read by
    // any gating logic.
    int clas_maxdiff_consecutive_streak_epochs_ = 0;
    // MRTKLIB clas.toml float_count=15: consecutive PPP-FLOAT epochs before
    // the kinematic CLAS filter is reinitialized from the current SPP seed.
    int clas_mrtklib_float_count_ = 0;
    int clas_seed_ar_quarantine_epochs_ = 0;
    // The CSV correction store is precomposed, while MRTKLIB's L6 decoder
    // starts empty and needs one 15 s CSSR cycle before CLAS OSR is usable.
    bool has_clas_mrtklib_stream_start_time_ = false;
    GNSSTime clas_mrtklib_stream_start_time_;
    // Per-epoch analogue of MRTKLIB ssat[].vsat[f]: ambiguity states whose
    // carrier row did not survive the final post-fit residual selection must
    // not enter ddmat/LAMBDA in this epoch.
    std::set<SatelliteId> clas_mrtklib_ar_rejected_ambiguities_;
    ppp_ar::WlnlHoldState clas_wlnl_hold_;
    // Current accepted ddmat/LAMBDA rows, carried to the post-fix chi-square
    // gate so holdamb parity does not reconstruct participants from stale
    // ambiguity flags.
    std::vector<ppp_ar::WlnlHoldConstraint> clas_wlnl_candidate_hold_constraints_;
    // MRTKLIB estpos() writes rtk->sol.rr before valsol(), but admitting that
    // validation-rejected candidate into native's filter regresses the
    // six-run gate. Keep the last admitted SPP seed separate from both filter
    // publications and the bounded rejected-candidate output tracker below.
    PositionSolution clas_last_valid_spp_seed_;
    bool has_clas_last_valid_spp_seed_ = false;
    // Output-only continuity tracker for validation-rejected SPP candidates.
    // These candidates never initialize or reset the PPP filter.
    Vector3d clas_last_rejected_output_position_ecef_ = Vector3d::Zero();
    GNSSTime clas_last_rejected_output_time_;
    bool has_clas_last_rejected_output_ = false;
    Vector3d last_published_solution_position_ecef_ = Vector3d::Zero();
    bool has_last_published_solution_position_ = false;
    // Anchors MRTKLIB rtk->sol.time's role in tt (=timediff) computation
    // (mrtk_rtkpos.c ~2395-2416): a failure before estpos() converges leaves
    // sol.time unchanged, while a later valsol() chi-square failure retains
    // the current time. In dynamics mode an early failure falls through
    // with tt==0 for that epoch and the accumulated gap is applied once a
    // usable current candidate returns. On the
    // kinematic CLAS parity path (ppp_clas_epoch.cpp), the redundancy/jump
    // guards on the SPP seed are this port's equivalent of pntpos()
    // rejecting a position. Native deliberately keeps its existing filter
    // lifecycle here: this anchor advances only for an admitted seed, while
    // rejected candidates can be used by the separate output-only tracker --
    // independent of last_processed_time_,
    // which keeps advancing every epoch for its other (non-parity) uses.
    GNSSTime clas_last_accepted_seed_time_;
    bool has_clas_last_accepted_seed_time_ = false;
    std::map<SatelliteId, double> windup_cache_;  ///< Phase wind-up cache for OSR
    std::map<SatelliteId, int> est_stec_outage_;   ///< Epochs since last seen (est-stec pruning)
    std::map<SatelliteId, std::map<uint8_t, int>> prev_phase_bias_discnt_;  ///< Last-seen SSR phase-bias discontinuity counters (GNSS_PPP_SSR_DISCNT_SLIP)
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> clas_phase_bias_repair_;
    ppp_clas_sd::SdFilterState clas_sd_state_;  ///< Clock-free SD filter
    ppp_clas_sd::DdAmbAccumulator clas_dd_accumulator_;  ///< Multi-epoch DD amb accumulator
    std::unique_ptr<ppp_clas_dd::DdFilterScaffold> clas_dd_filter_;
    PPPState last_clas_constrained_fixed_state_;
    bool last_clas_constrained_fixed_state_valid_ = false;
    // Tow of the most recent CLAS kinematic full-state reset (FLOATCNT wipe
    // in ppp_clas_epoch.cpp, or the MAXDIFFP/HOLDCONT-DBG-RESET teardown).
    // Feeds the measurement-only GNSS_PPP_CLAS_POST_RESET_RATIO_FLOOR
    // settle-window gate; unused when that override is at its default (0).
    GNSSTime clas_last_full_state_reset_time_;
    bool has_clas_last_full_state_reset_time_ = false;
    // Whether a maxdiff-only seed rejection has occurred since that reset,
    // while still inside its 1 s settle window. This measured signature
    // separates n2's destructive seed from its productive prompt re-fixes.
    bool clas_post_reset_saw_maxdiff_ = false;
    // Per-attempt signal from resolveAmbiguitiesWLNL(), then persistent
    // publication quarantine for the hold series seeded by that attempt.
    bool last_clas_post_reset_floor_failed_ = false;
    bool clas_post_reset_fix_quarantine_ = false;
    int last_obs_gps_week_ = 0;  ///< GPS week from latest observation (for L6 decode)

    // Statistics
    mutable std::mutex stats_mutex_;
    size_t total_epochs_processed_ = 0;
    size_t converged_solutions_ = 0;
    double total_convergence_time_ = 0.0;
    
    /**
     * @brief Initialize Kalman filter
     */
    bool initializeFilter(const ObservationData& obs,
                          const NavigationData& nav,
                          const PositionSolution* seed_solution = nullptr);

    PositionSolution processEpochStandard(
        const ObservationData& obs,
        const NavigationData& nav,
        const char* clas_hybrid_fallback_reason = nullptr);
    
    /**
     * @brief Predict filter state
     */
    void predictState(double dt, const PositionSolution* seed_solution = nullptr);

    struct IonosphereFreeObs;
    
    /**
     * @brief Update filter with measurements
     */
    bool updateFilter(const ObservationData& obs, const NavigationData& nav,
                      double dt, bool apply_iono_prediction = true);

    void updateMadocaPerFrequencyIonospherePrediction(
        const std::vector<IonosphereFreeObs>& observations, double dt);

    bool hasEnoughCoherentSsrObservations(const ObservationData& obs,
                                          const NavigationData& nav);
    
    /**
     * @brief Form ionosphere-free combinations
     */
    struct IonosphereFreeObs {
        struct AdditionalFrequencyObs {
            int ordinal = 2;
            SignalType signal = SignalType::SIGNAL_TYPE_COUNT;
            std::string observation_type;
            std::string pseudorange_observation_type;
            std::string carrier_phase_observation_type;
            int pseudorange_rtklib_code = 0;
            int carrier_phase_rtklib_code = 0;
            double pseudorange = 0.0;
            double carrier_phase = 0.0;
            double wavelength = 0.0;
            double frequency = 0.0;
            double variance_pr = 0.0;
            double variance_cp = 0.0;
            double code_bias_m = 0.0;
            double phase_bias_m = 0.0;
            double rx_ant_corr_m = 0.0;
            bool has_carrier_phase = false;
            bool valid = false;
        };
        SatelliteId satellite;
        SignalType primary_signal = SignalType::SIGNAL_TYPE_COUNT;
        SignalType secondary_signal = SignalType::SIGNAL_TYPE_COUNT;
        std::string primary_observation_type;
        std::string secondary_observation_type;
        std::string primary_pseudorange_observation_type;
        std::string primary_carrier_phase_observation_type;
        std::string secondary_pseudorange_observation_type;
        std::string secondary_carrier_phase_observation_type;
        int primary_pseudorange_rtklib_code = 0;
        int primary_carrier_phase_rtklib_code = 0;
        int secondary_pseudorange_rtklib_code = 0;
        int secondary_carrier_phase_rtklib_code = 0;
        double primary_code_bias_coeff = 1.0;
        double secondary_code_bias_coeff = 0.0;
        double primary_frequency_hz = 0.0;
        double secondary_frequency_hz = 0.0;
        int glonass_frequency_channel = -99;
        double pseudorange_if = 0.0;
        double carrier_phase_if = 0.0;
        double pseudorange_code_bias_m = 0.0;
        double carrier_phase_bias_m = 0.0;
        double variance_pr = 0.0;
        double variance_cp = 0.0;
        Vector3d receiver_position = Vector3d::Zero();
        Vector3d satellite_position = Vector3d::Zero();
        Vector3d satellite_velocity = Vector3d::Zero();
        double satellite_clock_bias = 0.0;
        double satellite_clock_drift = 0.0;
        double elevation = 0.0;
        double azimuth = 0.0;
        double trop_mapping = 0.0;
        double modeled_trop_delay_m = 0.0;
        double antenna_pco_m = 0.0;
        // Per-frequency receiver antenna corrections for the est-stec path.
        // The shared receiver_position already carries the L1 PCO via geodist;
        // these add the L1 PCV and the (L2-L1) PCO delta + L2 PCV so each
        // frequency's modeled range uses its own antenna phase centre. Zero in
        // the IFLC path (IF-combined PCO is applied via the position shift).
        double rx_ant_corr_l1_m = 0.0;
        double rx_ant_corr_l2_m = 0.0;
        double ambiguity_scale_m = 0.0;
        double atmospheric_trop_correction_m = 0.0;
        double atmospheric_iono_correction_m = 0.0;
        double ssr_orbit_age_s = -1.0;
        double ssr_clock_age_s = -1.0;
        int ssr_orbit_iod = -1;
        int ssr_clock_iod = -1;
        int ssr_orbit_iode = -1;
        std::map<std::string, std::string> ssr_atmos_tokens;
        bool has_carrier_phase = false;
        bool valid = false;

        // --- Per-frequency uncombined (est-stec) fields ---
        // Populated only when use_ionosphere_free == false. Each carries the
        // raw L1/L2 observable in meters with per-frequency SSR/DCB biases
        // already removed (NOT an ionosphere-free combination). The IFLC path
        // leaves these default-zeroed so the state layout is unchanged.
        double pseudorange_l1 = 0.0;
        double pseudorange_l2 = 0.0;
        double carrier_phase_l1 = 0.0;  // meters
        double carrier_phase_l2 = 0.0;  // meters
        double wavelength_l1 = 0.0;
        double wavelength_l2 = 0.0;
        double freq_l1 = 0.0;
        double freq_l2 = 0.0;
        double variance_pr_l1 = 0.0;
        double variance_pr_l2 = 0.0;
        double variance_cp_l1 = 0.0;
        double variance_cp_l2 = 0.0;
        double code_bias_l1_m = 0.0;
        double code_bias_l2_m = 0.0;
        double phase_bias_l1_m = 0.0;
        double phase_bias_l2_m = 0.0;
        // Geometry-free ionosphere seed (L1-equivalent meters):
        // (P1 - P2) / (1 - (f1/f2)^2).
        double iono_init_m = 0.0;
        bool has_iono_init = false;
        bool has_l2 = false;
        bool has_carrier_phase_l2 = false;
        std::vector<AdditionalFrequencyObs> additional_frequencies;
        // Wet mapping function and a priori zenith hydrostatic delay for the
        // RTKLIB-style split trop model (est-stec): trop = m_h*ZHD + m_w*(ZTD-ZHD).
        double trop_mapping_wet = 0.0;
        double trop_zhd_m = 0.0;
    };
    
    std::vector<IonosphereFreeObs> formIonosphereFree(const ObservationData& obs,
                                                    const NavigationData& nav);
    
    /**
     * @brief Apply precise corrections
     */
    void applyPreciseCorrections(std::vector<IonosphereFreeObs>& observations,
                               const NavigationData& nav,
                               const GNSSTime& time);

    Vector3d calculateReceiverAntennaOffsetEcef(const Vector3d& receiver_marker_position,
                                                const IonosphereFreeObs& observation) const;

    // Receiver antenna PCV range correction (metres) for a signal at the given
    // elevation, interpolated from the NOAZI grid. Returns 0 if no grid is
    // available. Sign matches RTKLIB antmodel(): the value is added to the
    // modeled range (equivalently subtracted from the observable).
    double receiverAntennaPcvMeters(SignalType signal, double elevation_rad) const;
    double clasReceiverAntennaRangeCorrectionMeters(
        SignalType signal,
        double azimuth_rad,
        double elevation_rad) const;
    void materializeClasReceiverAntennaCorrections(
        std::vector<OSRCorrection>& osr_corrections) const;

    /**
     * @brief Detect cycle slips
     */
    void detectCycleSlips(const ObservationData& obs, const NavigationData& nav);

    /**
     * @brief Ensure ambiguity states exist for valid carrier-phase observations.
     */
    void ensureAmbiguityStates(const std::vector<IonosphereFreeObs>& observations);

    /**
     * @brief Get ambiguity state index for a satellite, or -1 if absent.
     */
    int ambiguityStateIndex(const SatelliteId& satellite) const;

    /**
     * @brief Get or create ambiguity state for a satellite observation.
     */
    int getOrCreateAmbiguityState(const IonosphereFreeObs& observation);

    /**
     * @brief Initialize or reinitialize a phase-bias ambiguity state.
     */
    void initializeAmbiguityState(const IonosphereFreeObs& observation, int state_index);

    /**
     * @brief Get or create the per-frequency L2 ambiguity state (est-stec only).
     */
    int getOrCreateAmbiguityStateL2(const IonosphereFreeObs& observation);
    int getOrCreateAdditionalAmbiguityState(
        const IonosphereFreeObs& observation,
        const IonosphereFreeObs::AdditionalFrequencyObs& frequency);
    int getOrCreateReceiverFrequencyBiasState(const SatelliteId& satellite,
                                              int frequency_ordinal);

    /**
     * @brief Get or create the per-satellite ionosphere (STEC) state, seeded
     * from the geometry-free code combination (est-stec only). Lazily appended
     * so satellites that rise mid-arc still receive a state.
     */
    int getOrCreateIonosphereState(const IonosphereFreeObs& observation);
    
    /**
     * @brief Resolve PPP ambiguities
     */
    bool resolveAmbiguities(const ObservationData& obs, const NavigationData& nav);
    bool resolveAmbiguitiesWLNL(const ObservationData& obs, const NavigationData& nav);
    // Decoupled ionosphere-free PPP-AR (CNES/Laurichesse-style): recovers the
    // narrow-lane integer N1 from the IFLC ambiguity state plus the
    // Melbourne-Wubbena wide-lane integer, then applies the fixed N1 double
    // differences as a Kalman pseudo-measurement on the IFLC ambiguity states.
    bool resolveAmbiguitiesDecoupledIf(const ObservationData& obs, const NavigationData& nav);
    // Per-frequency (est-stec) PPP-AR mirroring the MADOCALIB oracle: forms the
    // wide-lane integer from the separate L1/L2 ambiguity states, then resolves
    // the narrow-lane N1 by LAMBDA on the L1 ambiguity double differences,
    // applying both as tight Kalman pseudo-measurements.
    bool resolveAmbiguitiesPerFreq(const ObservationData& obs, const NavigationData& nav);
    // Remove per-frequency ambiguity/ionosphere states for satellites not seen
    // for several epochs (est-stec only), compacting the filter state so its
    // dimension stays bounded by the currently-tracked satellites.
    void pruneStaleEstStecStates(const std::set<SatelliteId>& observed);
    std::map<SatelliteId, OSRCorrection> computeWlnlOsrCorrections(
        const ObservationData& obs,
        const NavigationData& nav,
        const Vector3d& receiver_position,
        double clock_bias_m,
        double trop_zenith,
        bool include_single_frequency = false) const;
    bool buildWlnlNlInfoForSatellite(
        const ObservationData& obs,
        const NavigationData& nav,
        const Vector3d& receiver_position,
        double clock_bias_m,
        double trop_zenith,
        const std::map<SatelliteId, OSRCorrection>& osr_by_sat,
        const SatelliteId& satellite,
        ppp_ar::WlnlNlInfo& info) const;
    bool buildFixedNlObservationForSatellite(
        const ObservationData& obs,
        const NavigationData& nav,
        const std::map<SatelliteId, OSRCorrection>& osr_by_sat,
        const SatelliteId& satellite,
        const PPPAmbiguityInfo& ambiguity,
        ppp_ar::FixedNlObservation& fixed_observation) const;
    bool buildFixedCarrierObservation(
        const IonosphereFreeObs& observation,
        ppp_ar::FixedCarrierObservation& fixed_observation) const;

    /// CLAS-PPP mode: process epoch using OSR-corrected observations
    PositionSolution processEpochCLAS(const ObservationData& obs, const NavigationData& nav);
    bool solveFixedPosition(const ObservationData& obs, const NavigationData& nav,
                            Vector3d& fixed_position);
    
    /**
     * @brief Calculate tropospheric delay
     */
    double calculateTroposphericDelay(const Vector3d& receiver_pos,
                                      const Vector3d& satellite_pos,
                                      const GNSSTime& time,
                                      double zenith_delay) const;
    
    /**
     * @brief Calculate tropospheric mapping function
     */
    double calculateMappingFunction(const Vector3d& receiver_pos,
                                    double elevation,
                                    const GNSSTime& time) const;
    
    /**
     * @brief Apply geophysical corrections
     */
    Vector3d applyGeophysicalCorrections(const Vector3d& position,
                                       const GNSSTime& time) const;
    
    /**
     * @brief Calculate solid Earth tides
     */
    Vector3d calculateSolidEarthTides(const Vector3d& position,
                                    const GNSSTime& time) const;
    
    /**
     * @brief Calculate ocean loading
     */
    Vector3d calculateOceanLoading(const Vector3d& position,
                                 const GNSSTime& time) const;

    /**
     * @brief Calculate IERS pole-tide station displacement (Phase D-1).
     *
     * Returns Vector3d::Zero() when no EOP table is loaded, since the
     * pole tide depends on instantaneous polar motion (xp, yp). Caller
     * is responsible for gating on `ppp_config_.use_iers_pole_tide`.
     */
    Vector3d calculatePoleTide(const Vector3d& position,
                               const GNSSTime& time) const;

    /**
     * @brief Calculate IERS atmospheric tidal loading (Phase D-3).
     *
     * Returns Vector3d::Zero() when no atmospheric tidal-loading
     * coefficient file is loaded. Caller is responsible for gating
     * on `ppp_config_.use_iers_atm_tidal_loading`.
     */
    Vector3d calculateAtmosphericTidalLoading(const Vector3d& position,
                                              const GNSSTime& time) const;

    /**
     * @brief Load per-site S1+S2 atmospheric tidal loading coefficients.
     *
     * File format: comment lines starting with `$$`; data block of
     * one site:
     *   <station_name>
     *   S1 <radial_amp_m> <west_amp_m> <south_amp_m>
     *      <radial_phase_deg> <west_phase_deg> <south_phase_deg>
     *   S2 <radial_amp_m> <west_amp_m> <south_amp_m>
     *      <radial_phase_deg> <west_phase_deg> <south_phase_deg>
     */
    bool loadAtmosphericTidalLoading(const std::string& path);
    
    /**
     * @brief Form measurement equations
     */
    struct MeasurementEquation {
        MatrixXd design_matrix;
        VectorXd observations;
        VectorXd predicted;
        MatrixXd weight_matrix;
        VectorXd residuals;
        std::vector<SatelliteId> row_satellites;  ///< Satellite for each row
        std::vector<bool> row_is_phase;           ///< true=carrier phase, false=code
        std::vector<double> row_elevations;        ///< Elevation angle in radians
        std::vector<double> row_azimuths;          ///< Azimuth angle in radians
        std::vector<SignalType> row_signals;       ///< Signal identity for each row
        std::vector<std::string> row_signal_bands; ///< IF/L1/L2 label for each row
        std::vector<std::string> row_rinex_codes;
        std::vector<int> row_rtklib_codes;
        std::vector<std::string> row_signal_families;
        std::vector<int> row_glonass_frequency_channels;
        std::vector<double> row_primary_frequencies_hz;
        std::vector<double> row_secondary_frequencies_hz;
        std::vector<double> row_primary_if_coefficients;
        std::vector<double> row_secondary_if_coefficients;
        std::vector<double> row_ssr_orbit_ages_s;
        std::vector<double> row_ssr_clock_ages_s;
        std::vector<int> row_ssr_orbit_iods;
        std::vector<int> row_ssr_clock_iods;
        std::vector<int> row_ssr_orbit_iodes;
    };
    
    MeasurementEquation formMeasurementEquations(
        const std::vector<IonosphereFreeObs>& observations,
        const NavigationData& nav,
        const GNSSTime& time,
        bool apply_outlier_detection = true);
    
    /**
     * @brief Check convergence
     */
    void checkConvergence(const GNSSTime& current_time);
    
    /**
     * @brief Generate PPP solution
     */
    PositionSolution generateSolution(const GNSSTime& time,
                                    const std::vector<IonosphereFreeObs>& observations);
    
    /**
     * @brief Calculate position accuracy
     */
    Vector3d calculatePositionAccuracy() const;
    
    /**
     * @brief Update ambiguity states
     */
    void updateAmbiguityStates(const ObservationData& obs);
    
    /**
     * @brief Reset ambiguity for satellite
     */
    void resetAmbiguity(const SatelliteId& satellite, SignalType signal);

    void reinitializeVectorState(int start_index, const Vector3d& value, double variance);
    void reinitializeScalarState(int index, double value, double variance);
    bool useLowDynamicsBroadcastSeedAssist() const;
    void recoverLowDynamicsBroadcastState(const ObservationData& obs,
                                          const PositionSolution* seed_solution);
    int receiverClockStateIndex(const SatelliteId& satellite) const;
    double receiverClockBiasMeters(const SatelliteId& satellite) const;
    double measurementVariance(const IonosphereFreeObs& observation, bool carrier_phase) const;
    
    /**
     * @brief Calculate measurement residuals
     */
        VectorXd calculateResiduals(const std::vector<IonosphereFreeObs>& observations,
                                  const NavigationData& nav,
                                  const GNSSTime& time) const;

    /**
     * @brief Clamp the unused velocity states when PPP runs in static or low-dynamics mode.
     */
    void constrainStaticVelocityStates();

    /**
     * @brief Keep static or low-dynamics PPP anchored to its initial SPP seed.
     */
    void constrainStaticAnchorPosition();

    bool solveFixedCarrierPhasePosition(const std::vector<IonosphereFreeObs>& observations,
                                        Vector3d& fixed_position) const;

    static double modeledZenithTroposphereDelayMeters(
        const Vector3d& receiver_position, const GNSSTime& time);

    void updateStatistics(bool converged) const;
};

/**
 * @brief PPP utility functions
 */
namespace ppp_utils {
    
    /**
     * @brief Calculate ionosphere-free combination coefficients
     */
    std::pair<double, double> getIonosphereFreeCoefficients(double f1, double f2);
    
    /**
     * @brief Calculate Melbourne-Wubbena combination
     */
    double calculateMelbourneWubbena(double l1_phase, double l2_phase,
                                   double p1_range, double p2_range,
                                   double f1, double f2);
    
    /**
     * @brief Calculate geometry-free combination
     */
    double calculateGeometryFree(double l1_phase, double l2_phase);
    
    /**
     * @brief Interpolate precise orbits
     */
    bool interpolatePreciseOrbit(const std::vector<PreciseOrbitClock>& orbit_data,
                               const GNSSTime& time,
                               Vector3d& position,
                               Vector3d& velocity);
    
    /**
     * @brief Interpolate precise clocks
     */
    bool interpolatePreciseClock(const std::vector<PreciseOrbitClock>& clock_data,
                               const GNSSTime& time,
                               double& clock_bias,
                               double& clock_drift);
    
    /**
     * @brief Calculate satellite antenna phase center offset
     */
    Vector3d calculateSatelliteAntennaPCO(const SatelliteId& satellite,
                                        const Vector3d& satellite_pos,
                                        const Vector3d& sun_pos);
    
    /**
     * @brief Calculate receiver antenna phase center offset
     */
    Vector3d calculateReceiverAntennaPCO(const Vector3d& receiver_pos,
                                       const Vector3d& satellite_pos,
                                       const std::string& antenna_type);
    
    /**
     * @brief Calculate phase windup correction (Wu et al. 1993).
     *
     * The satellite body frame is constructed from the sun direction so the
     * X-axis points away from the spacecraft–Sun plane normal — the geometry
     * used by IGS/RTKLIB. `sun_position_ecef` may be Vector3d::Zero() to fall
     * back to a line-of-sight approximation, but precise PPP should always
     * pass a real sun position.
     */
    double calculatePhaseWindup(const Vector3d& receiver_pos,
                              const Vector3d& satellite_pos,
                              const Vector3d& sun_position_ecef,
                              double previous_windup);
}

} // namespace libgnss
