#pragma once

#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

#include <array>
#include <cstdlib>
#include <map>
#include <string>

namespace libgnss::ppp_shared {

/// Check if PPP debug output is enabled via GNSS_PPP_DEBUG environment variable.
inline bool pppDebugEnabled() {
    return std::getenv("GNSS_PPP_DEBUG") != nullptr;
}

struct PPPConfig {
    enum class ClasAtmosSelectionPolicy {
        GRID_FIRST,
        GRID_GUARDED,
        BALANCED,
        FRESHNESS_FIRST,
    };

    enum class ClasEpochPolicy {
        STRICT_OSR,
        HYBRID_STANDARD_PPP_FALLBACK,
    };

    enum class ClasCorrectionApplicationPolicy {
        FULL_OSR,
        ORBIT_CLOCK_BIAS,
        ORBIT_CLOCK_ONLY,
    };

    enum class ClasPhaseContinuityPolicy {
        FULL_REPAIR,
        SIS_CONTINUITY_ONLY,
        REPAIR_ONLY,
        RAW_PHASE_BIAS,
        NO_PHASE_BIAS,
    };

    enum class ClasPhaseBiasValuePolicy {
        FULL,
        PHASE_BIAS_ONLY,
        COMPENSATION_ONLY,
    };

    enum class ClasPhaseBiasReferenceTimePolicy {
        PHASE_BIAS_REFERENCE,
        CLOCK_REFERENCE,
        OBSERVATION_EPOCH,
    };

    enum class ClasSsrTimingPolicy {
        LAG_TOLERANT,
        CLOCK_BOUND_PHASE_BIAS,
        CLOCK_BOUND_ATMOS_AND_PHASE_BIAS,
    };

    enum class ClasExpandedValueConstructionPolicy {
        FULL_COMPOSED,
        RESIDUAL_ONLY,
        POLYNOMIAL_ONLY,
    };

    enum class ClasExpandedResidualSamplingPolicy {
        INDEXED_OR_MEAN,
        INDEXED_ONLY,
        MEAN_ONLY,
    };

    enum class ClasSubtype12ValueConstructionPolicy {
        FULL,
        PLANAR,
        OFFSET_ONLY,
    };

    // Precise products
    bool use_precise_orbits = true;
    bool use_precise_clocks = true;
    std::string orbit_file_path;
    std::string clock_file_path;
    bool use_ssr_corrections = false;
    bool use_clas_osr_filter = false;
    std::string ssr_file_path;
    int l6_gps_week = 0;  // GPS week for L6 binary decode (0 = auto-detect)
    Vector3d approximate_position = Vector3d::Zero();  // RINEX APPROX POS for L6 network selection
    std::string ionex_file_path;
    std::string dcb_file_path;
    std::string antex_file_path;
    std::string ocean_loading_file_path;
    std::string ocean_loading_station_name;
    std::string receiver_antenna_type;
    Vector3d receiver_antenna_delta_enu = Vector3d::Zero();

    // Ambiguity resolution
    bool enable_ambiguity_resolution = false;
    double ar_ratio_threshold = 3.0;
    int min_satellites_for_ar = 6;
    enum class ARMethod { DD_IFLC, DD_WLNL, DD_PER_FREQ };
    ARMethod ar_method = ARMethod::DD_IFLC;
    int wl_min_averaging_epochs = 20;

    // Motion model
    bool kinematic_mode = false;
    bool low_dynamics_mode = false;
    bool use_dynamics_model = false;
    bool reset_clock_to_spp_each_epoch = true;
    bool reset_kinematic_position_to_spp_each_epoch = true;

    // Kalman filter parameters
    double process_noise_position = 0.0;
    double process_noise_velocity = 1e-4;
    double process_noise_clock = 0.0;
    double process_noise_troposphere = 1e-8;
    double process_noise_ambiguity = 1e-8;
    double initial_position_variance = 3600.0;
    double initial_velocity_variance = 100.0;
    double initial_clock_variance = 3600.0;
    double initial_troposphere_variance = 0.36;
    double initial_ambiguity_variance = 3600.0;

    // Measurement noise
    double pseudorange_sigma = 1.0;
    double carrier_phase_sigma = 0.01;
    bool use_rtklib_measurement_variance = true;
    double code_phase_error_ratio_l1 = 100.0;
    double code_phase_error_ratio_l2 = 100.0;
    double rtklib_phase_error_m = 0.003;
    double rtklib_phase_error_elevation_m = 0.003;
    int phase_measurement_min_lock_count = 1;
    bool use_carrier_phase_without_precise_products = true;

    // Atmospheric modeling
    bool estimate_troposphere = true;
    bool estimate_ionosphere = false;
    double initial_ionosphere_variance = 100.0;
    double process_noise_ionosphere = 1e-3;
    bool use_ionosphere_free = true;
    ClasEpochPolicy clas_epoch_policy = ClasEpochPolicy::STRICT_OSR;
    ClasCorrectionApplicationPolicy clas_correction_application_policy =
        ClasCorrectionApplicationPolicy::FULL_OSR;
    ClasPhaseContinuityPolicy clas_phase_continuity_policy =
        ClasPhaseContinuityPolicy::FULL_REPAIR;
    ClasPhaseBiasValuePolicy clas_phase_bias_value_policy =
        ClasPhaseBiasValuePolicy::FULL;
    ClasPhaseBiasReferenceTimePolicy clas_phase_bias_reference_time_policy =
        ClasPhaseBiasReferenceTimePolicy::PHASE_BIAS_REFERENCE;
    ClasSsrTimingPolicy clas_ssr_timing_policy =
        ClasSsrTimingPolicy::LAG_TOLERANT;
    ClasExpandedValueConstructionPolicy clas_expanded_value_construction_policy =
        ClasExpandedValueConstructionPolicy::FULL_COMPOSED;
    ClasExpandedResidualSamplingPolicy clas_expanded_residual_sampling_policy =
        ClasExpandedResidualSamplingPolicy::INDEXED_OR_MEAN;
    ClasSubtype12ValueConstructionPolicy clas_subtype12_value_construction_policy =
        ClasSubtype12ValueConstructionPolicy::FULL;
    ClasAtmosSelectionPolicy clas_atmos_selection_policy =
        ClasAtmosSelectionPolicy::GRID_FIRST;
    double clas_atmos_stale_after_seconds = 15.0;

    // CLAS PPP filter tuning (affects FULL_OSR mode)
    double clas_code_variance_scale = 8.0;        // Code observation variance multiplier
    double clas_phase_variance = 0.01;            // Phase observation base variance (m^2)
    double clas_trop_prior_variance = 0.0001;     // Tight CLAS grid trop constraint
    double clas_trop_initial_variance = 1.0;      // Allow trop to converge from Saastamoinen
    double clas_trop_process_noise = 1e-6;        // Small: CLAS grid trop is stable
    double clas_initial_position_variance = 100.0; // Position covariance at filter init
    double clas_clock_variance = 1e8;             // Clock state variance (reset each epoch)
    double clas_iono_prior_variance = 0.25;       // Ionosphere pseudo-observation variance
    double clas_ambiguity_reinit_threshold = 3000.0; // Re-init ambiguity when cov exceeds this
    double clas_anchor_sigma = 5.0;               // SPP anchor constraint sigma (m)
    double clas_outlier_sigma_scale = 50.0;       // Inflate variance when residual > N*sigma
    bool clas_decouple_clock_position = true;      // Zero clock cross-covariance each epoch

    bool apply_ocean_loading = false;
    bool apply_solid_earth_tides = true;
    // When true (and apply_solid_earth_tides is also true), use the
    // IERS Conventions 2010 §7.1.1 (Dehant) Step-1 + Step-2 model
    // from libgnss::iers::solidEarthTideDisplacement instead of the
    // built-in simplified Step-1-only Love-number approximation.
    // Default on after truth-bench validation against IGS final
    // products (TSKB 2026-04-15 static, max paired displacement
    // 4.8 cm matching the IERS Step-2 envelope; see
    // docs/iers-integration-plan.md). Pass --no-iers-solid-tide
    // to revert to the legacy Step-1-only path.
    bool use_iers_solid_tide = true;
    // Optional IERS 20 C04 EOP file path (Phase D-0 scaffolding).
    // When set, PPPProcessor loads the series at construction and
    // exposes per-epoch xp/yp/UT1-UTC via getEarthOrientationParams();
    // downstream Phase D-1 (pole tide) / D-2 (sub-daily EOP) consumers
    // will read it. Empty string = no EOP table (current default).
    // Source: https://hpiers.obspm.fr/iers/eop/eopc04/eopc04.1962-now
    std::string eop_path;
    // IERS Conventions 2010 §7.1.4 pole-tide displacement.
    // Requires an EOP table loaded via eop_path; otherwise the pole
    // tide is silently skipped (instantaneous polar motion is
    // unknown, so any non-zero displacement computed from the mean
    // pole alone would be biased rather than zero).
    //
    // Default true since 2026-05-09 — backed by the multi-site bench
    // in PR #69 (5 IGS stations, median displacement 0.4 mm at
    // mid-latitudes, sign reversal across the equator consistent with
    // the §7.1.4 sin(2θ) modulation, all within the IERS-stated
    // sub-cm envelope). `--no-iers-pole-tide` is a permanent escape
    // hatch.
    bool use_iers_pole_tide = true;
    // IERS Conventions 2010 §5.5.1.1 + §8.2 sub-daily EOP corrections.
    // When true, getEarthOrientationParams adds the harmonic libration
    // + ocean-tide deltas to the daily-interpolated xp / yp / UT1-UTC
    // before returning. Peak amplitudes are sub-mas / few-µs.
    //
    // Default true since 2026-05-09. The corrections are pure
    // deterministic harmonic series (no per-site data) and produce
    // RMS 1.5 µm at the receiver position over a static-mode
    // pole-tide-on arc — well below the noise floor and within
    // IERS-conformant cm-class CIP modeling. The flip is inert
    // without an EOP table loaded (same gate as use_iers_pole_tide).
    // `--no-iers-sub-daily-eop` is a permanent escape hatch.
    bool use_iers_sub_daily_eop = true;
    // IERS Conventions 2010 §7.1.5 atmospheric tidal loading (S1/S2)
    // station displacement (opt-in). Reads per-site amplitude / phase
    // coefficients from `atm_tidal_loading_path` and adds the diurnal
    // (S1) + semi-diurnal (S2) atmospheric pressure tide displacement
    // to the receiver position. Peak ~1 mm radial at mid- and
    // low-latitudes; sub-mm horizontal. Default off; requires the
    // coefficient file. Without a loaded file, the path is a no-op.
    bool use_iers_atm_tidal_loading = false;
    std::string atm_tidal_loading_path;
    bool apply_relativity = true;

    // Convergence criteria
    double convergence_threshold_horizontal = 0.1;
    double convergence_threshold_vertical = 0.2;
    int convergence_min_epochs = 20;

    // Quality control
    bool enable_outlier_detection = true;
    double outlier_threshold = 4.0;
    bool enable_cycle_slip_detection = true;
    double cycle_slip_threshold = 0.05;
    int filter_iterations = 8;
};

struct PPPState {
    VectorXd state;
    MatrixXd covariance;

    int pos_index = 0;
    int vel_index = 3;
    int clock_index = 6;
    int glo_clock_index = 7;
    int gal_clock_index = -1;
    int qzs_clock_index = -1;
    int bds_clock_index = -1;
    int trop_index = 8;
    int iono_index = 9;
    int amb_index = 9;

    std::map<SatelliteId, int> ionosphere_indices;
    std::map<SatelliteId, int> ambiguity_indices;
    int total_states = 9;
};

struct PPPAmbiguityInfo {
    double float_value = 0.0;
    double fixed_value = 0.0;
    bool is_fixed = false;
    int lock_count = 0;
    double last_phase = 0.0;
    GNSSTime last_time;
    double quality_indicator = 0.0;
    double ambiguity_scale_m = 0.0;
    bool needs_reinitialization = true;
    double fractional_bias_cycles = 0.0;
    int fractional_bias_samples = 0;
    double last_geometry_free_m = 0.0;
    bool has_last_geometry_free = false;
    double last_melbourne_wubbena_m = 0.0;
    bool has_last_melbourne_wubbena = false;
    double mw_sum_cycles = 0.0;
    int mw_count = 0;
    double mw_mean_cycles = 0.0;
    int wl_fixed_integer = 0;
    bool wl_is_fixed = false;
    double nl_fixed_cycles = 0.0;
    bool nl_is_fixed = false;
};

}  // namespace libgnss::ppp_shared
