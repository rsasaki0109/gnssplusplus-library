#pragma once

#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

#include <array>
#include <map>
#include <string>

namespace libgnss::ppp_shared {

struct PPPConfig {
    // Precise products
    bool use_precise_orbits = true;
    bool use_precise_clocks = true;
    std::string orbit_file_path;
    std::string clock_file_path;
    bool use_ssr_corrections = false;
    bool use_clas_osr_filter = false;
    std::string ssr_file_path;
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
    bool apply_ocean_loading = false;
    bool apply_solid_earth_tides = true;
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
