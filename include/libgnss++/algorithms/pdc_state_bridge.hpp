#pragma once

#include <libgnss++/core/types.hpp>

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace libgnss::pdc_state_bridge {

/**
 * @brief One raw, already-corrected pseudorange row for the PDC state solve.
 *
 * The row uses the same satellite state, corrected measurement, clock-group
 * and sigma as FGOProcessor::PseudorangeFactor. It is consumed in memory; no
 * coordinate or solver output file is part of this interface.
 */
struct PseudorangeRow {
    std::size_t epoch_index = 0;
    SatelliteId satellite;
    GNSSSystem clock_group = GNSSSystem::GPS;
    Vector3d satellite_position_ecef = Vector3d::Zero();
    double corrected_pseudorange_m = 0.0;
    double sigma_m = 0.0;
};

/** One corrected receiver-only Doppler row for the same state graph. */
struct DopplerRow {
    std::size_t epoch_index = 0;
    Vector3d los = Vector3d::Zero();
    double residual_mps = 0.0;
    double sigma_mps = 0.0;
};

struct EpochInput {
    GNSSTime time;
    Vector3d seed_position_ecef = Vector3d::Zero();
    double seed_clock_bias_m = 0.0;
    bool clock_jump = false;
    // Optional raw-observable Doppler-WLS initializer.  When present it is
    // only an initial value; the bridge still solves velocity and clock-rate
    // from its own P+D+temporal equations.  Invalid/unavailable estimates
    // deliberately retain the historical zero initializer.
    Vector3d seed_velocity_ecef_mps = Vector3d::Zero();
    double seed_clock_rate_mps = 0.0;
    bool has_seed_velocity = false;
    bool has_seed_clock_rate = false;
};

/** Physically predeclared, truth-free controls matching the native PDC path. */
struct Options {
    int max_iterations = 12;
    int min_pseudorange_rows = 4;
    int min_doppler_rows = 4;
    double pseudorange_huber_threshold_sigma = 4.0;
    double doppler_huber_threshold_sigma = 4.0;
    double position_prior_sigma_m = 1000.0;
    double clock_prior_sigma_m = 1.0e6;
    double velocity_prior_sigma_mps = 1000.0;
    double clock_rate_prior_sigma_mps = 1000.0;
    double motion_sigma_m = 0.1;
    double clock_motion_sigma_m = 0.1;
    double clock_jump_sigma_m = 1.0e6;
    double inter_system_clock_motion_sigma_m = 1.0e-6;
    double clock_rate_between_sigma_mps = 0.1;
    double max_velocity_mps = 70.0;
    double max_clock_rate_mps = 2000.0;
    double max_normalized_rms = 25.0;
    double max_position_norm_m = 1.0e7;
};

struct EpochState {
    Vector3d position_ecef = Vector3d::Zero();
    std::array<double, 5> clock_bias_m = {0.0, 0.0, 0.0, 0.0, 0.0};
    Vector3d velocity_ecef_mps = Vector3d::Zero();
    double clock_rate_mps = 0.0;
};

struct EpochEstimate {
    bool valid = false;
    EpochState state;
    int pseudorange_rows = 0;
    int doppler_rows = 0;
    int inlier_rows = 0;
    double normalized_rms = std::numeric_limits<double>::infinity();
    double max_abs_normalized_residual =
        std::numeric_limits<double>::infinity();
    std::string reason = "uninitialized";
};

struct SolveResult {
    bool valid = false;
    bool converged = false;
    int iterations = 0;
    std::size_t pseudorange_rows = 0;
    std::size_t doppler_rows = 0;
    std::size_t motion_intervals = 0;
    std::size_t valid_epochs = 0;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    double max_velocity_norm_mps = 0.0;
    double max_clock_rate_abs_mps = 0.0;
    std::string reason = "uninitialized";
    std::vector<EpochEstimate> epochs;
};

/** Map FGO's shared clock groups to the five-column native PDC contract. */
int clockGroupIndex(GNSSSystem group);

/**
 * @brief Solve the full raw P+D+temporal PDC state in memory.
 *
 * The state is 3 ECEF position, five receiver clock biases, 3 ECEF velocity
 * and one receiver clock-rate per epoch. Pseudorange, corrected undifferenced
 * Doppler, midpoint motion/clock continuity and bounded priors are solved
 * together. The returned state is intended as an initialization for another
 * graph that already contains the raw factors, avoiding a second measurement
 * weighting/prior contract.
 */
SolveResult solve(const std::vector<EpochInput>& epochs,
                  const std::vector<PseudorangeRow>& pseudorange_rows,
                  const std::vector<DopplerRow>& doppler_rows,
                  const Options& options = Options());

}  // namespace libgnss::pdc_state_bridge
