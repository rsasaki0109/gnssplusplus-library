#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/core/types.hpp>
#include <libgnss++/fusion/fusion_processor.hpp>
#include <libgnss++/fusion/preintegration.hpp>
#include <libgnss++/fusion/tight_coupling_processor.hpp>
#include <libgnss++/io/imu.hpp>

namespace libgnss {

/**
 * Epoch-synchronous RTK + IMU fusion: the per-epoch pipeline behind
 * `gnss fuse --base`.
 *
 * Owns the RTK filter (RTKProcessor), the main loose ESKF
 * (LooseCouplingProcessor), the optional independent output filter, the M1
 * INS time-update preintegrator and the M3 closed-loop INS
 * (TightCouplingProcessor), and wires them together exactly as `gnss fuse`
 * does. The caller owns file I/O and time alignment: for each rover epoch it
 * feeds every IMU sample up to the rover time through processImuSample()
 * (body FLU, after any axis remap), then calls processEpoch() with a base
 * epoch already aligned to the rover time (exact or interpolated).
 *
 * This is not OnlineRtkImuProcessor: that class takes received events in
 * arrival order and does its own queueing, base selection and resets.
 * Calls are serialized by the caller; the object is not thread-safe.
 */
class RtkImuFusionProcessor {
public:
    struct Config {
        /// Main loose ESKF configuration (`gnss fuse` fusion flags). The
        /// output filter, the preintegrator and the closed-loop INS take
        /// their process noise, lever arm and ZUPT/NHC settings from it.
        LooseCouplingProcessor::Config fusion_config;
        /// --independent-output-filter: write the output stream from a
        /// second loose ESKF fed the RTK position and the epoch SPP Doppler
        /// velocity (consider update before the heading latch). It never
        /// feeds the RTK filter or the closed-loop INS.
        bool independent_output_filter = false;
        /// --rtk-ins-prior: hand the main ESKF's predicted antenna position
        /// to RTKProcessor as the epoch's kinematic prior when the filter is
        /// initialized, anchored and heading-converged, subject to the two
        /// gates below.
        bool rtk_ins_prior = false;
        /// Multiplier on the ESKF predicted-position covariance.
        double rtk_ins_prior_inflation = 25.0;
        /// Only inject when the raw rover satellite count is below
        /// rtk_ins_prior_min_sats.
        bool rtk_ins_prior_spp_gate = true;
        int rtk_ins_prior_min_sats = 20;
        /// Only inject when the ESKF velocity NIS EMA is at most this
        /// value; <= 0 disables the gate.
        double rtk_ins_prior_max_nis = 3.0;
        /// Record the satellite count and NIS of each eligible epoch in
        /// Counters (`gnss fuse --verbose`).
        bool record_ins_prior_signals = false;
        /// --tc-ins-time-update: M1 RTK-hosted INS time update.
        bool tc_ins_time_update = false;
        /// --tc-closed-loop: M3 closed-loop INS time update.
        bool tc_closed_loop = false;
        /// --tc-velocity-states: RTK velocity states fed by the closed loop.
        bool tc_velocity_states = false;
        /// --tc-tdcp-diagnostics: accumulate RTK TDCP telemetry.
        bool tc_tdcp_diagnostics = false;
        /// Maximum IMU sample gap for the preintegrator and the closed loop.
        double tc_ins_max_sample_gap_s = 0.1;
        /// --tight-dd-imu: DD rows into the main ESKF after the RTK update.
        bool tightly_coupled_dd_imu = false;
        /// --tight-dd-code-only: drop the carrier rows of that update.
        bool tightly_coupled_dd_code_only = false;
    };

    /// Run counters for the `gnss fuse` summary.
    struct Counters {
        // M1 INS time update.
        int tc_update_supplied_count = 0;
        int tc_anchor_count = 0;
        int tc_invalid_interval_count = 0;
        // CP-vs-PR fixed gate (closed loop).
        int tc_cp_pr_evaluated_count = 0;
        int tc_cp_pr_rejected_count = 0;
        int tc_cp_pr_escalated_count = 0;
        int tc_ddpr_anchor_count = 0;
        // TDCP diagnostics.
        std::size_t tc_tdcp_candidate_count = 0;
        std::size_t tc_tdcp_residual_count = 0;
        std::size_t tc_tdcp_missing_previous_count = 0;
        std::size_t tc_tdcp_gap_count = 0;
        std::size_t tc_tdcp_loss_of_lock_count = 0;
        std::size_t tc_tdcp_invalid_count = 0;
        double tc_tdcp_residual_sum_squares = 0.0;
        double tc_tdcp_residual_max_abs_m = 0.0;
        // Tightly coupled DD/IMU update.
        int tight_dd_epochs = 0;
        int tight_dd_accepted = 0;
        int tight_dd_rejected = 0;
        int tight_dd_carrier_fallbacks = 0;
        int tight_dd_rows = 0;
        int tight_dd_partial_ar_epochs = 0;
        int tight_dd_fixed_ambiguities = 0;
        int tight_dd_sse_par_available_epochs = 0;
        int tight_dd_sse_par_eligible_epochs = 0;
        int tight_dd_sse_par_passed_epochs = 0;
        int tight_dd_sse_par_fixed_ambiguities = 0;
        int tight_dd_sse_par_subsets_evaluated = 0;
        int tight_dd_sse_par_ratio_passed_subsets = 0;
        int tight_dd_sse_par_separation_rejected_subsets = 0;
        int tight_dd_soft_resets = 0;
        int tight_dd_nis_samples = 0;
        double tight_dd_nis_sum = 0.0;
        double tight_dd_nis_max = 0.0;
        // INS position prior: epochs eligible to fire (initialized,
        // anchored, heading-converged) and epochs that injected a prior.
        int ins_prior_eligible_epochs = 0;
        int ins_prior_fire_count = 0;
        // Per eligible epoch, only with Config::record_ins_prior_signals.
        std::vector<int> ins_prior_eligible_nsats;
        std::vector<double> ins_prior_eligible_nis;
    };

    struct EpochResult {
        /// processRTKEpoch()'s solution. When it is valid, its velocity is
        /// replaced by the main ESKF's, and the tightly coupled DD update
        /// may adjust it.
        PositionSolution rtk;
        /// processRTKEpoch() returned a valid solution and the filters took
        /// a GNSS update.
        bool rtk_valid = false;
        /// The output filter (independent or main) has an ECEF origin, so
        /// output and the attitude below are set.
        bool has_output = false;
        /// Antenna position solution of the output filter.
        PositionSolution output;
        GNSSTime attitude_time;
        Eigen::Quaterniond attitude_body_to_enu = Eigen::Quaterniond::Identity();
    };

    /// rtk_config is the caller's RTK tuning; the fields that couple RTK to
    /// the inertial filters (external position prior and time update,
    /// velocity states, TDCP diagnostics, CP-vs-PR gate) are set from
    /// config.
    RtkImuFusionProcessor(const Config& config,
                          RTKProcessor::RTKConfig rtk_config,
                          const Eigen::Vector3d& base_position_ecef);

    /// One IMU sample (body FLU) with time at or before the next rover epoch.
    void processImuSample(const ImuSample& sample_body_flu);

    /// One rover epoch with its time-aligned base epoch.
    EpochResult processEpoch(const ObservationData& rover_obs,
                             const ObservationData& aligned_base_obs,
                             const NavigationData& nav_data);

    const Config& config() const { return config_; }
    const Counters& counters() const { return counters_; }
    const RTKProcessor& rtk() const { return rtk_processor_; }
    const LooseCouplingProcessor& fusion() const { return fusion_processor_; }
    const LooseCouplingProcessor& outputFilter() const { return output_filter_; }
    const TightCouplingProcessor& closedLoop() const { return tc_closed_loop_processor_; }

private:
    Config config_;
    RTKProcessor rtk_processor_;
    LooseCouplingProcessor fusion_processor_;
    LooseCouplingProcessor output_filter_;
    ImuPreintegrator tc_preintegrator_;
    TightCouplingProcessor tc_closed_loop_processor_;
    double tc_anchor_lat_rad_ = 0.0;
    double tc_anchor_lon_rad_ = 0.0;
    Counters counters_;
};

}  // namespace libgnss
