#pragma once

#include <optional>

#include <deque>
#include <memory>
#include <limits>
#include <string>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/fusion/fusion_processor.hpp>
#include <libgnss++/fusion/tight_coupling_processor.hpp>

namespace libgnss {

/** Received-event processor. No file, suffix scan, smoothing or future-base
 * interpolation is available through this interface. All timestamps are GPST.
 * Calls are serialized by the caller; the object is not thread-safe. */
class OnlineRtkImuProcessor {
public:
    struct Config {
        ProcessorConfig processor;
        RTKProcessor::RTKConfig rtk;
        LooseCouplingProcessor::Config fusion;
        bool tight_time_update = true;
        // Opt-in (velocity_consistency_v1). With tight_time_update the RTK
        // filter carries its own velocity state, which is the tight INS
        // prediction fed back by reanchor(). Feeding that state to the loose
        // filter and to tight reanchor() closes a self-confirming loop, so
        // use an independent Doppler least-squares velocity (with its own
        // covariance) at the RTK position for both. If it cannot be solved
        // the epoch carries no GNSS velocity (never the RTK state velocity).
        bool independent_doppler_velocity = false;
        // Opt-in (velocity_consistency_v8 (n)); requires
        // independent_doppler_velocity (the constructor rejects it without).
        // The independent velocity (with its covariance) fed to the fusion
        // filters and to tight reanchor() is then the SPP processor's own
        // velocity solved in the same RTK epoch (RTKProcessor::currentSpp(),
        // one signal per satellite, elevation mask, pseudorange outlier
        // rejection) instead of the all-rows Doppler least squares. When that
        // SPP is not valid with a finite velocity and covariance, the epoch
        // carries no GNSS velocity, exactly as when the least squares fails.
        // The same velocity, when available, also replaces the velocity and
        // velocity covariance of the exported Output::rtk; otherwise the
        // exported velocity is unchanged. false keeps the previous behavior
        // bit-for-bit.
        bool independent_velocity_from_epoch_spp = false;
        // Opt-in (velocity_consistency_v3). When set, the RTK filter's INS prior
        // is bootstrapped from a second, isolated loose-coupling filter built
        // from this configuration and fed the unmodified legacy RTK covariance,
        // so what the fused output does (gates, re-anchors, honest RTK
        // covariance) cannot change the RTK filter's own output. Unset
        // (default): the single fused filter bootstraps the RTK prior as before.
        std::optional<LooseCouplingProcessor::Config> rtk_prior_fusion;
        Vector3d base_position_ecef = Vector3d::Zero();
        double max_imu_gap_s = 0.1;
        double max_fusion_age_s = 0.02;
        double max_rover_gap_s = 2.0;
        double max_tight_interval_s = 2.0;
        std::size_t max_pending_imu = 10000;
        std::size_t max_pending_base = 16;
        std::size_t max_ephemerides_per_satellite = 8;
        // Opt-in (velocity_consistency_v5). A rover-only gap (> max_rover_gap_s
        // between rover epochs) is a GNSS-only outage: the IMU stream and the
        // loose fused filter's mechanization stayed continuous. When true, such
        // a gap recreates the RTK filter, the tight filter and the isolated
        // RTK-prior filter exactly as before but keeps the loose fused filter
        // (and have_imu_, so the unchanged IMU-gap checks still apply). It is
        // reported as reason "rover_gap_rtk_reset", counted in
        // Diagnostics::rover_gap_rtk_resets, and does not advance
        // reset_generation. False keeps the previous behavior bit-for-bit.
        bool rover_gap_keeps_inertial_filters = false;
        // Opt-in (velocity_consistency_v6). Every reset that recreates the
        // loose fused filter after construction (imu_gap_reset,
        // imu_stale_reset, and rover_gap_reset when
        // rover_gap_keeps_inertial_filters is false) carries the old fused
        // filter's nominal gyro bias, if that filter was initialized, into the
        // new one: it replaces the window-mean gyro bias at the new filter's
        // next static-window initialization (see
        // LooseCouplingProcessor::seedGyroBiasForNextInitialization). The
        // isolated RTK-prior filter is recreated without a seed. The public
        // reset() carries nothing. False keeps the previous behavior
        // bit-for-bit.
        bool carry_gyro_bias_across_reset = false;
        // Opt-in (rtk_base_extrapolation_v1, docs/online_rtk_base_extrapolation_v1.md).
        // 0 disables it and keeps the previous behavior bit-for-bit. When > 0
        // and no base epoch exists at the rover time, the latest base epoch
        // at or before the rover time is held to the rover time (geometry
        // corrected zero-order hold, see rtk_base_alignment.hpp) if its age is
        // <= this many seconds, and used like an exact base epoch for RTK and
        // for tight anchoring. Otherwise the existing SPP fallback runs.
        double base_extrapolation_max_age_s = 0.0;
        // Opt-in (rtk_online_product_v1, docs/online_rtk_product_config_v1.md).
        // Empty keeps the previous behavior bit-for-bit. A name accepted by
        // applyRtkPreset() (rtk_presets.hpp, e.g. "low-cost") is applied to a
        // copy of `rtk` when the RTK filter is (re)created, before the
        // processor's own overrides (use_external_position_time_update,
        // enable_velocity_states). Unknown names are rejected by the
        // constructor. The isolated RTK-prior filter and the fusion
        // configurations are not affected.
        std::string rtk_preset;
    };
    struct Output {
        PositionSolution rtk;
        PositionSolution fused;
        GNSSTime received_at;
        double input_age_s = 0.0;
        double fusion_age_s = 0.0;
        double processing_ms = 0.0;
        // True only for a received base epoch at the rover time (+-1e-6 s).
        bool exact_base_available = false;
        // True only when base_extrapolation_max_age_s > 0 and a held past base
        // epoch was used instead (never together with exact_base_available).
        bool extrapolated_base_available = false;
        bool fusion_initialized = false;
        bool heading_converged = false;
        // Snapshot of the loose-coupling attitude actually used for fused
        // output. Availability does not imply observed or accurate heading.
        // Uninitialized/stale states have NaNs, never a valid-looking identity.
        bool attitude_available = false;
        bool heading_aligned = false;
        GNSSTime attitude_time;
        Eigen::Quaterniond attitude_body_to_enu = Eigen::Quaterniond(
            std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
        Vector3d rpy_frd_ned_deg = Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        // Fixed filter frame, needed to transport attitude to another local
        // tangent frame. Row-major elements are exported with the snapshot.
        Matrix3d ecef_to_attitude_enu = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Vector3d accel_bias_body_mps2 = Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Vector3d gyro_bias_body_radps = Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        bool gnss_position_updated = false;
        bool tight_time_update_supplied = false;
        std::size_t reset_generation = 0;
        std::size_t imu_consumed = 0;
        std::string reason;
    };
    struct Diagnostics {
        std::size_t rover_epochs = 0;
        std::size_t missing_base_epochs = 0;
        std::size_t late_imu_dropped = 0;
        std::size_t expired_base_epochs = 0;
        // Rover epochs that used a held past base epoch (extrapolation only).
        std::size_t extrapolated_base_epochs = 0;
        std::size_t imu_gap_resets = 0;
        std::size_t rover_gap_resets = 0;
        std::size_t rover_gap_rtk_resets = 0;
        // velocity_consistency_v8 diagnostics, accumulated across filter
        // re-creations. Zero unless the matching option is enabled (the
        // gate-exceeded count is informational and needs a configured float
        // prefit gate).
        // Differential RTK epochs the FLOAT seeded at the base was rejected.
        std::size_t rtk_base_seed_rejections = 0;
        // INS-seeded RTK floats rejected because the update retained fewer
        // than RTKConfig::ins_prior_min_code_rows code rows (the INS prior was
        // dropped for the next epoch).
        std::size_t rtk_ins_prior_unsupported_rejections = 0;
        // Differential RTK epochs where the SPP-fallback blanking was skipped
        // because the trusted anchor was too old.
        std::size_t rtk_spp_blank_age_limited = 0;
        // Valid RTK epochs whose float_prefit_gate_exceeded was set.
        std::size_t rtk_float_prefit_gate_exceeded = 0;
        // Epochs where the fused filter refused a re-anchor for that reason.
        std::size_t fusion_reanchor_prefit_refusals = 0;
        // Epochs where the exported RTK velocity was the epoch SPP velocity.
        std::size_t epoch_spp_velocity_exports = 0;
        std::size_t reset_generation = 0;
    };

    explicit OnlineRtkImuProcessor(const Config& config);
    void pushNavigation(const NavigationData& navigation, const GNSSTime& received_at);
    void pushBase(const ObservationData& observation, const GNSSTime& received_at);
    bool pushImu(const ImuSample& sample_body_flu, const GNSSTime& received_at);
    Output processRover(const ObservationData& observation, const GNSSTime& received_at);
    void reset(const GNSSTime& received_at);
    Diagnostics diagnostics() const { return diagnostics_; }
    std::size_t pendingImu() const { return imu_.size(); }
    std::size_t pendingBase() const { return base_.size(); }
    /** Read-only view of the loose fused filter (diagnostics and tests). */
    const LooseCouplingProcessor& fusionFilter() const { return *fusion_; }
    /** Read-only view of the isolated RTK-prior filter; null unless
     * Config::rtk_prior_fusion is set (diagnostics and tests). */
    const LooseCouplingProcessor* priorFusionFilter() const { return prior_fusion_.get(); }
    /** Read-only view of the RTK filter (diagnostics and tests). */
    const RTKProcessor& rtkFilter() const { return *rtk_; }

private:
    Config config_;
    std::unique_ptr<RTKProcessor> rtk_;
    std::unique_ptr<LooseCouplingProcessor> fusion_;
    std::unique_ptr<LooseCouplingProcessor> prior_fusion_;
    std::unique_ptr<TightCouplingProcessor> tight_;
    NavigationData navigation_;
    std::deque<ObservationData> base_;
    // Latest received base epoch at or before the rover time that has left
    // base_ (expired or consumed). Maintained only when extrapolation is on.
    ObservationData last_past_base_;
    bool have_last_past_base_ = false;
    std::deque<ImuSample> imu_;
    Diagnostics diagnostics_;
    GNSSTime arrival_, rover_time_, imu_time_, last_queued_imu_;
    GNSSTime tight_anchor_time_;
    bool have_arrival_ = false, have_rover_ = false, have_imu_ = false;
    bool have_queued_imu_ = false;
    bool have_tight_anchor_ = false;
    void validateArrival(const GNSSTime& received_at) const;
    void acceptArrival(const GNSSTime& received_at);
    // carry_gyro_bias: true only for the internal gap/stale resets; the
    // constructor and the public reset() recreate without carrying.
    void recreateFilters(bool carry_gyro_bias = false);
    // Pieces of recreateFilters(). recreateRtkSideFilters() is everything but
    // the loose fused filter and the IMU continuity flag.
    void recreateRtkFilter();
    void countRtkEpochDiagnostics(const PositionSolution& rtk_solution);
    void recreateFusionFilter();
    void recreatePriorFusionFilter();
    void recreateRtkSideFilters();
    void recreateTightFilter();
};

} // namespace libgnss
