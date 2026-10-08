#pragma once

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
        Vector3d base_position_ecef = Vector3d::Zero();
        double max_imu_gap_s = 0.1;
        double max_fusion_age_s = 0.02;
        double max_rover_gap_s = 2.0;
        double max_tight_interval_s = 2.0;
        std::size_t max_pending_imu = 10000;
        std::size_t max_pending_base = 16;
        std::size_t max_ephemerides_per_satellite = 8;
    };
    struct Output {
        PositionSolution rtk;
        PositionSolution fused;
        GNSSTime received_at;
        double input_age_s = 0.0;
        double fusion_age_s = 0.0;
        double processing_ms = 0.0;
        bool exact_base_available = false;
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
        std::size_t imu_gap_resets = 0;
        std::size_t rover_gap_resets = 0;
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

private:
    Config config_;
    std::unique_ptr<RTKProcessor> rtk_;
    std::unique_ptr<LooseCouplingProcessor> fusion_;
    std::unique_ptr<TightCouplingProcessor> tight_;
    NavigationData navigation_;
    std::deque<ObservationData> base_;
    std::deque<ImuSample> imu_;
    Diagnostics diagnostics_;
    GNSSTime arrival_, rover_time_, imu_time_, last_queued_imu_;
    GNSSTime tight_anchor_time_;
    bool have_arrival_ = false, have_rover_ = false, have_imu_ = false;
    bool have_queued_imu_ = false;
    bool have_tight_anchor_ = false;
    void validateArrival(const GNSSTime& received_at) const;
    void acceptArrival(const GNSSTime& received_at);
    void recreateFilters();
    void recreateTightFilter();
};

} // namespace libgnss
