#pragma once

#include <cstddef>
#include <deque>

#include <Eigen/Dense>

#include <libgnss++/fusion/fusion_process_noise.hpp>
#include <libgnss++/fusion/fusion_types.hpp>
#include <libgnss++/fusion/preintegration.hpp>
#include <libgnss++/io/imu.hpp>

namespace libgnss {

class TightCouplingProcessor {
public:
    struct Config {
        ProcessNoiseConfig process_noise;
        Eigen::Vector3d lever_arm_body = Eigen::Vector3d::Zero();
        double max_sample_gap_s = 0.1;
        bool zupt_enable = true;
        double zupt_sigma_mps = 0.5;
        double zupt_max_accel_std = 0.55;
        double zupt_max_gyro_std = 0.030;
        double zupt_max_gyro_median = 0.020;
        bool nhc_enable = false;
        double nhc_sigma_lateral_mps = 0.3;
        double nhc_sigma_vertical_mps = 0.2;
    };

    struct TimeUpdate {
        bool valid = false;
        Eigen::Vector3d antenna_delta_ecef = Eigen::Vector3d::Zero();
        Eigen::Matrix3d process_noise_ecef = Eigen::Matrix3d::Zero();
    };

    struct Diagnostics {
        std::size_t anchors = 0;
        std::size_t supplied_updates = 0;
        std::size_t invalid_intervals = 0;
        std::size_t zupt_updates = 0;
        std::size_t nhc_updates = 0;
    };

    TightCouplingProcessor();
    explicit TightCouplingProcessor(Config config);

    void processImuSample(const ImuSample& sample_body_flu);
    TimeUpdate prepareTimeUpdate();

    /**
     * Re-anchor at RTK's FLOAT posterior antenna position. `bootstrap_state`
     * is required only for the first anchor; later calls retain this
     * processor's propagated attitude and biases.
     */
    bool reanchor(const Eigen::Vector3d& float_antenna_position_ecef,
                  const Eigen::Matrix3d& float_position_covariance_ecef,
                  const Eigen::Vector3d& antenna_velocity_ecef,
                  const GNSSTime& time,
                  const FusionState* bootstrap_state = nullptr);

    void invalidateInterval();
    bool initialized() const { return initialized_; }
    bool intervalValid() const { return preintegrator_.valid(); }
    const FusionState& state() const { return state_; }
    Diagnostics diagnostics() const { return diagnostics_; }

private:
    Config config_;
    ImuPreintegrator preintegrator_;
    FusionState state_;
    bool initialized_ = false;
    bool prepared_ = false;
    double anchor_lat_rad_ = 0.0;
    double anchor_lon_rad_ = 0.0;
    std::deque<ImuSample> stationary_window_;
    Eigen::Vector3d last_angular_rate_body_ = Eigen::Vector3d::Zero();
    Diagnostics diagnostics_;

    bool detectStationary() const;
    bool applyConstraintUpdates();
};

}  // namespace libgnss
