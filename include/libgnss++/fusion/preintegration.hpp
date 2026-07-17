#pragma once

#include <cstddef>

#include <Eigen/Dense>

#include <libgnss++/fusion/fusion_process_noise.hpp>
#include <libgnss++/fusion/fusion_types.hpp>
#include <libgnss++/io/imu.hpp>

namespace libgnss {

using FusionMatrix15 = Eigen::Matrix<double, fusion_index::SIZE, fusion_index::SIZE>;

enum class PreintegrationStatus {
    ACCEPTED,
    NOT_INITIALIZED,
    INTERVAL_INVALID,
    INVALID_CONFIG,
    INVALID_ANCHOR,
    NON_FINITE_SAMPLE,
    NON_MONOTONIC_TIME,
    GAP_TOO_LARGE,
    PROPAGATION_FAILURE,
};

struct ImuPreintegrationConfig {
    ProcessNoiseConfig process_noise;
    Eigen::Vector3d gravity_enu{0.0, 0.0, -kStandardGravityMps2};
    double max_sample_gap_s = 0.1;
};

/**
 * @brief One RTK-to-RTK IMU propagation interval in a caller-owned ENU frame.
 *
 * `transition` maps a 15-state error at the anchor to the end of the interval.
 * `process_noise` contains only noise accumulated inside the interval; it does
 * not include an anchor covariance. This lets an estimator propagate its own
 * covariance without duplicating state ownership inside the preintegrator.
 */
struct ImuPreintegrationResult {
    bool valid = false;
    PreintegrationStatus status = PreintegrationStatus::NOT_INITIALIZED;
    NominalState anchor_state;
    NominalState predicted_state;
    FusionMatrix15 transition = FusionMatrix15::Identity();
    FusionMatrix15 process_noise = FusionMatrix15::Zero();
    std::size_t sample_count = 0;
    double duration_s = 0.0;
};

/**
 * @brief Mechanize one IMU interval and compose its error-state transition.
 *
 * Samples must already be expressed in body FLU and gyroscope units must be
 * radians per second. Any invalid sample latches the whole interval invalid;
 * reset() is required before another sample can be accepted.
 */
class ImuPreintegrator {
public:
    explicit ImuPreintegrator(ImuPreintegrationConfig config = {});

    PreintegrationStatus reset(const NominalState& anchor_state);
    void clear();
    PreintegrationStatus integrate(const ImuSample& sample_body_flu);

    bool initialized() const { return initialized_; }
    bool valid() const { return initialized_ && !invalid_ && sample_count_ > 0; }
    ImuPreintegrationResult result() const;

private:
    ImuPreintegrationConfig config_;
    NominalState anchor_state_;
    NominalState predicted_state_;
    FusionMatrix15 transition_ = FusionMatrix15::Identity();
    FusionMatrix15 process_noise_ = FusionMatrix15::Zero();
    std::size_t sample_count_ = 0;
    PreintegrationStatus status_ = PreintegrationStatus::NOT_INITIALIZED;
    bool initialized_ = false;
    bool invalid_ = false;
};

}  // namespace libgnss
