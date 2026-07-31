#include <libgnss++/fusion/preintegration.hpp>

#include <cmath>
#include <utility>

#include <libgnss++/fusion/mechanization.hpp>

namespace libgnss {
namespace {

bool finiteVector(const Eigen::Vector3d& value) {
    return value.allFinite();
}

bool finiteQuaternion(const Eigen::Quaterniond& value) {
    return value.coeffs().allFinite() && value.squaredNorm() > 0.0;
}

bool finiteState(const NominalState& state) {
    return std::isfinite(state.time.tow) && finiteVector(state.position_enu) &&
           finiteVector(state.velocity_enu) && finiteQuaternion(state.attitude_body_to_enu) &&
           finiteVector(state.accel_bias) && finiteVector(state.gyro_bias);
}

bool finiteNonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool validConfig(const ImuPreintegrationConfig& config) {
    return finiteVector(config.gravity_enu) && std::isfinite(config.max_sample_gap_s) &&
           config.max_sample_gap_s > 0.0 &&
           finiteNonnegative(config.process_noise.accel_noise_density) &&
           finiteNonnegative(config.process_noise.gyro_noise_density) &&
           finiteNonnegative(config.process_noise.accel_bias_random_walk) &&
           finiteNonnegative(config.process_noise.gyro_bias_random_walk);
}

bool finiteSample(const ImuSample& sample) {
    return std::isfinite(sample.time.tow) && finiteVector(sample.accel_raw) &&
           finiteVector(sample.gyro_raw_radps);
}

}  // namespace

ImuPreintegrator::ImuPreintegrator(ImuPreintegrationConfig config)
    : config_(std::move(config)) {}

PreintegrationStatus ImuPreintegrator::reset(const NominalState& anchor_state) {
    clear();
    if (!validConfig(config_)) {
        status_ = PreintegrationStatus::INVALID_CONFIG;
        invalid_ = true;
        return status_;
    }
    if (!finiteState(anchor_state)) {
        status_ = PreintegrationStatus::INVALID_ANCHOR;
        invalid_ = true;
        return status_;
    }

    anchor_state_ = anchor_state;
    anchor_state_.attitude_body_to_enu.normalize();
    predicted_state_ = anchor_state_;
    initialized_ = true;
    status_ = PreintegrationStatus::ACCEPTED;
    return status_;
}

void ImuPreintegrator::clear() {
    anchor_state_ = NominalState{};
    predicted_state_ = NominalState{};
    transition_.setIdentity();
    process_noise_.setZero();
    sample_count_ = 0;
    status_ = PreintegrationStatus::NOT_INITIALIZED;
    initialized_ = false;
    invalid_ = false;
}

PreintegrationStatus ImuPreintegrator::integrate(const ImuSample& sample_body_flu) {
    if (!initialized_) {
        status_ = PreintegrationStatus::NOT_INITIALIZED;
        return status_;
    }
    if (invalid_) {
        status_ = PreintegrationStatus::INTERVAL_INVALID;
        return status_;
    }
    if (!finiteSample(sample_body_flu)) {
        status_ = PreintegrationStatus::NON_FINITE_SAMPLE;
        invalid_ = true;
        return status_;
    }

    const double dt = sample_body_flu.time - predicted_state_.time;
    if (!std::isfinite(dt) || dt <= 0.0) {
        status_ = PreintegrationStatus::NON_MONOTONIC_TIME;
        invalid_ = true;
        return status_;
    }
    if (!std::isfinite(config_.max_sample_gap_s) || config_.max_sample_gap_s <= 0.0 ||
        dt > config_.max_sample_gap_s) {
        status_ = PreintegrationStatus::GAP_TOO_LARGE;
        invalid_ = true;
        return status_;
    }

    const Eigen::Vector3d specific_force_body =
        sample_body_flu.accel_raw - predicted_state_.accel_bias;
    const Eigen::Vector3d angular_rate_body =
        sample_body_flu.gyro_raw_radps - predicted_state_.gyro_bias;
    const FusionMatrix15 phi = fusion_process_noise::transitionMatrix(
        predicted_state_, specific_force_body, angular_rate_body, dt);
    const FusionMatrix15 qd =
        fusion_process_noise::processNoiseCovariance(phi, config_.process_noise, dt);

    FusionMatrix15 next_process_noise = phi * process_noise_ * phi.transpose() + qd;
    next_process_noise = 0.5 * (next_process_noise + next_process_noise.transpose());
    const FusionMatrix15 next_transition = phi * transition_;
    const NominalState next_state = mechanization::propagate(
        predicted_state_, sample_body_flu, dt, config_.gravity_enu);
    if (!phi.allFinite() || !qd.allFinite() || !next_process_noise.allFinite() ||
        !next_transition.allFinite() || !finiteState(next_state)) {
        status_ = PreintegrationStatus::PROPAGATION_FAILURE;
        invalid_ = true;
        return status_;
    }

    process_noise_ = next_process_noise;
    transition_ = next_transition;
    predicted_state_ = next_state;
    ++sample_count_;
    status_ = PreintegrationStatus::ACCEPTED;
    return status_;
}

ImuPreintegrationResult ImuPreintegrator::result() const {
    ImuPreintegrationResult output;
    output.valid = valid();
    output.status = status_;
    output.anchor_state = anchor_state_;
    output.predicted_state = predicted_state_;
    output.transition = transition_;
    output.process_noise = process_noise_;
    output.sample_count = sample_count_;
    output.duration_s = initialized_ ? predicted_state_.time - anchor_state_.time : 0.0;
    return output;
}

}  // namespace libgnss
