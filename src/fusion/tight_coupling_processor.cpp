#include <libgnss++/fusion/tight_coupling_processor.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/fusion/attitude.hpp>
#include <libgnss++/fusion/fusion_measurement.hpp>
#include <libgnss++/fusion/fusion_update.hpp>

namespace libgnss {
namespace {

constexpr std::size_t kStationaryWindowSize = 20;

Eigen::Matrix3d ecefToEnuRotation(double lat, double lon) {
    Eigen::Matrix3d rotation;
    rotation.col(0) = ecef2enu(Eigen::Vector3d::UnitX(), lat, lon);
    rotation.col(1) = ecef2enu(Eigen::Vector3d::UnitY(), lat, lon);
    rotation.col(2) = ecef2enu(Eigen::Vector3d::UnitZ(), lat, lon);
    return rotation;
}

double standardDeviation(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    double mean = 0.0;
    for (double value : values) mean += value;
    mean /= values.size();
    double variance = 0.0;
    for (double value : values) variance += (value - mean) * (value - mean);
    return std::sqrt(variance / values.size());
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 == 0 ? 0.5 * (values[middle - 1] + values[middle])
                                  : values[middle];
}

void injectError(FusionState& state, Eigen::Matrix<double, 15, 1>& error) {
    state.nominal.position_enu += error.segment<3>(fusion_index::POSITION);
    state.nominal.velocity_enu += error.segment<3>(fusion_index::VELOCITY);
    state.nominal.attitude_body_to_enu =
        (state.nominal.attitude_body_to_enu *
         attitude::smallAngleQuaternion(error.segment<3>(fusion_index::ATTITUDE))).normalized();
    state.nominal.accel_bias += error.segment<3>(fusion_index::ACCEL_BIAS);
    state.nominal.gyro_bias += error.segment<3>(fusion_index::GYRO_BIAS);
    error.setZero();
}

}  // namespace

TightCouplingProcessor::TightCouplingProcessor() : TightCouplingProcessor(Config{}) {}

TightCouplingProcessor::TightCouplingProcessor(Config config)
    : config_(std::move(config)),
      preintegrator_(ImuPreintegrationConfig{
          config_.process_noise, {0.0, 0.0, -kStandardGravityMps2}, config_.max_sample_gap_s}) {}

void TightCouplingProcessor::processImuSample(const ImuSample& sample_body_flu) {
    stationary_window_.push_back(sample_body_flu);
    while (stationary_window_.size() > kStationaryWindowSize) stationary_window_.pop_front();
    if (!initialized_ || prepared_) return;
    const auto status = preintegrator_.integrate(sample_body_flu);
    if (status == PreintegrationStatus::ACCEPTED) {
        last_angular_rate_body_ = sample_body_flu.gyro_raw_radps - state_.nominal.gyro_bias;
    } else if (status != PreintegrationStatus::INTERVAL_INVALID) {
        ++diagnostics_.invalid_intervals;
    }
}

bool TightCouplingProcessor::detectStationary() const {
    if (stationary_window_.size() < kStationaryWindowSize) return false;
    std::vector<double> accel_norms, gyro_norms;
    accel_norms.reserve(stationary_window_.size());
    gyro_norms.reserve(stationary_window_.size());
    for (const auto& sample : stationary_window_) {
        accel_norms.push_back(sample.accel_raw.norm());
        gyro_norms.push_back(sample.gyro_raw_radps.norm());
    }
    return standardDeviation(accel_norms) <= config_.zupt_max_accel_std &&
           standardDeviation(gyro_norms) <= config_.zupt_max_gyro_std &&
           median(gyro_norms) <= config_.zupt_max_gyro_median;
}

bool TightCouplingProcessor::applyConstraintUpdates() {
    Eigen::Matrix<double, 15, 1> error = Eigen::Matrix<double, 15, 1>::Zero();
    if (config_.zupt_enable && detectStationary()) {
        const auto system = fusion_measurement::buildZuptUpdate(state_, config_.zupt_sigma_mps);
        const auto result = fusion_update::applyDenseUpdate(error, state_.covariance, system);
        if (result.ok) {
            injectError(state_, error);
            ++diagnostics_.zupt_updates;
        }
    }
    if (config_.nhc_enable) {
        const auto system = fusion_measurement::buildNhcUpdate(
            state_, config_.lever_arm_body, last_angular_rate_body_,
            config_.nhc_sigma_lateral_mps, config_.nhc_sigma_vertical_mps);
        const auto result = fusion_update::applyDenseUpdate(error, state_.covariance, system);
        if (result.ok) {
            injectError(state_, error);
            ++diagnostics_.nhc_updates;
        }
    }
    return state_.nominal.position_enu.allFinite() && state_.covariance.allFinite();
}

TightCouplingProcessor::TimeUpdate TightCouplingProcessor::prepareTimeUpdate() {
    TimeUpdate output;
    if (!initialized_ || prepared_ || !preintegrator_.valid()) return output;
    const auto result = preintegrator_.result();
    state_.nominal = result.predicted_state;
    state_.covariance = result.transition * state_.covariance * result.transition.transpose() +
                        result.process_noise;
    state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
    if (!applyConstraintUpdates()) {
        invalidateInterval();
        return output;
    }

    const Eigen::Matrix3d body_to_enu = state_.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d antenna_delta_enu =
        state_.nominal.position_enu + body_to_enu * config_.lever_arm_body;
    Eigen::Matrix<double, 3, fusion_index::SIZE> jacobian =
        Eigen::Matrix<double, 3, fusion_index::SIZE>::Zero();
    jacobian.block<3, 3>(0, fusion_index::POSITION).setIdentity();
    jacobian.block<3, 3>(0, fusion_index::ATTITUDE) =
        -body_to_enu * attitude::skew(config_.lever_arm_body);
    const Eigen::Matrix3d process_noise_enu =
        jacobian * result.process_noise * jacobian.transpose();
    Eigen::Matrix<double, 6, fusion_index::SIZE> navigation_jacobian =
        Eigen::Matrix<double, 6, fusion_index::SIZE>::Zero();
    navigation_jacobian.topRows<3>() = jacobian;
    navigation_jacobian.block<3, 3>(3, fusion_index::VELOCITY).setIdentity();
    const Eigen::Matrix<double, 6, 6> navigation_process_noise_enu =
        navigation_jacobian * result.process_noise * navigation_jacobian.transpose();
    const Eigen::Matrix3d enu_to_ecef =
        ecefToEnuRotation(anchor_lat_rad_, anchor_lon_rad_).transpose();
    const Eigen::Vector3d antenna_velocity_enu =
        state_.nominal.velocity_enu +
        body_to_enu * last_angular_rate_body_.cross(config_.lever_arm_body);
    Eigen::Matrix<double, 6, 6> navigation_rotation =
        Eigen::Matrix<double, 6, 6>::Zero();
    navigation_rotation.topLeftCorner<3, 3>() = enu_to_ecef;
    navigation_rotation.bottomRightCorner<3, 3>() = enu_to_ecef;
    output.antenna_delta_ecef = enu_to_ecef * antenna_delta_enu;
    output.antenna_velocity_ecef = enu_to_ecef * antenna_velocity_enu;
    output.process_noise_ecef =
        enu_to_ecef * process_noise_enu * enu_to_ecef.transpose();
    output.position_velocity_process_noise_ecef =
        navigation_rotation * navigation_process_noise_enu * navigation_rotation.transpose();
    output.velocity_covariance_ecef =
        enu_to_ecef *
        state_.covariance.block<3, 3>(fusion_index::VELOCITY, fusion_index::VELOCITY) *
        enu_to_ecef.transpose();
    output.valid = output.antenna_delta_ecef.allFinite() &&
                   output.antenna_velocity_ecef.allFinite() &&
                   output.process_noise_ecef.allFinite() &&
                   output.position_velocity_process_noise_ecef.allFinite() &&
                   output.velocity_covariance_ecef.allFinite();
    prepared_ = output.valid;
    if (output.valid) ++diagnostics_.supplied_updates;
    return output;
}

bool TightCouplingProcessor::reanchor(
    const Eigen::Vector3d& float_antenna_position_ecef,
    const Eigen::Matrix3d& float_position_covariance_ecef,
    const Eigen::Vector3d& antenna_velocity_ecef,
    const Eigen::Matrix3d& antenna_velocity_covariance_ecef,
    const GNSSTime& time,
    const FusionState* bootstrap_state) {
    if (!float_antenna_position_ecef.allFinite() ||
        !float_position_covariance_ecef.allFinite() || !antenna_velocity_ecef.allFinite() ||
        !antenna_velocity_covariance_ecef.allFinite() ||
        !std::isfinite(time.tow) || (!initialized_ && bootstrap_state == nullptr)) {
        invalidateInterval();
        return false;
    }
    if (!initialized_) state_ = *bootstrap_state;

    double new_lat = 0.0, new_lon = 0.0, height = 0.0;
    ecef2geodetic(float_antenna_position_ecef, new_lat, new_lon, height);
    const Eigen::Matrix3d new_ecef_to_enu = ecefToEnuRotation(new_lat, new_lon);
    if (initialized_) {
        const Eigen::Matrix3d old_enu_to_ecef =
            ecefToEnuRotation(anchor_lat_rad_, anchor_lon_rad_).transpose();
        state_.nominal.attitude_body_to_enu = Eigen::Quaterniond(
            new_ecef_to_enu * old_enu_to_ecef *
            state_.nominal.attitude_body_to_enu.toRotationMatrix()).normalized();
    }
    anchor_lat_rad_ = new_lat;
    anchor_lon_rad_ = new_lon;
    state_.nominal.time = time;
    state_.nominal.velocity_enu = new_ecef_to_enu * antenna_velocity_ecef;
    state_.nominal.position_enu =
        -state_.nominal.attitude_body_to_enu.toRotationMatrix() * config_.lever_arm_body;
    state_.covariance.block<3, fusion_index::SIZE>(
        fusion_index::POSITION, 0).setZero();
    state_.covariance.block<fusion_index::SIZE, 3>(
        0, fusion_index::POSITION).setZero();
    state_.covariance.block<3, 3>(fusion_index::POSITION, fusion_index::POSITION) =
        new_ecef_to_enu * float_position_covariance_ecef * new_ecef_to_enu.transpose();
    if (config_.velocity_state_output_enable) {
        state_.covariance.block<3, fusion_index::SIZE>(
            fusion_index::VELOCITY, 0).setZero();
        state_.covariance.block<fusion_index::SIZE, 3>(
            0, fusion_index::VELOCITY).setZero();
        state_.covariance.block<3, 3>(fusion_index::VELOCITY, fusion_index::VELOCITY) =
            new_ecef_to_enu * antenna_velocity_covariance_ecef * new_ecef_to_enu.transpose();
    }
    state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
    initialized_ = preintegrator_.reset(state_.nominal) == PreintegrationStatus::ACCEPTED;
    prepared_ = false;
    if (initialized_) ++diagnostics_.anchors;
    return initialized_;
}

void TightCouplingProcessor::invalidateInterval() {
    preintegrator_.clear();
    prepared_ = true;
}

}  // namespace libgnss
