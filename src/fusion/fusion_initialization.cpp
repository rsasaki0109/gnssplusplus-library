#include <libgnss++/fusion/fusion_initialization.hpp>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace libgnss {
namespace fusion_initialization {

NominalState alignStatic(const std::vector<ImuSample>& stationary_window_body_flu,
                         const Eigen::Vector3d& initial_position_enu,
                         double gravity_mps2) {
    NominalState state;
    state.position_enu = initial_position_enu;
    state.velocity_enu.setZero();

    if (stationary_window_body_flu.empty()) {
        return state;
    }

    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    for (const auto& sample : stationary_window_body_flu) {
        accel_sum += sample.accel_raw;
        gyro_sum += sample.gyro_raw_radps;
    }
    const double count = static_cast<double>(stationary_window_body_flu.size());
    const Eigen::Vector3d accel_mean = accel_sum / count;
    const Eigen::Vector3d gyro_mean = gyro_sum / count;

    state.time = stationary_window_body_flu.back().time;
    state.gyro_bias = gyro_mean;

    const double accel_mean_norm = accel_mean.norm();
    if (accel_mean_norm > 1e-9) {
        const Eigen::Vector3d up_body = accel_mean / accel_mean_norm;
        state.attitude_body_to_enu =
            Eigen::Quaterniond::FromTwoVectors(up_body, Eigen::Vector3d::UnitZ()).normalized();
        state.accel_bias = accel_mean - gravity_mps2 * up_body;
    } else {
        state.attitude_body_to_enu = Eigen::Quaterniond::Identity();
        state.accel_bias.setZero();
    }

    return state;
}

bool tryAlignHeading(FusionState& state, const Eigen::Vector3d& gnss_velocity_enu,
                    double min_speed_mps, double heading_sigma_deg) {
    const double horizontal_speed =
        std::sqrt(gnss_velocity_enu.x() * gnss_velocity_enu.x() +
                 gnss_velocity_enu.y() * gnss_velocity_enu.y());
    if (horizontal_speed < min_speed_mps) {
        return false;
    }

    const double target_heading = std::atan2(gnss_velocity_enu.x(), gnss_velocity_enu.y());
    const Eigen::Vector3d forward_enu = state.nominal.attitude_body_to_enu * Eigen::Vector3d::UnitX();
    const double current_heading = std::atan2(forward_enu.x(), forward_enu.y());
    // A rotation Rz(theta) applied to a vector v (heading(v) := atan2(v.x, v.y))
    // shifts heading(v) by -theta (heading(Rz(theta)*v) == heading(v) - theta),
    // so the correction that lands exactly on target_heading is
    // theta = current_heading - target_heading, not target - current.
    const double delta_yaw = current_heading - target_heading;

    const Eigen::Quaterniond yaw_correction(Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()));
    state.nominal.attitude_body_to_enu =
        (yaw_correction * state.nominal.attitude_body_to_enu).normalized();

    // Bug fix (found validating docs/design.md's Doppler-velocity task on
    // PPC tokyo/run1): this used to overwrite only the yaw *diagonal*
    // variance, leaving every yaw/other-state *off-diagonal* covariance
    // entry untouched. Those cross-terms had accumulated against the huge
    // pre-alignment yaw variance (~(180 deg)^2), so slashing the diagonal
    // alone by ~4 orders of magnitude (down to heading_sigma_deg^2, default
    // (5 deg)^2) left a covariance matrix whose implied yaw/other-state
    // correlation coefficients exceed 1 in magnitude -- not a valid
    // covariance (not positive semi-definite). Every subsequent Kalman
    // update (position or velocity) then solved against this inconsistent
    // matrix and produced wild, oscillating corrections (observed: yaw
    // swinging over the full 0-360 deg range epoch to epoch instead of
    // settling near the true course). The correct reset zeroes the entire
    // yaw row/column first -- treating the aligned heading as a fresh,
    // independent measurement that supersedes any prior correlation -- then
    // sets only the new (small) yaw variance.
    const double heading_sigma_rad = heading_sigma_deg * M_PI / 180.0;
    state.covariance.row(fusion_index::ATTITUDE + 2).setZero();
    state.covariance.col(fusion_index::ATTITUDE + 2).setZero();
    state.covariance(fusion_index::ATTITUDE + 2, fusion_index::ATTITUDE + 2) =
        heading_sigma_rad * heading_sigma_rad;

    return true;
}

void HeadingAlignmentTracker::addSample(const GNSSTime& time, const Eigen::Vector3d& gnss_velocity_enu,
                                        double min_speed_mps, double window_s) {
    const double horizontal_speed =
        std::sqrt(gnss_velocity_enu.x() * gnss_velocity_enu.x() +
                 gnss_velocity_enu.y() * gnss_velocity_enu.y());
    if (horizontal_speed >= min_speed_mps) {
        Sample sample;
        sample.time = time;
        sample.course_rad = std::atan2(gnss_velocity_enu.x(), gnss_velocity_enu.y());
        samples_.push_back(sample);
    }

    while (!samples_.empty() && (time - samples_.front().time) > window_s) {
        samples_.pop_front();
    }
}

bool HeadingAlignmentTracker::ready(int min_samples, double max_scatter_deg, double* mean_course_rad,
                                    double* scatter_deg) const {
    if (min_samples < 1) {
        min_samples = 1;
    }
    if (static_cast<int>(samples_.size()) < min_samples) {
        return false;
    }

    // Circular mean/scatter: average the unit vectors of each course sample
    // rather than the angles themselves (a plain arithmetic mean breaks
    // across the +-180 deg wrap). The mean resultant length R in [0, 1]
    // measures concentration (R -> 1 for tightly clustered courses, R -> 0
    // for uniformly scattered ones); the standard circular-standard-
    // deviation estimator sqrt(-2 ln R) converges to the ordinary linear
    // standard deviation for small scatter, which is what makes it usable
    // directly as a degrees-equivalent sigma alongside heading_sigma_deg.
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for (const auto& sample : samples_) {
        sum_sin += std::sin(sample.course_rad);
        sum_cos += std::cos(sample.course_rad);
    }
    const double n = static_cast<double>(samples_.size());
    const double mean_sin = sum_sin / n;
    const double mean_cos = sum_cos / n;
    const double resultant_length = std::sqrt(mean_sin * mean_sin + mean_cos * mean_cos);
    const double mean_course = std::atan2(mean_sin, mean_cos);

    const double clamped_r = std::min(1.0, std::max(1e-12, resultant_length));
    const double scatter_rad = std::sqrt(std::max(0.0, -2.0 * std::log(clamped_r)));
    const double this_scatter_deg = scatter_rad * 180.0 / M_PI;

    if (mean_course_rad != nullptr) {
        *mean_course_rad = mean_course;
    }
    if (scatter_deg != nullptr) {
        *scatter_deg = this_scatter_deg;
    }

    return this_scatter_deg <= max_scatter_deg;
}

void HeadingAlignmentTracker::reset() { samples_.clear(); }

}  // namespace fusion_initialization
}  // namespace libgnss
