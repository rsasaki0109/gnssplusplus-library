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

namespace {

double wrapHeadingDegrees(double degrees) {
    while (degrees > 180.0) degrees -= 360.0;
    while (degrees < -180.0) degrees += 360.0;
    return degrees;
}

}  // namespace

VelocityHeadingResult velocityToRpy(
    const std::vector<Eigen::Vector3d>& velocities_enu,
    const VelocityHeadingConfig& config) {
    VelocityHeadingResult result;
    if (velocities_enu.empty()) {
        result.error = "empty GNSS velocity sequence";
        return result;
    }
    if (config.smooth_window == 0) {
        result.error = "velocity smoothing window must be positive";
        return result;
    }
    if (!std::isfinite(config.velocity_threshold_mps) ||
        config.velocity_threshold_mps < 0.0 ||
        !std::isfinite(config.heading_offset_deg)) {
        result.error = "non-finite or negative velocity-heading parameter";
        return result;
    }
    for (const auto& velocity : velocities_enu) {
        if (!velocity.allFinite()) {
            result.error = "non-finite GNSS velocity";
            return result;
        }
    }

    // MATLAB smoothdata(..., "movmean", k) uses a centered window and shrinks
    // it at both endpoints.  For even k the window is centered about the
    // current and previous elements: k=20 therefore means 10 preceding and 9
    // following samples (the current sample is included in both descriptions).
    const std::size_t samples_before = config.smooth_window / 2;
    const std::size_t samples_after = config.smooth_window - 1 - samples_before;
    result.smoothed_velocity_enu.resize(velocities_enu.size());
    std::vector<bool> valid_heading(velocities_enu.size(), false);
    // Keep the raw atan2 course in degrees until after missing-value filling.
    // This is important at the +/-180 degree branch: MATLAB interpolates the
    // unwrapped atan2 output first and only then applies wrapTo180 to
    // `head+180`.
    std::vector<double> courses_deg(velocities_enu.size(), 0.0);
    for (std::size_t i = 0; i < velocities_enu.size(); ++i) {
        const std::size_t first = i > samples_before ? i - samples_before : 0;
        const std::size_t last = std::min(
            velocities_enu.size() - 1,
            i + samples_after);
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (std::size_t j = first; j <= last; ++j) sum += velocities_enu[j];
        result.smoothed_velocity_enu[i] =
            sum / static_cast<double>(last - first + 1);

        const double speed = result.smoothed_velocity_enu[i].norm();
        if (speed >= config.velocity_threshold_mps) {
            courses_deg[i] = std::atan2(
                result.smoothed_velocity_enu[i].y(),
                result.smoothed_velocity_enu[i].x()) * 180.0 / M_PI;
            valid_heading[i] = std::isfinite(courses_deg[i]);
        } else {
            ++result.low_speed_count;
        }
    }

    std::vector<std::size_t> valid_indices;
    valid_indices.reserve(velocities_enu.size());
    for (std::size_t i = 0; i < valid_heading.size(); ++i) {
        if (valid_heading[i]) valid_indices.push_back(i);
    }
    if (valid_indices.empty()) {
        result.error = "all smoothed GNSS velocities are below heading threshold";
        return result;
    }

    result.rpy_rad.resize(velocities_enu.size(), Eigen::Vector3d::Zero());
    for (std::size_t i = 0; i < velocities_enu.size(); ++i) {
        double course_deg = courses_deg[i];
        if (!valid_heading[i]) {
            const auto upper = std::lower_bound(valid_indices.begin(), valid_indices.end(), i);
            if (upper != valid_indices.begin() && upper != valid_indices.end()) {
                // The two-stage upstream contract is linear interior fill
                // followed by nearest endpoint fill.  Do not wrap the course
                // before interpolation (e.g. +179/-179 must pass through 0).
                const std::size_t previous = *(upper - 1);
                const std::size_t next = *upper;
                const double fraction = static_cast<double>(i - previous) /
                                        static_cast<double>(next - previous);
                course_deg = courses_deg[previous] +
                             fraction * (courses_deg[next] - courses_deg[previous]);
                ++result.linear_fill_count;
            } else {
                // Leading/trailing gaps have no pair of brackets and are
                // filled from the nearest finite course.  A preceding sample
                // wins only in the (non-endpoint) tie case; endpoints are
                // unambiguous here.
                const std::size_t source = upper == valid_indices.end()
                                               ? valid_indices.back()
                                               : valid_indices.front();
                course_deg = courses_deg[source];
                ++result.nearest_fill_count;
            }
        }
        result.rpy_rad[i] = Eigen::Vector3d(
            0.0, 0.0,
            wrapHeadingDegrees(course_deg + config.heading_offset_deg) * M_PI / 180.0);
    }

    result.ok = true;
    return result;
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
