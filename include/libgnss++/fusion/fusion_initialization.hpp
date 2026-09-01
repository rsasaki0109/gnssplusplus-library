#pragma once

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <libgnss++/core/types.hpp>
#include <libgnss++/fusion/fusion_types.hpp>
#include <libgnss++/io/imu.hpp>

namespace libgnss {
namespace fusion_initialization {

/**
 * @brief Coarse static leveling + gyro bias estimate from a stationary IMU
 * window.
 *
 * Roll/pitch come from the mean specific-force direction (the body-frame
 * direction that measured the gravity reaction while stationary): the
 * rotation that maps this direction onto ENU's Up axis is the minimal-angle
 * (yaw-free) tilt-only rotation, found via Eigen::Quaterniond::FromTwoVectors
 * -- equivalent in substance to the reference's
 * `pitch = atan2(ax, sqrt(ay^2+az^2))`, `roll = atan2(ay, az)` coarse
 * leveling (reference_notes.md 4.4/7 Step A), expressed without a frame- and
 * gimbal-order-dependent Euler-angle formula. Yaw is left at whatever this
 * minimal rotation implies (i.e. effectively unset/arbitrary) --
 * tryAlignHeading() is expected to correct it once GNSS course is available.
 * Gyro bias is the mean raw gyro over the window (stationary => raw gyro is
 * pure bias). Accel bias is the residual after removing gravity along the
 * sensed direction (reference_notes.md 4.4:
 * `bias_acc = acc_mean - g*(acc_mean/||acc_mean||)`).
 *
 * @param stationary_window_body_flu  IMU samples, already remapped to body FLU
 * @param initial_position_enu        Position to seed (ENU tangent frame)
 * @param gravity_mps2                Local gravity magnitude
 * @return                            Coarse-aligned nominal state
 */
NominalState alignStatic(const std::vector<ImuSample>& stationary_window_body_flu,
                         const Eigen::Vector3d& initial_position_enu,
                         double gravity_mps2 = kStandardGravityMps2);

/**
 * @brief Parameters for the upstream GNSS-velocity attitude initializer.
 *
 * This is the direct, truth-free translation of taroz/gsdc2023's
 * `vel2rpy.m`: a centered moving mean, a three-dimensional speed gate, linear
 * interior/nearest endpoint fill for low-speed headings, and a 180-degree
 * course offset.  It is kept in
 * the shared initialization module so Android and future native callers do
 * not grow subtly different copies of the MATLAB semantics.
 */
struct VelocityHeadingConfig {
    std::size_t smooth_window = 20;
    double velocity_threshold_mps = 0.5;
    double heading_offset_deg = 180.0;
};

struct VelocityHeadingResult {
    bool ok = false;
    std::string error;
    std::vector<Eigen::Vector3d> smoothed_velocity_enu;
    std::vector<Eigen::Vector3d> rpy_rad;
    std::size_t low_speed_count = 0;
    std::size_t linear_fill_count = 0;
    std::size_t nearest_fill_count = 0;
};

/**
 * @brief Convert GNSS ENU velocities to upstream-compatible RPY seeds.
 *
 * Roll and pitch are zero by contract.  Raw `atan2(N,E)` courses are linearly
 * interpolated across interior low-speed samples, nearest-filled only at the
 * endpoints, then offset and wrapped to degrees [-180, 180] before conversion
 * to radians.  Empty/all-low-speed/non-finite input fails closed.
 */
VelocityHeadingResult velocityToRpy(
    const std::vector<Eigen::Vector3d>& velocities_enu,
    const VelocityHeadingConfig& config = VelocityHeadingConfig{});

/**
 * @brief Course-over-ground heading alignment: once GNSS speed exceeds
 * `min_speed_mps`, rotates the current attitude about the ENU Up axis so the
 * body-forward direction matches the GNSS velocity direction, and tightens
 * the corresponding yaw variance to (heading_sigma_deg)^2.
 *
 * No magnetometer, heading unobservable at rest -- same rationale as
 * reference_notes.md 4.4/8.3 (course-based heading init only, never
 * magnetometer-based).
 *
 * @param state             Fusion state to update in place
 * @param gnss_velocity_enu GNSS-derived velocity, ENU, m/s
 * @param min_speed_mps     Minimum horizontal speed required to attempt alignment
 * @param heading_sigma_deg 1-sigma heading uncertainty to assign after alignment
 * @return                  true if alignment was applied (speed gate passed)
 */
bool tryAlignHeading(FusionState& state, const Eigen::Vector3d& gnss_velocity_enu,
                    double min_speed_mps, double heading_sigma_deg);

/**
 * @brief Buffers GNSS course-over-ground samples over a sliding time window
 * and decides when there is enough *consistent* motion to trust a heading
 * latch, instead of trusting a single epoch.
 *
 * Root cause this addresses (PPC nagoya/run1 investigation): the old
 * single-epoch tryAlignHeading() call latched heading on the very first GNSS
 * fix whose speed exceeded threshold. On nagoya/run1 that first qualifying
 * epoch happened during the vehicle *backing out of a parking space* --
 * GNSS course-over-ground is the direction of travel, not the vehicle's nose
 * heading, and during reverse motion those two differ by ~180 deg. The old
 * code had no way to notice this and latched a badly wrong yaw with an
 * artificially tight (5 deg) sigma, from which the filter never recovered
 * (tokyo's runs never hit this because their initial motion happens to be
 * forward). Requiring several seconds of *mutually consistent* course
 * samples before latching does not, by itself, distinguish forward from
 * reverse motion (a sustained reverse maneuver is just as "consistent" as a
 * sustained forward one) -- the complementary fix is
 * LooseCouplingProcessor's post-latch velocity-innovation health check
 * (fusion_processor.hpp Config::heading_recovery_*), which de-latches and
 * lets this tracker re-acquire once real forward motion produces a
 * persistently large velocity residual against the wrong latch.
 *
 * The assigned post-latch yaw sigma is `max(heading_sigma_deg,
 * observed circular scatter)`, so a latch coming from a noisier window is
 * never reported more confident than the data that produced it (contrast
 * with the old code's fixed, optimistic 5 deg regardless of input quality).
 */
class HeadingAlignmentTracker {
public:
    /**
     * @brief Record one GNSS-velocity sample (a no-op if horizontal speed is
     * below min_speed_mps). Samples older than window_s (relative to `time`)
     * are dropped.
     */
    void addSample(const GNSSTime& time, const Eigen::Vector3d& gnss_velocity_enu,
                  double min_speed_mps, double window_s);

    /**
     * @brief True if the buffered window has >= min_samples entries whose
     * circular scatter is <= max_scatter_deg. On success, *mean_course_rad
     * (circular mean, atan2(E,N) convention) and *scatter_deg (circular
     * standard deviation in degrees) are filled in.
     */
    bool ready(int min_samples, double max_scatter_deg, double* mean_course_rad,
              double* scatter_deg) const;

    /** @brief Discard all buffered samples (used by the recovery de-latch). */
    void reset();

    size_t sampleCount() const { return samples_.size(); }

private:
    struct Sample {
        GNSSTime time;
        double course_rad = 0.0;
    };
    std::deque<Sample> samples_;
};

}  // namespace fusion_initialization
}  // namespace libgnss
