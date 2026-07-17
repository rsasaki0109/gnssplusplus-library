#pragma once

#include <deque>
#include <vector>

#include <Eigen/Dense>

#include <libgnss++/core/solution.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>
#include <libgnss++/fusion/fusion_measurement.hpp>
#include <libgnss++/fusion/fusion_process_noise.hpp>
#include <libgnss++/fusion/fusion_types.hpp>
#include <libgnss++/fusion/fusion_update.hpp>
#include <libgnss++/io/imu.hpp>

namespace libgnss {

/**
 * @brief The "loose Kalman bridge" (docs/design.md 1.2, 3): a 15-state
 * error-state EKF that mechanizes 100 Hz IMU samples and opportunistically
 * corrects with whatever the library's existing SPP/RTK processors emit
 * (PositionSolution), i.e. this class sits DOWNSTREAM of
 * SPPProcessor/RTKProcessor rather than re-parsing RINEX itself.
 *
 * The local ENU tangent frame's origin is defined lazily, by the first
 * processGnssSolution() call that provides a usable position: that solution's
 * ECEF position/geodetic lat-lon becomes the frame origin (position_enu =
 * (0,0,0) there), and every later state/measurement is expressed relative to
 * it (flat-Earth simplification, docs/design.md 3.1 -- appropriate for the
 * PPC-Dataset's regional/urban vehicle runs, not for a global-scale run).
 */
class LooseCouplingProcessor {
public:
    struct Config {
        ProcessNoiseConfig process_noise;
        Eigen::Vector3d lever_arm_body = Eigen::Vector3d::Zero();  ///< IMU -> antenna, body FLU

        bool zupt_enable = true;
        double zupt_sigma_mps = 0.5;
        double zupt_max_accel_std = 0.55;
        double zupt_max_gyro_std = 0.030;
        double zupt_max_gyro_median = 0.020;

        bool nhc_enable = false;  ///< off unless platform == vehicle (docs/design.md 3.8)
        double nhc_sigma_lateral_mps = 0.3;
        double nhc_sigma_vertical_mps = 0.2;

        // Alignment.
        double align_static_window_s = 2.0;
        double align_velocity_threshold_mps = 1.0;
        // Heading is latched only once >= align_heading_min_samples GNSS
        // fixes above align_velocity_threshold_mps, all within the trailing
        // align_heading_window_s seconds, agree on course to within
        // align_heading_max_course_scatter_deg (circular standard
        // deviation) -- see fusion_initialization::HeadingAlignmentTracker.
        // A single noisy/unrepresentative epoch (e.g. Doppler picked up
        // during a brief reverse maneuver) can no longer latch heading on
        // its own the way the old one-shot check did.
        double align_heading_window_s = 2.0;
        double align_heading_sigma_deg = 5.0;
        int align_heading_min_samples = 3;
        double align_heading_max_course_scatter_deg = 10.0;

        // Post-alignment recovery: if the recent velocity-update NIS-per-
        // observation (an exponential moving average, reset whenever
        // recovery fires) stays above heading_recovery_nis_threshold for
        // heading_recovery_min_bad_epochs consecutive velocity updates, the
        // current yaw estimate is judged wrong (or stale) and its
        // covariance is re-inflated back to the initial "unobservable"
        // prior -- the nominal attitude itself and the latched/aligned
        // state are left untouched, so the ordinary per-epoch GNSS
        // position/velocity Kalman updates (which already track a turning
        // heading continuously via their normal gain, once given enough
        // covariance to work with) pull the estimate back toward truth,
        // rather than this class forcing a discontinuous override.
        //
        // An earlier version instead tore down heading_aligned_ and forced
        // a fresh HeadingAlignmentTracker re-latch on recovery. Validating
        // on PPC nagoya/run1 (a route with near-continuous turning soon
        // after its initial mis-latch, from a vehicle backing out of a
        // parking space) showed that approach thrashes: the vehicle's
        // heading keeps changing throughout the maneuver, so any freshly
        // re-latched course goes stale again within a few seconds,
        // triggering another hard override on repeat -- each one discarding
        // whatever the gyro-mechanized attitude had already tracked
        // correctly through the turn. Covariance-only inflation avoids that
        // discontinuity while still being what recovers from a latch that
        // passed the multi-epoch consistency check above but was still
        // wrong (e.g. the sustained reverse-out-of-parking course), once
        // genuine forward motion starts disagreeing with it. <= 0 for
        // either knob disables recovery (matches the pre-existing
        // behavior: a latch, once made, is permanent).
        //
        // Disabled (min_bad_epochs=0) by default: PPC nagoya/run1 validation
        // showed the velocity-update NIS, with a nonzero lever arm on a
        // route with frequent turning, is not a reliable per-epoch signal
        // of "yaw is wrong" specifically -- ordinary turning dynamics keep
        // it elevated too (lever-arm x angular-rate coupling amplifies any
        // small residual attitude/IMU/lever-arm-calibration error during
        // every turn, not just a genuinely mislatched yaw), so any
        // threshold/patience tuned tight enough to catch the initial bad
        // latch also fires repeatedly during normal driving and net
        // *hurts* accuracy (empirically: worse roll/pitch/yaw RMSE and
        // H-RMSE than leaving it off). The multi-epoch consistency gate
        // above (align_heading_*) already fixes the dominant failure mode
        // (a single bad epoch permanently mislatching heading); this knob
        // is kept, config-gated off by default, for datasets/tunings where
        // it can be made to help (see the recovery integration test in
        // test_fusion_processor_synthetic.cpp for the mechanism working as
        // intended on a controlled synthetic case).
        double heading_recovery_nis_threshold = 30.0;
        int heading_recovery_min_bad_epochs = 0;
        // Cooldown (in velocity-update epochs) after a recovery fires
        // before it is allowed to fire again. Without this, repeatedly
        // inflating yaw covariance during a stretch where yaw is genuinely
        // unobservable (e.g. constant-velocity cruise, no turning/
        // accelerating) gives each epoch's noisy measurement more freedom
        // to perturb yaw with no corrective information behind it -- an
        // unbounded random walk that gets *worse* over time, not better
        // (observed on the synthetic recovery test when extended well
        // past its acceleration phase into a long cruise). The cooldown
        // caps how often recovery can re-arm, bounding that walk.
        int heading_recovery_cooldown_epochs = 20;

        // NIS gates (<= 0 disables). Applied per fusion_update::applyDenseUpdate.
        double max_position_update_nis_per_observation = 0.0;
        double max_velocity_update_nis_per_observation = 0.0;

        // Recovery from a gate "lockout spiral": a hard NIS gate is only
        // safe against an *isolated* bad epoch. If the true state has
        // drifted (e.g. through a long degraded-RTK segment) far enough
        // that even the next *correct* GNSS fix looks like an outlier
        // relative to the (by-then-wrong) prediction, a naive gate rejects
        // it too -- and every subsequent fix looks even more like an
        // outlier as dead-reckoning error keeps growing, so the filter can
        // never resynchronize (found validating docs/design.md's Doppler
        // velocity task on PPC tokyo/run1: any single fixed NIS threshold
        // either let a genuine bad-geometry/float-reconvergence fix corrupt
        // attitude, or -- once tightened enough to block it -- locked the
        // filter out of every later correction for the rest of the run).
        // After this many *consecutive* gate rejections on the same channel
        // (position or velocity, tracked independently), force-accept the
        // next update unconditionally to break the spiral. <= 0 disables
        // (gate rejections are then permanent, matching the pre-existing
        // behavior).
        int max_consecutive_gate_rejections = 5;
    };

    explicit LooseCouplingProcessor(const Config& config);

    /** @brief 100 Hz mechanization + process-noise propagation + ZUPT/NHC gating. */
    void processImuSample(const ImuSample& sample_body_flu);

    /** @brief Opportunistic GNSS position/velocity update. */
    void processGnssSolution(const PositionSolution& solution);

    const FusionState& state() const { return state_; }

    bool isInitialized() const { return initialized_; }
    bool isOriginSet() const { return origin_set_; }
    /** @brief True once a heading latch has been made (see class doc: this alone does NOT mean the
     *  latch is trustworthy -- use isHeadingConverged() for a real health signal). */
    bool isHeadingAligned() const { return heading_aligned_; }
    /**
     * @brief Real heading-health metric, replacing the old always-"aligned"
     * flag: true only while heading_aligned_ AND the recent velocity-update
     * innovation (NIS-per-observation EMA) stays below
     * Config::heading_recovery_nis_threshold, i.e. the current yaw latch is
     * still being corroborated by GNSS velocity, not just "a latch happened
     * at some point in the past."
     */
    bool isHeadingConverged() const {
        return heading_aligned_ &&
               (config_.heading_recovery_nis_threshold <= 0.0 ||
                velocity_nis_ema_ <= config_.heading_recovery_nis_threshold);
    }

    /** @brief Convert the fused ENU state back into the library's PositionSolution shape. */
    PositionSolution toPositionSolution() const;

    /**
     * @brief Phase 1 GNSS/IMU coupling (docs/design.md): current best-
     * estimate ANTENNA-frame ECEF position + covariance, lever-arm
     * compensated, for feeding into e.g. RTKProcessor::
     * setExternalPositionPrior() ahead of a KINEMATIC epoch's time update.
     * Uses the same h(x) = position_enu + R*lever_arm model as
     * fusion_measurement::buildGnssPositionUpdate(), so the reported
     * covariance is the position update's own H P H^T (projecting [dp;
     * dtheta] through the lever-arm Jacobian), not just the raw POSITION
     * error-state block -- i.e. it already reflects attitude uncertainty's
     * contribution via the lever arm.
     *
     * Returns false (leaving ecef_pos/ecef_cov untouched) when the filter
     * has no ECEF anchor yet (isOriginSet() == false).
     */
    bool predictedAntennaPositionEcef(Eigen::Vector3d& ecef_pos, Eigen::Matrix3d& ecef_cov) const;

private:
    Config config_;
    FusionState state_;
    Eigen::Matrix<double, 15, 1> error_state_ = Eigen::Matrix<double, 15, 1>::Zero();
    bool initialized_ = false;
    bool origin_set_ = false;
    bool heading_aligned_ = false;

    Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    double origin_height_ = 0.0;

    std::vector<ImuSample> static_window_;
    std::deque<ImuSample> zupt_window_;
    Eigen::Vector3d last_angular_rate_body_ = Eigen::Vector3d::Zero();

    int position_consecutive_gate_rejections_ = 0;
    int velocity_consecutive_gate_rejections_ = 0;

    // Robust heading alignment + post-latch health/recovery (see Config's
    // align_heading_*/heading_recovery_* doc comments and
    // fusion_initialization::HeadingAlignmentTracker).
    fusion_initialization::HeadingAlignmentTracker heading_tracker_;
    double velocity_nis_ema_ = 0.0;
    int consecutive_bad_heading_epochs_ = 0;
    int recovery_cooldown_remaining_epochs_ = 0;

    void initializeFromStaticWindow();
    bool detectStationary() const;
    fusion_update::FusionUpdateResult applyUpdateAndInject(
        const fusion_measurement::FusionMeasurementSystem& system, double max_nis_per_observation,
        int& consecutive_gate_rejections);
    void injectAndReset();
    void updateHeadingHealthAndMaybeRecover(const fusion_update::FusionUpdateResult& velocity_result);
    void inflateYawUncertainty();
};

}  // namespace libgnss
