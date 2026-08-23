#pragma once

#include <deque>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <libgnss++/core/solution.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>
#include <libgnss++/fusion/dd_imu_bridge.hpp>
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
        // Suppress a stationary-IMU ZUPT after a GNSS Doppler velocity proves
        // the vehicle is moving.  The moving latch remains in force until a
        // later low-speed GNSS velocity releases it; when no GNSS velocity
        // has ever been available, the legacy detector remains enabled.
        bool zupt_gnss_speed_gate_enable = true;
        double zupt_gnss_speed_gate_threshold_mps = 0.5;

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
        // Carrier DD updates remain shadow-only by default: PPC validation
        // found apparently accepted carrier updates could make the augmented
        // covariance indefinite. Enable only for explicit research ablation.
        bool tight_dd_commit_carrier_updates = false;
        // SSE-PAR external-solution contract: do not feed FLOAT positions
        // back into the independent INS predictor.
        bool position_updates_require_fixed = false;
        double tight_dd_sse_fixed_anchor_max_age_s = 2.0;
        double tight_dd_sse_fixed_anchor_max_nis_per_observation = 10.0;

        // Recovery from a gate "lockout spiral". After this many consecutive
        // position-gate rejections, a trusted FIXED position may perform a
        // deterministic position-only re-anchor. The re-anchor is deliberately
        // not an ungated EKF update: it does not apply to FLOAT/SPP solutions,
        // velocity updates, attitude, or bias states. <= 0 disables this
        // recovery (gate rejections then remain rejected). The application
        // policy uses 30 consecutive FIXED epochs by default; the longer
        // patience is the trust condition for the unbounded default target.
        int max_consecutive_gate_rejections = 30;
        // A trusted FIXED re-anchor may optionally be bounded in local ENU
        // metres by setting a finite positive value. The default +infinity
        // deliberately has no arbitrary distance cap: after the FIXED
        // patience gate, a real outage can leave the inertial position tens
        // or hundreds of metres away even when the returning FIX is accurate.
        // <= 0 disables re-anchoring; a finite value remains available for
        // applications with an independent maximum-jump contract.
        double max_fixed_position_reanchor_m =
            std::numeric_limits<double>::infinity();

        // Recovery from a velocity-gate lockout. After this many consecutive
        // GNSS velocity rejections, a finite/PSD Doppler solution may perform
        // a velocity-only re-anchor. This is deliberately independent of the
        // position patience above: the velocity state is reset to the
        // lever-arm-compensated antenna velocity, while position, attitude,
        // and biases remain untouched. <= 0 disables this recovery.
        int max_consecutive_velocity_gate_rejections = 3;
        // Maximum norm of a velocity-only re-anchor correction in m/s. <= 0
        // disables re-anchoring; a large correction is treated as untrusted.
        double max_gnss_velocity_reanchor_mps = 20.0;
    };

    explicit LooseCouplingProcessor(const Config& config);

    /** @brief 100 Hz mechanization + process-noise propagation + ZUPT/NHC gating. */
    void processImuSample(const ImuSample& sample_body_flu);

    /** @brief Opportunistic GNSS position/velocity update. */
    void processGnssSolution(const PositionSolution& solution);

    struct TightlyCoupledDDResult {
        dd_imu_bridge::UpdateResult update;
        dd_imu_bridge::PartialARResult partial_ar;
        dd_imu_bridge::SSEPartialARResult sse_partial_ar;
        dd_imu_bridge::SoftResetAction reset_action =
            dd_imu_bridge::SoftResetAction::REJECTED;
    };

    /** Apply real DD code/carrier rows to the live INS state and ambiguities.
     * The optional position solution is used only for innovation-gated soft
     * recovery; a valid propagated nominal state is never hard-overwritten.
     */
    TightlyCoupledDDResult processTightlyCoupledDD(
        const std::vector<dd_imu_bridge::DDObservation>& observations,
        const PositionSolution* recovery_solution = nullptr);

    const FusionState& state() const { return state_; }

    bool isInitialized() const { return initialized_; }
    bool isOriginSet() const { return origin_set_; }
    bool hasHealthyFixedAnchor() const;
    Eigen::Matrix3d ecefToLocalEnuRotation() const;
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
     * @brief Convert the fused state to an antenna-frame PositionSolution.
     *
     * Unlike toPositionSolution(), whose position and velocity are the IMU
     * origin by contract, this output applies the configured body-FLU
     * IMU-to-antenna lever arm.  Position uses p + R*r and velocity uses
     * v + R*(omega x r), where omega is the latest bias-corrected gyro
     * sample.  Both reported covariances are H*P*H^T using the same
     * lever-arm Jacobians as the GNSS position/velocity measurement models.
     * The method is intended for external fused .pos/KML products; internal
     * propagated-solution consumers must continue to use toPositionSolution().
     */
    PositionSolution toAntennaPositionSolution() const;

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

    /**
     * @brief Phase 1b GNSS/IMU coupling (docs/imu_fusion.md): the same
     * velocity-update NIS-per-observation exponential moving average that
     * backs isHeadingConverged()'s health check, exposed directly so a
     * caller (e.g. gnss_fuse.cpp's INS position-prior injection) can gate on
     * a tighter/looser threshold than heading_recovery_nis_threshold without
     * this class needing to know about that caller's policy. 0.0 before the
     * first velocity update (i.e. reads as "healthy" until proven otherwise,
     * matching isHeadingConverged()'s own treatment of the EMA).
     */
    double getVelocityNisEma() const { return velocity_nis_ema_; }

    /** Position-state injection from the most recent accepted GNSS position
     * update, before any same-epoch velocity update. Used by offline
     * GNSS/IMU time-offset scoring; false means no position update applied.
     */
    bool lastGnssPositionUpdateApplied() const {
        return last_gnss_position_update_applied_;
    }
    const Eigen::Vector3d& lastGnssPositionCorrectionEnu() const {
        return last_gnss_position_correction_enu_;
    }
    /** True when the last position correction was a fixed-solution re-anchor. */
    bool lastGnssPositionReanchored() const { return last_gnss_position_reanchored_; }
    /** True when the last velocity correction was a bounded Doppler re-anchor. */
    bool lastGnssVelocityReanchored() const {
        return last_gnss_velocity_reanchored_;
    }
    const Eigen::Vector3d& lastGnssVelocityCorrectionEnu() const {
        return last_gnss_velocity_correction_enu_;
    }

    /** Number of applied loose-coupling ZUPT updates (diagnostic). */
    std::size_t zuptUpdateCount() const { return zupt_updates_; }

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
    bool has_gnss_velocity_ = false;
    double last_gnss_velocity_speed_mps_ = 0.0;
    std::size_t zupt_updates_ = 0;

    int position_consecutive_gate_rejections_ = 0;
    int velocity_consecutive_gate_rejections_ = 0;
    bool last_gnss_position_update_applied_ = false;
    bool last_gnss_position_reanchored_ = false;
    bool last_gnss_velocity_reanchored_ = false;
    Eigen::Vector3d last_gnss_position_correction_enu_ =
        Eigen::Vector3d::Zero();
    Eigen::Vector3d last_gnss_velocity_correction_enu_ =
        Eigen::Vector3d::Zero();
    std::unique_ptr<dd_imu_bridge::DDIMUBridge> dd_imu_bridge_;
    Eigen::Matrix<double, 15, 15> dd_transition_since_update_ =
        Eigen::Matrix<double, 15, 15>::Identity();

    // Robust heading alignment + post-latch health/recovery (see Config's
    // align_heading_*/heading_recovery_* doc comments and
    // fusion_initialization::HeadingAlignmentTracker).
    fusion_initialization::HeadingAlignmentTracker heading_tracker_;
    double velocity_nis_ema_ = 0.0;
    int consecutive_bad_heading_epochs_ = 0;
    int recovery_cooldown_remaining_epochs_ = 0;
    bool has_fixed_anchor_ = false;
    GNSSTime last_fixed_anchor_time_;
    double last_fixed_anchor_nis_per_observation_ =
        std::numeric_limits<double>::infinity();

    void initializeFromStaticWindow();
    bool detectStationary() const;
    bool gnssSpeedGateAllowsZupt() const;
    fusion_update::FusionUpdateResult applyUpdateAndInject(
        const fusion_measurement::FusionMeasurementSystem& system, double max_nis_per_observation,
        int& consecutive_gate_rejections);
    bool reanchorPositionFromFixedSolution(const Eigen::Vector3d& antenna_position_enu,
                                           const Eigen::Matrix3d& position_covariance_enu);
    bool reanchorVelocityFromGnssSolution(
        const Eigen::Vector3d& antenna_velocity_enu,
        const Eigen::Matrix3d& velocity_covariance_enu);
    void injectAndReset();
    void updateHeadingHealthAndMaybeRecover(const fusion_update::FusionUpdateResult& velocity_result);
    void inflateYawUncertainty();
};

}  // namespace libgnss
