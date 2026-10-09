#include <libgnss++/fusion/fusion_processor.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/fusion/attitude.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>
#include <libgnss++/fusion/fusion_update.hpp>
#include <libgnss++/fusion/mechanization.hpp>

namespace libgnss {

namespace {
// Opt-in diagnostic (set LIBGNSS_DEBUG_HEADING=1 in the environment) that
// logs heading-latch and recovery events to stderr: added during the PPC
// nagoya/run1 investigation to see exactly when/why the heading estimate
// latches or gets recovered, and kept because reconstructing that sequence
// after the fact (from the fused .pos/attitude-csv alone) is otherwise slow.
// No effect on filter behavior or output when unset.
bool debugHeadingEnabled() {
    static const bool enabled = std::getenv("LIBGNSS_DEBUG_HEADING") != nullptr;
    return enabled;
}

constexpr size_t kZuptWindowSize = 20;      // 0.2 s at 100 Hz; reference's zupt_min_samples=5 is the floor.
constexpr double kDefaultPositionSigmaM = 5.0;
constexpr double kDefaultVelocitySigmaMps = 0.5;
constexpr double kInitialPositionSigmaM = 1.0;
constexpr double kInitialVelocitySigmaMps = 0.1;
constexpr double kInitialTiltSigmaDeg = 1.0;
constexpr double kInitialYawSigmaDeg = 180.0;
constexpr double kInitialAccelBiasSigmaMps2 = 0.01;
constexpr double kInitialGyroBiasSigmaRadps = 0.01;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Linear map ECEF-difference -> ENU, expressed as a rotation matrix (same
// coefficients as coordinates.hpp's ecef2enu()/enu2ecef(), just materialized
// so covariances can be rotated with a single congruence transform).
Eigen::Matrix3d ecefToEnuRotation(double lat, double lon) {
    const double sinlat = std::sin(lat), coslat = std::cos(lat);
    const double sinlon = std::sin(lon), coslon = std::cos(lon);
    Eigen::Matrix3d r;
    r << -sinlon, coslon, 0.0,
         -sinlat * coslon, -sinlat * sinlon, coslat,
          coslat * coslon, coslat * sinlon, sinlat;
    return r;
}

double standardDeviation(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    double mean = 0.0;
    for (double v : values) mean += v;
    mean /= static_cast<double>(values.size());
    double variance = 0.0;
    for (double v : values) variance += (v - mean) * (v - mean);
    variance /= static_cast<double>(values.size());
    return std::sqrt(variance);
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const size_t n = values.size();
    return (n % 2 == 0) ? 0.5 * (values[n / 2 - 1] + values[n / 2]) : values[n / 2];
}

// Real-data hardening (docs/design.md task-B validation on PPC tokyo/run1
// surfaced this as a live bug, not a hypothetical): RTKProcessor's
// PositionSolution::position_covariance/velocity_covariance is a congruence
// transform of an internal DD Kalman marginal, ECEF -> ENU rotated here.
// Under aggressive kinematic tuning (e.g. the low-cost preset) that marginal
// is occasionally reported with a near-zero eigenvalue in one axis while
// staying nominally "valid" by the old trace()>0.0 check -- i.e. it passes
// the old guard but is numerically singular/ill-conditioned. Feeding that
// straight into fusion_update::applyDenseUpdate() as the update's R can make
// the innovation covariance S = H P H^T + R near-singular, so LDLT::solve()
// returns a finite but astronomically large gain/correction on the very
// next update (observed: a single bad epoch made the ESKF's ECEF position
// jump by 10^9 m within ~1 s, then "coast" at that bogus constant velocity
// for the rest of the run, since no later measurement could out-vote a
// covariance that far gone). Regularize with the same
// symmetrize-then-eigenvalue-floor recipe RTKProcessor::lambdaMethod already
// uses for its own covariance inputs (src/algorithms/rtk.cpp lambdaMethod)
// rather than only checking the trace.
constexpr double kMinCovarianceEigenvalue = 1e-6;  // (1 mm)^2 floor, matches rtk.cpp's MIN_VAR

Eigen::Matrix3d regularizeCovariance3x3(const Eigen::Matrix3d& raw, double default_sigma) {
    const Eigen::Matrix3d fallback = (default_sigma * default_sigma) * Eigen::Matrix3d::Identity();
    if (!raw.allFinite()) {
        return fallback;
    }
    Eigen::Matrix3d sym = 0.5 * (raw + raw.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(sym);
    if (eig.info() != Eigen::Success) {
        return fallback;
    }
    const double min_eig = eig.eigenvalues().minCoeff();
    if (!std::isfinite(min_eig)) {
        return fallback;
    }
    if (min_eig < kMinCovarianceEigenvalue) {
        sym.diagonal().array() += (kMinCovarianceEigenvalue - min_eig);
    }
    return sym;
}

}  // namespace

LooseCouplingProcessor::LooseCouplingProcessor(const Config& config) : config_(config) {}

void LooseCouplingProcessor::initializeFromStaticWindow() {
    state_.nominal =
        fusion_initialization::alignStatic(static_window_, Eigen::Vector3d::Zero(), kStandardGravityMps2);
    // velocity_consistency_v6: a pending seed replaces only the window-mean
    // gyro bias. alignStatic's attitude and accel bias depend on the mean
    // specific force alone, and the covariances below are unchanged.
    last_init_gyro_bias_seeded_ = false;
    if (pending_gyro_bias_seed_) {
        last_init_window_gyro_bias_ = state_.nominal.gyro_bias;
        state_.nominal.gyro_bias = *pending_gyro_bias_seed_;
        last_init_gyro_bias_seeded_ = true;
        if (debugHeadingEnabled()) {
            const auto& seed = state_.nominal.gyro_bias;
            const auto& mean = last_init_window_gyro_bias_;
            std::cerr << "[INIT] tow=" << state_.nominal.time.tow << " seeded=1 seed=("
                      << seed.x() << "," << seed.y() << "," << seed.z() << ") window_mean=("
                      << mean.x() << "," << mean.y() << "," << mean.z() << ")\n";
        }
        pending_gyro_bias_seed_.reset();
    }

    state_.covariance.setZero();
    state_.covariance.block<3, 3>(fusion_index::POSITION, fusion_index::POSITION) =
        (kInitialPositionSigmaM * kInitialPositionSigmaM) * Eigen::Matrix3d::Identity();
    state_.covariance.block<3, 3>(fusion_index::VELOCITY, fusion_index::VELOCITY) =
        (kInitialVelocitySigmaMps * kInitialVelocitySigmaMps) * Eigen::Matrix3d::Identity();
    const double tilt_sigma_rad = kInitialTiltSigmaDeg * M_PI / 180.0;
    const double yaw_sigma_rad = kInitialYawSigmaDeg * M_PI / 180.0;
    state_.covariance.block<3, 3>(fusion_index::ATTITUDE, fusion_index::ATTITUDE) =
        (tilt_sigma_rad * tilt_sigma_rad) * Eigen::Matrix3d::Identity();
    state_.covariance(fusion_index::ATTITUDE + 2, fusion_index::ATTITUDE + 2) =
        yaw_sigma_rad * yaw_sigma_rad;
    state_.covariance.block<3, 3>(fusion_index::ACCEL_BIAS, fusion_index::ACCEL_BIAS) =
        (kInitialAccelBiasSigmaMps2 * kInitialAccelBiasSigmaMps2) * Eigen::Matrix3d::Identity();
    state_.covariance.block<3, 3>(fusion_index::GYRO_BIAS, fusion_index::GYRO_BIAS) =
        (kInitialGyroBiasSigmaRadps * kInitialGyroBiasSigmaRadps) * Eigen::Matrix3d::Identity();

    error_state_.setZero();
    initialized_ = true;
    static_window_.clear();
    // The static-alignment window is not evidence of rest after a reset that
    // happens while moving: v_long is invalid until the stationarity
    // condition first holds (Config::heading_latch_direction_test).
    v_long_ = 0.0;
    v_long_valid_ = false;
}

bool LooseCouplingProcessor::detectStationary() const {
    if (zupt_window_.size() < 5) {
        return false;
    }
    std::vector<double> accel_devs;
    std::vector<double> gyro_devs;
    accel_devs.reserve(zupt_window_.size());
    gyro_devs.reserve(zupt_window_.size());
    for (const auto& sample : zupt_window_) {
        accel_devs.push_back((sample.accel_raw - state_.nominal.accel_bias).norm());
        gyro_devs.push_back((sample.gyro_raw_radps - state_.nominal.gyro_bias).norm());
    }
    const double accel_std = standardDeviation(accel_devs);
    const double gyro_std = standardDeviation(gyro_devs);
    const double gyro_median = median(gyro_devs);
    return accel_std <= config_.zupt_max_accel_std && gyro_std <= config_.zupt_max_gyro_std &&
           gyro_median <= config_.zupt_max_gyro_median;
}

bool LooseCouplingProcessor::gnssSpeedGateAllowsZupt() const {
    if (!config_.zupt_gnss_speed_gate_enable || !has_gnss_velocity_) {
        return true;
    }
    return last_gnss_velocity_speed_mps_ <= config_.zupt_gnss_speed_gate_threshold_mps;
}

void LooseCouplingProcessor::injectAndReset() {
    auto& nominal = state_.nominal;
    nominal.position_enu += error_state_.segment<3>(fusion_index::POSITION);
    nominal.velocity_enu += error_state_.segment<3>(fusion_index::VELOCITY);
    nominal.attitude_body_to_enu =
        (nominal.attitude_body_to_enu *
         attitude::smallAngleQuaternion(error_state_.segment<3>(fusion_index::ATTITUDE)))
            .normalized();
    nominal.accel_bias += error_state_.segment<3>(fusion_index::ACCEL_BIAS);
    nominal.gyro_bias += error_state_.segment<3>(fusion_index::GYRO_BIAS);
    error_state_.setZero();
}

fusion_update::FusionUpdateResult LooseCouplingProcessor::applyUpdateAndInject(
    const fusion_measurement::FusionMeasurementSystem& system, double max_nis_per_observation,
    int& consecutive_gate_rejections) {
    const auto result =
        fusion_update::applyDenseUpdate(error_state_, state_.covariance, system, max_nis_per_observation);

    if (result.ok) {
        injectAndReset();
        consecutive_gate_rejections = 0;
    } else if (result.rejected_by_innovation_gate ||
               result.rejected_by_invalid_innovation_covariance) {
        // A prolonged GNSS outage must not wrap this state back to a negative
        // value and accidentally disable a configured fixed-position recovery.
        if (consecutive_gate_rejections < std::numeric_limits<int>::max()) {
            ++consecutive_gate_rejections;
        }
    }
    return result;
}

bool LooseCouplingProcessor::floatPositionConsistentWithCoarse(
    const GNSSTime& time, const Eigen::Vector3d& antenna_position_enu,
    const Eigen::Matrix3d& position_covariance_enu) const {
    if (!have_coarse_position_) {
        return false;
    }
    const double age_s = time - coarse_position_time_;
    if (!std::isfinite(age_s) || age_s < 0.0 || age_s > config_.float_reanchor_max_coarse_age_s) {
        return false;
    }
    const double max_nis = config_.max_position_update_nis_per_observation;
    if (!(max_nis > 0.0)) {
        return true;  // No NIS gate configured: nothing to compare against.
    }
    const double displacement_m = state_.nominal.velocity_enu.norm() * age_s;
    const Eigen::Matrix3d innovation_covariance =
        position_covariance_enu + coarse_position_covariance_enu_ +
        displacement_m * displacement_m * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d difference = antenna_position_enu - coarse_antenna_position_enu_;
    const Eigen::LDLT<Eigen::Matrix3d> ldlt(innovation_covariance);
    if (!difference.allFinite() || ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
        return false;
    }
    const double nis = difference.dot(ldlt.solve(difference));
    return std::isfinite(nis) && nis / 3.0 <= max_nis;
}

bool LooseCouplingProcessor::reanchorPositionFromFixedSolution(
    const Eigen::Vector3d& antenna_position_enu,
    const Eigen::Matrix3d& position_covariance_enu) {
    const Eigen::Matrix3d rotation = state_.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d target_position_enu =
        antenna_position_enu - rotation * config_.lever_arm_body;
    const Eigen::Vector3d position_before = state_.nominal.position_enu;
    const Eigen::Vector3d position_correction = target_position_enu - position_before;
    const double max_position_correction_m =
        config_.max_fixed_position_reanchor_m;
    const bool finite_correction_bound_exceeded =
        std::isfinite(max_position_correction_m) &&
        position_correction.norm() > max_position_correction_m;
    if (!target_position_enu.allFinite() || !position_before.allFinite() ||
        !position_correction.allFinite() ||
        max_position_correction_m <= 0.0 ||
        std::isnan(max_position_correction_m) ||
        finite_correction_bound_exceeded) {
        return false;
    }

    Eigen::Matrix3d attitude_covariance =
        state_.covariance.block<3, 3>(fusion_index::ATTITUDE, fusion_index::ATTITUDE);
    attitude_covariance = regularizeCovariance3x3(attitude_covariance, 1.0);
    const Eigen::Matrix3d position_attitude_jacobian =
        -rotation * attitude::skew(config_.lever_arm_body);
    const Eigen::Matrix3d reanchor_covariance =
        position_covariance_enu +
        position_attitude_jacobian * attitude_covariance * position_attitude_jacobian.transpose();
    if (!reanchor_covariance.allFinite()) {
        return false;
    }

    state_.nominal.position_enu = target_position_enu;
    state_.covariance.block<3, 15>(fusion_index::POSITION, 0).setZero();
    state_.covariance.block<15, 3>(0, fusion_index::POSITION).setZero();
    state_.covariance.block<3, 3>(fusion_index::POSITION, fusion_index::POSITION) =
        reanchor_covariance;
    error_state_.segment<3>(fusion_index::POSITION).setZero();
    last_gnss_position_correction_enu_ = position_correction;
    return last_gnss_position_correction_enu_.allFinite();
}

bool LooseCouplingProcessor::reanchorVelocityFromGnssSolution(
    const Eigen::Vector3d& antenna_velocity_enu,
    const Eigen::Matrix3d& velocity_covariance_enu) {
    const Eigen::Matrix3d rotation = state_.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d lever_velocity_body =
        last_angular_rate_body_.cross(config_.lever_arm_body);
    const Eigen::Vector3d target_velocity_enu =
        antenna_velocity_enu - rotation * lever_velocity_body;
    const Eigen::Vector3d velocity_before = state_.nominal.velocity_enu;
    const Eigen::Vector3d velocity_correction = target_velocity_enu - velocity_before;
    if (!target_velocity_enu.allFinite() || !velocity_before.allFinite() ||
        !velocity_correction.allFinite() ||
        config_.max_gnss_velocity_reanchor_mps <= 0.0 ||
        !std::isfinite(config_.max_gnss_velocity_reanchor_mps) ||
        velocity_correction.norm() > config_.max_gnss_velocity_reanchor_mps ||
        !velocity_covariance_enu.allFinite()) {
        return false;
    }

    Eigen::Matrix3d attitude_covariance =
        state_.covariance.block<3, 3>(fusion_index::ATTITUDE,
                                      fusion_index::ATTITUDE);
    attitude_covariance = regularizeCovariance3x3(attitude_covariance, 1.0);
    const Eigen::Matrix3d velocity_attitude_jacobian =
        -rotation * attitude::skew(lever_velocity_body);
    Eigen::Matrix3d reanchor_covariance =
        velocity_covariance_enu +
        velocity_attitude_jacobian * attitude_covariance *
            velocity_attitude_jacobian.transpose();
    reanchor_covariance = regularizeCovariance3x3(
        reanchor_covariance, kDefaultVelocitySigmaMps);
    if (!reanchor_covariance.allFinite()) {
        return false;
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(reanchor_covariance);
    if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() ||
        eig.eigenvalues().minCoeff() < 0.0) {
        return false;
    }

    // This is a velocity-only reset. In particular, do not inject attitude,
    // position, or bias error and do not retain cross-covariances that would
    // immediately reintroduce the rejected velocity innovation.
    state_.nominal.velocity_enu = target_velocity_enu;
    state_.covariance.block<3, 15>(fusion_index::VELOCITY, 0).setZero();
    state_.covariance.block<15, 3>(0, fusion_index::VELOCITY).setZero();
    state_.covariance.block<3, 3>(fusion_index::VELOCITY,
                                  fusion_index::VELOCITY) = reanchor_covariance;
    error_state_.segment<3>(fusion_index::VELOCITY).setZero();
    last_gnss_velocity_correction_enu_ = velocity_correction;
    return last_gnss_velocity_correction_enu_.allFinite();
}

void LooseCouplingProcessor::inflateYawUncertainty() {
    // Recovery from an evidently-wrong (or merely stale, e.g. a sustained
    // turn the tight post-latch sigma couldn't keep up with) yaw estimate:
    // re-inflate yaw uncertainty back to the same "unobservable" prior
    // initializeFromStaticWindow() assigns.
    //
    // Earlier version of this recovery also cleared heading_aligned_ and
    // HeadingAlignmentTracker, forcing a fresh multi-epoch-consistency
    // re-latch (a hard, discontinuous override of the nominal attitude).
    // Validating on PPC nagoya/run1 (a route with near-continuous turning
    // soon after the initial mis-latch) showed that approach thrashes: as
    // soon as the vehicle keeps turning, the just-re-latched heading goes
    // stale again within a few seconds, triggering another hard override,
    // repeatedly -- each override discarding whatever the ordinary
    // gyro-mechanized attitude estimate had already tracked correctly
    // through the turn, and replacing it with a short (few-epoch), noisier
    // GNSS-course snapshot. The result was dozens of jarring yaw jumps
    // across the run and much worse roll/pitch RMSE than either the old
    // one-shot-latch code or a plain covariance inflation.
    //
    // Inflating yaw covariance *without* touching the nominal attitude or
    // forcing a re-latch instead lets the ordinary per-epoch GNSS position/
    // velocity Kalman updates -- which already track a turning heading
    // continuously and smoothly via their normal gain, once given enough
    // freedom -- pull a wrong or stale yaw back toward truth, rather than
    // periodically overriding it outright.
    velocity_nis_ema_ = 0.0;
    consecutive_bad_heading_epochs_ = 0;
    recovery_cooldown_remaining_epochs_ = config_.heading_recovery_cooldown_epochs;

    const double yaw_sigma_rad = kInitialYawSigmaDeg * M_PI / 180.0;
    const double current_yaw_variance =
        state_.covariance(fusion_index::ATTITUDE + 2, fusion_index::ATTITUDE + 2);
    state_.covariance.row(fusion_index::ATTITUDE + 2).setZero();
    state_.covariance.col(fusion_index::ATTITUDE + 2).setZero();
    state_.covariance(fusion_index::ATTITUDE + 2, fusion_index::ATTITUDE + 2) =
        std::max(current_yaw_variance, yaw_sigma_rad * yaw_sigma_rad);
}

void LooseCouplingProcessor::updateHeadingHealthAndMaybeRecover(
    const fusion_update::FusionUpdateResult& velocity_result) {
    if (!heading_aligned_ || velocity_result.observation_count == 0) {
        return;
    }

    // Exponential moving average of the velocity update's NIS-per-
    // observation: smoother than the raw per-epoch value (which is noisy
    // even for a genuinely-good latch), while still reacting within a
    // handful of epochs to a persistently bad fit. alpha=0.2 -> ~5-epoch
    // effective memory, well inside the several-second timescale needed to
    // tell "one noisy epoch" from "this yaw is actually wrong."
    constexpr double kEmaAlpha = 0.2;
    velocity_nis_ema_ = kEmaAlpha * velocity_result.normalized_innovation_squared_per_observation +
                       (1.0 - kEmaAlpha) * velocity_nis_ema_;

    if (config_.heading_recovery_nis_threshold <= 0.0 || config_.heading_recovery_min_bad_epochs <= 0) {
        return;  // Recovery disabled.
    }

    if (recovery_cooldown_remaining_epochs_ > 0) {
        // Give the correction from the last recovery time to actually take
        // effect via the ordinary Kalman updates before considering another
        // one -- see Config::heading_recovery_cooldown_epochs doc comment
        // for why an unthrottled recovery can make things worse, not
        // better, once yaw is genuinely unobservable (e.g. constant-
        // velocity cruise).
        --recovery_cooldown_remaining_epochs_;
        consecutive_bad_heading_epochs_ = 0;
        return;
    }

    if (velocity_nis_ema_ > config_.heading_recovery_nis_threshold) {
        ++consecutive_bad_heading_epochs_;
    } else {
        consecutive_bad_heading_epochs_ = 0;
    }

    if (consecutive_bad_heading_epochs_ >= config_.heading_recovery_min_bad_epochs) {
        if (debugHeadingEnabled()) {
            std::cerr << "[HEADING] RECOVERY (inflate) tow=" << state_.nominal.time.tow
                      << " velocity_nis_ema=" << velocity_nis_ema_
                      << " consecutive_bad=" << consecutive_bad_heading_epochs_ << "\n";
        }
        inflateYawUncertainty();
    }
}

void LooseCouplingProcessor::processImuSample(const ImuSample& sample_body_flu) {
    if (!initialized_) {
        static_window_.push_back(sample_body_flu);
        if (static_window_.size() >= 2) {
            const double duration = static_window_.back().time - static_window_.front().time;
            if (duration >= config_.align_static_window_s) {
                initializeFromStaticWindow();
            }
        }
        return;
    }

    const double dt = sample_body_flu.time - state_.nominal.time;
    if (dt <= 0.0) {
        return;
    }

    const Eigen::Vector3d specific_force_body = sample_body_flu.accel_raw - state_.nominal.accel_bias;
    const Eigen::Vector3d angular_rate_body = sample_body_flu.gyro_raw_radps - state_.nominal.gyro_bias;
    last_angular_rate_body_ = angular_rate_body;
    const bool track_v_long = config_.heading_latch_direction_test;

    const auto phi = fusion_process_noise::transitionMatrix(state_.nominal, specific_force_body,
                                                            angular_rate_body, dt);
    const auto qd = fusion_process_noise::processNoiseCovariance(phi, config_.process_noise, dt);

    state_.nominal = mechanization::propagate(state_.nominal, sample_body_flu, dt,
                                              Eigen::Vector3d(0.0, 0.0, -kStandardGravityMps2));
    state_.covariance = phi * state_.covariance * phi.transpose() + qd;
    state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
    dd_transition_since_update_ = phi * dd_transition_since_update_;

    zupt_window_.push_back(sample_body_flu);
    while (zupt_window_.size() > kZuptWindowSize) {
        zupt_window_.pop_front();
    }

    if (track_v_long) {
        // Body-forward kinematic acceleration of this sample. The bias is the
        // one the mechanization just used (specific_force_body, pre-update)
        // and the attitude is the mechanized attitude at the END of the
        // sample (state_.nominal after propagate()). Gravity is removed in
        // the body frame via R_bn^T (ENU -> body); only roll/pitch enter, so
        // the unknown yaw (and any later heading latch) does not matter.
        // v_long is then zeroed by the stationary condition below, using the
        // same window/thresholds as ZUPT and evaluated before the ZUPT/NHC
        // updates of this sample modify the state. It is a pure function of
        // the sample and does not influence the filter state.
        const Eigen::Matrix3d r_bn = state_.nominal.attitude_body_to_enu.toRotationMatrix();
        const double a_x =
            (specific_force_body + r_bn.transpose() * Eigen::Vector3d(0.0, 0.0, -kStandardGravityMps2))
                .x();
        v_long_ += a_x * dt;
        if (gnssSpeedGateAllowsZupt() && detectStationary()) {
            v_long_ = 0.0;
            v_long_valid_ = true;
        }
    }

    if (config_.zupt_enable && gnssSpeedGateAllowsZupt() && detectStationary()) {
        const auto system = fusion_measurement::buildZuptUpdate(state_, config_.zupt_sigma_mps);
        int unused_rejections = 0;  // ZUPT is always ungated (max_nis=0.0), never rejected.
        applyUpdateAndInject(system, 0.0, unused_rejections);
        ++zupt_updates_;
    }
    if (config_.nhc_enable && (!config_.nhc_require_heading_alignment || heading_aligned_)) {
        const auto system = fusion_measurement::buildNhcUpdate(
            state_, config_.lever_arm_body, angular_rate_body, config_.nhc_sigma_lateral_mps,
            config_.nhc_sigma_vertical_mps);
        int unused_rejections = 0;  // NHC is always ungated (max_nis=0.0), never rejected.
        applyUpdateAndInject(system, 0.0, unused_rejections);
    }
}

void LooseCouplingProcessor::processGnssSolution(const PositionSolution& solution) {
    last_gnss_position_update_applied_ = false;
    last_gnss_position_reanchored_ = false;
    last_gnss_velocity_reanchored_ = false;
    last_gnss_position_correction_enu_.setZero();
    last_gnss_velocity_correction_enu_.setZero();
    if (!initialized_ || !solution.isValid()) {
        return;
    }

    if (!origin_set_) {
        origin_ecef_ = solution.position_ecef;
        ecef2geodetic(origin_ecef_, origin_lat_, origin_lon_, origin_height_);
        origin_set_ = true;
    }

    const Eigen::Matrix3d r_e2n = ecefToEnuRotation(origin_lat_, origin_lon_);

    if (solution.has_velocity && solution.velocity_ecef.allFinite()) {
        const Eigen::Vector3d velocity_enu = r_e2n * solution.velocity_ecef;
        if (velocity_enu.allFinite()) {
            has_gnss_velocity_ = true;
            last_gnss_velocity_speed_mps_ = velocity_enu.norm();
        }
    }

    const Eigen::Vector3d antenna_position_enu = r_e2n * (solution.position_ecef - origin_ecef_);
    const Eigen::Matrix3d position_covariance_enu = regularizeCovariance3x3(
        r_e2n * solution.position_covariance * r_e2n.transpose(), kDefaultPositionSigmaM);

    if (config_.float_position_reanchor_after_rejections > 0 && !solution.isFixed() &&
        solution.status != SolutionStatus::FLOAT) {
        have_coarse_position_ = true;
        coarse_position_time_ = solution.time;
        coarse_antenna_position_enu_ = antenna_position_enu;
        coarse_position_covariance_enu_ = position_covariance_enu;
    }

    // A non-FIXED solution is never trusted for position recovery. Reset the
    // FIX patience even when position_updates_require_fixed skips the normal
    // FLOAT/SPP EKF update entirely; otherwise a stale rejection streak could
    // survive a FLOAT epoch and arm a later FIX too early.
    if (!solution.isFixed()) {
        position_consecutive_gate_rejections_ = 0;
    }

    if (!config_.position_updates_require_fixed ||
        solution.isFixed()) {
        const auto position_system =
            fusion_measurement::buildGnssPositionUpdate(
                state_, antenna_position_enu,
                position_covariance_enu,
                config_.lever_arm_body);
        const Eigen::Vector3d position_before =
            state_.nominal.position_enu;
        // Time since the last accepted GNSS position update of any class
        // (position_reanchor_after_gnss_gap_s).
        const bool after_gnss_gap =
            config_.position_reanchor_after_gnss_gap_s > 0.0 && have_position_update_time_ &&
            (solution.time - last_position_update_time_) >
                config_.position_reanchor_after_gnss_gap_s;
        auto position_result = applyUpdateAndInject(
            position_system,
            config_.max_position_update_nis_per_observation,
            position_consecutive_gate_rejections_);
        // Only a consecutive run of trusted FIXED rejections is eligible for
        // the deterministic position re-anchor. SPP/FLOAT rejects must not
        // consume the patience budget or leave a stale budget armed for the
        // next FIX epoch.
        if (!position_result.ok &&
            (position_result.rejected_by_innovation_gate ||
             position_result.rejected_by_invalid_innovation_covariance) &&
            solution.isFixed() && config_.max_consecutive_gate_rejections > 0 &&
            position_consecutive_gate_rejections_ >= config_.max_consecutive_gate_rejections &&
            reanchorPositionFromFixedSolution(antenna_position_enu, position_covariance_enu)) {
            position_result.ok = true;
            position_consecutive_gate_rejections_ = 0;
            last_gnss_position_update_applied_ = true;
            last_gnss_position_reanchored_ = true;
        }
        // velocity_consistency_v2: the precise (FLOAT/FIXED) class has its own
        // rejection streak, independent of coarse SPP acceptance.
        if ((config_.float_position_reanchor_after_rejections > 0 ||
             config_.position_reanchor_after_gnss_gap_s > 0.0) &&
            (solution.isFixed() || solution.status == SolutionStatus::FLOAT)) {
            if (position_result.ok) {
                float_class_consecutive_gate_rejections_ = 0;
            } else if (position_result.rejected_by_innovation_gate ||
                       position_result.rejected_by_invalid_innovation_covariance) {
                if (float_class_consecutive_gate_rejections_ < std::numeric_limits<int>::max()) {
                    ++float_class_consecutive_gate_rejections_;
                }
                const bool patience_reanchor =
                    config_.float_position_reanchor_after_rejections > 0 &&
                    float_class_consecutive_gate_rejections_ >=
                        config_.float_position_reanchor_after_rejections &&
                    floatPositionConsistentWithCoarse(solution.time, antenna_position_enu,
                                                      position_covariance_enu);
                if ((after_gnss_gap || patience_reanchor) &&
                    reanchorPositionFromFixedSolution(antenna_position_enu, position_covariance_enu)) {
                    position_result.ok = true;
                    position_consecutive_gate_rejections_ = 0;
                    float_class_consecutive_gate_rejections_ = 0;
                    last_gnss_position_update_applied_ = true;
                    last_gnss_position_reanchored_ = true;
                }
            }
        }
        if (position_result.ok) {
            if (!last_gnss_position_reanchored_) {
                last_gnss_position_update_applied_ = true;
                last_gnss_position_correction_enu_ =
                    state_.nominal.position_enu - position_before;
            }
        }
        if (last_gnss_position_update_applied_) {
            have_position_update_time_ = true;
            last_position_update_time_ = solution.time;
        }
        if (solution.isFixed() && position_result.ok &&
            std::isfinite(
                position_result
                    .normalized_innovation_squared_per_observation) &&
            position_result
                    .normalized_innovation_squared_per_observation <=
                config_
                    .tight_dd_sse_fixed_anchor_max_nis_per_observation) {
            has_fixed_anchor_ = true;
            last_fixed_anchor_time_ = solution.time;
            last_fixed_anchor_nis_per_observation_ =
                position_result
                    .normalized_innovation_squared_per_observation;
        }
    }

    if (solution.has_velocity) {
        const Eigen::Vector3d antenna_velocity_enu = r_e2n * solution.velocity_ecef;
        const Eigen::Matrix3d velocity_covariance_enu = regularizeCovariance3x3(
            r_e2n * solution.velocity_covariance * r_e2n.transpose(), kDefaultVelocitySigmaMps);

        // A velocity-only recovery is allowed only for an independently
        // solved, finite/PSD Doppler covariance. regularizeCovariance3x3()
        // is intentionally not the quality gate here: replacing a NaN or a
        // grossly indefinite covariance with a fallback would turn an
        // untrusted solution into an unconditional reset.
        bool velocity_solution_quality_ok = solution.velocity_ecef.allFinite() &&
            solution.velocity_covariance.allFinite();
        if (velocity_solution_quality_ok) {
            const Eigen::Matrix3d raw_velocity_covariance =
                0.5 * (solution.velocity_covariance +
                        solution.velocity_covariance.transpose());
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(
                raw_velocity_covariance);
            velocity_solution_quality_ok =
                eig.info() == Eigen::Success && eig.eigenvalues().allFinite() &&
                eig.eigenvalues().minCoeff() >= -1e-9;
        }

        const auto velocity_system = fusion_measurement::buildGnssVelocityUpdate(
            state_, antenna_velocity_enu, velocity_covariance_enu, config_.lever_arm_body,
            last_angular_rate_body_);
        const auto velocity_result = applyUpdateAndInject(
            velocity_system, config_.max_velocity_update_nis_per_observation,
            velocity_consecutive_gate_rejections_);
        if (!velocity_result.ok &&
            (velocity_result.rejected_by_innovation_gate ||
             velocity_result.rejected_by_invalid_innovation_covariance) &&
            velocity_solution_quality_ok &&
            config_.max_consecutive_velocity_gate_rejections > 0 &&
            velocity_consecutive_gate_rejections_ >=
                config_.max_consecutive_velocity_gate_rejections &&
            reanchorVelocityFromGnssSolution(
                antenna_velocity_enu, velocity_covariance_enu)) {
            velocity_consecutive_gate_rejections_ = 0;
            last_gnss_velocity_reanchored_ = true;
        }
        // Post-latch health/recovery (yaw-covariance inflation only -- see
        // Config::heading_recovery_* doc comment) must run before the fresh-
        // latch attempt below, though in practice it only ever affects that
        // branch when heading_aligned_ is still false to begin with.
        updateHeadingHealthAndMaybeRecover(velocity_result);

        heading_tracker_.addSample(solution.time, antenna_velocity_enu,
                                   config_.align_velocity_threshold_mps, config_.align_heading_window_s);

        if (!heading_aligned_) {
            double mean_course_rad = 0.0;
            double scatter_deg = 0.0;
            if (heading_tracker_.ready(config_.align_heading_min_samples,
                                      config_.align_heading_max_course_scatter_deg, &mean_course_rad,
                                      &scatter_deg)) {
                // Reconstruct a (direction-only) velocity vector from the
                // tracker's consensus course so tryAlignHeading()'s existing
                // math (target_heading = atan2(vx, vy)) can be reused
                // unchanged; magnitude is irrelevant to atan2, so any value
                // above the tracker's own min_speed_mps gate works, and the
                // tracker has already verified every buffered sample cleared
                // that gate.
                //
                // velocity_consistency_v5: the course is the direction of
                // travel; if the integrated longitudinal velocity says the
                // vehicle is moving backward, body +X points the other way.
                const bool flip_direction = config_.heading_latch_direction_test &&
                                            v_long_valid_ && v_long_ < 0.0;
                const double latch_course_rad =
                    flip_direction ? mean_course_rad + M_PI : mean_course_rad;
                const Eigen::Vector3d consensus_velocity_enu(std::sin(latch_course_rad),
                                                             std::cos(latch_course_rad), 0.0);
                const double honest_sigma_deg = std::max(config_.align_heading_sigma_deg, scatter_deg);
                if (fusion_initialization::tryAlignHeading(state_, consensus_velocity_enu,
                                                           /*min_speed_mps=*/0.5, honest_sigma_deg)) {
                    heading_aligned_ = true;
                    velocity_nis_ema_ = 0.0;
                    consecutive_bad_heading_epochs_ = 0;
                    last_latch_direction_flipped_ = flip_direction;
                    last_latch_v_long_mps_ =
                        (config_.heading_latch_direction_test && v_long_valid_)
                            ? v_long_
                            : std::numeric_limits<double>::quiet_NaN();
                    if (flip_direction) ++latch_direction_flip_count_;
                    // The latch rotated only the attitude. Make velocity and
                    // its correlations consistent with the new attitude by
                    // re-anchoring to this epoch's independent GNSS velocity.
                    if (config_.reanchor_velocity_on_heading_latch &&
                        velocity_solution_quality_ok &&
                        reanchorVelocityFromGnssSolution(antenna_velocity_enu,
                                                         velocity_covariance_enu)) {
                        velocity_consecutive_gate_rejections_ = 0;
                        last_gnss_velocity_reanchored_ = true;
                    }
                    if (debugHeadingEnabled()) {
                        std::cerr << "[HEADING] LATCH tow=" << solution.time.tow
                                  << " mean_course_deg=" << (mean_course_rad * 180.0 / M_PI)
                                  << " scatter_deg=" << scatter_deg << " honest_sigma_deg=" << honest_sigma_deg
                                  << " nsamples=" << heading_tracker_.sampleCount();
                        if (config_.heading_latch_direction_test) {
                            std::cerr << " v_long_mps=" << v_long_
                                      << " v_long_valid=" << (v_long_valid_ ? 1 : 0)
                                      << " direction_flipped=" << (flip_direction ? 1 : 0)
                                      << " latch_course_deg=" << (latch_course_rad * 180.0 / M_PI);
                        }
                        std::cerr << "\n";
                    }
                }
            }
        }
    }
}

LooseCouplingProcessor::TightlyCoupledDDResult
LooseCouplingProcessor::processTightlyCoupledDD(
    const std::vector<dd_imu_bridge::DDObservation>& observations,
    const PositionSolution* recovery_solution) {
    TightlyCoupledDDResult result;
    if (!initialized_ || !origin_set_ || observations.empty()) {
        return result;
    }
    if (!dd_imu_bridge_) {
        dd_imu_bridge::BridgeConfig bridge_config;
        bridge_config.commit_carrier_updates =
            config_.tight_dd_commit_carrier_updates;
        dd_imu_bridge_.reset(new dd_imu_bridge::DDIMUBridge(state_, bridge_config));
    } else {
        dd_imu_bridge_->acceptPropagatedINS(state_, dd_transition_since_update_);
    }
    dd_transition_since_update_.setIdentity();

    result.update = dd_imu_bridge_->update(observations);
    result.sse_partial_ar =
        dd_imu_bridge_->evaluateSSEPartialAmbiguities(
            observations, hasHealthyFixedAnchor());
    if (result.update.ok && result.update.carrier_update_accepted) {
        result.partial_ar = dd_imu_bridge_->resolvePartialAmbiguities(observations);
    } else if (result.update.rejected_by_innovation_gate &&
               recovery_solution != nullptr && recovery_solution->isValid()) {
        const Eigen::Matrix3d r_e2n = ecefToEnuRotation(origin_lat_, origin_lon_);
        const Eigen::Vector3d antenna_enu =
            r_e2n * (recovery_solution->position_ecef - origin_ecef_);
        const Eigen::Vector3d imu_position_enu = antenna_enu -
            state_.nominal.attitude_body_to_enu * config_.lever_arm_body;
        result.reset_action = dd_imu_bridge_->softResetPosition(
            imu_position_enu, /*propagated_state_valid=*/true);
    }
    state_ = dd_imu_bridge_->state().eskf;
    return result;
}

bool LooseCouplingProcessor::predictedAntennaPositionEcef(Eigen::Vector3d& ecef_pos,
                                                           Eigen::Matrix3d& ecef_cov) const {
    if (!origin_set_) {
        return false;
    }

    const Eigen::Matrix3d r_e2n = ecefToEnuRotation(origin_lat_, origin_lon_);
    const Eigen::Matrix3d r_n2e = r_e2n.transpose();
    const Eigen::Matrix3d rotation = state_.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d antenna_position_enu =
        state_.nominal.position_enu + rotation * config_.lever_arm_body;
    ecef_pos = origin_ecef_ + r_n2e * antenna_position_enu;

    // Same H as fusion_measurement::buildGnssPositionUpdate(): [I 0 -R*skew(lever) 0 0].
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(3, fusion_index::SIZE);
    h.block<3, 3>(0, fusion_index::POSITION) = Eigen::Matrix3d::Identity();
    h.block<3, 3>(0, fusion_index::ATTITUDE) = -rotation * attitude::skew(config_.lever_arm_body);
    const Eigen::Matrix3d cov_enu = h * state_.covariance * h.transpose();
    ecef_cov = r_n2e * cov_enu * r_e2n;
    return true;
}

bool LooseCouplingProcessor::hasHealthyFixedAnchor() const {
    if (!has_fixed_anchor_ || !isHeadingConverged() ||
        !std::isfinite(last_fixed_anchor_nis_per_observation_) ||
        last_fixed_anchor_nis_per_observation_ >
            config_
                .tight_dd_sse_fixed_anchor_max_nis_per_observation) {
        return false;
    }
    const double age_s =
        state_.nominal.time - last_fixed_anchor_time_;
    return std::isfinite(age_s) && age_s >= -1e-6 &&
           age_s <=
               config_.tight_dd_sse_fixed_anchor_max_age_s;
}

PositionSolution LooseCouplingProcessor::toPositionSolution() const {
    PositionSolution solution;
    solution.time = state_.nominal.time;
    solution.status = SolutionStatus::FLOAT;
    solution.num_satellites = 0;

    if (origin_set_) {
        const Eigen::Matrix3d r_n2e = ecefToEnuRotation(origin_lat_, origin_lon_).transpose();
        solution.position_ecef = origin_ecef_ + r_n2e * state_.nominal.position_enu;
        double lat = 0.0, lon = 0.0, height = 0.0;
        ecef2geodetic(solution.position_ecef, lat, lon, height);
        solution.position_geodetic = GeodeticCoord(lat, lon, height);
        solution.position_covariance =
            r_n2e * state_.covariance.block<3, 3>(fusion_index::POSITION, fusion_index::POSITION) *
            r_n2e.transpose();
        solution.velocity_ecef = r_n2e * state_.nominal.velocity_enu;
        solution.velocity_covariance =
            r_n2e * state_.covariance.block<3, 3>(fusion_index::VELOCITY, fusion_index::VELOCITY) *
            r_n2e.transpose();
        solution.has_velocity = true;
    } else {
        // No GNSS anchor yet: best-effort output in the local tangent frame,
        // not a real ECEF position.
        solution.position_ecef = state_.nominal.position_enu;
        solution.has_velocity = false;
    }

    return solution;
}

PositionSolution LooseCouplingProcessor::toAntennaPositionSolution() const {
    PositionSolution solution = toPositionSolution();
    if (!origin_set_) {
        return solution;
    }

    const Eigen::Matrix3d r_e2n = ecefToEnuRotation(origin_lat_, origin_lon_);
    const Eigen::Matrix3d r_n2e = r_e2n.transpose();
    const Eigen::Matrix3d rotation = state_.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d lever_velocity_body =
        last_angular_rate_body_.cross(config_.lever_arm_body);

    const Eigen::Vector3d antenna_position_enu =
        state_.nominal.position_enu + rotation * config_.lever_arm_body;
    solution.position_ecef = origin_ecef_ + r_n2e * antenna_position_enu;
    double lat = 0.0, lon = 0.0, height = 0.0;
    ecef2geodetic(solution.position_ecef, lat, lon, height);
    solution.position_geodetic = GeodeticCoord(lat, lon, height);

    Eigen::MatrixXd position_h = Eigen::MatrixXd::Zero(3, fusion_index::SIZE);
    position_h.block<3, 3>(0, fusion_index::POSITION) = Eigen::Matrix3d::Identity();
    position_h.block<3, 3>(0, fusion_index::ATTITUDE) =
        -rotation * attitude::skew(config_.lever_arm_body);
    const Eigen::Matrix3d position_covariance_enu =
        position_h * state_.covariance * position_h.transpose();
    solution.position_covariance =
        r_n2e * position_covariance_enu * r_e2n;

    const Eigen::Vector3d antenna_velocity_enu =
        state_.nominal.velocity_enu + rotation * lever_velocity_body;
    solution.velocity_ecef = r_n2e * antenna_velocity_enu;
    solution.has_velocity = true;

    // Keep this Jacobian identical to buildGnssVelocityUpdate().  The latest
    // bias-corrected gyro sample is treated as an observed input, so this
    // H*P*H^T covers state uncertainty but deliberately does not add a
    // separate gyro measurement-noise term.
    Eigen::MatrixXd velocity_h = Eigen::MatrixXd::Zero(3, fusion_index::SIZE);
    velocity_h.block<3, 3>(0, fusion_index::VELOCITY) = Eigen::Matrix3d::Identity();
    velocity_h.block<3, 3>(0, fusion_index::ATTITUDE) =
        -rotation * attitude::skew(lever_velocity_body);
    const Eigen::Matrix3d velocity_covariance_enu =
        velocity_h * state_.covariance * velocity_h.transpose();
    solution.velocity_covariance =
        r_n2e * velocity_covariance_enu * r_e2n;

    return solution;
}

Eigen::Matrix3d LooseCouplingProcessor::ecefToLocalEnuRotation() const {
    return origin_set_ ? ecefToEnuRotation(origin_lat_, origin_lon_)
                       : Eigen::Matrix3d::Identity();
}

}  // namespace libgnss
