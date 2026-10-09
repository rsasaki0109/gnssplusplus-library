#include <libgnss++/fusion/online_rtk_imu.hpp>
#include <libgnss++/fusion/attitude.hpp>
#include <libgnss++/algorithms/rtk_base_alignment.hpp>
#include <libgnss++/algorithms/rtk_presets.hpp>
#include <libgnss++/algorithms/spp_velocity.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace libgnss {
namespace {
constexpr double kExactEpochToleranceS = 1e-6;
void validateTime(const GNSSTime& time) {
    if (time.week < 0 || !std::isfinite(time.tow) || time.tow < 0.0 || time.tow >= 604800.0)
        throw std::invalid_argument("expected normalized finite GPST week/tow");
}
void validateSourceTime(const GNSSTime& time, const GNSSTime& arrival) {
    validateTime(time);
    if (time > arrival) throw std::invalid_argument("observation timestamp exceeds reception timestamp");
}
}

OnlineRtkImuProcessor::OnlineRtkImuProcessor(const Config& config) : config_(config) {
    if (!config_.base_position_ecef.allFinite() || config_.base_position_ecef.norm() < 1e6 ||
        !std::isfinite(config_.max_imu_gap_s) || config_.max_imu_gap_s <= 0.0 ||
        !std::isfinite(config_.max_fusion_age_s) || config_.max_fusion_age_s < 0.0 ||
        !std::isfinite(config_.max_rover_gap_s) || config_.max_rover_gap_s <= 0.0 ||
        !std::isfinite(config_.max_tight_interval_s) || config_.max_tight_interval_s <= 0.0 ||
        config_.max_pending_imu == 0 || config_.max_pending_base == 0 ||
        config_.max_ephemerides_per_satellite == 0 ||
        !std::isfinite(config_.base_extrapolation_max_age_s) || config_.base_extrapolation_max_age_s < 0.0 ||
        config_.rtk.position_mode != RTKProcessor::RTKConfig::PositionMode::KINEMATIC)
        throw std::invalid_argument("invalid online RTK/IMU configuration");
    if (!config_.rtk_preset.empty()) {
        RTKProcessor::RTKConfig probe;
        if (!applyRtkPreset(probe, config_.rtk_preset))
            throw std::invalid_argument("unknown RTK preset: " + config_.rtk_preset);
    }
    recreateFilters();
}

void OnlineRtkImuProcessor::recreateRtkFilter() {
    auto rtk_config = config_.rtk;
    if (!config_.rtk_preset.empty()) applyRtkPreset(rtk_config, config_.rtk_preset);
    rtk_config.use_external_position_time_update = config_.tight_time_update;
    rtk_config.enable_velocity_states = config_.tight_time_update;
    rtk_ = std::make_unique<RTKProcessor>(rtk_config);
    if (!rtk_->initialize(config_.processor)) throw std::invalid_argument("RTK initialization failed");
    rtk_->setBasePosition(config_.base_position_ecef);
}

void OnlineRtkImuProcessor::recreateFusionFilter() {
    fusion_ = std::make_unique<LooseCouplingProcessor>(config_.fusion);
}

void OnlineRtkImuProcessor::recreatePriorFusionFilter() {
    prior_fusion_ = config_.rtk_prior_fusion
        ? std::make_unique<LooseCouplingProcessor>(*config_.rtk_prior_fusion) : nullptr;
}

void OnlineRtkImuProcessor::recreateFilters(bool carry_gyro_bias) {
    std::optional<Vector3d> carried_gyro_bias;
    if (carry_gyro_bias && config_.carry_gyro_bias_across_reset && fusion_ && fusion_->isInitialized())
        carried_gyro_bias = fusion_->state().nominal.gyro_bias;
    recreateRtkFilter();
    recreateFusionFilter();
    if (carried_gyro_bias) fusion_->seedGyroBiasForNextInitialization(*carried_gyro_bias);
    recreatePriorFusionFilter();
    recreateTightFilter();
    have_imu_ = false;
}

void OnlineRtkImuProcessor::recreateRtkSideFilters() {
    recreateRtkFilter();
    recreatePriorFusionFilter();
    recreateTightFilter();
}

void OnlineRtkImuProcessor::recreateTightFilter() {
    TightCouplingProcessor::Config tight_config;
    tight_config.process_noise = config_.fusion.process_noise;
    tight_config.lever_arm_body = config_.fusion.lever_arm_body;
    tight_config.max_sample_gap_s = config_.max_imu_gap_s;
    tight_config.zupt_enable = config_.fusion.zupt_enable;
    tight_config.nhc_enable = config_.fusion.nhc_enable;
    tight_config.velocity_state_output_enable = true;
    tight_ = std::make_unique<TightCouplingProcessor>(tight_config);
    have_tight_anchor_ = false;
}

void OnlineRtkImuProcessor::validateArrival(const GNSSTime& time) const {
    validateTime(time);
    if (have_arrival_ && time < arrival_) throw std::invalid_argument("reception timestamps must be monotone");
}
void OnlineRtkImuProcessor::acceptArrival(const GNSSTime& time) {
    arrival_ = time;
    have_arrival_ = true;
}
void OnlineRtkImuProcessor::pushNavigation(const NavigationData& nav, const GNSSTime& arrival) {
    validateArrival(arrival);
    for (const auto& entry : nav.ephemeris_data) {
        for (const auto& eph : entry.second) navigation_.addEphemerisIfNew(eph);
        auto& retained = navigation_.ephemeris_data[entry.first];
        // addEphemeris sorts by toe; retain the most recent bounded records.
        if (retained.size() > config_.max_ephemerides_per_satellite)
            retained.erase(retained.begin(), retained.end() - config_.max_ephemerides_per_satellite);
    }
    if (nav.ionosphere_model.valid) navigation_.ionosphere_model = nav.ionosphere_model;
    acceptArrival(arrival);
}
void OnlineRtkImuProcessor::pushBase(const ObservationData& obs, const GNSSTime& arrival) {
    validateArrival(arrival);
    validateSourceTime(obs.time, arrival);
    if (have_rover_ && obs.time <= rover_time_) {
        ++diagnostics_.expired_base_epochs;
        acceptArrival(arrival);
        return; // An emitted rover epoch can never be revised by a late base.
    }
    auto at = std::lower_bound(base_.begin(), base_.end(), obs.time,
        [](const ObservationData& existing, const GNSSTime& time) { return existing.time < time; });
    if (at != base_.end() && std::abs(at->time - obs.time) <= kExactEpochToleranceS)
        throw std::invalid_argument("duplicate pending base epoch");
    if (base_.size() >= config_.max_pending_base) throw std::length_error("pending base capacity exceeded");
    base_.insert(at, obs);
    acceptArrival(arrival);
}
bool OnlineRtkImuProcessor::pushImu(const ImuSample& sample, const GNSSTime& arrival) {
    validateArrival(arrival);
    validateSourceTime(sample.time, arrival);
    if (!sample.accel_raw.allFinite() || !sample.gyro_raw_radps.allFinite())
        throw std::invalid_argument("nonfinite IMU payload");
    if ((have_rover_ && sample.time <= rover_time_) ||
        (have_queued_imu_ && sample.time <= last_queued_imu_)) {
        ++diagnostics_.late_imu_dropped;
        acceptArrival(arrival);
        return false;
    }
    if (imu_.size() >= config_.max_pending_imu) throw std::length_error("pending IMU capacity exceeded");
    imu_.push_back(sample);
    last_queued_imu_ = sample.time;
    have_queued_imu_ = true;
    acceptArrival(arrival);
    return true;
}

OnlineRtkImuProcessor::Output OnlineRtkImuProcessor::processRover(
    const ObservationData& obs, const GNSSTime& arrival) {
    validateArrival(arrival);
    validateSourceTime(obs.time, arrival);
    if (have_rover_ && obs.time <= rover_time_) throw std::invalid_argument("rover epochs must increase");
    const auto started = std::chrono::steady_clock::now();
    // NavigationData memoizes states across calls. Rebuild from the bounded
    // received records so a long live stream retains only this epoch's cache.
    NavigationData epoch_navigation;
    epoch_navigation.ionosphere_model = navigation_.ionosphere_model;
    for (const auto& entry : navigation_.ephemeris_data)
        for (const auto& eph : entry.second) epoch_navigation.addEphemeris(eph);
    Output out;
    out.received_at = arrival;
    out.input_age_s = arrival - obs.time;
    if (have_rover_ && obs.time - rover_time_ > config_.max_rover_gap_s) {
        if (config_.rover_gap_keeps_inertial_filters) {
            // GNSS-only gap: the fused loose filter and the IMU continuity
            // flag are kept; the IMU-gap checks below still recreate
            // everything if the IMU itself has a gap.
            recreateRtkSideFilters();
            ++diagnostics_.rover_gap_rtk_resets;
            out.reason = "rover_gap_rtk_reset";
        } else {
            recreateFilters(true);
            ++diagnostics_.rover_gap_resets;
            ++diagnostics_.reset_generation;
            out.reason = "rover_gap_reset";
        }
    }
    while (!imu_.empty() && imu_.front().time <= obs.time) {
        const ImuSample sample = imu_.front();
        imu_.pop_front();
        if (have_imu_ && sample.time - imu_time_ > config_.max_imu_gap_s + 1e-9) {
            recreateFilters(true);
            ++diagnostics_.imu_gap_resets;
            ++diagnostics_.reset_generation;
            out.reason = "imu_gap_reset";
        }
        fusion_->processImuSample(sample);
        if (prior_fusion_) prior_fusion_->processImuSample(sample);
        if (config_.tight_time_update) tight_->processImuSample(sample);
        imu_time_ = sample.time;
        have_imu_ = true;
        ++out.imu_consumed;
    }
    // A stale last sample also invalidates the filters, before GNSS feedback.
    if (have_imu_ && obs.time - imu_time_ > config_.max_imu_gap_s + 1e-9) {
        recreateFilters(true);
        ++diagnostics_.imu_gap_resets;
        ++diagnostics_.reset_generation;
        out.reason = "imu_stale_reset";
    }
    const bool extrapolation = config_.base_extrapolation_max_age_s > 0.0;
    while (!base_.empty() && base_.front().time < obs.time - kExactEpochToleranceS) {
        // Expired epochs are in time order; the last one popped is the latest
        // past epoch. Keep it only when extrapolation may use it.
        if (extrapolation) {
            last_past_base_ = std::move(base_.front());
            have_last_past_base_ = true;
        }
        base_.pop_front();
        ++diagnostics_.expired_base_epochs;
    }
    out.exact_base_available = !base_.empty() &&
        std::abs(base_.front().time - obs.time) <= kExactEpochToleranceS &&
        base_.front().time <= obs.time; // Never admit even a near future epoch.
    // Opt-in: without an exact base, hold the latest past base epoch to the
    // rover time. Never taken (and last_past_base_ never set) when off.
    ObservationData extrapolated_base;
    if (extrapolation && !out.exact_base_available && have_last_past_base_ &&
        rtk_base_alignment::holdBaseEpoch(last_past_base_, obs.time, config_.base_position_ecef,
            epoch_navigation, config_.base_extrapolation_max_age_s, extrapolated_base))
        out.extrapolated_base_available = true;
    // An extrapolated epoch advances the RTK filter exactly like an exact-base
    // epoch, so it takes the same time-update and anchoring branches below.
    // With the option off extrapolated_base_available is always false.
    const bool differential_base = out.exact_base_available || out.extrapolated_base_available;
    const bool imu_at_epoch = have_imu_ && std::abs(imu_time_ - obs.time) <= kExactEpochToleranceS;
    if (have_tight_anchor_ && obs.time - tight_anchor_time_ > config_.max_tight_interval_s)
        recreateTightFilter();
    if (differential_base) {
        if (config_.tight_time_update && imu_at_epoch) {
            const auto update = tight_->prepareTimeUpdate();
            if (update.valid) {
                rtk_->setExternalPositionVelocityTimeUpdate(update.antenna_delta_ecef,
                    update.antenna_velocity_ecef, update.position_velocity_process_noise_ecef,
                    update.velocity_covariance_ecef);
                out.tight_time_update_supplied = true;
            }
        }
        if (out.exact_base_available) {
            out.rtk = rtk_->processRTKEpoch(obs, base_.front(), epoch_navigation);
            if (extrapolation) {
                // The consumed exact epoch is the latest past epoch for the
                // following rover epochs (base_ no longer holds it).
                last_past_base_ = std::move(base_.front());
                have_last_past_base_ = true;
            }
            base_.pop_front();
        } else {
            out.rtk = rtk_->processRTKEpoch(obs, extrapolated_base, epoch_navigation);
            ++diagnostics_.extrapolated_base_epochs;
        }
    } else {
        // Explicit SPP fallback; no stale differential observation is stamped
        // with the rover time and no waiting for a future base occurs.
        out.rtk = rtk_->processEpoch(obs, epoch_navigation);
        ++diagnostics_.missing_base_epochs;
        if (out.reason.empty()) out.reason = "missing_exact_base";
    }
    // GNSS correction must be synchronous with the mechanized state. For an
    // unsampled epoch emit the older prediction with its real timestamp.
    // GNSS input to the loose/tight filters. Normally the RTK solution itself;
    // the opt-in candidate replaces its velocity by an independent Doppler LS
    // solution. out.rtk (the exported RTK result) is never modified.
    PositionSolution gnss_input = out.rtk;
    if (config_.independent_doppler_velocity && config_.tight_time_update && out.rtk.isValid()) {
        const auto doppler = spp_velocity::solveVelocityFromObservations(
            obs, epoch_navigation, out.rtk.position_ecef, rtk_->getDopplerVelocitySigma());
        gnss_input.has_velocity = doppler.ok && doppler.velocity_ecef.allFinite() &&
            doppler.velocity_covariance.allFinite();
        if (gnss_input.has_velocity) {
            gnss_input.velocity_ecef = doppler.velocity_ecef;
            gnss_input.velocity_covariance = doppler.velocity_covariance;
        } else {
            gnss_input.velocity_ecef.setZero();
            gnss_input.velocity_covariance.setZero();
        }
    }
    if (out.rtk.isValid() && imu_at_epoch) fusion_->processGnssSolution(gnss_input);
    if (prior_fusion_ && out.rtk.isValid() && imu_at_epoch) {
        // The isolated filter sees what the unmodified processor would have
        // reported, so the RTK prior is independent of the reporting mode.
        PositionSolution legacy_input = gnss_input;
        if (legacy_input.rtk_reported_covariance_replaced)
            legacy_input.position_covariance =
                Matrix3d::Identity() * RTKProcessor::RTKConfig::kLegacyReportedVarianceM2;
        prior_fusion_->processGnssSolution(legacy_input);
    }
    if (config_.tight_time_update) {
        bool anchored = false;
        Vector3d anchor;
        Matrix3d covariance;
        const LooseCouplingProcessor& prior = prior_fusion_ ? *prior_fusion_ : *fusion_;
        const bool bootstrap_ready = tight_->initialized() ||
            (prior.isInitialized() && prior.isOriginSet() && prior.isHeadingConverged());
        if (differential_base && imu_at_epoch && out.rtk.isValid() &&
            gnss_input.has_velocity && bootstrap_ready &&
            gnss_input.velocity_ecef.allFinite() && gnss_input.velocity_covariance.allFinite() &&
            rtk_->getFloatPosteriorPosition(anchor, covariance)) {
            anchored = tight_->reanchor(anchor, covariance, gnss_input.velocity_ecef,
                gnss_input.velocity_covariance, obs.time,
                tight_->initialized() ? nullptr : &prior.state());
        }
        if (anchored) {
            tight_anchor_time_ = obs.time;
            have_tight_anchor_ = true;
        } else if (differential_base) {
            // A failed differential anchor must bootstrap from a fresh LC
            // state rather than retain an unpropagated old attitude. An SPP
            // fallback between base epochs does not advance the RTK filter:
            // preserve its short IMU interval for the next exact-base epoch.
            recreateTightFilter();
        }
    }
    out.fusion_initialized = fusion_->isInitialized() && fusion_->isOriginSet();
    out.heading_converged = fusion_->isHeadingConverged();
    out.heading_aligned = fusion_->isHeadingAligned();
    out.gnss_position_updated = imu_at_epoch && out.rtk.isValid() &&
        fusion_->lastGnssPositionUpdateApplied();
    out.fused = fusion_->toAntennaPositionSolution();
    if (out.gnss_position_updated) {
        out.fused.num_satellites = out.rtk.num_satellites;
        out.fused.status = out.rtk.isFixed() ? SolutionStatus::FLOAT : out.rtk.status;
    } else {
        out.fused.status = SolutionStatus::PROPAGATED;
    }
    out.fusion_age_s = out.fusion_initialized ? obs.time - out.fused.time :
        std::numeric_limits<double>::quiet_NaN();
    if (!out.fusion_initialized || !std::isfinite(out.fusion_age_s) ||
        out.fusion_age_s < 0.0 || out.fusion_age_s > config_.max_fusion_age_s)
        out.fused = PositionSolution{};
    const auto& nominal = fusion_->state().nominal;
    const double attitude_age_s = obs.time - nominal.time;
    if (out.fusion_initialized && std::isfinite(attitude_age_s) && attitude_age_s >= 0.0 &&
        attitude_age_s <= config_.max_fusion_age_s && nominal.attitude_body_to_enu.coeffs().allFinite() &&
        std::abs(nominal.attitude_body_to_enu.norm() - 1.0) < 1e-6) {
        out.attitude_available = true;
        out.attitude_time = nominal.time;
        out.attitude_body_to_enu = nominal.attitude_body_to_enu;
        out.ecef_to_attitude_enu = fusion_->ecefToLocalEnuRotation();
        out.accel_bias_body_mps2 = nominal.accel_bias;
        out.gyro_bias_body_radps = nominal.gyro_bias;
        out.rpy_frd_ned_deg = attitude::fluEnuToFrdNedRpyDegrees(out.attitude_body_to_enu);
    }
    out.reset_generation = diagnostics_.reset_generation;
    ++diagnostics_.rover_epochs;
    rover_time_ = obs.time;
    have_rover_ = true;
    acceptArrival(arrival);
    out.processing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    return out;
}
void OnlineRtkImuProcessor::reset(const GNSSTime& arrival) {
    validateArrival(arrival);
    recreateFilters();
    base_.clear();
    last_past_base_ = ObservationData();
    have_last_past_base_ = false;
    imu_.clear();
    have_queued_imu_ = false;
    have_rover_ = false;
    ++diagnostics_.reset_generation;
    acceptArrival(arrival);
}
} // namespace libgnss
