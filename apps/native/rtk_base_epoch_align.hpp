#pragma once

// Shared rover/base RINEX epoch-alignment helpers used by every native CLI
// that drives libgnss::RTKProcessor over a rover.obs/base.obs pair
// (apps/native/gnss_solve.cpp, apps/native/gnss_fuse.cpp). Factored out so gnss_fuse's
// --base RTK path reuses the exact same base-epoch time alignment /
// linear-in-range interpolation logic gnss_solve already uses, instead of
// forking a second copy of it (docs/design.md 5 / CLI task notes).

#include <cmath>
#include <limits>
#include <map>
#include <string>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>
#include <libgnss++/models/troposphere.hpp>

namespace libgnss_apps {

constexpr double kExactTimeToleranceSeconds = 1e-6;

inline double timeDiffSeconds(const libgnss::GNSSTime& a, const libgnss::GNSSTime& b) {
    return a - b;
}

struct ObservationKey {
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal;

    bool operator<(const ObservationKey& other) const {
        if (satellite < other.satellite) return true;
        if (other.satellite < satellite) return false;
        return signal < other.signal;
    }
};

inline std::map<ObservationKey, const libgnss::Observation*> indexObservations(
    const libgnss::ObservationData& epoch) {
    std::map<ObservationKey, const libgnss::Observation*> indexed;
    for (const auto& obs : epoch.observations) {
        indexed[{obs.satellite, obs.signal}] = &obs;
    }
    return indexed;
}

inline double signalFrequencyHz(const libgnss::SatelliteId& satellite,
                                libgnss::SignalType signal,
                                const libgnss::GNSSTime& time,
                                const libgnss::NavigationData& nav) {
    const libgnss::Ephemeris* eph = nav.getEphemeris(satellite, time);
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
        case libgnss::SignalType::QZS_L1CA:
            return libgnss::constants::GPS_L1_FREQ;
        case libgnss::SignalType::GPS_L2C:
        case libgnss::SignalType::QZS_L2C:
            return libgnss::constants::GPS_L2_FREQ;
        case libgnss::SignalType::GPS_L5:
        case libgnss::SignalType::QZS_L5:
            return libgnss::constants::GPS_L5_FREQ;
        case libgnss::SignalType::GLO_L1CA:
        case libgnss::SignalType::GLO_L1P:
            if (eph && eph->satellite.system == libgnss::GNSSSystem::GLONASS) {
                return libgnss::constants::GLO_L1_BASE_FREQ +
                    eph->glonass_frequency_channel * libgnss::constants::GLO_L1_STEP_FREQ;
            }
            return libgnss::constants::GLO_L1_BASE_FREQ;
        case libgnss::SignalType::GLO_L2CA:
        case libgnss::SignalType::GLO_L2P:
            if (eph && eph->satellite.system == libgnss::GNSSSystem::GLONASS) {
                return libgnss::constants::GLO_L2_BASE_FREQ +
                    eph->glonass_frequency_channel * libgnss::constants::GLO_L2_STEP_FREQ;
            }
            return libgnss::constants::GLO_L2_BASE_FREQ;
        case libgnss::SignalType::GAL_E1:
            return libgnss::constants::GAL_E1_FREQ;
        case libgnss::SignalType::GAL_E5A:
            return libgnss::constants::GAL_E5A_FREQ;
        case libgnss::SignalType::GAL_E5B:
            return libgnss::constants::GAL_E5B_FREQ;
        case libgnss::SignalType::GAL_E6:
            return libgnss::constants::GAL_E6_FREQ;
        case libgnss::SignalType::BDS_B1I:
            return libgnss::constants::BDS_B1I_FREQ;
        case libgnss::SignalType::BDS_B2I:
            return libgnss::constants::BDS_B2I_FREQ;
        case libgnss::SignalType::BDS_B3I:
            return libgnss::constants::BDS_B3I_FREQ;
        case libgnss::SignalType::BDS_B1C:
            return libgnss::constants::BDS_B1C_FREQ;
        case libgnss::SignalType::BDS_B2A:
            return libgnss::constants::BDS_B2A_FREQ;
        default:
            return 0.0;
    }
}

inline double signalWavelength(const libgnss::SatelliteId& satellite,
                               libgnss::SignalType signal,
                               const libgnss::GNSSTime& time,
                               const libgnss::NavigationData& nav) {
    const double frequency = signalFrequencyHz(satellite, signal, time, nav);
    if (frequency <= 0.0) {
        return 0.0;
    }
    return libgnss::constants::SPEED_OF_LIGHT / frequency;
}

inline bool calculateModeledBaseRange(const libgnss::SatelliteId& satellite,
                                      const libgnss::GNSSTime& time,
                                      double approx_pseudorange,
                                      const libgnss::Vector3d& base_position,
                                      const libgnss::NavigationData& nav,
                                      double& modeled_range) {
    libgnss::Vector3d sat_pos;
    libgnss::Vector3d sat_vel;
    double sat_clk = 0.0;
    double sat_clk_drift = 0.0;

    const double travel_time = approx_pseudorange > 1.0
        ? approx_pseudorange / libgnss::constants::SPEED_OF_LIGHT
        : 0.075;
    libgnss::GNSSTime tx_time = time - travel_time;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    tx_time = tx_time - sat_clk;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    const auto geom = nav.calculateGeometry(base_position, sat_pos);
    if (geom.elevation <= 0.05) {
        return false;
    }

    modeled_range = libgnss::geodist(sat_pos, base_position) +
        libgnss::models::tropDelaySaastamoinen(base_position, geom.elevation);
    return std::isfinite(modeled_range);
}

// Linear-in-range interpolation of a base epoch to `target_time`, straddled
// by two real base epochs `before`/`after`. Same algorithm gnss_solve.cpp
// uses to align an asynchronous base stream to the rover epoch cadence:
// interpolate the modeled-range-relative code/phase/Doppler residual
// linearly in time, rather than the raw observable (which would alias the
// satellite geometry's own nonlinearity into the interpolated epoch).
inline bool interpolateBaseEpoch(const libgnss::ObservationData& before,
                                 const libgnss::ObservationData& after,
                                 const libgnss::GNSSTime& target_time,
                                 const libgnss::Vector3d& base_position,
                                 const libgnss::NavigationData& nav,
                                 libgnss::ObservationData& interpolated_epoch) {
    constexpr double kMaxInterpolationGapSeconds = 2.0;
    const double total_dt = timeDiffSeconds(after.time, before.time);
    if (!std::isfinite(total_dt) || total_dt <= 1e-6 || total_dt > kMaxInterpolationGapSeconds) {
        return false;
    }

    const double alpha = timeDiffSeconds(target_time, before.time) / total_dt;
    if (!std::isfinite(alpha) || alpha < -1e-6 || alpha > 1.0 + 1e-6) {
        return false;
    }

    interpolated_epoch = libgnss::ObservationData(target_time);
    interpolated_epoch.receiver_position =
        before.receiver_position.norm() > 0.0 ? before.receiver_position : after.receiver_position;
    interpolated_epoch.receiver_clock_bias =
        (1.0 - alpha) * before.receiver_clock_bias + alpha * after.receiver_clock_bias;

    const auto before_obs = indexObservations(before);
    const auto after_obs = indexObservations(after);
    struct ModeledRangeTriple {
        bool valid = false;
        double before_m = 0.0;
        double after_m = 0.0;
        double target_m = 0.0;
    };
    std::map<libgnss::SatelliteId, ModeledRangeTriple> modeled_range_cache;

    for (const auto& [key, obs_before_ptr] : before_obs) {
        const auto after_it = after_obs.find(key);
        if (after_it == after_obs.end()) continue;

        const auto& obs_before = *obs_before_ptr;
        const auto& obs_after = *after_it->second;
        const double wavelength = signalWavelength(key.satellite, key.signal, target_time, nav);
        if (wavelength <= 0.0 || !obs_before.has_pseudorange || !obs_after.has_pseudorange) {
            continue;
        }

        auto& modeled = modeled_range_cache[key.satellite];
        if (!modeled.valid && modeled.before_m == 0.0 && modeled.after_m == 0.0 &&
            modeled.target_m == 0.0) {
            if (!calculateModeledBaseRange(key.satellite, before.time, obs_before.pseudorange,
                                           base_position, nav, modeled.before_m) ||
                !calculateModeledBaseRange(key.satellite, after.time, obs_after.pseudorange,
                                           base_position, nav, modeled.after_m)) {
                modeled.before_m = std::numeric_limits<double>::quiet_NaN();
                continue;
            }
            modeled.target_m = modeled.before_m + alpha * (modeled.after_m - modeled.before_m);
            modeled.valid = true;
        }
        if (!modeled.valid) {
            continue;
        }

        libgnss::Observation obs(key.satellite, key.signal);
        obs.valid = obs_before.valid && obs_after.valid;
        obs.lli = alpha < 0.5 ? obs_before.lli : obs_after.lli;
        obs.code = alpha < 0.5 ? obs_before.code : obs_after.code;
        obs.signal_strength = std::max(obs_before.signal_strength, obs_after.signal_strength);
        obs.snr = (1.0 - alpha) * obs_before.snr + alpha * obs_after.snr;
        obs.loss_of_lock = obs_before.loss_of_lock || obs_after.loss_of_lock;

        const double code_residual_before = obs_before.pseudorange - modeled.before_m;
        const double code_residual_after = obs_after.pseudorange - modeled.after_m;
        obs.pseudorange = modeled.target_m +
            code_residual_before + alpha * (code_residual_after - code_residual_before);
        obs.has_pseudorange = std::isfinite(obs.pseudorange);

        if (obs_before.has_carrier_phase && obs_after.has_carrier_phase &&
            (obs_before.lli & 0x01) == 0 && (obs_after.lli & 0x01) == 0 && !obs.loss_of_lock) {
            const double phase_residual_before = obs_before.carrier_phase * wavelength - modeled.before_m;
            const double phase_residual_after = obs_after.carrier_phase * wavelength - modeled.after_m;
            const double phase_residual_target =
                phase_residual_before + alpha * (phase_residual_after - phase_residual_before);
            obs.carrier_phase = (modeled.target_m + phase_residual_target) / wavelength;
            obs.has_carrier_phase = std::isfinite(obs.carrier_phase);
        }

        if (obs_before.has_doppler && obs_after.has_doppler) {
            obs.doppler = obs_before.doppler + alpha * (obs_after.doppler - obs_before.doppler);
            obs.has_doppler = true;
        }

        if (obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler) {
            interpolated_epoch.addObservation(obs);
        }
    }

    return !interpolated_epoch.observations.empty();
}

// Small subset of apps/native/gnss_solve.cpp's `--preset` numeric tables, applied
// directly to an RTKConfig. gnss_solve.cpp keeps its own richer preset
// application (apply*RTKTuningPreset against SolveConfig, with per-flag
// "_set" override tracking across its much larger CLI surface) unchanged;
// this is a separate, smaller helper so gnss_fuse's `--preset` flag can
// request the same tuning numbers without gnss_fuse re-deriving them nor
// gnss_solve's own CLI-specific override bookkeeping being disturbed.
inline bool applyRtkConfigPreset(const std::string& preset,
                                 libgnss::RTKProcessor::RTKConfig& rtk_config) {
    if (preset.empty() || preset == "none") {
        return true;
    }
    if (preset == "survey") {
        rtk_config.ratio_threshold = 3.0;
        rtk_config.ambiguity_ratio_threshold = 3.0;
        rtk_config.enable_ar_filter = false;
        rtk_config.ar_filter_margin = 0.25;
        rtk_config.min_satellites_for_ar = 5;
        rtk_config.min_hold_count = 5;
        rtk_config.hold_ambiguity_ratio_threshold = 2.0;
        return true;
    }
    if (preset == "low-cost") {
        rtk_config.ratio_threshold = 3.0;
        rtk_config.ambiguity_ratio_threshold = 3.0;
        rtk_config.enable_ar_filter = true;
        rtk_config.ar_filter_margin = 0.35;
        rtk_config.min_satellites_for_ar = 6;
        rtk_config.min_hold_count = 8;
        rtk_config.hold_ambiguity_ratio_threshold = 2.5;
        rtk_config.max_position_jump_rate_mps = 30.0;
        rtk_config.max_position_jump_min_m = 5.0;
        rtk_config.min_full_ratio_for_subset_ar = 1.5;
        rtk_config.max_float_prefit_residual_rms_m = 4.0;
        rtk_config.max_float_prefit_residual_max_m = 10.0;
        rtk_config.max_float_prefit_residual_reset_streak = 5;
        return true;
    }
    if (preset == "moving-base") {
        rtk_config.ratio_threshold = 2.8;
        rtk_config.ambiguity_ratio_threshold = 2.8;
        rtk_config.enable_ar_filter = true;
        rtk_config.ar_filter_margin = 0.20;
        rtk_config.min_satellites_for_ar = 6;
        rtk_config.min_hold_count = 8;
        rtk_config.hold_ambiguity_ratio_threshold = 2.4;
        return true;
    }
    if (preset == "odaiba") {
        rtk_config.ratio_threshold = 3.0;
        rtk_config.ambiguity_ratio_threshold = 3.0;
        rtk_config.enable_ar_filter = true;
        rtk_config.ar_filter_margin = 0.35;
        rtk_config.min_satellites_for_ar = 6;
        rtk_config.min_hold_count = 8;
        rtk_config.hold_ambiguity_ratio_threshold = 2.5;
        rtk_config.enable_wide_lane_ar = true;
        rtk_config.wide_lane_acceptance_threshold = 0.12;
        return true;
    }
    return false;
}

}  // namespace libgnss_apps
