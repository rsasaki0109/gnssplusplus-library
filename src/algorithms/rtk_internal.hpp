#pragma once

// Shared file-local helpers for the RTK implementation TUs.
// Extracted from the former monolithic rtk.cpp anonymous namespace;
// every helper is inline and internal to the rtk_internal namespace.

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <string>

namespace libgnss {
namespace rtk_internal {

constexpr GNSSSystem kRTKSupportedSystems[] = {
    GNSSSystem::GPS,
    GNSSSystem::GLONASS,
    GNSSSystem::Galileo,
    GNSSSystem::BeiDou,
    GNSSSystem::QZSS,
};

constexpr double kGlonassHWBiasInitialVariance = 1.0;  // (m/MHz)^2
constexpr double kGlonassHWBiasProcessNoise = 1e-12;   // (m/MHz)^2 / s

// Delegate to extracted modules
inline double tropModel(const Vector3d& pos_ecef, double elevation) {
    return models::tropDelaySaastamoinen(pos_ecef, elevation);
}

// Geometric range with Sagnac correction (delegated to coordinates.hpp)
inline double geodist_range(const Vector3d& rs, const Vector3d& rr) {
    return geodist(rs, rr);
}

inline bool isAmbiguityResolutionSystem(const RTKProcessor::RTKConfig& config, GNSSSystem system) {
    return system == GNSSSystem::GPS ||
           (system == GNSSSystem::GLONASS &&
            config.glonass_ar_mode != RTKProcessor::RTKConfig::GlonassARMode::OFF) ||
           system == GNSSSystem::Galileo ||
           system == GNSSSystem::BeiDou ||
           system == GNSSSystem::QZSS;
}

inline bool usesHoldAmbiguitySystem(const RTKProcessor::RTKConfig& config, GNSSSystem system) {
    return isAmbiguityResolutionSystem(config, system);
}

inline bool requiresMatchedCarrierWavelength(const RTKProcessor::RTKConfig& config, GNSSSystem system) {
    return !(system == GNSSSystem::GLONASS &&
             config.glonass_ar_mode != RTKProcessor::RTKConfig::GlonassARMode::OFF);
}

inline bool usesGlonassAutocal(const RTKProcessor::RTKConfig& config) {
    return config.enable_glonass &&
           config.glonass_ar_mode == RTKProcessor::RTKConfig::GlonassARMode::AUTOCAL;
}

inline bool usesEstimatedIono(const RTKProcessor::RTKConfig& config) {
    return config.ionoopt == RTKProcessor::RTKConfig::IonoOpt::EST;
}

inline bool isMovingBasePositionMode(const RTKProcessor::RTKConfig& config) {
    return config.position_mode == RTKProcessor::RTKConfig::PositionMode::MOVING_BASE;
}

inline bool isDynamicPositionMode(const RTKProcessor::RTKConfig& config) {
    return config.position_mode != RTKProcessor::RTKConfig::PositionMode::STATIC;
}

inline bool isBeiDouGeoSatellite(const SatelliteId& sat) {
    return signal_policy::isBeiDouGeoSatellite(sat);
}

inline bool isUsableRTKSatellite(const SatelliteId& sat) {
    // Start with MEO/IGSO BeiDou support. GEO still needs tighter handling.
    return !isBeiDouGeoSatellite(sat);
}

inline bool isEnabledRTKSystem(const RTKProcessor::RTKConfig& config, GNSSSystem system) {
    if (system == GNSSSystem::GLONASS) {
        return config.enable_glonass;
    }
    if (system == GNSSSystem::BeiDou) {
        return config.enable_beidou;
    }
    return true;
}

inline double combinedSnrDbHz(double rover_snr, double base_snr) {
    const bool rover_valid = std::isfinite(rover_snr) && rover_snr > 0.0;
    const bool base_valid = std::isfinite(base_snr) && base_snr > 0.0;
    if (rover_valid && base_valid) {
        return std::min(rover_snr, base_snr);
    }
    if (rover_valid) {
        return rover_snr;
    }
    if (base_valid) {
        return base_snr;
    }
    return 0.0;
}

inline bool isPrimaryRTKSignal(GNSSSystem system, SignalType signal) {
    return signal_policy::isPrimarySignal(system, signal);
}

inline bool isSecondaryRTKSignal(GNSSSystem system, SignalType signal) {
    return signal_policy::isSecondarySignal(system, signal);
}

// Phase 18 Step 3: L5-class signal slot, distinct from L2 secondary.
inline bool isL5RTKSignal(GNSSSystem system, SignalType signal) {
    return signal_policy::isL5Signal(system, signal);
}

inline int signalSelectionPriority(GNSSSystem system, SignalType signal, bool primary) {
    return signal_policy::signalPriority(system, signal, primary);
}

inline const Observation* selectPreferredObservation(
    GNSSSystem system,
    const std::vector<const Observation*>& candidates,
    bool primary) {
    const Observation* best = nullptr;
    int best_priority = 1000;
    for (const auto* obs : candidates) {
        if (obs == nullptr) continue;
        const int priority = signalSelectionPriority(system, obs->signal, primary);
        if (priority < best_priority) {
            best = obs;
            best_priority = priority;
        }
    }
    return best;
}

inline bool selectMatchedObservationPair(
    GNSSSystem system,
    const std::vector<const Observation*>& rover_candidates,
    const std::vector<const Observation*>& base_candidates,
    bool primary,
    const Observation*& rover_selected,
    const Observation*& base_selected) {
    rover_selected = nullptr;
    base_selected = nullptr;
    int best_priority = 1000;

    for (const auto* rover_obs : rover_candidates) {
        if (rover_obs == nullptr) continue;
        for (const auto* base_obs : base_candidates) {
            if (base_obs == nullptr) continue;
            if (rover_obs->signal != base_obs->signal) continue;
            const int priority = signalSelectionPriority(system, rover_obs->signal, primary);
            if (priority < best_priority) {
                rover_selected = rover_obs;
                base_selected = base_obs;
                best_priority = priority;
            }
        }
    }
    return rover_selected != nullptr && base_selected != nullptr;
}

inline double ionoFreeCoeff1(double f1, double f2) {
    const double denom = f1 * f1 - f2 * f2;
    return std::abs(denom) > 0.0 ? (f1 * f1) / denom : 0.0;
}

inline double ionoFreeCoeff2(double f1, double f2) {
    const double denom = f1 * f1 - f2 * f2;
    return std::abs(denom) > 0.0 ? -(f2 * f2) / denom : 0.0;
}

inline double wideLaneWavelength(double f1, double f2) {
    const double denom = f1 - f2;
    return std::abs(denom) > 0.0 ? constants::SPEED_OF_LIGHT / denom : 0.0;
}

inline double narrowLaneWavelength(double f1, double f2) {
    const double c1 = ionoFreeCoeff1(f1, f2);
    const double c2 = ionoFreeCoeff2(f1, f2);
    if (c1 == 0.0 && c2 == 0.0) {
        return 0.0;
    }
    return c1 * (constants::SPEED_OF_LIGHT / f1) + c2 * (constants::SPEED_OF_LIGHT / f2);
}

inline double ionoFrequencyScale(int freq, double l1_frequency_hz, double current_frequency_hz) {
    if (freq == 0 || l1_frequency_hz <= 0.0 || current_frequency_hz <= 0.0) {
        return 1.0;
    }
    const double ratio = l1_frequency_hz / current_frequency_hz;
    return ratio * ratio;
}

inline double distanceToNearestInteger(double value) {
    return std::abs(value - std::round(value));
}

inline bool applyAmbiguityConstraintUpdate(VectorXd& head_state,
                                    VectorXd& dd_float,
                                    MatrixXd& Qb,
                                    MatrixXd& Qab,
                                    int lhs_index,
                                    int rhs_index,
                                    double fixed_difference,
                                    double measurement_variance) {
    if (lhs_index < 0 || rhs_index < 0 ||
        lhs_index >= dd_float.size() || rhs_index >= dd_float.size() ||
        lhs_index >= Qb.rows() || rhs_index >= Qb.rows() ||
        lhs_index >= Qb.cols() || rhs_index >= Qb.cols() ||
        lhs_index >= Qab.cols() || rhs_index >= Qab.cols()) {
        return false;
    }

    VectorXd h = VectorXd::Zero(dd_float.size());
    h(lhs_index) = 1.0;
    h(rhs_index) = -1.0;
    const VectorXd Qb_h = Qb * h;
    const auto h_Qb = h.transpose() * Qb;
    const VectorXd Qab_h = Qab * h;
    const double innovation_variance =
        h.dot(Qb_h) + std::max(measurement_variance, 1e-6);
    if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0) {
        return false;
    }

    const double innovation = fixed_difference - h.dot(dd_float);
    dd_float += (Qb_h / innovation_variance) * innovation;
    head_state += (Qab_h / innovation_variance) * innovation;
    Qb -= (Qb_h * h_Qb) / innovation_variance;
    Qab -= (Qab_h * h_Qb) / innovation_variance;
    Qb = (Qb + Qb.transpose()) * 0.5;
    for (int i = 0; i < Qb.rows(); ++i) {
        if (Qb(i, i) < 1e-6) {
            Qb(i, i) = 1e-6;
        }
    }
    return dd_float.allFinite() && head_state.allFinite() &&
           Qb.allFinite() && Qab.allFinite();
}

inline double glonassInterChannelBiasMeters(const RTKProcessor::RTKConfig& config,
                                     GNSSSystem ref_system,
                                     GNSSSystem sat_system,
                                     double ref_frequency_hz,
                                     double sat_frequency_hz,
                                     int freq) {
    if (ref_system != GNSSSystem::GLONASS || sat_system != GNSSSystem::GLONASS) {
        return 0.0;
    }
    if (ref_frequency_hz <= 0.0 || sat_frequency_hz <= 0.0) {
        return 0.0;
    }
    const double df_mhz = (ref_frequency_hz - sat_frequency_hz) / 1e6;
    const double slope = (freq == 0) ? config.glonass_icb_l1_m_per_mhz
                                     : config.glonass_icb_l2_m_per_mhz;
    return slope * df_mhz;
}

inline SPPProcessor::SPPConfig makeRTKSppConfig(const RTKProcessor::RTKConfig& rtk_config) {
    SPPProcessor::SPPConfig config;
    config.use_multi_constellation = true;
    config.enable_glonass = rtk_config.enable_glonass;
    config.enable_beidou = rtk_config.enable_beidou;
    return config;
}

inline SPPProcessor makeRTKSppProcessor(const RTKProcessor::RTKConfig& rtk_config) {
    return SPPProcessor(makeRTKSppConfig(rtk_config));
}


}  // namespace rtk_internal
}  // namespace libgnss
