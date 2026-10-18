#pragma once

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace libgnss::ppp_internal {

inline constexpr double kDefaultZenithDelayMeters = 2.3;

// Number of Kalman measurement updates PPPProcessor::updateFilter() commits
// per epoch. applyPreciseCorrections() materializes the observation geometry
// once, at the prior position, so every pass after the first re-reads the
// position innovation the state has already absorbed and pushes the position
// again on an already-shrunk covariance, while the clock, troposphere and
// ambiguity rows are re-evaluated and take up whatever the overshoot leaves.
// RTKLIB/MADOCALIB commit one update per epoch (ppp.c pppos() restarts every
// residual-screening pass from rtk->x/rtk->P). A single update linearized at
// the prior is also all the geometry needs: a position error of 100 m changes
// the line of sight by ~5e-6 rad, a sub-millimetre range term.
//
// One update per epoch everywhere: static, --low-dynamics and kinematic;
// broadcast, SP3/CLK, HAS / legacy RTCM SSR and MADOCA (uncombined and
// ionosphere-free). At start-up the repeated push moved a static solution
// tens to hundreds of metres (TSK2 IGS-final first hour 930 m 3D, 30 s
// GPS-only Kamakura 250 m; MADOCA static IF MIZU/ALIC 1 h 250/340 m 3D RMS
// once its SPP anchor blend was off) before the phase rows pulled it back.
inline constexpr int kMeasurementUpdatesPerEpoch = 1;

// Geometry-free and Melbourne-Wubbena cycle-slip detection (RTKLIB
// detslp_gf() / detslp_mw(), used by every RTKLIB PPP mode). Static
// ionosphere-free PPP on broadcast or SP3/CLK products used to rely on the
// receiver LLI flag alone, so a slip without LLI -- typically on a second
// frequency that drops out for a few epochs and comes back with a new
// ambiguity -- was absorbed by the float ambiguity and the position (TSK2
// 2024-01-01: 0.96 m east at the end of the day). SSR ionosphere-free and
// kinematic PPP already used these detectors; CLAS kinematic OSR has its own.
inline bool useCombinationSlipDetection(bool kinematic_mode,
                                        bool clas_kinematic_osr,
                                        bool use_ionosphere_free) {
    return (kinematic_mode && !clas_kinematic_osr) || use_ionosphere_free;
}

// Post-fit residual screening for kinematic PPP. RTKLIB/MADOCALIB ppp.c
// (ppp_res(), THRES_REJECT = 4) excludes the satellite with the largest
// post-fit residual beyond four sigmas and redoes the epoch's update from the
// predicted state. The native kinematic filter had no equivalent (its code
// gate only rejects kilometre-level blunders), so urban NLOS code rows tens of
// metres long were committed and carried into the persistent troposphere and
// float-ambiguity states. The residuals are standardized by their own
// covariance (Baarda w-test) rather than by the measurement sigma alone: the
// native phase sigma (~1 cm, no broadcast signal-in-space term) would otherwise
// make the phase rows of healthy satellites the "worst" rows whenever one code
// outlier bends the solution. Used for kinematic motion (not --low-dynamics),
// which commits one update per epoch; coherent MADOCA keeps the MADOCALIB
// bridge semantics it is pinned against, and the CLAS OSR lane never reaches
// this filter.
inline constexpr double kPostfitRejectSigma = 4.0;

// Position / velocity / acceleration dynamics (RTKLIB ppp-kine with
// pos1-dynamics=on) for the non-CLAS kinematic filter. --low-dynamics and the
// CLAS OSR filter (MRTKLIB dynamics of its own) never use it.
inline bool usePvaDynamics(bool use_pva_dynamics,
                           bool use_dynamics_model,
                           bool kinematic_mode,
                           bool low_dynamics_mode,
                           bool clas_osr_filter) {
    return use_pva_dynamics && use_dynamics_model && kinematic_mode &&
           !low_dynamics_mode && !clas_osr_filter;
}

// RTKLIB udpos_ppp() acceleration process noise: a random walk with
// standard deviation sigma_h (east, north) and sigma_v (up) per sqrt(s),
// defined in the local frame at `receiver_position_ecef` and rotated to ECEF
// (covecef()).
inline Eigen::Matrix3d pvaAccelerationProcessNoiseEcef(
    const Eigen::Vector3d& receiver_position_ecef,
    double sigma_horizontal,
    double sigma_vertical,
    double dt) {
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver_position_ecef, lat, lon, height);
    Eigen::Matrix3d enu_to_ecef;
    enu_to_ecef.col(0) = enu2ecef(Eigen::Vector3d::UnitX(), lat, lon);
    enu_to_ecef.col(1) = enu2ecef(Eigen::Vector3d::UnitY(), lat, lon);
    enu_to_ecef.col(2) = enu2ecef(Eigen::Vector3d::UnitZ(), lat, lon);
    Eigen::Matrix3d q_enu = Eigen::Matrix3d::Zero();
    q_enu(0, 0) = q_enu(1, 1) = sigma_horizontal * sigma_horizontal * std::abs(dt);
    q_enu(2, 2) = sigma_vertical * sigma_vertical * std::abs(dt);
    return enu_to_ecef * q_enu * enu_to_ecef.transpose();
}

// SPP position variance assumed when the seed carries no covariance (m^2).
inline constexpr double kPvaFallbackSeedVariance = 100.0;

// Normalized squared distance d' (P + C)^-1 d between the predicted filter
// position (covariance P) and the SPP seed (covariance C); infinity when the
// sum is not positive definite or an input is not finite.
inline double seedPositionDisagreement(const Eigen::Vector3d& predicted_position,
                                       const Eigen::Matrix3d& predicted_covariance,
                                       const Eigen::Vector3d& seed_position,
                                       const Eigen::Matrix3d& seed_covariance) {
    const Eigen::Vector3d d = predicted_position - seed_position;
    const Eigen::Matrix3d C = predicted_covariance + seed_covariance;
    if (!d.allFinite() || !C.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::LLT<Eigen::Matrix3d> llt(C);
    if (llt.info() != Eigen::Success) {
        return std::numeric_limits<double>::infinity();
    }
    return d.dot(llt.solve(d));
}

// Kalman update of the velocity states with a direct velocity measurement
// (the SPP Doppler velocity) of covariance R. Returns false, leaving the
// state untouched, when the normalized innovation squared exceeds
// chi_square_gate (<= 0 disables the gate) or the inputs are not finite.
inline bool applyVelocityMeasurement(ppp_shared::PPPState& filter_state,
                                     const Eigen::Vector3d& velocity,
                                     const Eigen::Matrix3d& covariance,
                                     double chi_square_gate) {
    const int v = filter_state.vel_index;
    if (v < 0 || v + 3 > filter_state.state.size() || !velocity.allFinite() ||
        !covariance.allFinite()) {
        return false;
    }
    auto& x = filter_state.state;
    auto& P = filter_state.covariance;
    const Eigen::Vector3d innovation = velocity - x.segment(v, 3);
    const Eigen::Matrix3d S = P.block(v, v, 3, 3) + covariance;
    const Eigen::LDLT<Eigen::Matrix3d> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
        return false;
    }
    const double nis = innovation.dot(ldlt.solve(innovation));
    if (!std::isfinite(nis) || (chi_square_gate > 0.0 && nis > chi_square_gate)) {
        return false;
    }
    const Eigen::MatrixXd PHt = P.middleCols(v, 3);  // P H'
    const Eigen::MatrixXd gain = ldlt.solve(PHt.transpose()).transpose();
    x += gain * innovation;
    P -= gain * PHt.transpose();
    P = 0.5 * (P + P.transpose()).eval();
    return true;
}

inline bool usePostfitResidualScreening(bool kinematic_motion,
                                        bool coherent_madoca_ssr,
                                        bool clas_osr_filter) {
    return kinematic_motion && !coherent_madoca_ssr && !clas_osr_filter;
}

// Index of the row with the largest standardized post-fit residual
// w_i = (S^-1 r)_i / sqrt((S^-1)_ii), S = H P H' + R the innovation covariance
// and r the innovations (for a Kalman update the post-fit residual is
// R S^-1 r with covariance R S^-1 R); -1 when every |w_i| <= threshold_sigma.
inline int worstStandardizedResidualRow(const Eigen::VectorXd& innovations,
                                        const Eigen::MatrixXd& innovation_inverse,
                                        double threshold_sigma) {
    const Eigen::VectorXd weighted = innovation_inverse * innovations;
    int worst = -1;
    double worst_ratio = threshold_sigma;
    for (int i = 0; i < weighted.size(); ++i) {
        const double variance = innovation_inverse(i, i);
        if (!(variance > 0.0) || !std::isfinite(weighted(i))) {
            continue;
        }
        const double ratio = std::abs(weighted(i)) / std::sqrt(variance);
        if (ratio > worst_ratio) {
            worst_ratio = ratio;
            worst = i;
        }
    }
    return worst;
}

inline double geometryFreeSlipThresholdMeters(
    bool madoca_per_frequency,
    double configured_threshold_m) {
    constexpr double kDefaultMinimumMeters = 0.5;
    constexpr double kMadocalibMinimumMeters = 0.15;
    return std::max(
        configured_threshold_m,
        madoca_per_frequency
            ? kMadocalibMinimumMeters
            : kDefaultMinimumMeters);
}

inline std::set<SignalType> geometryFreeSlippedSignals(
    const std::map<SignalType, double>& previous_m,
    const std::map<SignalType, double>& current_m,
    double threshold_m) {
    std::set<SignalType> slipped;
    for (const auto& [signal, current] : current_m) {
        const auto previous = previous_m.find(signal);
        if (previous != previous_m.end() &&
            std::isfinite(previous->second) &&
            std::isfinite(current) &&
            std::abs(current - previous->second) > threshold_m) {
            slipped.insert(signal);
        }
    }
    return slipped;
}

inline void clearCarrierIonospherePredictionHistory(
    ppp_shared::PPPAmbiguityInfo& ambiguity) {
    // A geometry-free discontinuity invalidates the carrier-derived temporal
    // ionosphere delta along with the affected ambiguities. Applying that
    // discontinuity to the ionosphere state before re-seeding the ambiguities
    // double-counts the slip; MADOCALIB keeps the ionosphere state unchanged
    // on the reset epoch and establishes a new carrier baseline instead.
    ambiguity.has_last_carrier_ionosphere = false;
}

inline int perFrequencyArMinLockCount(bool madoca_per_frequency,
                                      bool ssr_products_loaded,
                                      int convergence_min_epochs) {
    // MADOCALIB gen_sat_sd() admits every currently valid phase pair.  Its
    // ambiguity search has no separate lock-count gate in coherent SSR mode.
    if (madoca_per_frequency) {
        return 0;
    }
    return ssr_products_loaded
        ? std::min(convergence_min_epochs, 10)
        : convergence_min_epochs;
}

inline bool applyGpsL5MeasurementErrorFactor(
    bool madoca_per_frequency,
    SignalType primary_signal,
    SignalType secondary_signal) {
    const auto is_l5 = [](SignalType signal) {
        return signal == SignalType::GPS_L5 || signal == SignalType::QZS_L5;
    };
    // The generic RTKLIB-compatible path de-weights L5. MADOCALIB's
    // ppp.c::varerr() does not: its per-frequency profile applies the same
    // elevation/system variance to L1, L2/L5, and the additional bands.
    return !madoca_per_frequency &&
           (is_l5(primary_signal) || is_l5(secondary_signal));
}

inline bool madocaGalileoMwSupportsWideLaneAdmission(
    GNSSSystem system,
    double mw_double_difference_cycles,
    int reference_sample_count,
    int satellite_sample_count) {
    constexpr int kMinimumSamples = 60;
    constexpr double kMaximumFractionalCycles = 0.20;
    return system == GNSSSystem::Galileo &&
           reference_sample_count >= kMinimumSamples &&
           satellite_sample_count >= kMinimumSamples &&
           std::isfinite(mw_double_difference_cycles) &&
           std::abs(
               std::round(mw_double_difference_cycles) -
               mw_double_difference_cycles) <
               kMaximumFractionalCycles;
}

inline bool madocaHighAgreementRatioAccepted(bool madoca_per_frequency,
                                             double ratio,
                                             double threshold,
                                             double matching_candidate_rate) {
    constexpr double kRelativeTolerance = 0.01;
    return ratio >= threshold ||
           (madoca_per_frequency &&
            matching_candidate_rate > 0.90 &&
            ratio >= threshold * (1.0 - kRelativeTolerance));
}

inline bool alwaysRestoreArTrialState(PPPProcessor::PPPConfig::ARMethod method) {
    // MADOCALIB runs per-frequency EWL/WL/N1 constraints on xp/Pp, a copy of
    // the float filter.  The trial is never committed to rtk->x/P, including
    // early exits after EWL conditioning but before a usable WL set exists.
    return method == PPPProcessor::PPPConfig::ARMethod::DD_PER_FREQ;
}

struct MadocaIonoConstraintInput {
    SatelliteId satellite;
    int state_index = -1;
    double ionosphere_state_m = 0.0;
    double delay_m = 0.0;
    double std_m = 0.0;
    double age_s = std::numeric_limits<double>::infinity();
};

struct MadocaIonoConstraintRow {
    SatelliteId satellite;
    int state_index = -1;
    double target_m = 0.0;
    double residual_m = 0.0;
    double variance_m2 = 0.0;
    double system_bias_m = 0.0;
};

inline int madocaIonoConstraintSystemSlot(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return 0;
        case GNSSSystem::GLONASS: return 1;
        case GNSSSystem::Galileo: return 2;
        case GNSSSystem::QZSS: return 3;
        default: return -1;
    }
}

inline bool madocaIonoConstraintPositionGatePasses(
    double horizontal_position_std_m,
    double vertical_position_std_m) {
    constexpr double kHorizontalThresholdM = 2.0;
    constexpr double kVerticalThresholdM = 3.0;
    // MADOCALIB applies L6D constraints while the previous position covariance
    // is still loose. It stops only when both non-zero ENU standard deviations
    // are below their convergence thresholds.
    return !(
        horizontal_position_std_m != 0.0 &&
        vertical_position_std_m != 0.0 &&
        horizontal_position_std_m < kHorizontalThresholdM &&
        vertical_position_std_m < kVerticalThresholdM);
}

inline std::vector<MadocaIonoConstraintRow> buildMadocaIonoConstraintRows(
    const std::vector<MadocaIonoConstraintInput>& inputs,
    double horizontal_position_std_m,
    double vertical_position_std_m) {
    constexpr double kMaximumAgeSeconds = 300.0;
    constexpr double kMaximumStdM = 1.0;
    if (!madocaIonoConstraintPositionGatePasses(
            horizontal_position_std_m, vertical_position_std_m)) {
        return {};
    }

    std::array<double, 4> bias_sums{};
    std::array<int, 4> bias_counts{};
    const auto accepted = [&](const MadocaIonoConstraintInput& input) {
        return madocaIonoConstraintSystemSlot(input.satellite.system) >= 0 &&
               input.state_index >= 0 &&
               std::isfinite(input.ionosphere_state_m) &&
               std::isfinite(input.delay_m) &&
               std::isfinite(input.std_m) &&
               input.std_m <= kMaximumStdM &&
               std::isfinite(input.age_s) &&
               std::abs(input.age_s) <= kMaximumAgeSeconds;
    };
    for (const auto& input : inputs) {
        if (!accepted(input)) {
            continue;
        }
        const int slot = madocaIonoConstraintSystemSlot(input.satellite.system);
        bias_sums[static_cast<size_t>(slot)] +=
            input.delay_m - input.ionosphere_state_m;
        ++bias_counts[static_cast<size_t>(slot)];
    }

    std::array<double, 4> system_biases{};
    for (size_t slot = 0; slot < system_biases.size(); ++slot) {
        if (bias_counts[slot] > 0) {
            system_biases[slot] =
                bias_sums[slot] / static_cast<double>(bias_counts[slot]);
        }
    }

    std::vector<MadocaIonoConstraintRow> rows;
    rows.reserve(inputs.size());
    for (const auto& input : inputs) {
        if (!accepted(input)) {
            continue;
        }
        const int slot = madocaIonoConstraintSystemSlot(input.satellite.system);
        const double system_bias = system_biases[static_cast<size_t>(slot)];
        const double target = input.delay_m - system_bias;
        rows.push_back({
            input.satellite,
            input.state_index,
            target,
            target - input.ionosphere_state_m,
            input.std_m * input.std_m,
            system_bias,
        });
    }
    return rows;
}

inline Vector3d recenterPostfitReceiverPosition(
    const Vector3d& corrected_receiver_position,
    const Vector3d& prior_filter_position,
    const Vector3d& updated_filter_position) {
    // Precise corrections materialize the antenna phase-centre position at
    // the epoch prior.  Only absolute positions should be recentered; a zero
    // vector means the measurement model must obtain its receiver position
    // from the filter state.
    if (corrected_receiver_position.norm() <= 1000.0) {
        return corrected_receiver_position;
    }
    return corrected_receiver_position +
           (updated_filter_position - prior_filter_position);
}

inline double madocaIonosphereScale(double frequency_hz) {
    if (!(frequency_hz > 0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double ratio = constants::GPS_L1_FREQ / frequency_hz;
    return ratio * ratio;
}

inline double madocaIonosphereStateFromPrimaryMeters(
    double primary_frequency_ionosphere_m,
    double primary_frequency_hz) {
    const double primary_scale = madocaIonosphereScale(primary_frequency_hz);
    if (!std::isfinite(primary_scale) || !(primary_scale > 0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return primary_frequency_ionosphere_m / primary_scale;
}

inline double madocaCorrectedCodeIonosphereStateMeters(
    double fallback_primary_ionosphere_m,
    double corrected_primary_code_m,
    double corrected_secondary_code_m,
    double primary_frequency_hz,
    double secondary_frequency_hz) {
    double primary_ionosphere_m = fallback_primary_ionosphere_m;
    if (primary_frequency_hz > 0.0 &&
        secondary_frequency_hz > 0.0 &&
        std::isfinite(corrected_primary_code_m) &&
        std::isfinite(corrected_secondary_code_m)) {
        const double ratio = primary_frequency_hz / secondary_frequency_hz;
        const double denominator = 1.0 - ratio * ratio;
        if (std::abs(denominator) > 1e-6) {
            primary_ionosphere_m =
                (corrected_primary_code_m - corrected_secondary_code_m) /
                denominator;
        }
    }
    return madocaIonosphereStateFromPrimaryMeters(
        primary_ionosphere_m, primary_frequency_hz);
}

inline double madocaCarrierIonosphereMeters(double phase_l1_m,
                                            double phase_l2_m,
                                            double frequency_l1_hz,
                                            double frequency_l2_hz) {
    const double scale_l1 = madocaIonosphereScale(frequency_l1_hz);
    const double scale_l2 = madocaIonosphereScale(frequency_l2_hz);
    if (!std::isfinite(scale_l1) || !std::isfinite(scale_l2)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double denominator = scale_l1 - scale_l2;
    if (std::abs(denominator) < 1e-12) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // MADOCALIB udiono_ppp(): the estimated STEC state is referenced to the
    // fixed GPS L1 frequency, including for non-GPS primary signals.
    return -(phase_l1_m - phase_l2_m) / denominator;
}

inline double madocaCarrierIonosphereMetersExcludingWindup(
    double corrected_phase_l1_m,
    double corrected_phase_l2_m,
    double wavelength_l1_m,
    double wavelength_l2_m,
    double windup_cycles,
    double frequency_l1_hz,
    double frequency_l2_hz) {
    // MADOCALIB udiono_ppp() calls corr_meas(..., phw=0). Native corrected
    // phases already have phw*lambda removed, so restore that term before
    // deriving the temporal carrier-ionosphere increment.
    return madocaCarrierIonosphereMeters(
        corrected_phase_l1_m + windup_cycles * wavelength_l1_m,
        corrected_phase_l2_m + windup_cycles * wavelength_l2_m,
        frequency_l1_hz,
        frequency_l2_hz);
}

inline double madocaIonosphereProcessVariance(double zenith_variance_per_second,
                                              double elevation_rad,
                                              double dt_seconds) {
    constexpr double kMinimumElevationRad = 5.0 * M_PI / 180.0;
    const double sin_elevation = std::sin(std::max(elevation_rad,
                                                   kMinimumElevationRad));
    return zenith_variance_per_second * std::abs(dt_seconds) /
           (sin_elevation * sin_elevation);
}

inline double madocaGlonassCodeIfbVariance(bool madoca_per_frequency,
                                           GNSSSystem system) {
    // MADOCALIB ppp_res(): VAR_GLO_IFB=SQR(0.6) is added to GLONASS
    // pseudorange rows only. Carrier rows never call this helper.
    return madoca_per_frequency && system == GNSSSystem::GLONASS
        ? 0.6 * 0.6
        : 0.0;
}

inline double initialTroposphereVariance(bool madoca_per_frequency,
                                         bool broadcast_model,
                                         double configured_variance) {
    // MADOCALIB ppp.c initializes estimated ZTD with VAR_ZTD=SQR(0.12).
    if (madoca_per_frequency) {
        return 0.12 * 0.12;
    }
    return broadcast_model ? configured_variance : 25.0;
}

inline double initialIonosphereVariance(bool madoca_per_frequency,
                                        double configured_override,
                                        double configured_variance) {
    if (configured_override > 0.0) {
        return configured_override;
    }
    // MADOCALIB ppp.c initializes every estimated STEC state with
    // VAR_IONO=SQR(60.0).
    return madoca_per_frequency ? 60.0 * 60.0 : configured_variance;
}

inline std::string trimCopy(const std::string& text) {
    const auto is_not_space = [](unsigned char ch) {
        return !std::isspace(ch);
    };
    const auto first_it = std::find_if(text.begin(), text.end(), is_not_space);
    if (first_it == text.end()) {
        return "";
    }
    const auto last_it = std::find_if(text.rbegin(), text.rend(), is_not_space).base();
    return std::string(first_it, last_it);
}

inline std::string normalizeAntennaType(const std::string& antenna_type) {
    std::string normalized = trimCopy(antenna_type);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return normalized;
}

inline bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

inline const char* signalFamilyName(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return "GPS_L1CA";
        case SignalType::GPS_L1P: return "GPS_L1P";
        case SignalType::GPS_L2P: return "GPS_L2P";
        case SignalType::GPS_L2C: return "GPS_L2C";
        case SignalType::GPS_L5: return "GPS_L5";
        case SignalType::GLO_L1CA: return "GLO_L1CA";
        case SignalType::GLO_L1P: return "GLO_L1P";
        case SignalType::GLO_L2CA: return "GLO_L2CA";
        case SignalType::GLO_L2P: return "GLO_L2P";
        case SignalType::GAL_E1: return "GAL_E1";
        case SignalType::GAL_E5A: return "GAL_E5A";
        case SignalType::GAL_E5B: return "GAL_E5B";
        case SignalType::GAL_E6: return "GAL_E6";
        case SignalType::BDS_B1I: return "BDS_B1I";
        case SignalType::BDS_B2I: return "BDS_B2I";
        case SignalType::BDS_B3I: return "BDS_B3I";
        case SignalType::BDS_B1C: return "BDS_B1C";
        case SignalType::BDS_B2A: return "BDS_B2A";
        case SignalType::QZS_L1CA: return "QZS_L1CA";
        case SignalType::QZS_L2C: return "QZS_L2C";
        case SignalType::QZS_L5: return "QZS_L5";
        case SignalType::SIGNAL_TYPE_COUNT: return "UNKNOWN";
    }
    return "UNKNOWN";
}

inline std::vector<SignalType> primarySignals(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return {SignalType::GPS_L1CA};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L1CA, SignalType::GLO_L1P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E1};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B1I, SignalType::BDS_B1C};
        case GNSSSystem::QZSS: return {SignalType::QZS_L1CA};
        default: return {};
    }
}

inline std::vector<SignalType> secondarySignals(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return {SignalType::GPS_L2C, SignalType::GPS_L5};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L2CA, SignalType::GLO_L2P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E5A, SignalType::GAL_E5B, SignalType::GAL_E6};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B2I, SignalType::BDS_B2A, SignalType::BDS_B3I};
        case GNSSSystem::QZSS: return {SignalType::QZS_L2C, SignalType::QZS_L5};
        default: return {};
    }
}

// Ionosphere-free PPP whose satellite clocks come from the broadcast
// ephemeris, with no precise, SSR or DCB/OSB products supplying code biases.
// Ionosphere-free PPP on SP3/CLK products (no SSR) needs both frequencies of a
// satellite. With only the primary signal the code row would carry the full
// first-order ionospheric delay (no broadcast Klobuchar term is applied on this
// path) and the raw L1 carrier phase would be tied to the satellite's
// ionosphere-free ambiguity state, so a satellite that drops its second
// frequency for a few epochs corrupts that ambiguity (metres) when the second
// frequency returns. RTKLIB/MADOCALIB (IONOOPT_IFLC) skip such a satellite;
// do the same. Broadcast and SSR paths keep their own single-frequency rows.
inline bool dropSingleFrequencyPreciseProductSatellite(bool precise_products_loaded,
                                                       bool ssr_products_loaded) {
    return precise_products_loaded && !ssr_products_loaded;
}

inline bool broadcastClockIonosphereFree(bool use_ionosphere_free,
                                         bool precise_products_loaded,
                                         bool ssr_products_loaded,
                                         bool dcb_products_loaded) {
    return use_ionosphere_free && !precise_products_loaded &&
           !ssr_products_loaded && !dcb_products_loaded;
}

// Broadcast (D1/D2) BeiDou satellite clocks are referenced to B3I; the B1I and
// B2I codes carry the broadcast group delays TGD1 / TGD2 (BDS-SIS-ICD-2.1
// 5.2.4.10, RTKLIB pntpos.c prange()), which reach -45 ns (-13.5 m) on BDS-3
// satellites and are amplified by the ionosphere-free combination. The BDS-3
// B1C / B2a signals have no group delay in D1/D2 (only in B-CNAV1/2), so they
// cannot be used against a D1/D2 clock.
inline const std::vector<SignalType>& broadcastBeiDouPrimarySignals() {
    static const std::vector<SignalType> signals{SignalType::BDS_B1I};
    return signals;
}

// BDS-3 satellites (C19 and above) do not transmit B2I: their D1 message is
// on B1I and B3I only (BDS-SIS-ICD-B1I-3.0, BDS-SIS-ICD-B3I-1.0), and the
// RINEX band-7 code a receiver logs for them (C7D / C7P / C7Z) is B2b, which
// shares the B2I carrier (and so the BDS_B2I slot here) but whose group delay
// is only broadcast in B-CNAV3 (TGD_B2bI). Their D1 TGD2 field repeats TGD1
// (all BDS-3 satellites in the IGS merged BRDC files of 2025-046 / 2025-233),
// so pairing B1I with B2b and removing TGD1 / TGD2 left a
// 1.49 * (TGD1 - TGD_B2b) error of up to tens of metres per satellite. BDS-3
// uses B1I / B3I, whose only group delay is TGD1; BDS-2 keeps B2I / TGD2.
inline bool isBeiDou3Satellite(const SatelliteId& sat) {
    return sat.system == GNSSSystem::BeiDou && sat.prn >= 19;
}

inline const std::vector<SignalType>& broadcastBeiDouSecondarySignals(
    const SatelliteId& sat) {
    static const std::vector<SignalType> bds2_signals{
        SignalType::BDS_B2I, SignalType::BDS_B3I};
    static const std::vector<SignalType> bds3_signals{SignalType::BDS_B3I};
    return isBeiDou3Satellite(sat) ? bds3_signals : bds2_signals;
}

// The RINEX reader keeps one policy-selected secondary observation per
// satellite, with band 7 ahead of band 6, so a BDS-3 satellite logging B2b
// keeps B2b there and its B3I code is only in the per-tracking-code
// observations. Return the B3I (C6I / C6Q / C6X) observation of a BDS-3
// satellite, or the first broadcast secondary signal of a BDS-2 satellite.
// require_carrier selects an observation with a carrier phase (slip
// detection) instead of a pseudorange.
inline const Observation* findBroadcastBeiDouSecondaryObservation(
    const ObservationData& obs, const SatelliteId& sat, bool require_carrier) {
    const auto usable = [require_carrier](const Observation* candidate) {
        if (candidate == nullptr || !candidate->valid) {
            return false;
        }
        if (require_carrier) {
            return candidate->has_carrier_phase &&
                   std::isfinite(candidate->carrier_phase);
        }
        return candidate->has_pseudorange && candidate->pseudorange > 0.0 &&
               std::isfinite(candidate->pseudorange);
    };
    for (const auto signal : broadcastBeiDouSecondarySignals(sat)) {
        const Observation* candidate = obs.getObservation(sat, signal);
        if (usable(candidate)) {
            return candidate;
        }
    }
    if (!isBeiDou3Satellite(sat)) {
        return nullptr;
    }
    for (const char* tracking_code : {"6I", "6Q", "6X"}) {
        const Observation* candidate =
            obs.getRinexTrackingObservation(sat, tracking_code);
        if (usable(candidate)) {
            return candidate;
        }
    }
    return nullptr;
}

inline double broadcastBeiDouGroupDelayMeters(SignalType signal, const Ephemeris& eph) {
    switch (signal) {
        case SignalType::BDS_B1I:
            return constants::SPEED_OF_LIGHT * eph.tgd;
        case SignalType::BDS_B2I:
            return constants::SPEED_OF_LIGHT * eph.tgd_secondary;
        default:
            return 0.0;  // B3I is the clock reference.
    }
}

// Broadcast group delay (metres) to subtract from a single-frequency primary
// code (the satellite's secondary frequency is missing this epoch). GPS/QZSS
// LNAV and Galileo clocks are ionosphere-free references, so their L1 / E1
// code carries TGD / BGD; this is the SPP single-frequency model
// (spp.cpp groupDelayCorrectionMeters, legacy Galileo field). GLONASS
// broadcasts no L1 group delay.
inline double broadcastSingleFrequencyGroupDelayMeters(const SatelliteId& satellite,
                                                       SignalType signal,
                                                       const Ephemeris& eph) {
    switch (satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
        case GNSSSystem::Galileo:
            return constants::SPEED_OF_LIGHT * eph.tgd;
        case GNSSSystem::BeiDou:
            return broadcastBeiDouGroupDelayMeters(signal, eph);
        default:
            return 0.0;
    }
}

// Error factor of the broadcast (Klobuchar) ionosphere model applied to the
// single-frequency rows of a broadcast ionosphere-free solution: sigma = 0.5 *
// delay, as RTKLIB ERR_BRDCI.
inline constexpr double kBroadcastIonosphereErrorFactor = 0.5;

inline const Observation* findObservationForSignals(const ObservationData& obs,
                                                    const SatelliteId& sat,
                                                    const std::vector<SignalType>& candidates) {
    for (const auto signal : candidates) {
        const Observation* candidate = obs.getObservation(sat, signal);
        if (candidate == nullptr || !candidate->valid || !candidate->has_pseudorange) {
            continue;
        }
        if (candidate->pseudorange <= 0.0 || !std::isfinite(candidate->pseudorange)) {
            continue;
        }
        return candidate;
    }
    return nullptr;
}

inline const Observation* findCarrierObservationForSignals(const ObservationData& obs,
                                                           const SatelliteId& sat,
                                                           const std::vector<SignalType>& candidates) {
    return ppp_utils::findCarrierObservation(obs, sat, candidates);
}

inline Vector3d ssrRacToEcef(const Vector3d& position_ecef,
                             const Vector3d& velocity_ecef,
                             const Vector3d& rac_correction) {
    return ppp_utils::ssrRacToEcef(position_ecef, velocity_ecef, rac_correction);
}

inline double safeVariance(double variance, double floor_value) {
    if (!std::isfinite(variance) || variance <= 0.0) {
        return floor_value;
    }
    return std::max(variance, floor_value);
}

}  // namespace libgnss::ppp_internal
