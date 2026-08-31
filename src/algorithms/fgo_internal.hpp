#pragma once

// Shared file-local helpers for the FGO implementation TUs.
// Extracted from the former monolithic fgo.cpp anonymous namespace; every
// helper is inline and internal to the fgo_internal namespace.

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/doppler_contract.hpp>

namespace libgnss {

#ifdef GNSSPP_HAS_GTSAM
// Implemented in fgo_gtsam_backend.cpp. Kept as a free function (rather than
// another FGOProcessor member) so all GTSAM includes/types stay confined to
// that translation unit; fgo.cpp itself never includes any GTSAM header.
FGOProcessor::FGOResult optimizeProblemWithGtsam(
    const FGOProcessor::FGOProblem& problem,
    const FGOProcessor::FGOConfig& config,
    FGOProcessor::FGOResult result);
#endif

namespace fgo_internal {

inline bool isPrimaryFgoSignal(SignalType signal, bool use_multi_constellation) {
    if (!use_multi_constellation) {
        return signal == SignalType::GPS_L1CA;
    }

    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GLO_L1CA:
        case SignalType::GAL_E1:
        case SignalType::BDS_B1I:
        case SignalType::BDS_B1C:
        case SignalType::QZS_L1CA:
            return true;
        default:
            return false;
    }
}

// Multi-frequency DD gate: primary-band signals are always eligible (same as
// isPrimaryFgoSignal above). When use_multi_frequency_double_difference is
// also enabled (and multi-constellation is on), secondary-band signals
// (GPS L2C/L5, Galileo E5A/E5B/E6, BeiDou B2I/B3I/B2A, QZSS L2C/L5, ...) are
// eligible too, keyed by the satellite's system via signal_policy. This is
// the single choke point that previously restricted buildPseudorangeProblem /
// buildDoubleDifferenceProblem to L1/E1/B1I-only DD factors.
inline bool isEligibleFgoSignal(const SatelliteId& satellite,
                         SignalType signal,
                         bool use_multi_constellation,
                         bool use_multi_frequency_double_difference) {
    if (isPrimaryFgoSignal(signal, use_multi_constellation)) {
        return true;
    }
    if (!use_multi_constellation || !use_multi_frequency_double_difference) {
        return false;
    }
    return signal_policy::isSecondarySignal(satellite.system, signal);
}

// --- MF hygiene: per-receiver secondary-band observation-code alignment ---
//
// Port of the cssrlib/inuex35 (RTKLIB-style) signal-selection policy
// (tightly-coupled-gnss-imu-fgo utils/sig_autodetect.py): each RECEIVER uses
// exactly ONE RINEX observation code per (system, band) for the whole file,
// so every satellite of a system contributes the same code per band and the
// between-satellite (single) difference cancels the receiver-side inter-code
// bias exactly. Without this, the RINEX reader's per-satellite first-nonzero
// column selection MIXES codes across satellites within one band (observed on
// tokyo1: rover GPS L2 = C2W for 2527 sat-epochs but C2L for the satellites/
// epochs where C2W is empty; base BeiDou B2I = C7D vs rover C7I), leaving
// uncancelled inter-code/DCB and carrier phase-alignment biases in the DD --
// the measured multi-frequency FLOAT blowup. Satellites whose selected column
// is a different code simply drop that band (like-vs-like or nothing),
// exactly like cssrlib decoding only the chosen column.
//
// The preferred code per (system, SignalType) is chosen per receiver from the
// codes actually observed in its data, ordered by an RTKLIB-style tracking-
// code priority string (rtkcmn.c codepris), ties broken by observation count.
// Applied to SECONDARY bands only: the primary band (L1/E1/B1I) single-
// frequency path is measured-good and stays bit-identical.

// RTKLIB-style per-(system, band) tracking-code priority (earlier = higher).
inline const char* trackingCodePriorityString(GNSSSystem system, int band) {
    switch (system) {
        case GNSSSystem::GPS:
            switch (band) {
                case 1: return "CPYWMNSL";
                case 2: return "PYWCMNDLSX";
                case 5: return "IQX";
                default: return "";
            }
        case GNSSSystem::GLONASS:
            switch (band) {
                case 1: return "PC";
                case 2: return "PC";
                case 3: return "IQX";
                default: return "";
            }
        case GNSSSystem::Galileo:
            switch (band) {
                case 1: return "CABXZ";
                case 5: return "IQX";
                case 6: return "ABCXZ";
                case 7: return "IQX";
                case 8: return "IQX";
                default: return "";
            }
        case GNSSSystem::BeiDou:
            switch (band) {
                case 1: return "DPX";
                case 2: return "IQX";
                case 5: return "DPX";
                case 6: return "IQX";
                case 7: return "IQXDZ";
                case 8: return "DPX";
                default: return "";
            }
        case GNSSSystem::QZSS:
            switch (band) {
                case 1: return "CLSXZ";
                case 2: return "LSX";
                case 5: return "IQXDPZ";
                case 6: return "SXZ";
                default: return "";
            }
        case GNSSSystem::NavIC:
            switch (band) {
                case 5: return "ABCX";
                default: return "";
            }
        default:
            return "";
    }
}

inline int trackingCodePriority(GNSSSystem system, const std::string& obs_type) {
    if (obs_type.size() < 3) {
        return 100;
    }
    const int band = signal_policy::rinexBand(obs_type);
    const char code = obs_type[2];
    const char* priorities = trackingCodePriorityString(system, band);
    for (int i = 0; priorities[i] != '\0'; ++i) {
        if (priorities[i] == code) {
            return i;
        }
    }
    return 100;
}

// One preferred tracking-code char per (system, secondary SignalType),
// derived from the receiver's own observations (single pre-scan pass).
using SecondaryCodeTable = std::map<std::pair<GNSSSystem, SignalType>, char>;

inline SecondaryCodeTable buildSecondaryCodeTable(
    const std::vector<ObservationData>& epochs) {
    struct CodeStats {
        int priority = 100;
        std::size_t count = 0;
    };
    std::map<std::pair<GNSSSystem, SignalType>, std::map<char, CodeStats>> stats;
    for (const auto& epoch : epochs) {
        for (const auto& observation : epoch.observations) {
            if (!observation.valid || !observation.has_pseudorange) {
                continue;
            }
            const GNSSSystem system = observation.satellite.system;
            if (!signal_policy::isSecondarySignal(system, observation.signal)) {
                continue;
            }
            const std::string& obs_type = observation.exactBiasObservationType();
            if (obs_type.size() < 3) {
                continue;
            }
            auto& entry = stats[{system, observation.signal}][obs_type[2]];
            entry.priority = trackingCodePriority(system, obs_type);
            ++entry.count;
        }
    }

    SecondaryCodeTable table;
    for (const auto& [key, codes] : stats) {
        char best_code = '\0';
        int best_priority = 101;
        std::size_t best_count = 0;
        for (const auto& [code, s] : codes) {
            if (s.priority < best_priority ||
                (s.priority == best_priority && s.count > best_count)) {
                best_code = code;
                best_priority = s.priority;
                best_count = s.count;
            }
        }
        if (best_code != '\0') {
            table[key] = best_code;
        }
    }
    return table;
}

// True when the observation either is primary-band (never filtered here) or
// carries the receiver's preferred tracking code for its (system, signal).
inline bool passesSecondaryCodeAlignment(const Observation& observation,
                                  const SecondaryCodeTable* table) {
    if (table == nullptr) {
        return true;
    }
    const GNSSSystem system = observation.satellite.system;
    if (!signal_policy::isSecondarySignal(system, observation.signal)) {
        return true;
    }
    const auto it = table->find({system, observation.signal});
    if (it == table->end()) {
        return true;
    }
    const std::string& obs_type = observation.exactBiasObservationType();
    return obs_type.size() >= 3 && obs_type[2] == it->second;
}

inline GNSSSystem clockBiasGroup(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return GNSSSystem::GPS;
        case GNSSSystem::Galileo:
        case GNSSSystem::BeiDou:
        case GNSSSystem::GLONASS:
        case GNSSSystem::NavIC:
            return system;
        default:
            return GNSSSystem::UNKNOWN;
    }
}

inline bool usesSeparateClockBias(GNSSSystem group) {
    return group != GNSSSystem::UNKNOWN && group != GNSSSystem::GPS;
}

inline double groupDelayCorrectionMeters(const Observation& observation, const Ephemeris& eph) {
    switch (observation.satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
        case GNSSSystem::Galileo:
            return eph.tgd * constants::SPEED_OF_LIGHT;
        case GNSSSystem::BeiDou:
            switch (observation.signal) {
                case SignalType::BDS_B1I:
                case SignalType::BDS_B1C:
                    return eph.tgd * constants::SPEED_OF_LIGHT;
                case SignalType::BDS_B2I:
                case SignalType::BDS_B2A:
                    return eph.tgd_secondary * constants::SPEED_OF_LIGHT;
                default:
                    return 0.0;
            }
        default:
            return 0.0;
    }
}

inline bool isHealthyForPositioning(const Observation& observation, const Ephemeris& eph) {
    int sv_health = static_cast<int>(eph.health);
    if (observation.satellite.system == GNSSSystem::QZSS) {
        sv_health &= 0xFE;
    }
    return sv_health == 0;
}

inline Vector3d earthRotationCorrected(const Vector3d& satellite_position,
                                const Vector3d& receiver_position) {
    const double signal_travel_time =
        (satellite_position - receiver_position).norm() / constants::SPEED_OF_LIGHT;
    const double angle = constants::OMEGA_E * signal_travel_time;

    Eigen::Matrix3d earth_rotation;
    earth_rotation << std::cos(angle),  std::sin(angle), 0.0,
                     -std::sin(angle),  std::cos(angle), 0.0,
                      0.0,              0.0,             1.0;
    return earth_rotation * satellite_position;
}

inline PositionSolution makeInvalidSolution(const GNSSTime& time) {
    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::NONE;
    return solution;
}

inline Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix, double tolerance = 1e-12) {
    if (matrix.size() == 0) {
        return Eigen::MatrixXd();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        matrix,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& singular_values = svd.singularValues();
    Eigen::VectorXd inverse_values = Eigen::VectorXd::Zero(singular_values.size());
    const double threshold =
        tolerance * std::max(matrix.rows(), matrix.cols()) *
        (singular_values.size() > 0 ? singular_values.array().abs().maxCoeff() : 0.0);

    for (int i = 0; i < singular_values.size(); ++i) {
        if (singular_values(i) > threshold) {
            inverse_values(i) = 1.0 / singular_values(i);
        }
    }

    return svd.matrixV() * inverse_values.asDiagonal() * svd.matrixU().transpose();
}

inline double seedPositionDivergenceMeters(
    const Vector3d& position_ecef,
    const FGOProcessor::EpochSeed& seed) {
    if (!position_ecef.allFinite() || !seed.position_ecef.allFinite() ||
        seed.position_ecef.norm() <= 1e6) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return (position_ecef - seed.position_ecef).norm();
}

inline bool applyFloatSeedPositionDivergenceGate(
    PositionSolution& solution,
    const FGOProcessor::FGOProblem& problem,
    std::size_t epoch_index,
    double max_divergence_m,
    std::size_t& rejected_count) {
    if (solution.status != SolutionStatus::FLOAT ||
        !std::isfinite(max_divergence_m) || max_divergence_m <= 0.0) {
        return false;
    }
    if (!solution.position_ecef.allFinite()) {
        solution.status = SolutionStatus::NONE;
        ++rejected_count;
        return true;
    }
    if (epoch_index >= problem.epochs.size()) {
        return false;
    }

    const double divergence_m =
        seedPositionDivergenceMeters(solution.position_ecef,
                                     problem.epochs[epoch_index]);
    if (std::isfinite(divergence_m) && divergence_m > max_divergence_m) {
        solution.status = SolutionStatus::NONE;
        ++rejected_count;
        return true;
    }
    return false;
}

inline bool applyFloatPositionJumpGate(
    PositionSolution& solution,
    const Vector3d& previous_output_position,
    bool has_previous_output_position,
    double max_jump_m,
    bool& block_float_until_fixed,
    std::size_t& rejected_count) {
    if (!std::isfinite(max_jump_m) || max_jump_m <= 0.0) {
        return false;
    }
    if (solution.status == SolutionStatus::FIXED) {
        block_float_until_fixed = false;
        return false;
    }
    if (solution.status != SolutionStatus::FLOAT) {
        return false;
    }

    bool reject = block_float_until_fixed;
    if (has_previous_output_position &&
        solution.position_ecef.allFinite() &&
        previous_output_position.allFinite()) {
        const double jump_m =
            (solution.position_ecef - previous_output_position).norm();
        if (std::isfinite(jump_m) && jump_m > max_jump_m) {
            reject = true;
            block_float_until_fixed = true;
        }
    }
    if (reject) {
        solution.status = SolutionStatus::NONE;
        ++rejected_count;
        return true;
    }
    return false;
}

struct DoubleDifferencePrediction {
    bool valid = false;
    double geometry_m = 0.0;
    Vector3d position_jacobian = Vector3d::Zero();
};

inline DoubleDifferencePrediction doubleDifferencePredictionAt(
    const Vector3d& position,
    const Vector3d& base_position,
    const Vector3d& rover_satellite_position,
    const Vector3d& rover_reference_position,
    const Vector3d& base_satellite_position,
    const Vector3d& base_reference_position) {
    DoubleDifferencePrediction prediction;
    const Vector3d rover_satellite_delta = rover_satellite_position - position;
    const Vector3d rover_reference_delta = rover_reference_position - position;
    const double rover_satellite_range = rover_satellite_delta.norm();
    const double rover_reference_range = rover_reference_delta.norm();
    const double base_satellite_range =
        (base_satellite_position - base_position).norm();
    const double base_reference_range =
        (base_reference_position - base_position).norm();
    if (rover_satellite_range <= 0.0 || rover_reference_range <= 0.0 ||
        base_satellite_range <= 0.0 || base_reference_range <= 0.0) {
        return prediction;
    }

    const Vector3d rover_satellite_los =
        rover_satellite_delta / rover_satellite_range;
    const Vector3d rover_reference_los =
        rover_reference_delta / rover_reference_range;
    prediction.valid = true;
    prediction.geometry_m =
        (rover_satellite_range - base_satellite_range) -
        (rover_reference_range - base_reference_range);
    prediction.position_jacobian = -rover_satellite_los + rover_reference_los;
    return prediction;
}

struct PreparedCarrierObservation {
    SatelliteId satellite;
    SignalType signal = SignalType::GPS_L1CA;
    Vector3d satellite_position_ecef = Vector3d::Zero();
    double corrected_pseudorange_m = 0.0;
    double corrected_carrier_m = 0.0;
    double wavelength_m = 0.0;
    double sigma_m = 0.01;
    double elevation_rad = 0.0;
    bool loss_of_lock = false;
    bool has_carrier_phase = true;
    bool has_doppler_residual = false;
    double doppler_residual_mps = 0.0;
    double doppler_sigma_mps = 0.2;
    Vector3d los = Vector3d::Zero();
    FGOProcessor::ObservationModelDebug model_debug;
};

struct ActiveCarrierSegment {
    std::size_t ambiguity_index = 0;
    std::size_t last_epoch_index = 0;
};

// --- Elevation-dependent DD sigma ("varerr") ---
//
// Port of the inuex35 reference's preprocess/prefit.py varerr_dd_sigma
// (itself a port of RTKLIB-demo5's rtkpos.c:402 varerr()), gated by
// FGOConfig::use_elevation_dependent_sigma (see that knob's doc comment in
// fgo.hpp for the full formula, dt_s mapping, and composition-order
// rationale). `is_pseudorange` selects the reference's `fact` term
// (elevation_sigma_pseudorange_ratio for pseudorange, 1.0 for carrier);
// `el_pair_rad` is the caller-computed max(min(el_ref, el_target), el_min);
// `dt_s` is the rover epoch-to-epoch interval.
inline double varerrDdSigma(bool is_pseudorange, double el_pair_rad, double dt_s,
                     const FGOProcessor::FGOConfig& config) {
    // No clamping on the ratio -- faithful to the reference's
    // `fact = cfg.err_eratio_pr if code else 1.0` (no floor/ceiling there).
    const double fact =
        is_pseudorange ? config.elevation_sigma_pseudorange_ratio : 1.0;
    const double a = fact * config.elevation_sigma_err_a_m;
    const double b = fact * config.elevation_sigma_err_b_m;
    const double d = constants::SPEED_OF_LIGHT * config.elevation_sigma_clock_stability * dt_s;
    const double sinel = std::max(std::sin(el_pair_rad), 0.05);
    const double var_sd = 2.0 * (a * a + b * b / (sinel * sinel)) + d * d;
    return std::sqrt(2.0 * var_sd);
}

struct FixedAmbiguityConstraint {
    std::size_t ambiguity_index = 0;
    double fixed_ambiguity_m = 0.0;
    int fixed_cycles = 0;
    double residual_cycles = 0.0;
    bool fixed_by_lambda = false;
};

using CarrierKey = std::pair<SatelliteId, SignalType>;

// --- Code-Minus-Carrier (CMC) multipath screening state ---
//
// Port of inuex35's preprocess/slip_detect.py per-(satellite, signal) CMC
// tracking (see FGOConfig::use_code_minus_carrier_screening for the full
// semantics). Kept as function-local state in buildDoubleDifferenceProblem
// (like active_dd_segments below) rather than a FGOProcessor member: one
// build call processes one ordered epoch stream, and buildDoubleDifference-
// Problem is const.
struct CmcState {
    bool has_prev = false;
    double prev_cmc_m = 0.0;
    bool has_baseline = false;
    double baseline_m = 0.0;
    int warmup_count = 0;
};

struct DoubleDifferenceAmbiguityKey {
    SatelliteId satellite;
    SatelliteId reference_satellite;
    SignalType signal = SignalType::GPS_L1CA;
    std::size_t satellite_ambiguity_index = 0;
    std::size_t reference_ambiguity_index = 0;

    bool operator<(const DoubleDifferenceAmbiguityKey& other) const {
        return std::tie(satellite,
                        reference_satellite,
                        signal,
                        satellite_ambiguity_index,
                        reference_ambiguity_index) <
               std::tie(other.satellite,
                        other.reference_satellite,
                        other.signal,
                        other.satellite_ambiguity_index,
                        other.reference_ambiguity_index);
    }
};

struct DoubleDifferenceSegmentKey {
    SatelliteId satellite;
    SatelliteId reference_satellite;
    SignalType signal = SignalType::GPS_L1CA;

    bool operator<(const DoubleDifferenceSegmentKey& other) const {
        return std::tie(satellite, reference_satellite, signal) <
               std::tie(other.satellite, other.reference_satellite, other.signal);
    }
};

struct SingleDifferenceReferenceKey {
    std::size_t epoch_index = 0;
    GNSSSystem system = GNSSSystem::UNKNOWN;
    SignalType signal = SignalType::GPS_L1CA;

    bool operator<(const SingleDifferenceReferenceKey& other) const {
        return std::tie(epoch_index, system, signal) <
               std::tie(other.epoch_index, other.system, other.signal);
    }
};

struct SingleDifferenceDefaultReferenceKey {
    GNSSSystem system = GNSSSystem::UNKNOWN;
    SignalType signal = SignalType::GPS_L1CA;

    bool operator<(const SingleDifferenceDefaultReferenceKey& other) const {
        return std::tie(system, signal) <
               std::tie(other.system, other.signal);
    }
};

struct SingleDifferenceCarrierResidual {
    std::size_t epoch_index = 0;
    double residual_m = 0.0;
    Vector3d los = Vector3d::Zero();
};

struct GeometryFreeSlipState {
    GNSSTime time;
    double geometry_free_m = 0.0;
    std::size_t first_rover_ambiguity_index = 0;
    std::size_t second_rover_ambiguity_index = 0;
};

struct GeometryFreePendingReset {
    GNSSTime time;
    std::size_t rover_ambiguity_index = 0;
};

inline std::map<CarrierKey, PreparedCarrierObservation> prepareCarrierObservationsForReceiver(
    const ObservationData& epoch,
    const NavigationData& nav,
    const Vector3d& receiver_position,
    const FGOProcessor::FGOConfig& config,
    double min_snr_dbhz,
    bool require_carrier_phase = true,
    bool apply_elevation_mask = true,
    const SecondaryCodeTable* secondary_code_table = nullptr) {
    std::map<CarrierKey, PreparedCarrierObservation> carriers;
    if (receiver_position.norm() <= 1e6) {
        return carriers;
    }

    double receiver_lat = 0.0;
    double receiver_lon = 0.0;
    double receiver_height = 0.0;
    ecef2geodetic(receiver_position, receiver_lat, receiver_lon, receiver_height);

    const double min_elevation_rad = config.min_elevation_deg * M_PI / 180.0;
    for (const auto& observation : epoch.observations) {
        if (!isEligibleFgoSignal(observation.satellite,
                                 observation.signal,
                                 config.use_multi_constellation,
                                 config.use_multi_frequency_double_difference)) {
            continue;
        }
        if (!passesSecondaryCodeAlignment(observation, secondary_code_table)) {
            continue;
        }
        const bool has_carrier_phase =
            observation.has_carrier_phase && observation.carrier_phase != 0.0;
        if (!observation.valid || !observation.has_pseudorange ||
            observation.pseudorange <= 0.0 || observation.snr < min_snr_dbhz ||
            (require_carrier_phase && !has_carrier_phase)) {
            continue;
        }

        Vector3d satellite_position;
        Vector3d satellite_velocity;
        double satellite_clock_bias = 0.0;
        double satellite_clock_drift = 0.0;

        GNSSTime transmit_time =
            epoch.time - observation.pseudorange / constants::SPEED_OF_LIGHT;
        if (!nav.calculateSatelliteState(observation.satellite,
                                         transmit_time,
                                         satellite_position,
                                         satellite_velocity,
                                         satellite_clock_bias,
                                         satellite_clock_drift)) {
            continue;
        }

        transmit_time = transmit_time - satellite_clock_bias;
        if (!nav.calculateSatelliteState(observation.satellite,
                                         transmit_time,
                                         satellite_position,
                                         satellite_velocity,
                                         satellite_clock_bias,
                                         satellite_clock_drift)) {
            continue;
        }

        const Ephemeris* eph = nav.getEphemeris(observation.satellite, transmit_time);
        if (!eph || !isHealthyForPositioning(observation, *eph)) {
            continue;
        }

        const Vector3d corrected_satellite_position =
            earthRotationCorrected(satellite_position, receiver_position);
        const auto geometry =
            nav.calculateGeometry(receiver_position, corrected_satellite_position);
        if (apply_elevation_mask && geometry.elevation < min_elevation_rad) {
            continue;
        }

        double ionosphere_delay = 0.0;
        if (config.use_ionosphere_model && nav.ionosphere_model.valid) {
            ionosphere_delay = models::ionoDelayKlobuchar(
                receiver_lat,
                receiver_lon,
                geometry.azimuth,
                geometry.elevation,
                epoch.time.tow,
                nav.ionosphere_model.alpha,
                nav.ionosphere_model.beta);

            const double frequency_hz = signalFrequencyHz(observation.signal, eph);
            if (frequency_hz > 0.0) {
                const double scale = constants::GPS_L1_FREQ / frequency_hz;
                ionosphere_delay *= scale * scale;
            }
        }

        double troposphere_delay = 0.0;
        if (config.use_troposphere_model) {
            troposphere_delay =
                models::tropDelaySaastamoinen(receiver_position, geometry.elevation);
        }

        double wavelength = signalWavelengthMeters(observation);
        if (wavelength <= 0.0) {
            wavelength = signalWavelengthMeters(observation.signal, eph);
        }
        const bool usable_carrier = has_carrier_phase && wavelength > 0.0;
        if (require_carrier_phase && !usable_carrier) {
            continue;
        }

        const double satellite_clock_m =
            satellite_clock_bias * constants::SPEED_OF_LIGHT;
        const double group_delay_m = groupDelayCorrectionMeters(observation, *eph);
        const double corrected_pseudorange =
            observation.pseudorange +
            satellite_clock_m -
            ionosphere_delay -
            troposphere_delay -
            group_delay_m;
        const double corrected_carrier =
            usable_carrier
                ? observation.carrier_phase * wavelength +
                      satellite_clock_m -
                      troposphere_delay +
                      ionosphere_delay
                : 0.0;

        FGOProcessor::ObservationModelDebug model_debug;
        model_debug.raw_pseudorange_m = observation.pseudorange;
        model_debug.raw_carrier_m =
            usable_carrier ? observation.carrier_phase * wavelength : 0.0;
        model_debug.satellite_clock_m = satellite_clock_m;
        model_debug.ionosphere_delay_m = ionosphere_delay;
        model_debug.troposphere_delay_m = troposphere_delay;
        model_debug.group_delay_m = group_delay_m;
        model_debug.corrected_pseudorange_m = corrected_pseudorange;
        model_debug.corrected_carrier_m = corrected_carrier;
        const Vector3d corrected_delta =
            corrected_satellite_position - receiver_position;
        const double corrected_range = corrected_delta.norm();
        if (corrected_range <= 0.0) {
            continue;
        }
        model_debug.geometric_range_m = corrected_range;
        model_debug.elevation_rad = geometry.elevation;
        model_debug.azimuth_rad = geometry.azimuth;
        model_debug.snr_dbhz = observation.snr;

        const double sin_el = std::max(0.1, std::sin(geometry.elevation));
        PreparedCarrierObservation carrier;
        carrier.satellite = observation.satellite;
        carrier.signal = observation.signal;
        carrier.satellite_position_ecef = corrected_satellite_position;
        carrier.corrected_pseudorange_m = corrected_pseudorange;
        carrier.corrected_carrier_m = corrected_carrier;
        carrier.wavelength_m = wavelength;
        carrier.sigma_m = std::max(1e-4, config.carrier_phase_sigma_m / sin_el);
        carrier.elevation_rad = geometry.elevation;
        carrier.loss_of_lock =
            observation.loss_of_lock || ((observation.lli & 0x01U) != 0);
        carrier.has_carrier_phase = usable_carrier;
        carrier.los = -corrected_delta / corrected_range;
        carrier.doppler_sigma_mps =
            std::max(1e-4, config.single_difference_doppler_sigma_mps /
                               std::sqrt(sin_el));
        if (observation.has_doppler && wavelength > 0.0) {
            Vector3d doppler_los = Vector3d::Zero();
            double modeled_range_rate = 0.0;
            bool doppler_geometry_valid = false;
            if (config.use_corrected_undifferenced_doppler_factors) {
                doppler_geometry_valid =
                    doppler_contract::knownSatelliteRangeRate(
                        satellite_position, satellite_velocity,
                        receiver_position, true, doppler_los,
                        modeled_range_rate);
            } else {
                const Vector3d doppler_delta =
                    satellite_position - receiver_position;
                const double doppler_range = doppler_delta.norm();
                if (doppler_range > 0.0 && doppler_delta.allFinite()) {
                    doppler_los = doppler_delta / doppler_range;
                    const double sagnac_rate =
                        constants::OMEGA_E / constants::SPEED_OF_LIGHT *
                        (satellite_velocity(1) * receiver_position(0) -
                         satellite_velocity(0) * receiver_position(1));
                    modeled_range_rate =
                        satellite_velocity.dot(doppler_los) + sagnac_rate;
                    doppler_geometry_valid =
                        std::isfinite(modeled_range_rate);
                }
            }
            if (doppler_geometry_valid) {
                const double satellite_clock_drift_mps =
                    satellite_clock_drift * constants::SPEED_OF_LIGHT;
                const double measured_range_rate =
                    doppler_contract::rinexDopplerToRangeRate(
                        observation.doppler,
                        constants::SPEED_OF_LIGHT / wavelength);
                carrier.doppler_residual_mps =
                    doppler_contract::receiverOnlyResidual(
                        measured_range_rate, modeled_range_rate,
                        satellite_clock_drift);
                carrier.has_doppler_residual =
                    std::isfinite(carrier.doppler_residual_mps) &&
                    std::isfinite(measured_range_rate);
                model_debug.has_doppler_residual =
                    carrier.has_doppler_residual;
                model_debug.doppler_residual_mps =
                    carrier.doppler_residual_mps;
                model_debug.doppler_measured_range_rate_mps =
                    measured_range_rate;
                model_debug.doppler_satellite_range_rate_mps =
                    modeled_range_rate;
                model_debug.doppler_satellite_clock_drift_mps =
                    satellite_clock_drift_mps;
                model_debug.doppler_uses_rotated_satellite_state =
                    config.use_corrected_undifferenced_doppler_factors;
                if (config.use_corrected_undifferenced_doppler_factors) {
                    carrier.los = -doppler_los;
                }
            }
        }
        carrier.model_debug = model_debug;
        carriers[{observation.satellite, observation.signal}] = carrier;
    }

    return carriers;
}

inline Observation interpolateObservation(const Observation& lower,
                                   const Observation& upper,
                                   double alpha) {
    Observation interpolated = lower;
    interpolated.valid = lower.valid && upper.valid;
    interpolated.has_pseudorange =
        lower.has_pseudorange && upper.has_pseudorange;
    interpolated.has_carrier_phase =
        lower.has_carrier_phase && upper.has_carrier_phase;
    interpolated.has_doppler = lower.has_doppler && upper.has_doppler;
    if (interpolated.has_pseudorange) {
        interpolated.pseudorange =
            lower.pseudorange + alpha * (upper.pseudorange - lower.pseudorange);
    }
    if (interpolated.has_carrier_phase) {
        interpolated.carrier_phase =
            lower.carrier_phase +
            alpha * (upper.carrier_phase - lower.carrier_phase);
    }
    if (interpolated.has_doppler) {
        interpolated.doppler =
            lower.doppler + alpha * (upper.doppler - lower.doppler);
    }
    interpolated.snr = std::min(lower.snr, upper.snr);
    interpolated.signal_strength =
        std::min(lower.signal_strength, upper.signal_strength);
    interpolated.lli = lower.lli | upper.lli;
    interpolated.loss_of_lock =
        lower.loss_of_lock || upper.loss_of_lock ||
        ((interpolated.lli & 0x01U) != 0);
    interpolated.has_glonass_frequency_channel =
        lower.has_glonass_frequency_channel &&
        upper.has_glonass_frequency_channel &&
        lower.glonass_frequency_channel == upper.glonass_frequency_channel;
    // MF hygiene: never blend two different observation codes of the same
    // band across the interpolation boundary (e.g. BDS B2I C7D at t0 and C7I
    // at t1) -- the blend would carry an inter-code bias under a single code
    // label. Secondary bands only, so the single-frequency (primary band)
    // path is bit-identical.
    if (signal_policy::isSecondarySignal(lower.satellite.system, lower.signal) &&
        (lower.pseudorange_observation_type !=
             upper.pseudorange_observation_type ||
         lower.carrier_phase_observation_type !=
             upper.carrier_phase_observation_type)) {
        interpolated.valid = false;
    }
    return interpolated;
}

inline bool findMatchedBaseEpoch(const std::vector<ObservationData>& base_epochs,
                          const GNSSTime& rover_time,
                          std::size_t& cursor,
                          double exact_tolerance_s,
                          double interpolation_max_gap_s,
                          ObservationData& matched_epoch,
                          bool& interpolated) {
    if (base_epochs.empty()) {
        return false;
    }

    while (cursor + 1 < base_epochs.size() &&
           base_epochs[cursor + 1].time <= rover_time) {
        ++cursor;
    }

    const ObservationData* exact_match = nullptr;
    double best_abs_dt = std::numeric_limits<double>::infinity();
    auto consider_exact = [&](std::size_t index) {
        if (index >= base_epochs.size()) {
            return;
        }
        const double dt = std::abs(base_epochs[index].time - rover_time);
        if (dt <= exact_tolerance_s && dt < best_abs_dt) {
            best_abs_dt = dt;
            exact_match = &base_epochs[index];
        }
    };
    consider_exact(cursor);
    consider_exact(cursor + 1);
    if (cursor > 0) {
        consider_exact(cursor - 1);
    }

    if (exact_match != nullptr) {
        matched_epoch = *exact_match;
        interpolated = false;
        return true;
    }

    if (interpolation_max_gap_s <= 0.0) {
        return false;
    }

    std::size_t lower_index = cursor;
    if (base_epochs[lower_index].time > rover_time) {
        if (lower_index == 0) {
            return false;
        }
        --lower_index;
    }
    const std::size_t upper_index = lower_index + 1;
    if (upper_index >= base_epochs.size()) {
        return false;
    }

    const ObservationData& lower_epoch = base_epochs[lower_index];
    const ObservationData& upper_epoch = base_epochs[upper_index];
    const double total_dt = upper_epoch.time - lower_epoch.time;
    const double lower_dt = rover_time - lower_epoch.time;
    if (total_dt <= 0.0 || total_dt > interpolation_max_gap_s ||
        lower_dt < 0.0 || lower_dt > total_dt) {
        return false;
    }

    const double alpha = lower_dt / total_dt;
    std::map<CarrierKey, const Observation*> upper_observations;
    for (const auto& observation : upper_epoch.observations) {
        upper_observations[{observation.satellite, observation.signal}] =
            &observation;
    }

    ObservationData candidate(rover_time);
    candidate.receiver_position = lower_epoch.receiver_position;
    if (candidate.receiver_position.norm() <= 1e6) {
        candidate.receiver_position = upper_epoch.receiver_position;
    }
    candidate.receiver_clock_bias =
        lower_epoch.receiver_clock_bias +
        alpha * (upper_epoch.receiver_clock_bias -
                 lower_epoch.receiver_clock_bias);

    for (const auto& lower_observation : lower_epoch.observations) {
        const CarrierKey key{lower_observation.satellite, lower_observation.signal};
        const auto upper_it = upper_observations.find(key);
        if (upper_it == upper_observations.end()) {
            continue;
        }
        const Observation& upper_observation = *upper_it->second;
        if (!lower_observation.valid || !upper_observation.valid ||
            !lower_observation.has_pseudorange ||
            !upper_observation.has_pseudorange ||
            !lower_observation.has_carrier_phase ||
            !upper_observation.has_carrier_phase) {
            continue;
        }
        candidate.addObservation(
            interpolateObservation(lower_observation, upper_observation, alpha));
    }

    if (candidate.observations.empty()) {
        return false;
    }

    matched_epoch = std::move(candidate);
    interpolated = true;
    return true;
}

}  // namespace fgo_internal
}  // namespace libgnss
