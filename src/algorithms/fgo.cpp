#include <libgnss++/algorithms/fgo.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#ifdef GNSSPP_HAS_CUDA_FGO
#include "fgo_cuda_backend.hpp"
#endif

#include <Eigen/Dense>
#include <Eigen/Sparse>
#ifdef GNSSPP_HAS_CHOLMOD
#include <Eigen/CholmodSupport>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <tuple>

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

namespace {

bool cudaDenseSolverEnabled(int state_size) {
#ifdef GNSSPP_HAS_CUDA_FGO
    return detail::fgoCudaDenseSolverEnabled(state_size);
#else
    (void)state_size;
    return false;
#endif
}

struct CudaDenseSolveStats {
    std::size_t attempts = 0;
    std::size_t successes = 0;
    double processing_ms = 0.0;
};

bool tryCudaDenseSolve(const Eigen::MatrixXd& normal,
                       const Eigen::MatrixXd& rhs,
                       Eigen::MatrixXd& solution,
                       CudaDenseSolveStats* stats) {
#ifdef GNSSPP_HAS_CUDA_FGO
    if (normal.rows() <= 0 || normal.rows() != normal.cols() ||
        rhs.rows() != normal.rows() || rhs.cols() <= 0) {
        return false;
    }
    const auto start = std::chrono::steady_clock::now();
    if (stats != nullptr) {
        ++stats->attempts;
    }
    solution.resize(rhs.rows(), rhs.cols());
    const bool solved = detail::fgoCudaDenseSolve(
        normal.data(), static_cast<int>(normal.rows()),
        rhs.data(), static_cast<int>(rhs.cols()), solution.data());
    if (stats != nullptr) {
        stats->processing_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start)
                .count();
        if (solved) {
            ++stats->successes;
        }
    }
    return solved;
#else
    (void)normal;
    (void)rhs;
    (void)solution;
    (void)stats;
    return false;
#endif
}

bool isPrimaryFgoSignal(SignalType signal, bool use_multi_constellation) {
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
bool isEligibleFgoSignal(const SatelliteId& satellite,
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
const char* trackingCodePriorityString(GNSSSystem system, int band) {
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

int trackingCodePriority(GNSSSystem system, const std::string& obs_type) {
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

SecondaryCodeTable buildSecondaryCodeTable(
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
bool passesSecondaryCodeAlignment(const Observation& observation,
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

GNSSSystem clockBiasGroup(GNSSSystem system) {
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

bool usesSeparateClockBias(GNSSSystem group) {
    return group != GNSSSystem::UNKNOWN && group != GNSSSystem::GPS;
}

double groupDelayCorrectionMeters(const Observation& observation, const Ephemeris& eph) {
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

bool isHealthyForPositioning(const Observation& observation, const Ephemeris& eph) {
    int sv_health = static_cast<int>(eph.health);
    if (observation.satellite.system == GNSSSystem::QZSS) {
        sv_health &= 0xFE;
    }
    return sv_health == 0;
}

Vector3d earthRotationCorrected(const Vector3d& satellite_position,
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

PositionSolution makeInvalidSolution(const GNSSTime& time) {
    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::NONE;
    return solution;
}

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix, double tolerance = 1e-12) {
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

double seedPositionDivergenceMeters(
    const Vector3d& position_ecef,
    const FGOProcessor::EpochSeed& seed) {
    if (!position_ecef.allFinite() || !seed.position_ecef.allFinite() ||
        seed.position_ecef.norm() <= 1e6) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return (position_ecef - seed.position_ecef).norm();
}

bool applyFloatSeedPositionDivergenceGate(
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

bool applyFloatPositionJumpGate(
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

DoubleDifferencePrediction doubleDifferencePredictionAt(
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
double varerrDdSigma(bool is_pseudorange, double el_pair_rad, double dt_s,
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
    std::size_t reference_ambiguity_index =
        std::numeric_limits<std::size_t>::max();
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

struct SingleDifferenceAmbiguityKey {
    SatelliteId satellite;
    SignalType signal = SignalType::GPS_L1CA;
    std::size_t rover_ambiguity_index = 0;
    std::size_t integrity_segment_index = 0;

    bool operator<(const SingleDifferenceAmbiguityKey& other) const {
        return std::tie(satellite, signal, rover_ambiguity_index,
                        integrity_segment_index) <
               std::tie(other.satellite, other.signal,
                        other.rover_ambiguity_index,
                        other.integrity_segment_index);
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

std::map<CarrierKey, PreparedCarrierObservation> prepareCarrierObservationsForReceiver(
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
            const Vector3d doppler_delta = satellite_position - receiver_position;
            const double doppler_range = doppler_delta.norm();
            if (doppler_range > 0.0) {
                const Vector3d ex = doppler_delta / doppler_range;
                const double sagnac_rate =
                    constants::OMEGA_E / constants::SPEED_OF_LIGHT *
                    (satellite_velocity(1) * receiver_position(0) -
                     satellite_velocity(0) * receiver_position(1));
                const double modeled_range_rate =
                    satellite_velocity.dot(ex) + sagnac_rate;
                const double satellite_clock_drift_mps =
                    satellite_clock_drift * constants::SPEED_OF_LIGHT;
                const double measured_range_rate =
                    -observation.doppler * wavelength;
                carrier.doppler_residual_mps =
                    measured_range_rate -
                    (modeled_range_rate - satellite_clock_drift_mps);
                carrier.has_doppler_residual =
                    std::isfinite(carrier.doppler_residual_mps);
            }
        }
        carrier.model_debug = model_debug;
        carriers[{observation.satellite, observation.signal}] = carrier;
    }

    return carriers;
}

Observation interpolateObservation(const Observation& lower,
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

bool findMatchedBaseEpoch(const std::vector<ObservationData>& base_epochs,
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

}  // namespace

FGOProcessor::FGOProblem FGOProcessor::buildPseudorangeProblem(
    const std::vector<ObservationData>& input_epochs,
    const NavigationData& nav) const {
    FGOProblem problem;
    problem.diagnostics.input_epochs = input_epochs.size();
    if (input_epochs.empty()) {
        return problem;
    }

    // MF hygiene: one observation code per (system, secondary band) for this
    // receiver (cssrlib/RTKLIB-style). Only meaningful when secondary bands
    // are ingested at all.
    SecondaryCodeTable rover_secondary_codes;
    const SecondaryCodeTable* rover_secondary_codes_ptr = nullptr;
    if (config_.use_multi_frequency_double_difference &&
        config_.use_double_difference_secondary_code_alignment &&
        config_.use_multi_constellation) {
        rover_secondary_codes = buildSecondaryCodeTable(input_epochs);
        rover_secondary_codes_ptr = &rover_secondary_codes;
    }

    ProcessorConfig spp_processor_config;
    spp_processor_config.mode = PositioningMode::SPP;
    spp_processor_config.elevation_mask = config_.min_elevation_deg;
    spp_processor_config.snr_mask = config_.min_snr_dbhz;
    spp_processor_config.use_ionosphere_model = config_.use_ionosphere_model;
    spp_processor_config.use_troposphere_model = config_.use_troposphere_model;

    SPPProcessor::SPPConfig spp_config;
    spp_config.apply_atmospheric_corrections =
        config_.use_ionosphere_model || config_.use_troposphere_model;
    spp_config.use_multi_constellation = config_.use_multi_constellation;

    SPPProcessor spp_processor(spp_config);
    spp_processor.initialize(spp_processor_config);

    const double min_elevation_rad = config_.min_elevation_deg * M_PI / 180.0;
    const double dd_reference_min_snr_dbhz =
        config_.double_difference_reference_min_snr_dbhz >= 0.0
            ? config_.double_difference_reference_min_snr_dbhz
            : config_.min_snr_dbhz;
    std::vector<std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>>
        carrier_by_problem_epoch;
    std::vector<std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>>
        dd_pseudorange_by_problem_epoch;
    std::vector<std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>>
        dd_reference_carrier_by_problem_epoch;
    std::map<SatelliteId, double> previous_gps_pseudorange_by_satellite;

    // Seed continuity across a brief SPP outage (see FGOConfig::use_spp_seed):
    // a tunnel/underpass/urban-canyon gap can drop the rover to near-zero
    // tracked satellites for several seconds. The RINEX header's static
    // approximate position can be kilometres from the true kinematic position
    // at that point in the trajectory; seeding elevation/geometry from it
    // spuriously fails the elevation mask for whatever few satellites ARE
    // still tracked during recovery, rejecting far more marginal epochs than
    // the min_satellites_per_epoch gate actually requires. Coast on the last
    // valid SPP fix instead -- at the rover's epoch-to-epoch cadence this
    // stays far closer to truth through a brief outage than the static
    // fallback, and degrades no worse than the static fallback once the
    // outage is long enough that both are stale.
    Vector3d last_valid_seed_position_ecef = Vector3d::Zero();
    double last_valid_seed_clock_bias_m = 0.0;
    bool have_last_valid_seed = false;

    for (const auto& epoch : input_epochs) {
        EpochSeed seed;
        seed.time = epoch.time;

        PositionSolution spp_solution = makeInvalidSolution(epoch.time);
        if (config_.use_spp_seed) {
            spp_solution = spp_processor.processEpoch(epoch, nav);
        }

        if (spp_solution.isValid()) {
            seed.position_ecef = spp_solution.position_ecef;
            seed.receiver_clock_bias_m = spp_solution.receiver_clock_bias;
            last_valid_seed_position_ecef = seed.position_ecef;
            last_valid_seed_clock_bias_m = seed.receiver_clock_bias_m;
            have_last_valid_seed = true;
        } else if (have_last_valid_seed) {
            seed.position_ecef = last_valid_seed_position_ecef;
            seed.receiver_clock_bias_m = last_valid_seed_clock_bias_m;
        } else if (epoch.receiver_position.norm() > 1e6) {
            seed.position_ecef = epoch.receiver_position;
            seed.receiver_clock_bias_m = epoch.receiver_clock_bias * constants::SPEED_OF_LIGHT;
        } else {
            ++problem.diagnostics.skipped_epochs_without_seed;
            continue;
        }

        std::vector<PseudorangeFactor> epoch_factors;
        epoch_factors.reserve(epoch.observations.size());
        std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation> epoch_carriers;
        std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>
            epoch_dd_pseudorange_observations;
        std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>
            epoch_dd_reference_carriers;
        std::map<SatelliteId, double> gps_pseudorange_by_satellite;

        double receiver_lat = 0.0;
        double receiver_lon = 0.0;
        double receiver_height = 0.0;
        ecef2geodetic(seed.position_ecef, receiver_lat, receiver_lon, receiver_height);

        for (const auto& observation : epoch.observations) {
            if (!isEligibleFgoSignal(observation.satellite,
                                     observation.signal,
                                     config_.use_multi_constellation,
                                     config_.use_multi_frequency_double_difference)) {
                continue;
            }
            if (!passesSecondaryCodeAlignment(observation,
                                              rover_secondary_codes_ptr)) {
                continue;
            }
            if (!observation.valid || !observation.has_pseudorange ||
                observation.pseudorange <= 0.0) {
                continue;
            }
            const bool passes_snr_mask = observation.snr >= config_.min_snr_dbhz;
            const bool passes_dd_reference_snr_mask =
                dd_reference_min_snr_dbhz <= 0.0 ||
                observation.snr >= dd_reference_min_snr_dbhz;

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
                earthRotationCorrected(satellite_position, seed.position_ecef);
            const auto geometry =
                nav.calculateGeometry(seed.position_ecef, corrected_satellite_position);
            const bool passes_elevation_mask =
                geometry.elevation >= min_elevation_rad;

            double ionosphere_delay = 0.0;
            if (config_.use_ionosphere_model && nav.ionosphere_model.valid) {
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
            if (config_.use_troposphere_model) {
                troposphere_delay =
                    models::tropDelaySaastamoinen(seed.position_ecef, geometry.elevation);
            }

            const double satellite_clock_m =
                satellite_clock_bias * constants::SPEED_OF_LIGHT;
            const double group_delay_m =
                groupDelayCorrectionMeters(observation, *eph);
            const double corrected_pseudorange =
                observation.pseudorange +
                satellite_clock_m -
                ionosphere_delay -
                troposphere_delay -
                group_delay_m;

            const double sin_el = std::max(0.1, std::sin(geometry.elevation));
            const double pseudorange_elevation_scale = std::pow(
                sin_el,
                std::max(0.0, config_.pseudorange_elevation_sigma_power));
            if (passes_snr_mask && passes_elevation_mask) {
                PseudorangeFactor factor;
                factor.satellite = observation.satellite;
                factor.clock_group = clockBiasGroup(observation.satellite.system);
                factor.satellite_position_ecef = corrected_satellite_position;
                factor.corrected_pseudorange_m = corrected_pseudorange;
                factor.sigma_m =
                    std::max(1e-3,
                             config_.pseudorange_sigma_m /
                                 std::max(1e-6, pseudorange_elevation_scale));
                factor.elevation_rad = geometry.elevation;
                epoch_factors.push_back(factor);
                if (observation.satellite.system == GNSSSystem::GPS) {
                    gps_pseudorange_by_satellite[observation.satellite] =
                        observation.pseudorange;
                }
            }

            const bool carrier_loss_of_lock =
                observation.loss_of_lock || ((observation.lli & 0x01U) != 0);
            double wavelength = signalWavelengthMeters(observation);
            const bool has_carrier_phase =
                observation.has_carrier_phase && observation.carrier_phase != 0.0;
            if (wavelength <= 0.0) {
                wavelength = signalWavelengthMeters(observation.signal, eph);
            }
            const bool usable_carrier = has_carrier_phase && wavelength > 0.0;
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
                corrected_satellite_position - seed.position_ecef;
            const double corrected_range = corrected_delta.norm();
            if (corrected_range <= 0.0) {
                continue;
            }
            model_debug.geometric_range_m = corrected_range;
            model_debug.elevation_rad = geometry.elevation;
            model_debug.azimuth_rad = geometry.azimuth;
            model_debug.snr_dbhz = observation.snr;

            PreparedCarrierObservation carrier;
            carrier.satellite = observation.satellite;
            carrier.signal = observation.signal;
            carrier.satellite_position_ecef = corrected_satellite_position;
            carrier.corrected_pseudorange_m = corrected_pseudorange;
            carrier.corrected_carrier_m = corrected_carrier;
            carrier.wavelength_m = wavelength;
            carrier.sigma_m =
                std::max(1e-4, config_.carrier_phase_sigma_m / sin_el);
            carrier.elevation_rad = geometry.elevation;
            carrier.loss_of_lock = carrier_loss_of_lock;
            carrier.has_carrier_phase = usable_carrier;
            carrier.los = -corrected_delta / corrected_range;
            carrier.doppler_sigma_mps =
                std::max(1e-4,
                         config_.single_difference_doppler_sigma_mps /
                             std::sqrt(sin_el));
            if (observation.has_doppler && wavelength > 0.0) {
                const Vector3d doppler_delta =
                    satellite_position - seed.position_ecef;
                const double doppler_range = doppler_delta.norm();
                if (doppler_range > 0.0) {
                    const Vector3d ex = doppler_delta / doppler_range;
                    const double sagnac_rate =
                        constants::OMEGA_E / constants::SPEED_OF_LIGHT *
                        (satellite_velocity(1) * seed.position_ecef(0) -
                         satellite_velocity(0) * seed.position_ecef(1));
                    const double modeled_range_rate =
                        satellite_velocity.dot(ex) + sagnac_rate;
                    const double satellite_clock_drift_mps =
                        satellite_clock_drift * constants::SPEED_OF_LIGHT;
                    const double measured_range_rate =
                        -observation.doppler * wavelength;
                    carrier.doppler_residual_mps =
                        measured_range_rate -
                        (modeled_range_rate - satellite_clock_drift_mps);
                    carrier.has_doppler_residual =
                        std::isfinite(carrier.doppler_residual_mps);
                }
            }
            carrier.model_debug = model_debug;
            if (passes_dd_reference_snr_mask) {
                epoch_dd_reference_carriers[
                    {observation.satellite, observation.signal}] = carrier;
            }
            if (passes_snr_mask && passes_elevation_mask) {
                epoch_dd_pseudorange_observations[
                    {observation.satellite, observation.signal}] = carrier;
            }
            if (usable_carrier && passes_snr_mask && passes_elevation_mask) {
                if (!(config_.reject_rover_carrier_loss_of_lock &&
                      carrier_loss_of_lock)) {
                    epoch_carriers[{observation.satellite, observation.signal}] =
                        carrier;
                }
            }
        }

        const std::size_t usable_epoch_measurements =
            config_.use_pseudorange_factors ? epoch_factors.size()
                                            : epoch_carriers.size();
        if (static_cast<int>(usable_epoch_measurements) <
            config_.min_satellites_per_epoch) {
            continue;
        }

        const std::size_t epoch_index = problem.epochs.size();
        problem.epochs.push_back(seed);
        bool clock_jump = false;
        double gps_pseudorange_delta_sum = 0.0;
        std::size_t gps_pseudorange_delta_count = 0;
        for (const auto& [satellite, pseudorange] : gps_pseudorange_by_satellite) {
            const auto previous_it =
                previous_gps_pseudorange_by_satellite.find(satellite);
            if (previous_it == previous_gps_pseudorange_by_satellite.end()) {
                continue;
            }
            gps_pseudorange_delta_sum += pseudorange - previous_it->second;
            ++gps_pseudorange_delta_count;
        }
        if (gps_pseudorange_delta_count > 0) {
            const double mean_delta =
                gps_pseudorange_delta_sum /
                static_cast<double>(gps_pseudorange_delta_count);
            clock_jump = mean_delta > 1e5;
        }
        problem.clock_jumps.push_back(clock_jump);
        previous_gps_pseudorange_by_satellite =
            std::move(gps_pseudorange_by_satellite);
        problem.diagnostics.seeded_epochs = problem.epochs.size();
        carrier_by_problem_epoch.push_back(std::move(epoch_carriers));
        dd_pseudorange_by_problem_epoch.push_back(
            std::move(epoch_dd_pseudorange_observations));
        dd_reference_carrier_by_problem_epoch.push_back(
            std::move(epoch_dd_reference_carriers));
        if (config_.use_pseudorange_factors) {
            for (auto& factor : epoch_factors) {
                factor.epoch_index = epoch_index;
                problem.pseudorange_factors.push_back(factor);
            }
        }
    }

    if ((config_.use_carrier_phase_factors || config_.use_double_difference_factors) &&
        !carrier_by_problem_epoch.empty()) {
        using CarrierKey = std::pair<SatelliteId, SignalType>;
        std::map<CarrierKey, ActiveCarrierSegment> active_segments;
        std::map<CarrierKey, std::size_t> next_segment_indices;
        const double max_gap = std::max(0.0, config_.max_tdcp_gap_s);

        for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size(); ++epoch_index) {
            const auto& current_carriers = carrier_by_problem_epoch[epoch_index];
            for (const auto& [key, carrier] : current_carriers) {
                auto active_it = active_segments.find(key);
                bool start_new_segment = active_it == active_segments.end();
                if (active_it != active_segments.end()) {
                    const double dt = problem.epochs[epoch_index].time -
                                      problem.epochs[active_it->second.last_epoch_index].time;
                    if (dt <= 0.0 || (max_gap > 0.0 && dt > max_gap) ||
                        carrier.loss_of_lock) {
                        start_new_segment = true;
                    }
                }

                if (start_new_segment) {
                    const std::size_t segment_index = next_segment_indices[key]++;
                    std::size_t carrier_segment_index = segment_index;
                    if (config_.use_carrier_phase_factors) {
                        AmbiguityState ambiguity;
                        ambiguity.satellite = carrier.satellite;
                        ambiguity.signal = carrier.signal;
                        ambiguity.segment_index = segment_index;
                        ambiguity.wavelength_m = carrier.wavelength_m;
                        ambiguity.initial_ambiguity_m =
                            carrier.corrected_carrier_m -
                            carrier.corrected_pseudorange_m;
                        carrier_segment_index = problem.ambiguity_states.size();
                        problem.ambiguity_states.push_back(ambiguity);
                    }
                    active_it = active_segments
                                    .insert_or_assign(
                                        key,
                                        ActiveCarrierSegment{carrier_segment_index,
                                                             epoch_index})
                                    .first;
                } else {
                    active_it->second.last_epoch_index = epoch_index;
                }

                CarrierPhaseFactor carrier_observation;
                carrier_observation.epoch_index = epoch_index;
                carrier_observation.ambiguity_index = active_it->second.ambiguity_index;
                carrier_observation.satellite = carrier.satellite;
                carrier_observation.clock_group =
                    clockBiasGroup(carrier.satellite.system);
                carrier_observation.signal = carrier.signal;
                carrier_observation.satellite_position_ecef =
                    carrier.satellite_position_ecef;
                carrier_observation.corrected_pseudorange_m =
                    carrier.corrected_pseudorange_m;
                carrier_observation.corrected_carrier_m =
                    carrier.corrected_carrier_m;
                carrier_observation.wavelength_m = carrier.wavelength_m;
                carrier_observation.sigma_m = carrier.sigma_m;
                carrier_observation.elevation_rad = carrier.elevation_rad;
                carrier_observation.has_carrier_phase = carrier.has_carrier_phase;
                carrier_observation.loss_of_lock = carrier.loss_of_lock;
                carrier_observation.has_doppler_residual =
                    carrier.has_doppler_residual;
                carrier_observation.doppler_residual_mps =
                    carrier.doppler_residual_mps;
                carrier_observation.doppler_sigma_mps =
                    carrier.doppler_sigma_mps;
                carrier_observation.los = carrier.los;
                carrier_observation.model_debug = carrier.model_debug;
                problem.carrier_observations.push_back(carrier_observation);
                if (config_.use_carrier_phase_factors) {
                    problem.carrier_phase_factors.push_back(carrier_observation);
                }
            }
        }
    }

    if (config_.use_double_difference_factors) {
        for (std::size_t epoch_index = 0;
             epoch_index < dd_pseudorange_by_problem_epoch.size();
             ++epoch_index) {
            for (const auto& [key, carrier] :
                 dd_pseudorange_by_problem_epoch[epoch_index]) {
                CarrierPhaseFactor pseudorange_observation;
                pseudorange_observation.epoch_index = epoch_index;
                pseudorange_observation.satellite = carrier.satellite;
                pseudorange_observation.clock_group =
                    clockBiasGroup(carrier.satellite.system);
                pseudorange_observation.signal = carrier.signal;
                pseudorange_observation.satellite_position_ecef =
                    carrier.satellite_position_ecef;
                pseudorange_observation.corrected_pseudorange_m =
                    carrier.corrected_pseudorange_m;
                pseudorange_observation.corrected_carrier_m =
                    carrier.corrected_carrier_m;
                pseudorange_observation.wavelength_m = carrier.wavelength_m;
                pseudorange_observation.sigma_m = carrier.sigma_m;
                pseudorange_observation.elevation_rad = carrier.elevation_rad;
                pseudorange_observation.has_carrier_phase =
                    carrier.has_carrier_phase;
                pseudorange_observation.loss_of_lock = carrier.loss_of_lock;
                pseudorange_observation.has_doppler_residual =
                    carrier.has_doppler_residual;
                pseudorange_observation.doppler_residual_mps =
                    carrier.doppler_residual_mps;
                pseudorange_observation.doppler_sigma_mps =
                    carrier.doppler_sigma_mps;
                pseudorange_observation.los = carrier.los;
                pseudorange_observation.model_debug = carrier.model_debug;
                problem.double_difference_pseudorange_observations.push_back(
                    pseudorange_observation);
            }
        }
    }

    if (config_.use_tdcp_factors && problem.epochs.size() >= 2) {
        const double sigma = std::max(1e-4, config_.tdcp_sigma_m);
        const double max_gap = std::max(0.0, config_.max_tdcp_gap_s);
        for (std::size_t epoch_index = 1; epoch_index < problem.epochs.size(); ++epoch_index) {
            const double dt = problem.epochs[epoch_index].time -
                              problem.epochs[epoch_index - 1].time;
            if (dt <= 0.0 || (max_gap > 0.0 && dt > max_gap)) {
                problem.diagnostics.tdcp_rejected_gap +=
                    carrier_by_problem_epoch[epoch_index].size();
                continue;
            }

            const auto& previous_carriers = carrier_by_problem_epoch[epoch_index - 1];
            const auto& current_carriers = carrier_by_problem_epoch[epoch_index];
            for (const auto& [key, current] : current_carriers) {
                const auto previous_it = previous_carriers.find(key);
                if (previous_it == previous_carriers.end()) {
                    ++problem.diagnostics.tdcp_rejected_missing_previous;
                    continue;
                }
                const auto& previous = previous_it->second;
                ++problem.diagnostics.tdcp_candidate_pairs;
                if (config_.reject_tdcp_loss_of_lock &&
                    (previous.loss_of_lock || current.loss_of_lock)) {
                    ++problem.diagnostics.tdcp_rejected_loss_of_lock;
                    continue;
                }

                const double delta_carrier_m =
                    current.corrected_carrier_m - previous.corrected_carrier_m;
                const double delta_code_m =
                    current.corrected_pseudorange_m - previous.corrected_pseudorange_m;
                const double code_phase_jump_m = std::abs(delta_carrier_m - delta_code_m);
                if (config_.reject_tdcp_code_phase_jump &&
                    config_.tdcp_code_phase_jump_threshold_m > 0.0 &&
                    code_phase_jump_m > config_.tdcp_code_phase_jump_threshold_m) {
                    ++problem.diagnostics.tdcp_rejected_code_phase_jump;
                    continue;
                }

                TimeDifferencedCarrierFactor factor;
                factor.previous_epoch_index = epoch_index - 1;
                factor.current_epoch_index = epoch_index;
                factor.satellite = current.satellite;
                factor.signal = current.signal;
                factor.previous_satellite_position_ecef = previous.satellite_position_ecef;
                factor.current_satellite_position_ecef = current.satellite_position_ecef;
                factor.delta_carrier_m = delta_carrier_m;
                factor.sigma_m = sigma;
                factor.dt_s = dt;
                problem.tdcp_factors.push_back(factor);
            }
        }
    }

    if (config_.use_double_difference_factors) {
        for (std::size_t epoch_index = 0;
             epoch_index < dd_reference_carrier_by_problem_epoch.size();
             ++epoch_index) {
            for (const auto& [key, carrier] :
                 dd_reference_carrier_by_problem_epoch[epoch_index]) {
                CarrierPhaseFactor reference_observation;
                reference_observation.epoch_index = epoch_index;
                reference_observation.satellite = carrier.satellite;
                reference_observation.clock_group =
                    clockBiasGroup(carrier.satellite.system);
                reference_observation.signal = carrier.signal;
                reference_observation.satellite_position_ecef =
                    carrier.satellite_position_ecef;
                reference_observation.corrected_pseudorange_m =
                    carrier.corrected_pseudorange_m;
                reference_observation.corrected_carrier_m =
                    carrier.corrected_carrier_m;
                reference_observation.wavelength_m = carrier.wavelength_m;
                reference_observation.sigma_m = carrier.sigma_m;
                reference_observation.elevation_rad = carrier.elevation_rad;
                reference_observation.has_carrier_phase = carrier.has_carrier_phase;
                reference_observation.loss_of_lock = carrier.loss_of_lock;
                reference_observation.has_doppler_residual =
                    carrier.has_doppler_residual;
                reference_observation.doppler_residual_mps =
                    carrier.doppler_residual_mps;
                reference_observation.doppler_sigma_mps =
                    carrier.doppler_sigma_mps;
                reference_observation.los = carrier.los;
                reference_observation.model_debug = carrier.model_debug;
                problem.double_difference_reference_observations.push_back(
                    reference_observation);
            }
        }
    }

    return problem;
}

FGOProcessor::FGOProblem FGOProcessor::buildDoubleDifferenceProblem(
    const std::vector<ObservationData>& rover_epochs,
    const std::vector<ObservationData>& base_epochs,
    const NavigationData& nav,
    const Vector3d& base_position_ecef) const {
    FGOProblem problem = buildPseudorangeProblem(rover_epochs, nav);
    if (!config_.use_double_difference_factors ||
        problem.carrier_observations.empty() ||
        base_epochs.empty() ||
        base_position_ecef.norm() <= 1e6) {
        return problem;
    }

    std::map<std::size_t, std::vector<const CarrierPhaseFactor*>>
        rover_carriers_by_epoch;
    for (const auto& factor : problem.carrier_observations) {
        if (factor.epoch_index < problem.epochs.size()) {
            rover_carriers_by_epoch[factor.epoch_index].push_back(&factor);
        }
    }
    std::map<std::size_t, std::vector<const CarrierPhaseFactor*>>
        rover_pseudoranges_by_epoch;
    for (const auto& factor :
         problem.double_difference_pseudorange_observations) {
        if (factor.epoch_index < problem.epochs.size()) {
            rover_pseudoranges_by_epoch[factor.epoch_index].push_back(&factor);
        }
    }
    std::map<std::size_t, std::vector<const CarrierPhaseFactor*>>
        rover_reference_carriers_by_epoch;
    for (const auto& factor : problem.double_difference_reference_observations) {
        if (factor.epoch_index < problem.epochs.size()) {
            rover_reference_carriers_by_epoch[factor.epoch_index].push_back(&factor);
        }
    }

    // MF hygiene: the base receiver gets its own per-(system, secondary band)
    // code table, mirroring the rover-side table in buildPseudorangeProblem.
    SecondaryCodeTable base_secondary_codes;
    const SecondaryCodeTable* base_secondary_codes_ptr = nullptr;
    if (config_.use_multi_frequency_double_difference &&
        config_.use_double_difference_secondary_code_alignment &&
        config_.use_multi_constellation) {
        base_secondary_codes = buildSecondaryCodeTable(base_epochs);
        base_secondary_codes_ptr = &base_secondary_codes;
    }

    std::size_t base_cursor = 0;
    const double match_tolerance = std::max(0.0, config_.base_epoch_match_tolerance_s);
    const double pseudorange_sigma =
        std::max(1e-3, config_.double_difference_pseudorange_sigma_m);
    const double carrier_sigma =
        std::max(1e-4, config_.double_difference_carrier_sigma_m);
    // Elevation-dependent DD sigma ("varerr", FGOConfig::use_elevation_
    // dependent_sigma): el_min_rad is the pair-elevation floor (reference:
    // np.radians(max(1.0, cfg.el_mask_deg))); epoch_dt_s is the rover
    // epoch-to-epoch interval fed to the clock-stability term, persisted
    // across the epoch loop below and only updated on a positive delta --
    // mirroring the reference's tc._epoch_dt / _update_epoch_dt exactly
    // (state carried across epochs, default 0.2 s until the first update;
    // see the knob's doc comment in fgo.hpp for why 0.2 s is also the right
    // fallback for this dataset).
    const double el_min_rad = M_PI / 180.0 * std::max(1.0, config_.min_elevation_deg);
    double epoch_dt_s = 0.2;
    const double max_segment_gap = std::max(0.0, config_.max_tdcp_gap_s);
    std::map<DoubleDifferenceAmbiguityKey, ActiveCarrierSegment>
        active_dd_segments;
    std::map<DoubleDifferenceSegmentKey, std::size_t> next_dd_segment_indices;
    std::map<SingleDifferenceAmbiguityKey, ActiveCarrierSegment>
        active_sd_segments;
    std::map<CarrierKey, std::size_t> next_sd_segment_indices;
    std::map<CarrierKey, std::size_t> sd_integrity_segment_indices;
    using DdFactorKey =
        std::tuple<std::size_t, SatelliteId, SatelliteId, SignalType>;
    std::map<SingleDifferenceReferenceKey, SatelliteId> sd_reference_by_group;
    std::map<SingleDifferenceDefaultReferenceKey, std::map<SatelliteId, std::size_t>>
        sd_default_reference_counts;

    // --- CMC screening state (function-scope, persists across the epoch
    // loop below; see FGOConfig::use_code_minus_carrier_screening). ---
    std::map<CarrierKey, CmcState> cmc_states;
    // Last rover-side (single-receiver) ambiguity_index seen per (satellite,
    // signal); a change here (cycle slip / loss-of-lock / outage causing the
    // rover carrier arc in buildPseudorangeProblem to restart) is this port's
    // signal to also reset the CMC baseline for that key (deviation from the
    // reference documented on the config knob).
    std::map<CarrierKey, std::size_t> cmc_last_rover_ambiguity_index;
    std::size_t cmc_jump_reset_total = 0;
    std::size_t cmc_level_exclusion_total = 0;
    // FGOConfig::cmc_aware_reference_selection: count of DD reference
    // picks steered away from a CMC-level-excluded candidate this run.
    std::size_t cmc_ref_avoided_total = 0;

    for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size(); ++epoch_index) {
        // Reference: tc._update_epoch_dt(obs) runs unconditionally at the top
        // of every epoch's processing, before any DD-eligibility gates below
        // -- mirrored here the same way (only meaningful when varerr is on).
        if (config_.use_elevation_dependent_sigma && epoch_index > 0) {
            const double dt = problem.epochs[epoch_index].time -
                              problem.epochs[epoch_index - 1].time;
            if (dt > 0.0) {
                epoch_dt_s = dt;
            }
        }
        const auto rover_it = rover_carriers_by_epoch.find(epoch_index);
        const auto rover_pseudorange_it =
            rover_pseudoranges_by_epoch.find(epoch_index);
        const auto rover_reference_it =
            rover_reference_carriers_by_epoch.find(epoch_index);
        if ((rover_it == rover_carriers_by_epoch.end() ||
             rover_it->second.empty()) &&
            (rover_pseudorange_it == rover_pseudoranges_by_epoch.end() ||
             rover_pseudorange_it->second.empty())) {
            continue;
        }

        ObservationData matched_base_epoch;
        bool interpolated_base_epoch = false;
        const bool has_base_epoch =
            findMatchedBaseEpoch(base_epochs,
                                 problem.epochs[epoch_index].time,
                                 base_cursor,
                                 match_tolerance,
                                 config_.base_interpolation_max_gap_s,
                                 matched_base_epoch,
                                 interpolated_base_epoch);
        if (!has_base_epoch) {
            problem.diagnostics.double_difference_rejected_no_base_epoch +=
                rover_it == rover_carriers_by_epoch.end()
                    ? 0
                    : rover_it->second.size();
            continue;
        }
        ++problem.diagnostics.double_difference_matched_base_epochs;
        if (interpolated_base_epoch) {
            ++problem.diagnostics.double_difference_interpolated_base_epochs;
        }

        std::map<std::pair<GNSSSystem, SignalType>, std::vector<const CarrierPhaseFactor*>>
            grouped_rover_pseudoranges;
        std::map<std::pair<GNSSSystem, SignalType>, std::vector<const CarrierPhaseFactor*>>
            grouped_rover_carriers;
        std::map<CarrierKey, const CarrierPhaseFactor*> rover_references_by_key;
        if (rover_reference_it != rover_reference_carriers_by_epoch.end()) {
            for (const auto* reference_factor : rover_reference_it->second) {
                rover_references_by_key[{reference_factor->satellite,
                                          reference_factor->signal}] =
                    reference_factor;
            }
        }
        std::map<std::pair<GNSSSystem, SignalType>,
                 std::vector<const PreparedCarrierObservation*>>
            grouped_reference_observations;
        const double base_min_snr_dbhz =
            config_.double_difference_base_min_snr_dbhz >= 0.0
                ? config_.double_difference_base_min_snr_dbhz
                : config_.min_snr_dbhz;
        const double reference_min_snr_dbhz =
            config_.double_difference_reference_min_snr_dbhz >= 0.0
                ? config_.double_difference_reference_min_snr_dbhz
                : config_.min_snr_dbhz;
        const auto base_carriers =
            prepareCarrierObservationsForReceiver(
                matched_base_epoch,
                nav,
                base_position_ecef,
                config_,
                base_min_snr_dbhz,
                true,
                false,
                base_secondary_codes_ptr);
        const auto base_pseudorange_observations =
            prepareCarrierObservationsForReceiver(
                matched_base_epoch,
                nav,
                base_position_ecef,
                config_,
                base_min_snr_dbhz,
                false,
                false,
                base_secondary_codes_ptr);
        const auto base_reference_observations =
            prepareCarrierObservationsForReceiver(
                matched_base_epoch,
                nav,
                base_position_ecef,
                config_,
                reference_min_snr_dbhz,
                false,
                false,
                base_secondary_codes_ptr);

        // --- CMC screening pre-pass (this epoch) ---
        //
        // Runs once per (satellite, signal) that has usable carrier at BOTH
        // rover and base this epoch (same requirement as the reference:
        // pr/cp nonzero on both sides), independent of which satellite ends
        // up the DD reference for its (system, signal) group below. Results
        // gate the PR-factor loop (level exclusion, which -- because the CP
        // loop only builds a factor when a matching PR factor already exists
        // -- transitively also excludes the CP factor for the same epoch,
        // matching the reference's single `continue` before either) and the
        // CP loop's ambiguity-arc bookkeeping (jump forces a new arc).
        std::set<CarrierKey> cmc_level_exclude_this_epoch;
        std::set<CarrierKey> cmc_jump_reset_this_epoch;
        if (config_.use_code_minus_carrier_screening &&
            rover_it != rover_carriers_by_epoch.end()) {
            for (const auto* rover_factor : rover_it->second) {
                const CarrierKey key{rover_factor->satellite, rover_factor->signal};
                const auto base_it = base_carriers.find(key);
                if (base_it == base_carriers.end()) {
                    continue;
                }
                const auto& base_carrier = base_it->second;
                const double pr_rover = rover_factor->model_debug.raw_pseudorange_m;
                const double cp_rover = rover_factor->model_debug.raw_carrier_m;
                const double pr_base = base_carrier.model_debug.raw_pseudorange_m;
                const double cp_base = base_carrier.model_debug.raw_carrier_m;
                if (pr_rover == 0.0 || cp_rover == 0.0 || pr_base == 0.0 ||
                    cp_base == 0.0) {
                    continue;
                }
                const double cmc = (pr_rover - pr_base) - (cp_rover - cp_base);

                // Deviation from the reference: any rover-side arc restart
                // (cycle slip / loss-of-lock / outage -- surfaced here as a
                // change in the rover carrier's own ambiguity_index) resets
                // this key's CMC baseline/prev/warmup state, since CMC
                // embeds a -wavelength*N term that a new ambiguity
                // invalidates.
                const auto last_arc_it =
                    cmc_last_rover_ambiguity_index.find(key);
                const bool rover_arc_restarted =
                    last_arc_it == cmc_last_rover_ambiguity_index.end() ||
                    last_arc_it->second != rover_factor->ambiguity_index;
                cmc_last_rover_ambiguity_index[key] = rover_factor->ambiguity_index;

                CmcState& state = cmc_states[key];
                if (rover_arc_restarted) {
                    state = CmcState{};
                } else if (state.has_prev &&
                           std::abs(cmc - state.prev_cmc_m) >
                               config_.code_minus_carrier_jump_threshold_m) {
                    cmc_jump_reset_this_epoch.insert(key);
                    ++cmc_jump_reset_total;
                    // Same reasoning as above: the jump itself is a fresh
                    // arc from this epoch on, so the baseline resets too.
                    state = CmcState{};
                }

                state.prev_cmc_m = cmc;
                state.has_prev = true;

                const double level_threshold =
                    config_.code_minus_carrier_level_threshold_m;
                if (level_threshold > 0.0) {
                    if (!state.has_baseline) {
                        state.baseline_m = cmc;
                        state.warmup_count = 1;
                        state.has_baseline = true;
                    } else if (state.warmup_count <
                               config_.code_minus_carrier_warmup_epochs) {
                        state.baseline_m =
                            (state.baseline_m * state.warmup_count + cmc) /
                            (state.warmup_count + 1);
                        ++state.warmup_count;
                    } else if (std::abs(cmc - state.baseline_m) > level_threshold) {
                        cmc_level_exclude_this_epoch.insert(key);
                        ++cmc_level_exclusion_total;
                    } else {
                        const double alpha = config_.code_minus_carrier_baseline_alpha;
                        state.baseline_m =
                            (1.0 - alpha) * state.baseline_m + alpha * cmc;
                    }
                }
            }
        }
        for (const auto& key : cmc_jump_reset_this_epoch) {
            ++sd_integrity_segment_indices[key];
        }

        if (rover_pseudorange_it != rover_pseudoranges_by_epoch.end()) {
            for (const auto* rover_factor : rover_pseudorange_it->second) {
                const CarrierKey key{rover_factor->satellite,
                                     rover_factor->signal};
                if (base_pseudorange_observations.find(key) ==
                    base_pseudorange_observations.end()) {
                    continue;
                }
                grouped_rover_pseudoranges[{rover_factor->satellite.system,
                                            rover_factor->signal}]
                    .push_back(rover_factor);
            }
        }
        if (rover_it != rover_carriers_by_epoch.end()) {
            for (const auto* rover_factor : rover_it->second) {
                const CarrierKey key{rover_factor->satellite,
                                     rover_factor->signal};
                if (base_carriers.find(key) == base_carriers.end()) {
                    continue;
                }
                grouped_rover_carriers[{rover_factor->satellite.system,
                                        rover_factor->signal}]
                    .push_back(rover_factor);
            }
        }
        for (const auto& [key, reference_observation] :
             base_reference_observations) {
            grouped_reference_observations[{reference_observation.satellite.system,
                                            reference_observation.signal}]
                .push_back(&reference_observation);
        }

        const auto select_reference =
            [&](const std::pair<GNSSSystem, SignalType>& group_key,
                std::size_t rejected_count,
                const CarrierPhaseFactor*& reference,
                const PreparedCarrierObservation*& base_reference) {
                if (group_key.first == GNSSSystem::GLONASS) {
                    problem.diagnostics.double_difference_rejected_no_reference +=
                        rejected_count;
                    return false;
                }

                const auto reference_group_it =
                    grouped_reference_observations.find(group_key);
                if (reference_group_it == grouped_reference_observations.end() ||
                    reference_group_it->second.empty()) {
                    problem.diagnostics.double_difference_rejected_no_reference +=
                        rejected_count;
                    return false;
                }

                const PreparedCarrierObservation* base_reference_selection =
                    *std::max_element(
                        reference_group_it->second.begin(),
                        reference_group_it->second.end(),
                        [](const PreparedCarrierObservation* lhs,
                           const PreparedCarrierObservation* rhs) {
                            return lhs->elevation_rad < rhs->elevation_rad;
                        });

                // --- CMC-aware reference selection
                // (FGOConfig::cmc_aware_reference_selection) ---
                // The plain max-elevation pick above can land on a satellite
                // the CMC screening pre-pass above already flagged as
                // sustained multipath/NLOS this epoch
                // (cmc_level_exclude_this_epoch). Every DD pair in the group
                // is formed against this ONE reference, so a multipath-
                // biased reference poisons every DD residual in the group at
                // once (see the config knob's doc comment for the tokyo
                // run1 episode that motivated this). When the knob is on and
                // CMC screening is active, prefer the highest-elevation
                // NON-excluded candidate instead; if every candidate in the
                // group is CMC-excluded this epoch, keep the original
                // max-elevation choice so the group is never dropped.
                if (config_.cmc_aware_reference_selection &&
                    config_.use_code_minus_carrier_screening &&
                    !cmc_level_exclude_this_epoch.empty()) {
                    const CarrierKey default_key{
                        base_reference_selection->satellite,
                        base_reference_selection->signal};
                    if (cmc_level_exclude_this_epoch.count(default_key) > 0) {
                        const PreparedCarrierObservation* best_clean = nullptr;
                        for (const auto* candidate : reference_group_it->second) {
                            const CarrierKey candidate_key{candidate->satellite,
                                                            candidate->signal};
                            if (cmc_level_exclude_this_epoch.count(candidate_key) >
                                0) {
                                continue;
                            }
                            if (!best_clean || candidate->elevation_rad >
                                                    best_clean->elevation_rad) {
                                best_clean = candidate;
                            }
                        }
                        if (best_clean) {
                            base_reference_selection = best_clean;
                            ++cmc_ref_avoided_total;
                        }
                        // else: every candidate in the group is CMC-excluded
                        // -- fall back to the original max-elevation choice
                        // (base_reference_selection unchanged) rather than
                        // dropping the whole group.
                    }
                }

                const CarrierKey reference_key{
                    base_reference_selection->satellite,
                    base_reference_selection->signal};
                const auto rover_reference_by_key_it =
                    rover_references_by_key.find(reference_key);
                if (rover_reference_by_key_it == rover_references_by_key.end()) {
                    problem.diagnostics.double_difference_rejected_no_reference +=
                        rejected_count;
                    return false;
                }
                reference = rover_reference_by_key_it->second;
                base_reference = base_reference_selection;
                return true;
            };

        std::map<DdFactorKey, DoubleDifferencePseudorangeFactor>
            pseudorange_factors_by_key;
        // Includes CMC-level-excluded code observations solely for carrier
        // ambiguity initialization when code-only exclusion is requested.
        std::map<DdFactorKey, DoubleDifferencePseudorangeFactor>
            pseudorange_seed_factors_by_key;
        for (const auto& [group_key, group] : grouped_rover_pseudoranges) {
            if (group_key.first == GNSSSystem::GLONASS) {
                problem.diagnostics.double_difference_rejected_no_reference +=
                    group.size();
                continue;
            }

            const CarrierPhaseFactor* reference = nullptr;
            const PreparedCarrierObservation* base_reference_ptr = nullptr;
            if (!select_reference(group_key,
                                  group.size(),
                                  reference,
                                  base_reference_ptr)) {
                continue;
            }
            const auto& base_reference = *base_reference_ptr;
            for (const auto* satellite : group) {
                if (satellite->satellite == reference->satellite &&
                    satellite->signal == reference->signal) {
                    continue;
                }
                const CarrierKey satellite_key{satellite->satellite, satellite->signal};
                const bool cmc_level_excluded =
                    cmc_level_exclude_this_epoch.count(satellite_key) > 0;
                if (cmc_level_excluded &&
                    !config_.code_minus_carrier_level_pseudorange_only) {
                    // Sustained CMC multipath this epoch: skip the DD
                    // pseudorange factor. The CP loop below only builds a
                    // carrier factor when it finds a matching entry in
                    // pseudorange_factors_by_key, so omitting the PR factor
                    // here transitively excludes the CP factor too (matches
                    // the reference's single `continue` before either).
                    continue;
                }
                const auto base_satellite_it =
                    base_pseudorange_observations.find(satellite_key);
                if (base_satellite_it == base_pseudorange_observations.end()) {
                    continue;
                }

                const auto& base_satellite = base_satellite_it->second;
                const double satellite_sin_el =
                    std::max(0.1, std::sin(satellite->elevation_rad));
                const double satellite_sqrt_sin_el =
                    std::sqrt(satellite_sin_el);

                DoubleDifferencePseudorangeFactor pseudorange_factor;
                pseudorange_factor.epoch_index = epoch_index;
                pseudorange_factor.satellite = satellite->satellite;
                pseudorange_factor.reference_satellite = reference->satellite;
                pseudorange_factor.signal = satellite->signal;
                pseudorange_factor.rover_satellite_position_ecef =
                    satellite->satellite_position_ecef;
                pseudorange_factor.rover_reference_position_ecef =
                    reference->satellite_position_ecef;
                pseudorange_factor.base_satellite_position_ecef =
                    base_satellite.satellite_position_ecef;
                pseudorange_factor.base_reference_position_ecef =
                    base_reference.satellite_position_ecef;
                pseudorange_factor.base_position_ecef = base_position_ecef;
                pseudorange_factor.observed_dd_pseudorange_m =
                    (satellite->corrected_pseudorange_m -
                     base_satellite.corrected_pseudorange_m) -
                    (reference->corrected_pseudorange_m -
                     base_reference.corrected_pseudorange_m);
                const double pseudorange_band_scale =
                    signal_policy::isSecondarySignal(satellite->satellite.system,
                                                     satellite->signal)
                        ? std::max(1.0,
                                   config_
                                       .double_difference_secondary_pseudorange_sigma_scale)
                        : 1.0;
                // Base DD-PR sigma: varerr (FGOConfig::use_elevation_
                // dependent_sigma) when enabled, else the pre-existing flat/
                // elevation-power fallback -- see the knob's doc comment in
                // fgo.hpp for the full rationale/composition order. The
                // secondary-band scale above still multiplies on top either
                // way (unchanged).
                double pseudorange_base_sigma = pseudorange_sigma / satellite_sqrt_sin_el;
                if (config_.use_elevation_dependent_sigma) {
                    const double el_pair_rad = std::max(
                        std::min(reference->elevation_rad, satellite->elevation_rad),
                        el_min_rad);
                    pseudorange_base_sigma = varerrDdSigma(
                        /*is_pseudorange=*/true, el_pair_rad, epoch_dt_s, config_);
                }
                pseudorange_factor.sigma_m =
                    pseudorange_band_scale * pseudorange_base_sigma;
                pseudorange_factor.elevation_rad = satellite->elevation_rad;
                pseudorange_factor.rover_satellite_model =
                    satellite->model_debug;
                pseudorange_factor.rover_reference_model =
                    reference->model_debug;
                pseudorange_factor.base_satellite_model =
                    base_satellite.model_debug;
                pseudorange_factor.base_reference_model =
                    base_reference.model_debug;
                const DdFactorKey factor_key{
                    epoch_index,
                    satellite->satellite,
                    reference->satellite,
                    satellite->signal,
                };
                pseudorange_seed_factors_by_key[factor_key] = pseudorange_factor;
                if (!cmc_level_excluded) {
                    problem.double_difference_pseudorange_factors.push_back(
                        pseudorange_factor);
                    pseudorange_factors_by_key[factor_key] = pseudorange_factor;
                }
            }
        }

        for (const auto& [group_key, group] : grouped_rover_carriers) {
            const CarrierPhaseFactor* reference = nullptr;
            const PreparedCarrierObservation* base_reference_ptr = nullptr;
            if (!select_reference(group_key,
                                  group.size(),
                                  reference,
                                  base_reference_ptr)) {
                continue;
            }
            const auto& base_reference = *base_reference_ptr;

            for (const auto* satellite : group) {
                if (satellite->satellite == reference->satellite &&
                    satellite->signal == reference->signal) {
                    continue;
                }
                const CarrierKey satellite_key{satellite->satellite, satellite->signal};
                const auto base_satellite_it = base_carriers.find(satellite_key);
                if (base_satellite_it == base_carriers.end()) {
                    continue;
                }
                const DdFactorKey factor_key{
                        epoch_index,
                        satellite->satellite,
                        reference->satellite,
                        satellite->signal,
                    };
                const auto pseudorange_factor_it = pseudorange_factors_by_key.find(factor_key);
                const DoubleDifferencePseudorangeFactor* pseudorange_factor_ptr =
                    pseudorange_factor_it != pseudorange_factors_by_key.end()
                        ? &pseudorange_factor_it->second
                        : nullptr;
                if (!pseudorange_factor_ptr &&
                    config_.code_minus_carrier_level_pseudorange_only) {
                    const auto seed_it = pseudorange_seed_factors_by_key.find(factor_key);
                    if (seed_it != pseudorange_seed_factors_by_key.end()) {
                        pseudorange_factor_ptr = &seed_it->second;
                    }
                }
                if (!pseudorange_factor_ptr) {
                    // Sustained CMC multipath dropped this satellite's DD-PR
                    // factor at problem-build time (and, transitively, this
                    // DD-CP row -- code_minus_carrier_level_pseudorange_only
                    // is false, else pseudorange_factor_ptr would have come
                    // from the seed map above). Retain a minimal excluded
                    // row -- geometry + observed DD carrier, no ambiguity/
                    // segment tracking (ambiguity_index is the sentinel
                    // max()) -- solely so the surplus-satellite independent
                    // integrity validator (FGOConfig::use_surplus_
                    // satellite_validation) can see observations that never
                    // entered ambiguity resolution at all. NEVER added to
                    // double_difference_carrier_factors / the solved graph.
                    if (cmc_level_exclude_this_epoch.count(satellite_key) > 0 &&
                        reference->has_carrier_phase && base_reference.has_carrier_phase) {
                        const auto& base_satellite_excl = base_satellite_it->second;
                        DoubleDifferenceCarrierFactor excluded;
                        excluded.epoch_index = epoch_index;
                        excluded.use_ambiguity_difference = false;
                        excluded.ambiguity_index = std::numeric_limits<std::size_t>::max();
                        excluded.satellite = satellite->satellite;
                        excluded.reference_satellite = reference->satellite;
                        excluded.signal = satellite->signal;
                        excluded.rover_satellite_position_ecef = satellite->satellite_position_ecef;
                        excluded.rover_reference_position_ecef = reference->satellite_position_ecef;
                        excluded.base_satellite_position_ecef =
                            base_satellite_excl.satellite_position_ecef;
                        excluded.base_reference_position_ecef =
                            base_reference.satellite_position_ecef;
                        excluded.base_position_ecef = base_position_ecef;
                        excluded.observed_dd_carrier_m =
                            (satellite->corrected_carrier_m -
                             base_satellite_excl.corrected_carrier_m) -
                            (reference->corrected_carrier_m -
                             base_reference.corrected_carrier_m);
                        excluded.sigma_m = carrier_sigma;  // unused by the surplus validator
                        excluded.elevation_rad = satellite->elevation_rad;
                        problem.excluded_double_difference_carrier_factors.push_back(excluded);
                    }
                    continue;
                }
                ++problem.diagnostics.double_difference_candidate_pairs;

                const auto& base_satellite = base_satellite_it->second;
                const auto& pseudorange_factor = *pseudorange_factor_ptr;
                const double satellite_sin_el =
                    std::max(0.1, std::sin(satellite->elevation_rad));
                const double satellite_sqrt_sin_el =
                    std::sqrt(satellite_sin_el);

                if (!reference->has_carrier_phase ||
                    !base_reference.has_carrier_phase) {
                    continue;
                }

                DoubleDifferenceCarrierFactor factor;
                factor.epoch_index = epoch_index;
                factor.use_ambiguity_difference =
                    config_.use_multisd_ambiguities;
                factor.satellite = satellite->satellite;
                factor.reference_satellite = reference->satellite;
                factor.signal = satellite->signal;
                factor.rover_satellite_position_ecef =
                    satellite->satellite_position_ecef;
                factor.rover_reference_position_ecef =
                    reference->satellite_position_ecef;
                factor.base_satellite_position_ecef =
                    base_satellite.satellite_position_ecef;
                factor.base_reference_position_ecef =
                    base_reference.satellite_position_ecef;
                factor.base_position_ecef = base_position_ecef;
                factor.observed_dd_carrier_m =
                    (satellite->corrected_carrier_m -
                     base_satellite.corrected_carrier_m) -
                    (reference->corrected_carrier_m -
                     base_reference.corrected_carrier_m);
                const double carrier_band_scale =
                    signal_policy::isSecondarySignal(satellite->satellite.system,
                                                     satellite->signal)
                        ? std::max(1.0,
                                   config_
                                       .double_difference_secondary_carrier_sigma_scale)
                        : 1.0;
                // Base DD-CP sigma: varerr when enabled, else the
                // pre-existing flat/elevation-power fallback -- same source
                // switch as the DD-PR site above (see that comment and the
                // knob's doc comment in fgo.hpp). Secondary-band scale still
                // multiplies on top either way.
                double carrier_base_sigma = carrier_sigma / satellite_sqrt_sin_el;
                if (config_.use_elevation_dependent_sigma) {
                    const double el_pair_rad = std::max(
                        std::min(reference->elevation_rad, satellite->elevation_rad),
                        el_min_rad);
                    carrier_base_sigma = varerrDdSigma(
                        /*is_pseudorange=*/false, el_pair_rad, epoch_dt_s, config_);
                }
                factor.sigma_m = carrier_band_scale * carrier_base_sigma;
                factor.elevation_rad = satellite->elevation_rad;
                factor.rover_satellite_model = satellite->model_debug;
                factor.rover_reference_model = reference->model_debug;
                factor.base_satellite_model = base_satellite.model_debug;
                factor.base_reference_model = base_reference.model_debug;

                const Vector3d seed_position =
                    problem.epochs[epoch_index].position_ecef;
                const double rover_satellite_range =
                    (factor.rover_satellite_position_ecef - seed_position).norm();
                const double rover_reference_range =
                    (factor.rover_reference_position_ecef - seed_position).norm();
                const double base_satellite_range =
                    (factor.base_satellite_position_ecef - base_position_ecef).norm();
                const double base_reference_range =
                    (factor.base_reference_position_ecef - base_position_ecef).norm();

                const DoubleDifferenceAmbiguityKey ambiguity_key{
                    satellite->satellite,
                    reference->satellite,
                    satellite->signal,
                    satellite->ambiguity_index,
                    0,
                };
                auto active_it = active_dd_segments.find(ambiguity_key);
                bool start_new_segment =
                    config_.reset_double_difference_ambiguities_each_epoch ||
                    active_it == active_dd_segments.end();
                if (!start_new_segment) {
                    const double dt = problem.epochs[epoch_index].time -
                                      problem.epochs[active_it->second.last_epoch_index].time;
                    if (dt <= 0.0 ||
                        (max_segment_gap > 0.0 && dt > max_segment_gap)) {
                        start_new_segment = true;
                    }
                }
                if (cmc_jump_reset_this_epoch.count(
                        CarrierKey{satellite->satellite, satellite->signal}) > 0) {
                    // CMC jump: force the same arc break the loss-of-lock /
                    // gap checks above already perform (multipath jump
                    // breaks the integer ambiguity).
                    start_new_segment = true;
                }

                if (config_.use_multisd_ambiguities) {
                    auto get_or_create_sd_ambiguity =
                        [&](const CarrierPhaseFactor& rover,
                            const PreparedCarrierObservation& base)
                            -> std::size_t {
                        const SingleDifferenceAmbiguityKey key{
                            rover.satellite, rover.signal,
                            rover.ambiguity_index,
                            sd_integrity_segment_indices[CarrierKey{
                                rover.satellite, rover.signal}]};
                        auto it = active_sd_segments.find(key);
                        if (it != active_sd_segments.end()) {
                            it->second.last_epoch_index = epoch_index;
                            return it->second.ambiguity_index;
                        }

                        AmbiguityState ambiguity;
                        ambiguity.satellite = rover.satellite;
                        ambiguity.reference_satellite = SatelliteId{};
                        ambiguity.signal = rover.signal;
                        ambiguity.is_double_difference = false;
                        ambiguity.segment_index =
                            next_sd_segment_indices[CarrierKey{
                                rover.satellite, rover.signal}]++;
                        ambiguity.wavelength_m = rover.wavelength_m;
                        // Carrier-minus-code removes the geometric range and
                        // clock terms from the rover-base SD seed. The noisy
                        // code only initializes the graph; carrier factors
                        // determine the optimized ambiguity.
                        ambiguity.initial_ambiguity_m =
                            (rover.corrected_carrier_m -
                             base.corrected_carrier_m) -
                            (rover.corrected_pseudorange_m -
                             base.corrected_pseudorange_m);
                        const std::size_t index =
                            problem.ambiguity_states.size();
                        problem.ambiguity_states.push_back(ambiguity);
                        active_sd_segments.emplace(
                            key, ActiveCarrierSegment{index, epoch_index});
                        return index;
                    };

                    factor.ambiguity_index = get_or_create_sd_ambiguity(
                        *satellite, base_satellite);
                    factor.reference_ambiguity_index =
                        get_or_create_sd_ambiguity(*reference,
                                                   base_reference);
                    problem.double_difference_carrier_factors.push_back(
                        factor);
                    continue;
                }

                if (start_new_segment) {
                    AmbiguityState ambiguity;
                    ambiguity.satellite = satellite->satellite;
                    ambiguity.reference_satellite = reference->satellite;
                    ambiguity.signal = satellite->signal;
                    ambiguity.is_double_difference = true;
                    ambiguity.segment_index =
                        next_dd_segment_indices[DoubleDifferenceSegmentKey{
                            satellite->satellite,
                            reference->satellite,
                            satellite->signal,
                        }]++;
                    ambiguity.wavelength_m = satellite->wavelength_m;
                    const double dd_geometry_at_seed =
                        (rover_satellite_range - base_satellite_range) -
                        (rover_reference_range - base_reference_range);
                    const double carrier_residual_at_seed =
                        factor.observed_dd_carrier_m - dd_geometry_at_seed;
                    const double pseudorange_residual_at_seed =
                        pseudorange_factor.observed_dd_pseudorange_m -
                        dd_geometry_at_seed;
                    ambiguity.initial_ambiguity_m =
                        carrier_residual_at_seed -
                        pseudorange_residual_at_seed;
                    active_it =
                        active_dd_segments
                            .insert_or_assign(
                                ambiguity_key,
                                ActiveCarrierSegment{problem.ambiguity_states.size(),
                                                     epoch_index})
                            .first;
                    problem.ambiguity_states.push_back(ambiguity);
                } else {
                    active_it->second.last_epoch_index = epoch_index;
                }

                factor.ambiguity_index = active_it->second.ambiguity_index;
                problem.double_difference_carrier_factors.push_back(factor);
            }
        }
    }

    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        sd_reference_by_group[SingleDifferenceReferenceKey{
            factor.epoch_index,
            factor.satellite.system,
            factor.signal,
        }] = factor.reference_satellite;
        ++sd_default_reference_counts[SingleDifferenceDefaultReferenceKey{
            factor.satellite.system,
            factor.signal,
        }][factor.reference_satellite];
    }
    for (const auto& factor : problem.double_difference_carrier_factors) {
        sd_reference_by_group[SingleDifferenceReferenceKey{
            factor.epoch_index,
            factor.satellite.system,
            factor.signal,
        }] = factor.reference_satellite;
        ++sd_default_reference_counts[SingleDifferenceDefaultReferenceKey{
            factor.satellite.system,
            factor.signal,
        }][factor.reference_satellite];
    }

    if (config_.use_ambiguity_between_factors) {
        using AmbiguityTrackKey = std::pair<SatelliteId, SignalType>;
        std::map<AmbiguityTrackKey, const DoubleDifferenceCarrierFactor*>
            previous_by_track;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
                continue;
            }
            const AmbiguityTrackKey track_key{factor.satellite, factor.signal};
            const auto previous_it = previous_by_track.find(track_key);
            if (previous_it != previous_by_track.end()) {
                const auto* previous = previous_it->second;
                const double dt =
                    problem.epochs[factor.epoch_index].time -
                    problem.epochs[previous->epoch_index].time;
                if (factor.epoch_index == previous->epoch_index + 1 &&
                    dt > 0.0 &&
                    (config_.max_tdcp_gap_s <= 0.0 ||
                     dt <= config_.max_tdcp_gap_s) &&
                    previous->ambiguity_index != factor.ambiguity_index) {
                    const auto& ambiguity =
                        problem.ambiguity_states[factor.ambiguity_index];
                    if (ambiguity.wavelength_m > 0.0) {
                        AmbiguityBetweenFactor between;
                        between.previous_epoch_index = previous->epoch_index;
                        between.current_epoch_index = factor.epoch_index;
                        between.previous_ambiguity_index =
                            previous->ambiguity_index;
                        between.current_ambiguity_index = factor.ambiguity_index;
                        between.satellite = factor.satellite;
                        between.signal = factor.signal;
                        between.sigma_m =
                            std::max(1e-9,
                                     config_.ambiguity_between_sigma_cycles *
                                         ambiguity.wavelength_m);
                        problem.ambiguity_between_factors.push_back(between);
                    }
                }
            }
            previous_by_track[track_key] = &factor;
        }
    }

    std::map<SingleDifferenceDefaultReferenceKey, SatelliteId>
        sd_default_reference_by_group;
    for (const auto& [group_key, counts] : sd_default_reference_counts) {
        const auto best_it =
            std::max_element(counts.begin(),
                             counts.end(),
                             [](const auto& lhs, const auto& rhs) {
                                 return lhs.second < rhs.second;
                             });
        if (best_it != counts.end()) {
            sd_default_reference_by_group[group_key] = best_it->first;
        }
    }

    if (config_.use_single_difference_doppler_factors ||
        config_.use_external_doppler_dr_validation ||
        config_.use_single_difference_tdcp_factors) {
        std::map<CarrierKey, SingleDifferenceCarrierResidual>
            previous_sd_carrier_residuals;
        for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size();
             ++epoch_index) {
            std::map<CarrierKey, const CarrierPhaseFactor*> rover_by_key;
            std::vector<const CarrierPhaseFactor*> rover_factor_rows;
            const auto rover_pseudorange_it =
                rover_pseudoranges_by_epoch.find(epoch_index);
            if (rover_pseudorange_it != rover_pseudoranges_by_epoch.end()) {
                for (const auto* rover : rover_pseudorange_it->second) {
                    rover_by_key[{rover->satellite, rover->signal}] = rover;
                    rover_factor_rows.push_back(rover);
                }
            }

            std::map<CarrierKey, SingleDifferenceCarrierResidual>
                current_sd_carrier_residuals;
            for (const auto* rover : rover_factor_rows) {
                const CarrierKey carrier_key{rover->satellite, rover->signal};
                const auto reference_it =
                    sd_reference_by_group.find(SingleDifferenceReferenceKey{
                        epoch_index,
                        rover->satellite.system,
                        rover->signal,
                    });
                SatelliteId reference_satellite;
                if (reference_it != sd_reference_by_group.end()) {
                    reference_satellite = reference_it->second;
                } else {
                    const auto default_reference_it =
                        sd_default_reference_by_group.find(
                            SingleDifferenceDefaultReferenceKey{
                                rover->satellite.system,
                                rover->signal,
                            });
                    if (default_reference_it ==
                            sd_default_reference_by_group.end() ||
                        !(rover->satellite == default_reference_it->second)) {
                        continue;
                    }
                    reference_satellite = default_reference_it->second;
                }

                const auto reference_model_it =
                    rover_by_key.find({reference_satellite, rover->signal});
                if (reference_model_it == rover_by_key.end()) {
                    continue;
                }
                const auto* reference = reference_model_it->second;
                // A satellite differenced with itself is identically zero
                // and adds no information (but can distort factor counts and
                // residual diagnostics).
                if (rover->satellite == reference_satellite) {
                    continue;
                }
                const Vector3d sd_los = rover->los - reference->los;

                if ((config_.use_single_difference_doppler_factors ||
                     config_.use_external_doppler_dr_validation) &&
                    rover->has_doppler_residual &&
                    reference->has_doppler_residual) {
                    SingleDifferenceDopplerFactor factor;
                    factor.epoch_index = epoch_index;
                    factor.satellite = rover->satellite;
                    factor.reference_satellite = reference_satellite;
                    factor.signal = rover->signal;
                    factor.los = sd_los;
                    factor.residual_mps =
                        rover->doppler_residual_mps -
                        reference->doppler_residual_mps;
                    factor.sigma_mps = std::hypot(rover->doppler_sigma_mps,
                                                  reference->doppler_sigma_mps);
                    factor.elevation_rad = rover->elevation_rad;
                    if (std::isfinite(factor.residual_mps) &&
                        std::isfinite(factor.sigma_mps) &&
                        factor.sigma_mps > 0.0) {
                        problem.single_difference_doppler_factors.push_back(factor);
                    }
                }

                if (rover->has_carrier_phase &&
                    reference->has_carrier_phase &&
                    !rover->loss_of_lock &&
                    !reference->loss_of_lock) {
                    const double rover_carrier_residual =
                        rover->corrected_carrier_m -
                        rover->model_debug.geometric_range_m;
                    const double reference_carrier_residual =
                        reference->corrected_carrier_m -
                        reference->model_debug.geometric_range_m;
                    const double sd_carrier_residual =
                        rover_carrier_residual - reference_carrier_residual;
                    current_sd_carrier_residuals[carrier_key] =
                        SingleDifferenceCarrierResidual{
                            epoch_index,
                            sd_carrier_residual,
                            sd_los,
                        };

                    const auto previous_it =
                        previous_sd_carrier_residuals.find(carrier_key);
                    if (config_.use_single_difference_tdcp_factors &&
                        previous_it != previous_sd_carrier_residuals.end()) {
                        const double tdcp =
                            sd_carrier_residual - previous_it->second.residual_m;
                        if (std::isfinite(tdcp) && tdcp != 0.0) {
                            SingleDifferenceTdcpFactor factor;
                            factor.previous_epoch_index =
                                previous_it->second.epoch_index;
                            factor.current_epoch_index = epoch_index;
                            factor.satellite = rover->satellite;
                            factor.reference_satellite = reference_satellite;
                            factor.signal = rover->signal;
                            factor.previous_los = sd_los;
                            factor.los = sd_los;
                            factor.delta_carrier_m = tdcp;
                            const double sin_el =
                                std::max(0.1, std::sin(rover->elevation_rad));
                            factor.sigma_m =
                                std::max(1e-4,
                                         config_.single_difference_tdcp_sigma_m /
                                             std::sqrt(sin_el));
                            factor.elevation_rad = rover->elevation_rad;
                            problem.single_difference_tdcp_factors.push_back(factor);
                        }
                    }
                }
            }
            previous_sd_carrier_residuals =
                std::move(current_sd_carrier_residuals);
        }
    }

    problem.diagnostics.code_minus_carrier_jump_resets = cmc_jump_reset_total;
    problem.diagnostics.code_minus_carrier_level_exclusions =
        cmc_level_exclusion_total;
    problem.diagnostics.cmc_ref_avoided_count = cmc_ref_avoided_total;

    return problem;
}

FGOProcessor::FGOResult FGOProcessor::optimize(
    const std::vector<ObservationData>& epochs,
    const NavigationData& nav) const {
    return optimizeProblem(buildPseudorangeProblem(epochs, nav));
}

FGOProcessor::FGOResult FGOProcessor::optimize(
    const std::vector<ObservationData>& rover_epochs,
    const std::vector<ObservationData>& base_epochs,
    const NavigationData& nav,
    const Vector3d& base_position_ecef) const {
    return optimizeProblem(
        buildDoubleDifferenceProblem(rover_epochs, base_epochs, nav, base_position_ecef));
}

FGOProcessor::FGOResult FGOProcessor::optimizeProblem(
    const FGOProblem& input_problem) const {
    const auto optimize_problem_start =
        std::chrono::high_resolution_clock::now();
    FGOResult result;

    // Select the validation satellites before either float or fixed
    // optimization, then remove every factor involving those satellites from
    // the complete window.  Selection uses only observation metadata from the
    // latest epoch (elevation and satellite id), never a solved state or truth.
    // The weakest-elevation ambiguities are held out, matching satellite-PAR:
    // strong geometry remains in the ILS candidate while the unused surplus
    // observations provide validation evidence.
    // This makes the post-fix evidence genuinely disjoint from candidate
    // generation, including all earlier epochs in a multi-epoch window.
    FGOProblem disjoint_problem;
    const FGOProblem* active_problem = &input_problem;
    if (config_.backend == FGOBackend::Eigen &&
        config_.use_multisd_ambiguities &&
        config_.use_multisd_disjoint_validation &&
        !input_problem.double_difference_carrier_factors.empty()) {
        disjoint_problem = input_problem;
        std::size_t latest_epoch = 0;
        for (const auto& factor :
             input_problem.double_difference_carrier_factors) {
            latest_epoch = std::max(latest_epoch, factor.epoch_index);
        }

        std::map<SatelliteId, double> latest_target_elevation;
        for (const auto& factor :
             input_problem.double_difference_carrier_factors) {
            if (factor.epoch_index == latest_epoch &&
                factor.use_ambiguity_difference) {
                auto [it, inserted] = latest_target_elevation.emplace(
                    factor.satellite, factor.elevation_rad);
                if (!inserted) {
                    it->second = std::max(it->second, factor.elevation_rad);
                }
            }
        }
        std::vector<std::pair<SatelliteId, double>> ranked_targets(
            latest_target_elevation.begin(), latest_target_elevation.end());
        std::stable_sort(
            ranked_targets.begin(), ranked_targets.end(),
            [](const auto& lhs, const auto& rhs) {
                if (lhs.second != rhs.second) {
                    return lhs.second < rhs.second;
                }
                return lhs.first < rhs.first;
            });
        const std::size_t requested = static_cast<std::size_t>(
            std::max(1, config_.multisd_validation_holdout_satellites));
        std::set<SatelliteId> held_out_satellites;
        const std::size_t offset = ranked_targets.empty()
            ? 0
            : static_cast<std::size_t>(
                  std::max(0, config_.multisd_validation_holdout_offset)) %
                  ranked_targets.size();
        for (std::size_t i = 0;
             i < ranked_targets.size() &&
             held_out_satellites.size() < requested;
             ++i) {
            const auto& [satellite, elevation] =
                ranked_targets[(offset + i) % ranked_targets.size()];
            (void)elevation;
            held_out_satellites.insert(satellite);
        }
        result.diagnostics.multisd_validation_holdout_satellites =
            held_out_satellites.size();

        auto involves_holdout = [&](const auto& factor) {
            return held_out_satellites.count(factor.satellite) > 0 ||
                   held_out_satellites.count(factor.reference_satellite) > 0;
        };
        auto& carriers = disjoint_problem.double_difference_carrier_factors;
        for (const auto& factor : carriers) {
            if (involves_holdout(factor)) {
                disjoint_problem.multisd_validation_carrier_factors.push_back(
                    factor);
            }
        }
        carriers.erase(
            std::remove_if(carriers.begin(), carriers.end(), involves_holdout),
            carriers.end());

        auto& pseudoranges =
            disjoint_problem.double_difference_pseudorange_factors;
        for (const auto& factor : pseudoranges) {
            if (involves_holdout(factor)) {
                disjoint_problem.multisd_validation_pseudorange_factors.push_back(
                    factor);
            }
        }
        pseudoranges.erase(
            std::remove_if(pseudoranges.begin(), pseudoranges.end(),
                           involves_holdout),
            pseudoranges.end());
        active_problem = &disjoint_problem;
    }
    const FGOProblem& problem = *active_problem;
    result.diagnostics.epochs = problem.epochs.size();
    result.diagnostics.pseudorange_factors = problem.pseudorange_factors.size();
    result.diagnostics.tdcp_factors = problem.tdcp_factors.size();
    result.diagnostics.single_difference_doppler_factors =
        problem.single_difference_doppler_factors.size();
    result.diagnostics.single_difference_tdcp_factors =
        problem.single_difference_tdcp_factors.size();
    result.diagnostics.carrier_phase_factors = problem.carrier_phase_factors.size();
    result.diagnostics.double_difference_pseudorange_factors =
        problem.double_difference_pseudorange_factors.size();
    result.diagnostics.double_difference_carrier_factors =
        problem.double_difference_carrier_factors.size();
    result.diagnostics.multisd_carrier_factors =
        static_cast<std::size_t>(std::count_if(
            problem.double_difference_carrier_factors.begin(),
            problem.double_difference_carrier_factors.end(),
            [](const DoubleDifferenceCarrierFactor& factor) {
                return factor.use_ambiguity_difference;
            }));
    result.diagnostics.ambiguity_between_factors =
        problem.ambiguity_between_factors.size();
    result.diagnostics.ambiguity_states = problem.ambiguity_states.size();
    result.diagnostics.tdcp_candidate_pairs = problem.diagnostics.tdcp_candidate_pairs;
    result.diagnostics.tdcp_rejected_gap = problem.diagnostics.tdcp_rejected_gap;
    result.diagnostics.tdcp_rejected_missing_previous =
        problem.diagnostics.tdcp_rejected_missing_previous;
    result.diagnostics.tdcp_rejected_loss_of_lock =
        problem.diagnostics.tdcp_rejected_loss_of_lock;
    result.diagnostics.tdcp_rejected_code_phase_jump =
        problem.diagnostics.tdcp_rejected_code_phase_jump;
    result.diagnostics.double_difference_matched_base_epochs =
        problem.diagnostics.double_difference_matched_base_epochs;
    result.diagnostics.double_difference_interpolated_base_epochs =
        problem.diagnostics.double_difference_interpolated_base_epochs;
    result.diagnostics.double_difference_candidate_pairs =
        problem.diagnostics.double_difference_candidate_pairs;
    result.diagnostics.double_difference_rejected_no_base_epoch =
        problem.diagnostics.double_difference_rejected_no_base_epoch;
    result.diagnostics.double_difference_rejected_no_reference =
        problem.diagnostics.double_difference_rejected_no_reference;
    result.diagnostics.code_minus_carrier_jump_resets =
        problem.diagnostics.code_minus_carrier_jump_resets;
    result.diagnostics.code_minus_carrier_level_exclusions =
        problem.diagnostics.code_minus_carrier_level_exclusions;
    result.diagnostics.cmc_ref_avoided_count =
        problem.diagnostics.cmc_ref_avoided_count;

    if (problem.epochs.empty() ||
        (problem.pseudorange_factors.empty() &&
         problem.carrier_phase_factors.empty() &&
         problem.double_difference_pseudorange_factors.empty() &&
         problem.double_difference_carrier_factors.empty() &&
         problem.tdcp_factors.empty() &&
         problem.single_difference_doppler_factors.empty() &&
         problem.single_difference_tdcp_factors.empty())) {
        return result;
    }

#ifdef GNSSPP_HAS_GTSAM
    if (config_.backend == FGOBackend::GTSAM) {
        return optimizeProblemWithGtsam(problem, config_, std::move(result));
    }
#endif

    // The native optimizer consumes every TDCP entry in the problem as one
    // residual row. Keep this separate from tdcp_factors (measurements built)
    // so a backend can never silently report generated factors as inserted.
    result.diagnostics.tdcp_factors_inserted = problem.tdcp_factors.size();
    result.diagnostics.single_difference_tdcp_factors_inserted =
        problem.single_difference_tdcp_factors.size();

    const int num_epochs = static_cast<int>(problem.epochs.size());
    std::vector<GNSSSystem> bias_groups;
    std::map<GNSSSystem, int> bias_group_columns;
    if (config_.use_inter_system_biases) {
        std::map<GNSSSystem, bool> present_bias_groups;
        for (const auto& factor : problem.pseudorange_factors) {
            if (usesSeparateClockBias(factor.clock_group)) {
                present_bias_groups[factor.clock_group] = true;
            }
        }
        for (const auto& factor : problem.carrier_phase_factors) {
            if (usesSeparateClockBias(factor.clock_group)) {
                present_bias_groups[factor.clock_group] = true;
            }
        }
        for (const auto& [group, present] : present_bias_groups) {
            if (!present) {
                continue;
            }
            bias_group_columns[group] = static_cast<int>(bias_groups.size());
            bias_groups.push_back(group);
        }
    }
    const int epoch_state_size = 4;
    const int position_clock_state_size = epoch_state_size * num_epochs;
    const bool use_velocity_states = config_.use_velocity_states;
    const int velocity_state_size = use_velocity_states ? 3 * num_epochs : 0;
    const int velocity_state_offset = position_clock_state_size;
    const int bias_state_offset = velocity_state_offset + velocity_state_size;
    const int base_state_size =
        bias_state_offset + static_cast<int>(bias_groups.size());
    const int ambiguity_count = static_cast<int>(problem.ambiguity_states.size());
    const int state_size = base_state_size + ambiguity_count;
    constexpr int kSparseNormalStateThreshold = 300;
    // cuSOLVER consumes a dense SPD normal matrix. In auto/on mode retain the
    // dense assembly above the ordinary Eigen sparse threshold so large
    // fixed-lag windows actually reach the GPU backend. With CUDA disabled or
    // below its auto threshold, the historical sparse selection is unchanged.
    const bool use_cuda_dense_normal = cudaDenseSolverEnabled(state_size);
    const bool use_sparse_normal =
        state_size > kSparseNormalStateThreshold && !use_cuda_dense_normal;
    CudaDenseSolveStats cuda_solve_stats;
    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(state_size);
    for (int i = 0; i < num_epochs; ++i) {
        const int epoch_col = epoch_state_size * i;
        initial_state.segment<3>(epoch_col) = problem.epochs[i].position_ecef;
        initial_state(epoch_col + 3) = problem.epochs[i].receiver_clock_bias_m;
    }
    if (use_velocity_states) {
        for (int i = 0; i < num_epochs; ++i) {
            initial_state.segment<3>(velocity_state_offset + 3 * i).setZero();
        }
    }
    for (int i = 0; i < ambiguity_count; ++i) {
        initial_state(base_state_size + i) =
            problem.ambiguity_states[i].initial_ambiguity_m;
    }

    auto epoch_state_col = [&](std::size_t epoch_index) -> int {
        return epoch_state_size * static_cast<int>(epoch_index);
    };

    auto velocity_state_col = [&](std::size_t epoch_index) -> int {
        if (!use_velocity_states) {
            return -1;
        }
        return velocity_state_offset + 3 * static_cast<int>(epoch_index);
    };

    auto system_bias_col = [&](int /*epoch_col*/, GNSSSystem group) -> int {
        const auto bias_it = bias_group_columns.find(group);
        if (bias_it == bias_group_columns.end()) {
            return -1;
        }
        return bias_state_offset + bias_it->second;
    };

    auto motion_rows_per_pair = [&]() -> std::size_t {
        const bool use_velocity_motion =
            config_.use_velocity_motion_factors && use_velocity_states;
        if (!config_.use_motion_factors && !use_velocity_motion) {
            return 0;
        }
        std::size_t rows_per_pair = 0;
        if (config_.use_motion_factors &&
            config_.use_position_motion_factors) {
            rows_per_pair += 3;
        }
        if (config_.use_motion_factors && config_.use_clock_motion_factors) {
            rows_per_pair += 1;
        }
        if (use_velocity_motion) {
            rows_per_pair += 3;
        }
        return rows_per_pair;
    };

    auto motion_factor_count = [&]() -> std::size_t {
        const bool use_velocity_motion =
            config_.use_velocity_motion_factors && use_velocity_states;
        if ((!config_.use_motion_factors && !use_velocity_motion) ||
            num_epochs < 2) {
            return 0;
        }
        return static_cast<std::size_t>(num_epochs - 1) * motion_rows_per_pair();
    };

    auto ambiguity_prior_count = [&]() -> std::size_t {
        if (!config_.use_ambiguity_priors || config_.ambiguity_prior_sigma_m <= 0.0) {
            return 0;
        }
        return problem.ambiguity_states.size();
    };

    auto velocity_prior_count = [&]() -> std::size_t {
        if (!use_velocity_states || config_.velocity_prior_sigma_mps <= 0.0) {
            return 0;
        }
        return static_cast<std::size_t>(num_epochs) * 3U;
    };

    using SparseNormalFactorization =
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>;
    using DenseNormalFactorization = Eigen::LDLT<Eigen::MatrixXd>;
    struct OptimizationOutput {
        Eigen::VectorXd state;
        Eigen::MatrixXd normal_matrix;
        Eigen::SparseMatrix<double> sparse_normal_matrix;
        std::shared_ptr<SparseNormalFactorization> sparse_factorization;
        std::shared_ptr<DenseNormalFactorization> dense_factorization;
        double dense_damping = 0.0;
        int iterations = 0;
        bool converged = false;
        double final_cost = 0.0;
        double processing_ms = 0.0;
        double last_dx_norm = std::numeric_limits<double>::infinity();
        std::size_t robust_pseudorange_factors = 0;
        std::size_t robust_carrier_phase_factors = 0;
        std::size_t robust_double_difference_pseudorange_factors = 0;
        std::size_t robust_double_difference_carrier_factors = 0;
        std::size_t robust_tdcp_factors = 0;
        std::vector<FGOProcessor::CostTraceEntry> cost_trace_entries;
    };

    auto robust_scale = [&](double normalized_residual,
                            double threshold_sigma) -> double {
        if (!config_.use_robust_loss || threshold_sigma <= 0.0) {
            return 1.0;
        }
        const double abs_residual = std::abs(normalized_residual);
        if (!std::isfinite(abs_residual) || abs_residual <= threshold_sigma) {
            return 1.0;
        }
        return std::sqrt(threshold_sigma / abs_residual);
    };

    auto run_optimizer =
        [&](const Eigen::VectorXd& start_state,
            const std::vector<FixedAmbiguityConstraint>& fixed_constraints,
            const std::string& phase,
            int global_iteration_offset)
            -> OptimizationOutput {
        OptimizationOutput output;
        output.state = start_state;

        const auto start_time = std::chrono::high_resolution_clock::now();
        double previous_cost = std::numeric_limits<double>::infinity();
        for (int iter = 0; iter < config_.max_iterations; ++iter) {
            const int pr_rows = static_cast<int>(problem.pseudorange_factors.size());
            const int carrier_phase_rows =
                static_cast<int>(problem.carrier_phase_factors.size());
            const int double_difference_pseudorange_rows =
                static_cast<int>(problem.double_difference_pseudorange_factors.size());
            const int double_difference_carrier_rows =
                static_cast<int>(problem.double_difference_carrier_factors.size());
            const int ambiguity_between_rows =
                static_cast<int>(problem.ambiguity_between_factors.size());
            const int tdcp_rows = static_cast<int>(problem.tdcp_factors.size());
            const int single_difference_doppler_rows =
                static_cast<int>(problem.single_difference_doppler_factors.size());
            const int single_difference_tdcp_rows =
                static_cast<int>(problem.single_difference_tdcp_factors.size());
            const int motion_rows = static_cast<int>(motion_factor_count());
            const int ambiguity_prior_rows = static_cast<int>(ambiguity_prior_count());
            const int velocity_prior_rows = static_cast<int>(velocity_prior_count());
            const int fixed_ambiguity_rows = static_cast<int>(fixed_constraints.size());
            const int estimated_rows =
                pr_rows + carrier_phase_rows + double_difference_pseudorange_rows +
                double_difference_carrier_rows + ambiguity_between_rows + tdcp_rows +
                single_difference_doppler_rows + single_difference_tdcp_rows +
                motion_rows + ambiguity_prior_rows + velocity_prior_rows +
                fixed_ambiguity_rows;
            (void)estimated_rows;
            Eigen::MatrixXd normal_matrix;
            std::vector<Eigen::Triplet<double>> normal_triplets;
            if (use_sparse_normal) {
                normal_triplets.reserve(
                    static_cast<std::size_t>(std::max(1, estimated_rows)) * 25U);
            } else {
                normal_matrix = Eigen::MatrixXd::Zero(state_size, state_size);
            }
            Eigen::VectorXd normal_rhs = Eigen::VectorXd::Zero(state_size);
            double weighted_residual_square_sum = 0.0;
            std::size_t robust_pseudorange_count = 0;
            std::size_t robust_carrier_phase_count = 0;
            std::size_t robust_double_difference_pseudorange_count = 0;
            std::size_t robust_double_difference_carrier_count = 0;
            std::size_t robust_tdcp_count = 0;

            int row = 0;
            auto add_weighted_row =
                [&](const std::vector<std::pair<int, double>>& jacobian,
                    double weighted_residual) {
                    if (jacobian.empty()) {
                        return;
                    }
                    weighted_residual_square_sum +=
                        weighted_residual * weighted_residual;
                    for (const auto& lhs : jacobian) {
                        if (lhs.first < 0 || lhs.first >= state_size) {
                            continue;
                        }
                        normal_rhs(lhs.first) += lhs.second * weighted_residual;
                        for (const auto& rhs : jacobian) {
                            if (rhs.first < 0 || rhs.first >= state_size) {
                                continue;
                            }
                            const double normal_entry = lhs.second * rhs.second;
                            if (use_sparse_normal) {
                                normal_triplets.emplace_back(
                                    lhs.first, rhs.first, normal_entry);
                            } else {
                                normal_matrix(lhs.first, rhs.first) +=
                                    normal_entry;
                            }
                        }
                    }
                    ++row;
                };
            if (config_.position_prior_sigma_m > 0.0) {
                const double position_prior_weight =
                    1.0 / std::max(1e-3, config_.position_prior_sigma_m);
                for (int i = 0; i < num_epochs; ++i) {
                    const int epoch_col = epoch_state_col(static_cast<std::size_t>(i));
                    for (int axis = 0; axis < 3; ++axis) {
                        const double weighted_residual =
                            (problem.epochs[i].position_ecef(axis) -
                             output.state(epoch_col + axis)) *
                            position_prior_weight;
                        add_weighted_row(
                            {{epoch_col + axis, position_prior_weight}},
                            weighted_residual);
                    }
                }
            }

            if (config_.clock_prior_sigma_m > 0.0) {
                const double clock_prior_weight =
                    1.0 / std::max(1e-3, config_.clock_prior_sigma_m);
                for (int i = 0; i < num_epochs; ++i) {
                    const int epoch_col = epoch_state_col(static_cast<std::size_t>(i));
                    const double weighted_residual =
                        (problem.epochs[i].receiver_clock_bias_m -
                         output.state(epoch_col + 3)) *
                        clock_prior_weight;
                    add_weighted_row(
                        {{epoch_col + 3, clock_prior_weight}},
                        weighted_residual);
                }
                for (const auto& [group, offset] : bias_group_columns) {
                    (void)group;
                    const int bias_col = bias_state_offset + offset;
                    const double weighted_residual =
                        -output.state(bias_col) * clock_prior_weight;
                    add_weighted_row({{bias_col, clock_prior_weight}},
                                     weighted_residual);
                }
            }

            if (use_velocity_states && config_.velocity_prior_sigma_mps > 0.0) {
                const double velocity_prior_weight =
                    1.0 / std::max(1e-6, config_.velocity_prior_sigma_mps);
                for (int i = 0; i < num_epochs; ++i) {
                    const int velocity_col =
                        velocity_state_col(static_cast<std::size_t>(i));
                    for (int axis = 0; axis < 3; ++axis) {
                        const double weighted_residual =
                            -output.state(velocity_col + axis) *
                            velocity_prior_weight;
                        add_weighted_row(
                            {{velocity_col + axis, velocity_prior_weight}},
                            weighted_residual);
                    }
                }
            }

            for (const auto& factor : problem.pseudorange_factors) {
                if (factor.epoch_index >= problem.epochs.size()) {
                    continue;
                }

                const int epoch_col = epoch_state_col(factor.epoch_index);
                const Vector3d position = output.state.segment<3>(epoch_col);
                const double clock_bias_m = output.state(epoch_col + 3);
                const int bias_col = system_bias_col(epoch_col, factor.clock_group);
                const Vector3d delta = factor.satellite_position_ecef - position;
                const double range = delta.norm();
                if (range <= 0.0) {
                    continue;
                }

                const Vector3d los = delta / range;
                double predicted = range + clock_bias_m;
                if (bias_col >= 0) {
                    predicted += output.state(bias_col);
                }
                const double sigma = std::max(1e-3, factor.sigma_m);
                const double raw_residual =
                    factor.corrected_pseudorange_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.pseudorange_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_pseudorange_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(5);
                jacobian.emplace_back(epoch_col + 0, -los(0) * weight);
                jacobian.emplace_back(epoch_col + 1, -los(1) * weight);
                jacobian.emplace_back(epoch_col + 2, -los(2) * weight);
                jacobian.emplace_back(epoch_col + 3, 1.0 * weight);
                if (bias_col >= 0) {
                    jacobian.emplace_back(bias_col, 1.0 * weight);
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.carrier_phase_factors) {
                if (factor.epoch_index >= problem.epochs.size() ||
                    factor.ambiguity_index >= problem.ambiguity_states.size()) {
                    continue;
                }

                const int epoch_col = epoch_state_col(factor.epoch_index);
                const int ambiguity_col =
                    base_state_size + static_cast<int>(factor.ambiguity_index);
                const Vector3d position = output.state.segment<3>(epoch_col);
                const double clock_bias_m = output.state(epoch_col + 3);
                const int bias_col = system_bias_col(epoch_col, factor.clock_group);
                const double ambiguity_m = output.state(ambiguity_col);
                const Vector3d delta = factor.satellite_position_ecef - position;
                const double range = delta.norm();
                if (range <= 0.0) {
                    continue;
                }

                const Vector3d los = delta / range;
                double predicted = range + clock_bias_m + ambiguity_m;
                if (bias_col >= 0) {
                    predicted += output.state(bias_col);
                }
                const double sigma = std::max(1e-4, factor.sigma_m);
                const double raw_residual =
                    factor.corrected_carrier_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.carrier_phase_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_carrier_phase_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(6);
                jacobian.emplace_back(epoch_col + 0, -los(0) * weight);
                jacobian.emplace_back(epoch_col + 1, -los(1) * weight);
                jacobian.emplace_back(epoch_col + 2, -los(2) * weight);
                jacobian.emplace_back(epoch_col + 3, 1.0 * weight);
                if (bias_col >= 0) {
                    jacobian.emplace_back(bias_col, 1.0 * weight);
                }
                jacobian.emplace_back(ambiguity_col, 1.0 * weight);
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.double_difference_pseudorange_factors) {
                if (factor.epoch_index >= problem.epochs.size()) {
                    continue;
                }

                const int epoch_col = epoch_state_col(factor.epoch_index);
                const Vector3d position = output.state.segment<3>(epoch_col);
                const Vector3d seed_position =
                    problem.epochs[factor.epoch_index].position_ecef;
                const Vector3d linearization_position =
                    config_.linearize_double_difference_factors_at_seed
                        ? seed_position
                        : position;
                const DoubleDifferencePrediction dd_prediction =
                    doubleDifferencePredictionAt(
                        linearization_position,
                        factor.base_position_ecef,
                        factor.rover_satellite_position_ecef,
                        factor.rover_reference_position_ecef,
                        factor.base_satellite_position_ecef,
                        factor.base_reference_position_ecef);
                if (!dd_prediction.valid) {
                    continue;
                }

                double predicted = dd_prediction.geometry_m;
                if (config_.linearize_double_difference_factors_at_seed) {
                    predicted +=
                        dd_prediction.position_jacobian.dot(position - seed_position);
                }
                const double sigma = std::max(1e-3, factor.sigma_m);
                const double raw_residual =
                    factor.observed_dd_pseudorange_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.pseudorange_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_double_difference_pseudorange_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(3);
                jacobian.emplace_back(epoch_col + 0,
                                      dd_prediction.position_jacobian(0) * weight);
                jacobian.emplace_back(epoch_col + 1,
                                      dd_prediction.position_jacobian(1) * weight);
                jacobian.emplace_back(epoch_col + 2,
                                      dd_prediction.position_jacobian(2) * weight);
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.double_difference_carrier_factors) {
                if (factor.epoch_index >= problem.epochs.size() ||
                    factor.ambiguity_index >= problem.ambiguity_states.size() ||
                    (factor.use_ambiguity_difference &&
                     factor.reference_ambiguity_index >=
                         problem.ambiguity_states.size())) {
                    continue;
                }

                const int epoch_col = epoch_state_col(factor.epoch_index);
                const int ambiguity_col =
                    base_state_size + static_cast<int>(factor.ambiguity_index);
                const int reference_ambiguity_col =
                    factor.use_ambiguity_difference
                        ? base_state_size +
                              static_cast<int>(factor.reference_ambiguity_index)
                        : -1;
                const Vector3d position = output.state.segment<3>(epoch_col);
                const Vector3d seed_position =
                    problem.epochs[factor.epoch_index].position_ecef;
                const Vector3d linearization_position =
                    config_.linearize_double_difference_factors_at_seed
                        ? seed_position
                        : position;
                const DoubleDifferencePrediction dd_prediction =
                    doubleDifferencePredictionAt(
                        linearization_position,
                        factor.base_position_ecef,
                        factor.rover_satellite_position_ecef,
                        factor.rover_reference_position_ecef,
                        factor.base_satellite_position_ecef,
                        factor.base_reference_position_ecef);
                if (!dd_prediction.valid) {
                    continue;
                }

                const double ambiguity_m = output.state(ambiguity_col);
                double predicted = dd_prediction.geometry_m + ambiguity_m;
                if (config_.linearize_double_difference_factors_at_seed) {
                    predicted +=
                        dd_prediction.position_jacobian.dot(position - seed_position);
                }
                if (factor.use_ambiguity_difference) {
                    predicted -= output.state(reference_ambiguity_col);
                }
                const double sigma = std::max(1e-4, factor.sigma_m);
                const double raw_residual =
                    factor.observed_dd_carrier_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.carrier_phase_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_double_difference_carrier_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(5);
                jacobian.emplace_back(epoch_col + 0,
                                      dd_prediction.position_jacobian(0) * weight);
                jacobian.emplace_back(epoch_col + 1,
                                      dd_prediction.position_jacobian(1) * weight);
                jacobian.emplace_back(epoch_col + 2,
                                      dd_prediction.position_jacobian(2) * weight);
                jacobian.emplace_back(ambiguity_col, 1.0 * weight);
                if (factor.use_ambiguity_difference) {
                    jacobian.emplace_back(reference_ambiguity_col, -1.0 * weight);
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.ambiguity_between_factors) {
                if (factor.previous_ambiguity_index >=
                        problem.ambiguity_states.size() ||
                    factor.current_ambiguity_index >=
                        problem.ambiguity_states.size()) {
                    continue;
                }

                const int previous_col =
                    base_state_size +
                    static_cast<int>(factor.previous_ambiguity_index);
                const int current_col =
                    base_state_size +
                    static_cast<int>(factor.current_ambiguity_index);
                const double sigma = std::max(1e-9, factor.sigma_m);
                const double weight = 1.0 / sigma;
                const double predicted =
                    output.state(current_col) - output.state(previous_col);
                add_weighted_row({{previous_col, -weight},
                                  {current_col, weight}},
                                 -predicted * weight);
            }

            for (const auto& factor : problem.tdcp_factors) {
                if (factor.previous_epoch_index >= problem.epochs.size() ||
                    factor.current_epoch_index >= problem.epochs.size()) {
                    continue;
                }

                const int previous_col =
                    epoch_state_col(factor.previous_epoch_index);
                const int current_col = epoch_state_col(factor.current_epoch_index);
                const int previous_bias_col =
                    system_bias_col(previous_col,
                                    clockBiasGroup(factor.satellite.system));
                const int current_bias_col =
                    system_bias_col(current_col,
                                    clockBiasGroup(factor.satellite.system));
                const Vector3d previous_position = output.state.segment<3>(previous_col);
                const Vector3d current_position = output.state.segment<3>(current_col);
                const Vector3d previous_delta =
                    factor.previous_satellite_position_ecef - previous_position;
                const Vector3d current_delta =
                    factor.current_satellite_position_ecef - current_position;
                const double previous_range = previous_delta.norm();
                const double current_range = current_delta.norm();
                if (previous_range <= 0.0 || current_range <= 0.0) {
                    continue;
                }

                const Vector3d previous_los = previous_delta / previous_range;
                const Vector3d current_los = current_delta / current_range;
                double predicted =
                    current_range + output.state(current_col + 3) -
                    previous_range - output.state(previous_col + 3);
                if (previous_bias_col >= 0 && current_bias_col >= 0 &&
                    previous_bias_col != current_bias_col) {
                    predicted += output.state(current_bias_col) -
                                 output.state(previous_bias_col);
                }
                const double sigma = std::max(1e-4, factor.sigma_m);
                const double raw_residual = factor.delta_carrier_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.tdcp_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_tdcp_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(10);
                jacobian.emplace_back(previous_col + 0, previous_los(0) * weight);
                jacobian.emplace_back(previous_col + 1, previous_los(1) * weight);
                jacobian.emplace_back(previous_col + 2, previous_los(2) * weight);
                jacobian.emplace_back(previous_col + 3, -1.0 * weight);
                if (previous_bias_col >= 0 &&
                    previous_bias_col != current_bias_col) {
                    jacobian.emplace_back(previous_bias_col, -1.0 * weight);
                }
                jacobian.emplace_back(current_col + 0, -current_los(0) * weight);
                jacobian.emplace_back(current_col + 1, -current_los(1) * weight);
                jacobian.emplace_back(current_col + 2, -current_los(2) * weight);
                jacobian.emplace_back(current_col + 3, 1.0 * weight);
                if (current_bias_col >= 0 &&
                    previous_bias_col != current_bias_col) {
                    jacobian.emplace_back(current_bias_col, 1.0 * weight);
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.single_difference_doppler_factors) {
                if (factor.epoch_index >= problem.epochs.size()) {
                    continue;
                }

                int velocity_col = velocity_state_col(factor.epoch_index);
                Vector3d velocity = Vector3d::Zero();
                if (velocity_col >= 0) {
                    velocity = output.state.segment<3>(velocity_col);
                } else {
                    if (factor.epoch_index == 0) {
                        continue;
                    }
                    const std::size_t previous_epoch_index =
                        factor.epoch_index - 1;
                    const double dt =
                        problem.epochs[factor.epoch_index].time -
                        problem.epochs[previous_epoch_index].time;
                    if (dt <= 0.0 ||
                        (config_.max_tdcp_gap_s > 0.0 &&
                         dt > config_.max_tdcp_gap_s)) {
                        continue;
                    }

                    const int previous_col = epoch_state_col(previous_epoch_index);
                    const int current_col = epoch_state_col(factor.epoch_index);
                    velocity =
                        (output.state.segment<3>(current_col) -
                         output.state.segment<3>(previous_col)) /
                        dt;
                    velocity_col = current_col;
                }
                const double predicted = factor.los.dot(velocity);
                const double sigma = std::max(1e-4, factor.sigma_mps);
                const double raw_residual = factor.residual_mps - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.tdcp_huber_threshold_sigma);
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                if (use_velocity_states) {
                    jacobian.reserve(3);
                    for (int axis = 0; axis < 3; ++axis) {
                        jacobian.emplace_back(velocity_col + axis,
                                              factor.los(axis) * weight);
                    }
                } else {
                    if (factor.epoch_index == 0) {
                        continue;
                    }
                    const std::size_t previous_epoch_index =
                        factor.epoch_index - 1;
                    const double dt =
                        problem.epochs[factor.epoch_index].time -
                        problem.epochs[previous_epoch_index].time;
                    const int previous_col = epoch_state_col(previous_epoch_index);
                    const int current_col = epoch_state_col(factor.epoch_index);
                    jacobian.reserve(6);
                    for (int axis = 0; axis < 3; ++axis) {
                        jacobian.emplace_back(previous_col + axis,
                                              -factor.los(axis) / dt * weight);
                        jacobian.emplace_back(current_col + axis,
                                              factor.los(axis) / dt * weight);
                    }
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.single_difference_tdcp_factors) {
                if (factor.previous_epoch_index >= problem.epochs.size() ||
                    factor.current_epoch_index >= problem.epochs.size()) {
                    continue;
                }

                const int previous_col =
                    epoch_state_col(factor.previous_epoch_index);
                const int current_col = epoch_state_col(factor.current_epoch_index);
                const Vector3d previous_delta =
                    output.state.segment<3>(previous_col) -
                    problem.epochs[factor.previous_epoch_index].position_ecef;
                const Vector3d current_delta =
                    output.state.segment<3>(current_col) -
                    problem.epochs[factor.current_epoch_index].position_ecef;
                const double predicted =
                    factor.los.dot(current_delta) -
                    factor.previous_los.dot(previous_delta);
                const double sigma = std::max(1e-4, factor.sigma_m);
                const double raw_residual = factor.delta_carrier_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.tdcp_huber_threshold_sigma);
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(6);
                for (int axis = 0; axis < 3; ++axis) {
                    jacobian.emplace_back(previous_col + axis,
                                          -factor.previous_los(axis) * weight);
                    jacobian.emplace_back(current_col + axis,
                                          factor.los(axis) * weight);
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            const bool use_velocity_motion =
                config_.use_velocity_motion_factors && use_velocity_states;
            if ((config_.use_motion_factors || use_velocity_motion) &&
                num_epochs >= 2) {
                const double motion_sigma = std::max(1e-3, config_.motion_sigma_m);
                const double clock_motion_sigma =
                    std::max(1e-3, config_.clock_motion_sigma_m);
                const double velocity_motion_sigma =
                    std::max(1e-6, config_.velocity_motion_sigma_m);
                for (int i = 1; i < num_epochs; ++i) {
                    const int prev_col =
                        epoch_state_col(static_cast<std::size_t>(i - 1));
                    const int curr_col =
                        epoch_state_col(static_cast<std::size_t>(i));
                    const double dt = std::max(
                        1e-3,
                        std::abs(problem.epochs[i].time - problem.epochs[i - 1].time));
                    const double pos_weight = 1.0 / (motion_sigma * dt);
                    const bool clock_jump =
                        static_cast<std::size_t>(i) < problem.clock_jumps.size() &&
                        problem.clock_jumps[static_cast<std::size_t>(i)];
                    const double epoch_clock_motion_sigma =
                        clock_jump ? 1e6 : clock_motion_sigma;
                    const double clock_weight =
                        1.0 / (epoch_clock_motion_sigma * dt);

                    if (config_.use_motion_factors &&
                        config_.use_position_motion_factors) {
                        for (int axis = 0; axis < 3; ++axis) {
                            const double weighted_residual =
                                -(output.state(curr_col + axis) -
                                  output.state(prev_col + axis)) *
                                pos_weight;
                            add_weighted_row(
                                {{prev_col + axis, -pos_weight},
                                 {curr_col + axis, pos_weight}},
                                weighted_residual);
                        }
                    }

                    if (config_.use_motion_factors &&
                        config_.use_clock_motion_factors) {
                        const double weighted_clock_residual =
                            -(output.state(curr_col + 3) -
                              output.state(prev_col + 3)) *
                            clock_weight;
                        add_weighted_row(
                            {{prev_col + 3, -clock_weight},
                             {curr_col + 3, clock_weight}},
                            weighted_clock_residual);

                        for (int bias_offset = 4; bias_offset < epoch_state_size;
                             ++bias_offset) {
                            const double weighted_bias_residual =
                                -(output.state(curr_col + bias_offset) -
                                  output.state(prev_col + bias_offset)) *
                                clock_weight;
                            add_weighted_row(
                                {{prev_col + bias_offset, -clock_weight},
                                 {curr_col + bias_offset, clock_weight}},
                                weighted_bias_residual);
                        }
                    }

                    if (use_velocity_motion) {
                        const int prev_velocity_col =
                            velocity_state_col(static_cast<std::size_t>(i - 1));
                        const int curr_velocity_col =
                            velocity_state_col(static_cast<std::size_t>(i));
                        const double weight = 1.0 / velocity_motion_sigma;
                        for (int axis = 0; axis < 3; ++axis) {
                            const double predicted =
                                output.state(curr_col + axis) -
                                output.state(prev_col + axis) -
                                0.5 * dt *
                                    (output.state(prev_velocity_col + axis) +
                                     output.state(curr_velocity_col + axis));
                            add_weighted_row(
                                {{prev_col + axis, -weight},
                                 {curr_col + axis, weight},
                                 {prev_velocity_col + axis, -0.5 * dt * weight},
                                 {curr_velocity_col + axis, -0.5 * dt * weight}},
                                -predicted * weight);
                        }
                    }
                }
            }

            if (config_.use_ambiguity_priors && config_.ambiguity_prior_sigma_m > 0.0) {
                const double ambiguity_prior_sigma =
                    std::max(1e-3, config_.ambiguity_prior_sigma_m);
                const double ambiguity_prior_weight = 1.0 / ambiguity_prior_sigma;
                for (int i = 0; i < ambiguity_count; ++i) {
                    const int ambiguity_col = base_state_size + i;
                    const double weighted_residual =
                        (problem.ambiguity_states[i].initial_ambiguity_m -
                         output.state(ambiguity_col)) *
                        ambiguity_prior_weight;
                    add_weighted_row(
                        {{ambiguity_col, ambiguity_prior_weight}},
                        weighted_residual);
                }
            }

            if (!fixed_constraints.empty()) {
                const double fixed_sigma =
                    std::max(1e-5, config_.fixed_ambiguity_sigma_m);
                const double fixed_weight = 1.0 / fixed_sigma;
                for (const auto& fixed : fixed_constraints) {
                    if (fixed.ambiguity_index >= problem.ambiguity_states.size()) {
                        continue;
                    }
                    const int ambiguity_col =
                        base_state_size + static_cast<int>(fixed.ambiguity_index);
                    const bool use_difference =
                        fixed.reference_ambiguity_index <
                        problem.ambiguity_states.size();
                    const int reference_col = use_difference
                        ? base_state_size + static_cast<int>(
                              fixed.reference_ambiguity_index)
                        : -1;
                    const double current_ambiguity =
                        output.state(ambiguity_col) -
                        (use_difference ? output.state(reference_col) : 0.0);
                    const double weighted_residual =
                        (fixed.fixed_ambiguity_m - current_ambiguity) *
                        fixed_weight;
                    std::vector<std::pair<int, double>> jacobian = {
                        {ambiguity_col, fixed_weight}};
                    if (use_difference) {
                        jacobian.emplace_back(reference_col, -fixed_weight);
                    }
                    add_weighted_row(jacobian, weighted_residual);
                }
            }

            if (row == 0) {
                return output;
            }

            auto cost_delta = [&]() -> std::pair<double, double> {
                if (!std::isfinite(previous_cost) ||
                    !std::isfinite(weighted_residual_square_sum)) {
                    return {
                        std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN(),
                    };
                }
                const double absolute_decrease =
                    previous_cost - weighted_residual_square_sum;
                const double relative_decrease =
                    previous_cost > 0.0
                        ? absolute_decrease / previous_cost
                        : absolute_decrease;
                return {absolute_decrease, relative_decrease};
            };
            auto record_cost_trace = [&](double update_norm, bool converged) {
                const auto [absolute_decrease, relative_decrease] = cost_delta();
                FGOProcessor::CostTraceEntry entry;
                entry.phase = phase;
                entry.local_iteration = iter;
                entry.global_iteration = global_iteration_offset + iter;
                entry.cost = weighted_residual_square_sum;
                entry.absolute_decrease = absolute_decrease;
                entry.relative_decrease = relative_decrease;
                entry.update_norm = update_norm;
                entry.converged = converged;
                output.cost_trace_entries.push_back(std::move(entry));
            };

            Eigen::VectorXd dx;
            bool solved = false;
            Eigen::SparseMatrix<double> current_sparse_normal;
            auto store_current_linearization = [&]() {
                output.final_cost = weighted_residual_square_sum;
                if (!use_sparse_normal) {
                    output.normal_matrix = normal_matrix;
                }
                output.robust_pseudorange_factors = robust_pseudorange_count;
                output.robust_carrier_phase_factors = robust_carrier_phase_count;
                output.robust_double_difference_pseudorange_factors =
                    robust_double_difference_pseudorange_count;
                output.robust_double_difference_carrier_factors =
                    robust_double_difference_carrier_count;
                output.robust_tdcp_factors = robust_tdcp_count;
            };
            auto cost_converged = [&]() {
                if (iter == 0 || !std::isfinite(previous_cost) ||
                    !std::isfinite(weighted_residual_square_sum)) {
                    return false;
                }
                const double absolute_decrease =
                    previous_cost - weighted_residual_square_sum;
                if (absolute_decrease < 0.0) {
                    return false;
                }
                const double relative_decrease =
                    previous_cost > 0.0
                        ? absolute_decrease / previous_cost
                        : absolute_decrease;
                return (config_.absolute_cost_convergence_threshold > 0.0 &&
                        absolute_decrease <
                            config_.absolute_cost_convergence_threshold) ||
                       (config_.relative_cost_convergence_threshold > 0.0 &&
                        relative_decrease <
                            config_.relative_cost_convergence_threshold);
            };
            if (use_sparse_normal) {
                output.sparse_factorization.reset();
                Eigen::SparseMatrix<double> sparse_normal(state_size, state_size);
                sparse_normal.setFromTriplets(
                    normal_triplets.begin(), normal_triplets.end());
                sparse_normal.makeCompressed();
                if (cost_converged()) {
                    store_current_linearization();
                    if (config_.collect_lambda_debug ||
                        config_.use_epoch_lambda_fixed_output ||
                        (config_.fix_ambiguities &&
                         config_.use_lambda_ambiguity_fix)) {
                        output.sparse_normal_matrix = std::move(sparse_normal);
                    }
                    output.iterations = iter;
                    output.converged = true;
                    record_cost_trace(
                        std::numeric_limits<double>::quiet_NaN(), true);
                    break;
                }

                double max_diagonal = 0.0;
                for (int i = 0; i < state_size; ++i) {
                    max_diagonal =
                        std::max(max_diagonal, std::abs(sparse_normal.coeff(i, i)));
                }
                double damping = std::max(1e-12, max_diagonal * 1e-12);
                for (int attempt = 0; attempt < 6; ++attempt) {
                    auto ldlt =
                        std::make_shared<SparseNormalFactorization>();
                    ldlt->setShift(damping);
                    ldlt->compute(sparse_normal);
                    if (ldlt->info() == Eigen::Success) {
                        dx = ldlt->solve(normal_rhs);
                        if (ldlt->info() == Eigen::Success && dx.allFinite()) {
                            output.sparse_factorization = std::move(ldlt);
                            solved = true;
                            break;
                        }
                    }
                    damping *= 10.0;
                }
                if (config_.collect_lambda_debug ||
                    config_.use_epoch_lambda_fixed_output ||
                    (config_.fix_ambiguities &&
                     config_.use_lambda_ambiguity_fix)) {
                    current_sparse_normal = std::move(sparse_normal);
                }
            } else {
                output.dense_factorization.reset();
                output.dense_damping = 0.0;
                if (cost_converged()) {
                    store_current_linearization();
                    output.iterations = iter;
                    output.converged = true;
                    record_cost_trace(
                        std::numeric_limits<double>::quiet_NaN(), true);
                    break;
                }
                const double max_diagonal =
                    normal_matrix.diagonal().cwiseAbs().maxCoeff();
                double damping = std::max(1e-12, max_diagonal * 1e-12);
                for (int attempt = 0; attempt < 6; ++attempt) {
                    Eigen::MatrixXd damped_normal = normal_matrix;
                    damped_normal.diagonal().array() += damping;
                    if (cudaDenseSolverEnabled(state_size)) {
                        Eigen::MatrixXd cuda_rhs(normal_rhs.rows(), 1);
                        cuda_rhs.col(0) = normal_rhs;
                        Eigen::MatrixXd cuda_solution;
                        if (tryCudaDenseSolve(damped_normal, cuda_rhs,
                                              cuda_solution,
                                              &cuda_solve_stats) &&
                            cuda_solution.cols() == 1 &&
                            cuda_solution.allFinite()) {
                            dx = cuda_solution.col(0);
                            output.dense_damping = damping;
                            solved = true;
                            break;
                        }
                    }
                    auto ldlt =
                        std::make_shared<DenseNormalFactorization>(
                            damped_normal);
                    if (ldlt->info() == Eigen::Success) {
                        dx = ldlt->solve(normal_rhs);
                        if (dx.allFinite()) {
                            output.dense_factorization = std::move(ldlt);
                            output.dense_damping = damping;
                            solved = true;
                            break;
                        }
                    }
                    damping *= 10.0;
                }
            }

            if (!solved) {
                break;
            }

            output.state += dx;
            output.last_dx_norm = dx.norm();
            output.iterations = iter + 1;
            store_current_linearization();
            if (use_sparse_normal &&
                (config_.collect_lambda_debug ||
                 config_.use_epoch_lambda_fixed_output ||
                 (config_.fix_ambiguities &&
                  config_.use_lambda_ambiguity_fix))) {
                output.sparse_normal_matrix = std::move(current_sparse_normal);
            }
            const bool update_converged =
                output.last_dx_norm < config_.convergence_threshold_m;
            record_cost_trace(output.last_dx_norm, update_converged);
            previous_cost = weighted_residual_square_sum;

            if (update_converged) {
                output.converged = true;
                break;
            }
        }

        const auto end_time = std::chrono::high_resolution_clock::now();
        output.processing_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                end_time - start_time)
                .count();
        return output;
    };

    OptimizationOutput optimization = run_optimizer(initial_state, {}, "float", 0);
    result.cost_trace_entries = optimization.cost_trace_entries;
    int total_iterations = optimization.iterations;
    double total_processing_ms = optimization.processing_ms;
    std::vector<FixedAmbiguityConstraint> fixed_constraints;
    std::vector<std::vector<FixedAmbiguityConstraint>>
        lambda_hypothesis_constraints;
    std::vector<double> lambda_hypothesis_residual_square_sums;
    const bool has_ambiguity_measurements =
        !problem.carrier_phase_factors.empty() ||
        !problem.double_difference_carrier_factors.empty();
    if (config_.fix_ambiguities &&
        ambiguity_count > 0 &&
        has_ambiguity_measurements) {
        const double max_fractional =
            std::max(0.0, config_.ambiguity_fix_max_fractional_cycles);
        double fixed_residual_square_sum = 0.0;
        const bool has_double_difference_ambiguities =
            std::any_of(problem.ambiguity_states.begin(),
                        problem.ambiguity_states.end(),
                        [](const AmbiguityState& ambiguity) {
                            return ambiguity.is_double_difference;
                        });
        auto is_fix_candidate = [&](const AmbiguityState& ambiguity) -> bool {
            // A rover-base SD ambiguity contains a common receiver gauge.
            // It is not an integer-fix candidate by itself. MultiSD fixing
            // must first project the SD posterior to independent BSD/DD
            // combinations. Individual gauge-dependent states stay float;
            // the MultiSD path below constrains only projected BSD edges.
            if (config_.use_multisd_ambiguities &&
                !ambiguity.is_double_difference) {
                return false;
            }
            if (config_.prefer_double_difference_ambiguity_fixing &&
                has_double_difference_ambiguities) {
                return ambiguity.is_double_difference;
            }
            return true;
        };

        auto build_nearest_integer_constraints = [&]() {
            fixed_constraints.clear();
            fixed_residual_square_sum = 0.0;
            for (int i = 0; i < ambiguity_count; ++i) {
                const auto& ambiguity = problem.ambiguity_states[i];
                if (ambiguity.wavelength_m <= 0.0 || !is_fix_candidate(ambiguity)) {
                    continue;
                }

                const double ambiguity_m = optimization.state(base_state_size + i);
                const double ambiguity_cycles = ambiguity_m / ambiguity.wavelength_m;
                if (!std::isfinite(ambiguity_cycles)) {
                    continue;
                }

                ++result.diagnostics.ambiguity_fix_candidates;
                const double fixed_cycles_d = std::round(ambiguity_cycles);
                const double residual_cycles = ambiguity_cycles - fixed_cycles_d;
                if (std::abs(residual_cycles) > max_fractional) {
                    continue;
                }

                FixedAmbiguityConstraint fixed;
                fixed.ambiguity_index = static_cast<std::size_t>(i);
                fixed.fixed_cycles = static_cast<int>(fixed_cycles_d);
                fixed.fixed_ambiguity_m = fixed_cycles_d * ambiguity.wavelength_m;
                fixed.residual_cycles = residual_cycles;
                fixed_constraints.push_back(fixed);
                fixed_residual_square_sum += residual_cycles * residual_cycles;
            }
        };

        auto build_lambda_constraints = [&]() -> bool {
            if (!config_.use_lambda_ambiguity_fix) {
                return false;
            }

            Eigen::MatrixXd ambiguity_state_covariance =
                Eigen::MatrixXd::Zero(ambiguity_count, ambiguity_count);
            if (optimization.normal_matrix.rows() == state_size) {
                Eigen::MatrixXd ambiguity_rhs = Eigen::MatrixXd::Zero(
                    state_size, ambiguity_count);
                ambiguity_rhs.block(base_state_size, 0,
                                    ambiguity_count, ambiguity_count)
                    .setIdentity();
                Eigen::MatrixXd covariance_columns;
                bool covariance_solved = false;
                if (cudaDenseSolverEnabled(state_size)) {
                    Eigen::MatrixXd damped_normal =
                        optimization.normal_matrix;
                    damped_normal.diagonal().array() +=
                        std::max(1e-12, optimization.dense_damping);
                    covariance_solved = tryCudaDenseSolve(
                        damped_normal, ambiguity_rhs, covariance_columns,
                        &cuda_solve_stats);
                }
                if (!covariance_solved && optimization.dense_factorization) {
                    const Eigen::MatrixXd covariance_columns =
                        optimization.dense_factorization->solve(ambiguity_rhs);
                    if (optimization.dense_factorization->info() !=
                            Eigen::Success ||
                        !covariance_columns.allFinite()) {
                        return false;
                    }
                    ambiguity_state_covariance = covariance_columns.block(
                        base_state_size, 0,
                        ambiguity_count, ambiguity_count);
                    covariance_solved = true;
                } else if (covariance_solved) {
                    if (!covariance_columns.allFinite()) {
                        return false;
                    }
                    ambiguity_state_covariance = covariance_columns.block(
                        base_state_size, 0,
                        ambiguity_count, ambiguity_count);
                }
                if (!covariance_solved) {
                    const Eigen::MatrixXd float_covariance =
                        pseudoInverse(optimization.normal_matrix);
                    if (float_covariance.rows() != state_size) {
                        return false;
                    }
                    ambiguity_state_covariance = float_covariance.block(
                        base_state_size, base_state_size,
                        ambiguity_count, ambiguity_count);
                }
            } else if (optimization.sparse_normal_matrix.rows() == state_size) {
                auto solver = optimization.sparse_factorization;
                if (!solver) {
                    solver = std::make_shared<SparseNormalFactorization>();
                    double max_diagonal = 0.0;
                    for (int i = 0; i < state_size; ++i) {
                        max_diagonal = std::max(
                            max_diagonal,
                            std::abs(
                                optimization.sparse_normal_matrix.coeff(i, i)));
                    }
                    solver->setShift(
                        std::max(1e-12, max_diagonal * 1e-12));
                    solver->compute(optimization.sparse_normal_matrix);
                    if (solver->info() != Eigen::Success) {
                        return false;
                    }
                }
                // Factor once and solve every requested ambiguity covariance
                // column as a multi-RHS batch. The previous per-column solve
                // repeated sparse triangular-solve setup and dominated the
                // causal 10-epoch wall time. This is also the exact operation
                // that a CUDA sparse/dense backend can later offload.
                Eigen::MatrixXd ambiguity_rhs = Eigen::MatrixXd::Zero(
                    state_size, ambiguity_count);
                ambiguity_rhs.block(base_state_size, 0,
                                    ambiguity_count, ambiguity_count)
                    .setIdentity();
                const Eigen::MatrixXd covariance_columns =
                    solver->solve(ambiguity_rhs);
                if (solver->info() != Eigen::Success ||
                    !covariance_columns.allFinite()) {
                    return false;
                }
                ambiguity_state_covariance = covariance_columns.block(
                    base_state_size, 0,
                    ambiguity_count, ambiguity_count);
            } else {
                return false;
            }
            ambiguity_state_covariance =
                0.5 * (ambiguity_state_covariance +
                       ambiguity_state_covariance.transpose());

            struct LambdaCandidate {
                int ambiguity_index = 0;
                int reference_ambiguity_index = -1;
                double variance_cycles = 0.0;
                double fractional_cycles = 0.0;
            };

            std::vector<LambdaCandidate> candidates;
            candidates.reserve(problem.ambiguity_states.size());
            auto append_candidate = [&](int ambiguity_index,
                                        int reference_ambiguity_index) {
                const auto& ambiguity =
                    problem.ambiguity_states[ambiguity_index];
                if (ambiguity.wavelength_m <= 0.0) {
                    return;
                }
                const int col = base_state_size + ambiguity_index;
                double ambiguity_m = optimization.state(col);
                double variance_m2 = ambiguity_state_covariance(
                    ambiguity_index, ambiguity_index);
                if (reference_ambiguity_index >= 0) {
                    const auto& reference =
                        problem.ambiguity_states[reference_ambiguity_index];
                    if (reference.wavelength_m <= 0.0 ||
                        std::abs(reference.wavelength_m -
                                 ambiguity.wavelength_m) > 1e-9) {
                        return;
                    }
                    const int reference_col =
                        base_state_size + reference_ambiguity_index;
                    ambiguity_m -= optimization.state(reference_col);
                    variance_m2 += ambiguity_state_covariance(
                                       reference_ambiguity_index,
                                       reference_ambiguity_index) -
                                   2.0 * ambiguity_state_covariance(
                                             ambiguity_index,
                                             reference_ambiguity_index);
                } else if (!is_fix_candidate(ambiguity)) {
                    return;
                }
                const double variance_cycles =
                    variance_m2 /
                    (ambiguity.wavelength_m * ambiguity.wavelength_m);
                const double ambiguity_cycles =
                    ambiguity_m / ambiguity.wavelength_m;
                if (!std::isfinite(variance_cycles) ||
                    variance_cycles <= 0.0 ||
                    !std::isfinite(ambiguity_cycles)) {
                    return;
                }
                LambdaCandidate candidate;
                candidate.ambiguity_index = ambiguity_index;
                candidate.reference_ambiguity_index =
                    reference_ambiguity_index;
                candidate.variance_cycles = variance_cycles;
                candidate.fractional_cycles = std::abs(
                    ambiguity_cycles - std::round(ambiguity_cycles));
                candidates.push_back(candidate);
            };

            if (config_.use_multisd_ambiguities) {
                std::size_t latest_epoch = 0;
                for (const auto& factor :
                     problem.double_difference_carrier_factors) {
                    latest_epoch = std::max(latest_epoch,
                                            factor.epoch_index);
                }
                std::set<std::pair<std::size_t, std::size_t>> seen;
                for (const auto& factor :
                     problem.double_difference_carrier_factors) {
                    if (factor.epoch_index != latest_epoch ||
                        !factor.use_ambiguity_difference ||
                        factor.ambiguity_index >=
                            problem.ambiguity_states.size() ||
                        factor.reference_ambiguity_index >=
                            problem.ambiguity_states.size() ||
                        !seen.emplace(factor.ambiguity_index,
                                      factor.reference_ambiguity_index)
                             .second) {
                        continue;
                    }
                    append_candidate(
                        static_cast<int>(factor.ambiguity_index),
                        static_cast<int>(factor.reference_ambiguity_index));
                }
            } else {
                for (int i = 0; i < ambiguity_count; ++i) {
                    append_candidate(i, -1);
                }
            }

            std::stable_sort(candidates.begin(),
                             candidates.end(),
                             [](const LambdaCandidate& lhs,
                                const LambdaCandidate& rhs) {
                                 if (lhs.fractional_cycles == rhs.fractional_cycles) {
                                     return lhs.variance_cycles < rhs.variance_cycles;
                                 }
                                 return lhs.fractional_cycles < rhs.fractional_cycles;
                             });

            const int max_lambda_ambiguities =
                std::max(0, config_.max_lambda_ambiguities);
            if (max_lambda_ambiguities > 0 &&
                static_cast<int>(candidates.size()) > max_lambda_ambiguities) {
                candidates.resize(static_cast<std::size_t>(max_lambda_ambiguities));
            }

            result.diagnostics.lambda_ambiguity_candidates = candidates.size();
            result.diagnostics.ambiguity_fix_candidates = candidates.size();
            const int min_fixed_ambiguities = std::max(1, config_.min_fixed_ambiguities);
            if (static_cast<int>(candidates.size()) < min_fixed_ambiguities) {
                return false;
            }

            auto solve_candidate_subset = [&](std::size_t subset_size) -> bool {
                const int n = static_cast<int>(subset_size);
                Eigen::VectorXd float_ambiguities = Eigen::VectorXd::Zero(n);
                Eigen::MatrixXd ambiguity_covariance = Eigen::MatrixXd::Zero(n, n);
                for (int row = 0; row < n; ++row) {
                    const int ambiguity_row = candidates[row].ambiguity_index;
                    const auto& row_state = problem.ambiguity_states[ambiguity_row];
                    const int row_col = base_state_size + ambiguity_row;
                    float_ambiguities(row) =
                        optimization.state(row_col) / row_state.wavelength_m;
                    if (candidates[row].reference_ambiguity_index >= 0) {
                        const int reference_row_col =
                            base_state_size +
                            candidates[row].reference_ambiguity_index;
                        float_ambiguities(row) -=
                            optimization.state(reference_row_col) /
                            row_state.wavelength_m;
                    }
                    for (int col = 0; col < n; ++col) {
                        const int ambiguity_col = candidates[col].ambiguity_index;
                        const auto& col_state = problem.ambiguity_states[ambiguity_col];
                        double covariance_m2 = ambiguity_state_covariance(
                            ambiguity_row, ambiguity_col);
                        const int reference_row =
                            candidates[row].reference_ambiguity_index;
                        const int reference_col =
                            candidates[col].reference_ambiguity_index;
                        if (reference_row >= 0) {
                            covariance_m2 -= ambiguity_state_covariance(
                                reference_row, ambiguity_col);
                        }
                        if (reference_col >= 0) {
                            covariance_m2 -= ambiguity_state_covariance(
                                ambiguity_row, reference_col);
                        }
                        if (reference_row >= 0 && reference_col >= 0) {
                            covariance_m2 += ambiguity_state_covariance(
                                reference_row, reference_col);
                        }
                        ambiguity_covariance(row, col) = covariance_m2 /
                            (row_state.wavelength_m *
                             col_state.wavelength_m);
                    }
                }

                ambiguity_covariance =
                    0.5 * (ambiguity_covariance + ambiguity_covariance.transpose());
                for (int i = 0; i < n; ++i) {
                    const double diagonal = std::abs(ambiguity_covariance(i, i));
                    ambiguity_covariance(i, i) += std::max(1e-12, diagonal * 1e-9);
                }

                Eigen::VectorXd fixed_ambiguities;
                double lambda_ratio = 0.0;
                ++result.diagnostics.lambda_ambiguity_attempts;
                LambdaCandidateDiagnostics lambda_diagnostics;
                const int top_k =
                    std::clamp(config_.lambda_top_k_candidates, 2, 32);
                const bool lambda_solved = lambdaSearchTopK(
                    float_ambiguities, ambiguity_covariance, top_k,
                    lambda_diagnostics);
                if (lambda_solved &&
                    lambda_diagnostics.candidates.cols() >= 2 &&
                    lambda_diagnostics.squared_residuals.size() >= 2) {
                    fixed_ambiguities =
                        lambda_diagnostics.candidates.col(0);
                    const double best =
                        lambda_diagnostics.squared_residuals(0);
                    const double second =
                        lambda_diagnostics.squared_residuals(1);
                    lambda_ratio = best > 0.0
                        ? second / best
                        : std::numeric_limits<double>::infinity();
                    result.diagnostics.lambda_top_k_generated =
                        std::max(result.diagnostics.lambda_top_k_generated,
                                 static_cast<std::size_t>(
                                     lambda_diagnostics.candidates.cols()));
                    result.diagnostics.lambda_bootstrapped_success_rate =
                        std::max(
                            result.diagnostics
                                .lambda_bootstrapped_success_rate,
                            lambda_diagnostics
                                .bootstrapped_success_rate);
                }
                double adop_cycles =
                    std::numeric_limits<double>::quiet_NaN();
                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(
                    ambiguity_covariance);
                if (eigensolver.info() == Eigen::Success &&
                    (eigensolver.eigenvalues().array() > 0.0).all()) {
                    adop_cycles = std::exp(
                        eigensolver.eigenvalues().array().log().sum() /
                        (2.0 * static_cast<double>(n)));
                    result.diagnostics.lambda_adop_cycles = adop_cycles;
                }
                if (lambda_solved) {
                    result.diagnostics.lambda_ambiguity_fix_solved = true;
                }
                if (lambda_solved && std::isfinite(lambda_ratio)) {
                    result.diagnostics.lambda_ambiguity_ratio =
                        std::max(result.diagnostics.lambda_ambiguity_ratio,
                                 lambda_ratio);
                }
                if (!lambda_solved || fixed_ambiguities.size() != n ||
                    !std::isfinite(lambda_ratio) ||
                    (config_.lambda_ratio_threshold > 0.0 &&
                     lambda_ratio < config_.lambda_ratio_threshold) ||
                    (config_.lambda_min_bootstrapped_success_rate > 0.0 &&
                     lambda_diagnostics.bootstrapped_success_rate <
                         config_.lambda_min_bootstrapped_success_rate) ||
                    (config_.lambda_max_adop_cycles > 0.0 &&
                     (!std::isfinite(adop_cycles) ||
                      adop_cycles > config_.lambda_max_adop_cycles))) {
                    return false;
                }

                lambda_hypothesis_constraints.clear();
                lambda_hypothesis_residual_square_sums.clear();
                const int hypothesis_count =
                    lambda_diagnostics.candidates.cols();
                lambda_hypothesis_constraints.reserve(hypothesis_count);
                lambda_hypothesis_residual_square_sums.reserve(hypothesis_count);
                for (int hypothesis = 0;
                     hypothesis < hypothesis_count;
                     ++hypothesis) {
                    std::vector<FixedAmbiguityConstraint> constraints;
                    constraints.reserve(subset_size);
                    double residual_square_sum = 0.0;
                    for (int i = 0; i < n; ++i) {
                        const int ambiguity_index =
                            candidates[i].ambiguity_index;
                        const auto& ambiguity =
                            problem.ambiguity_states[ambiguity_index];
                        const double fixed_cycles_d = std::round(
                            lambda_diagnostics.candidates(i, hypothesis));
                        const double residual_cycles =
                            float_ambiguities(i) - fixed_cycles_d;

                        FixedAmbiguityConstraint fixed;
                        fixed.ambiguity_index =
                            static_cast<std::size_t>(ambiguity_index);
                        if (candidates[i].reference_ambiguity_index >= 0) {
                            fixed.reference_ambiguity_index =
                                static_cast<std::size_t>(
                                    candidates[i].reference_ambiguity_index);
                        }
                        fixed.fixed_cycles =
                            static_cast<int>(fixed_cycles_d);
                        fixed.fixed_ambiguity_m =
                            fixed_cycles_d * ambiguity.wavelength_m;
                        fixed.residual_cycles = residual_cycles;
                        fixed.fixed_by_lambda = true;
                        constraints.push_back(fixed);
                        residual_square_sum +=
                            residual_cycles * residual_cycles;
                    }
                    lambda_hypothesis_constraints.push_back(
                        std::move(constraints));
                    lambda_hypothesis_residual_square_sums.push_back(
                        residual_square_sum);
                }

                if (lambda_hypothesis_constraints.empty() ||
                    static_cast<int>(
                        lambda_hypothesis_constraints.front().size()) <
                        min_fixed_ambiguities) {
                    return false;
                }

                fixed_constraints = lambda_hypothesis_constraints.front();
                fixed_residual_square_sum =
                    lambda_hypothesis_residual_square_sums.front();
                result.diagnostics.lambda_ambiguity_fix_used = true;
                result.diagnostics.partial_lambda_ambiguity_fix_used =
                    subset_size < candidates.size();
                result.diagnostics.lambda_ambiguity_used_candidates = subset_size;
                result.diagnostics.lambda_ambiguity_ratio = lambda_ratio;
                return true;
            };

            const std::size_t min_subset_size =
                static_cast<std::size_t>(min_fixed_ambiguities);
            for (std::size_t subset_size = candidates.size();
                 subset_size >= min_subset_size;
                 --subset_size) {
                if (solve_candidate_subset(subset_size)) {
                    return true;
                }
                if (!config_.use_partial_lambda_ambiguity_fix ||
                    subset_size == min_subset_size) {
                    break;
                }
            }

            return false;
        };

        const bool can_attempt_lambda =
            config_.use_lambda_ambiguity_fix &&
            (optimization.normal_matrix.rows() == state_size ||
             optimization.sparse_normal_matrix.rows() == state_size);
        const bool lambda_constraints_built =
            can_attempt_lambda && build_lambda_constraints();
        if (!lambda_constraints_built &&
            (!config_.use_lambda_ambiguity_fix || !can_attempt_lambda)) {
            build_nearest_integer_constraints();
        }

        if (static_cast<int>(fixed_constraints.size()) >=
            std::max(1, config_.min_fixed_ambiguities)) {
            const bool run_disjoint_validation =
                config_.use_multisd_ambiguities &&
                config_.use_multisd_disjoint_validation;
            OptimizationOutput float_optimization;
            if (run_disjoint_validation) {
                float_optimization = optimization;
            }
            const int fixed_global_iteration_offset =
                result.cost_trace_entries.empty()
                    ? total_iterations
                    : result.cost_trace_entries.back().global_iteration + 1;
            OptimizationOutput fixed_optimization =
                run_optimizer(optimization.state,
                              fixed_constraints,
                              "fixed",
                              fixed_global_iteration_offset);
            total_iterations += fixed_optimization.iterations;
            total_processing_ms += fixed_optimization.processing_ms;
            result.cost_trace_entries.insert(
                result.cost_trace_entries.end(),
                fixed_optimization.cost_trace_entries.begin(),
                fixed_optimization.cost_trace_entries.end());
            optimization = std::move(fixed_optimization);
            result.diagnostics.fixed_solution = true;
            result.diagnostics.fixed_ambiguities = fixed_constraints.size();
            result.diagnostics.fixed_ambiguity_residual_rms_cycles =
                std::sqrt(fixed_residual_square_sum /
                          static_cast<double>(fixed_constraints.size()));

            if (run_disjoint_validation) {
                struct ValidationOutcome {
                    bool evaluated = false;
                    bool pass = false;
                    std::size_t carrier_used = 0;
                    std::size_t carrier_passed = 0;
                    std::size_t pseudorange_used = 0;
                    double maximum_integer_distance = 0.0;
                    double ddpr_rms =
                        std::numeric_limits<double>::infinity();
                };
                const std::size_t latest_epoch =
                    static_cast<std::size_t>(num_epochs - 1);
                const int required_holdout =
                    std::max(1, config_.multisd_validation_holdout_satellites);
                const double aperture =
                    std::max(0.0,
                             config_.multisd_validation_aperture_cycles);
                const bool enough_satellites =
                    static_cast<int>(result.diagnostics
                                         .multisd_validation_holdout_satellites) >=
                    required_holdout;
                auto validate_hypothesis =
                    [&](const OptimizationOutput& hypothesis)
                        -> ValidationOutcome {
                    ValidationOutcome outcome;
                    const Vector3d fixed_position =
                        hypothesis.state.segment<3>(
                            epoch_state_col(latest_epoch));
                    for (const auto& factor :
                         problem.multisd_validation_carrier_factors) {
                        if (factor.epoch_index != latest_epoch ||
                            factor.ambiguity_index >=
                                problem.ambiguity_states.size()) {
                            continue;
                        }
                        const double wavelength =
                            problem.ambiguity_states[factor.ambiguity_index]
                                .wavelength_m;
                        if (!(wavelength > 0.0)) {
                            continue;
                        }
                        const DoubleDifferencePrediction prediction =
                            doubleDifferencePredictionAt(
                                fixed_position, factor.base_position_ecef,
                                factor.rover_satellite_position_ecef,
                                factor.rover_reference_position_ecef,
                                factor.base_satellite_position_ecef,
                                factor.base_reference_position_ecef);
                        if (!prediction.valid) {
                            continue;
                        }
                        const double ambiguity_cycles =
                            (factor.observed_dd_carrier_m -
                             prediction.geometry_m) /
                            wavelength;
                        if (!std::isfinite(ambiguity_cycles)) {
                            continue;
                        }
                        const double integer_distance =
                            std::abs(ambiguity_cycles -
                                     std::round(ambiguity_cycles));
                        outcome.maximum_integer_distance = std::max(
                            outcome.maximum_integer_distance,
                            integer_distance);
                        if (integer_distance <= aperture) {
                            ++outcome.carrier_passed;
                        }
                        ++outcome.carrier_used;
                    }

                    double ddpr_square_sum = 0.0;
                    for (const auto& factor :
                         problem.multisd_validation_pseudorange_factors) {
                        if (factor.epoch_index != latest_epoch) {
                            continue;
                        }
                        const DoubleDifferencePrediction prediction =
                            doubleDifferencePredictionAt(
                                fixed_position, factor.base_position_ecef,
                                factor.rover_satellite_position_ecef,
                                factor.rover_reference_position_ecef,
                                factor.base_satellite_position_ecef,
                                factor.base_reference_position_ecef);
                        if (!prediction.valid) {
                            continue;
                        }
                        const double residual =
                            factor.observed_dd_pseudorange_m -
                            prediction.geometry_m;
                        if (!std::isfinite(residual)) {
                            continue;
                        }
                        ddpr_square_sum += residual * residual;
                        ++outcome.pseudorange_used;
                    }
                    if (outcome.pseudorange_used > 0) {
                        outcome.ddpr_rms = std::sqrt(
                            ddpr_square_sum /
                            static_cast<double>(outcome.pseudorange_used));
                    }
                    const bool enough_rows =
                        static_cast<int>(outcome.carrier_used) >=
                            required_holdout &&
                        static_cast<int>(outcome.pseudorange_used) >=
                            required_holdout;
                    outcome.evaluated = enough_satellites && enough_rows;
                    const std::size_t required_carrier_passes =
                        static_cast<std::size_t>(std::ceil(
                            std::clamp(
                                config_
                                    .multisd_validation_min_carrier_fraction,
                                0.0, 1.0) *
                            static_cast<double>(outcome.carrier_used)));
                    outcome.pass =
                        outcome.evaluated &&
                        outcome.carrier_passed >= required_carrier_passes &&
                        std::isfinite(outcome.ddpr_rms) &&
                        outcome.ddpr_rms <= std::max(
                            0.0,
                            config_.multisd_validation_max_ddpr_rms_m);
                    return outcome;
                };

                const std::size_t hypothesis_count =
                    lambda_hypothesis_constraints.empty()
                        ? 1
                        : lambda_hypothesis_constraints.size();
                std::size_t passing_hypotheses = 0;
                int selected_rank = -1;
                OptimizationOutput selected_optimization;
                ValidationOutcome selected_outcome;
                ValidationOutcome first_outcome;
                std::vector<std::future<OptimizationOutput>>
                    hypothesis_futures;
                // CUDA uses a retained thread-local cuSOLVER workspace. Keep
                // hypotheses on this thread so every rank reuses the same
                // allocations/handle instead of creating one CUDA context
                // workspace per std::async worker.
                if (config_.parallelize_lambda_hypotheses &&
                    !use_cuda_dense_normal &&
                    hypothesis_count > 1) {
                    hypothesis_futures.reserve(hypothesis_count - 1);
                    for (std::size_t rank = 1;
                         rank < hypothesis_count;
                         ++rank) {
                        hypothesis_futures.push_back(std::async(
                            std::launch::async,
                            run_optimizer,
                            float_optimization.state,
                            lambda_hypothesis_constraints[rank],
                            "fixed-hypothesis",
                            fixed_global_iteration_offset));
                    }
                }
                for (std::size_t rank = 0; rank < hypothesis_count; ++rank) {
                    OptimizationOutput hypothesis;
                    if (rank == 0) {
                        hypothesis = optimization;
                    } else if (!hypothesis_futures.empty()) {
                        hypothesis = hypothesis_futures[rank - 1].get();
                    } else {
                        hypothesis = run_optimizer(
                            float_optimization.state,
                            lambda_hypothesis_constraints[rank],
                            "fixed-hypothesis",
                            fixed_global_iteration_offset);
                        total_iterations += hypothesis.iterations;
                        total_processing_ms += hypothesis.processing_ms;
                    }
                    const ValidationOutcome outcome =
                        validate_hypothesis(hypothesis);
                    FGOResult::MultiSdValidationHypothesis hypothesis_record;
                    hypothesis_record.rank = static_cast<int>(rank);
                    hypothesis_record.evaluated = outcome.evaluated;
                    hypothesis_record.pass = outcome.pass;
                    hypothesis_record.latest_position_ecef =
                        hypothesis.state.segment<3>(
                            epoch_state_col(latest_epoch));
                    hypothesis_record.carrier_used = outcome.carrier_used;
                    hypothesis_record.carrier_passed = outcome.carrier_passed;
                    hypothesis_record.pseudorange_used =
                        outcome.pseudorange_used;
                    hypothesis_record.maximum_integer_distance_cycles =
                        outcome.maximum_integer_distance;
                    hypothesis_record.ddpr_rms_m = outcome.ddpr_rms;
                    result.multisd_validation_hypotheses.push_back(
                        hypothesis_record);
                    if (rank == 0) {
                        first_outcome = outcome;
                    }
                    if (outcome.pass) {
                        ++passing_hypotheses;
                        selected_rank = static_cast<int>(rank);
                        selected_optimization = std::move(hypothesis);
                        selected_outcome = outcome;
                    }
                }

                const bool unique_pass = passing_hypotheses == 1;
                const ValidationOutcome& reported_outcome =
                    unique_pass ? selected_outcome : first_outcome;
                result.diagnostics.multisd_validation_evaluated =
                    reported_outcome.evaluated;
                result.diagnostics.multisd_validation_pass = unique_pass;
                result.diagnostics.multisd_validation_carrier_used =
                    reported_outcome.carrier_used;
                result.diagnostics.multisd_validation_carrier_passed =
                    reported_outcome.carrier_passed;
                result.diagnostics.multisd_validation_pseudorange_used =
                    reported_outcome.pseudorange_used;
                result.diagnostics.multisd_validation_hypotheses_evaluated =
                    hypothesis_count;
                result.diagnostics.multisd_validation_hypotheses_passed =
                    passing_hypotheses;
                result.diagnostics.multisd_validation_selected_rank =
                    unique_pass ? selected_rank : -1;
                result.diagnostics
                    .multisd_validation_max_integer_distance_cycles =
                    reported_outcome.maximum_integer_distance;
                result.diagnostics.multisd_validation_ddpr_rms_m =
                    reported_outcome.ddpr_rms;

                if (unique_pass) {
                    optimization = std::move(selected_optimization);
                    if (!lambda_hypothesis_constraints.empty()) {
                        fixed_constraints =
                            lambda_hypothesis_constraints[
                                static_cast<std::size_t>(selected_rank)];
                        fixed_residual_square_sum =
                            lambda_hypothesis_residual_square_sums[
                                static_cast<std::size_t>(selected_rank)];
                        result.diagnostics.fixed_ambiguity_residual_rms_cycles =
                            std::sqrt(
                                fixed_residual_square_sum /
                                static_cast<double>(fixed_constraints.size()));
                    }
                } else {
                    optimization = std::move(float_optimization);
                    fixed_constraints.clear();
                    result.diagnostics.fixed_solution = false;
                    result.diagnostics.fixed_ambiguities = 0;
                    result.diagnostics.lambda_ambiguity_fix_used = false;
                    result.diagnostics.partial_lambda_ambiguity_fix_used = false;
                    result.diagnostics.fixed_ambiguity_residual_rms_cycles = 0.0;
                }
            }
        } else {
            fixed_constraints.clear();
        }
    }

    const Eigen::VectorXd& state = optimization.state;
    result.diagnostics.iterations = total_iterations;
    if (!result.cost_trace_entries.empty()) {
        result.diagnostics.initial_cost = result.cost_trace_entries.front().cost;
    }
    result.diagnostics.final_cost = optimization.final_cost;
    result.diagnostics.converged = optimization.converged;
    if (!result.diagnostics.converged &&
        std::isfinite(optimization.last_dx_norm) &&
        optimization.last_dx_norm < config_.convergence_threshold_m) {
        result.diagnostics.converged = true;
    }
    const double processing_ms = total_processing_ms;
    result.diagnostics.processing_time_ms = processing_ms;
    result.diagnostics.dense_normal_state_size =
        static_cast<std::size_t>(state_size);
    result.diagnostics.cuda_dense_solver_selected = use_cuda_dense_normal;
    result.diagnostics.cuda_dense_solve_attempts = cuda_solve_stats.attempts;
    result.diagnostics.cuda_dense_solve_successes = cuda_solve_stats.successes;
    result.diagnostics.cuda_dense_solve_fallbacks =
        cuda_solve_stats.attempts - cuda_solve_stats.successes;
    result.diagnostics.cuda_dense_solve_time_ms =
        cuda_solve_stats.processing_ms;
    result.diagnostics.last_update_norm_m =
        std::isfinite(optimization.last_dx_norm) ? optimization.last_dx_norm : 0.0;

    Eigen::MatrixXd covariance;
    if (optimization.normal_matrix.rows() == state_size) {
        bool covariance_solved = false;
        if (cudaDenseSolverEnabled(state_size)) {
            Eigen::MatrixXd damped_normal = optimization.normal_matrix;
            damped_normal.diagonal().array() +=
                std::max(1e-12, optimization.dense_damping);
            covariance_solved = tryCudaDenseSolve(
                damped_normal,
                Eigen::MatrixXd::Identity(state_size, state_size),
                covariance, &cuda_solve_stats);
            result.diagnostics.cuda_dense_solve_attempts =
                cuda_solve_stats.attempts;
            result.diagnostics.cuda_dense_solve_successes =
                cuda_solve_stats.successes;
            result.diagnostics.cuda_dense_solve_fallbacks =
                cuda_solve_stats.attempts - cuda_solve_stats.successes;
            result.diagnostics.cuda_dense_solve_time_ms =
                cuda_solve_stats.processing_ms;
        }
        if (!covariance_solved && optimization.dense_factorization) {
            covariance = optimization.dense_factorization->solve(
                Eigen::MatrixXd::Identity(state_size, state_size));
            covariance_solved = covariance.allFinite();
        }
        if (!covariance_solved) {
            covariance = pseudoInverse(optimization.normal_matrix);
        }
    }

    std::vector<Vector3d> epoch_output_positions(
        static_cast<std::size_t>(num_epochs),
        Vector3d::Zero());
    std::vector<bool> epoch_lambda_fixed_output(
        static_cast<std::size_t>(num_epochs),
        false);
    std::vector<double> epoch_lambda_ratios(
        static_cast<std::size_t>(num_epochs),
        0.0);
    std::vector<int> epoch_fixed_ambiguity_counts(
        static_cast<std::size_t>(num_epochs),
        0);
    for (int i = 0; i < num_epochs; ++i) {
        epoch_output_positions[static_cast<std::size_t>(i)] =
            state.segment<3>(epoch_state_col(static_cast<std::size_t>(i)));
    }
    std::size_t epoch_lambda_fixed_solution_count = 0;
    std::size_t epoch_lambda_fixed_ambiguity_total = 0;

    const bool compute_epoch_lambda =
        (config_.collect_lambda_debug ||
         config_.use_epoch_lambda_fixed_output) &&
        ambiguity_count > 0;
    const auto epoch_lambda_start =
        std::chrono::high_resolution_clock::now();
    if (compute_epoch_lambda) {
        const auto elapsed_ms = [](const auto& start, const auto& end) {
            return std::chrono::duration_cast<
                       std::chrono::duration<double, std::milli>>(end - start)
                .count();
        };
        const auto setup_start = std::chrono::high_resolution_clock::now();
        std::map<std::size_t, std::set<std::size_t>> ambiguity_indices_by_epoch;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.epoch_index < problem.epochs.size() &&
                factor.ambiguity_index < problem.ambiguity_states.size()) {
                ambiguity_indices_by_epoch[factor.epoch_index].insert(
                    factor.ambiguity_index);
            }
        }

        const int min_lambda_debug_candidates =
            std::max(6, std::max(1, config_.min_fixed_ambiguities + 1));
        struct EpochLambdaWork {
            std::size_t epoch_index = 0;
            std::vector<std::size_t> candidate_indices;
            std::vector<int> state_columns;
            int epoch_col = 0;
            int candidate_count = 0;
        };
        std::vector<EpochLambdaWork> epoch_lambda_work;
        epoch_lambda_work.reserve(ambiguity_indices_by_epoch.size());
        for (const auto& [epoch_index, ambiguity_set] :
             ambiguity_indices_by_epoch) {
            if (static_cast<int>(ambiguity_set.size()) <
                min_lambda_debug_candidates) {
                continue;
            }

            EpochLambdaWork work;
            work.epoch_index = epoch_index;
            work.candidate_indices.assign(ambiguity_set.begin(),
                                          ambiguity_set.end());
            std::sort(
                work.candidate_indices.begin(),
                work.candidate_indices.end(),
                [&](std::size_t lhs, std::size_t rhs) {
                    const auto& lhs_ambiguity = problem.ambiguity_states[lhs];
                    const auto& rhs_ambiguity = problem.ambiguity_states[rhs];
                    return std::tie(lhs_ambiguity.satellite,
                                    lhs_ambiguity.reference_satellite,
                                    lhs_ambiguity.signal,
                                    lhs) <
                           std::tie(rhs_ambiguity.satellite,
                                    rhs_ambiguity.reference_satellite,
                                    rhs_ambiguity.signal,
                                    rhs);
                });

            work.candidate_count =
                static_cast<int>(work.candidate_indices.size());
            work.state_columns.reserve(
                static_cast<std::size_t>(work.candidate_count));
            for (const std::size_t ambiguity_index : work.candidate_indices) {
                work.state_columns.push_back(
                    base_state_size + static_cast<int>(ambiguity_index));
            }
            work.epoch_col = epoch_state_col(epoch_index);
            epoch_lambda_work.push_back(std::move(work));
        }

        if (config_.collect_lambda_debug) {
            std::size_t lambda_debug_capacity = 0;
            for (const auto& work : epoch_lambda_work) {
                lambda_debug_capacity +=
                    static_cast<std::size_t>(work.candidate_count) *
                    static_cast<std::size_t>(work.candidate_count);
            }
            result.lambda_debug_entries.reserve(lambda_debug_capacity);
        }
        result.diagnostics.epoch_lambda_setup_time_ms += elapsed_ms(
            setup_start, std::chrono::high_resolution_clock::now());

#ifdef GNSSPP_HAS_CHOLMOD
        Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>>
            sparse_covariance_ldlt;
#else
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> sparse_covariance_ldlt;
#endif
        bool has_sparse_covariance_solver = false;
        if (!epoch_lambda_work.empty() &&
            covariance.rows() != state_size &&
            optimization.sparse_normal_matrix.rows() == state_size) {
            const auto factorization_start =
                std::chrono::high_resolution_clock::now();
            double max_diagonal = 0.0;
            for (int i = 0; i < state_size; ++i) {
                max_diagonal =
                    std::max(max_diagonal,
                             std::abs(optimization.sparse_normal_matrix.coeff(i, i)));
            }
            const double damping = std::max(1e-12, max_diagonal * 1e-12);
            sparse_covariance_ldlt.setShift(damping);
            sparse_covariance_ldlt.compute(optimization.sparse_normal_matrix);
            has_sparse_covariance_solver =
                sparse_covariance_ldlt.info() == Eigen::Success;
            result.diagnostics.epoch_lambda_factorization_time_ms +=
                elapsed_ms(factorization_start,
                           std::chrono::high_resolution_clock::now());
        }

        auto process_epoch_lambda_work =
            [&](const EpochLambdaWork& work, const auto& covarianceValue) {
                const std::size_t epoch_index = work.epoch_index;
                const auto& candidate_indices = work.candidate_indices;
                const int candidate_count = work.candidate_count;
                const int epoch_col = work.epoch_col;
                Eigen::VectorXd ambiguity_float =
                    Eigen::VectorXd::Zero(candidate_count);
                Eigen::MatrixXd ambiguity_covariance =
                    Eigen::MatrixXd::Zero(candidate_count, candidate_count);
                bool valid_covariance = true;
                for (int row = 0; row < candidate_count; ++row) {
                    const auto& row_ambiguity =
                        problem.ambiguity_states[candidate_indices[row]];
                    if (row_ambiguity.wavelength_m <= 0.0) {
                        valid_covariance = false;
                        break;
                    }
                    const int row_col =
                        work.state_columns[static_cast<std::size_t>(row)];
                    ambiguity_float(row) =
                        state(row_col) / row_ambiguity.wavelength_m;
                    for (int col = 0; col < candidate_count; ++col) {
                        const auto& col_ambiguity =
                            problem.ambiguity_states[candidate_indices[col]];
                        if (col_ambiguity.wavelength_m <= 0.0) {
                            valid_covariance = false;
                            break;
                        }
                        ambiguity_covariance(row, col) =
                            covarianceValue(row_col, col) /
                            (row_ambiguity.wavelength_m *
                             col_ambiguity.wavelength_m);
                    }
                    if (!valid_covariance) {
                        break;
                    }
                }
                if (!valid_covariance || !ambiguity_float.allFinite() ||
                    !ambiguity_covariance.allFinite()) {
                    return;
                }

                ambiguity_covariance =
                    0.5 * (ambiguity_covariance +
                           ambiguity_covariance.transpose());
                for (int i = 0; i < candidate_count; ++i) {
                    const double diagonal = std::abs(ambiguity_covariance(i, i));
                    ambiguity_covariance(i, i) +=
                        std::max(1e-12, diagonal * 1e-9);
                }

                Eigen::VectorXd fixed_ambiguities;
                double lambda_ratio = 0.0;
                const auto lambda_search_start =
                    std::chrono::high_resolution_clock::now();
                const bool lambda_solved =
                    lambdaSearch(ambiguity_float,
                                 ambiguity_covariance,
                                 fixed_ambiguities,
                                 lambda_ratio);
                result.diagnostics.epoch_lambda_search_time_ms += elapsed_ms(
                    lambda_search_start,
                    std::chrono::high_resolution_clock::now());
                const bool fixed_epoch =
                    lambda_solved && std::isfinite(lambda_ratio) &&
                    (config_.lambda_ratio_threshold <= 0.0 ||
                     lambda_ratio > config_.lambda_ratio_threshold);

                result.diagnostics.lambda_ambiguity_candidates +=
                    static_cast<std::size_t>(candidate_count);
                ++result.diagnostics.lambda_ambiguity_attempts;

                if (lambda_solved) {
                    result.diagnostics.lambda_ambiguity_fix_solved = true;
                    result.diagnostics.lambda_ambiguity_ratio =
                        std::max(result.diagnostics.lambda_ambiguity_ratio,
                                 lambda_ratio);
                    epoch_lambda_ratios[epoch_index] = lambda_ratio;
                }

                if (config_.use_epoch_lambda_fixed_output && fixed_epoch &&
                    fixed_ambiguities.size() == candidate_count) {
                    const auto fixed_output_start =
                        std::chrono::high_resolution_clock::now();
                    Eigen::MatrixXd position_ambiguity_covariance =
                        Eigen::MatrixXd::Zero(3, candidate_count);
                    for (int row = 0; row < candidate_count; ++row) {
                        const auto& row_ambiguity =
                            problem.ambiguity_states[candidate_indices[row]];
                        if (row_ambiguity.wavelength_m <= 0.0) {
                            valid_covariance = false;
                            break;
                        }
                        position_ambiguity_covariance(0, row) =
                            covarianceValue(epoch_col, row) /
                            row_ambiguity.wavelength_m;
                        position_ambiguity_covariance(1, row) =
                            covarianceValue(epoch_col + 1, row) /
                            row_ambiguity.wavelength_m;
                        position_ambiguity_covariance(2, row) =
                            covarianceValue(epoch_col + 2, row) /
                            row_ambiguity.wavelength_m;
                    }
                    if (valid_covariance &&
                        position_ambiguity_covariance.allFinite()) {
                        Eigen::LDLT<Eigen::MatrixXd> ambiguity_ldlt(
                            ambiguity_covariance);
                        if (ambiguity_ldlt.info() == Eigen::Success) {
                            const Eigen::VectorXd ambiguity_delta =
                                ambiguity_float - fixed_ambiguities;
                            const Eigen::VectorXd correction =
                                ambiguity_ldlt.solve(ambiguity_delta);
                            if (ambiguity_ldlt.info() == Eigen::Success &&
                                correction.allFinite()) {
                                const Vector3d position_delta =
                                    position_ambiguity_covariance * correction;
                                if (position_delta.allFinite()) {
                                    epoch_output_positions[epoch_index] =
                                        state.segment<3>(epoch_col) -
                                        position_delta;
                                    epoch_lambda_fixed_output[epoch_index] = true;
                                    epoch_fixed_ambiguity_counts[epoch_index] =
                                        candidate_count;
                                    ++epoch_lambda_fixed_solution_count;
                                    epoch_lambda_fixed_ambiguity_total +=
                                        static_cast<std::size_t>(candidate_count);
                                    result.diagnostics
                                        .lambda_ambiguity_used_candidates +=
                                        static_cast<std::size_t>(candidate_count);
                                    result.diagnostics
                                        .lambda_ambiguity_fix_used = true;
                                }
                            }
                        }
                    }
                    result.diagnostics.epoch_lambda_fixed_output_time_ms +=
                        elapsed_ms(fixed_output_start,
                                   std::chrono::high_resolution_clock::now());
                }

                if (config_.collect_lambda_debug) {
                    const auto debug_record_start =
                        std::chrono::high_resolution_clock::now();
                    for (int row = 0; row < candidate_count; ++row) {
                        const std::size_t row_ambiguity_index =
                            candidate_indices[row];
                        const auto& row_ambiguity =
                            problem.ambiguity_states[row_ambiguity_index];
                        for (int col = 0; col < candidate_count; ++col) {
                            const auto& col_ambiguity =
                                problem.ambiguity_states[candidate_indices[col]];
                            LambdaDebugEntry entry;
                            entry.epoch_index = epoch_index;
                            entry.time = problem.epochs[epoch_index].time;
                            entry.solved = lambda_solved;
                            entry.fixed_epoch = fixed_epoch;
                            entry.ratio = lambda_solved ? lambda_ratio : 0.0;
                            entry.candidate_count = candidate_count;
                            entry.row = row;
                            entry.col = col;
                            entry.local_index = row;
                            entry.other_local_index = col;
                            entry.satellite = row_ambiguity.satellite;
                            entry.other_satellite = col_ambiguity.satellite;
                            entry.ambiguity_float = ambiguity_float(row);
                            entry.fixed_ambiguity =
                                lambda_solved && fixed_ambiguities.size() > row
                                    ? fixed_ambiguities(row)
                                    : std::numeric_limits<double>::quiet_NaN();
                            entry.covariance = ambiguity_covariance(row, col);
                            entry.position_covariance_x =
                                covarianceValue(epoch_col, row) /
                                row_ambiguity.wavelength_m;
                            entry.position_covariance_y =
                                covarianceValue(epoch_col + 1, row) /
                                row_ambiguity.wavelength_m;
                            entry.position_covariance_z =
                                covarianceValue(epoch_col + 2, row) /
                                row_ambiguity.wavelength_m;
                            result.lambda_debug_entries.push_back(entry);
                        }
                    }
                    result.diagnostics.epoch_lambda_debug_record_time_ms +=
                        elapsed_ms(debug_record_start,
                                   std::chrono::high_resolution_clock::now());
                }
            };

        if (covariance.rows() == state_size) {
            for (const auto& work : epoch_lambda_work) {
                process_epoch_lambda_work(
                    work,
                    [&](int row, int column_index) -> double {
                        return covariance(
                            row,
                            work.state_columns[static_cast<std::size_t>(
                                column_index)]);
                    });
            }
        } else if (has_sparse_covariance_solver) {
            constexpr int kMaxSparseCovarianceBatchColumns = 64;
            for (std::size_t begin = 0; begin < epoch_lambda_work.size();) {
                std::size_t end = begin;
                int batch_column_count = 0;
                while (end < epoch_lambda_work.size()) {
                    const int next_count =
                        epoch_lambda_work[end].candidate_count;
                    if (batch_column_count > 0 &&
                        batch_column_count + next_count >
                            kMaxSparseCovarianceBatchColumns) {
                        break;
                    }
                    batch_column_count += next_count;
                    ++end;
                }

                Eigen::MatrixXd rhs =
                    Eigen::MatrixXd::Zero(state_size, batch_column_count);
                int column_offset = 0;
                for (std::size_t work_index = begin; work_index < end;
                     ++work_index) {
                    const auto& work = epoch_lambda_work[work_index];
                    for (int col = 0; col < work.candidate_count; ++col) {
                        rhs(work.state_columns[static_cast<std::size_t>(col)],
                            column_offset + col) = 1.0;
                    }
                    column_offset += work.candidate_count;
                }

                const auto covariance_solve_start =
                    std::chrono::high_resolution_clock::now();
                const Eigen::MatrixXd batch_covariance_columns =
                    sparse_covariance_ldlt.solve(rhs);
                result.diagnostics.epoch_lambda_covariance_solve_time_ms +=
                    elapsed_ms(covariance_solve_start,
                               std::chrono::high_resolution_clock::now());
                if (sparse_covariance_ldlt.info() != Eigen::Success ||
                    batch_covariance_columns.cols() != batch_column_count) {
                    begin = end;
                    continue;
                }

                column_offset = 0;
                for (std::size_t work_index = begin; work_index < end;
                     ++work_index) {
                    const auto& work = epoch_lambda_work[work_index];
                    const int work_column_offset = column_offset;
                    process_epoch_lambda_work(
                        work,
                        [&](int row, int column_index) -> double {
                            return batch_covariance_columns(
                                row, work_column_offset + column_index);
                        });
                    column_offset += work.candidate_count;
                }

                begin = end;
            }
        }
    }
    if (compute_epoch_lambda) {
        const auto epoch_lambda_end =
            std::chrono::high_resolution_clock::now();
        result.diagnostics.epoch_lambda_processing_time_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                epoch_lambda_end - epoch_lambda_start)
                .count();
    }

    const bool has_epoch_lambda_fixed_outputs =
        epoch_lambda_fixed_solution_count > 0;
    if (has_epoch_lambda_fixed_outputs) {
        result.diagnostics.fixed_solution = true;
        result.diagnostics.fixed_ambiguities =
            epoch_lambda_fixed_ambiguity_total;
    }

    const std::size_t rows_per_motion_pair = motion_rows_per_pair();
    result.diagnostics.motion_factors =
        rows_per_motion_pair > 0
            ? motion_factor_count() / rows_per_motion_pair
            : 0;
    const std::size_t position_prior_factors =
        config_.position_prior_sigma_m > 0.0
            ? static_cast<std::size_t>(num_epochs)
            : 0;
    const std::size_t clock_prior_factors =
        config_.clock_prior_sigma_m > 0.0
            ? static_cast<std::size_t>(num_epochs)
            : 0;
    const std::size_t velocity_prior_factors =
        use_velocity_states && config_.velocity_prior_sigma_mps > 0.0
            ? static_cast<std::size_t>(num_epochs)
            : 0;
    const std::size_t ambiguity_prior_factors =
        config_.use_ambiguity_priors && config_.ambiguity_prior_sigma_m > 0.0
            ? problem.ambiguity_states.size()
            : 0;
    result.diagnostics.graph_factors =
        result.diagnostics.pseudorange_factors +
        result.diagnostics.tdcp_factors +
        result.diagnostics.single_difference_doppler_factors +
        result.diagnostics.single_difference_tdcp_factors +
        result.diagnostics.carrier_phase_factors +
        result.diagnostics.double_difference_pseudorange_factors +
        result.diagnostics.double_difference_carrier_factors +
        result.diagnostics.ambiguity_between_factors +
        result.diagnostics.motion_factors +
        position_prior_factors +
        clock_prior_factors +
        velocity_prior_factors +
        ambiguity_prior_factors +
        fixed_constraints.size();
    result.diagnostics.graph_values =
        static_cast<std::size_t>(state_size);
    result.diagnostics.robust_pseudorange_factors =
        optimization.robust_pseudorange_factors;
    result.diagnostics.robust_carrier_phase_factors =
        optimization.robust_carrier_phase_factors;
    result.diagnostics.robust_double_difference_pseudorange_factors =
        optimization.robust_double_difference_pseudorange_factors;
    result.diagnostics.robust_double_difference_carrier_factors =
        optimization.robust_double_difference_carrier_factors;
    result.diagnostics.robust_tdcp_factors = optimization.robust_tdcp_factors;
    result.ambiguity_estimates.reserve(problem.ambiguity_states.size());
    for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
        const auto& ambiguity = problem.ambiguity_states[i];
        AmbiguityEstimate estimate;
        estimate.satellite = ambiguity.satellite;
        estimate.signal = ambiguity.signal;
        estimate.segment_index = ambiguity.segment_index;
        estimate.wavelength_m = ambiguity.wavelength_m;
        estimate.ambiguity_m = state(base_state_size + static_cast<int>(i));
        if (ambiguity.wavelength_m > 0.0) {
            estimate.ambiguity_cycles = estimate.ambiguity_m / ambiguity.wavelength_m;
        }
        const auto fixed_it =
            std::find_if(fixed_constraints.begin(),
                         fixed_constraints.end(),
                         [i](const FixedAmbiguityConstraint& fixed) {
                             return fixed.ambiguity_index == i &&
                                    fixed.reference_ambiguity_index ==
                                        std::numeric_limits<std::size_t>::max();
                         });
        if (fixed_it != fixed_constraints.end()) {
            estimate.is_fixed = true;
            estimate.fixed_cycles = fixed_it->fixed_cycles;
            estimate.fixed_ambiguity_m = fixed_it->fixed_ambiguity_m;
            estimate.fix_residual_cycles = fixed_it->residual_cycles;
            estimate.fixed_by_lambda = fixed_it->fixed_by_lambda;
        }
        result.ambiguity_estimates.push_back(estimate);
    }

    std::map<std::size_t, std::vector<const PseudorangeFactor*>> factors_by_epoch;
    for (const auto& factor : problem.pseudorange_factors) {
        factors_by_epoch[factor.epoch_index].push_back(&factor);
    }
    std::map<std::size_t, std::set<SatelliteId>> dd_satellites_by_epoch;
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }
    for (const auto& factor : problem.double_difference_carrier_factors) {
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }
    std::map<std::size_t, std::size_t> dd_carrier_factors_by_epoch;
    for (const auto& factor : problem.double_difference_carrier_factors) {
        ++dd_carrier_factors_by_epoch[factor.epoch_index];
    }

    double residual_square_sum = 0.0;
    std::size_t residual_count = 0;
    double carrier_phase_residual_square_sum = 0.0;
    std::size_t carrier_phase_residual_count = 0;
    double double_difference_pseudorange_residual_square_sum = 0.0;
    std::size_t double_difference_pseudorange_residual_count = 0;
    double double_difference_carrier_residual_square_sum = 0.0;
    std::size_t double_difference_carrier_residual_count = 0;
    double tdcp_residual_square_sum = 0.0;
    std::size_t tdcp_residual_count = 0;
    double single_difference_doppler_residual_square_sum = 0.0;
    std::size_t single_difference_doppler_residual_count = 0;
    double single_difference_tdcp_residual_square_sum = 0.0;
    std::size_t single_difference_tdcp_residual_count = 0;
    const bool has_float_ambiguity_solution =
        ambiguity_count > 0 && has_ambiguity_measurements;
    const SolutionStatus fgo_solution_status =
        result.diagnostics.fixed_solution && !has_epoch_lambda_fixed_outputs
            ? SolutionStatus::FIXED
            : (has_float_ambiguity_solution ? SolutionStatus::FLOAT
                                            : SolutionStatus::SPP);

    if (use_velocity_states) {
        result.epoch_velocities_ecef_mps.resize(static_cast<std::size_t>(num_epochs));
    }

    Vector3d previous_output_position = Vector3d::Zero();
    bool has_previous_output_position = false;
    bool block_float_until_fixed = false;
    for (int i = 0; i < num_epochs; ++i) {
        const int epoch_col = epoch_state_col(static_cast<std::size_t>(i));
        PositionSolution solution;
        solution.time = problem.epochs[i].time;
        solution.status = fgo_solution_status;
        const std::size_t epoch_index = static_cast<std::size_t>(i);
        if (has_epoch_lambda_fixed_outputs && has_float_ambiguity_solution) {
            solution.status = epoch_lambda_fixed_output[epoch_index]
                                  ? SolutionStatus::FIXED
                                  : SolutionStatus::FLOAT;
        }
        solution.position_ecef = epoch_output_positions[epoch_index];
        solution.receiver_clock_bias =
            state(epoch_col + 3) / constants::SPEED_OF_LIGHT;
        solution.num_frequencies = 1;
        if (has_epoch_lambda_fixed_outputs && has_float_ambiguity_solution) {
            solution.ratio = epoch_lambda_ratios[epoch_index] > 0.0
                                 ? epoch_lambda_ratios[epoch_index]
                                 : 0.0;
        } else {
            solution.ratio =
                epoch_lambda_ratios[epoch_index] > 0.0
                    ? epoch_lambda_ratios[epoch_index]
                    : result.diagnostics.lambda_ambiguity_ratio;
        }
        solution.num_fixed_ambiguities =
            has_epoch_lambda_fixed_outputs
                ? (epoch_lambda_fixed_output[epoch_index]
                       ? epoch_fixed_ambiguity_counts[epoch_index]
                       : 0)
                : static_cast<int>(result.diagnostics.fixed_ambiguities);
        solution.iterations = result.diagnostics.iterations;
        solution.processing_time_ms = processing_ms / static_cast<double>(num_epochs);
        solution.position_covariance = Matrix3d::Identity() * 9999.0;

        double lat = 0.0;
        double lon = 0.0;
        double height = 0.0;
        ecef2geodetic(solution.position_ecef, lat, lon, height);
        solution.position_geodetic = GeodeticCoord(lat, lon, height);

        const auto epoch_factor_it = factors_by_epoch.find(static_cast<std::size_t>(i));
        bool has_epoch_pseudorange_solution = false;
        if (epoch_factor_it != factors_by_epoch.end()) {
            has_epoch_pseudorange_solution = true;
            const auto& epoch_factors = epoch_factor_it->second;
            solution.num_satellites = static_cast<int>(epoch_factors.size());
            solution.satellites_used.reserve(epoch_factors.size());
            solution.satellite_elevations.reserve(epoch_factors.size());
            solution.satellite_residuals.reserve(epoch_factors.size());

            Eigen::MatrixXd geometry = Eigen::MatrixXd::Zero(epoch_factors.size(), 4);
            for (int j = 0; j < static_cast<int>(epoch_factors.size()); ++j) {
                const auto& factor = *epoch_factors[j];
                const Vector3d delta = factor.satellite_position_ecef - solution.position_ecef;
                const double range = delta.norm();
                const Vector3d los = delta / range;
                const int bias_col = system_bias_col(epoch_col, factor.clock_group);
                double predicted_pseudorange = range + state(epoch_col + 3);
                if (bias_col >= 0) {
                    predicted_pseudorange += state(bias_col);
                }
                const double pseudorange_residual =
                    factor.corrected_pseudorange_m - predicted_pseudorange;

                solution.satellites_used.push_back(factor.satellite);
                solution.satellite_elevations.push_back(factor.elevation_rad);
                solution.satellite_residuals.push_back(pseudorange_residual);
                residual_square_sum += pseudorange_residual * pseudorange_residual;
                ++residual_count;

                geometry(j, 0) = -los(0);
                geometry(j, 1) = -los(1);
                geometry(j, 2) = -los(2);
                geometry(j, 3) = 1.0;
            }

            solution.residual_rms = std::sqrt(std::accumulate(
                solution.satellite_residuals.begin(),
                solution.satellite_residuals.end(),
                0.0,
                [](double acc, double residual) {
                    return acc + residual * residual;
                }) / static_cast<double>(solution.satellite_residuals.size()));

            if (geometry.rows() >= 4) {
                const Eigen::MatrixXd q = pseudoInverse(geometry.transpose() * geometry);
                solution.gdop = std::sqrt(std::max(0.0, q.trace()));
                solution.pdop = std::sqrt(std::max(0.0, q(0, 0) + q(1, 1) + q(2, 2)));
                solution.hdop = std::sqrt(std::max(0.0, q(0, 0) + q(1, 1)));
                solution.vdop = std::sqrt(std::max(0.0, q(2, 2)));
            }
        } else {
            const auto dd_satellite_it =
                dd_satellites_by_epoch.find(static_cast<std::size_t>(i));
            if (dd_satellite_it != dd_satellites_by_epoch.end()) {
                solution.num_satellites =
                    static_cast<int>(dd_satellite_it->second.size());
                solution.satellites_used.assign(dd_satellite_it->second.begin(),
                                                dd_satellite_it->second.end());
            }
        }

        if (!has_epoch_pseudorange_solution &&
            config_.min_output_double_difference_carrier_factors_per_epoch > 0) {
            const auto dd_carrier_count_it =
                dd_carrier_factors_by_epoch.find(static_cast<std::size_t>(i));
            const std::size_t dd_carrier_count =
                dd_carrier_count_it == dd_carrier_factors_by_epoch.end()
                    ? 0
                    : dd_carrier_count_it->second;
            if (dd_carrier_count <
                static_cast<std::size_t>(
                    config_
                        .min_output_double_difference_carrier_factors_per_epoch)) {
                solution.status = SolutionStatus::NONE;
            }
        }

        applyFloatPositionJumpGate(
            solution,
            previous_output_position,
            has_previous_output_position,
            config_.max_float_position_jump_m,
            block_float_until_fixed,
            result.diagnostics.float_rejected_position_jump);
        applyFloatSeedPositionDivergenceGate(
            solution,
            problem,
            static_cast<std::size_t>(i),
            config_.max_float_seed_position_divergence_m,
            result.diagnostics.float_rejected_seed_position_divergence);

        if (covariance.rows() == state_size) {
            solution.position_covariance =
                covariance.block<3, 3>(epoch_col, epoch_col) *
                (config_.pseudorange_sigma_m * config_.pseudorange_sigma_m);
        }

        result.solution.addSolution(solution);
        previous_output_position = solution.position_ecef;
        has_previous_output_position = true;

        if (use_velocity_states) {
            const int velocity_col = velocity_state_col(static_cast<std::size_t>(i));
            result.epoch_velocities_ecef_mps[static_cast<std::size_t>(i)] =
                state.segment<3>(velocity_col);
        }
    }

    for (const auto& factor : problem.carrier_phase_factors) {
        if (factor.epoch_index >= problem.epochs.size() ||
            factor.ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }

        const int epoch_col = epoch_state_col(factor.epoch_index);
        const int ambiguity_col = base_state_size + static_cast<int>(factor.ambiguity_index);
        const Vector3d position = state.segment<3>(epoch_col);
        const double range = (factor.satellite_position_ecef - position).norm();
        const int bias_col = system_bias_col(epoch_col, factor.clock_group);
        double predicted = range + state(epoch_col + 3) + state(ambiguity_col);
        if (bias_col >= 0) {
            predicted += state(bias_col);
        }
        const double carrier_phase_residual = factor.corrected_carrier_m - predicted;
        carrier_phase_residual_square_sum +=
            carrier_phase_residual * carrier_phase_residual;
        ++carrier_phase_residual_count;
    }

    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        if (factor.epoch_index >= problem.epochs.size()) {
            continue;
        }

        const int epoch_col = epoch_state_col(factor.epoch_index);
        const Vector3d position = state.segment<3>(epoch_col);
        const Vector3d seed_position =
            problem.epochs[factor.epoch_index].position_ecef;
        const Vector3d linearization_position =
            config_.linearize_double_difference_factors_at_seed
                ? seed_position
                : position;
        const DoubleDifferencePrediction dd_prediction =
            doubleDifferencePredictionAt(linearization_position,
                                         factor.base_position_ecef,
                                         factor.rover_satellite_position_ecef,
                                         factor.rover_reference_position_ecef,
                                         factor.base_satellite_position_ecef,
                                         factor.base_reference_position_ecef);
        if (!dd_prediction.valid) {
            continue;
        }
        double predicted = dd_prediction.geometry_m;
        if (config_.linearize_double_difference_factors_at_seed) {
            predicted +=
                dd_prediction.position_jacobian.dot(position - seed_position);
        }
        const double residual =
            factor.observed_dd_pseudorange_m - predicted;
        double_difference_pseudorange_residual_square_sum += residual * residual;
        ++double_difference_pseudorange_residual_count;
    }

    for (const auto& factor : problem.double_difference_carrier_factors) {
        if (factor.epoch_index >= problem.epochs.size() ||
            factor.ambiguity_index >= problem.ambiguity_states.size() ||
            (factor.use_ambiguity_difference &&
             factor.reference_ambiguity_index >= problem.ambiguity_states.size())) {
            continue;
        }

        const int epoch_col = epoch_state_col(factor.epoch_index);
        const int ambiguity_col = base_state_size + static_cast<int>(factor.ambiguity_index);
        const int reference_ambiguity_col =
            factor.use_ambiguity_difference
                ? base_state_size +
                      static_cast<int>(factor.reference_ambiguity_index)
                : -1;
        const Vector3d position = state.segment<3>(epoch_col);
        const Vector3d seed_position =
            problem.epochs[factor.epoch_index].position_ecef;
        const Vector3d linearization_position =
            config_.linearize_double_difference_factors_at_seed
                ? seed_position
                : position;
        const DoubleDifferencePrediction dd_prediction =
            doubleDifferencePredictionAt(linearization_position,
                                         factor.base_position_ecef,
                                         factor.rover_satellite_position_ecef,
                                         factor.rover_reference_position_ecef,
                                         factor.base_satellite_position_ecef,
                                         factor.base_reference_position_ecef);
        if (!dd_prediction.valid) {
            continue;
        }
        double predicted = dd_prediction.geometry_m + state(ambiguity_col);
        if (config_.linearize_double_difference_factors_at_seed) {
            predicted +=
                dd_prediction.position_jacobian.dot(position - seed_position);
        }
        if (factor.use_ambiguity_difference) {
            predicted -= state(reference_ambiguity_col);
        }
        const double residual =
            factor.observed_dd_carrier_m - predicted;
        double_difference_carrier_residual_square_sum += residual * residual;
        ++double_difference_carrier_residual_count;
    }

    for (const auto& factor : problem.tdcp_factors) {
        if (factor.previous_epoch_index >= problem.epochs.size() ||
            factor.current_epoch_index >= problem.epochs.size()) {
            continue;
        }

        const int previous_col = epoch_state_col(factor.previous_epoch_index);
        const int current_col = epoch_state_col(factor.current_epoch_index);
        const int previous_bias_col =
            system_bias_col(previous_col, clockBiasGroup(factor.satellite.system));
        const int current_bias_col =
            system_bias_col(current_col, clockBiasGroup(factor.satellite.system));
        const Vector3d previous_position = state.segment<3>(previous_col);
        const Vector3d current_position = state.segment<3>(current_col);
        const double previous_range =
            (factor.previous_satellite_position_ecef - previous_position).norm();
        const double current_range =
            (factor.current_satellite_position_ecef - current_position).norm();
        double predicted =
            current_range + state(current_col + 3) -
            previous_range - state(previous_col + 3);
        if (previous_bias_col >= 0 && current_bias_col >= 0 &&
            previous_bias_col != current_bias_col) {
            predicted += state(current_bias_col) - state(previous_bias_col);
        }
        const double tdcp_residual = factor.delta_carrier_m - predicted;
        tdcp_residual_square_sum += tdcp_residual * tdcp_residual;
        ++tdcp_residual_count;
    }

    for (const auto& factor : problem.single_difference_doppler_factors) {
        if (factor.epoch_index >= problem.epochs.size()) {
            continue;
        }

        Vector3d velocity = Vector3d::Zero();
        const int velocity_col = velocity_state_col(factor.epoch_index);
        if (velocity_col >= 0) {
            velocity = state.segment<3>(velocity_col);
        } else {
            if (factor.epoch_index == 0) {
                continue;
            }
            const std::size_t previous_epoch_index = factor.epoch_index - 1;
            const double dt =
                problem.epochs[factor.epoch_index].time -
                problem.epochs[previous_epoch_index].time;
            if (dt <= 0.0 ||
                (config_.max_tdcp_gap_s > 0.0 && dt > config_.max_tdcp_gap_s)) {
                continue;
            }

            const int previous_col = epoch_state_col(previous_epoch_index);
            const int current_col = epoch_state_col(factor.epoch_index);
            velocity =
                (state.segment<3>(current_col) -
                 state.segment<3>(previous_col)) /
                dt;
        }
        const double residual = factor.residual_mps - factor.los.dot(velocity);
        single_difference_doppler_residual_square_sum += residual * residual;
        ++single_difference_doppler_residual_count;
    }

    for (const auto& factor : problem.single_difference_tdcp_factors) {
        if (factor.previous_epoch_index >= problem.epochs.size() ||
            factor.current_epoch_index >= problem.epochs.size()) {
            continue;
        }

        const int previous_col = epoch_state_col(factor.previous_epoch_index);
        const int current_col = epoch_state_col(factor.current_epoch_index);
        const Vector3d previous_delta =
            state.segment<3>(previous_col) -
            problem.epochs[factor.previous_epoch_index].position_ecef;
        const Vector3d current_delta =
            state.segment<3>(current_col) -
            problem.epochs[factor.current_epoch_index].position_ecef;
        const double predicted =
            factor.los.dot(current_delta) -
            factor.previous_los.dot(previous_delta);
        const double residual = factor.delta_carrier_m - predicted;
        single_difference_tdcp_residual_square_sum += residual * residual;
        ++single_difference_tdcp_residual_count;
    }

    if (residual_count > 0) {
        result.diagnostics.residual_rms_m =
            std::sqrt(residual_square_sum / static_cast<double>(residual_count));
    }
    if (carrier_phase_residual_count > 0) {
        result.diagnostics.carrier_phase_residual_rms_m =
            std::sqrt(carrier_phase_residual_square_sum /
                      static_cast<double>(carrier_phase_residual_count));
    }
    if (double_difference_pseudorange_residual_count > 0) {
        result.diagnostics.double_difference_pseudorange_residual_rms_m =
            std::sqrt(double_difference_pseudorange_residual_square_sum /
                      static_cast<double>(
                          double_difference_pseudorange_residual_count));
    }
    if (double_difference_carrier_residual_count > 0) {
        result.diagnostics.double_difference_carrier_residual_rms_m =
            std::sqrt(double_difference_carrier_residual_square_sum /
                      static_cast<double>(double_difference_carrier_residual_count));
    }
    if (tdcp_residual_count > 0) {
        result.diagnostics.tdcp_residual_rms_m =
            std::sqrt(tdcp_residual_square_sum / static_cast<double>(tdcp_residual_count));
    }
    if (single_difference_doppler_residual_count > 0) {
        result.diagnostics.single_difference_doppler_residual_rms_mps =
            std::sqrt(single_difference_doppler_residual_square_sum /
                      static_cast<double>(
                          single_difference_doppler_residual_count));
    }
    if (single_difference_tdcp_residual_count > 0) {
        result.diagnostics.single_difference_tdcp_residual_rms_m =
            std::sqrt(single_difference_tdcp_residual_square_sum /
                      static_cast<double>(
                          single_difference_tdcp_residual_count));
    }

    const auto optimize_problem_end =
        std::chrono::high_resolution_clock::now();
    result.diagnostics.total_processing_time_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            optimize_problem_end - optimize_problem_start)
            .count();
    result.diagnostics.postprocessing_time_ms =
        std::max(0.0,
                 result.diagnostics.total_processing_time_ms -
                     result.diagnostics.processing_time_ms);

    return result;
}

}  // namespace libgnss
