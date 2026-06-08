#include <libgnss++/algorithms/fgo.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#ifdef GNSSPP_HAS_CHOLMOD
#include <Eigen/CholmodSupport>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <tuple>

namespace libgnss {
namespace {

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

struct FixedAmbiguityConstraint {
    std::size_t ambiguity_index = 0;
    double fixed_ambiguity_m = 0.0;
    int fixed_cycles = 0;
    double residual_cycles = 0.0;
    bool fixed_by_lambda = false;
};

using CarrierKey = std::pair<SatelliteId, SignalType>;

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

std::map<CarrierKey, PreparedCarrierObservation> prepareCarrierObservationsForReceiver(
    const ObservationData& epoch,
    const NavigationData& nav,
    const Vector3d& receiver_position,
    const FGOProcessor::FGOConfig& config,
    double min_snr_dbhz,
    bool require_carrier_phase = true,
    bool apply_elevation_mask = true) {
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
        if (!isPrimaryFgoSignal(observation.signal, config.use_multi_constellation)) {
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
            if (!isPrimaryFgoSignal(observation.signal, config_.use_multi_constellation)) {
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

    std::size_t base_cursor = 0;
    const double match_tolerance = std::max(0.0, config_.base_epoch_match_tolerance_s);
    const double pseudorange_sigma =
        std::max(1e-3, config_.double_difference_pseudorange_sigma_m);
    const double carrier_sigma =
        std::max(1e-4, config_.double_difference_carrier_sigma_m);
    const double max_segment_gap = std::max(0.0, config_.max_tdcp_gap_s);
    std::map<DoubleDifferenceAmbiguityKey, ActiveCarrierSegment>
        active_dd_segments;
    std::map<DoubleDifferenceSegmentKey, std::size_t> next_dd_segment_indices;
    using DdFactorKey =
        std::tuple<std::size_t, SatelliteId, SatelliteId, SignalType>;
    std::map<SingleDifferenceReferenceKey, SatelliteId> sd_reference_by_group;
    std::map<SingleDifferenceDefaultReferenceKey, std::map<SatelliteId, std::size_t>>
        sd_default_reference_counts;

    for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size(); ++epoch_index) {
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
                false);
        const auto base_pseudorange_observations =
            prepareCarrierObservationsForReceiver(
                matched_base_epoch,
                nav,
                base_position_ecef,
                config_,
                base_min_snr_dbhz,
                false,
                false);
        const auto base_reference_observations =
            prepareCarrierObservationsForReceiver(
                matched_base_epoch,
                nav,
                base_position_ecef,
                config_,
                reference_min_snr_dbhz,
                false,
                false);
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
                pseudorange_factor.sigma_m =
                    pseudorange_sigma / satellite_sqrt_sin_el;
                pseudorange_factor.elevation_rad = satellite->elevation_rad;
                pseudorange_factor.rover_satellite_model =
                    satellite->model_debug;
                pseudorange_factor.rover_reference_model =
                    reference->model_debug;
                pseudorange_factor.base_satellite_model =
                    base_satellite.model_debug;
                pseudorange_factor.base_reference_model =
                    base_reference.model_debug;
                problem.double_difference_pseudorange_factors.push_back(
                    pseudorange_factor);
                pseudorange_factors_by_key[DdFactorKey{
                    epoch_index,
                    satellite->satellite,
                    reference->satellite,
                    satellite->signal,
                }] = pseudorange_factor;
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
                const auto pseudorange_factor_it =
                    pseudorange_factors_by_key.find(DdFactorKey{
                        epoch_index,
                        satellite->satellite,
                        reference->satellite,
                        satellite->signal,
                    });
                if (pseudorange_factor_it == pseudorange_factors_by_key.end()) {
                    continue;
                }
                ++problem.diagnostics.double_difference_candidate_pairs;

                const auto& base_satellite = base_satellite_it->second;
                const auto& pseudorange_factor = pseudorange_factor_it->second;
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
                factor.use_ambiguity_difference = false;
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
                factor.sigma_m = carrier_sigma / satellite_sqrt_sin_el;
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
                const Vector3d sd_los = rover->los - reference->los;

                if (config_.use_single_difference_doppler_factors &&
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
                    factor.sigma_mps = rover->doppler_sigma_mps;
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

FGOProcessor::FGOResult FGOProcessor::optimizeProblem(const FGOProblem& problem) const {
    const auto optimize_problem_start =
        std::chrono::high_resolution_clock::now();
    FGOResult result;
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
    const bool use_sparse_normal = state_size > kSparseNormalStateThreshold;
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

    struct OptimizationOutput {
        Eigen::VectorXd state;
        Eigen::MatrixXd normal_matrix;
        Eigen::SparseMatrix<double> sparse_normal_matrix;
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
                    const double weighted_residual =
                        (fixed.fixed_ambiguity_m - output.state(ambiguity_col)) *
                        fixed_weight;
                    add_weighted_row({{ambiguity_col, fixed_weight}},
                                     weighted_residual);
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
                Eigen::SparseMatrix<double> sparse_normal(state_size, state_size);
                sparse_normal.setFromTriplets(
                    normal_triplets.begin(), normal_triplets.end());
                sparse_normal.makeCompressed();
                if (cost_converged()) {
                    store_current_linearization();
                    if (config_.collect_lambda_debug ||
                        config_.use_epoch_lambda_fixed_output) {
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
                    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
                    ldlt.setShift(damping);
                    ldlt.compute(sparse_normal);
                    if (ldlt.info() == Eigen::Success) {
                        dx = ldlt.solve(normal_rhs);
                        if (ldlt.info() == Eigen::Success && dx.allFinite()) {
                            solved = true;
                            break;
                        }
                    }
                    damping *= 10.0;
                }
                if (config_.collect_lambda_debug ||
                    config_.use_epoch_lambda_fixed_output) {
                    current_sparse_normal = std::move(sparse_normal);
                }
            } else {
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
                    Eigen::LDLT<Eigen::MatrixXd> ldlt(damped_normal);
                    if (ldlt.info() == Eigen::Success) {
                        dx = ldlt.solve(normal_rhs);
                        if (dx.allFinite()) {
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
                 config_.use_epoch_lambda_fixed_output)) {
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
            if (!config_.use_lambda_ambiguity_fix ||
                optimization.normal_matrix.rows() != state_size) {
                return false;
            }

            const Eigen::MatrixXd float_covariance =
                pseudoInverse(optimization.normal_matrix);
            if (float_covariance.rows() != state_size) {
                return false;
            }

            struct LambdaCandidate {
                int ambiguity_index = 0;
                double variance_cycles = 0.0;
                double fractional_cycles = 0.0;
            };

            std::vector<LambdaCandidate> candidates;
            candidates.reserve(problem.ambiguity_states.size());
            for (int i = 0; i < ambiguity_count; ++i) {
                const auto& ambiguity = problem.ambiguity_states[i];
                if (ambiguity.wavelength_m <= 0.0 || !is_fix_candidate(ambiguity)) {
                    continue;
                }

                const int col = base_state_size + i;
                const double variance_m2 = float_covariance(col, col);
                const double variance_cycles =
                    variance_m2 / (ambiguity.wavelength_m * ambiguity.wavelength_m);
                const double ambiguity_cycles =
                    optimization.state(col) / ambiguity.wavelength_m;
                if (!std::isfinite(variance_cycles) || variance_cycles <= 0.0 ||
                    !std::isfinite(ambiguity_cycles)) {
                    continue;
                }

                const double nearest_cycles = std::round(ambiguity_cycles);
                LambdaCandidate candidate;
                candidate.ambiguity_index = i;
                candidate.variance_cycles = variance_cycles;
                candidate.fractional_cycles = std::abs(ambiguity_cycles - nearest_cycles);
                candidates.push_back(candidate);
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
                    for (int col = 0; col < n; ++col) {
                        const int ambiguity_col = candidates[col].ambiguity_index;
                        const auto& col_state = problem.ambiguity_states[ambiguity_col];
                        const int state_col = base_state_size + ambiguity_col;
                        ambiguity_covariance(row, col) =
                            float_covariance(row_col, state_col) /
                            (row_state.wavelength_m * col_state.wavelength_m);
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
                const bool lambda_solved =
                    lambdaSearch(float_ambiguities,
                                 ambiguity_covariance,
                                 fixed_ambiguities,
                                 lambda_ratio);
                if (lambda_solved) {
                    result.diagnostics.lambda_ambiguity_fix_solved = true;
                }
                if (lambda_solved && std::isfinite(lambda_ratio)) {
                    result.diagnostics.lambda_ambiguity_ratio =
                        std::max(result.diagnostics.lambda_ambiguity_ratio,
                                 lambda_ratio);
                }
                if (!lambda_solved || !std::isfinite(lambda_ratio) ||
                    (config_.lambda_ratio_threshold > 0.0 &&
                     lambda_ratio < config_.lambda_ratio_threshold)) {
                    return false;
                }

                std::vector<FixedAmbiguityConstraint> lambda_constraints;
                lambda_constraints.reserve(subset_size);
                double lambda_residual_square_sum = 0.0;
                for (int i = 0; i < n; ++i) {
                    const int ambiguity_index = candidates[i].ambiguity_index;
                    const auto& ambiguity = problem.ambiguity_states[ambiguity_index];
                    const double fixed_cycles_d = std::round(fixed_ambiguities(i));
                    const double residual_cycles = float_ambiguities(i) - fixed_cycles_d;

                    FixedAmbiguityConstraint fixed;
                    fixed.ambiguity_index = static_cast<std::size_t>(ambiguity_index);
                    fixed.fixed_cycles = static_cast<int>(fixed_cycles_d);
                    fixed.fixed_ambiguity_m = fixed_cycles_d * ambiguity.wavelength_m;
                    fixed.residual_cycles = residual_cycles;
                    fixed.fixed_by_lambda = true;
                    lambda_constraints.push_back(fixed);
                    lambda_residual_square_sum += residual_cycles * residual_cycles;
                }

                if (static_cast<int>(lambda_constraints.size()) <
                    min_fixed_ambiguities) {
                    return false;
                }

                fixed_constraints = std::move(lambda_constraints);
                fixed_residual_square_sum = lambda_residual_square_sum;
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
            optimization.normal_matrix.rows() == state_size;
        const bool lambda_constraints_built =
            can_attempt_lambda && build_lambda_constraints();
        if (!lambda_constraints_built &&
            (!config_.use_lambda_ambiguity_fix || !can_attempt_lambda)) {
            build_nearest_integer_constraints();
        }

        if (static_cast<int>(fixed_constraints.size()) >=
            std::max(1, config_.min_fixed_ambiguities)) {
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
    result.diagnostics.last_update_norm_m =
        std::isfinite(optimization.last_dx_norm) ? optimization.last_dx_norm : 0.0;

    Eigen::MatrixXd covariance;
    if (optimization.normal_matrix.rows() == state_size) {
        covariance = pseudoInverse(optimization.normal_matrix);
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
                             return fixed.ambiguity_index == i;
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
