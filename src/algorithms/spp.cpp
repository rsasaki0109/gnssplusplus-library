#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <string>

namespace libgnss {

namespace {

bool isPrimarySPPSignal(SignalType signal, const SPPProcessor::SPPConfig& config) {
    switch (signal) {
        case SignalType::GPS_L1CA:
            return config.enable_gps;
        case SignalType::GAL_E1:
            return config.enable_galileo;
        case SignalType::QZS_L1CA:
            return config.enable_qzss;
        case SignalType::GLO_L1CA:
            return config.enable_glonass;
        case SignalType::BDS_B1I:
        case SignalType::BDS_B1C:
            return config.enable_beidou;
        default:
            return false;
    }
}

ReceiverClockBiasGroup clockBiasGroup(const SatelliteId& satellite) {
    switch (satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return ReceiverClockBiasGroup::GPS;
        case GNSSSystem::GLONASS:
            return ReceiverClockBiasGroup::GLONASS;
        case GNSSSystem::Galileo:
            return ReceiverClockBiasGroup::Galileo;
        case GNSSSystem::BeiDou:
            if (signal_policy::isBeiDou2Satellite(satellite)) {
                return ReceiverClockBiasGroup::BeiDou2;
            }
            if (signal_policy::isBeiDou3Satellite(satellite)) {
                return ReceiverClockBiasGroup::BeiDou3;
            }
            return ReceiverClockBiasGroup::BeiDou;
        case GNSSSystem::NavIC:
            return ReceiverClockBiasGroup::NavIC;
        default:
            return ReceiverClockBiasGroup::UNKNOWN;
    }
}

bool usesSeparateClockBias(ReceiverClockBiasGroup group) {
    return group != ReceiverClockBiasGroup::UNKNOWN && group != ReceiverClockBiasGroup::GPS;
}

std::string signalName(SignalType signal) {
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
        case SignalType::SIGNAL_TYPE_COUNT: return "SIGNAL_TYPE_COUNT";
    }
    return "UNKNOWN";
}

std::string diagnosticSignalName(SignalType signal) {
    if (signal == SignalType::SIGNAL_TYPE_COUNT) {
        return "";
    }
    return signalName(signal);
}

void writeCsvField(std::ostream& out, const std::string& value) {
    if (value.find_first_of(",\"\n\r") == std::string::npos) {
        out << value;
        return;
    }

    out << '"';
    for (const char ch : value) {
        if (ch == '"') {
            out << "\"\"";
        } else {
            out << ch;
        }
    }
    out << '"';
}

std::string clockGroupName(ReceiverClockBiasGroup group) {
    switch (group) {
        case ReceiverClockBiasGroup::GPS: return "GPS";
        case ReceiverClockBiasGroup::GLONASS: return "GLONASS";
        case ReceiverClockBiasGroup::Galileo: return "Galileo";
        case ReceiverClockBiasGroup::BeiDou: return "BeiDou";
        case ReceiverClockBiasGroup::BeiDou2: return "BeiDou2";
        case ReceiverClockBiasGroup::BeiDou3: return "BeiDou3";
        case ReceiverClockBiasGroup::QZSS: return "QZSS";
        case ReceiverClockBiasGroup::NavIC: return "NavIC";
        case ReceiverClockBiasGroup::UNKNOWN: return "UNKNOWN";
    }
    return "UNKNOWN";
}

ReceiverClockBiasGroup selectReferenceClockGroup(
    const std::map<ReceiverClockBiasGroup, int>& group_counts) {
    const auto gps_it = group_counts.find(ReceiverClockBiasGroup::GPS);
    if (gps_it != group_counts.end() && gps_it->second > 0) {
        return ReceiverClockBiasGroup::GPS;
    }

    ReceiverClockBiasGroup best_group = ReceiverClockBiasGroup::UNKNOWN;
    int best_count = -1;
    for (const auto& [group, count] : group_counts) {
        if (count > best_count) {
            best_group = group;
            best_count = count;
        }
    }
    return best_group;
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
                case SignalType::BDS_B3I:
                default:
                    return 0.0;
            }
        default:
            return 0.0;
    }
}

const Observation* findBestSecondarySPPObservation(const ObservationData& obs_data,
                                                   const Observation& primary) {
    const Observation* best = nullptr;
    int best_priority = 1000;
    for (const auto& candidate : obs_data.observations) {
        if (!(candidate.satellite == primary.satellite) ||
            !candidate.valid ||
            !candidate.has_pseudorange ||
            candidate.pseudorange <= 0.0 ||
            !signal_policy::isSecondarySignal(candidate.satellite.system, candidate.signal)) {
            continue;
        }
        const int priority = signal_policy::signalPriority(
            candidate.satellite.system, candidate.signal, false);
        if (priority < best_priority) {
            best = &candidate;
            best_priority = priority;
        }
    }
    return best;
}

struct IonosphereFreeCodeResult {
    Observation combined;
    SignalType primary_signal = SignalType::SIGNAL_TYPE_COUNT;
    SignalType secondary_signal = SignalType::SIGNAL_TYPE_COUNT;
    std::string primary_observation_code;
    std::string secondary_observation_code;
    double primary_pseudorange = 0.0;
    double secondary_pseudorange = 0.0;
    double primary_frequency_hz = 0.0;
    double secondary_frequency_hz = 0.0;
    double primary_coeff = 0.0;
    double secondary_coeff = 0.0;
};

bool formIonosphereFreeCodeObservation(const ObservationData& obs_data,
                                       const NavigationData& nav,
                                       const GNSSTime& time,
                                       const Observation& primary,
                                       IonosphereFreeCodeResult& result) {
    const Observation* secondary = findBestSecondarySPPObservation(obs_data, primary);
    if (secondary == nullptr) {
        return false;
    }

    const double travel_time = primary.pseudorange / constants::SPEED_OF_LIGHT;
    const GNSSTime tx_time = time - travel_time;
    const Ephemeris* eph = nav.getEphemeris(primary.satellite, tx_time);
    if (eph == nullptr) {
        return false;
    }

    const double f1 = signalFrequencyHz(primary.signal, eph);
    const double f2 = signalFrequencyHz(secondary->signal, eph);
    const double denominator = f1 * f1 - f2 * f2;
    if (f1 <= 0.0 || f2 <= 0.0 || std::abs(denominator) < 1.0) {
        return false;
    }

    const double c1 = (f1 * f1) / denominator;
    const double c2 = -(f2 * f2) / denominator;
    const double pseudorange_if =
        c1 * primary.pseudorange + c2 * secondary->pseudorange;
    if (!std::isfinite(pseudorange_if)) {
        return false;
    }

    result.combined = primary;
    result.combined.pseudorange = pseudorange_if;
    result.combined.observation_code =
        "IFLC(" + primary.observation_code + "," + secondary->observation_code + ")";
    result.primary_signal = primary.signal;
    result.secondary_signal = secondary->signal;
    result.primary_observation_code = primary.observation_code;
    result.secondary_observation_code = secondary->observation_code;
    result.primary_pseudorange = primary.pseudorange;
    result.secondary_pseudorange = secondary->pseudorange;
    result.primary_frequency_hz = f1;
    result.secondary_frequency_hz = f2;
    result.primary_coeff = c1;
    result.secondary_coeff = c2;
    return true;
}

bool envFlagEnabled(const char* name) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return false;
    }
    return std::string(value) != "0";
}

double envDoubleValue(const char* name, double fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    return end != value ? parsed : fallback;
}

int envIntValue(const char* name, int fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    return end != value ? static_cast<int>(parsed) : fallback;
}

double broadcastEphemerisVarianceForCodeWeight(const Ephemeris& eph) {
    constexpr double kUraValues[] = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0,
        192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0
    };

    const double accuracy =
        std::isfinite(eph.sv_accuracy) && eph.sv_accuracy >= 0.0 ? eph.sv_accuracy : 0.0;
    if (eph.satellite.system == GNSSSystem::Galileo) {
        const double sisa_cm = accuracy * 100.0;
        int index = 255;
        if (sisa_cm >= 0.0 && sisa_cm < 50.0) {
            index = static_cast<int>(std::ceil(sisa_cm));
        } else if (sisa_cm >= 50.0 && sisa_cm < 100.0) {
            index = static_cast<int>(std::ceil((sisa_cm - 50.0) / 2.0)) + 50;
        } else if (sisa_cm >= 100.0 && sisa_cm < 200.0) {
            index = static_cast<int>(std::ceil((sisa_cm - 100.0) / 4.0)) + 75;
        } else if (sisa_cm >= 200.0 && sisa_cm <= 600.0) {
            index = static_cast<int>(std::ceil((sisa_cm - 200.0) / 16.0)) + 100;
        }

        double sisa_value_cm = 6144.0 * 100.0;
        if (index >= 0 && index <= 49) {
            sisa_value_cm = static_cast<double>(index);
        } else if (index >= 50 && index <= 74) {
            sisa_value_cm = static_cast<double>(index - 50) * 2.0 + 50.0;
        } else if (index >= 75 && index <= 99) {
            sisa_value_cm = static_cast<double>(index - 75) * 4.0 + 100.0;
        } else if (index >= 100 && index <= 125) {
            sisa_value_cm = static_cast<double>(index - 100) * 16.0 + 200.0;
        }
        const double sisa_m = sisa_value_cm / 100.0;
        return sisa_m * sisa_m;
    }

    int index = 15;
    for (int i = 0; i < 15; ++i) {
        if (kUraValues[i] >= accuracy) {
            index = i;
            break;
        }
    }
    const double ura_m = (index >= 0 && index <= 14) ? kUraValues[index] : 6144.0;
    return ura_m * ura_m;
}

}  // namespace

SPPProcessor::SPPProcessor() {
    // Initialize with default configuration
    estimated_position_.setZero();
    receiver_clock_bias_ = 0.0;
}

SPPProcessor::SPPProcessor(const SPPConfig& spp_config) : spp_config_(spp_config) {
    estimated_position_.setZero();
    receiver_clock_bias_ = 0.0;
}

bool SPPProcessor::initialize(const ProcessorConfig& config) {
    config_ = config;
    reset();
    return true;
}

PositionSolution SPPProcessor::processEpoch(const ObservationData& obs, const NavigationData& nav) {
    auto start_time = std::chrono::high_resolution_clock::now();

    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;

    try {
        // Use RINEX header position for initialization if available
        const bool use_zero_initial_position =
            spp_config_.use_zero_initial_position ||
            envFlagEnabled("GNSS_SPP_ZERO_SEED") ||
            envFlagEnabled("GNSS_SPP_CLASLIB_ZERO_SEED");
        if (!use_zero_initial_position &&
            estimated_position_.norm() < 1000.0 &&
            obs.receiver_position.norm() > 1e6) {
            estimated_position_ = obs.receiver_position;
        }

        // Validate and filter observations
        auto valid_obs = validateObservations(obs, nav, obs.time);

        if (valid_obs.size() < 4) {
            updateStatistics(0.0, false);
            return solution;
        }

        // Solve position using native least-squares solver
        solution = solvePosition(valid_obs, nav, obs.time);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        solution.processing_time_ms = duration.count();

        updateStatistics(duration.count(), solution.isValid());

    } catch (const std::exception&) {
        updateStatistics(0.0, false);
    }

    return solution;
}

ProcessorStats SPPProcessor::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ProcessorStats stats;
    stats.total_epochs = total_epochs_processed_;
    stats.valid_solutions = successful_solutions_;
    if (total_epochs_processed_ > 0) {
        stats.average_processing_time_ms = total_processing_time_ms_ / total_epochs_processed_;
    }
    return stats;
}

void SPPProcessor::reset() {
    estimated_position_.setZero();
    receiver_clock_bias_ = 0.0;
    system_biases_.clear();

    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_ = 0;
    successful_solutions_ = 0;
    total_processing_time_ms_ = 0.0;
}

PositionSolution SPPProcessor::solvePosition(const std::vector<SPPCodeObservation>& valid_obs,
                                           const NavigationData& nav,
                                           const GNSSTime& time) {
    // Direct call to native least-squares solver
    return solvePositionLS(valid_obs, nav, time);
}

PositionSolution SPPProcessor::solvePositionLS(const std::vector<SPPCodeObservation>& valid_obs,
                                               const NavigationData& nav,
                                               const GNSSTime& time) {
    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::SPP;

    const bool use_zero_initial_position =
        spp_config_.use_zero_initial_position ||
        envFlagEnabled("GNSS_SPP_ZERO_SEED") ||
        envFlagEnabled("GNSS_SPP_CLASLIB_ZERO_SEED");
    if (estimated_position_.norm() < 1000.0 && !use_zero_initial_position) {
        initializePosition(valid_obs, nav, time);
    }

    struct MeasurementModel {
        Observation observation;
        SPPCodeObservation code_observation;
        bool ionosphere_free_code = false;
        Vector3d satellite_position;
        ReceiverClockBiasGroup clock_group = ReceiverClockBiasGroup::UNKNOWN;
        double corrected_pseudorange = 0.0;
        double elevation = 0.0;
        double weight = 1.0;
        double satellite_clock_correction = 0.0;
        double troposphere_delay = 0.0;
        double ionosphere_delay = 0.0;
        double group_delay = 0.0;
        double ephemeris_variance = 0.0;
        double variance = 1.0;
    };

    const double elevation_mask_rad = std::max(0.0, config_.elevation_mask) * M_PI / 180.0;
    const bool use_pntpos_code_weight =
        spp_config_.use_pntpos_code_weight ||
        spp_config_.use_claslib_code_weight ||
        envFlagEnabled("GNSS_SPP_CODE_WEIGHT") ||
        envFlagEnabled("GNSS_SPP_CLASLIB_WEIGHT");
    const double pntpos_err0 = envDoubleValue(
        "GNSS_SPP_CODE_WEIGHT_ERR0",
        envDoubleValue("GNSS_SPP_CLASLIB_ERR0", 100.0));
    const double pntpos_err_phase_m = envDoubleValue(
        "GNSS_SPP_CODE_WEIGHT_ERR_PHASE_M",
        envDoubleValue("GNSS_SPP_CLASLIB_ERR_PHASE_M", 0.010));
    const double pntpos_err_phase_el_m = envDoubleValue(
        "GNSS_SPP_CODE_WEIGHT_ERR_PHASE_EL_M",
        envDoubleValue("GNSS_SPP_CLASLIB_ERR_PHASE_EL_M", 0.005));

    auto buildMeasurements = [&](const Vector3d& current_position,
                                 const std::set<SatelliteId>& rejected_sats) {
        std::vector<MeasurementModel> measurements;
        measurements.reserve(valid_obs.size());

        const bool origin_position = current_position.norm() < 1000.0;
        double rcv_lat = 0.0;
        double rcv_lon = 0.0;
        double rcv_h = 0.0;
        if (!origin_position) {
            ecef2geodetic(current_position, rcv_lat, rcv_lon, rcv_h);
        }

        for (const auto& code_obs : valid_obs) {
            const Observation& obs = code_obs.observation;
            if (rejected_sats.find(obs.satellite) != rejected_sats.end()) {
                continue;
            }

            Vector3d sat_pos;
            Vector3d sat_vel;
            double sat_clk = 0.0;
            double sat_clk_drift = 0.0;

            const double tx_pseudorange = code_obs.primary_pseudorange > 0.0 ?
                code_obs.primary_pseudorange : obs.pseudorange;
            double travel_time = tx_pseudorange / constants::SPEED_OF_LIGHT;
            GNSSTime tx_time = time - travel_time;
            if (!nav.calculateSatelliteState(obs.satellite, tx_time,
                                             sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
                continue;
            }

            tx_time = tx_time - sat_clk;
            if (!nav.calculateSatelliteState(obs.satellite, tx_time,
                                             sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
                continue;
            }

            double signal_travel = (sat_pos - current_position).norm() / constants::SPEED_OF_LIGHT;
            double angle = constants::OMEGA_E * signal_travel;
            Eigen::Matrix3d earth_rotation;
            earth_rotation << std::cos(angle),  std::sin(angle), 0.0,
                             -std::sin(angle),  std::cos(angle), 0.0,
                              0.0,              0.0,             1.0;
            Vector3d corrected_sat_pos = earth_rotation * sat_pos;

            NavigationData::SatelliteGeometry geom;
            if (origin_position) {
                geom.distance = (corrected_sat_pos - current_position).norm();
                geom.elevation = M_PI / 2.0;
                geom.azimuth = 0.0;
            } else {
                geom = nav.calculateGeometry(current_position, corrected_sat_pos);
            }
            if (!origin_position && geom.elevation < elevation_mask_rad) {
                continue;
            }

            double trop_delay = models::tropDelaySaastamoinen(current_position, geom.elevation);

            const Ephemeris* eph = nav.getEphemeris(obs.satellite, tx_time);
            if (!eph) {
                continue;
            }

            double iono_delay = 0.0;
            if (!code_obs.ionosphere_free_code && nav.ionosphere_model.valid) {
                iono_delay = models::ionoDelayKlobuchar(
                    rcv_lat, rcv_lon, geom.azimuth, geom.elevation,
                    time.tow,
                    nav.ionosphere_model.alpha,
                    nav.ionosphere_model.beta);

                const double signal_frequency = signalFrequencyHz(obs.signal, eph);
                if (signal_frequency > 0.0) {
                    const double freq_scale = constants::GPS_L1_FREQ / signal_frequency;
                    iono_delay *= freq_scale * freq_scale;
                }
            }

            const double sat_clock_correction = sat_clk * constants::SPEED_OF_LIGHT;
            const double group_delay =
                code_obs.ionosphere_free_code ? 0.0 : groupDelayCorrectionMeters(obs, *eph);
            double corrected_pr = obs.pseudorange
                                  + sat_clock_correction
                                  - trop_delay
                                  - iono_delay
                                  - group_delay;

            double sin_el = std::sin(geom.elevation);
            if (sin_el < 0.1) {
                sin_el = 0.1;
            }

            double weight = sin_el * sin_el;
            double variance = weight > 0.0 ? 1.0 / weight : 1.0;
            double ephemeris_variance = 0.0;
            if (use_pntpos_code_weight) {
                const double fact = obs.satellite.system == GNSSSystem::GLONASS ? 1.5 : 1.0;
                ephemeris_variance = broadcastEphemerisVarianceForCodeWeight(*eph);
                variance = fact * fact * pntpos_err0 * pntpos_err0 *
                    (pntpos_err_phase_m * pntpos_err_phase_m +
                     pntpos_err_phase_el_m * pntpos_err_phase_el_m / sin_el);
                if (code_obs.ionosphere_free_code) {
                    variance *= 9.0;
                }
                variance += ephemeris_variance;
                variance += 0.3 * 0.3; // CLASLIB ERR_CBIAS
                variance += std::pow(0.3 / (sin_el + 0.1), 2); // CLASLIB TROPOPT_SAAS variance
                if (variance > 0.0 && std::isfinite(variance)) {
                    weight = 1.0 / variance;
                } else {
                    variance = weight > 0.0 ? 1.0 / weight : 1.0;
                }
            }

            MeasurementModel measurement;
            measurement.observation = obs;
            measurement.code_observation = code_obs;
            measurement.ionosphere_free_code = code_obs.ionosphere_free_code;
            measurement.satellite_position = corrected_sat_pos;
            measurement.clock_group = clockBiasGroup(obs.satellite);
            measurement.corrected_pseudorange = corrected_pr;
            measurement.elevation = geom.elevation;
            measurement.weight = weight;
            measurement.satellite_clock_correction = sat_clock_correction;
            measurement.troposphere_delay = trop_delay;
            measurement.ionosphere_delay = iono_delay;
            measurement.group_delay = group_delay;
            measurement.ephemeris_variance = ephemeris_variance;
            measurement.variance = variance;
            if (measurement.clock_group == ReceiverClockBiasGroup::UNKNOWN) {
                continue;
            }
            measurements.push_back(measurement);
        }

        return measurements;
    };

    Vector3d position = estimated_position_;
    double clock_bias = receiver_clock_bias_;
    std::map<ReceiverClockBiasGroup, double> bias_estimates;
    std::vector<MeasurementModel> final_measurements;
    ReceiverClockBiasGroup final_reference_group = ReceiverClockBiasGroup::UNKNOWN;
    std::vector<ReceiverClockBiasGroup> final_bias_groups;
    std::set<SatelliteId> rejected_sats;
    const bool residual_rejection_env =
        std::getenv("GNSS_SPP_RESIDUAL_REJECT_M") != nullptr ||
        std::getenv("GNSS_SPP_CLASLIB_REJECT_M") != nullptr;
    const double residual_rejection_config_threshold =
        spp_config_.enable_residual_rejection ?
            spp_config_.residual_rejection_threshold :
            spp_config_.claslib_residual_rejection_threshold;
    const int residual_rejection_config_min_observations =
        spp_config_.enable_residual_rejection ?
            spp_config_.residual_rejection_min_observations :
            spp_config_.claslib_residual_rejection_min_observations;
    const double residual_rejection_threshold = envDoubleValue(
        "GNSS_SPP_RESIDUAL_REJECT_M",
        envDoubleValue("GNSS_SPP_CLASLIB_REJECT_M", residual_rejection_config_threshold));
    const int residual_rejection_min_observations = envIntValue(
        "GNSS_SPP_RESIDUAL_REJECT_MIN_OBS",
        envIntValue("GNSS_SPP_CLASLIB_REJECT_MIN_OBS",
                    residual_rejection_config_min_observations));
    const bool enable_residual_rejection =
        residual_rejection_threshold > 0.0 &&
        (spp_config_.enable_residual_rejection ||
         spp_config_.enable_claslib_residual_rejection ||
         residual_rejection_env);

    auto updateRejectedSatellites = [&](const std::vector<MeasurementModel>& measurements,
                                        const VectorXd& residuals) {
        if (!enable_residual_rejection ||
            static_cast<int>(measurements.size()) <= residual_rejection_min_observations ||
            residuals.size() != static_cast<int>(measurements.size())) {
            return false;
        }

        const double residual_rms =
            std::sqrt(residuals.squaredNorm() / static_cast<double>(measurements.size()));
        if (residual_rms > residual_rejection_threshold) {
            return false;
        }

        bool rejected = false;
        for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
            if (std::abs(residuals(i)) <= residual_rejection_threshold) {
                continue;
            }
            rejected = rejected_sats.insert(measurements[i].observation.satellite).second || rejected;
        }
        return rejected;
    };

    auto dumpIteration = [&](int iter,
                             const std::vector<MeasurementModel>& measurements,
                             const VectorXd& residuals,
                             const VectorXd& dx,
                             const Vector3d& position_before,
                             double clock_bias_before,
                             const Vector3d& position_after,
                             double clock_bias_after,
                             ReceiverClockBiasGroup reference_group,
                             int num_unknowns) {
        const char* dump_path = std::getenv("GNSS_SPP_ITERATION_DUMP");
        if (dump_path == nullptr || dump_path[0] == '\0' ||
            residuals.size() != static_cast<int>(measurements.size())) {
            return;
        }

        std::ifstream existing(dump_path);
        const bool write_header = !existing.good() ||
            existing.peek() == std::ifstream::traits_type::eof();
        existing.close();

        std::ofstream dump(dump_path, std::ios::app);
        if (!dump) {
            return;
        }

        dump << std::setprecision(17);
        if (write_header) {
            dump << "week,tow,iteration,sat,signal,obs_code,ionosphere_free_code,"
                 << "rejected_after,abs_residual_gt_threshold,num_measurements,num_unknowns,"
                 << "residual_rms_m,dx_norm_m,pos_before_x_m,pos_before_y_m,pos_before_z_m,"
                 << "pos_after_x_m,pos_after_y_m,pos_after_z_m,clock_before_m,clock_after_m,"
                 << "elevation_deg,weight,raw_pseudorange_m,corrected_pseudorange_m,residual_m,"
                 << "geometric_range_m,predicted_pseudorange_m,sat_clock_correction_m,"
                 << "troposphere_delay_m,ionosphere_delay_m,group_delay_m,"
                 << "ephemeris_variance_m2,variance_m2,clock_group,"
                 << "reference_clock_group\n";
        }

        const double residual_rms = measurements.empty() ? 0.0 :
            std::sqrt(residuals.squaredNorm() / static_cast<double>(measurements.size()));
        const double dx_norm = dx.size() >= 3 ? dx.head<3>().norm() : 0.0;
        for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
            const auto& measurement = measurements[i];
            const double geometric_range =
                (measurement.satellite_position - position_before).norm();
            const double predicted = measurement.corrected_pseudorange - residuals(i);
            dump << time.week << ',' << time.tow << ',' << iter << ',';
            writeCsvField(dump, measurement.observation.satellite.toString());
            dump << ',';
            writeCsvField(dump, signalName(measurement.observation.signal));
            dump << ',';
            writeCsvField(dump, measurement.observation.observation_code);
            dump << ','
                 << (measurement.ionosphere_free_code ? 1 : 0) << ','
                 << (rejected_sats.find(measurement.observation.satellite) != rejected_sats.end() ? 1 : 0) << ','
                 << (std::abs(residuals(i)) > residual_rejection_threshold ? 1 : 0) << ','
                 << measurements.size() << ','
                 << num_unknowns << ','
                 << residual_rms << ','
                 << dx_norm << ','
                 << position_before.x() << ','
                 << position_before.y() << ','
                 << position_before.z() << ','
                 << position_after.x() << ','
                 << position_after.y() << ','
                 << position_after.z() << ','
                 << clock_bias_before << ','
                 << clock_bias_after << ','
                 << measurement.elevation * 180.0 / M_PI << ','
                 << measurement.weight << ','
                 << measurement.observation.pseudorange << ','
                 << measurement.corrected_pseudorange << ','
                 << residuals(i) << ','
                 << geometric_range << ','
                 << predicted << ','
                 << measurement.satellite_clock_correction << ','
                 << measurement.troposphere_delay << ','
                 << measurement.ionosphere_delay << ','
                 << measurement.group_delay << ','
                 << measurement.ephemeris_variance << ','
                 << measurement.variance << ',';
            writeCsvField(dump, clockGroupName(measurement.clock_group));
            dump << ',';
            writeCsvField(dump, clockGroupName(reference_group));
            dump << '\n';
        }
    };

    for (int iter = 0; iter < spp_config_.max_iterations; ++iter) {
        auto measurements = buildMeasurements(position, rejected_sats);
        if (measurements.size() < 4) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        std::map<ReceiverClockBiasGroup, int> group_counts;
        for (const auto& measurement : measurements) {
            group_counts[measurement.clock_group]++;
        }

        const ReceiverClockBiasGroup reference_group = selectReferenceClockGroup(group_counts);
        std::vector<ReceiverClockBiasGroup> bias_groups;
        if (spp_config_.model_intersystem_bias) {
            for (const auto& [group, count] : group_counts) {
                if (count <= 0 || group == reference_group || !usesSeparateClockBias(group)) {
                    continue;
                }
                bias_groups.push_back(group);
            }
        }

        const int n = static_cast<int>(measurements.size());
        const int num_unknowns = 4 + static_cast<int>(bias_groups.size());
        if (n < num_unknowns) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        std::map<ReceiverClockBiasGroup, int> bias_columns;
        for (int i = 0; i < static_cast<int>(bias_groups.size()); ++i) {
            bias_columns[bias_groups[i]] = 4 + i;
        }

        MatrixXd H = MatrixXd::Zero(n, num_unknowns);
        VectorXd residuals = VectorXd::Zero(n);
        MatrixXd weighted_H = MatrixXd::Zero(n, num_unknowns);
        VectorXd weighted_residuals = VectorXd::Zero(n);

        for (int i = 0; i < n; ++i) {
            const auto& measurement = measurements[i];
            const Vector3d los = (measurement.satellite_position - position).normalized();
            const double geometric_range = (measurement.satellite_position - position).norm();

            H(i, 0) = -los(0);
            H(i, 1) = -los(1);
            H(i, 2) = -los(2);
            H(i, 3) = 1.0;

            double predicted = geometric_range + clock_bias;
            const auto bias_col_it = bias_columns.find(measurement.clock_group);
            if (bias_col_it != bias_columns.end()) {
                predicted += bias_estimates[measurement.clock_group];
                H(i, bias_col_it->second) = 1.0;
            }

            residuals(i) = measurement.corrected_pseudorange - predicted;

            const double sigma = std::sqrt(measurement.weight);
            weighted_H.row(i) = H.row(i) * sigma;
            weighted_residuals(i) = residuals(i) * sigma;
        }

        Eigen::ColPivHouseholderQR<MatrixXd> qr(weighted_H);
        if (qr.rank() < num_unknowns) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        VectorXd dx = qr.solve(weighted_residuals);
        if (!dx.allFinite()) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        const Vector3d position_before = position;
        const double clock_bias_before = clock_bias;
        position += dx.head<3>();
        clock_bias += dx(3);
        for (int i = 0; i < static_cast<int>(bias_groups.size()); ++i) {
            bias_estimates[bias_groups[i]] += dx(4 + i);
        }

        final_measurements = std::move(measurements);
        final_reference_group = reference_group;
        final_bias_groups = bias_groups;

        const bool converged = dx.head<3>().norm() < spp_config_.position_convergence_threshold;
        if (!converged) {
            updateRejectedSatellites(final_measurements, residuals);
        }
        dumpIteration(iter,
                      final_measurements,
                      residuals,
                      dx,
                      position_before,
                      clock_bias_before,
                      position,
                      clock_bias,
                      reference_group,
                      num_unknowns);

        if (converged) {
            solution.iterations = iter + 1;
            break;
        }
        if (iter == spp_config_.max_iterations - 1) {
            solution.iterations = spp_config_.max_iterations;
        }
    }

    final_measurements = buildMeasurements(position, rejected_sats);
    if (final_measurements.size() < 4) {
        solution.status = SolutionStatus::NONE;
        return solution;
    }

    std::map<ReceiverClockBiasGroup, int> final_group_counts;
    for (const auto& measurement : final_measurements) {
        final_group_counts[measurement.clock_group]++;
    }
    final_reference_group = selectReferenceClockGroup(final_group_counts);
    final_bias_groups.clear();
    if (spp_config_.model_intersystem_bias) {
        for (const auto& [group, count] : final_group_counts) {
            if (count <= 0 || group == final_reference_group || !usesSeparateClockBias(group)) {
                continue;
            }
            final_bias_groups.push_back(group);
        }
    }

    std::map<ReceiverClockBiasGroup, int> final_bias_columns;
    for (int i = 0; i < static_cast<int>(final_bias_groups.size()); ++i) {
        final_bias_columns[final_bias_groups[i]] = 4 + i;
    }

    VectorXd final_residuals = VectorXd::Zero(static_cast<int>(final_measurements.size()));
    std::vector<Vector3d> final_sat_positions;
    final_sat_positions.reserve(final_measurements.size());
    for (int i = 0; i < static_cast<int>(final_measurements.size()); ++i) {
        const auto& measurement = final_measurements[i];
        const double geometric_range = (measurement.satellite_position - position).norm();
        double predicted = geometric_range + clock_bias;
        const auto bias_col_it = final_bias_columns.find(measurement.clock_group);
        if (bias_col_it != final_bias_columns.end()) {
            predicted += bias_estimates[measurement.clock_group];
        }
        final_residuals(i) = measurement.corrected_pseudorange - predicted;
        final_sat_positions.push_back(measurement.satellite_position);
    }

    if (const char* dump_path = std::getenv("GNSS_SPP_RESIDUAL_DUMP")) {
        if (dump_path[0] != '\0') {
            std::ifstream existing(dump_path);
            const bool write_header = !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
            existing.close();

            std::ofstream dump(dump_path, std::ios::app);
            if (dump) {
                dump << std::setprecision(17);
                if (write_header) {
                    dump << "week,tow,sat,signal,obs_code,ionosphere_free_code,raw_pseudorange_m,corrected_pseudorange_m,"
                         << "primary_signal,primary_obs_code,secondary_signal,secondary_obs_code,"
                         << "primary_pseudorange_m,secondary_pseudorange_m,iflc_primary_coeff,iflc_secondary_coeff,"
                         << "primary_frequency_hz,secondary_frequency_hz,"
                         << "sat_clock_correction_m,troposphere_delay_m,ionosphere_delay_m,group_delay_m,"
                         << "ephemeris_variance_m2,variance_m2,"
                         << "sat_x_m,sat_y_m,sat_z_m,elevation_deg,weight,residual_m,"
                         << "geometric_range_m,predicted_pseudorange_m,clock_group,reference_clock_group,"
                         << "receiver_clock_bias_m,system_bias_m,pos_x_m,pos_y_m,pos_z_m,"
                         << "num_measurements,residual_rms_m\n";
                }

                const double residual_rms = std::sqrt(final_residuals.squaredNorm() /
                                                      static_cast<double>(final_measurements.size()));
                for (int i = 0; i < static_cast<int>(final_measurements.size()); ++i) {
                    const auto& measurement = final_measurements[i];
                    const double geometric_range = (measurement.satellite_position - position).norm();
                    double system_bias_m = 0.0;
                    double predicted = geometric_range + clock_bias;
                    const auto bias_col_it = final_bias_columns.find(measurement.clock_group);
                    if (bias_col_it != final_bias_columns.end()) {
                        system_bias_m = bias_estimates[measurement.clock_group];
                        predicted += system_bias_m;
                    }

                    const auto& code_observation = measurement.code_observation;
                    dump << time.week << ',' << time.tow << ',';
                    writeCsvField(dump, measurement.observation.satellite.toString());
                    dump << ',';
                    writeCsvField(dump, signalName(measurement.observation.signal));
                    dump << ',';
                    writeCsvField(dump, measurement.observation.observation_code);
                    dump << ','
                         << (measurement.ionosphere_free_code ? 1 : 0) << ','
                         << measurement.observation.pseudorange << ','
                         << measurement.corrected_pseudorange << ',';
                    writeCsvField(dump, diagnosticSignalName(code_observation.primary_signal));
                    dump << ',';
                    writeCsvField(dump, code_observation.primary_observation_code);
                    dump << ',';
                    writeCsvField(dump, diagnosticSignalName(code_observation.secondary_signal));
                    dump << ',';
                    writeCsvField(dump, code_observation.secondary_observation_code);
                    dump << ','
                         << code_observation.primary_pseudorange << ','
                         << code_observation.secondary_pseudorange << ','
                         << code_observation.iflc_primary_coeff << ','
                         << code_observation.iflc_secondary_coeff << ','
                         << code_observation.primary_frequency_hz << ','
                         << code_observation.secondary_frequency_hz << ','
                         << measurement.satellite_clock_correction << ','
                         << measurement.troposphere_delay << ','
                         << measurement.ionosphere_delay << ','
                         << measurement.group_delay << ','
                         << measurement.ephemeris_variance << ','
                         << measurement.variance << ','
                         << measurement.satellite_position.x() << ','
                         << measurement.satellite_position.y() << ','
                         << measurement.satellite_position.z() << ','
                         << measurement.elevation * 180.0 / M_PI << ','
                         << measurement.weight << ','
                         << final_residuals(i) << ','
                         << geometric_range << ','
                         << predicted << ',';
                    writeCsvField(dump, clockGroupName(measurement.clock_group));
                    dump << ',';
                    writeCsvField(dump, clockGroupName(final_reference_group));
                    dump << ','
                         << clock_bias << ','
                         << system_bias_m << ','
                         << position.x() << ','
                         << position.y() << ','
                         << position.z() << ','
                         << final_measurements.size() << ','
                         << residual_rms << '\n';
                }
            }
        }
    }

    solution.position_ecef = position;
    solution.receiver_clock_bias = clock_bias;
    solution.receiver_clock_biases_m.clear();
    for (const auto& [group, count] : final_group_counts) {
        if (count <= 0 || group == ReceiverClockBiasGroup::UNKNOWN) {
            continue;
        }
        double group_clock_bias_m = clock_bias;
        if (group != final_reference_group) {
            const auto bias_it = bias_estimates.find(group);
            if (bias_it != bias_estimates.end()) {
                group_clock_bias_m += bias_it->second;
            }
        }
        solution.receiver_clock_biases_m[group] = group_clock_bias_m;
    }
    solution.num_satellites = static_cast<int>(final_measurements.size());
    solution.num_frequencies = 1;
    solution.residual_rms = std::sqrt(final_residuals.squaredNorm() /
                                      static_cast<double>(final_measurements.size()));

    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(position, lat, lon, height);
    solution.position_geodetic = GeodeticCoord(lat, lon, height);

    // Calculate DOP from satellite geometry using final position
    if (final_sat_positions.size() >= 4) {
        MatrixXd H = calculateGeometryMatrix(position, final_sat_positions);
        calculateDOP(H, solution);
    }

    for (int i = 0; i < static_cast<int>(final_measurements.size()); ++i) {
        solution.satellites_used.push_back(final_measurements[i].observation.satellite);
        solution.satellite_elevations.push_back(final_measurements[i].elevation);
        solution.satellite_residuals.push_back(final_residuals(i));
    }

    estimated_position_ = position;
    receiver_clock_bias_ = clock_bias;
    system_biases_.clear();
    return solution;
}

std::vector<SPPProcessor::SPPCodeObservation> SPPProcessor::validateObservations(
    const ObservationData& obs,
    const NavigationData& nav,
    const GNSSTime& time) const {
    std::vector<SPPCodeObservation> valid_obs;
    const bool use_ionosphere_free_code =
        spp_config_.use_ionosphere_free_code || envFlagEnabled("GNSS_SPP_CODE_IFLC");

    for (const auto& observation : obs.observations) {
        if (spp_config_.use_multi_constellation) {
            if (!isPrimarySPPSignal(observation.signal, spp_config_)) continue;
        } else if (observation.signal != SignalType::GPS_L1CA) {
            continue;
        }

        // Check observation validity first
        if (!observation.valid || !observation.has_pseudorange || observation.pseudorange <= 0.0) {
            continue;
        }

        // BeiDou GEO handling still needs tighter validation than the current
        // broadcast model provides. Keep MEO/IGSO enabled and gate GEO for now.
        if (signal_policy::isBeiDouGeoSatellite(observation.satellite)) {
            continue;
        }

        // Estimate transmission time to check for ephemeris validity
        double travel_time = observation.pseudorange / constants::SPEED_OF_LIGHT;
        GNSSTime tx_time = time - travel_time;

        // Check if ephemeris is available at the estimated transmission time
        const Ephemeris* eph = nav.getEphemeris(observation.satellite, tx_time);
        if (eph == nullptr) {
            continue;
        }

        // Exclude unhealthy satellites (RTKLIB satexclude convention).
        // QZSS encodes the LEX/L6 health on bit 0 of the 6-bit GPS-style health
        // field; mask it off so a satellite flagged only for LEX is still usable.
        unsigned int svh = eph->health;
        if (observation.satellite.system == GNSSSystem::QZSS) {
            svh &= 0xFE;
        }
        if (svh != 0) {
            continue;
        }

        // Check SNR threshold
        if (observation.snr < config_.snr_mask) {
            continue;
        }

        SPPCodeObservation code_observation;
        code_observation.observation = observation;
        code_observation.primary_signal = observation.signal;
        code_observation.primary_observation_code = observation.observation_code;
        code_observation.primary_pseudorange = observation.pseudorange;
        code_observation.primary_frequency_hz = signalFrequencyHz(observation.signal, eph);
        if (use_ionosphere_free_code) {
            IonosphereFreeCodeResult iflc;
            if (formIonosphereFreeCodeObservation(obs, nav, time, observation, iflc)) {
                code_observation.observation = iflc.combined;
                code_observation.ionosphere_free_code = true;
                code_observation.primary_signal = iflc.primary_signal;
                code_observation.secondary_signal = iflc.secondary_signal;
                code_observation.primary_observation_code = iflc.primary_observation_code;
                code_observation.secondary_observation_code = iflc.secondary_observation_code;
                code_observation.primary_pseudorange = iflc.primary_pseudorange;
                code_observation.secondary_pseudorange = iflc.secondary_pseudorange;
                code_observation.primary_frequency_hz = iflc.primary_frequency_hz;
                code_observation.secondary_frequency_hz = iflc.secondary_frequency_hz;
                code_observation.iflc_primary_coeff = iflc.primary_coeff;
                code_observation.iflc_secondary_coeff = iflc.secondary_coeff;
            }
        }

        valid_obs.push_back(code_observation);
    }

    return valid_obs;
}

bool SPPProcessor::initializePosition(const std::vector<SPPCodeObservation>& observations,
                                    const NavigationData& nav,
                                    const GNSSTime& time) {
    // Simple initialization - use first satellite position as rough estimate
    if (!observations.empty()) {
        auto sat_states = calculateSatelliteStates(observations, nav, time);

        if (!sat_states.empty()) {
            estimated_position_ = sat_states.begin()->second.position;
            estimated_position_ *= 0.9; // Move closer to Earth center
            return true;
        }
    }
    return false;
}

std::map<SatelliteId, SPPProcessor::SatelliteState> SPPProcessor::calculateSatelliteStates(
    const std::vector<SPPCodeObservation>& observations,
    const NavigationData& nav,
    const GNSSTime& time) const {

    std::map<SatelliteId, SatelliteState> states;

    for (const auto& code_obs : observations) {
        const Observation& obs = code_obs.observation;
        SatelliteState state;

        // Estimate signal travel time and calculate transmission time
        const double tx_pseudorange = code_obs.primary_pseudorange > 0.0 ?
            code_obs.primary_pseudorange : obs.pseudorange;
        double travel_time = tx_pseudorange / constants::SPEED_OF_LIGHT;
        GNSSTime tx_time = time - travel_time;

        // First: get satellite clock at approximate tx time
        state.valid = nav.calculateSatelliteState(obs.satellite, tx_time,
                                                state.position, state.velocity,
                                                state.clock_bias, state.clock_drift);
        if (state.valid) {
            // Correct tx time for satellite clock bias
            tx_time = tx_time - state.clock_bias;
            // Recompute at corrected tx time
            state.valid = nav.calculateSatelliteState(obs.satellite, tx_time,
                                                    state.position, state.velocity,
                                                    state.clock_bias, state.clock_drift);
        }
        states[obs.satellite] = state;
    }

    return states;
}

double SPPProcessor::calculatePredictedRange(const Vector3d& receiver_pos,
                                           const Vector3d& satellite_pos,
                                           double receiver_clock_bias_m,
                                           double satellite_clock_bias_s) const {
    double geometric_range = (satellite_pos - receiver_pos).norm();
    // receiver_clock_bias_m is in meters, satellite_clock_bias is in seconds
    double clock_correction = receiver_clock_bias_m - satellite_clock_bias_s * constants::SPEED_OF_LIGHT;
    return geometric_range + clock_correction;
}

MatrixXd SPPProcessor::calculateGeometryMatrix(const Vector3d& receiver_pos,
                                             const std::vector<Vector3d>& satellite_positions) const {
    int n_sats = satellite_positions.size();
    MatrixXd H(n_sats, 4);

    for (int i = 0; i < n_sats; ++i) {
        Vector3d los = (satellite_positions[i] - receiver_pos).normalized();
        H(i, 0) = -los(0);
        H(i, 1) = -los(1);
        H(i, 2) = -los(2);
        H(i, 3) = 1.0;
    }

    return H;
}

void SPPProcessor::calculateDOP(const MatrixXd& geometry_matrix, PositionSolution& solution) const {
    MatrixXd Q = (geometry_matrix.transpose() * geometry_matrix).inverse();

    solution.gdop = std::sqrt(Q.trace());
    solution.pdop = std::sqrt(Q(0,0) + Q(1,1) + Q(2,2));
    solution.hdop = std::sqrt(Q(0,0) + Q(1,1));
    solution.vdop = std::sqrt(Q(2,2));
}

void SPPProcessor::updateStatistics(double processing_time_ms, bool success) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_++;
    total_processing_time_ms_ += processing_time_ms;
    if (success) {
        successful_solutions_++;
    }
}

// Utility functions
namespace spp_utils {

double calculateElevation(const Vector3d& receiver_pos, const Vector3d& satellite_pos) {
    Vector3d los = satellite_pos - receiver_pos;
    Vector3d up = receiver_pos.normalized();
    return std::asin(los.dot(up) / los.norm());
}

double calculateAzimuth(const Vector3d& receiver_pos, const Vector3d& satellite_pos) {
    // Simplified azimuth calculation
    Vector3d los = satellite_pos - receiver_pos;
    return std::atan2(los(1), los(0));
}

GeodeticCoord ecefToGeodetic(const Vector3d& ecef_pos) {
    double lat, lon, h;
    ecef2geodetic(ecef_pos, lat, lon, h);
    return GeodeticCoord(lat, lon, h);
}

Vector3d geodeticToEcef(const GeodeticCoord& geodetic_pos) {
    return geodetic2ecef(geodetic_pos.latitude, geodetic_pos.longitude, geodetic_pos.height);
}

} // namespace spp_utils

} // namespace libgnss
