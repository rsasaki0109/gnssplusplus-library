#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <chrono>
#include <cmath>
#include <limits>

namespace libgnss {

namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;

bool isSPPSystemEnabled(GNSSSystem system, const SPPProcessor::SPPConfig& config) {
    switch (system) {
        case GNSSSystem::GPS:
        case GNSSSystem::Galileo:
        case GNSSSystem::QZSS:
            return true;
        case GNSSSystem::GLONASS:
            return config.enable_glonass;
        case GNSSSystem::BeiDou:
            return config.enable_beidou;
        default:
            return false;
    }
}

bool isPrimarySPPSignal(const Observation& observation,
                        const SPPProcessor::SPPConfig& config) {
    return isSPPSystemEnabled(observation.satellite.system, config) &&
           signal_policy::isPrimarySignal(observation.satellite.system, observation.signal);
}

bool isCandidateSPPSignal(const Observation& observation,
                          const SPPProcessor::SPPConfig& config) {
    if (!isSPPSystemEnabled(observation.satellite.system, config)) {
        return false;
    }
    if (signal_policy::isPrimarySignal(observation.satellite.system, observation.signal)) {
        return true;
    }
    return config.use_ionosphere_free_combination &&
           signal_policy::isSecondarySignal(observation.satellite.system, observation.signal);
}

const Observation* selectBestObservation(const std::vector<Observation>& observations,
                                         bool primary) {
    const Observation* best = nullptr;
    int best_priority = 1000;
    double best_snr = -std::numeric_limits<double>::infinity();
    for (const auto& observation : observations) {
        const GNSSSystem system = observation.satellite.system;
        const bool matches =
            primary ? signal_policy::isPrimarySignal(system, observation.signal)
                    : signal_policy::isSecondarySignal(system, observation.signal);
        if (!matches) {
            continue;
        }
        const int priority = signal_policy::signalPriority(system, observation.signal, primary);
        const double snr = std::isfinite(observation.snr) ? observation.snr : 0.0;
        if (!best || priority < best_priority ||
            (priority == best_priority && snr > best_snr)) {
            best = &observation;
            best_priority = priority;
            best_snr = snr;
        }
    }
    return best;
}

double combinedSnr(double primary_snr, double secondary_snr) {
    const bool primary_valid = std::isfinite(primary_snr) && primary_snr > 0.0;
    const bool secondary_valid = std::isfinite(secondary_snr) && secondary_snr > 0.0;
    if (primary_valid && secondary_valid) {
        return std::min(primary_snr, secondary_snr);
    }
    if (primary_valid) {
        return primary_snr;
    }
    return secondary_valid ? secondary_snr : 0.0;
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

GNSSSystem selectReferenceClockGroup(const std::map<GNSSSystem, int>& group_counts) {
    const auto gps_it = group_counts.find(GNSSSystem::GPS);
    if (gps_it != group_counts.end() && gps_it->second > 0) {
        return GNSSSystem::GPS;
    }

    GNSSSystem best_group = GNSSSystem::UNKNOWN;
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

std::string biasObservationCode(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return "C1C";
        case SignalType::GPS_L1P: return "C1P";
        case SignalType::GPS_L2P: return "C2P";
        case SignalType::GPS_L2C: return "C2W";
        case SignalType::GPS_L5: return "C5Q";
        case SignalType::GLO_L1CA: return "C1C";
        case SignalType::GLO_L1P: return "C1P";
        case SignalType::GLO_L2CA: return "C2C";
        case SignalType::GLO_L2P: return "C2P";
        case SignalType::GAL_E1: return "C1C";
        case SignalType::GAL_E5A: return "C5Q";
        case SignalType::GAL_E5B: return "C7Q";
        case SignalType::GAL_E6: return "C6C";
        case SignalType::BDS_B1I: return "C2I";
        case SignalType::BDS_B2I: return "C7I";
        case SignalType::BDS_B3I: return "C6I";
        case SignalType::BDS_B1C: return "C1C";
        case SignalType::BDS_B2A: return "C5Q";
        case SignalType::QZS_L1CA: return "C1C";
        case SignalType::QZS_L2C: return "C2L";
        case SignalType::QZS_L5: return "C5Q";
        default: return "";
    }
}

double dcbEntryBiasMeters(const DCBEntry& entry) {
    std::string unit = entry.unit;
    std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    if (unit == "NS") {
        return entry.bias * constants::SPEED_OF_LIGHT * 1e-9;
    }
    if (unit == "M" || unit == "METER" || unit == "METERS") {
        return entry.bias;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool findOsbBiasMeters(const DCBProducts& dcb_products,
                       const SatelliteId& satellite,
                       const std::string& observation_code,
                       double& bias_m) {
    for (const auto& entry : dcb_products.entries) {
        if (!entry.valid || !(entry.satellite == satellite) || entry.bias_type != "OSB") {
            continue;
        }
        if (entry.observation_1 != observation_code && entry.observation_2 != observation_code) {
            continue;
        }
        const double converted = dcbEntryBiasMeters(entry);
        if (!std::isfinite(converted)) {
            continue;
        }
        bias_m = converted;
        return true;
    }
    return false;
}

double observationDcbBiasMeters(const DCBProducts& dcb_products,
                                const SatelliteId& satellite,
                                SignalType primary_signal,
                                SignalType secondary_signal,
                                bool use_ionosphere_free,
                                double coeff_primary,
                                double coeff_secondary) {
    const std::string primary_code = biasObservationCode(primary_signal);
    if (primary_code.empty()) {
        return 0.0;
    }

    double primary_bias_m = 0.0;
    const bool have_primary =
        findOsbBiasMeters(dcb_products, satellite, primary_code, primary_bias_m);
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return have_primary ? primary_bias_m : 0.0;
    }

    const std::string secondary_code = biasObservationCode(secondary_signal);
    if (secondary_code.empty()) {
        return have_primary ? coeff_primary * primary_bias_m : 0.0;
    }

    double secondary_bias_m = 0.0;
    const bool have_secondary =
        findOsbBiasMeters(dcb_products, satellite, secondary_code, secondary_bias_m);
    if (!have_primary && !have_secondary) {
        return 0.0;
    }
    return coeff_primary * (have_primary ? primary_bias_m : 0.0) +
           coeff_secondary * (have_secondary ? secondary_bias_m : 0.0);
}

double ssrBiasLookup(const std::map<uint8_t, double>& code_bias_m,
                     GNSSSystem system,
                     uint8_t signal_id) {
    const auto it = code_bias_m.find(signal_id);
    if (it != code_bias_m.end()) {
        return it->second;
    }
    if (system == GNSSSystem::GPS && (signal_id == 8U || signal_id == 9U)) {
        const auto sibling = code_bias_m.find(signal_id == 8U ? 9U : 8U);
        if (sibling != code_bias_m.end()) {
            return sibling->second;
        }
    }
    return 0.0;
}

double observationSsrCodeBiasMeters(GNSSSystem system,
                                    SignalType primary_signal,
                                    SignalType secondary_signal,
                                    bool use_ionosphere_free,
                                    double coeff_primary,
                                    double coeff_secondary,
                                    const std::map<uint8_t, double>& code_bias_m) {
    const uint8_t primary_id = rtcmSsrSignalId(system, primary_signal);
    if (primary_id == 0U) {
        return 0.0;
    }

    const double primary_bias_m = ssrBiasLookup(code_bias_m, system, primary_id);
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return primary_bias_m;
    }

    const uint8_t secondary_id = rtcmSsrSignalId(system, secondary_signal);
    if (secondary_id == 0U) {
        return coeff_primary * primary_bias_m;
    }
    const double secondary_bias_m = ssrBiasLookup(code_bias_m, system, secondary_id);
    return coeff_primary * primary_bias_m + coeff_secondary * secondary_bias_m;
}

bool ionexPiercePointAndMapping(const IONEXProducts& ionex_products,
                                const Vector3d& receiver_position,
                                double azimuth_rad,
                                double elevation_rad,
                                double& ipp_lat_deg,
                                double& ipp_lon_deg,
                                double& mapping_factor) {
    if (elevation_rad <= 0.0) {
        return false;
    }

    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);

    const double earth_radius_m =
        ionex_products.base_radius_km > 0.0
            ? ionex_products.base_radius_km * 1000.0
            : constants::WGS84_A;
    const double shell_height_m =
        !ionex_products.height_grid.empty()
            ? ionex_products.height_grid.front() * 1000.0
            : 450000.0;
    if (earth_radius_m <= 0.0 || shell_height_m < 0.0) {
        return false;
    }

    const double ratio = earth_radius_m / (earth_radius_m + shell_height_m);
    const double cos_elevation = std::cos(elevation_rad);
    const double argument = std::clamp(ratio * cos_elevation, -1.0, 1.0);
    const double psi = M_PI_2 - elevation_rad - std::asin(argument);
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lat = std::cos(latitude_rad);
    const double sin_psi = std::sin(psi);
    const double cos_psi = std::cos(psi);

    const double ipp_lat_rad = std::asin(
        std::clamp(sin_lat * cos_psi + cos_lat * sin_psi * std::cos(azimuth_rad),
                   -1.0,
                   1.0));
    const double ipp_lon_rad = longitude_rad + std::atan2(
        sin_psi * std::sin(azimuth_rad),
        cos_lat * cos_psi - sin_lat * sin_psi * std::cos(azimuth_rad));

    const double mapping_argument = ratio * cos_elevation;
    mapping_factor = 1.0 /
        std::sqrt(std::max(1e-12, 1.0 - mapping_argument * mapping_argument));
    ipp_lat_deg = ipp_lat_rad / kDegreesToRadians;
    ipp_lon_deg = ipp_lon_rad / kDegreesToRadians;
    while (ipp_lon_deg > 180.0) {
        ipp_lon_deg -= 360.0;
    }
    while (ipp_lon_deg < -180.0) {
        ipp_lon_deg += 360.0;
    }
    return std::isfinite(mapping_factor);
}

double ionosphereDelayMetersFromTecu(SignalType signal,
                                     const Ephemeris* eph,
                                     double stec_tecu) {
    const double frequency_hz = signalFrequencyHz(signal, eph);
    if (frequency_hz <= 0.0 || !std::isfinite(stec_tecu)) {
        return 0.0;
    }
    return 40.3e16 * stec_tecu / (frequency_hz * frequency_hz);
}

double elevationMaskRadians(double configured_mask) {
    if (!std::isfinite(configured_mask) || configured_mask <= 0.0) {
        return 0.0;
    }
    // ProcessorConfig documents degrees, but some older examples passed
    // radians. Accept both to avoid surprising existing callers.
    return configured_mask > M_PI / 2.0 ? configured_mask * M_PI / 180.0 : configured_mask;
}

double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const size_t mid = values.size() / 2U;
    if (values.size() % 2U == 1U) {
        return values[mid];
    }
    return 0.5 * (values[mid - 1U] + values[mid]);
}

double residualRms(const VectorXd& residuals) {
    if (residuals.size() == 0) {
        return 0.0;
    }

    double sum_sq = 0.0;
    int count = 0;
    for (int i = 0; i < residuals.size(); ++i) {
        if (!std::isfinite(residuals(i))) {
            continue;
        }
        sum_sq += residuals(i) * residuals(i);
        ++count;
    }
    return count > 0 ? std::sqrt(sum_sq / static_cast<double>(count)) : 0.0;
}

double maxAbsResidual(const VectorXd& residuals) {
    double max_abs = 0.0;
    for (int i = 0; i < residuals.size(); ++i) {
        if (std::isfinite(residuals(i))) {
            max_abs = std::max(max_abs, std::abs(residuals(i)));
        }
    }
    return max_abs;
}

double robustHuberWeightFactor(double residual_m,
                               double sigma_m,
                               double threshold_sigma,
                               double min_factor) {
    if (!std::isfinite(residual_m) ||
        !std::isfinite(sigma_m) ||
        sigma_m <= 0.0 ||
        !std::isfinite(threshold_sigma) ||
        threshold_sigma <= 0.0) {
        return 1.0;
    }
    const double normalized_residual = std::abs(residual_m) / sigma_m;
    if (normalized_residual <= threshold_sigma) {
        return 1.0;
    }
    const double bounded_min =
        std::clamp(std::isfinite(min_factor) ? min_factor : 0.0, 0.0, 1.0);
    return std::clamp(threshold_sigma / normalized_residual, bounded_min, 1.0);
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
    last_applied_precise_orbit_clock_measurements_ = 0;
    last_applied_ssr_orbit_clock_corrections_ = 0;
    last_applied_ssr_code_bias_corrections_ = 0;
    last_applied_ionex_corrections_ = 0;
    last_applied_dcb_corrections_ = 0;
    last_applied_ssr_orbit_m_ = 0.0;
    last_applied_ssr_clock_m_ = 0.0;
    last_applied_ssr_code_bias_m_ = 0.0;
    last_applied_ionex_m_ = 0.0;
    last_applied_dcb_m_ = 0.0;

    try {
        // Use RINEX header position for initialization if available
        if (estimated_position_.norm() < 1000.0 && obs.receiver_position.norm() > 1e6) {
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
        if (solution.isValid()) {
            const double dt = has_last_valid_position_ ? solution.time - last_valid_time_ : 0.0;
            if (spp_config_.enable_position_jump_gate &&
                has_last_valid_position_ &&
                dt > 0.0 &&
                std::isfinite(dt) &&
                spp_config_.max_position_jump_rate_mps > 0.0) {
                const double jump_m = (solution.position_ecef - last_valid_position_).norm();
                const double jump_rate_mps = jump_m / dt;
                const double allowed_jump_m =
                    std::max(spp_config_.max_position_jump_min_m,
                             spp_config_.max_position_jump_rate_mps * dt);
                solution.spp_position_jump_m = jump_m;
                solution.spp_position_jump_rate_mps = jump_rate_mps;
                if (std::isfinite(jump_m) &&
                    std::isfinite(allowed_jump_m) &&
                    allowed_jump_m > 0.0 &&
                    jump_m > allowed_jump_m) {
                    solution.spp_position_jump_gate_rejections = 1;
                    solution.status = SolutionStatus::NONE;
                    estimated_position_ = last_valid_position_;
                    receiver_clock_bias_ = last_valid_clock_bias_;
                }
            }
            if (solution.isValid()) {
                has_last_valid_position_ = true;
                last_valid_position_ = solution.position_ecef;
                last_valid_time_ = solution.time;
                last_valid_clock_bias_ = solution.receiver_clock_bias;
            }
        }
        last_applied_precise_orbit_clock_measurements_ =
            solution.spp_precise_orbit_clock_measurements;
        last_applied_ssr_orbit_clock_corrections_ =
            solution.spp_ssr_orbit_clock_corrections;
        last_applied_ssr_code_bias_corrections_ =
            solution.spp_ssr_code_bias_corrections;
        last_applied_ionex_corrections_ = solution.spp_ionex_corrections;
        last_applied_dcb_corrections_ = solution.spp_dcb_corrections;
        last_applied_ssr_orbit_m_ = solution.spp_ssr_orbit_meters;
        last_applied_ssr_clock_m_ = solution.spp_ssr_clock_meters;
        last_applied_ssr_code_bias_m_ = solution.spp_ssr_code_bias_meters;
        last_applied_ionex_m_ = solution.spp_ionex_meters;
        last_applied_dcb_m_ = solution.spp_dcb_meters;

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
    has_last_valid_position_ = false;
    last_valid_position_.setZero();
    last_valid_time_ = GNSSTime();
    last_valid_clock_bias_ = 0.0;
    last_applied_precise_orbit_clock_measurements_ = 0;
    last_applied_ssr_orbit_clock_corrections_ = 0;
    last_applied_ssr_code_bias_corrections_ = 0;
    last_applied_ionex_corrections_ = 0;
    last_applied_dcb_corrections_ = 0;
    last_applied_ssr_orbit_m_ = 0.0;
    last_applied_ssr_clock_m_ = 0.0;
    last_applied_ssr_code_bias_m_ = 0.0;
    last_applied_ionex_m_ = 0.0;
    last_applied_dcb_m_ = 0.0;

    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_ = 0;
    successful_solutions_ = 0;
    total_processing_time_ms_ = 0.0;
}

bool SPPProcessor::loadPreciseProducts(const std::string& orbit_file,
                                       const std::string& clock_file) {
    precise_products_.clear();

    bool loaded_any = false;
    if (!orbit_file.empty()) {
        loaded_any = precise_products_.loadSP3File(orbit_file) || loaded_any;
    }
    if (!clock_file.empty()) {
        loaded_any = precise_products_.loadClockFile(clock_file) || loaded_any;
    }
    precise_products_loaded_ = loaded_any;
    return precise_products_loaded_;
}

bool SPPProcessor::loadSSRProducts(const std::string& ssr_file) {
    ssr_products_.clear();
    if (ssr_file.empty()) {
        ssr_products_loaded_ = false;
        return false;
    }
    ssr_products_loaded_ = ssr_products_.loadCSVFile(ssr_file);
    return ssr_products_loaded_;
}

bool SPPProcessor::loadIONEXProducts(const std::string& ionex_file) {
    ionex_products_.clear();
    if (ionex_file.empty()) {
        ionex_products_loaded_ = false;
        return false;
    }
    ionex_products_loaded_ = ionex_products_.loadIONEXFile(ionex_file);
    return ionex_products_loaded_;
}

bool SPPProcessor::loadDCBProducts(const std::string& dcb_file) {
    dcb_products_.clear();
    if (dcb_file.empty()) {
        dcb_products_loaded_ = false;
        return false;
    }
    dcb_products_loaded_ = dcb_products_.loadFile(dcb_file);
    return dcb_products_loaded_;
}

PositionSolution SPPProcessor::solvePosition(const std::vector<SPPObservation>& valid_obs,
                                           const NavigationData& nav,
                                           const GNSSTime& time) {
    // Direct call to native least-squares solver
    return solvePositionLS(valid_obs, nav, time);
}

PositionSolution SPPProcessor::solvePositionLS(const std::vector<SPPObservation>& valid_obs,
                                               const NavigationData& nav,
                                               const GNSSTime& time,
                                               bool allow_qc_rejection) {
    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::SPP;

    if (estimated_position_.norm() < 1000.0) {
        initializePosition(valid_obs, nav, time);
    }

    struct MeasurementModel {
        SPPObservation spp_observation;
        Vector3d satellite_position;
        GNSSSystem clock_group = GNSSSystem::UNKNOWN;
        double corrected_pseudorange = 0.0;
        double elevation = 0.0;
        double variance = 1.0;
        double weight = 1.0;
        double ionex_correction_m = 0.0;
        double ssr_orbit_correction_m = 0.0;
        double ssr_clock_correction_m = 0.0;
        double ssr_code_bias_m = 0.0;
        double dcb_correction_m = 0.0;
        bool precise_orbit_clock = false;
        bool ssr_orbit_clock_applied = false;
        bool ssr_code_bias_applied = false;
        bool ionex_applied = false;
        bool dcb_applied = false;
    };

    auto buildMeasurements = [&](const Vector3d& current_position) {
        std::vector<MeasurementModel> measurements;
        measurements.reserve(valid_obs.size());
        const double min_elevation_rad = elevationMaskRadians(config_.elevation_mask);
        const bool apply_atmosphere =
            spp_config_.apply_atmospheric_corrections &&
            (config_.use_ionosphere_model || config_.use_troposphere_model);

        double rcv_lat = 0.0;
        double rcv_lon = 0.0;
        double rcv_h = 0.0;
        ecef2geodetic(current_position, rcv_lat, rcv_lon, rcv_h);

        for (const auto& spp_obs : valid_obs) {
            const Observation& obs = spp_obs.observation;
            Vector3d sat_pos;
            Vector3d sat_vel;
            double sat_clk = 0.0;
            double sat_clk_drift = 0.0;
            bool precise_orbit_clock = false;
            bool ssr_orbit_clock_applied = false;
            bool ssr_code_bias_applied = false;
            Vector3d ssr_orbit_correction = Vector3d::Zero();
            double ssr_clock_correction_m = 0.0;
            double ssr_code_bias_m = 0.0;
            int ssr_orbit_iode = -1;
            std::map<uint8_t, double> ssr_code_biases;
            const bool ssr_enabled =
                spp_config_.use_ssr_corrections && ssr_products_loaded_;
            const bool ssr_ok = ssr_enabled &&
                ssr_products_.interpolateCorrection(
                    obs.satellite,
                    time,
                    ssr_orbit_correction,
                    ssr_clock_correction_m,
                    nullptr,
                    &ssr_code_biases,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    0,
                    &ssr_orbit_iode);

            double travel_time = obs.pseudorange / constants::SPEED_OF_LIGHT;
            GNSSTime tx_time = time - travel_time;
            if (spp_config_.use_precise_products && precise_products_loaded_) {
                precise_orbit_clock = precise_products_.interpolateOrbitClock(
                    obs.satellite,
                    time,
                    sat_pos,
                    sat_vel,
                    sat_clk,
                    sat_clk_drift);
                if (precise_orbit_clock) {
                    const double initial_distance = (sat_pos - current_position).norm();
                    if (std::isfinite(initial_distance) && initial_distance > 0.0) {
                        travel_time = initial_distance / constants::SPEED_OF_LIGHT;
                        sat_pos -= sat_vel * travel_time;
                        sat_clk -= sat_clk_drift * travel_time;
                        tx_time = time - travel_time;
                    } else {
                        precise_orbit_clock = false;
                    }
                }
            }

            if (!precise_orbit_clock) {
                if (!nav.calculateSatelliteState(obs.satellite, tx_time,
                                                 sat_pos, sat_vel, sat_clk, sat_clk_drift,
                                                 ssr_orbit_iode)) {
                    continue;
                }

                tx_time = tx_time - sat_clk;
                if (!nav.calculateSatelliteState(obs.satellite, tx_time,
                                                 sat_pos, sat_vel, sat_clk, sat_clk_drift,
                                                 ssr_orbit_iode)) {
                    continue;
                }
            }

            if (ssr_ok) {
                if (!precise_orbit_clock) {
                    Vector3d orbit_correction_ecef = ssr_orbit_correction;
                    if (ssr_products_.orbitCorrectionsAreRac()) {
                        orbit_correction_ecef =
                            ppp_utils::ssrRacToEcef(sat_pos, sat_vel, ssr_orbit_correction);
                    }
                    if (orbit_correction_ecef.allFinite() &&
                        std::isfinite(ssr_clock_correction_m)) {
                        sat_pos += orbit_correction_ecef;
                        sat_clk += ssr_clock_correction_m / constants::SPEED_OF_LIGHT;
                        ssr_orbit_correction = orbit_correction_ecef;
                        ssr_orbit_clock_applied =
                            orbit_correction_ecef.norm() > 0.0 ||
                            std::abs(ssr_clock_correction_m) > 0.0;
                    }
                }
                ssr_code_bias_m = observationSsrCodeBiasMeters(
                    obs.satellite.system,
                    spp_obs.primary_signal,
                    spp_obs.secondary_signal,
                    spp_obs.ionosphere_free,
                    spp_obs.primary_coeff,
                    spp_obs.secondary_coeff,
                    ssr_code_biases);
                ssr_code_bias_applied =
                    std::isfinite(ssr_code_bias_m) && std::abs(ssr_code_bias_m) > 0.0;
                if (!std::isfinite(ssr_code_bias_m)) {
                    ssr_code_bias_m = 0.0;
                    ssr_code_bias_applied = false;
                }
            }

            double signal_travel = (sat_pos - current_position).norm() / constants::SPEED_OF_LIGHT;
            double angle = constants::OMEGA_E * signal_travel;
            Eigen::Matrix3d earth_rotation;
            earth_rotation << std::cos(angle),  std::sin(angle), 0.0,
                             -std::sin(angle),  std::cos(angle), 0.0,
                              0.0,              0.0,             1.0;
            Vector3d corrected_sat_pos = earth_rotation * sat_pos;

            auto geom = nav.calculateGeometry(current_position, corrected_sat_pos);
            if (geom.elevation < min_elevation_rad) {
                continue;
            }

            double trop_delay = 0.0;
            const bool trop_corrected = apply_atmosphere && config_.use_troposphere_model;
            if (trop_corrected) {
                trop_delay = models::tropDelaySaastamoinen(current_position, geom.elevation);
            }

            const Ephemeris* eph = nav.getEphemeris(obs.satellite, tx_time);
            if (!eph) {
                continue;
            }

            double iono_delay = 0.0;
            bool iono_corrected = false;
            bool ionex_applied = false;
            if (spp_obs.ionosphere_free) {
                iono_corrected = true;
            } else if (apply_atmosphere && config_.use_ionosphere_model &&
                       spp_config_.use_ionex_corrections &&
                       ionex_products_loaded_) {
                double ipp_lat_deg = 0.0;
                double ipp_lon_deg = 0.0;
                double mapping_factor = 0.0;
                double vertical_tecu = 0.0;
                if (ionexPiercePointAndMapping(
                        ionex_products_,
                        current_position,
                        geom.azimuth,
                        geom.elevation,
                        ipp_lat_deg,
                        ipp_lon_deg,
                        mapping_factor) &&
                    ionex_products_.interpolateTecu(
                        time, ipp_lat_deg, ipp_lon_deg, vertical_tecu, nullptr)) {
                    iono_delay = ionosphereDelayMetersFromTecu(
                        obs.signal, eph, mapping_factor * vertical_tecu);
                    if (std::isfinite(iono_delay) && std::abs(iono_delay) > 0.0) {
                        iono_corrected = true;
                        ionex_applied = true;
                    } else {
                        iono_delay = 0.0;
                    }
                }
            }
            if (!iono_corrected &&
                apply_atmosphere &&
                config_.use_ionosphere_model &&
                nav.ionosphere_model.valid) {
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
                iono_corrected = true;
            }

            double dcb_bias_m = 0.0;
            if (!ssr_code_bias_applied &&
                spp_config_.use_dcb_corrections &&
                dcb_products_loaded_) {
                dcb_bias_m = observationDcbBiasMeters(
                    dcb_products_,
                    obs.satellite,
                    spp_obs.primary_signal,
                    spp_obs.secondary_signal,
                    spp_obs.ionosphere_free,
                    spp_obs.primary_coeff,
                    spp_obs.secondary_coeff);
                if (!std::isfinite(dcb_bias_m)) {
                    dcb_bias_m = 0.0;
                }
            }

            double group_delay = groupDelayCorrectionMeters(obs, *eph);
            if (spp_obs.ionosphere_free) {
                Observation primary_obs = obs;
                primary_obs.signal = spp_obs.primary_signal;
                Observation secondary_obs = obs;
                secondary_obs.signal = spp_obs.secondary_signal;
                group_delay =
                    spp_obs.primary_coeff * groupDelayCorrectionMeters(primary_obs, *eph) +
                    spp_obs.secondary_coeff * groupDelayCorrectionMeters(secondary_obs, *eph);
            }

            double corrected_pr = obs.pseudorange
                                  + sat_clk * constants::SPEED_OF_LIGHT
                                  - trop_delay
                                  - iono_delay
                                  - group_delay
                                  - ssr_code_bias_m
                                  - dcb_bias_m;

            double sin_el = std::sin(geom.elevation);
            if (sin_el < 0.1) {
                sin_el = 0.1;
            }

            double variance = spp_config_.pseudorange_sigma * spp_config_.pseudorange_sigma /
                              (sin_el * sin_el);
            if (spp_config_.use_variance_model) {
                spp_utils::MeasurementVarianceInputs variance_inputs;
                variance_inputs.elevation_rad = geom.elevation;
                variance_inputs.snr_dbhz = obs.snr;
                variance_inputs.pseudorange_sigma_m = spp_config_.pseudorange_sigma;
                variance_inputs.ionosphere_delay_m = iono_delay;
                variance_inputs.troposphere_delay_m = trop_delay;
                variance_inputs.ionosphere_corrected = iono_corrected;
                variance_inputs.troposphere_corrected = trop_corrected;
                variance_inputs.snr_reference_dbhz = spp_config_.snr_reference_dbhz;
                variance_inputs.ionosphere_sigma_scale = spp_config_.ionosphere_sigma_scale;
                variance_inputs.troposphere_sigma_scale = spp_config_.troposphere_sigma_scale;
                variance = spp_utils::calculatePseudorangeVariance(variance_inputs);
            }
            variance *= std::max(1.0, spp_obs.variance_scale);
            if (!std::isfinite(variance) || variance <= 0.0) {
                continue;
            }

            MeasurementModel measurement;
            measurement.spp_observation = spp_obs;
            measurement.satellite_position = corrected_sat_pos;
            measurement.clock_group = clockBiasGroup(obs.satellite.system);
            measurement.corrected_pseudorange = corrected_pr;
            measurement.elevation = geom.elevation;
            measurement.variance = variance;
            measurement.weight = 1.0 / variance;
            measurement.precise_orbit_clock = precise_orbit_clock;
            measurement.ssr_orbit_correction_m =
                ssr_orbit_clock_applied ? ssr_orbit_correction.norm() : 0.0;
            measurement.ssr_clock_correction_m =
                ssr_orbit_clock_applied ? ssr_clock_correction_m : 0.0;
            measurement.ssr_code_bias_m = ssr_code_bias_m;
            measurement.ssr_orbit_clock_applied = ssr_orbit_clock_applied;
            measurement.ssr_code_bias_applied = ssr_code_bias_applied;
            measurement.ionex_correction_m =
                ionex_applied ? iono_delay : 0.0;
            measurement.ionex_applied =
                ionex_applied &&
                std::abs(measurement.ionex_correction_m) > 0.0;
            measurement.dcb_correction_m = dcb_bias_m;
            measurement.dcb_applied =
                !ssr_code_bias_applied &&
                spp_config_.use_dcb_corrections &&
                std::isfinite(dcb_bias_m) &&
                std::abs(dcb_bias_m) > 0.0;
            if (measurement.clock_group == GNSSSystem::UNKNOWN) {
                continue;
            }
            measurements.push_back(measurement);
        }

        return measurements;
    };

    Vector3d position = estimated_position_;
    double clock_bias = receiver_clock_bias_;
    std::map<GNSSSystem, double> bias_estimates;
    std::vector<MeasurementModel> final_measurements;
    GNSSSystem final_reference_group = GNSSSystem::UNKNOWN;
    std::vector<GNSSSystem> final_bias_groups;
    int robust_weighted_measurements = 0;
    double min_robust_weight_factor = 1.0;
    int adaptive_robust_activations = 0;
    int adaptive_robust_tail_measurements = 0;

    for (int iter = 0; iter < spp_config_.max_iterations; ++iter) {
        auto measurements = buildMeasurements(position);
        if (measurements.size() < 4) {
            solution.status = SolutionStatus::NONE;
            return solution;
        }

        std::map<GNSSSystem, int> group_counts;
        for (const auto& measurement : measurements) {
            group_counts[measurement.clock_group]++;
        }

        const GNSSSystem reference_group = selectReferenceClockGroup(group_counts);
        std::vector<GNSSSystem> bias_groups;
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

        std::map<GNSSSystem, int> bias_columns;
        for (int i = 0; i < static_cast<int>(bias_groups.size()); ++i) {
            bias_columns[bias_groups[i]] = 4 + i;
        }

        MatrixXd H = MatrixXd::Zero(n, num_unknowns);
        VectorXd residuals = VectorXd::Zero(n);
        MatrixXd weighted_H = MatrixXd::Zero(n, num_unknowns);
        VectorXd weighted_residuals = VectorXd::Zero(n);
        int iteration_robust_weighted_measurements = 0;
        double iteration_min_robust_weight_factor = 1.0;
        int iteration_tail_measurements = 0;

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
        }

        bool robust_weighting_active = spp_config_.enable_robust_weighting && iter > 0;
        if (robust_weighting_active &&
            spp_config_.enable_adaptive_robust_weighting) {
            const double activation_threshold =
                std::max(spp_config_.robust_weight_threshold_sigma,
                         spp_config_.adaptive_robust_activation_threshold_sigma);
            for (int i = 0; i < n; ++i) {
                const double sigma_m = std::sqrt(measurements[i].variance);
                if (!std::isfinite(residuals(i)) ||
                    !std::isfinite(sigma_m) ||
                    sigma_m <= 0.0) {
                    continue;
                }
                const double normalized_residual = std::abs(residuals(i)) / sigma_m;
                if (normalized_residual > activation_threshold) {
                    ++iteration_tail_measurements;
                }
            }
            const int min_tail_count =
                std::max(1, spp_config_.adaptive_robust_min_tail_measurements);
            const double min_tail_fraction =
                std::clamp(spp_config_.adaptive_robust_min_tail_fraction, 0.0, 1.0);
            const double tail_fraction =
                static_cast<double>(iteration_tail_measurements) /
                static_cast<double>(std::max(1, n));
            robust_weighting_active =
                iteration_tail_measurements >= min_tail_count &&
                tail_fraction >= min_tail_fraction;
        }
        if (robust_weighting_active &&
            spp_config_.enable_adaptive_robust_weighting) {
            ++adaptive_robust_activations;
            adaptive_robust_tail_measurements += iteration_tail_measurements;
        }

        for (int i = 0; i < n; ++i) {
            const auto& measurement = measurements[i];
            double robust_factor = 1.0;
            if (robust_weighting_active) {
                robust_factor = robustHuberWeightFactor(
                    residuals(i),
                    std::sqrt(measurement.variance),
                    spp_config_.robust_weight_threshold_sigma,
                    spp_config_.robust_weight_min_factor);
                if (robust_factor < 0.999999) {
                    ++iteration_robust_weighted_measurements;
                    iteration_min_robust_weight_factor =
                        std::min(iteration_min_robust_weight_factor, robust_factor);
                }
            }

            const double sqrt_weight = std::sqrt(measurement.weight * robust_factor);
            weighted_H.row(i) = H.row(i) * sqrt_weight;
            weighted_residuals(i) = residuals(i) * sqrt_weight;
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

        position += dx.head<3>();
        clock_bias += dx(3);
        for (int i = 0; i < static_cast<int>(bias_groups.size()); ++i) {
            bias_estimates[bias_groups[i]] += dx(4 + i);
        }

        final_measurements = std::move(measurements);
        final_reference_group = reference_group;
        final_bias_groups = bias_groups;
        robust_weighted_measurements = iteration_robust_weighted_measurements;
        min_robust_weight_factor =
            iteration_robust_weighted_measurements > 0
                ? iteration_min_robust_weight_factor
                : 1.0;

        if (dx.head<3>().norm() < spp_config_.position_convergence_threshold) {
            solution.iterations = iter + 1;
            break;
        }
        if (iter == spp_config_.max_iterations - 1) {
            solution.iterations = spp_config_.max_iterations;
        }
    }

    final_measurements = buildMeasurements(position);
    if (final_measurements.size() < 4) {
        solution.status = SolutionStatus::NONE;
        return solution;
    }

    std::map<GNSSSystem, int> final_group_counts;
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

    std::map<GNSSSystem, int> final_bias_columns;
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

    const int final_num_unknowns = 4 + static_cast<int>(final_bias_groups.size());
    const int final_degrees_of_freedom =
        static_cast<int>(final_measurements.size()) - final_num_unknowns;
    const double final_residual_rms = residualRms(final_residuals);
    const double final_max_abs_residual = maxAbsResidual(final_residuals);
    double final_chi_square = 0.0;
    for (int i = 0; i < static_cast<int>(final_measurements.size()); ++i) {
        final_chi_square += final_residuals(i) * final_residuals(i) /
                            final_measurements[i].variance;
    }
    const double final_chi_square_per_dof =
        final_degrees_of_freedom > 0
            ? final_chi_square / static_cast<double>(final_degrees_of_freedom)
            : 0.0;

    std::vector<SPPObservation> final_observations;
    final_observations.reserve(final_measurements.size());
    for (const auto& measurement : final_measurements) {
        final_observations.push_back(measurement.spp_observation);
    }

    int raim_attempts = 0;
    if (allow_qc_rejection &&
        spp_config_.enable_outlier_detection &&
        final_measurements.size() > 4U + final_bias_groups.size()) {
        const auto inlier_observations =
            detectOutliers(final_observations, final_residuals, spp_config_.outlier_threshold_sigma);
        if (inlier_observations.size() < final_observations.size() &&
            inlier_observations.size() >= 4U) {
            auto filtered_solution = solvePositionLS(inlier_observations, nav, time, false);
            if (filtered_solution.isValid()) {
                filtered_solution.spp_pre_qc_measurements =
                    static_cast<int>(final_observations.size());
                filtered_solution.spp_pre_qc_residual_rms_m = final_residual_rms;
                filtered_solution.spp_pre_qc_max_abs_residual_m = final_max_abs_residual;
                filtered_solution.spp_outlier_rejections +=
                    static_cast<int>(final_observations.size() - inlier_observations.size());
                for (const auto& observation : final_observations) {
                    const auto inlier_it = std::find_if(
                        inlier_observations.begin(),
                        inlier_observations.end(),
                        [&](const SPPObservation& inlier) {
                            return inlier.observation.satellite ==
                                       observation.observation.satellite &&
                                   inlier.observation.signal ==
                                       observation.observation.signal &&
                                   inlier.ionosphere_free == observation.ionosphere_free;
                        });
                    if (inlier_it == inlier_observations.end()) {
                        filtered_solution.spp_rejected_satellites.push_back(
                            observation.observation.satellite);
                    }
                }
                return filtered_solution;
            }
        }
    }

    if (allow_qc_rejection &&
        spp_config_.enable_raim_fde &&
        final_degrees_of_freedom >= 2 &&
        std::isfinite(final_residual_rms) &&
        final_residual_rms > 0.0) {
        std::vector<double> candidate_rms(final_observations.size(),
                                          std::numeric_limits<double>::infinity());
        std::vector<PositionSolution> candidate_solutions(final_observations.size());

        const Vector3d saved_position = estimated_position_;
        const double saved_clock_bias = receiver_clock_bias_;
        const auto saved_system_biases = system_biases_;

        for (size_t excluded = 0; excluded < final_observations.size(); ++excluded) {
            std::vector<SPPObservation> candidate_observations;
            candidate_observations.reserve(final_observations.size() - 1U);
            for (size_t i = 0; i < final_observations.size(); ++i) {
                if (i != excluded) {
                    candidate_observations.push_back(final_observations[i]);
                }
            }
            if (candidate_observations.size() < 4U) {
                continue;
            }

            estimated_position_ = position;
            receiver_clock_bias_ = clock_bias;
            system_biases_ = saved_system_biases;
            auto candidate_solution = solvePositionLS(candidate_observations, nav, time, false);
            ++raim_attempts;
            if (candidate_solution.isValid() &&
                std::isfinite(candidate_solution.residual_rms)) {
                candidate_rms[excluded] = candidate_solution.residual_rms;
                candidate_solutions[excluded] = candidate_solution;
            }
        }

        estimated_position_ = saved_position;
        receiver_clock_bias_ = saved_clock_bias;
        system_biases_ = saved_system_biases;

        const int selected = spp_utils::selectRaimFdeCandidate(
            final_residual_rms,
            candidate_rms,
            spp_config_.raim_fde_min_rms_improvement_ratio,
            spp_config_.raim_fde_min_rms_improvement_m);
        if (selected >= 0) {
            auto fde_solution = candidate_solutions[static_cast<size_t>(selected)];
            fde_solution.spp_pre_qc_measurements =
                static_cast<int>(final_observations.size());
            fde_solution.spp_pre_qc_residual_rms_m = final_residual_rms;
            fde_solution.spp_pre_qc_max_abs_residual_m = final_max_abs_residual;
            fde_solution.spp_raim_fde_attempts += raim_attempts;
            fde_solution.spp_raim_fde_rejections += 1;
            fde_solution.spp_rejected_satellites.push_back(
                final_observations[static_cast<size_t>(selected)].observation.satellite);
            estimated_position_ = fde_solution.position_ecef;
            receiver_clock_bias_ = fde_solution.receiver_clock_bias;
            system_biases_.clear();
            return fde_solution;
        }
    }

    solution.position_ecef = position;
    solution.receiver_clock_bias = clock_bias;
    solution.num_satellites = static_cast<int>(final_measurements.size());
    int ionosphere_free_measurements = 0;
    int precise_orbit_clock_measurements = 0;
    int ssr_orbit_clock_corrections = 0;
    int ssr_code_bias_corrections = 0;
    int ionex_corrections = 0;
    int dcb_corrections = 0;
    double ssr_orbit_meters = 0.0;
    double ssr_clock_meters = 0.0;
    double ssr_code_bias_meters = 0.0;
    double ionex_meters = 0.0;
    double dcb_meters = 0.0;
    for (const auto& measurement : final_measurements) {
        if (measurement.spp_observation.ionosphere_free) {
            ++ionosphere_free_measurements;
        }
        if (measurement.precise_orbit_clock) {
            ++precise_orbit_clock_measurements;
        }
        if (measurement.ssr_orbit_clock_applied) {
            ++ssr_orbit_clock_corrections;
            ssr_orbit_meters += std::abs(measurement.ssr_orbit_correction_m);
            ssr_clock_meters += std::abs(measurement.ssr_clock_correction_m);
        }
        if (measurement.ssr_code_bias_applied) {
            ++ssr_code_bias_corrections;
            ssr_code_bias_meters += std::abs(measurement.ssr_code_bias_m);
        }
        if (measurement.ionex_applied) {
            ++ionex_corrections;
            ionex_meters += std::abs(measurement.ionex_correction_m);
        }
        if (measurement.dcb_applied) {
            ++dcb_corrections;
            dcb_meters += std::abs(measurement.dcb_correction_m);
        }
    }
    solution.num_frequencies = ionosphere_free_measurements > 0 ? 2 : 1;
    solution.residual_rms = final_residual_rms;
    solution.spp_pre_qc_measurements = static_cast<int>(final_measurements.size());
    solution.spp_pre_qc_residual_rms_m = final_residual_rms;
    solution.spp_pre_qc_max_abs_residual_m = final_max_abs_residual;
    solution.spp_max_abs_residual_m = final_max_abs_residual;
    solution.spp_chi_square = final_chi_square;
    solution.spp_chi_square_per_dof = final_chi_square_per_dof;
    solution.spp_degrees_of_freedom = final_degrees_of_freedom;
    solution.spp_robust_weighted_measurements = robust_weighted_measurements;
    solution.spp_min_robust_weight_factor = min_robust_weight_factor;
    solution.spp_adaptive_robust_activations = adaptive_robust_activations;
    solution.spp_adaptive_robust_tail_measurements = adaptive_robust_tail_measurements;
    solution.spp_ionosphere_free_measurements = ionosphere_free_measurements;
    solution.spp_precise_orbit_clock_measurements = precise_orbit_clock_measurements;
    solution.spp_ssr_orbit_clock_corrections = ssr_orbit_clock_corrections;
    solution.spp_ssr_code_bias_corrections = ssr_code_bias_corrections;
    solution.spp_ionex_corrections = ionex_corrections;
    solution.spp_dcb_corrections = dcb_corrections;
    solution.spp_ssr_orbit_meters = ssr_orbit_meters;
    solution.spp_ssr_clock_meters = ssr_clock_meters;
    solution.spp_ssr_code_bias_meters = ssr_code_bias_meters;
    solution.spp_ionex_meters = ionex_meters;
    solution.spp_dcb_meters = dcb_meters;
    solution.spp_raim_fde_attempts = raim_attempts;

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
        solution.satellites_used.push_back(
            final_measurements[i].spp_observation.observation.satellite);
        solution.satellite_elevations.push_back(final_measurements[i].elevation);
        solution.satellite_residuals.push_back(final_residuals(i));
    }

    if (spp_config_.max_gdop > 0.0 &&
        (!std::isfinite(solution.gdop) || solution.gdop > spp_config_.max_gdop)) {
        solution.spp_gdop_gate_rejections = 1;
        solution.status = SolutionStatus::NONE;
        return solution;
    }
    if (spp_config_.max_residual_rms > 0.0 &&
        std::isfinite(solution.residual_rms) &&
        solution.residual_rms > spp_config_.max_residual_rms) {
        solution.spp_residual_gate_rejections = 1;
        solution.status = SolutionStatus::NONE;
        return solution;
    }
    if (spp_config_.max_chi_square_per_dof > 0.0 &&
        final_degrees_of_freedom > 0 &&
        std::isfinite(solution.spp_chi_square_per_dof) &&
        solution.spp_chi_square_per_dof > spp_config_.max_chi_square_per_dof) {
        solution.spp_chi_square_gate_rejections = 1;
        solution.status = SolutionStatus::NONE;
        return solution;
    }

    estimated_position_ = position;
    receiver_clock_bias_ = clock_bias;
    system_biases_.clear();
    return solution;
}

std::vector<SPPProcessor::SPPObservation> SPPProcessor::detectOutliers(
    const std::vector<SPPObservation>& observations,
    const VectorXd& residuals,
    double threshold_sigma) const {
    if (observations.size() != static_cast<size_t>(residuals.size()) ||
        observations.size() <= 4U ||
        !std::isfinite(threshold_sigma) ||
        threshold_sigma <= 0.0) {
        return observations;
    }

    std::vector<double> residual_values;
    residual_values.reserve(observations.size());
    for (int i = 0; i < residuals.size(); ++i) {
        residual_values.push_back(residuals(i));
    }
    const auto inlier_mask = spp_utils::calculateResidualInlierMask(
        residual_values, threshold_sigma, std::max(1.0, spp_config_.pseudorange_sigma));

    std::vector<SPPObservation> inliers;
    inliers.reserve(observations.size());
    for (size_t i = 0; i < observations.size(); ++i) {
        if (i < inlier_mask.size() && inlier_mask[i]) {
            inliers.push_back(observations[i]);
        }
    }

    return inliers.size() >= 4U ? inliers : observations;
}

std::vector<SPPProcessor::SPPObservation> SPPProcessor::validateObservations(
    const ObservationData& obs,
    const NavigationData& nav,
    const GNSSTime& time) const {
    std::vector<SPPObservation> valid_obs;
    std::map<SatelliteId, std::vector<Observation>> observations_by_satellite;

    for (const auto& observation : obs.observations) {
        if (!spp_config_.use_multi_constellation &&
            observation.satellite.system != GNSSSystem::GPS) {
            continue;
        }

        if (spp_config_.use_ionosphere_free_combination) {
            if (!isCandidateSPPSignal(observation, spp_config_)) {
                continue;
            }
        } else if (!isPrimarySPPSignal(observation, spp_config_)) {
            continue;
        }

        // Check observation validity first
        if (!observation.valid || !observation.has_pseudorange || observation.pseudorange <= 0.0 ||
            !std::isfinite(observation.pseudorange)) {
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

        // Exclude satellites broadcasting an unhealthy status, mirroring RTKLIB
        // satexclude(). QZSS masks its LEX (L6) signal-health bit (bit 0), which
        // does not affect L1/L2/L5 positioning; any other set health bit, or any
        // nonzero health on the remaining systems, excludes the satellite. The
        // MIZU BRDC nav carries unhealthy R11/R26/R27, E14/E18 and several BDS
        // GEO/IGSO; including them drove the SPP solution kilometers off.
        int sv_health = static_cast<int>(eph->health);
        if (observation.satellite.system == GNSSSystem::QZSS) {
            sv_health &= 0xFE;
        }
        if (sv_health != 0) {
            continue;
        }

        // Check SNR threshold
        if (observation.snr < config_.snr_mask) {
            continue;
        }

        if (!spp_config_.use_ionosphere_free_combination) {
            SPPObservation entry;
            entry.observation = observation;
            entry.primary_signal = observation.signal;
            valid_obs.push_back(entry);
            continue;
        }

        observations_by_satellite[observation.satellite].push_back(observation);
    }

    if (!spp_config_.use_ionosphere_free_combination) {
        return valid_obs;
    }

    for (const auto& [satellite, satellite_observations] : observations_by_satellite) {
        const Observation* primary = selectBestObservation(satellite_observations, true);
        if (primary == nullptr) {
            continue;
        }

        const Observation* secondary = selectBestObservation(satellite_observations, false);
        if (secondary != nullptr) {
            const double travel_time = primary->pseudorange / constants::SPEED_OF_LIGHT;
            const GNSSTime tx_time = time - travel_time;
            const Ephemeris* eph = nav.getEphemeris(satellite, tx_time);
            const double f1 = signalFrequencyHz(primary->signal, eph);
            const double f2 = signalFrequencyHz(secondary->signal, eph);
            if (f1 > 0.0 && f2 > 0.0 && std::abs(f1 - f2) >= 1.0) {
                const auto coefficients = spp_utils::ionosphereFreeCoefficients(f1, f2);
                SPPObservation entry;
                entry.observation = *primary;
                entry.observation.pseudorange = spp_utils::calculateIonosphereFreePseudorange(
                    primary->pseudorange, secondary->pseudorange, f1, f2);
                entry.observation.snr = combinedSnr(primary->snr, secondary->snr);
                entry.ionosphere_free = true;
                entry.primary_signal = primary->signal;
                entry.secondary_signal = secondary->signal;
                entry.primary_coeff = coefficients.first;
                entry.secondary_coeff = coefficients.second;
                entry.variance_scale =
                    coefficients.first * coefficients.first +
                    coefficients.second * coefficients.second;
                valid_obs.push_back(entry);
                continue;
            }
        }

        SPPObservation entry;
        entry.observation = *primary;
        entry.primary_signal = primary->signal;
        valid_obs.push_back(entry);
    }

    return valid_obs;
}

bool SPPProcessor::initializePosition(const std::vector<SPPObservation>& observations,
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
    const std::vector<SPPObservation>& observations,
    const NavigationData& nav,
    const GNSSTime& time) const {

    std::map<SatelliteId, SatelliteState> states;

    for (const auto& spp_obs : observations) {
        const Observation& obs = spp_obs.observation;
        SatelliteState state;

        // Estimate signal travel time and calculate transmission time
        double travel_time = obs.pseudorange / constants::SPEED_OF_LIGHT;
        GNSSTime tx_time = time - travel_time;
        int ssr_orbit_iode = -1;
        Vector3d ssr_orbit_correction = Vector3d::Zero();
        double ssr_clock_correction_m = 0.0;
        const bool ssr_enabled =
            spp_config_.use_ssr_corrections && ssr_products_loaded_;
        const bool ssr_ok = ssr_enabled &&
            ssr_products_.interpolateCorrection(
                obs.satellite,
                time,
                ssr_orbit_correction,
                ssr_clock_correction_m,
                nullptr,
                nullptr,
                nullptr,
                nullptr,
                nullptr,
                nullptr,
                nullptr,
                0,
                &ssr_orbit_iode);

        if (spp_config_.use_precise_products &&
            precise_products_loaded_ &&
            precise_products_.interpolateOrbitClock(obs.satellite,
                                                     time,
                                                     state.position,
                                                     state.velocity,
                                                     state.clock_bias,
                                                     state.clock_drift)) {
            state.valid = true;
            state.precise_orbit_clock = true;
            states[obs.satellite] = state;
            continue;
        }

        // First: get satellite clock at approximate tx time
        state.valid = nav.calculateSatelliteState(obs.satellite, tx_time,
                                                state.position, state.velocity,
                                                state.clock_bias, state.clock_drift,
                                                ssr_orbit_iode);
        if (state.valid) {
            // Correct tx time for satellite clock bias
            tx_time = tx_time - state.clock_bias;
            // Recompute at corrected tx time
            state.valid = nav.calculateSatelliteState(obs.satellite, tx_time,
                                                    state.position, state.velocity,
                                                    state.clock_bias, state.clock_drift,
                                                    ssr_orbit_iode);
            if (state.valid && ssr_ok) {
                if (ssr_products_.orbitCorrectionsAreRac()) {
                    ssr_orbit_correction =
                        ppp_utils::ssrRacToEcef(
                            state.position, state.velocity, ssr_orbit_correction);
                }
                if (ssr_orbit_correction.allFinite() &&
                    std::isfinite(ssr_clock_correction_m)) {
                    state.position += ssr_orbit_correction;
                    state.clock_bias += ssr_clock_correction_m /
                                        constants::SPEED_OF_LIGHT;
                    state.ssr_orbit_clock =
                        ssr_orbit_correction.norm() > 0.0 ||
                        std::abs(ssr_clock_correction_m) > 0.0;
                }
            }
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

double calculatePseudorangeVariance(const MeasurementVarianceInputs& inputs) {
    const double base_sigma = std::max(0.1, inputs.pseudorange_sigma_m);
    double sin_el = std::sin(inputs.elevation_rad);
    if (!std::isfinite(sin_el) || sin_el < 0.1) {
        sin_el = 0.1;
    }

    double code_sigma = base_sigma / sin_el;
    const double snr_reference =
        std::isfinite(inputs.snr_reference_dbhz) && inputs.snr_reference_dbhz > 0.0
            ? inputs.snr_reference_dbhz
            : 45.0;
    if (std::isfinite(inputs.snr_dbhz) &&
        inputs.snr_dbhz > 0.0 &&
        inputs.snr_dbhz < snr_reference) {
        const double snr_scale =
            std::pow(10.0, (snr_reference - inputs.snr_dbhz) / 20.0);
        code_sigma *= std::clamp(snr_scale, 1.0, 4.0);
    }

    double variance = code_sigma * code_sigma;

    if (inputs.ionosphere_corrected) {
        const double scale =
            std::isfinite(inputs.ionosphere_sigma_scale) && inputs.ionosphere_sigma_scale >= 0.0
                ? inputs.ionosphere_sigma_scale
                : 0.5;
        const double iono_sigma =
            std::clamp(std::abs(inputs.ionosphere_delay_m) * scale, 0.25, 8.0);
        variance += iono_sigma * iono_sigma;
    } else {
        const double iono_sigma = 2.5 / sin_el;
        variance += iono_sigma * iono_sigma;
    }

    if (inputs.troposphere_corrected) {
        const double scale =
            std::isfinite(inputs.troposphere_sigma_scale) && inputs.troposphere_sigma_scale >= 0.0
                ? inputs.troposphere_sigma_scale
                : 0.1;
        const double trop_sigma =
            std::clamp(std::abs(inputs.troposphere_delay_m) * scale, 0.05, 3.0);
        variance += trop_sigma * trop_sigma;
    } else {
        const double trop_sigma = 0.5 / sin_el;
        variance += trop_sigma * trop_sigma;
    }

    return std::isfinite(variance) && variance > 0.0 ? variance : base_sigma * base_sigma;
}

std::pair<double, double> ionosphereFreeCoefficients(double f1_hz, double f2_hz) {
    const double denom = f1_hz * f1_hz - f2_hz * f2_hz;
    if (!std::isfinite(denom) || std::abs(denom) < 1.0) {
        return {1.0, 0.0};
    }
    return {
        (f1_hz * f1_hz) / denom,
        -(f2_hz * f2_hz) / denom,
    };
}

double calculateIonosphereFreePseudorange(double p1_m,
                                          double p2_m,
                                          double f1_hz,
                                          double f2_hz) {
    const auto coefficients = ionosphereFreeCoefficients(f1_hz, f2_hz);
    return coefficients.first * p1_m + coefficients.second * p2_m;
}

std::vector<bool> calculateResidualInlierMask(const std::vector<double>& residuals,
                                              double threshold_sigma,
                                              double sigma_floor) {
    std::vector<bool> inlier_mask(residuals.size(), true);
    if (residuals.size() <= 4U ||
        !std::isfinite(threshold_sigma) ||
        threshold_sigma <= 0.0) {
        return inlier_mask;
    }

    if (!std::isfinite(sigma_floor) || sigma_floor <= 0.0) {
        sigma_floor = 1.0;
    }

    std::vector<double> finite_residuals;
    finite_residuals.reserve(residuals.size());
    for (double residual : residuals) {
        if (std::isfinite(residual)) {
            finite_residuals.push_back(residual);
        }
    }
    if (finite_residuals.size() <= 4U) {
        return inlier_mask;
    }

    const double center = median(finite_residuals);
    std::vector<double> deviations;
    deviations.reserve(finite_residuals.size());
    for (double residual : finite_residuals) {
        deviations.push_back(std::abs(residual - center));
    }

    const double mad_sigma = 1.4826 * median(deviations);
    double sigma = std::max(mad_sigma, sigma_floor);
    if (!std::isfinite(sigma) || sigma <= 0.0) {
        double sum_sq = 0.0;
        for (double residual : finite_residuals) {
            const double centered = residual - center;
            sum_sq += centered * centered;
        }
        sigma = std::sqrt(sum_sq / static_cast<double>(finite_residuals.size()));
    }
    if (!std::isfinite(sigma) || sigma <= 0.0) {
        return inlier_mask;
    }

    const double threshold = threshold_sigma * sigma;
    for (size_t i = 0; i < residuals.size(); ++i) {
        const double residual = residuals[i];
        inlier_mask[i] = std::isfinite(residual) && std::abs(residual - center) <= threshold;
    }

    size_t inlier_count = 0;
    for (bool is_inlier : inlier_mask) {
        if (is_inlier) {
            ++inlier_count;
        }
    }
    return inlier_count >= 4U ? inlier_mask : std::vector<bool>(residuals.size(), true);
}

int selectRaimFdeCandidate(double baseline_rms,
                           const std::vector<double>& candidate_rms_values,
                           double min_improvement_ratio,
                           double min_improvement_m) {
    if (!std::isfinite(baseline_rms) || baseline_rms <= 0.0 ||
        candidate_rms_values.empty()) {
        return -1;
    }

    int best_index = -1;
    double best_rms = baseline_rms;
    for (size_t i = 0; i < candidate_rms_values.size(); ++i) {
        const double candidate_rms = candidate_rms_values[i];
        if (std::isfinite(candidate_rms) && candidate_rms < best_rms) {
            best_rms = candidate_rms;
            best_index = static_cast<int>(i);
        }
    }
    if (best_index < 0) {
        return -1;
    }

    if (!std::isfinite(min_improvement_ratio) || min_improvement_ratio < 0.0) {
        min_improvement_ratio = 0.0;
    }
    if (!std::isfinite(min_improvement_m) || min_improvement_m < 0.0) {
        min_improvement_m = 0.0;
    }

    const double improvement = baseline_rms - best_rms;
    const double required_improvement =
        std::max(min_improvement_m, baseline_rms * min_improvement_ratio);
    return improvement >= required_improvement ? best_index : -1;
}

} // namespace spp_utils

std::pair<PositionSolution, std::vector<SPPProcessor::CorrectedMeasurement>>
SPPProcessor::preprocessEpoch(const ObservationData& obs, const NavigationData& nav) {
    // Run normal SPP processing
    auto solution = processEpoch(obs, nav);

    // Re-run measurement construction to extract corrected pseudoranges
    std::vector<CorrectedMeasurement> result;
    auto valid_obs = validateObservations(obs, nav, obs.time);
    if (valid_obs.empty()) return {solution, result};

    auto sat_states = calculateSatelliteStates(valid_obs, nav, obs.time);
    Vector3d position = estimated_position_;

    for (const auto& spp_obs : valid_obs) {
        const Observation& o = spp_obs.observation;
        auto it = sat_states.find(o.satellite);
        if (it == sat_states.end() || !it->second.valid) continue;

        const auto& st = it->second;
        Vector3d sat_position = st.position;
        double sat_clk = st.clock_bias;

        // Signal travel time and Earth rotation correction (Sagnac)
        double range_approx = (sat_position - position).norm();
        double travel_time = range_approx / constants::SPEED_OF_LIGHT;
        if (st.precise_orbit_clock) {
            sat_position -= st.velocity * travel_time;
            sat_clk -= st.clock_drift * travel_time;
            range_approx = (sat_position - position).norm();
            travel_time = range_approx / constants::SPEED_OF_LIGHT;
        }
        double angle = constants::OMEGA_E * travel_time;
        double cos_a = std::cos(angle), sin_a = std::sin(angle);
        Vector3d corrected_sat_pos(
            sat_position.x() * cos_a + sat_position.y() * sin_a,
           -sat_position.x() * sin_a + sat_position.y() * cos_a,
            sat_position.z());

        // Geometry
        auto geom = nav.calculateGeometry(position, corrected_sat_pos);
        if (geom.elevation < elevationMaskRadians(config_.elevation_mask)) continue;

        // Receiver LLH for atmospheric models
        auto rcv_geo = spp_utils::ecefToGeodetic(position);
        double rcv_lat = rcv_geo.latitude;
        double rcv_lon = rcv_geo.longitude;

        // Troposphere
        double trop_delay = 0.0;
        const bool trop_corrected =
            spp_config_.apply_atmospheric_corrections && config_.use_troposphere_model;
        if (trop_corrected) {
            trop_delay = models::tropDelaySaastamoinen(position, geom.elevation);
        }

        // Ionosphere
        double iono_delay = 0.0;
        const auto* eph = nav.getEphemeris(o.satellite, obs.time);
        bool iono_corrected = false;
        if (spp_obs.ionosphere_free) {
            iono_corrected = true;
        } else if (spp_config_.apply_atmospheric_corrections &&
            config_.use_ionosphere_model &&
            spp_config_.use_ionex_corrections &&
            ionex_products_loaded_) {
            double ipp_lat_deg = 0.0;
            double ipp_lon_deg = 0.0;
            double mapping_factor = 0.0;
            double vertical_tecu = 0.0;
            if (ionexPiercePointAndMapping(
                    ionex_products_,
                    position,
                    geom.azimuth,
                    geom.elevation,
                    ipp_lat_deg,
                    ipp_lon_deg,
                    mapping_factor) &&
                ionex_products_.interpolateTecu(
                    obs.time, ipp_lat_deg, ipp_lon_deg, vertical_tecu, nullptr)) {
                iono_delay = ionosphereDelayMetersFromTecu(
                    o.signal, eph, mapping_factor * vertical_tecu);
                if (std::isfinite(iono_delay) && std::abs(iono_delay) > 0.0) {
                    iono_corrected = true;
                } else {
                    iono_delay = 0.0;
                }
            }
        }
        if (!iono_corrected &&
            spp_config_.apply_atmospheric_corrections &&
            config_.use_ionosphere_model &&
            nav.ionosphere_model.valid) {
            iono_delay = models::ionoDelayKlobuchar(
                rcv_lat, rcv_lon, geom.azimuth, geom.elevation,
                obs.time.tow,
                nav.ionosphere_model.alpha,
                nav.ionosphere_model.beta);
            double signal_frequency = eph ? signalFrequencyHz(o.signal, eph) : 0.0;
            if (signal_frequency > 0.0) {
                double freq_scale = constants::GPS_L1_FREQ / signal_frequency;
                iono_delay *= freq_scale * freq_scale;
            }
            iono_corrected = true;
        }

        double ssr_code_bias_m = 0.0;
        bool ssr_code_bias_applied = false;
        if (spp_config_.use_ssr_corrections && ssr_products_loaded_) {
            Vector3d ssr_orbit_correction = Vector3d::Zero();
            double ssr_clock_correction_m = 0.0;
            std::map<uint8_t, double> ssr_code_biases;
            if (ssr_products_.interpolateCorrection(
                    o.satellite,
                    obs.time,
                    ssr_orbit_correction,
                    ssr_clock_correction_m,
                    nullptr,
                    &ssr_code_biases)) {
                ssr_code_bias_m = observationSsrCodeBiasMeters(
                    o.satellite.system,
                    spp_obs.primary_signal,
                    spp_obs.secondary_signal,
                    spp_obs.ionosphere_free,
                    spp_obs.primary_coeff,
                    spp_obs.secondary_coeff,
                    ssr_code_biases);
                ssr_code_bias_applied =
                    std::isfinite(ssr_code_bias_m) && std::abs(ssr_code_bias_m) > 0.0;
                if (!std::isfinite(ssr_code_bias_m)) {
                    ssr_code_bias_m = 0.0;
                    ssr_code_bias_applied = false;
                }
            }
        }

        double dcb_bias_m = 0.0;
        if (!ssr_code_bias_applied &&
            spp_config_.use_dcb_corrections &&
            dcb_products_loaded_) {
            dcb_bias_m = observationDcbBiasMeters(
                dcb_products_,
                o.satellite,
                spp_obs.primary_signal,
                spp_obs.secondary_signal,
                spp_obs.ionosphere_free,
                spp_obs.primary_coeff,
                spp_obs.secondary_coeff);
            if (!std::isfinite(dcb_bias_m)) {
                dcb_bias_m = 0.0;
            }
        }

        double group_delay = eph ? groupDelayCorrectionMeters(o, *eph) : 0.0;
        if (eph && spp_obs.ionosphere_free) {
            Observation primary_obs = o;
            primary_obs.signal = spp_obs.primary_signal;
            Observation secondary_obs = o;
            secondary_obs.signal = spp_obs.secondary_signal;
            group_delay =
                spp_obs.primary_coeff * groupDelayCorrectionMeters(primary_obs, *eph) +
                spp_obs.secondary_coeff * groupDelayCorrectionMeters(secondary_obs, *eph);
        }

        double corrected_pr = o.pseudorange
                              + sat_clk * constants::SPEED_OF_LIGHT
                              - trop_delay
                              - iono_delay
                              - group_delay
                              - ssr_code_bias_m
                              - dcb_bias_m;

        double sin_el = std::sin(geom.elevation);
        if (sin_el < 0.1) sin_el = 0.1;

        double variance = spp_config_.pseudorange_sigma * spp_config_.pseudorange_sigma /
                          (sin_el * sin_el);
        if (spp_config_.use_variance_model) {
            spp_utils::MeasurementVarianceInputs variance_inputs;
            variance_inputs.elevation_rad = geom.elevation;
            variance_inputs.snr_dbhz = o.snr;
            variance_inputs.pseudorange_sigma_m = spp_config_.pseudorange_sigma;
            variance_inputs.ionosphere_delay_m = iono_delay;
            variance_inputs.troposphere_delay_m = trop_delay;
            variance_inputs.ionosphere_corrected = iono_corrected;
            variance_inputs.troposphere_corrected = trop_corrected;
            variance_inputs.snr_reference_dbhz = spp_config_.snr_reference_dbhz;
            variance_inputs.ionosphere_sigma_scale = spp_config_.ionosphere_sigma_scale;
            variance_inputs.troposphere_sigma_scale = spp_config_.troposphere_sigma_scale;
            variance = spp_utils::calculatePseudorangeVariance(variance_inputs);
        }
        variance *= std::max(1.0, spp_obs.variance_scale);
        if (!std::isfinite(variance) || variance <= 0.0) {
            continue;
        }

        int sys_id = 0;
        switch (o.satellite.system) {
            case GNSSSystem::GPS:     sys_id = 0; break;
            case GNSSSystem::GLONASS: sys_id = 1; break;
            case GNSSSystem::Galileo: sys_id = 2; break;
            case GNSSSystem::BeiDou:  sys_id = 3; break;
            case GNSSSystem::QZSS:    sys_id = 4; break;
            default: continue;
        }

        CorrectedMeasurement cm;
        cm.satellite_ecef = {corrected_sat_pos.x(), corrected_sat_pos.y(), corrected_sat_pos.z()};
        cm.corrected_pseudorange = corrected_pr;
        cm.weight = 1.0 / variance;
        cm.variance = variance;
        cm.elevation = geom.elevation;
        cm.system_id = sys_id;
        cm.ionosphere_free = spp_obs.ionosphere_free;
        result.push_back(cm);
    }

    return {solution, result};
}

} // namespace libgnss
