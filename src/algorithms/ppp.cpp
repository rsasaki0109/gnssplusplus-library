#include "ppp_internal.hpp"

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cctype>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace libgnss {

using namespace ppp_internal;

namespace {

std::string normalizeStationName(const std::string& station_name) {
    return normalizeAntennaType(station_name);
}
SignalType antexSignalType(const std::string& code) {
    const std::string trimmed = trimCopy(code);
    if (trimmed == "G01") return SignalType::GPS_L1CA;
    if (trimmed == "G02") return SignalType::GPS_L2C;
    if (trimmed == "G05") return SignalType::GPS_L5;
    if (trimmed == "R01") return SignalType::GLO_L1CA;
    if (trimmed == "R02") return SignalType::GLO_L2CA;
    if (trimmed == "E01") return SignalType::GAL_E1;
    if (trimmed == "E05") return SignalType::GAL_E5A;
    if (trimmed == "E07") return SignalType::GAL_E5B;
    if (trimmed == "E06") return SignalType::GAL_E6;
    if (trimmed == "C02") return SignalType::BDS_B1I;
    if (trimmed == "C07") return SignalType::BDS_B2I;
    if (trimmed == "C06") return SignalType::BDS_B3I;
    if (trimmed == "C01") return SignalType::BDS_B1C;
    if (trimmed == "C05") return SignalType::BDS_B2A;
    if (trimmed == "J01") return SignalType::QZS_L1CA;
    if (trimmed == "J02") return SignalType::QZS_L2C;
    if (trimmed == "J05") return SignalType::QZS_L5;
    return SignalType::SIGNAL_TYPE_COUNT;
}

std::string extractAntexFrequencyCode(const std::string& line) {
    for (size_t index = 0; index + 2 < std::min<size_t>(line.size(), 20U); ++index) {
        if (std::isalpha(static_cast<unsigned char>(line[index])) &&
            std::isdigit(static_cast<unsigned char>(line[index + 1])) &&
            std::isdigit(static_cast<unsigned char>(line[index + 2]))) {
            return line.substr(index, 3);
        }
    }
    return "";
}

GNSSTime parseAntexEpochLine(const std::string& line) {
    int year = 0, month = 0, day = 0, hour = 0, minute = 0;
    double second = 0.0;
    std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 60U)));
    if (!(iss >> year >> month >> day >> hour >> minute >> second)) {
        return GNSSTime{};
    }
    std::tm tm{};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = static_cast<int>(second);
    const std::time_t calendar = timegm(&tm);
    if (calendar == static_cast<std::time_t>(-1)) {
        return GNSSTime{};
    }
    return GNSSTime::fromSystemTime(
        std::chrono::system_clock::from_time_t(calendar) +
        std::chrono::microseconds(static_cast<long>((second - tm.tm_sec) * 1e6)));
}

bool loadSatelliteAntexOffsets(
    const std::string& filename,
    std::vector<SatelliteAntexEntry>& satellite_entries) {
    satellite_entries.clear();
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }
    SatelliteAntexEntry current;
    bool in_antenna = false;
    bool satellite_entry = false;
    SignalType current_signal = SignalType::SIGNAL_TYPE_COUNT;
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() >= 60 ? trimCopy(line.substr(60)) : "";
        if (label == "START OF ANTENNA") {
            current = SatelliteAntexEntry{};
            satellite_entry = false;
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            in_antenna = true;
            continue;
        }
        if (!in_antenna) continue;
        if (label == "END OF ANTENNA") {
            if (satellite_entry && !current.body_offsets_m.empty()) {
                satellite_entries.push_back(current);
            }
            in_antenna = false;
            continue;
        }
        if (label == "TYPE / SERIAL NO") {
            // Satellite entries place the PRN at columns 20-22 (3-char
            // SVS code: G01, R26, E07, ...). Receiver entries put a
            // free-form serial there.
            const std::string serial = trimCopy(line.substr(20, 20));
            satellite_entry =
                serial.size() == 3 &&
                std::isalpha(static_cast<unsigned char>(serial[0])) &&
                std::isdigit(static_cast<unsigned char>(serial[1])) &&
                std::isdigit(static_cast<unsigned char>(serial[2]));
            if (satellite_entry) {
                GNSSSystem system = GNSSSystem::UNKNOWN;
                switch (serial[0]) {
                    case 'G': system = GNSSSystem::GPS; break;
                    case 'R': system = GNSSSystem::GLONASS; break;
                    case 'E': system = GNSSSystem::Galileo; break;
                    case 'C': system = GNSSSystem::BeiDou; break;
                    case 'J': system = GNSSSystem::QZSS; break;
                    case 'S': system = GNSSSystem::SBAS; break;
                    case 'I': system = GNSSSystem::NavIC; break;
                    default: break;
                }
                if (system == GNSSSystem::UNKNOWN) {
                    satellite_entry = false;
                } else {
                    try {
                        current.satellite = SatelliteId(
                            system,
                            static_cast<uint8_t>(std::stoi(serial.substr(1))));
                    } catch (const std::exception&) {
                        satellite_entry = false;
                    }
                }
            }
            continue;
        }
        if (!satellite_entry) continue;
        if (label == "VALID FROM") {
            current.valid_from = parseAntexEpochLine(line);
            continue;
        }
        if (label == "VALID UNTIL") {
            current.valid_until = parseAntexEpochLine(line);
            continue;
        }
        if (label == "START OF FREQUENCY") {
            current_signal = antexSignalType(extractAntexFrequencyCode(line));
            continue;
        }
        if (label == "END OF FREQUENCY") {
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            continue;
        }
        if (label == "NORTH / EAST / UP" &&
            current_signal != SignalType::SIGNAL_TYPE_COUNT) {
            std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 30U)));
            double north_mm = 0.0, east_mm = 0.0, up_mm = 0.0;
            if (iss >> north_mm >> east_mm >> up_mm) {
                current.body_offsets_m[current_signal] =
                    Vector3d(north_mm * 1e-3, east_mm * 1e-3, up_mm * 1e-3);
            }
        }
    }
    return !satellite_entries.empty();
}

bool loadReceiverAntexOffsets(
    const std::string& filename,
    std::map<std::string, std::map<SignalType, Vector3d>>& receiver_offsets,
    std::map<std::string, std::map<SignalType, ReceiverPcvGrid>>&
        receiver_pcv) {
    receiver_offsets.clear();
    receiver_pcv.clear();
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_antenna = false;
    bool receiver_entry = false;
    SignalType current_signal = SignalType::SIGNAL_TYPE_COUNT;
    std::string current_type;
    std::map<SignalType, Vector3d> current_offsets;
    std::map<SignalType, ReceiverPcvGrid> current_pcv;
    double zen1_deg = 0.0, zen2_deg = 90.0, dzen_deg = 5.0;
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() >= 60 ? trimCopy(line.substr(60)) : "";
        if (label == "START OF ANTENNA") {
            in_antenna = true;
            receiver_entry = false;
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            current_type.clear();
            current_offsets.clear();
            current_pcv.clear();
            zen1_deg = 0.0;
            zen2_deg = 90.0;
            dzen_deg = 5.0;
            continue;
        }
        if (!in_antenna) {
            continue;
        }
        if (label == "END OF ANTENNA") {
            if (receiver_entry && !current_type.empty() && !current_offsets.empty()) {
                receiver_offsets[current_type] = current_offsets;
                if (!current_pcv.empty()) {
                    receiver_pcv[current_type] = current_pcv;
                }
            }
            in_antenna = false;
            continue;
        }
        if (label == "TYPE / SERIAL NO") {
            current_type = normalizeAntennaType(line.substr(0, 20));
            const std::string serial = trimCopy(line.substr(20, 20));
            receiver_entry = !(serial.size() == 3 &&
                               std::isalpha(static_cast<unsigned char>(serial[0])) &&
                               std::isdigit(static_cast<unsigned char>(serial[1])) &&
                               std::isdigit(static_cast<unsigned char>(serial[2])));
            continue;
        }
        if (!receiver_entry) {
            continue;
        }
        if (label == "ZEN1 / ZEN2 / DZEN") {
            std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 60U)));
            iss >> zen1_deg >> zen2_deg >> dzen_deg;
            continue;
        }
        if (label == "START OF FREQUENCY") {
            current_signal = antexSignalType(extractAntexFrequencyCode(line));
            continue;
        }
        if (label == "END OF FREQUENCY") {
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            continue;
        }
        if (label == "NORTH / EAST / UP" &&
            current_signal != SignalType::SIGNAL_TYPE_COUNT) {
            std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 30U)));
            double north_mm = 0.0;
            double east_mm = 0.0;
            double up_mm = 0.0;
            if (iss >> north_mm >> east_mm >> up_mm) {
                current_offsets[current_signal] =
                    Vector3d(east_mm * 1e-3, north_mm * 1e-3, up_mm * 1e-3);
            }
            continue;
        }
        // Elevation-dependent PCV row ("NOAZI" followed by one value per zenith
        // node). Azimuth-dependent rows (leading numeric azimuth) are ignored;
        // the NOAZI mean matches MADOCALIB's interpvar() lookup.
        if (current_signal != SignalType::SIGNAL_TYPE_COUNT) {
            std::istringstream iss(line);
            std::string tag;
            if (iss >> tag && tag == "NOAZI") {
                ReceiverPcvGrid grid;
                grid.zen1_deg = zen1_deg;
                grid.zen2_deg = zen2_deg;
                grid.dzen_deg = dzen_deg;
                double v_mm = 0.0;
                while (iss >> v_mm) {
                    grid.noazi_m.push_back(v_mm * 1e-3);
                }
                if (!grid.noazi_m.empty()) {
                    current_pcv[current_signal] = std::move(grid);
                }
            }
        }
    }

    return !receiver_offsets.empty();
}

bool parseBlqValues(const std::string& line, std::array<double, 11>& values) {
    std::istringstream iss(line);
    for (double& value : values) {
        if (!(iss >> value)) {
            return false;
        }
    }
    return true;
}

bool loadOceanLoadingCoefficients(
    const std::string& filename,
    const std::string& station_name,
    std::array<double, 11>& up_amplitudes_m,
    std::array<double, 11>& west_amplitudes_m,
    std::array<double, 11>& south_amplitudes_m,
    std::array<double, 11>& up_phases_deg,
    std::array<double, 11>& west_phases_deg,
    std::array<double, 11>& south_phases_deg) {
    const std::string normalized_station = normalizeStationName(station_name);
    if (normalized_station.empty()) {
        return false;
    }

    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool collecting = false;
    int collected_rows = 0;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed.rfind("$$", 0) == 0) {
            continue;
        }

        if (!collecting) {
            std::istringstream header_stream(trimmed);
            std::string first_token;
            header_stream >> first_token;
            if (first_token.empty()) {
                continue;
            }
            const bool numeric_header =
                std::isdigit(static_cast<unsigned char>(first_token.front())) ||
                first_token.front() == '-' || first_token.front() == '+';
            if (numeric_header) {
                continue;
            }
            collecting = normalizeStationName(first_token) == normalized_station;
            collected_rows = 0;
            continue;
        }

        std::array<double, 11> values{};
        if (!parseBlqValues(trimmed, values)) {
            collecting = false;
            collected_rows = 0;
            continue;
        }

        switch (collected_rows) {
            case 0: up_amplitudes_m = values; break;
            case 1: west_amplitudes_m = values; break;
            case 2: south_amplitudes_m = values; break;
            case 3: up_phases_deg = values; break;
            case 4: west_phases_deg = values; break;
            case 5: south_phases_deg = values; break;
            default: break;
        }
        ++collected_rows;
        if (collected_rows == 6) {
            return true;
        }
    }

    return false;
}
double clampDt(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0) {
        return 1.0;
    }
    return std::min(dt, 300.0);
}

}  // namespace

PPPProcessor::PPPProcessor()
    : env_overrides_(PPPEnvOverrides::fromEnvironment()), spp_processor_() {
    applyEnvironmentOverridesToPPPConfig();
    reset();
}

PPPProcessor::PPPProcessor(const PPPConfig& ppp_config)
    : ppp_config_(ppp_config),
      env_overrides_(PPPEnvOverrides::fromEnvironment()),
      spp_processor_() {
    applyEnvironmentOverridesToPPPConfig();
    reset();
}

void PPPProcessor::applyEnvironmentOverridesToPPPConfig() {
    if (env_overrides_.clas_dd_filter) {
        ppp_config_.use_clas_dd_filter = true;
    }
}

bool PPPProcessor::initialize(const ProcessorConfig& config) {
    config_ = config;
    ppp_config_.use_precise_orbits = config.use_precise_orbits;
    ppp_config_.use_precise_clocks = config.use_precise_clocks;
    if (ppp_config_.orbit_file_path.empty()) {
        ppp_config_.orbit_file_path = config.orbit_file_path;
    }
    if (ppp_config_.clock_file_path.empty()) {
        ppp_config_.clock_file_path = config.clock_file_path;
    }

    spp_processor_.initialize(config);
    reset();

    receiver_antex_offsets_.clear();
    receiver_antex_pcv_.clear();
    receiver_antex_loaded_ = false;
    satellite_antex_offsets_.clear();
    satellite_antex_loaded_ = false;
    if (!ppp_config_.antex_file_path.empty()) {
        receiver_antex_loaded_ =
            loadReceiverAntexOffsets(ppp_config_.antex_file_path, receiver_antex_offsets_,
                                     receiver_antex_pcv_);
        // Receiver section may be empty for sat-only ANTEX files; the load
        // is fatal only if no satellite block also ends up populated.
        satellite_antex_loaded_ =
            loadSatelliteAntexOffsets(ppp_config_.antex_file_path, satellite_antex_offsets_);
        if (!receiver_antex_loaded_ && !satellite_antex_loaded_) {
            return false;
        }
    }

    ocean_loading_loaded_ = false;
    ocean_loading_coefficients_ = OceanLoadingCoefficients{};
    if (ppp_config_.apply_ocean_loading && !ppp_config_.ocean_loading_file_path.empty()) {
        ocean_loading_loaded_ = loadOceanLoadingCoefficients(
            ppp_config_.ocean_loading_file_path,
            ppp_config_.ocean_loading_station_name,
            ocean_loading_coefficients_.up_amplitudes_m,
            ocean_loading_coefficients_.west_amplitudes_m,
            ocean_loading_coefficients_.south_amplitudes_m,
            ocean_loading_coefficients_.up_phases_deg,
            ocean_loading_coefficients_.west_phases_deg,
            ocean_loading_coefficients_.south_phases_deg);
        if (!ocean_loading_loaded_) {
            return false;
        }
    }

    if ((ppp_config_.use_precise_orbits && !ppp_config_.orbit_file_path.empty()) ||
        (ppp_config_.use_precise_clocks && !ppp_config_.clock_file_path.empty())) {
        loadPreciseProducts(ppp_config_.orbit_file_path, ppp_config_.clock_file_path);
    }
    if (ppp_config_.use_ssr_corrections && !ppp_config_.ssr_file_path.empty()) {
        loadSSRProducts(ppp_config_.ssr_file_path);
    }
    if (!ppp_config_.ionex_file_path.empty() &&
        !loadIONEXProducts(ppp_config_.ionex_file_path)) {
        return false;
    }
    if (!ppp_config_.dcb_file_path.empty() &&
        !loadDCBProducts(ppp_config_.dcb_file_path)) {
        return false;
    }
    if (!ppp_config_.eop_path.empty() && !loadEopC04(ppp_config_.eop_path)) {
        return false;
    }
    atm_tidal_loading_loaded_ = false;
    atm_tidal_loading_coefficients_ = libgnss::iers::AtmosphericTidalLoadingCoefficients{};
    if (!ppp_config_.atm_tidal_loading_path.empty() &&
        !loadAtmosphericTidalLoading(ppp_config_.atm_tidal_loading_path)) {
        return false;
    }
    return true;
}

PositionSolution PPPProcessor::processEpochStandard(
    const ObservationData& obs,
    const NavigationData& nav,
    const char* clas_hybrid_fallback_reason) {
    const auto processing_start = std::chrono::steady_clock::now();

    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;
    auto finalizeSolution = [&](PositionSolution result) {
        const auto processing_end = std::chrono::steady_clock::now();
        result.processing_time_ms =
            std::chrono::duration<double, std::milli>(processing_end - processing_start).count();
        updateStatistics(result.isValid());
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            total_convergence_time_ += result.processing_time_ms;
        }
        return result;
    };
    last_clas_hybrid_fallback_used_ = clas_hybrid_fallback_reason != nullptr;
    last_clas_hybrid_fallback_reason_ =
        clas_hybrid_fallback_reason != nullptr ? clas_hybrid_fallback_reason : "";
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    last_applied_atmos_trop_corrections_ = 0;
    last_applied_atmos_iono_corrections_ = 0;
    last_applied_atmos_trop_m_ = 0.0;
    last_applied_atmos_iono_m_ = 0.0;
    last_applied_ionex_corrections_ = 0;
    last_applied_dcb_corrections_ = 0;
    last_applied_ionex_m_ = 0.0;
    last_applied_dcb_m_ = 0.0;
    if (require_coherent_ssr_ && ssr_products_loaded_ &&
        !hasEnoughCoherentSsrObservations(obs, nav)) {
        return finalizeSolution(solution);
    }
    PositionSolution seed_solution;
    const bool use_seed_assist = useLowDynamicsBroadcastSeedAssist();
    const bool need_seed_solution =
        ppp_config_.reset_clock_to_spp_each_epoch ||
        (ppp_config_.kinematic_mode && ppp_config_.reset_kinematic_position_to_spp_each_epoch) ||
        use_seed_assist ||
        !filter_initialized_;
    const PositionSolution* seed_ptr = nullptr;
    const auto runSeedSpp = [&]() {
        const SPPProcessor::SPPConfig original_spp_config = spp_processor_.getSPPConfig();
        if (ssr_products_loaded_ && original_spp_config.enable_raim_fde) {
            SPPProcessor::SPPConfig seed_spp_config = original_spp_config;
            // The internal seed solve only initializes/resets the PPP clock.
            // Full leave-one-out SPP FDE can dominate long SSR runs, while
            // fallback SPP output still uses the normal processor settings.
            seed_spp_config.enable_raim_fde = false;
            spp_processor_.setSPPConfig(seed_spp_config);
            try {
                PositionSolution seed = spp_processor_.processEpoch(obs, nav);
                spp_processor_.setSPPConfig(original_spp_config);
                return seed;
            } catch (...) {
                spp_processor_.setSPPConfig(original_spp_config);
                throw;
            }
        }
        return spp_processor_.processEpoch(obs, nav);
    };

    try {
        if (need_seed_solution) {
            seed_solution = runSeedSpp();
            if (seed_solution.isValid()) {
                seed_ptr = &seed_solution;
            }
        }
        if (!filter_initialized_) {
            if (!initializeFilter(obs, nav, seed_ptr)) {
                if (!require_coherent_ssr_) {
                    solution = seed_ptr != nullptr ? seed_solution : spp_processor_.processEpoch(obs, nav);
                }
            } else {
                filter_initialized_ = true;
                convergence_start_time_ = obs.time;
            }
        }

        if (filter_initialized_) {
            const double dt = has_last_processed_time_ ? clampDt(obs.time - last_processed_time_) : 1.0;
            detectCycleSlips(obs, nav);
            predictState(dt, seed_ptr);

            bool updated = updateFilter(obs, nav);
            if (!updated && ppp_config_.enable_ambiguity_resolution) {
                bool had_fixed_ambiguities = false;
                for (auto& [satellite, ambiguity] : ambiguity_states_) {
                    if (!ambiguity.is_fixed) {
                        continue;
                    }
                    had_fixed_ambiguities = true;
                    resetAmbiguity(satellite, SignalType::GPS_L1CA);
                }
                if (had_fixed_ambiguities) {
                    if (pppDebugEnabled()) {
                        std::cerr << "[PPP-AR] retry update after resetting fixed ambiguities\n";
                    }
                    updated = updateFilter(obs, nav);
                }
            }

            if (updated) {
                const auto if_obs = formIonosphereFree(obs, nav);
                auto corrected_if_obs = if_obs;
                applyPreciseCorrections(corrected_if_obs, nav, obs.time);
                checkConvergence(obs.time);
                PositionSolution float_solution = generateSolution(obs.time, corrected_if_obs);
                float_solution.status = SolutionStatus::PPP_FLOAT;
                float_solution.ratio = 0.0;
                float_solution.num_fixed_ambiguities = 0;
                const PPPState float_filter_state = filter_state_;
                const auto float_ambiguity_states = ambiguity_states_;
                const bool fixed =
                    ppp_config_.enable_ambiguity_resolution && resolveAmbiguities(obs, nav);
                solution = fixed ? generateSolution(obs.time, corrected_if_obs) : float_solution;
                solution.status = fixed ? SolutionStatus::PPP_FIXED : SolutionStatus::PPP_FLOAT;
                solution.ratio = fixed ? last_ar_ratio_ : 0.0;
                solution.num_fixed_ambiguities = fixed ? last_fixed_ambiguities_ : 0;
                // For WLNL fix: override position with WLS using fixed NL ambiguities
                const double max_fixed_position_jump_m =
                    ppp_config_.kinematic_mode ? 25.0 : 10.0;
                bool accepted_fixed_solution = fixed;
                if (fixed &&
                    (solution.position_ecef - float_solution.position_ecef).norm() >
                        max_fixed_position_jump_m) {
                    accepted_fixed_solution = false;
                    if (pppDebugEnabled()) {
                        std::cerr << "[PPP-AR] reject fixed filter-state jump="
                                  << (solution.position_ecef - float_solution.position_ecef).norm()
                                  << "\n";
                    }
                }
                if (fixed && ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
                    !ppp_config_.use_ionosphere_free) {
                    // Per-frequency WLNL: override position with a clean WLS over
                    // the fixed narrow-lane observations.  The decoupled IFLC
                    // path instead applies the fix as a Kalman update, so the
                    // filter-state position already reflects it.
                    Vector3d fixed_position;
                    if (solveFixedPosition(obs, nav, fixed_position)) {
                        if ((fixed_position - float_solution.position_ecef).norm() <= max_fixed_position_jump_m) {
                            solution.position_ecef = fixed_position;
                        } else if (pppDebugEnabled()) {
                            accepted_fixed_solution = false;
                            std::cerr << "[PPP-AR] reject WLNL fixed position jump="
                                      << (fixed_position - float_solution.position_ecef).norm()
                                      << "\n";
                        } else {
                            accepted_fixed_solution = false;
                        }
                    }
                }
                if (!accepted_fixed_solution && fixed) {
                    solution = float_solution;
                    solution.status = SolutionStatus::PPP_FLOAT;
                    solution.ratio = 0.0;
                    solution.num_fixed_ambiguities = 0;
                }
                if (accepted_fixed_solution) {
                    double latitude = 0.0;
                    double longitude = 0.0;
                    double height = 0.0;
                    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
                    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);
                }
                had_fixed_last_epoch_ = accepted_fixed_solution;
                if (fixed && !accepted_fixed_solution) {
                    filter_state_ = float_filter_state;
                    ambiguity_states_ = float_ambiguity_states;
                } else if (accepted_fixed_solution && ppp_config_.ar_method != PPPConfig::ARMethod::DD_WLNL) {
                    // For DD_IFLC/DD_PER_FREQ: revert to float state to avoid
                    // poisoning later epochs with a bad fix.
                    filter_state_ = float_filter_state;
                    ambiguity_states_ = float_ambiguity_states;
                }
                // For DD_WLNL: keep holdamb-updated state — the constraint
                // is applied via Kalman update and propagates naturally.
            } else if (!solution.isValid()) {
                if (pppDebugEnabled()) {
                    std::cerr << "[PPP] updateFilter returned false at week=" << obs.time.week
                              << " tow=" << obs.time.tow << "\n";
                }
                if (use_seed_assist) {
                    recoverLowDynamicsBroadcastState(obs, seed_ptr);
                }
                if (!require_coherent_ssr_) {
                    solution = seed_ptr != nullptr ? seed_solution : spp_processor_.processEpoch(obs, nav);
                }
            }
        }
    } catch (const std::exception& e) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP] exception at week=" << obs.time.week
                      << " tow=" << obs.time.tow << ": " << e.what() << "\n";
        }
        if (!require_coherent_ssr_) {
            solution = seed_ptr != nullptr ? seed_solution : spp_processor_.processEpoch(obs, nav);
        }
    }

    has_last_processed_time_ = true;
    last_processed_time_ = obs.time;

    return finalizeSolution(solution);
}

PositionSolution PPPProcessor::processEpoch(const ObservationData& obs, const NavigationData& nav) {
    if (ppp_config_.use_clas_osr_filter) {
        return processEpochCLAS(obs, nav);
    }

    // Standard PPP path: precise/broadcast navigation with optional inline
    // OSR corrections. Experiment arms use this as the stable control and as
    // the hybrid fallback target for CLAS epoch-policy trials.
    return processEpochStandard(obs, nav);
}

ProcessorStats PPPProcessor::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ProcessorStats stats;
    stats.total_epochs = total_epochs_processed_;
    stats.valid_solutions = converged_solutions_;
    if (total_epochs_processed_ > 0) {
        stats.average_processing_time_ms = total_convergence_time_ / total_epochs_processed_;
    }
    return stats;
}

void PPPProcessor::reset() {
    filter_initialized_ = false;
    converged_ = false;
    convergence_time_ = 0.0;
    filter_state_ = PPPState{};
    ambiguity_states_.clear();
    clas_dispersion_compensation_.clear();
    clas_sis_continuity_.clear();
    clas_phase_bias_repair_.clear();
    clas_dd_filter_.reset();
    recent_positions_.clear();
    has_last_processed_time_ = false;
    last_processed_time_ = GNSSTime();
    last_clas_atmos_network_id_ = -1;
    has_last_clas_atmos_network_id_ = false;
    precise_products_loaded_ = !precise_products_.orbit_clock_data.empty();
    ssr_products_loaded_ = !ssr_products_.orbit_clock_corrections.empty();
    ionex_products_loaded_ = !ionex_products_.tec_maps.empty();
    dcb_products_loaded_ = !dcb_products_.entries.empty();
    ocean_loading_loaded_ = false;
    ocean_loading_coefficients_ = OceanLoadingCoefficients{};
    static_anchor_position_.setZero();
    has_static_anchor_position_ = false;
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    last_applied_atmos_trop_corrections_ = 0;
    last_applied_atmos_iono_corrections_ = 0;
    last_applied_atmos_trop_m_ = 0.0;
    last_applied_atmos_iono_m_ = 0.0;
    last_applied_ionex_corrections_ = 0;
    last_applied_dcb_corrections_ = 0;
    last_applied_ionex_m_ = 0.0;
    last_applied_dcb_m_ = 0.0;

    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_ = 0;
    converged_solutions_ = 0;
    total_convergence_time_ = 0.0;
}

void PPPProcessor::updateStatistics(bool converged) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    const_cast<size_t&>(total_epochs_processed_)++;
    if (converged) {
        const_cast<size_t&>(converged_solutions_)++;
    }
}

namespace ppp_utils {

std::pair<double, double> getIonosphereFreeCoefficients(double f1, double f2) {
    const double denom = f1 * f1 - f2 * f2;
    if (!std::isfinite(denom) || std::abs(denom) < 1.0) {
        return {1.0, 0.0};
    }
    return {
        (f1 * f1) / denom,
        -(f2 * f2) / denom,
    };
}

double calculateMelbourneWubbena(double l1_phase,
                                 double l2_phase,
                                 double p1_range,
                                 double p2_range,
                                 double f1,
                                 double f2) {
    if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
        return 0.0;
    }
    return (l1_phase - l2_phase) * constants::SPEED_OF_LIGHT / (f1 - f2) -
           (f1 * p1_range + f2 * p2_range) / (f1 + f2);
}

double calculateGeometryFree(double l1_phase, double l2_phase) {
    return l1_phase - l2_phase;
}

bool interpolatePreciseOrbit(const std::vector<PreciseOrbitClock>& orbit_data,
                             const GNSSTime& time,
                             Vector3d& position,
                             Vector3d& velocity) {
    if (orbit_data.empty()) {
        return false;
    }
    auto upper = std::lower_bound(
        orbit_data.begin(), orbit_data.end(), time,
        [](const PreciseOrbitClock& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    if (upper == orbit_data.begin()) {
        if (!upper->position_valid) {
            return false;
        }
        position = upper->position;
        velocity = upper->velocity;
        return true;
    }
    if (upper == orbit_data.end()) {
        const auto& last = orbit_data.back();
        if (!last.position_valid) {
            return false;
        }
        position = last.position;
        velocity = last.velocity;
        return true;
    }

    const auto& before = *(upper - 1);
    const auto& after = *upper;
    if (!before.position_valid || !after.position_valid) {
        return false;
    }
    const double dt = after.time - before.time;
    if (std::abs(dt) < 1e-9) {
        position = before.position;
        velocity = before.velocity;
        return true;
    }
    const double alpha = (time - before.time) / dt;
    position = before.position + alpha * (after.position - before.position);
    velocity = (after.position - before.position) / dt;
    return true;
}

bool interpolatePreciseClock(const std::vector<PreciseOrbitClock>& clock_data,
                             const GNSSTime& time,
                             double& clock_bias,
                             double& clock_drift) {
    if (clock_data.empty()) {
        return false;
    }
    auto upper = std::lower_bound(
        clock_data.begin(), clock_data.end(), time,
        [](const PreciseOrbitClock& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    if (upper == clock_data.begin()) {
        if (!upper->clock_valid) {
            return false;
        }
        clock_bias = upper->clock_bias;
        clock_drift = upper->clock_drift;
        return true;
    }
    if (upper == clock_data.end()) {
        const auto& last = clock_data.back();
        if (!last.clock_valid) {
            return false;
        }
        clock_bias = last.clock_bias;
        clock_drift = last.clock_drift;
        return true;
    }

    const auto& before = *(upper - 1);
    const auto& after = *upper;
    if (!before.clock_valid || !after.clock_valid) {
        return false;
    }
    const double dt = after.time - before.time;
    if (std::abs(dt) < 1e-9) {
        clock_bias = before.clock_bias;
        clock_drift = before.clock_drift;
        return true;
    }
    const double alpha = (time - before.time) / dt;
    clock_bias = before.clock_bias + alpha * (after.clock_bias - before.clock_bias);
    clock_drift = (after.clock_bias - before.clock_bias) / dt;
    return true;
}

Vector3d calculateSatelliteAntennaPCO(const SatelliteId& satellite,
                                      const Vector3d& satellite_pos,
                                      const Vector3d& sun_pos) {
    (void)satellite;
    (void)satellite_pos;
    (void)sun_pos;
    return Vector3d::Zero();
}

Vector3d calculateReceiverAntennaPCO(const Vector3d& receiver_pos,
                                     const Vector3d& satellite_pos,
                                     const std::string& antenna_type) {
    (void)receiver_pos;
    (void)satellite_pos;
    (void)antenna_type;
    return Vector3d::Zero();
}

double calculatePhaseWindup(const Vector3d& receiver_pos,
                            const Vector3d& satellite_pos,
                            const Vector3d& sun_position_ecef,
                            double previous_windup) {
    // Phase wind-up (Wu et al. 1993) — accumulated cycles between the
    // satellite and receiver right-handed dipole frames.
    const Vector3d los_sr = receiver_pos - satellite_pos;
    const double los_norm = los_sr.norm();
    if (los_norm < 1.0) return previous_windup;
    const Vector3d e_sr = los_sr / los_norm;

    // Satellite body frame using a nominal yaw-steering attitude:
    //   z = -rs / |rs|              (toward Earth center)
    //   y = z × (sun - rs) / |...|  (perpendicular to spacecraft–Sun plane)
    //   x = y × z                   (toward Sun side)
    const Vector3d e_z_sat = -satellite_pos.normalized();
    Vector3d sun_dir;
    if (sun_position_ecef.norm() > 1.0) {
        sun_dir = sun_position_ecef - satellite_pos;
    } else {
        // Fallback: cross-track approximation (gives wrong magnitude but
        // keeps the correction bounded when no solar ephemeris is supplied).
        sun_dir = e_sr.cross(e_z_sat);
    }
    const double sun_norm = sun_dir.norm();
    if (sun_norm < 1e-3) return previous_windup;
    sun_dir /= sun_norm;
    Vector3d e_y_sat = e_z_sat.cross(sun_dir);
    const double y_norm = e_y_sat.norm();
    if (y_norm < 1e-10) return previous_windup;
    e_y_sat /= y_norm;
    const Vector3d e_x_sat = e_y_sat.cross(e_z_sat);

    // Receiver local north-east-up (north, east, up are standard PPP/RTKLIB):
    //   x_rcv → North, y_rcv → East, z_rcv → Up.
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_pos, latitude_rad, longitude_rad, height_m);
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lat = std::cos(latitude_rad);
    const double sin_lon = std::sin(longitude_rad);
    const double cos_lon = std::cos(longitude_rad);
    // North (X): -sin(lat) cos(lon), -sin(lat) sin(lon), cos(lat)
    const Vector3d e_x_rcv(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    // East  (Y): -sin(lon),                cos(lon),           0
    const Vector3d e_y_rcv(-sin_lon, cos_lon, 0.0);

    // Effective dipole vectors (Wu et al. 1993, eq. 7)
    const Vector3d d_sat = e_x_sat - e_sr * (e_sr.dot(e_x_sat))
                         - e_sr.cross(e_y_sat);
    const Vector3d d_rcv = e_x_rcv - e_sr * (e_sr.dot(e_x_rcv))
                         + e_sr.cross(e_y_rcv);

    const double d_sat_norm = d_sat.norm();
    const double d_rcv_norm = d_rcv.norm();
    if (d_sat_norm < 1e-10 || d_rcv_norm < 1e-10) return previous_windup;
    const double cos_phi = std::clamp(d_sat.dot(d_rcv) / (d_sat_norm * d_rcv_norm), -1.0, 1.0);
    double dphi = std::acos(cos_phi) / (2.0 * M_PI);
    if (e_sr.dot(d_sat.cross(d_rcv)) < 0.0) dphi = -dphi;

    // Resolve the 2π ambiguity against the prior cycle count.
    const double n = std::round(previous_windup - dphi);
    return dphi + n;
}

}  // namespace ppp_utils

}  // namespace libgnss
