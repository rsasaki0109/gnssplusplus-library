#include <libgnss++/algorithms/ppp_clas.hpp>

#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace libgnss::ppp_clas {

namespace {

constexpr std::array<double, 100> kClaslibChiSquare001 = {
    10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
    31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
    46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
    61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
    74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
    88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100.0,
    101.0,102.0,103.0,104.0,105.0,107.0,108.0,109.0,110.0,112.0,
    113.0,114.0,115.0,116.0,118.0,119.0,120.0,122.0,123.0,125.0,
    126.0,127.0,128.0,129.0,131.0,132.0,133.0,134.0,135.0,137.0,
    138.0,139.0,140.0,142.0,143.0,144.0,145.0,147.0,148.0,149.0
};

double claslibChiSquare001ForDof(int dof) {
    if (dof <= 0) {
        return 0.0;
    }
    if (dof <= static_cast<int>(kClaslibChiSquare001.size())) {
        return kClaslibChiSquare001[static_cast<size_t>(dof - 1)];
    }
    const double tail = std::max(0, dof - static_cast<int>(kClaslibChiSquare001.size()));
    return kClaslibChiSquare001.back() + 1.1 * tail;
}

struct PhaseResidualInfo {
    SatelliteId ambiguity_satellite;
    SatelliteId real_satellite;
    double residual_m = 0.0;
    double variance_m2 = 0.0;
    double frequency_hz = 0.0;
    double wavelength_m = 0.0;
};

struct PhasePairInfo {
    std::array<double, 2> residual_m{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 2> variance_m2{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 2> frequency_hz{0.0, 0.0};
    std::array<double, 2> wavelength_m{0.0, 0.0};
};

double elevationWeight(double elevation_rad) {
    const double s = std::sin(elevation_rad);
    return 1.0 / (s * s);
}

double clasCodeVarianceForRow(
    const ppp_shared::PPPConfig& config,
    GNSSSystem system,
    double elevation_rad) {
    if (!config.wlnl_strict_claslib_parity) {
        return 5.5 * config.clas_code_variance_scale * elevationWeight(elevation_rad);
    }
    constexpr double kCodePhaseRatio = 50.0;
    constexpr double kPhaseSigmaM = 0.01;
    constexpr double kPhaseElevationSigmaM = 0.005;
    const double system_factor =
        system == GNSSSystem::GLONASS ? 1.5 : 1.0;
    const double sin_el = std::max(std::sin(elevation_rad), 1e-3);
    const double a = system_factor * kCodePhaseRatio * kPhaseSigmaM;
    const double b = system_factor * kCodePhaseRatio * kPhaseElevationSigmaM;
    return a * a + (b * b) / (sin_el * sin_el);
}

double clasPhaseVarianceForRow(
    const ppp_shared::PPPConfig& config,
    GNSSSystem system,
    double elevation_rad,
    int freq_index) {
    if (!config.wlnl_strict_claslib_parity) {
        return config.clas_phase_variance * elevationWeight(elevation_rad);
    }
    constexpr double kPhaseSigmaM = 0.01;
    constexpr double kPhaseElevationSigmaM = 0.005;
    const double system_factor =
        system == GNSSSystem::GLONASS ? 1.5 : 1.0;
    const double sin_el = std::max(std::sin(elevation_rad), 1e-3);
    const double a = system_factor * kPhaseSigmaM;
    const double b = system_factor * kPhaseElevationSigmaM;
    double variance = a * a + (b * b) / (sin_el * sin_el);
    if (freq_index == 1) {
        variance *= std::pow(2.55 / 1.55, 2);
    }
    return variance;
}

double claslibIonoMappingFunction(
    const Vector3d& receiver_position,
    double elevation_rad) {
    constexpr double kIonosphereHeightM = 350000.0;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver_position, lat, lon, height);
    (void)lat;
    (void)lon;
    if (height >= kIonosphereHeightM) {
        return 1.0;
    }
    const double ratio =
        (constants::WGS84_A + height) /
        (constants::WGS84_A + kIonosphereHeightM);
    const double arg = std::clamp(ratio * std::cos(elevation_rad), -1.0, 1.0);
    const double denom = std::cos(std::asin(arg));
    return denom > 0.0 ? 1.0 / denom : 1.0;
}

constexpr double kStartupAmbiguityVarianceScale = 0.5;
constexpr double kStartupPhaseVarianceInflation = 4.0;
constexpr double kClaslibVarPos = 30.0 * 30.0;
constexpr double kClaslibVarVel = 1.0;
constexpr double kClaslibVarAcc = 1.0;
constexpr double kClaslibInitZwd = 0.001;
constexpr double kClaslibStdBiasCycles = 100.0;
constexpr double kClaslibStdIonoMeters = 0.010;
constexpr double kClaslibStdTropMeters = 0.005;
constexpr double kClaslibPrnBiasMeters = 0.001;
constexpr double kClaslibPrnIonoMeters = 0.001;
constexpr double kClaslibPrnIonoMaxMeters = 0.050;
constexpr double kClaslibPrnTropMeters = 0.001;
constexpr double kClaslibPrnPosHorizontalMeters = 0.0001;
constexpr double kClaslibPrnPosVerticalMeters = 0.0001;
constexpr int kClaslibMaxOut = 90;
constexpr int kClaslibMinLock = 5;

bool shouldLogClasPhaseRow(const SatelliteId& satellite);
bool shouldLogClasAmbSeed(const SatelliteId& satellite, double tow);
SatelliteId clasRealSatelliteForUpdateDump(const SatelliteId& satellite);
double updateDumpWavelengthMeters(const SatelliteId& satellite, int freq_index);

void resizeState(ppp_shared::PPPState& filter_state, int new_size) {
    if (new_size <= filter_state.total_states) {
        return;
    }
    const int old_size = filter_state.total_states;
    VectorXd new_state = VectorXd::Zero(new_size);
    if (filter_state.state.size() > 0) {
        new_state.head(std::min<int>(old_size, filter_state.state.size())) =
            filter_state.state.head(std::min<int>(old_size, filter_state.state.size()));
    }
    MatrixXd new_covariance = MatrixXd::Zero(new_size, new_size);
    if (filter_state.covariance.size() > 0) {
        const int copy_size =
            std::min<int>(old_size, std::min(filter_state.covariance.rows(),
                                            filter_state.covariance.cols()));
        new_covariance.topLeftCorner(copy_size, copy_size) =
            filter_state.covariance.topLeftCorner(copy_size, copy_size);
    }
    filter_state.state = std::move(new_state);
    filter_state.covariance = std::move(new_covariance);
    filter_state.total_states = new_size;
}

void initx(ppp_shared::PPPState& filter_state, double xi, double var, int i) {
    if (i < 0) {
        return;
    }
    resizeState(filter_state, i + 1);
    filter_state.state(i) = xi;
    filter_state.covariance.row(i).setZero();
    filter_state.covariance.col(i).setZero();
    filter_state.covariance(i, i) = var;
}

SatelliteId ambiguitySatelliteForFreq(const SatelliteId& satellite, int f) {
    if (f <= 0) {
        return satellite;
    }
    return SatelliteId(
        satellite.system,
        static_cast<uint8_t>(std::min(255, static_cast<int>(satellite.prn) + 100 * f)));
}

int ambiguityFreqFromSatellite(const SatelliteId& ambiguity_satellite) {
    return ambiguity_satellite.prn > 100 ? 1 : 0;
}

double ambiguityWavelengthForState(
    const SatelliteId& ambiguity_satellite,
    const std::map<SatelliteId, std::array<double, OSR_MAX_FREQ>>& wavelengths_by_sat) {
    const int f = ambiguityFreqFromSatellite(ambiguity_satellite);
    const SatelliteId real_satellite = clasRealSatelliteForUpdateDump(ambiguity_satellite);
    const auto wl_it = wavelengths_by_sat.find(real_satellite);
    if (wl_it != wavelengths_by_sat.end()) {
        return wl_it->second[static_cast<size_t>(f)];
    }
    return updateDumpWavelengthMeters(real_satellite, f);
}

int ensurePortedAmbiguityState(
    ppp_shared::PPPState& filter_state,
    const SatelliteId& ambiguity_satellite) {
    const auto it = filter_state.ambiguity_indices.find(ambiguity_satellite);
    if (it != filter_state.ambiguity_indices.end()) {
        return it->second;
    }
    const int state_index = filter_state.total_states;
    filter_state.ambiguity_indices[ambiguity_satellite] = state_index;
    resizeState(filter_state, state_index + 1);
    filter_state.state(state_index) = 0.0;
    filter_state.covariance(state_index, state_index) = 0.0;
    return state_index;
}

void resetPortedPhaseRepair(
    const SatelliteId& satellite,
    int f,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair) {
    if (f >= 0 && f < 2) {
        dispersion_compensation[satellite].slip[static_cast<size_t>(f)] = true;
    }
    auto repair_it = phase_bias_repair.find(satellite);
    if (repair_it == phase_bias_repair.end()) {
        return;
    }
    if (f >= 0 && f < OSR_MAX_FREQ) {
        repair_it->second.reference_time = GNSSTime();
        repair_it->second.last_continuity_m[static_cast<size_t>(f)] = 0.0;
        repair_it->second.offset_cycles[static_cast<size_t>(f)] = 0.0;
        repair_it->second.pending_state_shift_cycles[static_cast<size_t>(f)] = 0.0;
        repair_it->second.has_last[static_cast<size_t>(f)] = false;
    }
}

void udposPppPorted(
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const PositionSolution& seed_solution,
    double tt) {
    (void)config;
    if (filter_state.total_states < 3) {
        resizeState(filter_state, 3);
    }
    if (filter_state.state.head(3).norm() <= 0.0) {
        if (seed_solution.isValid()) {
            for (int i = 0; i < 3; ++i) {
                initx(filter_state, seed_solution.position_ecef(i), kClaslibVarPos, i);
            }
        }
        return;
    }

    double var = 0.0;
    for (int i = 0; i < 3; ++i) {
        var += filter_state.covariance(i, i);
    }
    var /= 3.0;
    if (var > kClaslibVarPos && seed_solution.isValid()) {
        for (int i = 0; i < 3; ++i) {
            initx(filter_state, seed_solution.position_ecef(i), kClaslibVarPos, i);
        }
        if (filter_state.total_states >= 6) {
            for (int i = 3; i < 6; ++i) {
                initx(filter_state, 0.0, kClaslibVarVel, i);
            }
        }
        if (filter_state.total_states >= 9) {
            for (int i = 6; i < 9; ++i) {
                initx(filter_state, 1e-6, kClaslibVarAcc, i);
            }
        }
        return;
    }

    const double q = kClaslibPrnPosHorizontalMeters *
                     kClaslibPrnPosHorizontalMeters * std::abs(tt);
    const double qv = kClaslibPrnPosVerticalMeters *
                      kClaslibPrnPosVerticalMeters * std::abs(tt);
    const double qpos = std::max(q, qv);
    for (int i = 0; i < 3; ++i) {
        filter_state.covariance(i, i) += qpos;
    }
}

void udclkPppPorted(
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const PositionSolution& seed_solution) {
    const std::array<int, 4> clock_indices{
        filter_state.clock_index,
        filter_state.glo_clock_index,
        filter_state.gal_clock_index,
        filter_state.bds_clock_index};
    for (const int index : clock_indices) {
        if (index < 0 || index >= filter_state.total_states) {
            continue;
        }
        filter_state.covariance(index, index) = config.clas_clock_variance;
        if (seed_solution.isValid() &&
            config.clas_epoch_policy ==
                ppp_shared::PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK) {
            filter_state.state(index) = seed_solution.receiver_clock_bias;
        }
    }
}

void udtropPppPorted(
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    double tt) {
    if (!config.estimate_troposphere ||
        filter_state.trop_index < 0 ||
        filter_state.trop_index >= filter_state.total_states) {
        return;
    }
    const int j = filter_state.trop_index;
    if (filter_state.state(j) == 0.0) {
        initx(filter_state, kClaslibInitZwd,
              kClaslibStdTropMeters * kClaslibStdTropMeters, j);
    } else {
        filter_state.covariance(j, j) +=
            kClaslibPrnTropMeters * kClaslibPrnTropMeters * tt;
    }
}

void udionPppPorted(
    const std::vector<OSRCorrection>& osr_corrections,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    double tt,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states) {
    if (!config.estimate_ionosphere) {
        return;
    }
    std::set<SatelliteId> observed_satellites;
    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            observed_satellites.insert(osr.satellite);
        }
    }
    for (auto& [satellite, state_index] : filter_state.ionosphere_indices) {
        if (state_index < 0 || state_index >= filter_state.total_states) {
            continue;
        }
        if (observed_satellites.find(satellite) == observed_satellites.end()) {
            const auto amb_it = ambiguity_states.find(satellite);
            if (amb_it != ambiguity_states.end() &&
                amb_it->second.outage_count > kClaslibMaxOut) {
                filter_state.state(state_index) = 0.0;
            }
            continue;
        }
        if (filter_state.state(state_index) == 0.0) {
            initx(filter_state, 1e-6,
                  kClaslibStdIonoMeters * kClaslibStdIonoMeters, state_index);
            continue;
        }
        const auto osr_it = std::find_if(
            osr_corrections.begin(), osr_corrections.end(),
            [&](const OSRCorrection& osr) {
                return osr.valid && osr.satellite == satellite;
            });
        const double el = osr_it != osr_corrections.end() ? osr_it->elevation : 0.0;
        const double fact = std::cos(el);
        double qi = kClaslibPrnIonoMeters * fact;
        qi *= qi;
        qi = std::clamp(qi,
                        kClaslibPrnIonoMeters * kClaslibPrnIonoMeters,
                        kClaslibPrnIonoMaxMeters * kClaslibPrnIonoMaxMeters);
        filter_state.covariance(state_index, state_index) += qi * tt;
    }
}

void udbiasPppPorted(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    double tt,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    bool debug_enabled) {
    std::map<SatelliteId, std::array<double, OSR_MAX_FREQ>> wavelengths_by_sat;
    std::map<SatelliteId, std::array<bool, OSR_MAX_FREQ>> slip_by_sat;
    int nf = 0;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        nf = std::max(nf, osr.num_frequencies);
        auto& wavelengths = wavelengths_by_sat[osr.satellite];
        auto& slips = slip_by_sat[osr.satellite];
        for (int f = 0; f < osr.num_frequencies && f < OSR_MAX_FREQ; ++f) {
            wavelengths[static_cast<size_t>(f)] = osr.wavelengths[f];
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (raw && raw->valid && raw->has_carrier_phase && raw->loss_of_lock) {
                slips[static_cast<size_t>(f)] = true;
            }
        }
        if (osr.num_frequencies >= 2 && osr.wavelengths[0] > 0.0 &&
            osr.wavelengths[1] > 0.0) {
            const Observation* l1 = obs.getObservation(osr.satellite, osr.signals[0]);
            const Observation* l2 = obs.getObservation(osr.satellite, osr.signals[1]);
            if (l1 && l2 && l1->valid && l2->valid &&
                l1->has_carrier_phase && l2->has_carrier_phase &&
                std::isfinite(l1->carrier_phase) &&
                std::isfinite(l2->carrier_phase)) {
                const double g1 = l1->carrier_phase * osr.wavelengths[0] -
                                  l2->carrier_phase * osr.wavelengths[1];
                auto& ambiguity = ambiguity_states[osr.satellite];
                if (ambiguity.has_last_geometry_free &&
                    std::abs(g1 - ambiguity.last_geometry_free_m) >
                        config.cycle_slip_threshold) {
                    slips[0] = true;
                    slips[1] = true;
                }
                ambiguity.last_geometry_free_m = g1;
                ambiguity.has_last_geometry_free = true;
            }
        }
    }
    nf = std::clamp(nf, 0, OSR_MAX_FREQ);

    for (const auto& [amb_satellite, state_index] : filter_state.ambiguity_indices) {
        if (state_index < 0 || state_index >= filter_state.total_states) {
            continue;
        }
        ambiguity_states[amb_satellite].outage_count++;
    }

    for (int f = 0; f < nf; ++f) {
        for (const auto& [amb_satellite, state_index] : filter_state.ambiguity_indices) {
            if (ambiguityFreqFromSatellite(amb_satellite) != f ||
                state_index < 0 ||
                state_index >= filter_state.total_states) {
                continue;
            }
            auto& ambiguity = ambiguity_states[amb_satellite];
            if (ambiguity.outage_count > kClaslibMaxOut &&
                filter_state.state(state_index) != 0.0) {
                initx(filter_state, 0.0, 0.0, state_index);
                ambiguity.lock_count = -kClaslibMinLock;
                ambiguity.needs_reinitialization = true;
            }
        }

        for (const auto& osr : osr_corrections) {
            if (!osr.valid || f >= osr.num_frequencies ||
                osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const SatelliteId amb_satellite = ambiguitySatelliteForFreq(osr.satellite, f);
            const int state_index = ensurePortedAmbiguityState(filter_state, amb_satellite);
            filter_state.covariance(state_index, state_index) +=
                kClaslibPrnBiasMeters * kClaslibPrnBiasMeters * tt *
                osr.wavelengths[f] * osr.wavelengths[f];

            auto slip_it = slip_by_sat.find(osr.satellite);
            const bool slip =
                slip_it != slip_by_sat.end() && slip_it->second[static_cast<size_t>(f)];
            if (slip && filter_state.state(state_index) != 0.0) {
                initx(filter_state, 0.0, 0.0, state_index);
                auto& ambiguity = ambiguity_states[amb_satellite];
                ambiguity.lock_count = -kClaslibMinLock;
                ambiguity.needs_reinitialization = true;
                resetPortedPhaseRepair(
                    osr.satellite, f, dispersion_compensation, phase_bias_repair);
            }
        }

        std::map<SatelliteId, double> bias_m;
        double offset = 0.0;
        int j = 0;
        for (const auto& osr : osr_corrections) {
            if (!osr.valid || f >= osr.num_frequencies ||
                osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid || !raw->has_carrier_phase ||
                !raw->has_pseudorange || !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange) ||
                raw->carrier_phase == 0.0 || raw->pseudorange == 0.0) {
                continue;
            }
            const SatelliteId amb_satellite = ambiguitySatelliteForFreq(osr.satellite, f);
            const double bias = raw->carrier_phase * osr.wavelengths[f] -
                                raw->pseudorange;
            bias_m[amb_satellite] = bias;
            const int state_index = ensurePortedAmbiguityState(filter_state, amb_satellite);
            if (filter_state.state(state_index) != 0.0) {
                offset += bias - filter_state.state(state_index);
                j++;
            }
        }
        const double com_bias = j > 0 ? offset / static_cast<double>(j) : 0.0;

        for (const auto& osr : osr_corrections) {
            if (!osr.valid || f >= osr.num_frequencies ||
                osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const SatelliteId amb_satellite = ambiguitySatelliteForFreq(osr.satellite, f);
            const auto bias_it = bias_m.find(amb_satellite);
            if (bias_it == bias_m.end()) {
                continue;
            }
            const int state_index = ensurePortedAmbiguityState(filter_state, amb_satellite);
            if (filter_state.state(state_index) != 0.0) {
                continue;
            }
            const double seed_m = bias_it->second - com_bias;
            initx(filter_state, seed_m,
                  std::pow(kClaslibStdBiasCycles * osr.wavelengths[f], 2),
                  state_index);
            auto& ambiguity = ambiguity_states[amb_satellite];
            ambiguity.lock_count = -kClaslibMinLock;
            ambiguity.outage_count = 0;
            ambiguity.needs_reinitialization = true;
            if (debug_enabled && shouldLogClasAmbSeed(osr.satellite, obs.time.tow)) {
                const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
                const double seed_cycles = seed_m / osr.wavelengths[f];
                std::cerr << "[CLAS-UDSTATE-SEED] tow=" << obs.time.tow
                          << " sat=" << osr.satellite.toString()
                          << " amb_sat=" << amb_satellite.toString()
                          << " f=" << f
                          << " raw_cp=" << (raw ? raw->carrier_phase : 0.0)
                          << " raw_pr=" << (raw ? raw->pseudorange : 0.0)
                          << " lambda=" << osr.wavelengths[f]
                          << " bias_m=" << bias_it->second
                          << " com_bias_m=" << com_bias
                          << " seed_m=" << seed_m
                          << " seed_cycles=" << seed_cycles
                          << "\n";
            }
        }
    }
}

void dumpUdstatePorted(
    const ObservationData& obs,
    const ppp_shared::PPPState& filter_state,
    const std::vector<OSRCorrection>& osr_corrections,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states) {
    std::map<SatelliteId, std::array<double, OSR_MAX_FREQ>> wavelengths_by_sat;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        auto& wavelengths = wavelengths_by_sat[osr.satellite];
        for (int f = 0; f < osr.num_frequencies && f < OSR_MAX_FREQ; ++f) {
            wavelengths[static_cast<size_t>(f)] = osr.wavelengths[f];
        }
    }
    const double clk =
        filter_state.clock_index >= 0 &&
                filter_state.clock_index < filter_state.state.size()
            ? filter_state.state(filter_state.clock_index)
            : 0.0;
    const double trop =
        filter_state.trop_index >= 0 &&
                filter_state.trop_index < filter_state.state.size()
            ? filter_state.state(filter_state.trop_index)
            : 0.0;
    std::cerr << std::setprecision(15)
              << "[CLAS-UDSTATE] tow=" << obs.time.tow
              << " pos_x=" << filter_state.state(0)
              << " pos_y=" << filter_state.state(1)
              << " pos_z=" << filter_state.state(2)
              << " clk=" << clk
              << " trop=" << trop
              << "\n";
    for (const auto& [satellite, state_index] : filter_state.ionosphere_indices) {
        if (state_index < 0 || state_index >= filter_state.state.size() ||
            !shouldLogClasPhaseRow(satellite)) {
            continue;
        }
        std::cerr << "[CLAS-UDSTATE-IONO] tow=" << obs.time.tow
                  << " sat=" << satellite.toString()
                  << " iono=" << filter_state.state(state_index)
                  << " var=" << filter_state.covariance(state_index, state_index)
                  << "\n";
    }
    for (const auto& [amb_satellite, state_index] : filter_state.ambiguity_indices) {
        if (state_index < 0 || state_index >= filter_state.state.size()) {
            continue;
        }
        const SatelliteId real_satellite = clasRealSatelliteForUpdateDump(amb_satellite);
        if (!shouldLogClasPhaseRow(real_satellite)) {
            continue;
        }
        const int f = ambiguityFreqFromSatellite(amb_satellite);
        const double wavelength =
            ambiguityWavelengthForState(amb_satellite, wavelengths_by_sat);
        const double x_m = filter_state.state(state_index);
        const double x_cycles =
            wavelength > 0.0 ? x_m / wavelength : std::numeric_limits<double>::quiet_NaN();
        const auto amb_it = ambiguity_states.find(amb_satellite);
        const int lock =
            amb_it != ambiguity_states.end() ? amb_it->second.lock_count : 0;
        const int outc =
            amb_it != ambiguity_states.end() ? amb_it->second.outage_count : 0;
        std::cerr << "[CLAS-UDSTATE-AMB] tow=" << obs.time.tow
                  << " sat=" << real_satellite.toString()
                  << " amb_sat=" << amb_satellite.toString()
                  << " freq_group=" << f
                  << " x_m=" << x_m
                  << " x_cycles=" << x_cycles
                  << " var_m2=" << filter_state.covariance(state_index, state_index)
                  << " lock=" << lock
                  << " outc=" << outc
                  << "\n";
    }
}

std::vector<int> collectFreshAmbiguityIndices(
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config) {
    std::vector<int> fresh_indices;
    const double startup_variance_threshold =
        config.clas_ambiguity_reinit_threshold * kStartupAmbiguityVarianceScale;
    for (const auto& [_, ambiguity_index] : filter_state.ambiguity_indices) {
        if (ambiguity_index < 0 || ambiguity_index >= filter_state.total_states) {
            continue;
        }
        if (filter_state.covariance(ambiguity_index, ambiguity_index) >=
            startup_variance_threshold) {
            fresh_indices.push_back(ambiguity_index);
        }
    }
    return fresh_indices;
}

bool hasFreshAmbiguitySeed(
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config) {
    return !collectFreshAmbiguityIndices(filter_state, config).empty();
}

const char* signalObservationCode(SignalType signal) {
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

bool shouldLogClasPhaseRow(const SatelliteId& satellite) {
    if (satellite.system == GNSSSystem::GPS) {
        return satellite.prn == 14 || satellite.prn == 25 || satellite.prn == 26 ||
               satellite.prn == 29 || satellite.prn == 31 || satellite.prn == 32;
    }
    if (satellite.system == GNSSSystem::Galileo) {
        return satellite.prn == 7 || satellite.prn == 27 || satellite.prn == 30;
    }
    if (satellite.system == GNSSSystem::QZSS) {
        return satellite.prn == 1 || satellite.prn == 2 || satellite.prn == 3;
    }
    return false;
}

bool shouldLogClasAmbSeed(const SatelliteId& satellite, double tow) {
    return tow >= 230420.0 - 1e-6 &&
           tow <= 230425.0 + 1e-6 &&
           shouldLogClasPhaseRow(satellite);
}

bool shouldDumpClasLongParityTime(const GNSSTime& time) {
    return time.week == 2068 &&
           ((time.tow >= 230420.0 - 1e-6 && time.tow <= 230425.0 + 1e-6) ||
            std::abs(time.tow - 230620.0) <= 1e-6 ||
            std::abs(time.tow - 231420.0) <= 1e-6 ||
            std::abs(time.tow - 232419.0) <= 1e-6);
}

SatelliteId clasRealSatelliteForUpdateDump(const SatelliteId& satellite) {
    if (satellite.prn > 100) {
        return SatelliteId(
            satellite.system,
            static_cast<uint8_t>(std::max(1, static_cast<int>(satellite.prn) - 100)));
    }
    return satellite;
}

SatelliteId clasAmbiguitySatelliteForUpdateDump(
    const SatelliteId& satellite,
    int freq_index) {
    const SatelliteId real_satellite = clasRealSatelliteForUpdateDump(satellite);
    if (freq_index == 1) {
        return SatelliteId(
            real_satellite.system,
            static_cast<uint8_t>(std::min(255, static_cast<int>(real_satellite.prn) + 100)));
    }
    return real_satellite;
}

bool shouldLogClasUpdateTarget(const SatelliteId& satellite) {
    const SatelliteId real_satellite = clasRealSatelliteForUpdateDump(satellite);
    if (real_satellite.system == GNSSSystem::GPS) {
        return real_satellite.prn == 14 || real_satellite.prn == 25 ||
               real_satellite.prn == 26 || real_satellite.prn == 29 ||
               real_satellite.prn == 31 || real_satellite.prn == 32;
    }
    if (real_satellite.system == GNSSSystem::Galileo) {
        return real_satellite.prn == 7 || real_satellite.prn == 27;
    }
    return real_satellite.system == GNSSSystem::QZSS &&
           (real_satellite.prn == 1 || real_satellite.prn == 3);
}

double updateDumpWavelengthMeters(const SatelliteId& satellite, int freq_index) {
    const SatelliteId real_satellite = clasRealSatelliteForUpdateDump(satellite);
    if (real_satellite.system == GNSSSystem::GPS ||
        real_satellite.system == GNSSSystem::QZSS) {
        return freq_index == 1 ? constants::GPS_L2_WAVELENGTH
                               : constants::GPS_L1_WAVELENGTH;
    }
    if (real_satellite.system == GNSSSystem::Galileo) {
        return freq_index == 1 ? constants::GAL_E5A_WAVELENGTH
                               : constants::GAL_E1_WAVELENGTH;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

double updateDumpHAt(const Eigen::RowVectorXd& H, int index) {
    if (index < 0 || index >= H.size()) {
        return 0.0;
    }
    return H(index);
}

double updateDumpVecAt(const Eigen::VectorXd& v, int index) {
    if (index < 0 || index >= v.size()) {
        return 0.0;
    }
    return v(index);
}

double subtractComponent(double lhs, double rhs) {
    if (!std::isfinite(lhs) || !std::isfinite(rhs)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return lhs - rhs;
}

MeasurementRow::ModelComponents subtractModelComponents(
    const MeasurementRow::ModelComponents& lhs,
    const MeasurementRow::ModelComponents& rhs) {
    MeasurementRow::ModelComponents out;
    out.valid = lhs.valid && rhs.valid;
    auto sub = [&](double& dst, double a, double b) {
        dst = subtractComponent(a, b);
    };
    sub(out.y, lhs.y, rhs.y);
    sub(out.obs, lhs.obs, rhs.obs);
    sub(out.rho, lhs.rho, rhs.rho);
    sub(out.dts, lhs.dts, rhs.dts);
    sub(out.dtr, lhs.dtr, rhs.dtr);
    sub(out.rel, lhs.rel, rhs.rel);
    sub(out.sagnac, lhs.sagnac, rhs.sagnac);
    sub(out.trop_dry, lhs.trop_dry, rhs.trop_dry);
    sub(out.trop_wet, lhs.trop_wet, rhs.trop_wet);
    sub(out.trop_model, lhs.trop_model, rhs.trop_model);
    sub(out.trop_grid, lhs.trop_grid, rhs.trop_grid);
    sub(out.iono_grid, lhs.iono_grid, rhs.iono_grid);
    sub(out.iono_state_term, lhs.iono_state_term, rhs.iono_state_term);
    sub(out.prc, lhs.prc, rhs.prc);
    sub(out.cpc, lhs.cpc, rhs.cpc);
    sub(out.osr_corr, lhs.osr_corr, rhs.osr_corr);
    sub(out.code_bias, lhs.code_bias, rhs.code_bias);
    sub(out.phase_bias, lhs.phase_bias, rhs.phase_bias);
    sub(out.phase_comp, lhs.phase_comp, rhs.phase_comp);
    sub(out.windup, lhs.windup, rhs.windup);
    sub(out.pco_rcv, lhs.pco_rcv, rhs.pco_rcv);
    sub(out.pco_sat, lhs.pco_sat, rhs.pco_sat);
    sub(out.pcv_rcv, lhs.pcv_rcv, rhs.pcv_rcv);
    sub(out.pcv_sat, lhs.pcv_sat, rhs.pcv_sat);
    sub(out.tide_solid, lhs.tide_solid, rhs.tide_solid);
    sub(out.tide_ocean, lhs.tide_ocean, rhs.tide_ocean);
    sub(out.tide_pole, lhs.tide_pole, rhs.tide_pole);
    sub(out.amb_m, lhs.amb_m, rhs.amb_m);
    sub(out.modeled_sum, lhs.modeled_sum, rhs.modeled_sum);
    return out;
}

void dumpClasModelComponentRow(
    int row_index,
    const MeasurementRow& measurement,
    const GNSSTime& observation_time) {
    const auto real_satellite = clasRealSatelliteForUpdateDump(measurement.satellite);
    const auto real_reference =
        clasRealSatelliteForUpdateDump(measurement.reference_satellite);
    if (!shouldLogClasUpdateTarget(real_satellite) &&
        !shouldLogClasUpdateTarget(real_reference)) {
        return;
    }
    if (measurement.freq_index < 0 || measurement.freq_index > 1 ||
        !measurement.components.valid) {
        return;
    }

    const auto& c = measurement.components;
    std::cerr << "[CLAS-MODEL-COMP] source=lib"
              << " week=" << observation_time.week
              << " tow=" << observation_time.tow
              << " row=" << row_index
              << " sat=" << measurement.satellite.toString()
              << " ref=" << measurement.reference_satellite.toString()
              << " f=" << measurement.freq_index
              << " freq_group=" << measurement.freq_index
              << " type=" << (measurement.is_phase ? "phase" : "code")
              << std::setprecision(12)
              << " y=" << c.y
              << " obs=" << c.obs
              << " rho=" << c.rho
              << " dts=" << c.dts
              << " dtr=" << c.dtr
              << " rel=" << c.rel
              << " sagnac=" << c.sagnac
              << " trop_dry=" << c.trop_dry
              << " trop_wet=" << c.trop_wet
              << " trop_model=" << c.trop_model
              << " trop_grid=" << c.trop_grid
              << " iono_grid=" << c.iono_grid
              << " iono_state_term=" << c.iono_state_term
              << " prc=" << c.prc
              << " cpc=" << c.cpc
              << " osr_corr=" << c.osr_corr
              << " code_bias=" << c.code_bias
              << " phase_bias=" << c.phase_bias
              << " phase_comp=" << c.phase_comp
              << " windup=" << c.windup
              << " pco_rcv=" << c.pco_rcv
              << " pco_sat=" << c.pco_sat
              << " pcv_rcv=" << c.pcv_rcv
              << " pcv_sat=" << c.pcv_sat
              << " tide_solid=" << c.tide_solid
              << " tide_ocean=" << c.tide_ocean
              << " tide_pole=" << c.tide_pole
              << " amb_m=" << c.amb_m
              << " modeled_sum=" << c.modeled_sum
              << "\n";
}

void dumpClasUpdateRows(
    const ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const Eigen::MatrixXd& R,
    const Eigen::VectorXd& dx,
    const ppp_shared::PPPConfig& config,
    const GNSSTime* observation_time) {
    if (!config.wlnl_strict_claslib_parity ||
        !ppp_shared::pppDebugEnabled() ||
        observation_time == nullptr ||
        !shouldDumpClasLongParityTime(*observation_time)) {
        return;
    }

    struct Focus {
        SatelliteId real_satellite;
        SatelliteId ambiguity_satellite;
        const char* role = "";
    };

    auto append_focus = [](std::vector<Focus>& foci,
                           const SatelliteId& real_satellite,
                           const SatelliteId& ambiguity_satellite,
                           const char* role) {
        if (real_satellite.prn == 0 || !shouldLogClasUpdateTarget(real_satellite)) {
            return;
        }
        foci.push_back({real_satellite, ambiguity_satellite, role});
    };

    for (int row_index = 0; row_index < static_cast<int>(measurements.size()); ++row_index) {
        const auto& measurement = measurements[static_cast<size_t>(row_index)];
        if (measurement.freq_index < 0 || measurement.freq_index > 1) {
            continue;
        }

        std::vector<Focus> foci;
        if (measurement.satellite.prn != 0) {
            const SatelliteId real_satellite =
                clasRealSatelliteForUpdateDump(measurement.satellite);
            const SatelliteId ambiguity_satellite =
                measurement.is_phase && measurement.ambiguity_satellite.prn != 0
                    ? measurement.ambiguity_satellite
                    : clasAmbiguitySatelliteForUpdateDump(
                          real_satellite, measurement.freq_index);
            append_focus(foci, real_satellite, ambiguity_satellite, "sat");
        }
        if (measurement.reference_satellite.prn != 0) {
            const SatelliteId real_reference =
                clasRealSatelliteForUpdateDump(measurement.reference_satellite);
            const SatelliteId ambiguity_reference =
                measurement.is_phase
                    ? measurement.reference_satellite
                    : clasAmbiguitySatelliteForUpdateDump(
                          real_reference, measurement.freq_index);
            append_focus(foci, real_reference, ambiguity_reference, "ref");
        }

        if (foci.empty()) {
            continue;
        }

        const double h_pos_x = updateDumpHAt(measurement.H, 0);
        const double h_pos_y = updateDumpHAt(measurement.H, 1);
        const double h_pos_z = updateDumpHAt(measurement.H, 2);
        const double h_pos_norm =
            std::sqrt(h_pos_x * h_pos_x + h_pos_y * h_pos_y + h_pos_z * h_pos_z);
        const double h_trop = updateDumpHAt(measurement.H, filter_state.trop_index);
        const double r_diag =
            (row_index >= 0 && row_index < R.rows() && row_index < R.cols())
                ? R(row_index, row_index)
                : measurement.variance;
        dumpClasModelComponentRow(row_index, measurement, *observation_time);

        for (const auto& focus : foci) {
            const auto iono_it =
                filter_state.ionosphere_indices.find(focus.real_satellite);
            const int iono_index =
                iono_it != filter_state.ionosphere_indices.end() ? iono_it->second : -1;
            const auto amb_it =
                filter_state.ambiguity_indices.find(focus.ambiguity_satellite);
            const int amb_index =
                amb_it != filter_state.ambiguity_indices.end() ? amb_it->second : -1;
            const double lambda =
                updateDumpWavelengthMeters(focus.ambiguity_satellite,
                                           measurement.freq_index);
            const double h_iono = updateDumpHAt(measurement.H, iono_index);
            const double h_amb = updateDumpHAt(measurement.H, amb_index);
            const double h_amb_eff =
                std::isfinite(lambda) && lambda > 0.0 ? h_amb * lambda : h_amb;
            const double dx_state = updateDumpVecAt(dx, amb_index);
            const double dx_state_cycles =
                std::isfinite(lambda) && lambda > 0.0 ? dx_state / lambda : dx_state;
            const double state_before = updateDumpVecAt(filter_state.state, amb_index);
            const double state_before_cycles =
                std::isfinite(lambda) && lambda > 0.0 ? state_before / lambda
                                                       : state_before;
            const double state_after_cycles =
                std::isfinite(lambda) && lambda > 0.0
                    ? (state_before + dx_state) / lambda
                    : state_before + dx_state;

            std::cerr << "[CLAS-UPDATE-ROW] source=lib"
                      << " week=" << observation_time->week
                      << " tow=" << observation_time->tow
                      << " row=" << row_index
                      << " focus_sat=" << focus.real_satellite.toString()
                      << " focus_amb_sat=" << focus.ambiguity_satellite.toString()
                      << " focus_role=" << focus.role
                      << " sat=" << measurement.satellite.toString()
                      << " ref=" << measurement.reference_satellite.toString()
                      << " f=" << measurement.freq_index
                      << " freq_group=" << measurement.freq_index
                      << " type=" << (measurement.is_phase ? "phase" : "code")
                      << std::setprecision(12)
                      << " h_pos_x=" << h_pos_x
                      << " h_pos_y=" << h_pos_y
                      << " h_pos_z=" << h_pos_z
                      << " h_pos_norm=" << h_pos_norm
                      << " h_iono=" << h_iono
                      << " h_trop=" << h_trop
                      << " h_amb=" << h_amb
                      << " h_amb_eff=" << h_amb_eff
                      << " y=" << measurement.residual
                      << " R=" << r_diag
                      << " dx_state=" << dx_state
                      << " dx_state_cycles=" << dx_state_cycles
                      << " lambda=" << lambda
                      << " state_before_cycles=" << state_before_cycles
                      << " state_after_cycles=" << state_after_cycles
                      << "\n";
        }
    }
}

}  // namespace

AppliedOsrCorrections selectAppliedOsrCorrections(
    const OSRCorrection& osr,
    int freq_index,
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    AppliedOsrCorrections corrections;
    if (freq_index < 0 || freq_index >= osr.num_frequencies) {
        return corrections;
    }

    const double relativity = osr.relativity_correction_m;
    const double receiver_antenna = osr.receiver_antenna_m[freq_index];
    const double code_bias = osr.code_bias_m[freq_index];
    const double phase_bias = osr.phase_bias_m[freq_index];
    const double windup = osr.windup_m[freq_index];
    const double phase_compensation = osr.phase_compensation_m[freq_index];
    const double tide = osr.solid_earth_tide_m;

    switch (policy) {
    case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR:
        corrections.pseudorange_correction_m =
            osr.PRC[freq_index] - osr.trop_correction_m;
        corrections.carrier_phase_correction_m =
            osr.CPC[freq_index] - osr.trop_correction_m;
        break;
    case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS:
        corrections.pseudorange_correction_m =
            relativity + receiver_antenna + code_bias + tide;
        corrections.carrier_phase_correction_m =
            relativity + receiver_antenna + phase_bias + windup + phase_compensation + tide;
        break;
    case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_ONLY:
        corrections.pseudorange_correction_m =
            relativity + receiver_antenna + tide;
        corrections.carrier_phase_correction_m =
            relativity + receiver_antenna + windup + tide;
        break;
    }

    return corrections;
}

bool usesClasTropospherePrior(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    return policy ==
           ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

std::vector<SatelliteId> collectResidualIonoSatellites(
    const ObservationData& obs,
    const SSRProducts& ssr_products) {
    auto supports_clas_residual_iono = [](const SatelliteId& satellite) {
        return satellite.system == GNSSSystem::GPS ||
               satellite.system == GNSSSystem::Galileo ||
               satellite.system == GNSSSystem::QZSS;
    };

    std::set<SatelliteId> unique_satellites;
    for (const auto& satellite : obs.getSatellites()) {
        if (supports_clas_residual_iono(satellite)) {
            unique_satellites.insert(satellite);
        }
    }
    for (const auto& [satellite, _] : ssr_products.orbit_clock_corrections) {
        if (supports_clas_residual_iono(satellite)) {
            unique_satellites.insert(satellite);
        }
    }
    return std::vector<SatelliteId>(unique_satellites.begin(), unique_satellites.end());
}

EpochPreparationResult prepareEpochState(
    const ObservationData& obs,
    const PositionSolution& seed_solution,
    const SSRProducts& ssr_products,
    ppp_shared::PPPState& filter_state,
    bool& filter_initialized,
    GNSSTime& convergence_start_time,
    Vector3d& static_anchor_position,
    bool& has_static_anchor_position,
    const ppp_shared::PPPConfig& config,
    double modeled_zenith_troposphere_delay_m,
    bool has_last_processed_time,
    const GNSSTime& last_processed_time,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance) {
    EpochPreparationResult result;
    if (!filter_initialized) {
        if (!seed_solution.isValid()) {
            return result;
        }
        PositionSolution initial_solution = seed_solution;
        const bool use_receiver_seed_for_static =
            (!config.wlnl_strict_claslib_parity) &&
            (!config.kinematic_mode || config.low_dynamics_mode) &&
            obs.receiver_position.squaredNorm() > 0.0;
        if (use_receiver_seed_for_static) {
            initial_solution.position_ecef = obs.receiver_position;
        }
        const auto iono_satellites =
            config.estimate_ionosphere
                ? collectResidualIonoSatellites(obs, ssr_products)
                : std::vector<SatelliteId>{};
        initializeFilterState(
            filter_state,
            initial_solution,
            obs.time,
            iono_satellites,
            config,
            modeled_zenith_troposphere_delay_m);
        filter_initialized = true;
        convergence_start_time = obs.time;
        static_anchor_position = initial_solution.position_ecef;
        has_static_anchor_position = true;
    }

    const double dt =
        has_last_processed_time ? std::max(obs.time - last_processed_time, 0.001) : 1.0;
    if (config.wlnl_strict_claslib_parity && config.use_ported_udstate) {
        result.ready = true;
        return result;
    }
    syncSlipState(
        obs,
        filter_state,
        ambiguity_states,
        dispersion_compensation,
        phase_bias_repair,
        ambiguity_reset_variance);
    predictFilterState(
        filter_state,
        config,
        dt,
        seed_solution.receiver_clock_bias,
        seed_solution.isValid());
    markSlipCompensationFromAmbiguities(
        obs, ambiguity_states, dispersion_compensation);
    result.ready = true;
    return result;
}

void initializeFilterState(
    ppp_shared::PPPState& filter_state,
    const PositionSolution& seed_solution,
    const GNSSTime& /*time*/,
    const std::vector<SatelliteId>& iono_satellites,
    const ppp_shared::PPPConfig& config,
    double modeled_zenith_troposphere_delay_m) {
    filter_state.gal_clock_index = 9;
    const int isb_end = 10;
    const int base = isb_end + static_cast<int>(iono_satellites.size());
    filter_state.ionosphere_indices.clear();
    filter_state.state = VectorXd::Zero(base);
    filter_state.covariance = MatrixXd::Identity(base, base);
    filter_state.state.segment(0, 3) = seed_solution.position_ecef;
    filter_state.state(filter_state.clock_index) = seed_solution.receiver_clock_bias;
    filter_state.state(filter_state.glo_clock_index) = seed_solution.receiver_clock_bias;
    filter_state.state(filter_state.gal_clock_index) = seed_solution.receiver_clock_bias;
    filter_state.state(filter_state.trop_index) = modeled_zenith_troposphere_delay_m;
    filter_state.covariance.block(0, 0, 3, 3) *= config.clas_initial_position_variance;
    filter_state.covariance(6, 6) = config.clas_clock_variance;
    filter_state.covariance(7, 7) = config.clas_clock_variance;
    filter_state.covariance(8, 8) = config.clas_trop_initial_variance;
    filter_state.covariance(9, 9) = config.clas_clock_variance;
    filter_state.iono_index = isb_end;
    for (size_t index = 0; index < iono_satellites.size(); ++index) {
        const int state_index = filter_state.iono_index + static_cast<int>(index);
        filter_state.ionosphere_indices[iono_satellites[index]] = state_index;
        filter_state.state(state_index) =
            config.wlnl_strict_claslib_parity ? 1e-6 : 0.0;
        filter_state.covariance(state_index, state_index) =
            config.wlnl_strict_claslib_parity
                ? config.initial_ionosphere_variance
                : std::min(config.initial_ionosphere_variance, 1.0);
    }
    filter_state.amb_index = base;
    filter_state.total_states = base;
}

void syncSlipState(
    const ObservationData& obs,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance) {
    for (const auto& satellite : obs.getSatellites()) {
        const auto slip_it = ambiguity_states.find(satellite);
        if (slip_it == ambiguity_states.end() || !slip_it->second.needs_reinitialization) {
            continue;
        }

        const SatelliteId l2_satellite(
            satellite.system,
            static_cast<uint8_t>(std::min(255, satellite.prn + 100)));
        auto reset_ambiguity = [&](const SatelliteId& ambiguity_satellite) {
            auto& ambiguity = ambiguity_states[ambiguity_satellite];
            ambiguity = ppp_shared::PPPAmbiguityInfo{};
            ambiguity.needs_reinitialization = true;

            const auto ambiguity_index_it =
                filter_state.ambiguity_indices.find(ambiguity_satellite);
            if (ambiguity_index_it == filter_state.ambiguity_indices.end()) {
                return;
            }
            const int ambiguity_index = ambiguity_index_it->second;
            if (ambiguity_index < 0 || ambiguity_index >= filter_state.total_states) {
                return;
            }
            filter_state.state(ambiguity_index) = 0.0;
            filter_state.covariance.row(ambiguity_index).setZero();
            filter_state.covariance.col(ambiguity_index).setZero();
            filter_state.covariance(ambiguity_index, ambiguity_index) =
                ambiguity_reset_variance;
            const int real_prn =
                ambiguity_satellite.prn > 100 ? ambiguity_satellite.prn - 100
                                              : ambiguity_satellite.prn;
            if (ppp_shared::pppDebugEnabled() &&
                ambiguity_satellite.system == GNSSSystem::GPS &&
                (real_prn == 14 || real_prn == 25 || real_prn == 26 || real_prn == 29)) {
                std::cerr << "[CLAS-AMB-RESET] sat="
                          << ambiguity_satellite.toString()
                          << " reset_var=" << ambiguity_reset_variance
                          << "\n";
            }
        };

        if (filter_state.ambiguity_indices.find(l2_satellite) !=
            filter_state.ambiguity_indices.end()) {
            reset_ambiguity(l2_satellite);
        }

        dispersion_compensation[satellite].slip = {true, true};
        auto repair_it = phase_bias_repair.find(satellite);
        if (repair_it != phase_bias_repair.end()) {
            repair_it->second.reference_time = GNSSTime();
            repair_it->second.last_continuity_m = {0.0, 0.0, 0.0};
            repair_it->second.offset_cycles = {0.0, 0.0, 0.0};
            repair_it->second.pending_state_shift_cycles = {0.0, 0.0, 0.0};
            repair_it->second.has_last = {false, false, false};
        }
    }
}

void predictFilterState(
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    double dt,
    double seed_receiver_clock_bias_m,
    bool seed_valid) {
    const int nx = filter_state.total_states;
    MatrixXd Q = MatrixXd::Zero(nx, nx);
    if (config.wlnl_strict_claslib_parity) {
        constexpr double kClaslibStaticPositionProcessVariance = 1e-8;
        for (int axis = 0; axis < 3 && axis < nx; ++axis) {
            Q(axis, axis) = kClaslibStaticPositionProcessVariance * std::abs(dt);
        }
    }
    Q(filter_state.clock_index, filter_state.clock_index) = config.clas_clock_variance;
    Q(filter_state.glo_clock_index, filter_state.glo_clock_index) = config.clas_clock_variance;
    if (filter_state.gal_clock_index >= 0) {
        Q(filter_state.gal_clock_index, filter_state.gal_clock_index) = config.clas_clock_variance;
    }
    Q(filter_state.trop_index, filter_state.trop_index) = config.clas_trop_process_noise * dt;
    if (config.estimate_ionosphere) {
        for (const auto& [_, state_index] : filter_state.ionosphere_indices) {
            if (state_index >= 0 && state_index < nx) {
                Q(state_index, state_index) = config.process_noise_ionosphere * dt;
            }
        }
    }
    for (const auto& [_, state_index] : filter_state.ambiguity_indices) {
        if (state_index >= 0 && state_index < nx) {
            Q(state_index, state_index) = config.process_noise_ambiguity * dt;
        }
    }
    filter_state.covariance += Q;
    const bool reset_clock_to_seed =
        seed_valid &&
        config.clas_epoch_policy ==
            ppp_shared::PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK;
    if (reset_clock_to_seed) {
        filter_state.state(filter_state.clock_index) = seed_receiver_clock_bias_m;
        filter_state.state(filter_state.glo_clock_index) = seed_receiver_clock_bias_m;
        if (filter_state.gal_clock_index >= 0) {
            filter_state.state(filter_state.gal_clock_index) = seed_receiver_clock_bias_m;
        }
    }
    // Decouple clock from position: zero cross-covariance to prevent
    // code observation noise from leaking into position via KF coupling.
    if (config.clas_decouple_clock_position) {
        const int ci = filter_state.clock_index;
        const int gi = filter_state.glo_clock_index;
        const int ei = filter_state.gal_clock_index;
        for (int i = 0; i < nx; ++i) {
            if (i != ci) { filter_state.covariance(ci, i) = 0; filter_state.covariance(i, ci) = 0; }
            if (i != gi) { filter_state.covariance(gi, i) = 0; filter_state.covariance(i, gi) = 0; }
            if (ei >= 0 && i != ei) { filter_state.covariance(ei, i) = 0; filter_state.covariance(i, ei) = 0; }
        }
    }
}

void udstatePppPorted(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    double tt,
    const PositionSolution& seed_solution,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance,
    bool debug_enabled) {
    (void)ambiguity_reset_variance;
    const double tt_abs = std::abs(tt);

    udposPppPorted(filter_state, config, seed_solution, tt_abs);
    udclkPppPorted(filter_state, config, seed_solution);
    udtropPppPorted(filter_state, config, tt_abs);
    udbiasPppPorted(
        obs,
        osr_corrections,
        filter_state,
        config,
        tt_abs,
        ambiguity_states,
        dispersion_compensation,
        phase_bias_repair,
        debug_enabled);
    udionPppPorted(
        osr_corrections,
        filter_state,
        config,
        tt_abs,
        ambiguity_states);

    if (debug_enabled) {
        dumpUdstatePorted(obs, filter_state, osr_corrections, ambiguity_states);
    }
}

void markSlipCompensationFromAmbiguities(
    const ObservationData& obs,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation) {
    for (const auto& satellite : obs.getSatellites()) {
        auto& compensation = dispersion_compensation[satellite];
        const auto l1_ambiguity_it = ambiguity_states.find(satellite);
        if (l1_ambiguity_it != ambiguity_states.end() &&
            l1_ambiguity_it->second.needs_reinitialization) {
            compensation.slip[0] = true;
        }
        const SatelliteId l2_satellite(
            satellite.system,
            static_cast<uint8_t>(std::min(255, satellite.prn + 100)));
        const auto l2_ambiguity_it = ambiguity_states.find(l2_satellite);
        if (l2_ambiguity_it != ambiguity_states.end() &&
            l2_ambiguity_it->second.needs_reinitialization) {
            compensation.slip[1] = true;
        }
    }
}

void ensureAmbiguityStates(
    ppp_shared::PPPState& filter_state,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config) {
    constexpr double kClaslibInitialBiasStdCycles = 100.0;
    auto allocate_ambiguity = [&](const SatelliteId& ambiguity_satellite,
                                  double initial_state,
                                  double initial_variance) {
        if (filter_state.ambiguity_indices.find(ambiguity_satellite) !=
            filter_state.ambiguity_indices.end()) {
            return;
        }
        const int new_index = filter_state.total_states;
        filter_state.ambiguity_indices[ambiguity_satellite] = new_index;
        filter_state.total_states++;

        VectorXd new_state = VectorXd::Zero(filter_state.total_states);
        new_state.head(new_index) = filter_state.state;
        new_state(new_index) = initial_state;
        filter_state.state = new_state;

        MatrixXd new_covariance =
            MatrixXd::Zero(filter_state.total_states, filter_state.total_states);
        new_covariance.topLeftCorner(new_index, new_index) = filter_state.covariance;
        new_covariance(new_index, new_index) = initial_variance;
        filter_state.covariance = new_covariance;
    };

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const bool strict_claslib_parity = config.wlnl_strict_claslib_parity;
        const double l1_initial_variance =
            strict_claslib_parity && osr.wavelengths[0] > 0.0
                ? std::pow(kClaslibInitialBiasStdCycles * osr.wavelengths[0], 2)
                : config.initial_ambiguity_variance;
        allocate_ambiguity(
            osr.satellite,
            strict_claslib_parity && !config.use_ported_udstate
                ? std::numeric_limits<double>::quiet_NaN()
                : 0.0,
            l1_initial_variance);
        if (osr.num_frequencies >= 2) {
            const uint8_t l2_prn =
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const double l2_initial_variance =
                strict_claslib_parity && osr.wavelengths[1] > 0.0
                    ? std::pow(kClaslibInitialBiasStdCycles * osr.wavelengths[1], 2)
                    : config.initial_ambiguity_variance;
            allocate_ambiguity(
                SatelliteId(osr.satellite.system, l2_prn),
                strict_claslib_parity && !config.use_ported_udstate
                    ? std::numeric_limits<double>::quiet_NaN()
                    : 0.0,
                l2_initial_variance);
        }
    }
}

void applyPendingPhaseBiasStateShifts(
    ppp_shared::PPPState& filter_state,
    const std::vector<OSRCorrection>& osr_corrections,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    bool debug_enabled) {
    for (const auto& osr : osr_corrections) {
        auto repair_it = phase_bias_repair.find(osr.satellite);
        if (repair_it == phase_bias_repair.end()) {
            continue;
        }
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const double shift_cycles =
                repair_it->second.pending_state_shift_cycles[static_cast<size_t>(f)];
            if (shift_cycles == 0.0 || osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const uint8_t ambiguity_prn = f == 0 ? osr.satellite.prn :
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId ambiguity_satellite(osr.satellite.system, ambiguity_prn);
            const auto ambiguity_it =
                filter_state.ambiguity_indices.find(ambiguity_satellite);
            if (ambiguity_it == filter_state.ambiguity_indices.end()) {
                continue;
            }
            filter_state.state(ambiguity_it->second) += shift_cycles * osr.wavelengths[f];
            repair_it->second.pending_state_shift_cycles[static_cast<size_t>(f)] = 0.0;
            const int real_prn =
                ambiguity_satellite.prn > 100 ? ambiguity_satellite.prn - 100
                                              : ambiguity_satellite.prn;
            if (debug_enabled &&
                ambiguity_satellite.system == GNSSSystem::GPS &&
                (real_prn == 14 || real_prn == 25 || real_prn == 26 || real_prn == 29)) {
                std::cerr << "[CLAS-AMB-SHIFT] sat="
                          << ambiguity_satellite.toString()
                          << " f=" << f
                          << " cycles=" << shift_cycles
                          << " shift_m=" << shift_cycles * osr.wavelengths[f]
                          << " new_state=" << filter_state.state(ambiguity_it->second)
                          << "\n";
            }
            if (debug_enabled) {
                std::cerr << "[CLAS-PBIAS] state shift "
                          << ambiguity_satellite.toString()
                          << " f=" << f
                          << " cycles=" << shift_cycles
                          << " meters=" << shift_cycles * osr.wavelengths[f]
                          << "\n";
            }
        }
    }
}

MeasurementBuildResult buildEpochMeasurements(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position,
    double receiver_clock_m,
    double trop_zenith,
    const std::map<std::string, std::string>& epoch_atmos,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function,
    bool debug_enabled) {
    MeasurementBuildResult result;
    std::map<SatelliteId, double> strict_phase_code_bias_m;
    std::array<double, OSR_MAX_FREQ> strict_com_bias_sum_m{};
    std::array<int, OSR_MAX_FREQ> strict_com_bias_count{};

    if (config.wlnl_strict_claslib_parity) {
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) {
                continue;
            }
            for (int f = 0; f < osr.num_frequencies; ++f) {
                const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
                if (!raw || !raw->valid || !raw->has_carrier_phase ||
                    !std::isfinite(raw->carrier_phase) || !raw->has_pseudorange ||
                    !std::isfinite(raw->pseudorange) || osr.wavelengths[f] <= 0.0) {
                    continue;
                }
                // CLASLIB udbias_ppp() seeds ambiguities from raw phase-code bias
                // before OSR corrections are applied.
                const double bias_m =
                    raw->carrier_phase * osr.wavelengths[f] - raw->pseudorange;
                const uint8_t amb_prn = f == 0 ? osr.satellite.prn
                    : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
                const SatelliteId amb_sat(osr.satellite.system, amb_prn);
                strict_phase_code_bias_m[amb_sat] = bias_m;

                const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
                if (amb_it == filter_state.ambiguity_indices.end()) {
                    continue;
                }
                const int amb_idx = amb_it->second;
                if (!std::isfinite(filter_state.state(amb_idx)) ||
                    filter_state.state(amb_idx) == 0.0 ||
                    filter_state.covariance(amb_idx, amb_idx) >=
                        config.clas_ambiguity_reinit_threshold) {
                    continue;
                }
                strict_com_bias_sum_m[static_cast<size_t>(f)] +=
                    bias_m - filter_state.state(amb_idx);
                strict_com_bias_count[static_cast<size_t>(f)]++;
            }
        }
    }

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }

        const double geo = geodist(osr.satellite_position, receiver_position);
        const double rho_no_sagnac =
            (osr.satellite_position - receiver_position).norm();
        const double sagnac = geo - rho_no_sagnac;
        const Vector3d los =
            (osr.satellite_position - receiver_position).normalized();
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_modeled = trop_mapping * trop_zenith;

        std::array<const Observation*, OSR_MAX_FREQ> raw_observations{};
        const auto iono_state_it = filter_state.ionosphere_indices.find(osr.satellite);
        const int iono_state_index =
            iono_state_it != filter_state.ionosphere_indices.end() ?
                iono_state_it->second : -1;
        const bool use_residual_iono_state =
            config.estimate_ionosphere &&
            iono_state_index >= 0 &&
            iono_state_index < filter_state.total_states;
        for (int f = 0; f < osr.num_frequencies; ++f) {
            raw_observations[static_cast<size_t>(f)] =
                obs.getObservation(osr.satellite, osr.signals[f]);
        }

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const Observation* raw = raw_observations[static_cast<size_t>(f)];
            if (!raw || !raw->valid) {
                continue;
            }
            const auto applied_corrections = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);
            const double iono_scale =
                (osr.frequencies[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            const double iono_mapping =
                config.wlnl_strict_claslib_parity
                    ? claslibIonoMappingFunction(receiver_position, osr.elevation)
                    : 1.0;
            const double iono_coefficient = iono_scale * iono_mapping;
            const double fi_for_grid =
                (osr.wavelengths[0] > 0.0 && osr.wavelengths[f] > 0.0)
                    ? osr.wavelengths[f] / osr.wavelengths[0]
                    : 1.0;
            const double iono_grid_m =
                fi_for_grid * fi_for_grid *
                (constants::GPS_L2_FREQ / constants::GPS_L1_FREQ) *
                osr.iono_l1_m;
            const double iono_state_m =
                use_residual_iono_state ? filter_state.state(iono_state_index) : 0.0;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const double p_corr =
                    raw->pseudorange - applied_corrections.pseudorange_correction_m;
                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + trop_modeled +
                    iono_coefficient * iono_state_m;
                const double residual = p_corr - predicted;

                const double code_variance =
                    clasCodeVarianceForRow(config, osr.satellite.system, osr.elevation);

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                const int clk_idx = (osr.satellite.system == GNSSSystem::Galileo &&
                                     filter_state.gal_clock_index >= 0)
                    ? filter_state.gal_clock_index
                    : (osr.satellite.system == GNSSSystem::GLONASS
                        ? filter_state.glo_clock_index
                        : filter_state.clock_index);
                row.H(clk_idx) = 1.0;
                if (config.estimate_troposphere) {
                    row.H(filter_state.trop_index) = trop_mapping;
                }
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = iono_coefficient;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.residual = residual;
                row.variance = code_variance;
                row.satellite = osr.satellite;
                row.ambiguity_satellite = SatelliteId{};
                row.reference_satellite = SatelliteId{};
                row.is_phase = false;
                row.freq_index = f;
                row.components.valid = true;
                row.components.y = residual;
                row.components.obs = raw->pseudorange;
                row.components.rho = rho_no_sagnac;
                row.components.dts = sat_clk_m;
                row.components.dtr = receiver_clock_m;
                row.components.rel = osr.relativity_correction_m;
                row.components.sagnac = sagnac;
                row.components.trop_dry = trop_modeled;
                row.components.trop_model = trop_modeled;
                row.components.trop_grid = osr.trop_correction_m;
                row.components.iono_grid = iono_grid_m;
                row.components.iono_state_term = iono_coefficient * iono_state_m;
                row.components.prc = osr.PRC[f];
                row.components.osr_corr = applied_corrections.pseudorange_correction_m;
                row.components.code_bias = osr.code_bias_m[f];
                row.components.pcv_rcv = osr.receiver_antenna_m[f];
                row.components.tide_solid =
                    osr.tide_geometry_m != 0.0 ? osr.tide_geometry_m : osr.solid_earth_tide_m;
                row.components.modeled_sum = raw->pseudorange - residual;
                if (ppp_shared::pppDebugEnabled() &&
                    shouldLogClasPhaseRow(osr.satellite)) {
                    std::cerr << "[CLAS-CODE-ROW] tow=" << obs.time.tow
                              << " sat=" << osr.satellite.toString()
                              << " f=" << f
                              << " raw_sig=" << signalObservationCode(raw->signal)
                              << " osr_sig=" << signalObservationCode(osr.signals[f])
                              << " p=" << raw->pseudorange
                              << " code_corr=" << applied_corrections.pseudorange_correction_m
                              << " p_corr=" << p_corr
                              << " geo=" << geo
                              << " sat_clk=" << sat_clk_m
                              << " rcv_clk=" << receiver_clock_m
                              << " trop=" << trop_modeled
                              << " iono_term=" << (iono_coefficient * iono_state_m)
                              << " residual=" << residual
                              << "\n";
                }
                result.measurements.push_back(row);
            }

            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase)) {
                const double l_m = raw->carrier_phase * osr.wavelengths[f];
                const double l_corr =
                    l_m - applied_corrections.carrier_phase_correction_m;

                const uint8_t amb_prn = f == 0 ? osr.satellite.prn
                    : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
                const SatelliteId amb_sat(osr.satellite.system, amb_prn);
                const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
                if (amb_it == filter_state.ambiguity_indices.end()) {
                    continue;
                }
                const int amb_idx = amb_it->second;

                if (raw->loss_of_lock && ambiguity_reset_function) {
                    ambiguity_reset_function(amb_sat, raw->signal);
                }

                if (!std::isfinite(filter_state.state(amb_idx)) ||
                    filter_state.covariance(amb_idx, amb_idx) >=
                        config.clas_ambiguity_reinit_threshold) {
                    double seed =
                        l_corr - (geo - sat_clk_m + receiver_clock_m + trop_modeled
                                  - iono_coefficient * iono_state_m);
                    if (config.wlnl_strict_claslib_parity) {
                        const auto bias_it = strict_phase_code_bias_m.find(amb_sat);
                        if (bias_it != strict_phase_code_bias_m.end()) {
                            const int bias_count =
                                strict_com_bias_count[static_cast<size_t>(f)];
                            const double com_bias_m =
                                bias_count > 0
                                    ? strict_com_bias_sum_m[static_cast<size_t>(f)] /
                                          static_cast<double>(bias_count)
                                    : 0.0;
                            seed = bias_it->second - com_bias_m;
                        }
                    }
                    filter_state.state(amb_idx) = seed;
                    if (config.wlnl_strict_claslib_parity &&
                        ppp_shared::pppDebugEnabled() &&
                        shouldLogClasAmbSeed(osr.satellite, obs.time.tow)) {
                        const auto bias_it = strict_phase_code_bias_m.find(amb_sat);
                        const int bias_count =
                            strict_com_bias_count[static_cast<size_t>(f)];
                        const double com_bias_m =
                            bias_count > 0
                                ? strict_com_bias_sum_m[static_cast<size_t>(f)] /
                                      static_cast<double>(bias_count)
                                : 0.0;
                        const double bias_raw_m =
                            raw->carrier_phase * osr.wavelengths[f] - raw->pseudorange;
                        const double seed_cycles =
                            osr.wavelengths[f] > 0.0
                                ? (bias_raw_m - com_bias_m) / osr.wavelengths[f]
                                : std::numeric_limits<double>::quiet_NaN();
                        const double state_cycles_after_seed =
                            osr.wavelengths[f] > 0.0
                                ? filter_state.state(amb_idx) / osr.wavelengths[f]
                                : std::numeric_limits<double>::quiet_NaN();
                        std::cerr << "[CLAS-AMB-SEED] tow=" << obs.time.tow
                                  << " sat=" << osr.satellite.toString()
                                  << " sat_id=" << osr.satellite.toString()
                                  << " amb_sat=" << amb_sat.toString()
                                  << " f=" << f
                                  << " freq_group=" << f
                                  << std::setprecision(12)
                                  << " raw_cp=" << raw->carrier_phase
                                  << " raw_pr=" << raw->pseudorange
                                  << " lambda=" << osr.wavelengths[f]
                                  << " bias_raw=" << bias_raw_m
                                  << " bias_raw_m=" << bias_raw_m
                                  << " com_bias_m=" << com_bias_m
                                  << " seed_cycles=" << seed_cycles
                                  << " state_cycles_after_seed=" << state_cycles_after_seed
                                  << " l_corr=" << l_corr
                                  << " geo=" << geo
                                  << " sat_clk=" << sat_clk_m
                                  << " rcv_clk=" << receiver_clock_m
                                  << " trop=" << trop_modeled
                                  << " iono_term=" << (-iono_coefficient * iono_state_m)
                                  << " phase_code_bias="
                                  << (bias_it != strict_phase_code_bias_m.end() ?
                                          bias_it->second : std::numeric_limits<double>::quiet_NaN())
                                  << " seed=" << filter_state.state(amb_idx)
                                  << " cov=" << filter_state.covariance(amb_idx, amb_idx)
                                  << "\n";
                    }
                }

                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + trop_modeled
                    - iono_coefficient * iono_state_m + filter_state.state(amb_idx);
                const double residual = l_corr - predicted;

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                const int clk_idx_ph = (osr.satellite.system == GNSSSystem::Galileo &&
                                        filter_state.gal_clock_index >= 0)
                    ? filter_state.gal_clock_index
                    : (osr.satellite.system == GNSSSystem::GLONASS
                        ? filter_state.glo_clock_index
                        : filter_state.clock_index);
                row.H(clk_idx_ph) = 1.0;
                if (config.estimate_troposphere) {
                    row.H(filter_state.trop_index) = trop_mapping;
                }
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = -iono_coefficient;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.H(amb_idx) = 1.0;
                row.residual = residual;
                row.variance =
                    clasPhaseVarianceForRow(config, osr.satellite.system, osr.elevation, f);
                row.satellite = osr.satellite;
                row.ambiguity_satellite = amb_sat;
                row.reference_satellite = SatelliteId{};
                row.is_phase = true;
                row.freq_index = f;
                row.components.valid = true;
                row.components.y = residual;
                row.components.obs = l_m;
                row.components.rho = rho_no_sagnac;
                row.components.dts = sat_clk_m;
                row.components.dtr = receiver_clock_m;
                row.components.rel = osr.relativity_correction_m;
                row.components.sagnac = sagnac;
                row.components.trop_dry = trop_modeled;
                row.components.trop_model = trop_modeled;
                row.components.trop_grid = osr.trop_correction_m;
                row.components.iono_grid = -iono_grid_m;
                row.components.iono_state_term = -iono_coefficient * iono_state_m;
                row.components.cpc = osr.CPC[f];
                row.components.osr_corr = applied_corrections.carrier_phase_correction_m;
                row.components.phase_bias = osr.phase_bias_m[f];
                row.components.phase_comp = osr.phase_compensation_m[f];
                row.components.windup = osr.windup_m[f];
                row.components.pcv_rcv = osr.receiver_antenna_m[f];
                row.components.tide_solid =
                    osr.tide_geometry_m != 0.0 ? osr.tide_geometry_m : osr.solid_earth_tide_m;
                row.components.amb_m = filter_state.state(amb_idx);
                row.components.modeled_sum = l_m - residual;
                if (ppp_shared::pppDebugEnabled() &&
                    shouldLogClasPhaseRow(osr.satellite)) {
                    std::cerr << "[CLAS-PHASE-ROW] tow=" << obs.time.tow
                              << " sat=" << osr.satellite.toString()
                              << " f=" << f
                              << " raw_sig=" << signalObservationCode(raw->signal)
                              << " osr_sig=" << signalObservationCode(osr.signals[f])
                              << " carrier_obs=" << raw->carrier_observation_type
                              << " code_obs=" << raw->pseudorange_observation_type
                              << " l_m=" << l_m
                              << " phase_corr=" << applied_corrections.carrier_phase_correction_m
                              << " l_corr=" << l_corr
                              << " geo=" << geo
                              << " sat_clk=" << sat_clk_m
                              << " rcv_clk=" << receiver_clock_m
                              << " trop=" << trop_modeled
                              << " iono_term=" << (-iono_coefficient * iono_state_m)
                              << " amb=" << filter_state.state(amb_idx)
                              << " residual=" << residual
                              << "\n";
                }
                result.measurements.push_back(row);
                result.observed_ambiguities.push_back(
                    {amb_sat, raw->signal, osr.wavelengths[f], raw->carrier_phase, raw->snr});
            }
        }
    }

    if (config.estimate_troposphere && !epoch_atmos.empty() &&
        usesClasTropospherePrior(config.clas_correction_application_policy)) {
        const double clas_trop_zenith =
            ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
                epoch_atmos,
                receiver_position,
                obs.time,
                M_PI_2,
                config.clas_expanded_value_construction_policy,
                config.clas_subtype12_value_construction_policy,
                config.clas_expanded_residual_sampling_policy);
        if (std::isfinite(clas_trop_zenith) && clas_trop_zenith > 0.0) {
            MeasurementRow row;
            row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
            row.H(filter_state.trop_index) = 1.0;
            row.residual = clas_trop_zenith - trop_zenith;
            row.variance = config.clas_trop_prior_variance;
            row.satellite = SatelliteId{};
            row.is_phase = false;
            row.freq_index = -1;
            result.measurements.push_back(row);
            if (debug_enabled) {
                std::cerr << "[CLAS-TROP] prior=" << clas_trop_zenith
                          << " state=" << trop_zenith << "\n";
            }
        }
    }

    if (config.estimate_ionosphere) {
        for (const auto& satellite : result.observed_iono_states) {
            const auto iono_it = filter_state.ionosphere_indices.find(satellite);
            if (iono_it == filter_state.ionosphere_indices.end()) {
                continue;
            }
            const int iono_state_index = iono_it->second;
            if (iono_state_index < 0 || iono_state_index >= filter_state.total_states) {
                continue;
            }
            MeasurementRow row;
            row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
            row.H(iono_state_index) = 1.0;
            row.residual = -filter_state.state(iono_state_index);
            row.variance = config.clas_iono_prior_variance;
            row.satellite = satellite;
            row.is_phase = false;
            row.freq_index = -1;
            result.measurements.push_back(row);
        }
    }

    // CLASLIB forms single-differenced observation rows before the filter
    // update. Keep the existing mixed mode by default, but in strict parity
    // mode also single-difference code rows.
    if (config.use_clas_osr_filter) {
        // Build elevation map for reference satellite selection
        std::map<SatelliteId, double> elevation_map;
        for (const auto& osr : osr_corrections) {
            if (osr.valid) {
                elevation_map[osr.satellite] = osr.elevation;
            }
        }

        // Group phase measurements by GNSS system and frequency
        struct SdGroupKey {
            GNSSSystem system;
            int freq_index;
            bool is_phase;
            bool operator<(const SdGroupKey& rhs) const {
                if (system != rhs.system) return system < rhs.system;
                if (freq_index != rhs.freq_index) return freq_index < rhs.freq_index;
                return is_phase < rhs.is_phase;
            }
        };
        std::map<SdGroupKey, std::vector<size_t>> sd_groups;
        for (size_t i = 0; i < result.measurements.size(); ++i) {
            const auto& m = result.measurements[i];
            if (m.freq_index < 0) continue;
            if (!config.wlnl_strict_claslib_parity && !m.is_phase) continue;
            sd_groups[{m.satellite.system, m.freq_index, m.is_phase}].push_back(i);
        }

        // Keep only non-observation priors unless a row is intentionally left UD.
        std::vector<MeasurementRow> sd_measurements;
        for (size_t i = 0; i < result.measurements.size(); ++i) {
            const auto& m = result.measurements[i];
            const bool keep_undifferenced =
                m.freq_index < 0 ||
                (!config.wlnl_strict_claslib_parity && !m.is_phase);
            if (keep_undifferenced) {
                sd_measurements.push_back(m);
            }
        }

        for (auto& [group_key, member_indices] : sd_groups) {
            if (member_indices.size() < 2) continue;

            // Select reference satellite: highest elevation
            size_t ref_index = member_indices[0];
            double max_elevation = -1.0;
            for (size_t idx : member_indices) {
                auto el_it = elevation_map.find(result.measurements[idx].satellite);
                if (el_it != elevation_map.end() && el_it->second > max_elevation) {
                    max_elevation = el_it->second;
                    ref_index = idx;
                }
            }

            // Form SD: reference - satellite
            const auto& ref_row = result.measurements[ref_index];
            for (size_t idx : member_indices) {
                if (idx == ref_index) continue;
                const auto& sat_row = result.measurements[idx];
                MeasurementRow sd_row;
                sd_row.H = ref_row.H - sat_row.H;
                sd_row.residual = ref_row.residual - sat_row.residual;
                sd_row.variance = ref_row.variance + sat_row.variance;
                sd_row.reference_variance = ref_row.variance;
                sd_row.satellite = sat_row.satellite;
                sd_row.ambiguity_satellite =
                    group_key.is_phase ? sat_row.ambiguity_satellite : SatelliteId{};
                sd_row.reference_satellite =
                    group_key.is_phase ? ref_row.ambiguity_satellite : ref_row.satellite;
                sd_row.is_phase = group_key.is_phase;
                sd_row.freq_index = group_key.freq_index;
                sd_row.components =
                    subtractModelComponents(ref_row.components, sat_row.components);
                sd_row.components.y = sd_row.residual;
                if (std::isfinite(sd_row.components.obs)) {
                    sd_row.components.modeled_sum =
                        sd_row.components.obs - sd_row.residual;
                }
                sd_measurements.push_back(sd_row);
            }
        }
        result.measurements = std::move(sd_measurements);
    }

    return result;
}

KalmanUpdateStats applyMeasurementUpdate(
    ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const ppp_shared::PPPConfig& config,
    const PositionSolution* seed_solution,
    const GNSSTime* observation_time) {
    KalmanUpdateStats stats;
    stats.nobs = static_cast<int>(measurements.size());
    if (stats.nobs < 4) {
        return stats;
    }

    MatrixXd H = MatrixXd::Zero(stats.nobs, filter_state.total_states);
    VectorXd z = VectorXd::Zero(stats.nobs);
    MatrixXd R = MatrixXd::Zero(stats.nobs, stats.nobs);
    std::vector<double> diagonal_variances(static_cast<size_t>(stats.nobs), 0.0);
    std::vector<double> reference_variances(static_cast<size_t>(stats.nobs), 0.0);

    for (int i = 0; i < stats.nobs; ++i) {
        H.row(i) = measurements[static_cast<size_t>(i)].H;
        z(i) = measurements[static_cast<size_t>(i)].residual;
        diagonal_variances[static_cast<size_t>(i)] =
            measurements[static_cast<size_t>(i)].variance;
        reference_variances[static_cast<size_t>(i)] =
            measurements[static_cast<size_t>(i)].reference_variance;
    }

    const auto fresh_ambiguity_indices =
        collectFreshAmbiguityIndices(filter_state, config);
    if (!config.wlnl_strict_claslib_parity && !fresh_ambiguity_indices.empty()) {
        for (int i = 0; i < stats.nobs; ++i) {
            if (measurements[static_cast<size_t>(i)].is_phase) {
                diagonal_variances[static_cast<size_t>(i)] *= kStartupPhaseVarianceInflation;
                reference_variances[static_cast<size_t>(i)] *= kStartupPhaseVarianceInflation;
            }
        }
    }

    if (!config.wlnl_strict_claslib_parity) {
        for (int i = 0; i < stats.nobs; ++i) {
            const double sigma = std::sqrt(diagonal_variances[static_cast<size_t>(i)]);
            if (std::abs(z(i)) > config.clas_outlier_sigma_scale * sigma) {
                diagonal_variances[static_cast<size_t>(i)] = 1e10;
                reference_variances[static_cast<size_t>(i)] = 0.0;
            }
        }
    }

    for (int i = 0; i < stats.nobs; ++i) {
        R(i, i) = diagonal_variances[static_cast<size_t>(i)];
    }
    if (config.wlnl_strict_claslib_parity) {
        for (int i = 0; i < stats.nobs; ++i) {
            const auto& lhs = measurements[static_cast<size_t>(i)];
            if (lhs.reference_satellite.prn == 0 ||
                reference_variances[static_cast<size_t>(i)] <= 0.0) {
                continue;
            }
            for (int j = i + 1; j < stats.nobs; ++j) {
                const auto& rhs = measurements[static_cast<size_t>(j)];
                if (rhs.is_phase != lhs.is_phase ||
                    !(rhs.reference_satellite == lhs.reference_satellite) ||
                    rhs.freq_index != lhs.freq_index ||
                    reference_variances[static_cast<size_t>(j)] <= 0.0) {
                    continue;
                }
                const double common_reference_variance =
                    0.5 * (reference_variances[static_cast<size_t>(i)] +
                           reference_variances[static_cast<size_t>(j)]);
                R(i, j) = common_reference_variance;
                R(j, i) = common_reference_variance;
            }
        }
    }

    const MatrixXd covariance_before_update = filter_state.covariance;
    MatrixXd PHt = covariance_before_update * H.transpose();
    MatrixXd K = MatrixXd::Zero(filter_state.total_states, stats.nobs);
    VectorXd dx = VectorXd::Zero(filter_state.total_states);
    MatrixXd covariance_after_update = covariance_before_update;

    if (config.wlnl_strict_claslib_parity) {
        std::vector<int> active_indices;
        active_indices.reserve(static_cast<size_t>(filter_state.total_states));
        for (int i = 0; i < filter_state.total_states; ++i) {
            if (std::isfinite(filter_state.state(i)) &&
                filter_state.state(i) != 0.0 &&
                covariance_before_update(i, i) > 0.0) {
                active_indices.push_back(i);
            }
        }
        const int active_count = static_cast<int>(active_indices.size());
        if (active_count == 0) {
            return stats;
        }

        MatrixXd active_covariance = MatrixXd::Zero(active_count, active_count);
        MatrixXd active_H = MatrixXd::Zero(stats.nobs, active_count);
        for (int i = 0; i < active_count; ++i) {
            const int source_i = active_indices[static_cast<size_t>(i)];
            active_H.col(i) = H.col(source_i);
            for (int j = 0; j < active_count; ++j) {
                const int source_j = active_indices[static_cast<size_t>(j)];
                active_covariance(i, j) = covariance_before_update(source_i, source_j);
            }
        }

        const MatrixXd active_PHt = active_covariance * active_H.transpose();
        const MatrixXd innovation_covariance = active_H * active_PHt + R;
        Eigen::LDLT<MatrixXd> innovation_ldlt(innovation_covariance);
        if (innovation_ldlt.info() != Eigen::Success) {
            return stats;
        }

        const MatrixXd active_K =
            active_PHt * innovation_ldlt.solve(MatrixXd::Identity(stats.nobs, stats.nobs));
        const VectorXd active_dx = active_K * z;
        const MatrixXd active_I_KH =
            MatrixXd::Identity(active_count, active_count) - active_K * active_H;
        const MatrixXd active_covariance_after = active_I_KH * active_covariance;

        PHt.setZero();
        for (int i = 0; i < active_count; ++i) {
            const int target_i = active_indices[static_cast<size_t>(i)];
            dx(target_i) = active_dx(i);
            K.row(target_i) = active_K.row(i);
            PHt.row(target_i) = active_PHt.row(i);
            for (int j = 0; j < active_count; ++j) {
                const int target_j = active_indices[static_cast<size_t>(j)];
                covariance_after_update(target_i, target_j) =
                    active_covariance_after(i, j);
            }
        }
    } else {
        PHt = covariance_before_update * H.transpose();
        const MatrixXd S = H * covariance_before_update * H.transpose() + R;
        K = PHt * S.inverse();
        dx = K * z;
        const MatrixXd I_KH =
            MatrixXd::Identity(filter_state.total_states, filter_state.total_states) - K * H;
        covariance_after_update =
            I_KH * filter_state.covariance * I_KH.transpose() + K * R * K.transpose();
    }
    dumpClasUpdateRows(
        filter_state,
        measurements,
        R,
        dx,
        config,
        observation_time);
    if (ppp_shared::pppDebugEnabled() &&
        observation_time != nullptr &&
        std::abs(observation_time->tow - 230420.0) < 1e-6) {
        for (const auto& [amb_sat, amb_idx] : filter_state.ambiguity_indices) {
            const bool dump_gps_amb =
                amb_sat.system == GNSSSystem::GPS &&
                (amb_sat.prn == 14 || amb_sat.prn == 25 ||
                 amb_sat.prn == 26 || amb_sat.prn == 29 ||
                 amb_sat.prn == 31 || amb_sat.prn == 32 ||
                 amb_sat.prn == 114 || amb_sat.prn == 125 ||
                 amb_sat.prn == 126 || amb_sat.prn == 129 ||
                 amb_sat.prn == 131 || amb_sat.prn == 132);
            const bool dump_gal_amb =
                amb_sat.system == GNSSSystem::Galileo &&
                (amb_sat.prn == 7 || amb_sat.prn == 27 ||
                 amb_sat.prn == 107 || amb_sat.prn == 127);
            if (!dump_gps_amb && !dump_gal_amb) {
                continue;
            }
            SatelliteId iono_sat = amb_sat;
            if (iono_sat.prn >= 100) {
                iono_sat.prn = static_cast<uint8_t>(iono_sat.prn - 100);
            }
            int iono_idx = -1;
            const auto iono_it = filter_state.ionosphere_indices.find(iono_sat);
            if (iono_it != filter_state.ionosphere_indices.end()) {
                iono_idx = iono_it->second;
            }
            const double cov_clk =
                (filter_state.clock_index >= 0 &&
                 filter_state.clock_index < covariance_before_update.cols())
                    ? covariance_before_update(amb_idx, filter_state.clock_index)
                    : 0.0;
            const double cov_trop =
                (filter_state.trop_index >= 0 &&
                 filter_state.trop_index < covariance_before_update.cols())
                    ? covariance_before_update(amb_idx, filter_state.trop_index)
                    : 0.0;
            const double cov_iono =
                (iono_idx >= 0 && iono_idx < covariance_before_update.cols())
                    ? covariance_before_update(amb_idx, iono_idx)
                    : 0.0;
            const double cov_pos_norm =
                covariance_before_update.row(amb_idx).segment(0, 3).norm();
            std::cerr << "[CLAS-COV-AMB] tow=" << observation_time->tow
                      << " amb_sat=" << amb_sat.toString()
                      << " var=" << covariance_before_update(amb_idx, amb_idx)
                      << " cov_pos_norm=" << cov_pos_norm
                      << " cov_clk=" << cov_clk
                      << " cov_trop=" << cov_trop
                      << " cov_iono=" << cov_iono
                      << "\n";
            double dx_sum = 0.0;
            for (int i = 0; i < stats.nobs; ++i) {
                const double contrib = K(amb_idx, i) * z(i);
                const auto& m = measurements[static_cast<size_t>(i)];
                const double ph_pos =
                    covariance_before_update.row(amb_idx).segment(0, 3).dot(
                        m.H.segment(0, 3));
                const double ph_clk =
                    (filter_state.clock_index >= 0 &&
                     filter_state.clock_index < covariance_before_update.cols())
                        ? covariance_before_update(amb_idx, filter_state.clock_index) *
                              m.H(filter_state.clock_index)
                        : 0.0;
                const double ph_trop =
                    (filter_state.trop_index >= 0 &&
                     filter_state.trop_index < covariance_before_update.cols())
                        ? covariance_before_update(amb_idx, filter_state.trop_index) *
                              m.H(filter_state.trop_index)
                        : 0.0;
                const double ph_iono =
                    (iono_idx >= 0 && iono_idx < covariance_before_update.cols())
                        ? covariance_before_update(amb_idx, iono_idx) * m.H(iono_idx)
                        : 0.0;
                const double ph_amb =
                    covariance_before_update(amb_idx, amb_idx) * m.H(amb_idx);
                if (std::abs(contrib) < 1e-4 && std::abs(PHt(amb_idx, i)) < 1e-4) {
                    continue;
                }
                const char* row_kind = "unknown";
                if (m.is_phase && m.freq_index >= 0) {
                    row_kind = "phase-sd";
                } else if (!m.is_phase && m.freq_index >= 0) {
                    row_kind = "code-ud";
                } else if (!m.is_phase && m.freq_index < 0 &&
                           m.satellite.system == GNSSSystem::GPS &&
                           m.satellite.prn != 0) {
                    row_kind = "iono-prior";
                } else if (!m.is_phase && m.freq_index < 0 &&
                           m.satellite.prn == 0) {
                    row_kind = "trop-prior";
                }
                std::cerr << "[CLAS-K-AMB] source=lib"
                          << " week=" << observation_time->week
                          << " tow=" << observation_time->tow
                          << " amb_sat=" << amb_sat.toString()
                          << " row=" << i
                          << " kind=" << row_kind
                          << " row_sat=" << m.satellite.toString()
                          << " row_ref=" << m.reference_satellite.toString()
                          << " is_phase=" << (m.is_phase ? 1 : 0)
                          << " f=" << m.freq_index
                          << " h_clk=" << m.H(filter_state.clock_index)
                          << " h_trop=" << m.H(filter_state.trop_index)
                          << " h_amb=" << m.H(amb_idx)
                          << " ph=" << PHt(amb_idx, i)
                          << " ph_pos=" << ph_pos
                          << " ph_clk=" << ph_clk
                          << " ph_trop=" << ph_trop
                          << " ph_iono=" << ph_iono
                          << " ph_amb=" << ph_amb
                          << " z=" << z(i)
                          << " K=" << K(amb_idx, i)
                          << " contrib=" << contrib
                          << "\n";
                dx_sum += contrib;
            }
            std::cerr << "[CLAS-K-AMB-SUM] tow=" << observation_time->tow
                      << " amb_sat=" << amb_sat.toString()
                      << " old=" << filter_state.state(amb_idx)
                      << " dx=" << dx_sum
                      << " new=" << (filter_state.state(amb_idx) + dx_sum)
                      << "\n";
        }
    }
    filter_state.state += dx;
    filter_state.covariance = covariance_after_update;
    if (config.wlnl_strict_claslib_parity) {
        filter_state.covariance =
            0.5 * (filter_state.covariance + filter_state.covariance.transpose());
    }

    stats.updated = true;
    stats.dx = dx;
    stats.residuals = z;
    stats.variances = R.diagonal();
    stats.pre_anchor_covariance = filter_state.covariance;

    const bool use_seed_anchor =
        seed_solution != nullptr && seed_solution->isValid() &&
        !config.wlnl_strict_claslib_parity &&
        (config.clas_epoch_policy ==
             ppp_shared::PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK ||
         !config.kinematic_mode || config.low_dynamics_mode);
    if (use_seed_anchor) {
        const double anchor_sigma = config.clas_anchor_sigma;
        for (int axis = 0; axis < 3; ++axis) {
            const int idx = axis;
            const double innovation =
                seed_solution->position_ecef(axis) - filter_state.state(idx);
            const double innovation_covariance =
                filter_state.covariance(idx, idx) + anchor_sigma * anchor_sigma;
            if (innovation_covariance > 0.0) {
                VectorXd K_col = filter_state.covariance.col(idx) / innovation_covariance;
                filter_state.state += K_col * innovation;
                filter_state.covariance -= K_col * filter_state.covariance.row(idx);
            }
        }
    }

    return stats;
}

EpochUpdateResult runEpochMeasurementUpdate(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const PositionSolution& seed_solution,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function,
    const AmbiguityIndexFunction& ambiguity_index_function,
    bool debug_enabled) {
    EpochUpdateResult result;
    auto measurement_build_result = buildEpochMeasurements(
        obs,
        epoch_context.osr_corrections,
        filter_state,
        config,
        epoch_context.receiver_position,
        epoch_context.receiver_clock_m,
        epoch_context.trop_zenith_m,
        epoch_context.epoch_atmos_tokens,
        trop_mapping_function,
        ambiguity_reset_function,
        debug_enabled);

    if (ppp_shared::pppDebugEnabled()) {
        int phase_rows = 0;
        int code_rows = 0;
        int prior_rows = 0;
        for (const auto& row : measurement_build_result.measurements) {
            if (row.is_phase && row.freq_index >= 0) {
                ++phase_rows;
            } else if (!row.is_phase && row.freq_index >= 0) {
                ++code_rows;
            } else {
                ++prior_rows;
            }
        }
        std::cerr << "[CLAS-MEAS-BUILD] tow=" << obs.time.tow
                  << " total=" << measurement_build_result.measurements.size()
                  << " phase=" << phase_rows
                  << " code=" << code_rows
                  << " prior=" << prior_rows
                  << " amb_obs=" << measurement_build_result.observed_ambiguities.size()
                  << "\n";
    }

    if (measurement_build_result.measurements.size() < 5) {
        return result;
    }

    result.update_stats = applyMeasurementUpdate(
        filter_state,
        measurement_build_result.measurements,
        config,
        &seed_solution,
        &obs.time);
    if (!result.update_stats.updated) {
        return result;
    }

    for (int i = 0; i < static_cast<int>(measurement_build_result.measurements.size()); ++i) {
        const auto& measurement = measurement_build_result.measurements[static_cast<size_t>(i)];
        if (!measurement.is_phase || result.update_stats.variances.size() <= i) {
            continue;
        }
        const double variance = result.update_stats.variances(i);
        const bool accepted = variance < 1e9;
        if (ppp_shared::pppDebugEnabled() &&
            (measurement.ambiguity_satellite.prn != 0 ||
             measurement.reference_satellite.prn != 0)) {
            std::cerr << "[CLAS-AR-ROW] tow=" << obs.time.tow
                      << " sat=" << measurement.ambiguity_satellite.toString()
                      << " ref=" << measurement.reference_satellite.toString()
                      << " freq=" << measurement.freq_index
                      << " residual=" << result.update_stats.residuals(i)
                      << " variance=" << variance
                      << " accepted=" << (accepted ? 1 : 0)
                      << "\n";
        }
        if (!accepted) {
            continue;
        }
        if (measurement.ambiguity_satellite.prn != 0) {
            result.ar_phase_ambiguities.insert(measurement.ambiguity_satellite);
        }
        if (measurement.reference_satellite.prn != 0) {
            result.ar_phase_ambiguities.insert(measurement.reference_satellite);
        }
    }

    updateObservedAmbiguities(
        obs.time,
        measurement_build_result.observed_ambiguities,
        filter_state,
        ambiguity_states,
        ambiguity_index_function);
    result.updated = true;
    return result;
}

void updateObservedAmbiguities(
    const GNSSTime& time,
    const std::vector<AmbiguityObservation>& observed_ambiguities,
    const ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const AmbiguityIndexFunction& ambiguity_index_function) {
    for (const auto& ambiguity_obs : observed_ambiguities) {
        auto& ambiguity = ambiguity_states[ambiguity_obs.ambiguity_satellite];
        const double old_float_value = ambiguity.float_value;
        ambiguity.last_phase = ambiguity_obs.carrier_phase_cycles;
        ambiguity.last_time = time;
        ambiguity.lock_count += 1;
        ambiguity.outage_count = 0;
        ambiguity.quality_indicator = ambiguity_obs.snr;
        ambiguity.ambiguity_scale_m = ambiguity_obs.wavelength_m;
        ambiguity.needs_reinitialization = false;
        const int ambiguity_index =
            ambiguity_index_function(ambiguity_obs.ambiguity_satellite);
        if (ambiguity_index >= 0 && ambiguity_index < filter_state.total_states) {
            ambiguity.float_value = filter_state.state(ambiguity_index);
            if (ppp_shared::pppDebugEnabled() &&
                ambiguity_obs.ambiguity_satellite.system == GNSSSystem::GPS &&
                (ambiguity_obs.ambiguity_satellite.prn == 14 ||
                ambiguity_obs.ambiguity_satellite.prn == 25 ||
                 ambiguity_obs.ambiguity_satellite.prn == 26 ||
                 ambiguity_obs.ambiguity_satellite.prn == 29 ||
                 ambiguity_obs.ambiguity_satellite.prn == 31 ||
                 ambiguity_obs.ambiguity_satellite.prn == 32 ||
                 ambiguity_obs.ambiguity_satellite.prn == 114 ||
                 ambiguity_obs.ambiguity_satellite.prn == 125 ||
                 ambiguity_obs.ambiguity_satellite.prn == 126 ||
                 ambiguity_obs.ambiguity_satellite.prn == 129 ||
                 ambiguity_obs.ambiguity_satellite.prn == 131 ||
                 ambiguity_obs.ambiguity_satellite.prn == 132)) {
                std::cerr << "[CLAS-AMB-OBS] tow=" << time.tow
                          << " amb_sat=" << ambiguity_obs.ambiguity_satellite.toString()
                          << " old=" << old_float_value
                          << " new=" << ambiguity.float_value
                          << " lock=" << ambiguity.lock_count
                          << "\n";
            }
        }
    }
}

FixValidationStats validateFixedSolution(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityIndexFunction& ambiguity_index_function,
    bool debug_enabled) {
    FixValidationStats stats;
    double phase_sum_sq = 0.0;
    double code_sum_sq = 0.0;
    double phase_chi_sq = 0.0;
    bool pair_validation_ok = true;
    SatelliteId worst_reference_satellite;
    SatelliteId worst_satellite;
    int worst_freq_group = -1;
    double worst_dd_residual_m = 0.0;
    double worst_dd_sigma = 0.0;
    SatelliteId worst_pair_satellite;
    double worst_pair_dispersive = 0.0;
    double worst_pair_nondispersive = 0.0;
    double worst_pair_sigma = 0.0;

    std::vector<PhaseResidualInfo> phase_residuals;

    const Vector3d receiver_position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const double receiver_clock_m = filter_state.state(filter_state.clock_index);
    const double trop_zenith = filter_state.state(filter_state.trop_index);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }

        const double geo = geodist(osr.satellite_position, receiver_position);
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_modeled = trop_mapping * trop_zenith;

        std::array<const Observation*, OSR_MAX_FREQ> raw_observations{};
        for (int f = 0; f < osr.num_frequencies; ++f) {
            raw_observations[static_cast<size_t>(f)] =
                obs.getObservation(osr.satellite, osr.signals[f]);
        }

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const Observation* raw = raw_observations[static_cast<size_t>(f)];
            if (raw == nullptr || !raw->valid) {
                continue;
            }
            const auto applied_corrections = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);

            const auto iono_it = filter_state.ionosphere_indices.find(osr.satellite);
            const double iono_scale =
                (config.estimate_ionosphere &&
                 iono_it != filter_state.ionosphere_indices.end() &&
                 iono_it->second >= 0 &&
                 iono_it->second < filter_state.total_states &&
                 osr.frequencies[f] > 0.0 &&
                 osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 0.0;
            const double iono_state_m =
                iono_scale > 0.0 ? filter_state.state(iono_it->second) : 0.0;

            const double predicted =
                geo - sat_clk_m + receiver_clock_m + trop_modeled;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const double residual =
                    (raw->pseudorange - applied_corrections.pseudorange_correction_m) -
                    (predicted + iono_scale * iono_state_m);
                code_sum_sq += residual * residual;
                ++stats.code_rows;
            }

            if (!raw->has_carrier_phase || !std::isfinite(raw->carrier_phase)) {
                continue;
            }

            const uint8_t ambiguity_prn = f == 0 ? osr.satellite.prn :
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId ambiguity_satellite(osr.satellite.system, ambiguity_prn);
            const int ambiguity_index = ambiguity_index_function(ambiguity_satellite);
            if (ambiguity_index < 0 || ambiguity_index >= filter_state.total_states) {
                continue;
            }

            const double carrier_phase_m = raw->carrier_phase * osr.wavelengths[f];
            const double residual =
                (carrier_phase_m - applied_corrections.carrier_phase_correction_m) -
                (predicted - iono_scale * iono_state_m +
                 filter_state.state(ambiguity_index));
            const double variance = config.clas_phase_variance * elevationWeight(osr.elevation);
            phase_residuals.push_back({
                ambiguity_satellite,
                osr.satellite,
                residual,
                variance,
                osr.frequencies[f],
                osr.wavelengths[f]});
        }
    }

    std::map<std::pair<GNSSSystem, int>, std::vector<PhaseResidualInfo>> dd_groups;
    for (const auto& phase_residual : phase_residuals) {
        dd_groups[ppp_ar::ambiguityDdGroup(phase_residual.ambiguity_satellite)]
            .push_back(phase_residual);
    }

    std::map<SatelliteId, PhasePairInfo> phase_pairs;
    for (auto& [group, residuals] : dd_groups) {
        std::sort(residuals.begin(), residuals.end(),
                  [](const PhaseResidualInfo& lhs, const PhaseResidualInfo& rhs) {
                      if (lhs.real_satellite.system != rhs.real_satellite.system) {
                          return static_cast<int>(lhs.real_satellite.system) <
                                 static_cast<int>(rhs.real_satellite.system);
                      }
                      return lhs.real_satellite.prn < rhs.real_satellite.prn;
                  });
        if (residuals.size() < 2) {
            continue;
        }

        const auto& reference = residuals.front();
        const size_t freq_slot = static_cast<size_t>(group.second);
        for (size_t index = 1; index < residuals.size(); ++index) {
            const auto& residual = residuals[index];
            const double dd_residual = reference.residual_m - residual.residual_m;
            const double dd_variance = reference.variance_m2 + residual.variance_m2;
            const double sigma = std::sqrt(std::max(dd_variance, 1e-12));

            phase_sum_sq += dd_residual * dd_residual;
            phase_chi_sq += dd_residual * dd_residual /
                            std::max(dd_variance, 1e-12);
            ++stats.phase_rows;
            stats.max_phase_sigma =
                std::max(stats.max_phase_sigma, std::abs(dd_residual) / sigma);
            if (sigma > 0.0 &&
                std::abs(dd_residual) / sigma >=
                    std::abs(worst_dd_residual_m) /
                        std::max(worst_dd_sigma, 1e-12)) {
                worst_reference_satellite = reference.real_satellite;
                worst_satellite = residual.real_satellite;
                worst_freq_group = static_cast<int>(freq_slot);
                worst_dd_residual_m = dd_residual;
                worst_dd_sigma = sigma;
            }

            if (freq_slot < 2) {
                auto& pair_info = phase_pairs[residual.real_satellite];
                pair_info.residual_m[freq_slot] = dd_residual;
                pair_info.variance_m2[freq_slot] = dd_variance;
                pair_info.frequency_hz[freq_slot] = residual.frequency_hz;
                pair_info.wavelength_m[freq_slot] = residual.wavelength_m;
            }
        }
    }

    for (const auto& [satellite, pair_info] : phase_pairs) {
        const bool has_l1 =
            std::isfinite(pair_info.residual_m[0]) &&
            std::isfinite(pair_info.variance_m2[0]) &&
            pair_info.frequency_hz[0] > 0.0 &&
            pair_info.wavelength_m[0] > 0.0;
        const bool has_l2 =
            std::isfinite(pair_info.residual_m[1]) &&
            std::isfinite(pair_info.variance_m2[1]) &&
            pair_info.frequency_hz[1] > 0.0 &&
            pair_info.wavelength_m[1] > 0.0;
        if (!has_l1 || !has_l2) {
            continue;
        }

        const double gamma =
            std::pow(pair_info.wavelength_m[1] / pair_info.wavelength_m[0], 2);
        const double denom = 1.0 - gamma;
        if (std::abs(denom) <= 1e-9) {
            continue;
        }

        const double max_variance =
            std::max(pair_info.variance_m2[0], pair_info.variance_m2[1]);
        const double dispersive =
            (pair_info.frequency_hz[0] / pair_info.frequency_hz[1]) *
            (pair_info.residual_m[0] - pair_info.residual_m[1]) / denom;
        const double nondispersive =
            (gamma * pair_info.residual_m[0] - pair_info.residual_m[1]) /
            (gamma - 1.0);
        worst_pair_satellite = satellite;
        worst_pair_dispersive = dispersive;
        worst_pair_nondispersive = nondispersive;
        worst_pair_sigma = std::sqrt(std::max(max_variance, 1e-12));
        constexpr double kPairValidationScale = 64.0;  // 8-sigma squared
        if (dispersive * dispersive > kPairValidationScale * max_variance ||
            nondispersive * nondispersive > kPairValidationScale * max_variance) {
            pair_validation_ok = false;
            break;
        }
    }

    const int position_dof =
        (config.kinematic_mode || config.use_dynamics_model) ? 9 : 3;
    if (stats.phase_rows > 0) {
        stats.phase_rms = std::sqrt(phase_sum_sq / stats.phase_rows);
    }
    if (stats.code_rows > 0) {
        stats.code_rms = std::sqrt(code_sum_sq / stats.code_rows);
    }
    if (stats.phase_rows > position_dof) {
        const double chi_square_limit =
            claslibChiSquare001ForDof(stats.phase_rows - position_dof);
        if (chi_square_limit > 0.0) {
            stats.phase_chisq = phase_chi_sq / chi_square_limit;
        }
    }

    constexpr double kMaxPhaseSigma = 4.0;
    constexpr double kMaxPhaseChisq = 5.0;
    stats.accepted =
        stats.phase_rows > position_dof &&
        stats.max_phase_sigma < kMaxPhaseSigma &&
        pair_validation_ok &&
        stats.phase_chisq < kMaxPhaseChisq;
    if (debug_enabled && !stats.accepted) {
        std::cerr << "[CLAS-FIX-DBG] worst_dd ref="
                  << worst_reference_satellite.toString()
                  << " sat=" << worst_satellite.toString()
                  << " freq=" << worst_freq_group
                  << " resid=" << worst_dd_residual_m
                  << " sigma=" << worst_dd_sigma
                  << " pair_sat=" << worst_pair_satellite.toString()
                  << " dispersive=" << worst_pair_dispersive
                  << " nondisp=" << worst_pair_nondispersive
                  << " pair_sigma=" << worst_pair_sigma
                  << "\n";
    }
    return stats;
}

AmbiguityResolutionResult resolveAndValidateAmbiguities(
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const ResolveAmbiguitiesFunction& resolve_ambiguities,
    const ValidateFixedSolutionFunction& validate_fixed_solution,
    bool preserve_float_state_on_accept,
    bool debug_enabled) {
    AmbiguityResolutionResult result;
    if (!resolve_ambiguities) {
        return result;
    }

    const ppp_shared::PPPState pre_fix_state = filter_state;
    const auto pre_fix_ambiguities = ambiguity_states;

    result.attempted = true;
    if (!resolve_ambiguities()) {
        return result;
    }

    if (validate_fixed_solution) {
        result.validation_stats = validate_fixed_solution();
        result.accepted = result.validation_stats.accepted;
    } else {
        result.accepted = true;
    }

    if (!result.accepted) {
        filter_state = pre_fix_state;
        ambiguity_states = pre_fix_ambiguities;
        result.rejected_after_fix = true;
        if (debug_enabled) {
            std::cerr << "[CLAS-FIX] reject: phase_rows="
                      << result.validation_stats.phase_rows
                      << " phase_rms=" << result.validation_stats.phase_rms
                      << " phase_chisq=" << result.validation_stats.phase_chisq
                      << " max_phase_sigma=" << result.validation_stats.max_phase_sigma
                      << "\n";
        }
        return result;
    }

    if (preserve_float_state_on_accept) {
        result.fixed_filter_state = filter_state;
        result.has_fixed_filter_state = true;
        filter_state = pre_fix_state;
        ambiguity_states = pre_fix_ambiguities;
    }

    if (debug_enabled) {
        std::cerr << "[CLAS-FIX] accept: phase_rows="
                  << result.validation_stats.phase_rows
                  << " phase_rms=" << result.validation_stats.phase_rms
                  << " phase_chisq=" << result.validation_stats.phase_chisq
                  << " max_phase_sigma=" << result.validation_stats.max_phase_sigma
                  << "\n";
    }

    return result;
}

void logUpdateSummary(
    const KalmanUpdateStats& update_stats,
    size_t satellite_count,
    const ppp_shared::PPPState& filter_state) {
    double code_rms = 0.0;
    double phase_rms = 0.0;
    int n_code = 0;
    int n_phase = 0;
    constexpr double kCodePhaseVarianceBoundary = 0.5;
    for (int i = 0; i < update_stats.nobs; ++i) {
        if (update_stats.variances(i) > kCodePhaseVarianceBoundary) {
            code_rms += update_stats.residuals(i) * update_stats.residuals(i);
            ++n_code;
        } else {
            phase_rms += update_stats.residuals(i) * update_stats.residuals(i);
            ++n_phase;
        }
    }
    if (n_code > 0) {
        code_rms = std::sqrt(code_rms / n_code);
    }
    if (n_phase > 0) {
        phase_rms = std::sqrt(phase_rms / n_phase);
    }
    const int clock_index = filter_state.clock_index;
    const int trop_index = filter_state.trop_index;
    const double clock_delta =
        (clock_index >= 0 && clock_index < update_stats.dx.size())
            ? update_stats.dx(clock_index)
            : 0.0;
    const double trop_delta =
        (trop_index >= 0 && trop_index < update_stats.dx.size())
            ? update_stats.dx(trop_index)
            : 0.0;
    const double clock_state =
        (clock_index >= 0 && clock_index < filter_state.state.size())
            ? filter_state.state(clock_index)
            : 0.0;
    const double trop_state =
        (trop_index >= 0 && trop_index < filter_state.state.size())
            ? filter_state.state(trop_index)
            : 0.0;
    std::cerr << "[CLAS-PPP] rows=" << update_stats.nobs
              << " sats=" << satellite_count
              << " pos_delta=" << update_stats.dx.head(3).norm()
              << " clock_delta=" << clock_delta
              << " trop_delta=" << trop_delta
              << " clock_state=" << clock_state
              << " trop_state=" << trop_state
              << " code_rms=" << code_rms
              << " phase_rms=" << phase_rms
              << "\n";
}

PositionSolution finalizeEpochSolution(
    const ppp_shared::PPPState& filter_state,
    bool fixed,
    double ar_ratio,
    int fixed_ambiguities,
    int num_satellites) {
    PositionSolution solution;
    solution.position_ecef = filter_state.state.segment(0, 3);
    solution.receiver_clock_bias = filter_state.state(filter_state.clock_index);
    solution.status = fixed ? SolutionStatus::PPP_FIXED : SolutionStatus::PPP_FLOAT;
    solution.ratio = fixed ? ar_ratio : 0.0;
    solution.num_fixed_ambiguities = fixed ? fixed_ambiguities : 0;
    solution.num_satellites = num_satellites;
    return solution;
}

}  // namespace libgnss::ppp_clas
