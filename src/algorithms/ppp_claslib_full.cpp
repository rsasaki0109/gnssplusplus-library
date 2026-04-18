#include <libgnss++/algorithms/ppp_claslib_full.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_claslib_pntpos.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <vector>

namespace libgnss::ppp_claslib_full {

namespace {

constexpr double kIonoInitM = 1e-6;
constexpr double kIonoInitVariance = 1e-4;
constexpr double kIonoProcessMPerSqrtS = 1e-3;
constexpr double kIonoProcessMaxM = 0.050;
constexpr double kBiasInitStdCycles = 100.0;
constexpr double kBiasProcessCyclesPerSqrtS = 1e-3;
constexpr int kMinLockForAr = 1;
constexpr int kMaxAmbOutage = 90;
constexpr double kElevationMaskRad = 15.0 * M_PI / 180.0;
constexpr int kResidualFreqCount = 2;

struct AppliedCorrections {
    double code_m = 0.0;
    double phase_m = 0.0;
};

struct ZdRow {
    const OSRCorrection* osr = nullptr;
    SatelliteId satellite;
    SatelliteId ambiguity_satellite;
    int freq_index = 0;
    int group = 0;
    bool is_phase = false;
    bool valid = false;
    double y = 0.0;
    double variance = 0.0;
    Eigen::RowVectorXd H_base;
};

struct FullMeasurementRow {
    Eigen::RowVectorXd H;
    double residual = 0.0;
    double variance = 0.0;
    double reference_variance = 0.0;
    SatelliteId satellite;
    SatelliteId ambiguity_satellite;
    SatelliteId reference_satellite;
    bool is_phase = false;
    int freq_index = 0;
};

struct MeasurementBuild {
    std::vector<FullMeasurementRow> rows;
    std::set<SatelliteId> observed_ambiguities;
};

struct UpdateStats {
    bool updated = false;
    int nobs = 0;
    int active_states = 0;
    VectorXd residuals;
    VectorXd variances;
    MatrixXd pre_update_covariance;
};

double elevationWeight(double elevation_rad) {
    const double sin_el = std::max(std::sin(elevation_rad), 1e-3);
    return 1.0 / (sin_el * sin_el);
}

double clasCodeVarianceForRow(GNSSSystem system, double elevation_rad) {
    constexpr double kCodePhaseRatio = 50.0;
    constexpr double kPhaseSigmaM = 0.01;
    constexpr double kPhaseElevationSigmaM = 0.005;
    const double system_factor = system == GNSSSystem::GLONASS ? 1.5 : 1.0;
    const double a = system_factor * kCodePhaseRatio * kPhaseSigmaM;
    const double b = system_factor * kCodePhaseRatio * kPhaseElevationSigmaM;
    return a * a + b * b * elevationWeight(elevation_rad);
}

double clasPhaseVarianceForRow(GNSSSystem system, double elevation_rad, int freq_index) {
    constexpr double kPhaseSigmaM = 0.01;
    constexpr double kPhaseElevationSigmaM = 0.005;
    const double system_factor = system == GNSSSystem::GLONASS ? 1.5 : 1.0;
    const double a = system_factor * kPhaseSigmaM;
    const double b = system_factor * kPhaseElevationSigmaM;
    double variance = a * a + b * b * elevationWeight(elevation_rad);
    if (freq_index == 1) {
        variance *= std::pow(2.55 / 1.55, 2);
    }
    return variance;
}

double claslibIonoMappingFunction(const Vector3d& receiver_position, double elevation_rad) {
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

int clasSystemOrder(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return 0;
        case GNSSSystem::GLONASS: return 1;
        case GNSSSystem::Galileo: return 2;
        case GNSSSystem::BeiDou: return 3;
        case GNSSSystem::QZSS: return 4;
        default: return 6;
    }
}

int clasResidualGroup(const OSRCorrection& osr, int freq_index) {
    if (osr.satellite.system == GNSSSystem::GPS && freq_index == 1 &&
        osr.signals[freq_index] == SignalType::GPS_L2C) {
        return 5;
    }
    return clasSystemOrder(osr.satellite.system);
}

int clasSatNo(const SatelliteId& satellite) {
    const int prn = static_cast<int>(satellite.prn);
    switch (satellite.system) {
        case GNSSSystem::GPS:
            return prn >= 1 && prn <= 32 ? prn : 0;
        case GNSSSystem::Galileo:
            return prn >= 1 && prn <= 36 ? 32 + prn : 0;
        case GNSSSystem::QZSS: {
            const int qprn = prn >= 193 ? prn - 192 : prn;
            return qprn >= 1 && qprn <= 7 ? 68 + qprn : 0;
        }
        default:
            return 0;
    }
}

bool sameClasSatellite(const SatelliteId& lhs, const SatelliteId& rhs) {
    return lhs.system == rhs.system && clasSatNo(lhs) == clasSatNo(rhs) &&
           clasSatNo(lhs) > 0;
}

SatelliteId realSatelliteForAmbiguity(const SatelliteId& satellite) {
    if (satellite.system == GNSSSystem::QZSS &&
        satellite.prn >= 193 && satellite.prn <= 199) {
        return satellite;
    }
    int prn = static_cast<int>(satellite.prn);
    while (prn > 100) {
        prn -= 100;
    }
    return SatelliteId(satellite.system, static_cast<uint8_t>(std::max(1, prn)));
}

SatelliteId ambiguitySatelliteForFreq(const SatelliteId& satellite, int freq_index) {
    int base_prn = static_cast<int>(satellite.prn);
    if (satellite.system == GNSSSystem::QZSS && base_prn >= 193 && base_prn <= 199) {
        base_prn -= 192;
    }
    if (freq_index <= 0) {
        return SatelliteId(satellite.system, static_cast<uint8_t>(base_prn));
    }
    const int prn = base_prn + 100 * freq_index;
    return SatelliteId(
        satellite.system,
        static_cast<uint8_t>(std::min(255, std::max(0, prn))));
}

int ionoIndex(const SatelliteId& satellite) {
    const int satno = clasSatNo(satellite);
    return satno > 0 ? kClasNp + satno - 1 : -1;
}

int ambIndex(const SatelliteId& satellite, int freq_index) {
    const int satno = clasSatNo(satellite);
    if (satno <= 0 || freq_index < 0 || freq_index >= kClasNfreq) {
        return -1;
    }
    return kClasAmbStart + kClasMaxSat * freq_index + satno - 1;
}

int freqIndexFromAmbIndex(int index) {
    if (index < kClasAmbStart || index >= kClasNx) {
        return -1;
    }
    return (index - kClasAmbStart) / kClasMaxSat;
}

bool rowLessClasOrder(const ZdRow& lhs, const ZdRow& rhs) {
    if (lhs.group != rhs.group) return lhs.group < rhs.group;
    if (lhs.freq_index != rhs.freq_index) return lhs.freq_index < rhs.freq_index;
    if (lhs.is_phase != rhs.is_phase) return lhs.is_phase && !rhs.is_phase;
    if (lhs.satellite.system != rhs.satellite.system) {
        return lhs.satellite.system < rhs.satellite.system;
    }
    return lhs.satellite.prn < rhs.satellite.prn;
}

ppp_shared::PPPConfig fullConfig(const ppp_shared::PPPConfig& base) {
    ppp_shared::PPPConfig config = base;
    config.use_clas_osr_filter = true;
    config.wlnl_strict_claslib_parity = true;
    config.use_ionosphere_free = false;
    config.estimate_ionosphere = true;
    config.estimate_troposphere = false;
    config.ar_method = ppp_shared::PPPConfig::ARMethod::DD_PER_FREQ;
    config.clas_epoch_policy = ppp_shared::PPPConfig::ClasEpochPolicy::STRICT_OSR;
    config.clas_correction_application_policy =
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
    config.clas_atmos_selection_policy =
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_FIRST;
    config.clas_decouple_clock_position = false;
    config.initial_ionosphere_variance = kIonoInitVariance;
    config.process_noise_ionosphere = 1e-6;
    return config;
}

AppliedCorrections selectAppliedCorrections(
    const OSRCorrection& osr,
    int freq_index,
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    AppliedCorrections corrections;
    if (freq_index < 0 || freq_index >= osr.num_frequencies) {
        return corrections;
    }
    switch (policy) {
        case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR:
            corrections.code_m = osr.PRC[freq_index];
            corrections.phase_m = osr.CPC[freq_index];
            break;
        case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS:
            corrections.code_m =
                osr.relativity_correction_m +
                osr.receiver_antenna_m[freq_index] +
                osr.code_bias_m[freq_index] +
                osr.solid_earth_tide_m;
            corrections.phase_m =
                osr.relativity_correction_m +
                osr.receiver_antenna_m[freq_index] +
                osr.phase_bias_m[freq_index] +
                osr.windup_m[freq_index] +
                osr.phase_compensation_m[freq_index] +
                osr.solid_earth_tide_m;
            break;
        case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_ONLY:
            corrections.code_m =
                osr.relativity_correction_m +
                osr.receiver_antenna_m[freq_index] +
                osr.solid_earth_tide_m;
            corrections.phase_m = corrections.code_m;
            break;
    }
    return corrections;
}

void ensureStorage(ClaslibRtkState& rtk) {
    if (rtk.x.size() == kClasNx && rtk.P.rows() == kClasNx && rtk.P.cols() == kClasNx) {
        return;
    }
    rtk.x = VectorXd::Zero(kClasNx);
    rtk.P = MatrixXd::Zero(kClasNx, kClasNx);
    rtk.ionosphere_indices.clear();
    rtk.ambiguity_indices.clear();
    rtk.ambiguity_states.clear();
}

void initx(ClaslibRtkState& rtk, int index, double value, double variance) {
    if (index < 0 || index >= kClasNx) {
        return;
    }
    rtk.x(index) = value;
    rtk.P.row(index).setZero();
    rtk.P.col(index).setZero();
    rtk.P(index, index) = variance;
}

void ensureStateMaps(ClaslibRtkState& rtk, const std::vector<OSRCorrection>& osr_corrections) {
    ensureStorage(rtk);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const int ii = ionoIndex(osr.satellite);
        if (ii >= 0) {
            rtk.ionosphere_indices[osr.satellite] = ii;
        }
        for (int f = 0; f < std::min(osr.num_frequencies, kClasNfreq); ++f) {
            const SatelliteId amb_sat = ambiguitySatelliteForFreq(osr.satellite, f);
            const int ai = ambIndex(osr.satellite, f);
            if (ai < 0) {
                continue;
            }
            rtk.ambiguity_indices[amb_sat] = ai;
            auto& ambiguity = rtk.ambiguity_states[amb_sat];
            ambiguity.ambiguity_scale_m = 1.0;
        }
    }
}

bool solveSeed(const ObservationData& obs,
               const NavigationData& nav,
               const ClaslibRtkState& rtk,
               PositionSolution& seed) {
    ppp_claslib_pntpos::PntposOptions options;
    options.mode = ppp_claslib_pntpos::PntposOptions::Mode::PppRtk;
    options.debug_dump = ppp_shared::pppDebugEnabled();
    Vector3d initial_rr = Vector3d::Zero();
    if (rtk.initialized && rtk.x.size() >= 3 && rtk.x.head<3>().norm() > 0.0) {
        initial_rr = rtk.x.head<3>();
    } else if (obs.receiver_position.norm() > 0.0) {
        initial_rr = obs.receiver_position;
    }
    return ppp_claslib_pntpos::solvePntposSeed(
        obs, nav, initial_rr, seed, nullptr, options);
}

void predictPosition(ClaslibRtkState& rtk,
                     const PositionSolution& seed,
                     const ppp_shared::PPPConfig& config) {
    ensureStorage(rtk);
    if (!rtk.initialized || rtk.x.head<3>().norm() == 0.0) {
        rtk.x.head<3>() = seed.position_ecef;
        const double variance = std::max(1.0, config.clas_initial_position_variance);
        rtk.P.block<3, 3>(0, 0).setIdentity();
        rtk.P.block<3, 3>(0, 0) *= variance;
        rtk.initialized = true;
        return;
    }
    if (config.kinematic_mode) {
        rtk.x.head<3>() = seed.position_ecef;
    }
}

void predictIonosphere(ClaslibRtkState& rtk,
                       const std::vector<OSRCorrection>& osr_corrections,
                       double dt) {
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const auto it = rtk.ionosphere_indices.find(osr.satellite);
        if (it == rtk.ionosphere_indices.end()) {
            continue;
        }
        const int idx = it->second;
        if (idx < 0 || idx >= kClasNx) {
            continue;
        }
        if (rtk.x(idx) == 0.0 || rtk.P(idx, idx) <= 0.0) {
            initx(rtk, idx, kIonoInitM, kIonoInitVariance);
        } else if (dt > 0.0) {
            const double sin_el = std::max(std::sin(osr.elevation), 1e-3);
            const double q = std::pow(std::min(kIonoProcessMaxM,
                                               kIonoProcessMPerSqrtS / sin_el), 2) *
                             dt;
            rtk.P(idx, idx) += q;
        }
    }
}

void predictAmbiguities(ClaslibRtkState& rtk,
                        const ObservationData& obs,
                        const std::vector<OSRCorrection>& osr_corrections,
                        double dt) {
    for (auto& [_, ambiguity] : rtk.ambiguity_states) {
        ++ambiguity.outage_count;
    }

    for (auto& [amb_sat, idx] : rtk.ambiguity_indices) {
        auto& ambiguity = rtk.ambiguity_states[amb_sat];
        if (ambiguity.outage_count > kMaxAmbOutage && idx >= 0 && idx < kClasNx) {
            initx(rtk, idx, 0.0, 0.0);
            ambiguity = ppp_shared::PPPAmbiguityInfo{};
            ambiguity.ambiguity_scale_m = 1.0;
            ambiguity.lock_count = -5;
        }
    }

    std::array<double, kClasNfreq> bias_sum{};
    std::array<int, kClasNfreq> bias_count{};
    std::vector<std::pair<SatelliteId, double>> raw_bias;

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        for (int f = 0; f < std::min(osr.num_frequencies, kClasNfreq); ++f) {
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid || !raw->has_carrier_phase ||
                !raw->has_pseudorange || osr.wavelengths[f] <= 0.0 ||
                !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange)) {
                continue;
            }

            const SatelliteId amb_sat = ambiguitySatelliteForFreq(osr.satellite, f);
            const auto idx_it = rtk.ambiguity_indices.find(amb_sat);
            if (idx_it == rtk.ambiguity_indices.end()) {
                continue;
            }
            const int idx = idx_it->second;
            auto& ambiguity = rtk.ambiguity_states[amb_sat];
            if (raw->loss_of_lock && idx >= 0 && idx < kClasNx) {
                initx(rtk, idx, 0.0, 0.0);
                ambiguity.lock_count = -5;
                ambiguity.outage_count = 0;
                ambiguity.needs_reinitialization = true;
            }
            if (idx >= 0 && idx < kClasNx && rtk.P(idx, idx) > 0.0 && dt > 0.0) {
                rtk.P(idx, idx) += kBiasProcessCyclesPerSqrtS *
                                   kBiasProcessCyclesPerSqrtS * dt;
            }

            const double bias_m =
                raw->carrier_phase * osr.wavelengths[f] - raw->pseudorange;
            raw_bias.emplace_back(amb_sat, bias_m);
            if (idx >= 0 && idx < kClasNx &&
                rtk.x(idx) != 0.0 && std::isfinite(rtk.x(idx)) &&
                osr.wavelengths[f] > 0.0) {
                bias_sum[static_cast<size_t>(f)] +=
                    bias_m - rtk.x(idx) * osr.wavelengths[f];
                bias_count[static_cast<size_t>(f)]++;
            }
        }
    }

    for (const auto& [amb_sat, bias_m] : raw_bias) {
        const auto idx_it = rtk.ambiguity_indices.find(amb_sat);
        if (idx_it == rtk.ambiguity_indices.end()) {
            continue;
        }
        const int idx = idx_it->second;
        if (idx < 0 || idx >= kClasNx ||
            (rtk.x(idx) != 0.0 && std::isfinite(rtk.x(idx)) && rtk.P(idx, idx) > 0.0)) {
            continue;
        }
        const int freq_index = std::clamp(
            freqIndexFromAmbIndex(idx), 0, kClasNfreq - 1);
        const SatelliteId real_sat = realSatelliteForAmbiguity(amb_sat);
        const auto osr_it = std::find_if(
            osr_corrections.begin(), osr_corrections.end(),
            [&](const OSRCorrection& osr) { return sameClasSatellite(osr.satellite, real_sat); });
        if (osr_it == osr_corrections.end() ||
            freq_index >= osr_it->num_frequencies ||
            osr_it->wavelengths[freq_index] <= 0.0) {
            continue;
        }
        const int count = bias_count[static_cast<size_t>(freq_index)];
        const double common_bias_m =
            count > 0 ? bias_sum[static_cast<size_t>(freq_index)] / count : 0.0;
        const double value_cycles =
            (bias_m - common_bias_m) / osr_it->wavelengths[freq_index];
        initx(rtk, idx, value_cycles, kBiasInitStdCycles * kBiasInitStdCycles);
        auto& ambiguity = rtk.ambiguity_states[amb_sat];
        ambiguity.lock_count = -5;
        ambiguity.outage_count = 0;
        ambiguity.needs_reinitialization = true;
        ambiguity.ambiguity_scale_m = 1.0;
        ambiguity.float_value = value_cycles;
    }
}

void propagateState(ClaslibRtkState& rtk,
                    const ObservationData& obs,
                    const std::vector<OSRCorrection>& osr_corrections,
                    const PositionSolution& seed,
                    const ppp_shared::PPPConfig& config) {
    const double dt =
        rtk.has_last_time ? std::max(0.0, std::abs(obs.time - rtk.last_time)) : 0.0;
    predictPosition(rtk, seed, config);
    ensureStateMaps(rtk, osr_corrections);
    predictIonosphere(rtk, osr_corrections, dt);
    predictAmbiguities(rtk, obs, osr_corrections, dt);
}

std::vector<ZdRow> buildZeroDifferenceRows(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position) {
    std::vector<ZdRow> rows;
    rows.reserve(osr_corrections.size() * 2 * kClasNfreq);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.elevation < kElevationMaskRad) {
            continue;
        }
        const double rho_sagnac = geodist(osr.satellite_position, receiver_position);
        const double rho_no_sagnac =
            (osr.satellite_position - receiver_position).norm();
        if (rho_sagnac <= 0.0 || rho_no_sagnac <= 0.0) {
            continue;
        }
        const Vector3d los = (osr.satellite_position - receiver_position).normalized();
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;

        for (int f = 0; f < std::min({osr.num_frequencies, kClasNfreq, kResidualFreqCount}); ++f) {
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid ||
                !raw->has_carrier_phase || !raw->has_pseudorange ||
                !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange) ||
                raw->carrier_phase == 0.0 || raw->pseudorange == 0.0 ||
                osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const AppliedCorrections applied = selectAppliedCorrections(
                osr, f, config.clas_correction_application_policy);
            Eigen::RowVectorXd h_base = Eigen::RowVectorXd::Zero(kClasNx);
            h_base.segment(0, 3) = -los.transpose();

            const double phase_obs_m = raw->carrier_phase * osr.wavelengths[f];
            const double phase_model_m = rho_sagnac - sat_clk_m + applied.phase_m;
            ZdRow phase_row;
            phase_row.osr = &osr;
            phase_row.satellite = osr.satellite;
            phase_row.ambiguity_satellite = ambiguitySatelliteForFreq(osr.satellite, f);
            phase_row.freq_index = f;
            phase_row.group = clasResidualGroup(osr, f);
            phase_row.is_phase = true;
            phase_row.valid = true;
            phase_row.y = phase_obs_m - phase_model_m;
            phase_row.variance =
                clasPhaseVarianceForRow(osr.satellite.system, osr.elevation, f);
            phase_row.H_base = h_base;
            rows.push_back(phase_row);

            const double code_model_m = rho_sagnac - sat_clk_m + applied.code_m;
            ZdRow code_row;
            code_row.osr = &osr;
            code_row.satellite = osr.satellite;
            code_row.freq_index = f;
            code_row.group = clasResidualGroup(osr, f);
            code_row.is_phase = false;
            code_row.valid = true;
            code_row.y = raw->pseudorange - code_model_m;
            code_row.variance =
                clasCodeVarianceForRow(osr.satellite.system, osr.elevation);
            code_row.H_base = h_base;
            rows.push_back(code_row);
        }
    }
    std::sort(rows.begin(), rows.end(), rowLessClasOrder);
    return rows;
}

MeasurementBuild formSingleDifferenceRows(
    const ObservationData& obs,
    const std::vector<ZdRow>& zd_rows,
    ClaslibRtkState& rtk,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position) {
    MeasurementBuild build;
    struct GroupKey {
        int group = 0;
        int freq_index = 0;
        bool is_phase = false;
        bool operator<(const GroupKey& rhs) const {
            if (group != rhs.group) return group < rhs.group;
            if (freq_index != rhs.freq_index) return freq_index < rhs.freq_index;
            return is_phase > rhs.is_phase;
        }
    };
    std::map<GroupKey, std::vector<const ZdRow*>> groups;
    for (const auto& row : zd_rows) {
        if (row.valid) {
            groups[{row.group, row.freq_index, row.is_phase}].push_back(&row);
        }
    }

    for (auto& [key, members] : groups) {
        if (members.size() < 2) {
            continue;
        }
        const ZdRow* ref = nullptr;
        for (const ZdRow* row : members) {
            if (ref == nullptr || row->osr->elevation >= ref->osr->elevation) {
                ref = row;
            }
        }
        if (ref == nullptr) {
            continue;
        }
        for (const ZdRow* sat : members) {
            if (sat == ref) {
                continue;
            }
            FullMeasurementRow row;
            row.H = ref->H_base - sat->H_base;
            row.residual = ref->y - sat->y;
            row.variance = ref->variance + sat->variance;
            row.reference_variance = ref->variance;
            row.satellite = sat->satellite;
            row.ambiguity_satellite =
                key.is_phase ? sat->ambiguity_satellite : SatelliteId{};
            row.reference_satellite =
                key.is_phase ? ref->ambiguity_satellite : ref->satellite;
            row.is_phase = key.is_phase;
            row.freq_index = key.freq_index;

            if (config.estimate_ionosphere) {
                const double ref_fi =
                    ref->osr->wavelengths[key.freq_index] / constants::GPS_L1_WAVELENGTH;
                const double sat_fi =
                    sat->osr->wavelengths[key.freq_index] / constants::GPS_L1_WAVELENGTH;
                const double sign = key.is_phase ? -1.0 : 1.0;
                const double ref_coeff =
                    sign * ref_fi * ref_fi *
                    claslibIonoMappingFunction(receiver_position, ref->osr->elevation);
                const double sat_coeff =
                    sign * sat_fi * sat_fi *
                    claslibIonoMappingFunction(receiver_position, sat->osr->elevation);
                const auto ref_iono_it =
                    rtk.ionosphere_indices.find(ref->satellite);
                const auto sat_iono_it =
                    rtk.ionosphere_indices.find(sat->satellite);
                const bool ref_has_iono =
                    ref_iono_it != rtk.ionosphere_indices.end() &&
                    ref_iono_it->second >= 0 &&
                    ref_iono_it->second < kClasNx;
                const bool sat_has_iono =
                    sat_iono_it != rtk.ionosphere_indices.end() &&
                    sat_iono_it->second >= 0 &&
                    sat_iono_it->second < kClasNx;
                const double ref_state =
                    ref_has_iono ? rtk.x(ref_iono_it->second) : 0.0;
                const double sat_state =
                    sat_has_iono ? rtk.x(sat_iono_it->second) : 0.0;
                row.residual -= ref_coeff * ref_state - sat_coeff * sat_state;
                if (ref_has_iono) {
                    row.H(ref_iono_it->second) = ref_coeff;
                }
                if (sat_has_iono) {
                    row.H(sat_iono_it->second) = -sat_coeff;
                }
            }

            if (key.is_phase) {
                const auto ref_amb_it =
                    rtk.ambiguity_indices.find(ref->ambiguity_satellite);
                const auto sat_amb_it =
                    rtk.ambiguity_indices.find(sat->ambiguity_satellite);
                const bool ref_has_amb =
                    ref_amb_it != rtk.ambiguity_indices.end() &&
                    ref_amb_it->second >= 0 &&
                    ref_amb_it->second < kClasNx;
                const bool sat_has_amb =
                    sat_amb_it != rtk.ambiguity_indices.end() &&
                    sat_amb_it->second >= 0 &&
                    sat_amb_it->second < kClasNx;
                if (!sat_has_amb) {
                    continue;
                }
                const double ref_amb_state =
                    ref_has_amb ? rtk.x(ref_amb_it->second) : 0.0;
                const double sat_amb_state = rtk.x(sat_amb_it->second);
                const double ref_lambda = ref->osr->wavelengths[key.freq_index];
                const double sat_lambda = sat->osr->wavelengths[key.freq_index];
                const double amb_state_term =
                    ref_lambda * ref_amb_state - sat_lambda * sat_amb_state;
                row.residual -= amb_state_term;
                if (ref_has_amb) {
                    row.H(ref_amb_it->second) = ref_lambda;
                    build.observed_ambiguities.insert(ref->ambiguity_satellite);
                }
                row.H(sat_amb_it->second) = -sat_lambda;
                build.observed_ambiguities.insert(sat->ambiguity_satellite);

                const Observation* raw =
                    obs.getObservation(sat->satellite, sat->osr->signals[key.freq_index]);
                if (raw && raw->loss_of_lock) {
                    initx(rtk, sat_amb_it->second, 0.0, 0.0);
                    auto& ambiguity = rtk.ambiguity_states[sat->ambiguity_satellite];
                    ambiguity.lock_count = -5;
                    ambiguity.needs_reinitialization = true;
                }
            }
            build.rows.push_back(row);
        }
    }
    return build;
}

MeasurementBuild buildMeasurements(const ObservationData& obs,
                                   const std::vector<OSRCorrection>& osr_corrections,
                                   ClaslibRtkState& rtk,
                                   const ppp_shared::PPPConfig& config,
                                   const Vector3d& receiver_position) {
    const auto zd_rows =
        buildZeroDifferenceRows(obs, osr_corrections, config, receiver_position);
    return formSingleDifferenceRows(obs, zd_rows, rtk, config, receiver_position);
}

UpdateStats applyMeasurementUpdate(ClaslibRtkState& rtk,
                                   const std::vector<FullMeasurementRow>& rows,
                                   bool debug_enabled,
                                   const GNSSTime& time) {
    UpdateStats stats;
    stats.nobs = static_cast<int>(rows.size());
    if (stats.nobs < 4) {
        return stats;
    }
    MatrixXd H = MatrixXd::Zero(stats.nobs, kClasNx);
    VectorXd z = VectorXd::Zero(stats.nobs);
    MatrixXd R = MatrixXd::Zero(stats.nobs, stats.nobs);
    for (int i = 0; i < stats.nobs; ++i) {
        const auto& row = rows[static_cast<size_t>(i)];
        H.row(i) = row.H;
        z(i) = row.residual;
        R(i, i) = std::max(row.variance, 1e-8);
    }

    for (int i = 0; i < stats.nobs; ++i) {
        const auto& lhs = rows[static_cast<size_t>(i)];
        if (lhs.reference_variance <= 0.0) {
            continue;
        }
        for (int j = i + 1; j < stats.nobs; ++j) {
            const auto& rhs = rows[static_cast<size_t>(j)];
            if (rhs.reference_variance <= 0.0 ||
                lhs.is_phase != rhs.is_phase ||
                lhs.freq_index != rhs.freq_index ||
                !(lhs.reference_satellite == rhs.reference_satellite)) {
                continue;
            }
            const double common_reference_variance =
                0.5 * (lhs.reference_variance + rhs.reference_variance);
            R(i, j) = common_reference_variance;
            R(j, i) = common_reference_variance;
        }
    }

    std::vector<int> active_indices;
    active_indices.reserve(kClasNx);
    for (int i = 0; i < kClasNx; ++i) {
        if (std::isfinite(rtk.x(i)) && rtk.x(i) != 0.0 &&
            std::isfinite(rtk.P(i, i)) && rtk.P(i, i) > 0.0) {
            active_indices.push_back(i);
        }
    }
    stats.active_states = static_cast<int>(active_indices.size());
    if (active_indices.empty()) {
        return stats;
    }

    stats.pre_update_covariance = rtk.P;
    const int na = static_cast<int>(active_indices.size());
    MatrixXd Pa = MatrixXd::Zero(na, na);
    MatrixXd Ha = MatrixXd::Zero(stats.nobs, na);
    for (int i = 0; i < na; ++i) {
        const int src_i = active_indices[static_cast<size_t>(i)];
        Ha.col(i) = H.col(src_i);
        for (int j = 0; j < na; ++j) {
            Pa(i, j) = rtk.P(src_i, active_indices[static_cast<size_t>(j)]);
        }
    }

    const MatrixXd PHt = Pa * Ha.transpose();
    const MatrixXd S = Ha * PHt + R;
    Eigen::LDLT<MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
        return stats;
    }
    const MatrixXd K = PHt * ldlt.solve(MatrixXd::Identity(stats.nobs, stats.nobs));
    const VectorXd dxa = K * z;
    if (!dxa.allFinite()) {
        return stats;
    }
    const MatrixXd I_KH = MatrixXd::Identity(na, na) - K * Ha;
    MatrixXd Pa_after = I_KH * Pa;
    Pa_after = 0.5 * (Pa_after + Pa_after.transpose());

    for (int i = 0; i < na; ++i) {
        const int dst_i = active_indices[static_cast<size_t>(i)];
        rtk.x(dst_i) += dxa(i);
        for (int j = 0; j < na; ++j) {
            rtk.P(dst_i, active_indices[static_cast<size_t>(j)]) = Pa_after(i, j);
        }
    }
    stats.updated = true;
    stats.residuals = z;
    stats.variances = R.diagonal();

    if (debug_enabled) {
        std::cerr << "[CLAS-FULL] tow=" << time.tow
                  << " stage=filter updated=1"
                  << " nobs=" << stats.nobs
                  << " active=" << stats.active_states
                  << " dx_pos=" << dxa.head(std::min<Eigen::Index>(3, dxa.size())).norm()
                  << " rms=" << std::sqrt(z.squaredNorm() / stats.nobs)
                  << "\n";
    }
    return stats;
}

void markObservedAmbiguities(ClaslibRtkState& rtk,
                             const MeasurementBuild& build,
                             const ObservationData& obs) {
    for (const auto& amb_sat : build.observed_ambiguities) {
        const auto idx_it = rtk.ambiguity_indices.find(amb_sat);
        if (idx_it == rtk.ambiguity_indices.end() ||
            idx_it->second < 0 || idx_it->second >= kClasNx) {
            continue;
        }
        auto& ambiguity = rtk.ambiguity_states[amb_sat];
        ambiguity.float_value = rtk.x(idx_it->second);
        ambiguity.ambiguity_scale_m = 1.0;
        ambiguity.last_time = obs.time;
        ambiguity.outage_count = 0;
        ambiguity.lock_count += 1;
        if (ambiguity.lock_count > 0) {
            ambiguity.needs_reinitialization = false;
        }
    }
}

ppp_shared::PPPState makeArState(const ClaslibRtkState& rtk, bool include_l5) {
    ppp_shared::PPPState state;
    state.state = rtk.x;
    state.covariance = rtk.P;
    state.pos_index = 0;
    state.vel_index = -1;
    state.clock_index = 0;
    state.glo_clock_index = 0;
    state.gal_clock_index = 0;
    state.qzs_clock_index = 0;
    state.bds_clock_index = 0;
    state.trop_index = 0;
    state.iono_index = kClasNp;
    state.amb_index = kClasAmbStart;
    state.ionosphere_indices = rtk.ionosphere_indices;
    state.total_states = kClasNx;
    for (const auto& [satellite, idx] : rtk.ambiguity_indices) {
        if (!include_l5 && freqIndexFromAmbIndex(idx) >= 2) {
            continue;
        }
        state.ambiguity_indices[satellite] = idx;
    }
    return state;
}

void copyFromArState(const ppp_shared::PPPState& state, ClaslibRtkState& rtk) {
    if (state.state.size() == kClasNx &&
        state.covariance.rows() == kClasNx &&
        state.covariance.cols() == kClasNx) {
        rtk.x = state.state;
        rtk.P = state.covariance;
    }
}

bool tryAmbiguityResolution(ClaslibRtkState& rtk,
                            const std::vector<OSRCorrection>& osr_corrections,
                            const ppp_shared::PPPConfig& config,
                            const MatrixXd& pre_update_covariance,
                            const GNSSTime& time,
                            Vector3d& fixed_position) {
    rtk.last_ar_ratio = 0.0;
    rtk.last_fixed_ambiguities = 0;
    if (!config.enable_ambiguity_resolution) {
        return false;
    }

    ppp_shared::PPPState ar_state = makeArState(rtk, true);
    std::map<SatelliteId, double> real_elevations;
    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            real_elevations[osr.satellite] = osr.elevation;
        }
    }

    auto eligible = ppp_ar::collectEligibleAmbiguities(
        ar_state, rtk.ambiguity_states, kMinLockForAr, &time);
    if (eligible.satellites.size() < 4) {
        return false;
    }

    (void)pre_update_covariance;
    MatrixXd ar_cross_covariance;
    const auto attempt = ppp_ar::tryDirectDdFixWithPar(
        config,
        ar_state,
        ar_cross_covariance,
        rtk.ambiguity_states,
        eligible,
        real_elevations,
        ppp_shared::pppDebugEnabled(),
        &time);
    rtk.last_ar_ratio = attempt.ratio;
    rtk.last_fixed_ambiguities = attempt.nb;
    if (!attempt.fixed) {
        return false;
    }
    if (attempt.has_hold_state) {
        copyFromArState(attempt.hold_state, rtk);
        fixed_position = rtk.x.head<3>();
    } else {
        copyFromArState(attempt.state, rtk);
        fixed_position = rtk.x.head<3>();
    }
    rtk.ambiguity_states = attempt.ambiguities;
    return true;
}

PositionSolution makeSolution(const ObservationData& obs,
                              const Vector3d& position,
                              const MatrixXd& covariance,
                              SolutionStatus status,
                              int num_satellites,
                              int num_measurements,
                              double residual_rms,
                              double ratio,
                              int fixed_ambiguities,
                              double receiver_clock_bias_m) {
    PositionSolution solution;
    solution.time = obs.time;
    solution.status = status;
    solution.position_ecef = position;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, lat, lon, height);
    solution.position_geodetic = GeodeticCoord(lat, lon, height);
    if (covariance.rows() >= 3 && covariance.cols() >= 3) {
        solution.position_covariance = covariance.block<3, 3>(0, 0);
    } else {
        solution.position_covariance.setIdentity();
        solution.position_covariance *= 100.0;
    }
    solution.receiver_clock_bias = receiver_clock_bias_m;
    solution.num_satellites = num_satellites;
    solution.num_frequencies = kClasNfreq;
    solution.iterations = 1;
    solution.residual_rms = residual_rms;
    solution.ratio = ratio;
    solution.num_fixed_ambiguities = fixed_ambiguities;
    solution.satellites_used = obs.getSatellites();
    (void)num_measurements;
    return solution;
}

double residualRms(const VectorXd& residuals) {
    return residuals.size() > 0
               ? std::sqrt(residuals.squaredNorm() / static_cast<double>(residuals.size()))
               : 0.0;
}

}  // namespace

void reset(ClaslibRtkState& state) {
    state = ClaslibRtkState{};
}

EpochResult runEpoch(const ObservationData& obs,
                     const NavigationData& nav,
                     const SSRProducts& ssr,
                     ClaslibRtkState& rtk,
                     const ppp_shared::PPPConfig& config) {
    EpochResult result;
    const ppp_shared::PPPConfig epoch_config = fullConfig(config);

    PositionSolution seed;
    const bool seed_ok = solveSeed(obs, nav, rtk, seed);
    if (!seed_ok && (!rtk.initialized || rtk.x.size() < 3)) {
        return result;
    }
    if (!seed_ok) {
        seed = makeSolution(
            obs,
            rtk.x.head<3>(),
            rtk.P,
            SolutionStatus::PPP_FLOAT,
            static_cast<int>(obs.getNumSatellites()),
            0,
            0.0,
            rtk.last_ar_ratio,
            rtk.last_fixed_ambiguities,
            0.0);
    }

    const Vector3d context_position =
        rtk.initialized && rtk.x.size() >= 3 && rtk.x.head<3>().norm() > 0.0
            ? rtk.x.head<3>()
            : seed.position_ecef;
    const auto epoch_context = prepareClasEpochContext(
        obs,
        nav,
        ssr,
        context_position,
        seed.receiver_clock_bias,
        0.0,
        epoch_config,
        rtk.windup_cache,
        rtk.dispersion_compensation,
        rtk.sis_continuity,
        rtk.phase_bias_repair);

    if (epoch_context.osr_corrections.size() < 4) {
        result.solution = seed;
        rtk.has_last_time = true;
        rtk.last_time = obs.time;
        return result;
    }

    propagateState(rtk, obs, epoch_context.osr_corrections, seed, epoch_config);
    const auto measurement_build = buildMeasurements(
        obs,
        epoch_context.osr_corrections,
        rtk,
        epoch_config,
        rtk.x.head<3>());
    result.measurement_count =
        static_cast<int>(measurement_build.rows.size());
    const auto update = applyMeasurementUpdate(
        rtk, measurement_build.rows, ppp_shared::pppDebugEnabled(), obs.time);
    result.updated = update.updated;
    result.active_state_count = update.active_states;
    if (!update.updated) {
        result.solution = seed;
        rtk.has_last_time = true;
        rtk.last_time = obs.time;
        return result;
    }
    markObservedAmbiguities(rtk, measurement_build, obs);

    Vector3d output_position = rtk.x.head<3>();
    result.fixed = tryAmbiguityResolution(
        rtk,
        epoch_context.osr_corrections,
        epoch_config,
        update.pre_update_covariance,
        obs.time,
        output_position);

    const auto valid_osr_count = std::count_if(
        epoch_context.osr_corrections.begin(),
        epoch_context.osr_corrections.end(),
        [](const OSRCorrection& osr) { return osr.valid; });
    result.solution = makeSolution(
        obs,
        output_position,
        rtk.P,
        result.fixed ? SolutionStatus::PPP_FIXED : SolutionStatus::PPP_FLOAT,
        static_cast<int>(valid_osr_count),
        result.measurement_count,
        residualRms(update.residuals),
        rtk.last_ar_ratio,
        rtk.last_fixed_ambiguities,
        seed.receiver_clock_bias);

    if (ppp_shared::pppDebugEnabled()) {
        std::cerr << "[CLAS-FULL] tow=" << obs.time.tow
                  << " nx=" << kClasNx
                  << " nobs=" << result.measurement_count
                  << " updated=" << (result.updated ? 1 : 0)
                  << " fixed=" << (result.fixed ? 1 : 0)
                  << " ratio=" << rtk.last_ar_ratio
                  << " nb=" << rtk.last_fixed_ambiguities
                  << " pos=" << output_position.transpose()
                  << "\n";
    }

    ++rtk.epoch_count;
    rtk.has_last_time = true;
    rtk.last_time = obs.time;
    return result;
}

}  // namespace libgnss::ppp_claslib_full
