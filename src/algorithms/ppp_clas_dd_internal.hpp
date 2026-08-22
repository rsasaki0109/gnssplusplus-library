#pragma once

// Shared file-local helpers for the PPP CLAS DD implementation TUs;
// extracted from the former monolithic ppp_clas_dd.cpp anonymous namespace.

#include <libgnss++/algorithms/ppp_clas_dd.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

namespace libgnss::ppp_clas_dd {
namespace internal {


constexpr int kMinPrnGps = 1;
constexpr int kMaxPrnGps = 32;
constexpr int kNsatGps = kMaxPrnGps - kMinPrnGps + 1;

constexpr int kMinPrnGlo = 1;
constexpr int kMaxPrnGlo = 27;
constexpr int kNsatGlo = kMaxPrnGlo - kMinPrnGlo + 1;

constexpr int kMinPrnGal = 1;
constexpr int kMaxPrnGal = 36;
constexpr int kNsatGal = kMaxPrnGal - kMinPrnGal + 1;

constexpr int kMinPrnQzsClaslib = 193;
constexpr int kMaxPrnQzsClaslib = 202;
constexpr int kNsatQzs = kMaxPrnQzsClaslib - kMinPrnQzsClaslib + 1;

constexpr int kMinPrnBds = 1;
constexpr int kMaxPrnBds = 63;
constexpr int kNsatBds = kMaxPrnBds - kMinPrnBds + 1;

constexpr int kMinPrnSbas = 120;
constexpr int kMaxPrnSbas = 158;
constexpr int kNsatNavic = 14;

constexpr double kDdInitialPositionVarianceM2 = 0.003;
constexpr double kClaslibInitialPositionVarianceM2 = 30.0 * 30.0;
constexpr double kClaslibInitialZwdM = 0.001;
constexpr double kClaslibStdBiasCycles = 100.0;
constexpr double kClaslibStdIonoM = 0.01;
constexpr double kClaslibStdTropM = 0.005;
constexpr double kClaslibPrnBiasCycles = 1e-3;
constexpr double kClaslibPrnIonoM = 1e-3;
constexpr double kClaslibPrnIonoMaxM = 0.05;
constexpr double kClaslibIonoTimeConstantS = 10.0;
constexpr double kClaslibPrnTropM = 1e-3;
constexpr double kClaslibForgetIono = 0.3;
constexpr double kClaslibAfGainIono = 3.0;
constexpr double kClaslibForgetPosition = 0.3;
constexpr double kClaslibAfGainPosition = 1.0;
constexpr int kClaslibMaxOutage = 5;
constexpr double kGpsL2VarianceScale = (2.55 / 1.55) * (2.55 / 1.55);
constexpr double kDdPostfitResidualRmsLimitM = 5.0;
constexpr double kDdPostfitResidualMaxLimitM = 20.0;
constexpr int kClaslibMinLockCount = 5;
constexpr int kClaslibMinFixCount = 0;
constexpr double kClaslibArElevationMaskRad = 20.0 * M_PI / 180.0;
constexpr double kClaslibHoldElevationMaskRad = 30.0 * M_PI / 180.0;
constexpr double kClaslibMaxPdopAr = 50.0;
constexpr double kClaslibMaxPdopHold = 50.0;
constexpr double kClaslibVarHoldAmbCycles2 = 0.001;
constexpr double kClaslibResidualSigmaGate = 1.0e9;
constexpr double kClaslibFixedChiSquareGate = 500.0;
constexpr double kClaslibHoldChiSquareGate = kClaslibFixedChiSquareGate;
constexpr double kDdFixedPostfitPhaseRmsLimitM = 0.75;
constexpr double kDdFixedPostfitPhaseMaxLimitM = 1.80;

struct ZeroDiffMeasurement {
    SatelliteId satellite;
    int satno = 0;
    int system_group = 0;
    bool is_phase = false;
    int frequency_index = 0;
    double residual_m = 0.0;
    Vector3d los = Vector3d::Zero();
    double elevation_rad = 0.0;
    double ionosphere_map = 1.0;
    double trop_mapping = 0.0;
    double wavelength_m = 0.0;
    double ionosphere_scale = 1.0;
    double variance_m2 = 0.0;
};

struct DdGroupKey {
    int system_group = 0;
    int frequency_index = 0;
    bool is_phase = false;

    bool operator<(const DdGroupKey& rhs) const {
        return std::tie(system_group, frequency_index, is_phase) <
               std::tie(rhs.system_group, rhs.frequency_index, rhs.is_phase);
    }
};

struct DdAmbiguityCandidate {
    rtk_measurement::AmbiguityDifference difference;
    int frequency_index = 0;
    SatelliteId target_satellite;
    double target_elevation_rad = 0.0;
};

// CLASLIB static_linux.conf selects pos2-aralpha=10%.  resamb_LAMBDA()
// therefore uses qf[4][nb-1], not RTKLIB's conventional fixed ratio of 3.
constexpr std::array<double, 60> kClaslibRatioThresholdAlpha10{{
    39.86, 9.00, 5.39, 4.11, 3.45, 3.05, 2.78, 2.59, 2.44, 2.32,
    2.23, 2.15, 2.08, 2.02, 1.97, 1.93, 1.89, 1.85, 1.82, 1.79,
    1.77, 1.74, 1.72, 1.70, 1.68, 1.67, 1.65, 1.63, 1.62, 1.61,
    1.59, 1.58, 1.57, 1.56, 1.55, 1.54, 1.53, 1.52, 1.51, 1.51,
    1.50, 1.49, 1.48, 1.48, 1.47, 1.46, 1.46, 1.45, 1.45, 1.44,
    1.44, 1.43, 1.43, 1.42, 1.42, 1.41, 1.41, 1.40, 1.40, 1.40,
}};

inline double claslibRatioThreshold(int ambiguity_count) {
    if (ambiguity_count <= 0) {
        return std::numeric_limits<double>::infinity();
    }
    const size_t index = static_cast<size_t>(std::min(ambiguity_count, 60) - 1);
    return kClaslibRatioThresholdAlpha10[index];
}

inline int qzssPrnForClaslib(int prn) {
    if (prn >= kMinPrnQzsClaslib && prn <= kMaxPrnQzsClaslib) {
        return prn;
    }
    if (prn >= 1 && prn <= kNsatQzs) {
        return kMinPrnQzsClaslib + prn - 1;
    }
    return 0;
}

inline bool readAtmosphereNetworkId(
    const std::map<std::string, std::string>& epoch_atmos,
    int& network_id) {
    const auto it = epoch_atmos.find("atmos_network_id");
    if (it == epoch_atmos.end()) {
        return false;
    }
    char* end = nullptr;
    const long parsed = std::strtol(it->second.c_str(), &end, 10);
    if (end == it->second.c_str()) {
        return false;
    }
    network_id = static_cast<int>(parsed);
    return true;
}

inline int systemGroup(const SatelliteId& satellite, SignalType signal) {
    (void)signal;
    switch (satellite.system) {
        case GNSSSystem::GPS:
            return 0;
        case GNSSSystem::SBAS:
            return 0;
        case GNSSSystem::GLONASS:
            return 1;
        case GNSSSystem::Galileo:
            return 2;
        case GNSSSystem::BeiDou:
            return 3;
        case GNSSSystem::QZSS:
            return 4;
        default:
            return -1;
    }
}

inline std::string groupFrequencyModeLabel(
    int system_group,
    int frequency_index,
    bool is_phase) {
    std::ostringstream label;
    label << 'm' << system_group
          << 'f' << frequency_index
          << (is_phase ? 'P' : 'C');
    return label.str();
}

inline std::string summarizeRowsByGroupFrequency(const std::vector<DdRow>& rows) {
    std::map<std::tuple<int, int, bool>, int> counts;
    for (const auto& row : rows) {
        ++counts[{row.system_group, row.frequency_index, row.is_phase}];
    }
    std::ostringstream summary;
    bool first = true;
    for (const auto& [key, count] : counts) {
        if (!first) {
            summary << ';';
        }
        first = false;
        const auto [system_group, frequency_index, is_phase] = key;
        summary << groupFrequencyModeLabel(system_group, frequency_index, is_phase)
                << '=' << count;
    }
    return summary.str();
}

inline std::string summarizeReferenceGroups(
    const std::vector<DdReferenceGroup>& reference_groups) {
    std::ostringstream summary;
    bool first = true;
    for (const auto& group : reference_groups) {
        if (!first) {
            summary << ';';
        }
        first = false;
        summary << groupFrequencyModeLabel(
                       group.system_group,
                       group.frequency_index,
                       group.is_phase)
                << '=' << group.reference_satellite.toString();
    }
    return summary.str();
}

inline int claslibFrequencyIndex(
    const SatelliteId& satellite,
    SignalType signal,
    int osr_frequency_index) {
    if (satellite.system == GNSSSystem::Galileo) {
        switch (signal) {
            case SignalType::GAL_E1:
                return 0;
            case SignalType::GAL_E5B:
                return 1;
            case SignalType::GAL_E5A:
                return 2;
            default:
                break;
        }
    }
    return osr_frequency_index;
}

inline double osrWavelengthForClaslibFrequency(
    const OSRCorrection& osr,
    int claslib_frequency_index) {
    for (int f = 0; f < osr.num_frequencies; ++f) {
        if (claslibFrequencyIndex(osr.satellite, osr.signals[f], f) ==
            claslib_frequency_index) {
            return osr.wavelengths[f];
        }
    }
    return 0.0;
}

inline double systemErrorFactor(GNSSSystem system) {
    if (system == GNSSSystem::GLONASS) {
        return 1.5;
    }
    if (system == GNSSSystem::SBAS) {
        return 3.0;
    }
    return 1.0;
}

inline double ionosphereMapFactor(const Vector3d& receiver_position, double elevation_rad) {
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver_position, lat, lon, height);
    constexpr double kIonosphereHeightM = 350000.0;
    if (height >= kIonosphereHeightM) {
        return 1.0;
    }
    const double sin_z =
        (constants::WGS84_A + height) / (constants::WGS84_A + kIonosphereHeightM) *
        std::cos(elevation_rad);
    const double clamped = std::clamp(sin_z, -0.999999, 0.999999);
    return 1.0 / std::cos(std::asin(clamped));
}

inline double claslibVarerr(
    GNSSSystem system,
    double elevation_rad,
    bool is_phase,
    int frequency_index) {
    const double sin_el = std::max(std::abs(std::sin(elevation_rad)), 1e-3);
    double factor = systemErrorFactor(system);
    if (!is_phase) {
        factor *= 50.0;
    }
    const double a = factor * 0.010;
    const double b = factor * 0.005;
    double variance = a * a + b * b / (sin_el * sin_el);
    if (is_phase && frequency_index == 1) {
        variance *= kGpsL2VarianceScale;
    }
    return std::max(variance, 1e-12);
}

inline void addStateCoefficient(
    std::vector<rtk_measurement::StateCoefficient>& coefficients,
    int state_index,
    double coefficient) {
    if (state_index < 0 || coefficient == 0.0) {
        return;
    }
    for (auto& existing : coefficients) {
        if (existing.state_index == state_index) {
            existing.coefficient += coefficient;
            return;
        }
    }
    coefficients.push_back({state_index, coefficient});
}

inline std::vector<rtk_measurement::MeasurementBlock> rowsToBlocks(
    const std::map<DdGroupKey, std::vector<DdRow>>& rows_by_group) {
    std::vector<rtk_measurement::MeasurementBlock> blocks;
    blocks.reserve(rows_by_group.size());
    for (const auto& [key, rows] : rows_by_group) {
        if (rows.empty()) {
            continue;
        }
        rtk_measurement::MeasurementBlock block;
        block.kind = key.is_phase
            ? rtk_measurement::MeasurementKind::PHASE
            : rtk_measurement::MeasurementKind::CODE;
        block.frequency_index = key.frequency_index;
        block.rows.reserve(rows.size());
        for (const auto& row : rows) {
            rtk_measurement::MeasurementRow measurement_row;
            measurement_row.residual = row.residual_m;
            measurement_row.baseline_coefficients = row.position_coefficients;
            measurement_row.state_coefficients = row.state_coefficients;
            measurement_row.reference_variance = row.reference_variance_m2;
            measurement_row.satellite_variance = row.target_variance_m2;
            block.rows.push_back(std::move(measurement_row));
        }
        blocks.push_back(std::move(block));
    }
    return blocks;
}

inline int countObservedSatellites(const std::vector<OSRCorrection>& osr_corrections) {
    int count = 0;
    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            ++count;
        }
    }
    return count;
}

inline double rowResidualRms(const std::vector<DdRow>& rows) {
    if (rows.empty()) {
        return 0.0;
    }
    double sum_sq = 0.0;
    for (const auto& row : rows) {
        sum_sq += row.residual_m * row.residual_m;
    }
    return std::sqrt(sum_sq / static_cast<double>(rows.size()));
}

inline double rowResidualMaxAbs(const std::vector<DdRow>& rows) {
    double max_abs = 0.0;
    for (const auto& row : rows) {
        max_abs = std::max(max_abs, std::abs(row.residual_m));
    }
    return max_abs;
}

inline bool postfitRowsAccepted(const DdMeasurementBuildResult& postfit_build) {
    if (postfit_build.rows.size() < 4) {
        return false;
    }
    return rowResidualRms(postfit_build.rows) <= kDdPostfitResidualRmsLimitM &&
           rowResidualMaxAbs(postfit_build.rows) <= kDdPostfitResidualMaxLimitM;
}

inline std::vector<DdAmbiguityCandidate> collectDdAmbiguityCandidates(
    const std::vector<DdRow>& rows,
    const StateLayout& layout,
    const VectorXd& state,
    const MatrixXd& covariance,
    const std::map<std::pair<int, int>, int>& lock_by_satno_freq) {
    std::vector<DdAmbiguityCandidate> candidates;
    std::set<std::tuple<int, int, int>> seen;
    for (const auto& row : rows) {
        if (!row.is_phase) {
            continue;
        }
        const int ref_satno = claslibSatelliteNumber(row.reference_satellite);
        const int target_satno = claslibSatelliteNumber(row.target_satellite);
        const int ref_index =
            layout.ambiguityIndex(ref_satno, row.frequency_index);
        const int target_index =
            layout.ambiguityIndex(target_satno, row.frequency_index);
        if (ref_index < 0 || target_index < 0 ||
            ref_index >= state.size() || target_index >= state.size() ||
            ref_index >= covariance.rows() || target_index >= covariance.rows() ||
            ref_index >= covariance.cols() || target_index >= covariance.cols()) {
            continue;
        }
        const auto ref_lock_it =
            lock_by_satno_freq.find({ref_satno, row.frequency_index});
        const auto target_lock_it =
            lock_by_satno_freq.find({target_satno, row.frequency_index});
        if (ref_lock_it == lock_by_satno_freq.end() ||
            target_lock_it == lock_by_satno_freq.end() ||
            ref_lock_it->second <= 0 ||
            target_lock_it->second <= 0 ||
            row.reference_elevation_rad < kClaslibArElevationMaskRad ||
            row.target_elevation_rad < kClaslibArElevationMaskRad) {
            continue;
        }
        if (!std::isfinite(state(ref_index)) || !std::isfinite(state(target_index)) ||
            !std::isfinite(covariance(ref_index, ref_index)) ||
            !std::isfinite(covariance(target_index, target_index)) ||
            covariance(ref_index, ref_index) <= 0.0 ||
            covariance(target_index, target_index) <= 0.0) {
            continue;
        }
        const auto key = std::make_tuple(
            row.frequency_index, ref_index, target_index);
        if (!seen.insert(key).second) {
            continue;
        }
        candidates.push_back(
            {{ref_index, target_index}, row.frequency_index,
             row.target_satellite, row.target_elevation_rad});
    }
    return candidates;
}

inline const char* clasDdDiagnosticsPath() {
    const auto& path = pppEnvOverrides().clas_dd_diag_path;
    return path.empty() ? nullptr : path.c_str();
}

}  // namespace internal
}  // namespace libgnss::ppp_clas_dd
