#include <libgnss++/algorithms/ppp_clas_dd.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>
#include <tuple>
#include <utility>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

namespace libgnss::ppp_clas_dd {
namespace {

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

constexpr double kDdStaticInitialPositionVariance = 1.0;
constexpr double kClaslibInitialZwdM = 0.001;
constexpr double kClaslibStdBiasCycles = 100.0;
constexpr double kClaslibStdIonoM = 0.01;
constexpr double kClaslibStdTropM = 0.005;
constexpr double kClaslibPrnBiasCycles = 1e-3;
constexpr double kClaslibPrnIonoM = 1e-3;
constexpr double kClaslibPrnIonoMaxM = 0.05;
constexpr double kClaslibPrnTropM = 1e-3;
constexpr double kClaslibForgetIono = 0.5;
constexpr double kClaslibAfGainIono = 1.0;
constexpr int kClaslibMaxOutage = 5;
constexpr double kGpsL2VarianceScale = (2.55 / 1.55) * (2.55 / 1.55);
constexpr double kDdStaticAnchorVarianceM2 = 0.003;
constexpr double kDdStaticPositionCovarianceFloorM2 = 1.0;
constexpr double kDdPostfitResidualRmsLimitM = 5.0;
constexpr double kDdPostfitResidualMaxLimitM = 20.0;

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

int qzssPrnForClaslib(int prn) {
    if (prn >= kMinPrnQzsClaslib && prn <= kMaxPrnQzsClaslib) {
        return prn;
    }
    if (prn >= 1 && prn <= kNsatQzs) {
        return kMinPrnQzsClaslib + prn - 1;
    }
    return 0;
}

bool readAtmosphereNetworkId(
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

int systemGroup(const SatelliteId& satellite) {
    switch (satellite.system) {
        case GNSSSystem::GPS:
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

double systemErrorFactor(GNSSSystem system) {
    if (system == GNSSSystem::GLONASS) {
        return 1.5;
    }
    if (system == GNSSSystem::SBAS) {
        return 3.0;
    }
    return 1.0;
}

double ionosphereMapFactor(const Vector3d& receiver_position, double elevation_rad) {
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

double claslibVarerr(
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

void addStateCoefficient(
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

std::vector<rtk_measurement::MeasurementBlock> rowsToBlocks(
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

int countObservedSatellites(const std::vector<OSRCorrection>& osr_corrections) {
    int count = 0;
    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            ++count;
        }
    }
    return count;
}

double rowResidualRms(const std::vector<DdRow>& rows) {
    if (rows.empty()) {
        return 0.0;
    }
    double sum_sq = 0.0;
    for (const auto& row : rows) {
        sum_sq += row.residual_m * row.residual_m;
    }
    return std::sqrt(sum_sq / static_cast<double>(rows.size()));
}

double rowResidualMaxAbs(const std::vector<DdRow>& rows) {
    double max_abs = 0.0;
    for (const auto& row : rows) {
        max_abs = std::max(max_abs, std::abs(row.residual_m));
    }
    return max_abs;
}

bool postfitRowsAccepted(const DdMeasurementBuildResult& postfit_build) {
    if (postfit_build.rows.size() < 4) {
        return false;
    }
    return rowResidualRms(postfit_build.rows) <= kDdPostfitResidualRmsLimitM &&
           rowResidualMaxAbs(postfit_build.rows) <= kDdPostfitResidualMaxLimitM;
}

void appendStaticPositionAnchor(rtk_measurement::MeasurementSystem& system,
                                const Vector3d& state_position,
                                const Vector3d& anchor_position) {
    const int old_rows = static_cast<int>(system.residuals.size());
    const int n_states = static_cast<int>(system.design_matrix.cols());
    if (n_states < 3 || system.design_matrix.rows() != old_rows ||
        system.covariance.rows() != old_rows ||
        system.covariance.cols() != old_rows) {
        return;
    }

    MatrixXd design = MatrixXd::Zero(old_rows + 3, n_states);
    VectorXd residuals = VectorXd::Zero(old_rows + 3);
    MatrixXd covariance = MatrixXd::Zero(old_rows + 3, old_rows + 3);
    if (old_rows > 0) {
        design.topRows(old_rows) = system.design_matrix;
        residuals.head(old_rows) = system.residuals;
        covariance.topLeftCorner(old_rows, old_rows) = system.covariance;
    }
    for (int axis = 0; axis < 3; ++axis) {
        design(old_rows + axis, axis) = 1.0;
        residuals(old_rows + axis) = anchor_position(axis) - state_position(axis);
        covariance(old_rows + axis, old_rows + axis) = kDdStaticAnchorVarianceM2;
    }
    system.design_matrix = std::move(design);
    system.residuals = std::move(residuals);
    system.covariance = std::move(covariance);
}

}  // namespace

int StateLayout::np() const {
    return options.dynamics ? 9 : 3;
}

int StateLayout::nf() const {
    return std::max(1, options.frequencies);
}

int StateLayout::ni() const {
    return options.ionosphere_mode == IonosphereMode::Off ? 0 : options.max_satellites;
}

int StateLayout::nt() const {
    switch (options.troposphere_mode) {
        case TroposphereMode::Off:
            return 0;
        case TroposphereMode::EstimateZtd:
            return 1;
        case TroposphereMode::EstimateZtdGradients:
            return 3;
    }
    return 0;
}

int StateLayout::nl() const {
    return options.glonass_frequency_bias_states
        ? kClaslibGlonassFrequencyBiasStates
        : 0;
}

int StateLayout::nb() const {
    return options.max_satellites * nf();
}

int StateLayout::nr() const {
    return np() + ni() + nt() + nl();
}

int StateLayout::nx() const {
    return nr() + nb();
}

int StateLayout::receiverClockIndex(int system_index) const {
    return np() + system_index;
}

int StateLayout::macroTroposphereIndexAfterClocks() const {
    return receiverClockIndex(0) + options.system_count;
}

int StateLayout::ionosphereIndex(int satno_one_based) const {
    return validSatelliteNumber(satno_one_based) && ni() > 0
        ? np() + satno_one_based - 1
        : -1;
}

int StateLayout::troposphereIndex() const {
    return nt() > 0 ? np() + ni() : -1;
}

int StateLayout::ambiguityIndex(int satno_one_based, int frequency_index) const {
    if (!validSatelliteNumber(satno_one_based) ||
        frequency_index < 0 ||
        frequency_index >= nf()) {
        return -1;
    }
    return nr() + options.max_satellites * frequency_index + satno_one_based - 1;
}

bool StateLayout::validSatelliteNumber(int satno_one_based) const {
    return satno_one_based >= 1 && satno_one_based <= options.max_satellites;
}

int claslibSatelliteNumber(const SatelliteId& satellite) {
    const int prn = static_cast<int>(satellite.prn);
    if (prn <= 0) {
        return 0;
    }

    switch (satellite.system) {
        case GNSSSystem::GPS:
            return prn <= kMaxPrnGps ? prn : 0;
        case GNSSSystem::GLONASS:
            return prn <= kMaxPrnGlo ? kNsatGps + prn : 0;
        case GNSSSystem::Galileo:
            return prn <= kMaxPrnGal ? kNsatGps + kNsatGlo + prn : 0;
        case GNSSSystem::QZSS: {
            const int claslib_prn = qzssPrnForClaslib(prn);
            return claslib_prn == 0
                ? 0
                : kNsatGps + kNsatGlo + kNsatGal +
                      (claslib_prn - kMinPrnQzsClaslib + 1);
        }
        case GNSSSystem::BeiDou:
            return prn <= kMaxPrnBds
                ? kNsatGps + kNsatGlo + kNsatGal + kNsatQzs + prn
                : 0;
        case GNSSSystem::SBAS:
            return prn >= kMinPrnSbas && prn <= kMaxPrnSbas
                ? kNsatGps + kNsatGlo + kNsatGal + kNsatQzs + kNsatBds +
                      kNsatNavic + (prn - kMinPrnSbas + 1)
                : 0;
        default:
            return 0;
    }
}

StateLayoutOptions layoutOptionsFromConfig(
    const ppp_shared::PPPConfig& config,
    const std::vector<OSRCorrection>& osr_corrections) {
    StateLayoutOptions options;
    options.dynamics = config.kinematic_mode && config.use_dynamics_model;
    options.ionosphere_mode = config.estimate_ionosphere
        ? IonosphereMode::EstimateAdaptive
        : IonosphereMode::Off;
    options.troposphere_mode = config.estimate_troposphere
        ? TroposphereMode::EstimateZtd
        : TroposphereMode::Off;

    int frequencies = config.use_ionosphere_free ? 1 : 2;
    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            frequencies = std::max(frequencies, osr.num_frequencies);
        }
    }
    options.frequencies = std::max(1, frequencies);
    return options;
}

DdMeasurementBuildResult buildDdMeasurementSystem(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const StateLayout& layout,
    const VectorXd& state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function) {
    DdMeasurementBuildResult result;
    const int nx = layout.nx();
    if (state.size() < nx || nx < 3 || trop_mapping_function == nullptr) {
        return result;
    }

    const Vector3d receiver_position = state.segment(0, 3);
    if (!receiver_position.allFinite() || receiver_position.norm() <= 0.0) {
        return result;
    }

    double rx_lat = 0.0;
    double rx_lon = 0.0;
    double rx_height = 0.0;
    ecef2geodetic(receiver_position, rx_lat, rx_lon, rx_height);

    std::map<DdGroupKey, std::vector<ZeroDiffMeasurement>> groups;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.num_frequencies <= 0) {
            continue;
        }
        const int satno = claslibSatelliteNumber(osr.satellite);
        const int group = systemGroup(osr.satellite);
        if (!layout.validSatelliteNumber(satno) || group < 0) {
            continue;
        }
        const Vector3d range_vector = osr.satellite_position - receiver_position;
        const double range_norm = range_vector.norm();
        const double geo = geodist(osr.satellite_position, receiver_position);
        if (!std::isfinite(geo) || range_norm <= 0.0) {
            continue;
        }
        const Vector3d los = range_vector / range_norm;
        const Vector3d los_enu = ecef2enu(range_vector, rx_lat, rx_lon);
        const double elevation =
            std::atan2(los_enu.z(), std::hypot(los_enu.x(), los_enu.y()));
        const double sat_clock_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double ion_map = ionosphereMapFactor(receiver_position, elevation);
        const double trop_mapping =
            config.estimate_troposphere
                ? trop_mapping_function(receiver_position, elevation, obs.time)
                : 0.0;

        for (int f = 0; f < osr.num_frequencies && f < layout.nf(); ++f) {
            if (osr.wavelengths[f] <= 0.0 || osr.wavelengths[0] <= 0.0) {
                continue;
            }
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (raw == nullptr || !raw->valid) {
                continue;
            }
            const double ion_scale =
                std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2);

            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase)) {
                ZeroDiffMeasurement zd;
                zd.satellite = osr.satellite;
                zd.satno = satno;
                zd.system_group = group;
                zd.is_phase = true;
                zd.frequency_index = f;
                zd.residual_m =
                    raw->carrier_phase * osr.wavelengths[f] -
                    (geo - sat_clock_m + osr.CPC[f]);
                zd.los = los;
                zd.elevation_rad = elevation;
                zd.ionosphere_map = ion_map;
                zd.trop_mapping = trop_mapping;
                zd.wavelength_m = osr.wavelengths[f];
                zd.ionosphere_scale = ion_scale;
                zd.variance_m2 =
                    claslibVarerr(osr.satellite.system, elevation, true, f);
                groups[{group, f, true}].push_back(zd);
            }

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                ZeroDiffMeasurement zd;
                zd.satellite = osr.satellite;
                zd.satno = satno;
                zd.system_group = group;
                zd.is_phase = false;
                zd.frequency_index = f;
                zd.residual_m =
                    raw->pseudorange - (geo - sat_clock_m + osr.PRC[f]);
                zd.los = los;
                zd.elevation_rad = elevation;
                zd.ionosphere_map = ion_map;
                zd.trop_mapping = trop_mapping;
                zd.wavelength_m = osr.wavelengths[f];
                zd.ionosphere_scale = ion_scale;
                zd.variance_m2 =
                    claslibVarerr(osr.satellite.system, elevation, false, f);
                groups[{group, f, false}].push_back(zd);
            }
        }
    }

    std::map<DdGroupKey, std::vector<DdRow>> rows_by_group;
    for (const auto& [key, measurements] : groups) {
        if (measurements.size() < 2) {
            continue;
        }

        const auto ref_it = std::max_element(
            measurements.begin(),
            measurements.end(),
            [](const ZeroDiffMeasurement& lhs, const ZeroDiffMeasurement& rhs) {
                return lhs.elevation_rad < rhs.elevation_rad;
            });
        if (ref_it == measurements.end()) {
            continue;
        }
        ++result.reference_groups;
        const ZeroDiffMeasurement& ref = *ref_it;

        for (const auto& sat : measurements) {
            if (sat.satellite == ref.satellite) {
                continue;
            }

            DdRow row;
            row.reference_satellite = ref.satellite;
            row.target_satellite = sat.satellite;
            row.is_phase = key.is_phase;
            row.frequency_index = key.frequency_index;
            row.residual_m = ref.residual_m - sat.residual_m;
            row.position_coefficients = -ref.los + sat.los;
            row.reference_variance_m2 = ref.variance_m2;
            row.target_variance_m2 = sat.variance_m2;

            if (config.estimate_ionosphere &&
                layout.options.ionosphere_mode != IonosphereMode::Off) {
                const double sign = row.is_phase ? -1.0 : 1.0;
                const double ref_coeff =
                    sign * ref.ionosphere_scale * ref.ionosphere_map;
                const double sat_coeff =
                    sign * sat.ionosphere_scale * sat.ionosphere_map;
                const int ref_iono = layout.ionosphereIndex(ref.satno);
                const int sat_iono = layout.ionosphereIndex(sat.satno);
                row.residual_m -=
                    ref_coeff * state(ref_iono) - sat_coeff * state(sat_iono);
                addStateCoefficient(row.state_coefficients, ref_iono, ref_coeff);
                addStateCoefficient(row.state_coefficients, sat_iono, -sat_coeff);
            }

            const int trop_index = layout.troposphereIndex();
            if (trop_index >= 0 && config.estimate_troposphere) {
                const double trop_coeff = ref.trop_mapping - sat.trop_mapping;
                row.residual_m -= trop_coeff * state(trop_index);
                addStateCoefficient(row.state_coefficients, trop_index, trop_coeff);
            }

            if (row.is_phase) {
                const int ref_ambiguity =
                    layout.ambiguityIndex(ref.satno, key.frequency_index);
                const int sat_ambiguity =
                    layout.ambiguityIndex(sat.satno, key.frequency_index);
                row.residual_m -=
                    ref.wavelength_m * state(ref_ambiguity) -
                    sat.wavelength_m * state(sat_ambiguity);
                addStateCoefficient(
                    row.state_coefficients, ref_ambiguity, ref.wavelength_m);
                addStateCoefficient(
                    row.state_coefficients, sat_ambiguity, -sat.wavelength_m);
                ++result.phase_rows;
            } else {
                ++result.code_rows;
            }

            rows_by_group[key].push_back(row);
            result.rows.push_back(std::move(row));
        }
    }

    const auto blocks = rowsToBlocks(rows_by_group);
    result.measurement_system =
        rtk_measurement::assembleMeasurementSystem(blocks, nx);
    return result;
}

void DdFilterScaffold::ensureSnapshotStorage(
    const StateLayout& layout,
    bool preserve_existing) {
    const int nx = layout.nx();
    if (!has_snapshot_ || snapshot_.state.size() != nx) {
        snapshot_.state = VectorXd::Zero(nx);
        snapshot_.covariance = MatrixXd::Zero(nx, nx);
    } else if (!preserve_existing) {
        snapshot_.state.setZero();
        snapshot_.covariance.setZero();
    }
    snapshot_.layout = layout;
    has_snapshot_ = true;
}

void DdFilterScaffold::resetStateElement(
    int index,
    double value,
    double variance) {
    if (index < 0 || index >= snapshot_.state.size()) {
        return;
    }
    snapshot_.state(index) = value;
    snapshot_.covariance.row(index).setZero();
    snapshot_.covariance.col(index).setZero();
    snapshot_.covariance(index, index) = variance;
}

void DdFilterScaffold::initializeFromNativeFloat(
    const GNSSTime& time,
    const StateLayout& layout,
    const ppp_shared::PPPState& native_state,
    const PositionSolution& native_float_solution,
    const ppp_shared::PPPConfig& config,
    const std::vector<OSRCorrection>& osr_corrections,
    const std::map<std::string, std::string>& epoch_atmos) {
    ensureSnapshotStorage(layout, false);
    snapshot_.time = time;
    snapshot_.seeded_from_native_float = true;
    snapshot_.native_total_states = native_state.total_states;
    snapshot_.has_atmosphere_network_id =
        readAtmosphereNetworkId(epoch_atmos, snapshot_.atmosphere_network_id);

    const Vector3d seed_position =
        native_float_solution.position_ecef.allFinite()
            ? native_float_solution.position_ecef
            : native_state.state.segment(native_state.pos_index, 3);
    if (!has_static_anchor_) {
        static_anchor_position_ = seed_position;
        has_static_anchor_ = true;
    }
    for (int axis = 0; axis < 3 && axis < layout.np(); ++axis) {
        resetStateElement(axis, seed_position(axis), kDdStaticInitialPositionVariance);
    }

    if (layout.options.dynamics && native_state.state.size() >= native_state.pos_index + 9) {
        for (int axis = 3; axis < 9; ++axis) {
            const double variance = axis < 6 ? 1.0 : 1.0;
            resetStateElement(axis, native_state.state(native_state.pos_index + axis), variance);
        }
    }

    const int trop_index = layout.troposphereIndex();
    if (trop_index >= 0 && config.estimate_troposphere) {
        resetStateElement(
            trop_index,
            kClaslibInitialZwdM,
            kClaslibStdTropM * kClaslibStdTropM);
    }

    updateObservedStateBookkeeping(
        ObservationData(time), osr_corrections, config, 0.0);
}

void DdFilterScaffold::predictState(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config,
    double dt) {
    if (!has_snapshot_) {
        return;
    }
    const double abs_dt = std::abs(dt);
    const StateLayout& layout = snapshot_.layout;
    if (layout.options.dynamics && layout.np() >= 9) {
        for (int axis = 0; axis < 3; ++axis) {
            snapshot_.state(axis) +=
                snapshot_.state(axis + 3) * dt +
                0.5 * snapshot_.state(axis + 6) * dt * dt;
            snapshot_.state(axis + 3) += snapshot_.state(axis + 6) * dt;
        }
    }

    const double position_q = std::max(0.0, config.process_noise_position) * abs_dt;
    if (position_q > 0.0) {
        for (int axis = 0; axis < 3; ++axis) {
            snapshot_.covariance(axis, axis) += position_q;
        }
    }

    const int trop_index = layout.troposphereIndex();
    if (trop_index >= 0 && snapshot_.state(trop_index) == 0.0) {
        resetStateElement(
            trop_index,
            kClaslibInitialZwdM,
            kClaslibStdTropM * kClaslibStdTropM);
    } else if (trop_index >= 0) {
        snapshot_.covariance(trop_index, trop_index) +=
            kClaslibPrnTropM * kClaslibPrnTropM * abs_dt;
    }

    updateObservedStateBookkeeping(obs, osr_corrections, config, abs_dt);
}

void DdFilterScaffold::updateObservedStateBookkeeping(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config,
    double dt) {
    if (!has_snapshot_) {
        return;
    }

    const StateLayout& layout = snapshot_.layout;
    std::map<int, std::vector<std::pair<int, double>>> ambiguity_biases_m;

    for (auto& [_, outage] : ionosphere_outage_by_satno_) {
        ++outage;
    }
    for (auto& [_, outage] : ambiguity_outage_by_satno_freq_) {
        ++outage;
    }

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const int satno = claslibSatelliteNumber(osr.satellite);
        if (!layout.validSatelliteNumber(satno)) {
            continue;
        }
        ionosphere_outage_by_satno_[satno] = 0;

        const int iono_index = layout.ionosphereIndex(satno);
        if (config.estimate_ionosphere && iono_index >= 0) {
            if (snapshot_.state(iono_index) == 0.0 ||
                snapshot_.covariance(iono_index, iono_index) <= 0.0) {
                resetStateElement(
                    iono_index,
                    1e-6,
                    kClaslibStdIonoM * kClaslibStdIonoM);
                ionosphere_process_noise_by_satno_[satno] = 0.0;
            } else if (dt > 0.0) {
                const double elevation = osr.elevation;
                double qi = ionosphere_process_noise_by_satno_[satno];
                if (layout.options.ionosphere_mode == IonosphereMode::EstimateAdaptive) {
                    if (qi == 0.0) {
                        const double fact = std::cos(elevation);
                        qi = kClaslibPrnIonoM * kClaslibPrnIonoM * fact * fact;
                    } else {
                        qi = std::clamp(
                            qi,
                            kClaslibPrnIonoM * kClaslibPrnIonoM,
                            kClaslibPrnIonoMaxM * kClaslibPrnIonoMaxM);
                    }
                } else {
                    const double fact = std::cos(elevation);
                    qi = kClaslibPrnIonoM * kClaslibPrnIonoM * fact * fact;
                }
                ionosphere_process_noise_by_satno_[satno] = qi;
                snapshot_.covariance(iono_index, iono_index) += qi * dt;
            }
        }

        for (int f = 0; f < osr.num_frequencies && f < layout.nf(); ++f) {
            if (osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (raw == nullptr || !raw->valid || !raw->has_carrier_phase ||
                !raw->has_pseudorange || !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange)) {
                continue;
            }
            const auto key = std::make_pair(satno, f);
            ambiguity_outage_by_satno_freq_[key] = 0;

            const int ambiguity_index = layout.ambiguityIndex(satno, f);
            if (ambiguity_index < 0) {
                continue;
            }
            if (raw->loss_of_lock) {
                resetStateElement(ambiguity_index, 0.0, 0.0);
            } else if (snapshot_.covariance(ambiguity_index, ambiguity_index) > 0.0 &&
                       dt > 0.0) {
                snapshot_.covariance(ambiguity_index, ambiguity_index) +=
                    kClaslibPrnBiasCycles * kClaslibPrnBiasCycles * dt;
            }

            const double bias_m =
                raw->carrier_phase * osr.wavelengths[f] - raw->pseudorange;
            if (std::isfinite(bias_m)) {
                ambiguity_biases_m[f].push_back({ambiguity_index, bias_m});
            }
        }
    }

    for (const auto& [satno, outage] : ionosphere_outage_by_satno_) {
        if (outage <= kClaslibMaxOutage) {
            continue;
        }
        const int iono_index = layout.ionosphereIndex(satno);
        if (iono_index >= 0) {
            resetStateElement(iono_index, 0.0, 0.0);
        }
    }
    for (const auto& [key, outage] : ambiguity_outage_by_satno_freq_) {
        if (outage <= kClaslibMaxOutage) {
            continue;
        }
        const int ambiguity_index = layout.ambiguityIndex(key.first, key.second);
        if (ambiguity_index >= 0) {
            resetStateElement(ambiguity_index, 0.0, 0.0);
        }
    }

    for (const auto& [frequency, biases] : ambiguity_biases_m) {
        double common_bias_m = 0.0;
        int common_count = 0;
        for (const auto& [ambiguity_index, bias_m] : biases) {
            if (snapshot_.covariance(ambiguity_index, ambiguity_index) > 0.0 &&
                snapshot_.state(ambiguity_index) != 0.0) {
                double wavelength = 0.0;
                for (const auto& osr : osr_corrections) {
                    const int satno = claslibSatelliteNumber(osr.satellite);
                    if (layout.ambiguityIndex(satno, frequency) == ambiguity_index &&
                        frequency < osr.num_frequencies) {
                        wavelength = osr.wavelengths[frequency];
                        break;
                    }
                }
                if (wavelength > 0.0) {
                    common_bias_m += bias_m - snapshot_.state(ambiguity_index) * wavelength;
                    ++common_count;
                }
            }
        }
        if (common_count > 0) {
            common_bias_m /= static_cast<double>(common_count);
        }
        for (const auto& [ambiguity_index, bias_m] : biases) {
            if (snapshot_.state(ambiguity_index) != 0.0 &&
                snapshot_.covariance(ambiguity_index, ambiguity_index) > 0.0) {
                continue;
            }
            double wavelength = 0.0;
            for (const auto& osr : osr_corrections) {
                const int satno = claslibSatelliteNumber(osr.satellite);
                if (layout.ambiguityIndex(satno, frequency) == ambiguity_index &&
                    frequency < osr.num_frequencies) {
                    wavelength = osr.wavelengths[frequency];
                    break;
                }
            }
            if (wavelength <= 0.0) {
                continue;
            }
            double cycles = (bias_m - common_bias_m) / wavelength;
            if (cycles == 0.0) {
                cycles = 1e-6;
            }
            resetStateElement(
                ambiguity_index,
                cycles,
                kClaslibStdBiasCycles * kClaslibStdBiasCycles);
        }
    }
}

PositionSolution DdFilterScaffold::fallbackSolution(
    const PositionSolution& native_float_solution,
    const std::string& reason,
    int observed_satellites) {
    ++total_fallback_epochs_;
    last_diagnostics_.updated = false;
    last_diagnostics_.fallback_reason = reason;
    PositionSolution solution = native_float_solution;
    solution.status = SolutionStatus::PPP_FLOAT;
    solution.ratio = 0.0;
    solution.num_fixed_ambiguities = 0;
    solution.num_satellites = observed_satellites;
    solution.iterations = 0;
    return solution;
}

PositionSolution DdFilterScaffold::publishSolution(
    const PositionSolution& native_float_solution,
    const rtk_measurement::MeasurementDiagnostics& measurement_diagnostics,
    int observed_satellites) const {
    PositionSolution solution = native_float_solution;
    solution.position_ecef = snapshot_.state.segment(0, 3);
    if (snapshot_.covariance.rows() >= 3 && snapshot_.covariance.cols() >= 3) {
        solution.position_covariance = snapshot_.covariance.block<3, 3>(0, 0);
    }
    solution.status = SolutionStatus::PPP_FLOAT;
    solution.ratio = 0.0;
    solution.num_fixed_ambiguities = 0;
    solution.num_satellites = observed_satellites;
    solution.iterations = 1;
    solution.residual_rms = measurement_diagnostics.residual_rms_m;
    solution.rtk_update_observations = measurement_diagnostics.observation_count;
    solution.rtk_update_phase_observations =
        measurement_diagnostics.phase_observation_count;
    solution.rtk_update_code_observations =
        measurement_diagnostics.code_observation_count;
    solution.rtk_update_suppressed_outliers =
        last_diagnostics_.filter_update.suppressed_outliers;
    solution.rtk_update_prefit_residual_rms_m =
        last_diagnostics_.filter_update.prefit_residual_rms_m;
    solution.rtk_update_prefit_residual_max_m =
        last_diagnostics_.filter_update.prefit_residual_max_abs_m;
    solution.rtk_update_post_suppression_residual_rms_m =
        last_diagnostics_.filter_update.post_suppression_residual_rms_m;
    solution.rtk_update_post_suppression_residual_max_m =
        last_diagnostics_.filter_update.post_suppression_residual_max_abs_m;
    solution.rtk_update_normalized_innovation_squared =
        last_diagnostics_.filter_update.normalized_innovation_squared;
    solution.rtk_update_normalized_innovation_squared_per_observation =
        last_diagnostics_.filter_update.normalized_innovation_squared_per_observation;
    solution.rtk_update_rejected_by_innovation_gate =
        last_diagnostics_.filter_update.rejected_by_innovation_gate ? 1 : 0;
    return solution;
}

PositionSolution DdFilterScaffold::processFloatUpdate(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    const ppp_shared::PPPState& native_state,
    const PositionSolution& native_float_solution,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function) {
    last_diagnostics_ = DdUpdateDiagnostics{};
    const auto& osr_corrections = epoch_context.osr_corrections;
    const auto& epoch_atmos = epoch_context.epoch_atmos_tokens;
    const StateLayout layout{layoutOptionsFromConfig(config, osr_corrections)};
    const int observed_satellites = countObservedSatellites(osr_corrections);

    const bool needs_seed =
        !has_snapshot_ || snapshot_.state.size() != layout.nx() ||
        snapshot_.layout.nx() != layout.nx();
    if (needs_seed) {
        initializeFromNativeFloat(
            obs.time,
            layout,
            native_state,
            native_float_solution,
            config,
            osr_corrections,
            epoch_atmos);
    } else {
        snapshot_.layout = layout;
    }

    snapshot_.observed_satellites.clear();
    snapshot_.observed_satellites.reserve(osr_corrections.size());
    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            snapshot_.observed_satellites.push_back(osr.satellite);
        }
    }
    snapshot_.native_total_states = native_state.total_states;

    int network_id = -1;
    const bool has_network_id = readAtmosphereNetworkId(epoch_atmos, network_id);
    if (has_network_id) {
        if (snapshot_.has_atmosphere_network_id &&
            snapshot_.atmosphere_network_id != network_id) {
            for (int satno = 1; satno <= layout.options.max_satellites; ++satno) {
                const int iono_index = layout.ionosphereIndex(satno);
                if (iono_index >= 0) {
                    resetStateElement(iono_index, 0.0, 0.0);
                }
            }
            ionosphere_process_noise_by_satno_.clear();
            ionosphere_outage_by_satno_.clear();
        }
        snapshot_.atmosphere_network_id = network_id;
        snapshot_.has_atmosphere_network_id = true;
    } else {
        snapshot_.has_atmosphere_network_id = false;
    }

    const double dt = needs_seed ? 1.0 : std::max(obs.time - snapshot_.time, 0.001);
    if (std::abs(dt) > 30.0) {
        initializeFromNativeFloat(
            obs.time,
            layout,
            native_state,
            native_float_solution,
            config,
            osr_corrections,
            epoch_atmos);
        updateObservedStateBookkeeping(obs, osr_corrections, config, 0.0);
    } else {
        predictState(obs, osr_corrections, config, dt);
    }
    snapshot_.time = obs.time;

    const auto measurement_build = buildDdMeasurementSystem(
        obs,
        osr_corrections,
        snapshot_.layout,
        snapshot_.state,
        config,
        trop_mapping_function);
    last_diagnostics_.measurement_rows =
        static_cast<int>(measurement_build.rows.size());
    last_diagnostics_.phase_rows = measurement_build.phase_rows;
    last_diagnostics_.code_rows = measurement_build.code_rows;
    last_diagnostics_.reference_groups = measurement_build.reference_groups;

    rtk_measurement::MeasurementDiagnostics measurement_diagnostics;
    measurement_diagnostics.observation_count =
        static_cast<int>(measurement_build.rows.size());
    measurement_diagnostics.phase_observation_count = measurement_build.phase_rows;
    measurement_diagnostics.code_observation_count = measurement_build.code_rows;
    double residual_sum_sq = 0.0;
    for (const auto& row : measurement_build.rows) {
        residual_sum_sq += row.residual_m * row.residual_m;
        measurement_diagnostics.residual_max_abs_m =
            std::max(measurement_diagnostics.residual_max_abs_m, std::abs(row.residual_m));
    }
    if (measurement_diagnostics.observation_count > 0) {
        measurement_diagnostics.residual_rms_m =
            std::sqrt(residual_sum_sq /
                      static_cast<double>(measurement_diagnostics.observation_count));
    }

    if (measurement_build.rows.size() < 4) {
        PositionSolution solution = fallbackSolution(
            native_float_solution, "insufficient_dd_rows", observed_satellites);
        solution.time = obs.time;
        return solution;
    }

    auto apply_update = [&](DdMeasurementBuildResult& build) {
        auto measurement_system = build.measurement_system;
        if (!snapshot_.layout.options.dynamics && has_static_anchor_) {
            for (int axis = 0; axis < 3; ++axis) {
                snapshot_.covariance(axis, axis) = std::max(
                    snapshot_.covariance(axis, axis),
                    kDdStaticPositionCovarianceFloorM2);
            }
            appendStaticPositionAnchor(
                measurement_system,
                snapshot_.state.segment(0, 3),
                static_anchor_position_);
        }
        last_diagnostics_.filter_update = rtk_update::applyMeasurementUpdate(
            snapshot_.state,
            snapshot_.covariance,
            measurement_system,
            std::numeric_limits<double>::infinity(),
            4);
        if (!last_diagnostics_.filter_update.ok) {
            return false;
        }
        const auto postfit_build = buildDdMeasurementSystem(
            obs,
            osr_corrections,
            snapshot_.layout,
            snapshot_.state,
            config,
            trop_mapping_function);
        if (!postfitRowsAccepted(postfit_build)) {
            return false;
        }
        if (!snapshot_.layout.options.dynamics && has_static_anchor_ &&
            (snapshot_.state.segment(0, 3) - static_anchor_position_).norm() > 8.0) {
            return false;
        }
        return true;
    };

    const VectorXd predicted_state = snapshot_.state;
    const MatrixXd predicted_covariance = snapshot_.covariance;
    auto update_build = measurement_build;
    bool update_accepted = apply_update(update_build);
    if (!update_accepted) {
        snapshot_.state = predicted_state;
        snapshot_.covariance = predicted_covariance;

        initializeFromNativeFloat(
            obs.time,
            layout,
            native_state,
            native_float_solution,
            config,
            osr_corrections,
            epoch_atmos);
        updateObservedStateBookkeeping(obs, osr_corrections, config, 0.0);
        auto retry_build = buildDdMeasurementSystem(
            obs,
            osr_corrections,
            snapshot_.layout,
            snapshot_.state,
            config,
            trop_mapping_function);
        last_diagnostics_.measurement_rows =
            static_cast<int>(retry_build.rows.size());
        last_diagnostics_.phase_rows = retry_build.phase_rows;
        last_diagnostics_.code_rows = retry_build.code_rows;
        last_diagnostics_.reference_groups = retry_build.reference_groups;
        if (retry_build.rows.size() < 4 || !apply_update(retry_build)) {
            PositionSolution solution = fallbackSolution(
                native_float_solution,
                last_diagnostics_.filter_update.ok
                    ? "postfit_dd_residual"
                    : "kalman_update",
                observed_satellites);
            solution.time = obs.time;
            return solution;
        }
    }

    if (snapshot_.layout.options.ionosphere_mode == IonosphereMode::EstimateAdaptive) {
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) {
                continue;
            }
            const int satno = claslibSatelliteNumber(osr.satellite);
            const int iono_index = snapshot_.layout.ionosphereIndex(satno);
            if (iono_index < 0 ||
                iono_index >= snapshot_.covariance.rows()) {
                continue;
            }
            const double old_q = ionosphere_process_noise_by_satno_[satno];
            ionosphere_process_noise_by_satno_[satno] =
                kClaslibForgetIono * old_q +
                (1.0 - kClaslibForgetIono) *
                    kClaslibAfGainIono * kClaslibAfGainIono *
                    snapshot_.covariance(iono_index, iono_index);
        }
    }

    ++total_updated_epochs_;
    last_diagnostics_.updated = true;
    PositionSolution solution = publishSolution(
        native_float_solution, measurement_diagnostics, observed_satellites);
    solution.time = obs.time;
    return solution;
}

PositionSolution DdFilterScaffold::processFloatPassthrough(
    const GNSSTime& time,
    const ppp_shared::PPPState& native_state,
    const PositionSolution& native_float_solution,
    const ppp_shared::PPPConfig& config,
    const std::vector<OSRCorrection>& osr_corrections,
    const std::map<std::string, std::string>& epoch_atmos) {
    const StateLayout layout{layoutOptionsFromConfig(config, osr_corrections)};
    ensureSnapshotStorage(layout, false);

    snapshot_.time = time;
    snapshot_.observed_satellites.clear();
    snapshot_.observed_satellites.reserve(osr_corrections.size());
    snapshot_.seeded_from_native_float = true;
    snapshot_.native_total_states = native_state.total_states;
    snapshot_.has_atmosphere_network_id =
        readAtmosphereNetworkId(epoch_atmos, snapshot_.atmosphere_network_id);

    if (native_state.state.size() >= native_state.pos_index + 3) {
        snapshot_.state.segment(0, 3) =
            native_state.state.segment(native_state.pos_index, 3);
    }
    if (native_state.covariance.rows() >= native_state.pos_index + 3 &&
        native_state.covariance.cols() >= native_state.pos_index + 3) {
        snapshot_.covariance.block(0, 0, 3, 3) =
            native_state.covariance.block(native_state.pos_index, native_state.pos_index, 3, 3);
    }

    const int trop_index = layout.troposphereIndex();
    if (trop_index >= 0 &&
        native_state.trop_index >= 0 &&
        native_state.state.size() > native_state.trop_index) {
        snapshot_.state(trop_index) = native_state.state(native_state.trop_index);
        if (native_state.covariance.rows() > native_state.trop_index &&
            native_state.covariance.cols() > native_state.trop_index) {
            snapshot_.covariance(trop_index, trop_index) =
                native_state.covariance(native_state.trop_index, native_state.trop_index);
        }
    }

    for (const auto& [satellite, native_index] : native_state.ionosphere_indices) {
        const int satno = claslibSatelliteNumber(satellite);
        const int dd_index = layout.ionosphereIndex(satno);
        if (dd_index < 0 || native_index < 0 ||
            native_state.state.size() <= native_index) {
            continue;
        }
        snapshot_.state(dd_index) = native_state.state(native_index);
        if (native_state.covariance.rows() > native_index &&
            native_state.covariance.cols() > native_index) {
            snapshot_.covariance(dd_index, dd_index) =
                native_state.covariance(native_index, native_index);
        }
    }

    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            snapshot_.observed_satellites.push_back(osr.satellite);
        }
    }

    PositionSolution solution = native_float_solution;
    solution.time = time;
    solution.status = SolutionStatus::PPP_FLOAT;
    solution.ratio = 0.0;
    solution.num_fixed_ambiguities = 0;
    solution.num_satellites = static_cast<int>(snapshot_.observed_satellites.size());
    return solution;
}

void DdFilterScaffold::reset() {
    snapshot_ = StateSnapshot{};
    ionosphere_outage_by_satno_.clear();
    ionosphere_process_noise_by_satno_.clear();
    ambiguity_outage_by_satno_freq_.clear();
    last_diagnostics_ = DdUpdateDiagnostics{};
    static_anchor_position_ = Vector3d::Zero();
    total_updated_epochs_ = 0;
    total_fallback_epochs_ = 0;
    has_snapshot_ = false;
    has_static_anchor_ = false;
}

}  // namespace libgnss::ppp_clas_dd
