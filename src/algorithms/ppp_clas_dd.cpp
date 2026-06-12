#include <libgnss++/algorithms/ppp_clas_dd.hpp>

#include <algorithm>
#include <cstdlib>

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

void DdFilterScaffold::ensureSnapshotStorage(const StateLayout& layout) {
    const int nx = layout.nx();
    if (!has_snapshot_ || snapshot_.state.size() != nx) {
        snapshot_.state = VectorXd::Zero(nx);
        snapshot_.covariance = MatrixXd::Zero(nx, nx);
    } else {
        snapshot_.state.setZero();
        snapshot_.covariance.setZero();
    }
    snapshot_.layout = layout;
    has_snapshot_ = true;
}

PositionSolution DdFilterScaffold::processFloatPassthrough(
    const GNSSTime& time,
    const ppp_shared::PPPState& native_state,
    const PositionSolution& native_float_solution,
    const ppp_shared::PPPConfig& config,
    const std::vector<OSRCorrection>& osr_corrections,
    const std::map<std::string, std::string>& epoch_atmos) {
    const StateLayout layout{layoutOptionsFromConfig(config, osr_corrections)};
    ensureSnapshotStorage(layout);

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
    has_snapshot_ = false;
}

}  // namespace libgnss::ppp_clas_dd
