#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/solution.hpp>

#include <map>
#include <string>
#include <vector>

namespace libgnss::ppp_clas_dd {

inline constexpr int kClaslibMaxSatellites = 221;
inline constexpr int kClaslibSystemCount = 6;
inline constexpr int kClaslibGlonassFrequencyBiasStates = 2;

enum class IonosphereMode {
    Off,
    Estimate,
    EstimateAdaptive,
};

enum class TroposphereMode {
    Off,
    EstimateZtd,
    EstimateZtdGradients,
};

struct StateLayoutOptions {
    bool dynamics = false;
    int frequencies = 2;
    IonosphereMode ionosphere_mode = IonosphereMode::EstimateAdaptive;
    TroposphereMode troposphere_mode = TroposphereMode::EstimateZtd;
    bool glonass_frequency_bias_states = false;
    int max_satellites = kClaslibMaxSatellites;
    int system_count = kClaslibSystemCount;
};

struct StateLayout {
    StateLayoutOptions options;

    int np() const;
    int nf() const;
    int ni() const;
    int nt() const;
    int nl() const;
    int nb() const;
    int nr() const;
    int nx() const;

    int receiverClockIndex(int system_index) const;
    int macroTroposphereIndexAfterClocks() const;
    int ionosphereIndex(int satno_one_based) const;
    int troposphereIndex() const;
    int ambiguityIndex(int satno_one_based, int frequency_index) const;
    bool validSatelliteNumber(int satno_one_based) const;
};

int claslibSatelliteNumber(const SatelliteId& satellite);

StateLayoutOptions layoutOptionsFromConfig(
    const ppp_shared::PPPConfig& config,
    const std::vector<OSRCorrection>& osr_corrections);

struct StateSnapshot {
    StateLayout layout;
    GNSSTime time;
    VectorXd state;
    MatrixXd covariance;
    std::vector<SatelliteId> observed_satellites;
    bool seeded_from_native_float = false;
    int native_total_states = 0;
    int atmosphere_network_id = -1;
    bool has_atmosphere_network_id = false;
};

class DdFilterScaffold {
public:
    PositionSolution processFloatPassthrough(
        const GNSSTime& time,
        const ppp_shared::PPPState& native_state,
        const PositionSolution& native_float_solution,
        const ppp_shared::PPPConfig& config,
        const std::vector<OSRCorrection>& osr_corrections,
        const std::map<std::string, std::string>& epoch_atmos);

    const StateSnapshot& snapshot() const { return snapshot_; }
    bool hasSnapshot() const { return has_snapshot_; }
    void reset();

private:
    void ensureSnapshotStorage(const StateLayout& layout);

    StateSnapshot snapshot_;
    bool has_snapshot_ = false;
};

}  // namespace libgnss::ppp_clas_dd
