#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/algorithms/rtk_measurement.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/solution.hpp>

#include <functional>
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

using TropMappingFunction =
    std::function<double(const Vector3d&, double, const GNSSTime&)>;

struct DdRow {
    SatelliteId reference_satellite;
    SatelliteId target_satellite;
    bool is_phase = false;
    int frequency_index = 0;
    double residual_m = 0.0;
    Vector3d position_coefficients = Vector3d::Zero();
    std::vector<rtk_measurement::StateCoefficient> state_coefficients;
    double reference_variance_m2 = 0.0;
    double target_variance_m2 = 0.0;
};

struct DdMeasurementBuildResult {
    std::vector<DdRow> rows;
    rtk_measurement::MeasurementSystem measurement_system;
    int phase_rows = 0;
    int code_rows = 0;
    int reference_groups = 0;
};

DdMeasurementBuildResult buildDdMeasurementSystem(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const StateLayout& layout,
    const VectorXd& state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function);

struct DdUpdateDiagnostics {
    bool updated = false;
    std::string fallback_reason;
    int measurement_rows = 0;
    int phase_rows = 0;
    int code_rows = 0;
    int reference_groups = 0;
    rtk_update::FilterUpdateResult filter_update;
};

class DdFilterScaffold {
public:
    PositionSolution processFloatUpdate(
        const ObservationData& obs,
        const CLASEpochContext& epoch_context,
        const ppp_shared::PPPState& native_state,
        const PositionSolution& native_float_solution,
        const ppp_shared::PPPConfig& config,
        const TropMappingFunction& trop_mapping_function);

    PositionSolution processFloatPassthrough(
        const GNSSTime& time,
        const ppp_shared::PPPState& native_state,
        const PositionSolution& native_float_solution,
        const ppp_shared::PPPConfig& config,
        const std::vector<OSRCorrection>& osr_corrections,
        const std::map<std::string, std::string>& epoch_atmos);

    const StateSnapshot& snapshot() const { return snapshot_; }
    bool hasSnapshot() const { return has_snapshot_; }
    const DdUpdateDiagnostics& lastDiagnostics() const { return last_diagnostics_; }
    int totalUpdatedEpochs() const { return total_updated_epochs_; }
    int totalFallbackEpochs() const { return total_fallback_epochs_; }
    void reset();

private:
    void ensureSnapshotStorage(const StateLayout& layout, bool preserve_existing);
    void initializeFromNativeFloat(
        const GNSSTime& time,
        const StateLayout& layout,
        const ppp_shared::PPPState& native_state,
        const PositionSolution& native_float_solution,
        const ppp_shared::PPPConfig& config,
        const std::vector<OSRCorrection>& osr_corrections,
        const std::map<std::string, std::string>& epoch_atmos);
    void predictState(
        const ObservationData& obs,
        const std::vector<OSRCorrection>& osr_corrections,
        const ppp_shared::PPPConfig& config,
        double dt);
    void updateObservedStateBookkeeping(
        const ObservationData& obs,
        const std::vector<OSRCorrection>& osr_corrections,
        const ppp_shared::PPPConfig& config,
        double dt);
    void resetStateElement(int index, double value, double variance);
    PositionSolution fallbackSolution(
        const PositionSolution& native_float_solution,
        const std::string& reason,
        int observed_satellites);
    PositionSolution publishSolution(
        const PositionSolution& native_float_solution,
        const rtk_measurement::MeasurementDiagnostics& measurement_diagnostics,
        int observed_satellites) const;

    StateSnapshot snapshot_;
    std::map<int, int> ionosphere_outage_by_satno_;
    std::map<int, double> ionosphere_process_noise_by_satno_;
    std::map<std::pair<int, int>, int> ambiguity_outage_by_satno_freq_;
    DdUpdateDiagnostics last_diagnostics_;
    Vector3d static_anchor_position_ = Vector3d::Zero();
    int total_updated_epochs_ = 0;
    int total_fallback_epochs_ = 0;
    bool has_snapshot_ = false;
    bool has_static_anchor_ = false;
};

}  // namespace libgnss::ppp_clas_dd
