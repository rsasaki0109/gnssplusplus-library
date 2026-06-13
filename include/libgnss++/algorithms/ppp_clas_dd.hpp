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
#include <tuple>
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
    int system_group = -1;
    double residual_m = 0.0;
    double reference_elevation_rad = 0.0;
    double target_elevation_rad = 0.0;
    Vector3d position_coefficients = Vector3d::Zero();
    std::vector<rtk_measurement::StateCoefficient> state_coefficients;
    double reference_variance_m2 = 0.0;
    double target_variance_m2 = 0.0;
};

struct DdReferenceGroup {
    int system_group = -1;
    int frequency_index = 0;
    bool is_phase = false;
    SatelliteId reference_satellite;
    double reference_elevation_rad = 0.0;
};

struct DdMeasurementBuildResult {
    std::vector<DdRow> rows;
    std::vector<DdReferenceGroup> reference_groups_detail;
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

struct DdPostfitValidationResult {
    bool accepted = false;
    std::string reject_reason;
    int row_count = 0;
    int phase_row_count = 0;
    double residual_rms_m = 0.0;
    double residual_max_abs_m = 0.0;
    double phase_residual_rms_m = 0.0;
    double phase_residual_max_abs_m = 0.0;
    double chi_square_ratio = 0.0;
    int worst_phase_system_group = -1;
    int worst_phase_frequency_index = -1;
    double worst_phase_residual_m = 0.0;
    std::string worst_phase_pair;
};

DdPostfitValidationResult validateDdPostfitResiduals(
    const DdMeasurementBuildResult& postfit_build,
    const StateLayout& layout,
    const MatrixXd& covariance);

struct DdUpdateDiagnostics {
    bool updated = false;
    std::string fallback_reason;
    int measurement_rows = 0;
    int phase_rows = 0;
    int code_rows = 0;
    int reference_groups = 0;
    bool lambda_attempted = false;
    bool lambda_accepted = false;
    int lambda_ambiguities = 0;
    double lambda_ratio = 0.0;
    double lambda_required_ratio = 0.0;
    std::string lambda_reject_reason;
    bool fixed_postfit_accepted = false;
    std::string fixed_reject_reason;
    DdPostfitValidationResult fixed_postfit;
    double ar_pdop = 0.0;
    double hold_pdop = 0.0;
    int reference_change_groups = 0;
    int consecutive_fix_count = 0;
    bool hold_applied = false;
    int hold_rows = 0;
    std::string hold_reject_reason;
    std::string row_summary;
    std::string reference_summary;
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
    const StateSnapshot& fixedSnapshot() const { return fixed_snapshot_; }
    bool hasSnapshot() const { return has_snapshot_; }
    bool hasFixedSnapshot() const { return has_fixed_snapshot_; }
    const DdUpdateDiagnostics& lastDiagnostics() const { return last_diagnostics_; }
    int totalUpdatedEpochs() const { return total_updated_epochs_; }
    int totalFallbackEpochs() const { return total_fallback_epochs_; }
    int totalLambdaAcceptedEpochs() const { return total_lambda_accepted_epochs_; }
    int totalLambdaRejectedEpochs() const { return total_lambda_rejected_epochs_; }
    int totalFixedRejectedByRatio() const { return fixed_reject_ratio_epochs_; }
    int totalFixedRejectedByPostfit() const { return fixed_reject_postfit_epochs_; }
    int totalFixedRejectedByPdop() const { return fixed_reject_pdop_epochs_; }
    int totalFixedRejectedByMinFix() const { return fixed_reject_minfix_epochs_; }
    int totalFixedRejectedByReferenceChange() const { return fixed_reject_refchange_epochs_; }
    int totalHoldAppliedEpochs() const { return total_hold_applied_epochs_; }
    double meanLambdaRatio() const {
        return lambda_ratio_count_ > 0
            ? lambda_ratio_sum_ / static_cast<double>(lambda_ratio_count_)
            : 0.0;
    }
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
        int observed_satellites,
        const StateSnapshot& source_snapshot,
        bool fixed) const;
    bool conditionFixedSnapshot(
        const ObservationData& obs,
        const std::vector<OSRCorrection>& osr_corrections,
        const std::vector<DdRow>& rows,
        const ppp_shared::PPPConfig& config,
        const TropMappingFunction& trop_mapping_function,
        int reference_change_groups,
        double ar_pdop);
    int countReferenceChanges(const DdMeasurementBuildResult& build) const;
    void rememberReferenceGroups(const DdMeasurementBuildResult& build);
    void updateAmbiguityLockCounts(const std::vector<DdRow>& rows);
    double computePdop(const std::vector<DdRow>& rows, double min_elevation_rad) const;
    bool applyHoldAmbiguity(const std::vector<DdRow>& rows);
    void appendDiagnosticsCsv(const GNSSTime& time, bool fixed) const;

    StateSnapshot snapshot_;
    StateSnapshot fixed_snapshot_;
    std::map<int, int> ionosphere_outage_by_satno_;
    std::map<int, double> ionosphere_process_noise_by_satno_;
    std::map<std::pair<int, int>, int> ambiguity_outage_by_satno_freq_;
    std::map<std::pair<int, int>, int> ambiguity_lock_by_satno_freq_;
    std::map<std::tuple<int, int, bool>, SatelliteId> reference_by_group_;
    std::vector<rtk_measurement::AmbiguityDifference> validated_hold_differences_;
    VectorXd validated_hold_ambiguities_;
    DdUpdateDiagnostics last_diagnostics_;
    int total_updated_epochs_ = 0;
    int total_fallback_epochs_ = 0;
    int total_lambda_accepted_epochs_ = 0;
    int total_lambda_rejected_epochs_ = 0;
    int fixed_reject_ratio_epochs_ = 0;
    int fixed_reject_postfit_epochs_ = 0;
    int fixed_reject_pdop_epochs_ = 0;
    int fixed_reject_minfix_epochs_ = 0;
    int fixed_reject_refchange_epochs_ = 0;
    int total_hold_applied_epochs_ = 0;
    int consecutive_validated_fixes_ = 0;
    double lambda_ratio_sum_ = 0.0;
    int lambda_ratio_count_ = 0;
    bool has_snapshot_ = false;
    bool has_fixed_snapshot_ = false;
};

}  // namespace libgnss::ppp_clas_dd
