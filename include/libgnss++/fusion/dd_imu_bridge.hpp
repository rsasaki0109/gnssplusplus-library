#pragma once

#include <vector>

#include <Eigen/Dense>

#include <libgnss++/fusion/fusion_types.hpp>

namespace libgnss {
namespace dd_imu_bridge {

struct AmbiguityKey {
    int satellite_prn = 0;
    int frequency_index = -1;
    int generation = 0;
    // PRN is not globally unique and a DD ambiguity also changes when its
    // reference satellite changes.  Keep these fields after the legacy three
    // so existing aggregate initializers remain source-compatible.
    int satellite_system = 0;
    int reference_satellite_prn = 0;
    int reference_satellite_system = 0;
    int reference_generation = 0;
    int signal_type = 0;

    bool operator==(const AmbiguityKey& other) const {
        return satellite_prn == other.satellite_prn &&
               frequency_index == other.frequency_index && generation == other.generation &&
               satellite_system == other.satellite_system &&
               reference_satellite_prn == other.reference_satellite_prn &&
               reference_satellite_system == other.reference_satellite_system &&
               reference_generation == other.reference_generation &&
               signal_type == other.signal_type;
    }
};

struct AmbiguityErrorState {
    AmbiguityKey key;
    double float_value_cycles = 0.0;
    double variance_cycles2 = 100.0;
    bool held = false;
};

struct TightlyCoupledState {
    FusionState eskf;
    std::vector<AmbiguityErrorState> ambiguities;
    /// [15-state INS error, ambiguity errors], including all cross-covariances.
    Eigen::MatrixXd augmented_covariance;
};

/** One already differenced RTK observation, evaluated at eskf.nominal.
 * geometry_enu is d(predicted DD range)/d(position error).  Carrier residual
 * is observed-minus-predicted range before subtracting wavelength*N.
 */
struct DDObservation {
    AmbiguityKey key;
    Eigen::RowVector3d geometry_enu = Eigen::RowVector3d::Zero();
    double code_residual_m = 0.0;
    double code_variance_m2 = 0.0;       ///< <=0 disables the code row
    double carrier_residual_m = 0.0;
    double carrier_variance_m2 = 0.0;    ///< <=0 disables the carrier row
    double wavelength_m = 0.0;
    double elevation_rad = 0.0;
    double body_azimuth_rad = 0.0;
    double posterior_abs_residual_m = 0.0;
    int lock_count = 0;
    bool cycle_slip = false;
};

struct BridgeConfig {
    bool commit_carrier_updates = true;
    double max_nis_per_observation = 16.0;
    double lambda_ratio_threshold = 3.0;
    int partial_ar_min_ambiguities = 3;
    int partial_ar_min_lock_count = 300;
    double fixed_ambiguity_sigma_cycles = 0.01;
    double soft_reset_max_innovation_m = 30.0;
    double soft_reset_position_sigma_m = 5.0;
    double rejected_reset_covariance_scale = 4.0;
    int soft_reset_rejection_patience = 30;
    double soft_reset_max_position_variance_m2 = 100.0;
};

struct UpdateResult {
    bool ok = false;
    bool rejected_by_innovation_gate = false;
    bool carrier_update_accepted = false;
    bool carrier_fallback_used = false;
    int observation_count = 0;
    double nis_per_observation = 0.0;
    double joint_nis_per_observation = 0.0;
};

struct PartialARResult {
    bool fixed = false;
    int attempted = 0;
    int fixed_count = 0;
    double ratio = 0.0;
};

enum class SoftResetAction { REJECTED, MEASUREMENT_UPDATE, COVARIANCE_INFLATION };

class DDIMUBridge {
public:
    explicit DDIMUBridge(const FusionState& initial, BridgeConfig config = {});

    const TightlyCoupledState& state() const { return state_; }

    /// Install an externally mechanized INS state and propagate ambiguity cross-covariance.
    void acceptPropagatedINS(const FusionState& propagated,
                             const Eigen::Matrix<double, 15, 15>& transition);

    /// Build live ambiguity slots, apply joint DD code/carrier update, and inject errors.
    UpdateResult update(const std::vector<DDObservation>& observations);

    /// Quality-ordered sequential partial AR; never holds a subset that fails LAMBDA ratio.
    PartialARResult resolvePartialAmbiguities(const std::vector<DDObservation>& observations);

    /// Preserve a valid propagated state during reacquisition; never hard-overwrite position.
    SoftResetAction softResetPosition(const Eigen::Vector3d& spp_position_enu,
                                      bool propagated_state_valid);

private:
    int findAmbiguity(const AmbiguityKey& key) const;
    int findSignalAmbiguity(const AmbiguityKey& key) const;
    void removeAmbiguity(int index);
    int ensureAmbiguity(const DDObservation& observation);
    UpdateResult applyLinearUpdate(const Eigen::MatrixXd& h,
                                   const Eigen::VectorXd& residual,
                                   const Eigen::MatrixXd& covariance,
                                   double max_nis_per_observation);
    void inject(const Eigen::VectorXd& correction);

    TightlyCoupledState state_;
    BridgeConfig config_;
    int consecutive_innovation_rejections_ = 0;
};

}  // namespace dd_imu_bridge
}  // namespace libgnss
