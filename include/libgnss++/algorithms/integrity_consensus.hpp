#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace libgnss {

class IntegrityConsensusManager {
public:
    enum class State { NORMAL, SUSPECT, QUARANTINE, RECOVERY };

    enum Reason : std::uint32_t {
        NONE = 0,
        PRIMARY_SUSPECT = 1U << 0,
        ESTIMATOR_DISAGREEMENT = 1U << 1,
        INDEPENDENT_UNAVAILABLE = 1U << 2,
        INDEPENDENT_UNCERTAIN = 1U << 3,
        RESET_GENERATION_CHANGED = 1U << 4,
    };

    struct Config {
        int suspect_streak = 2;
        int quarantine_streak = 3;
        int recovery_streak = 5;
        double aperture_min_m = 1.0;
        double aperture_max_m = 10.0;
        double aperture_sigma_multiplier = 3.0;
        double max_primary_covariance_trace_m2 = 25.0;
        double max_independent_covariance_trace_m2 = 25.0;
        double max_independent_age_s = 1.0;
        // A soft primary-only telemetry warning must not create a quarantine
        // that can never recover while the independent estimator is absent.
        // Hard primary suspects remain fail-closed regardless of availability.
        bool soft_suspect_requires_independent = true;
        // Optional high-specificity mode for tighter apertures: estimator
        // disagreement advances its streak only with an independent primary
        // telemetry warning. Existing profiles retain disagreement-only entry.
        bool disagreement_requires_primary_suspect = false;
    };

    struct Estimate {
        bool valid = false;
        Eigen::Vector3d position_ecef = Eigen::Vector3d::Zero();
        double covariance_trace_m2 = std::numeric_limits<double>::quiet_NaN();
    };

    struct Input {
        Estimate primary;
        Estimate independent;
        bool fixed_candidate = false;
        bool primary_suspect = false;
        bool hard_primary_suspect = false;
        // Caller-owned health gate for provisional FIX output while the state
        // machine is still accumulating its recovery streak. This must use
        // runtime-only evidence and does not authorize anchor promotion.
        bool recovery_candidate_healthy = false;
        double independent_age_s = std::numeric_limits<double>::infinity();
        std::uint64_t independent_reset_generation = 0;
    };

    struct Decision {
        State state = State::NORMAL;
        bool allow_fixed = true;
        bool request_primary_reset = false;
        bool promote_joint_anchor = false;
        bool estimators_agree = false;
        double disagreement_m = std::numeric_limits<double>::quiet_NaN();
        double aperture_m = std::numeric_limits<double>::quiet_NaN();
        int recovery_streak = 0;
        std::uint32_t reasons = NONE;
    };

    IntegrityConsensusManager();
    explicit IntegrityConsensusManager(Config config);

    Decision update(const Input& input);
    void reset();
    State state() const { return state_; }

private:
    Config config_;
    State state_ = State::NORMAL;
    int suspect_count_ = 0;
    int disagreement_count_ = 0;
    int recovery_count_ = 0;
    std::uint64_t independent_reset_generation_ = 0;
    bool has_independent_reset_generation_ = false;
};

// Stateless recovery authority for several independently restarted position
// shadows.  The caller remains responsible for producing genuinely
// independent shadows; source_id prevents accidentally counting one source
// twice in a single decision.
class MultiShadowPositionConsensus {
public:
    enum Reason : std::uint32_t {
        POSITION_NONE = 0,
        PRIMARY_NOT_FIXED = 1U << 0,
        INSUFFICIENT_SHADOWS = 1U << 1,
        DUPLICATE_SHADOW_SOURCE = 1U << 2,
        NO_UNIQUE_CLUSTER = 1U << 3,
        AMBIGUOUS_CLUSTERS = 1U << 4,
        PRIMARY_TOO_CLOSE = 1U << 5,
        PREDICTION_UNAVAILABLE = 1U << 6,
        PREDICTION_DISAGREEMENT = 1U << 7,
    };

    struct Config {
        int min_independent_shadows = 2;
        double shadow_agreement_aperture_m = 0.25;
        double primary_separation_min_m = 0.5;
        // Zero disables the freshness gate.  Otherwise at least one member of
        // the authority cluster must have age_epochs strictly below this value.
        std::uint64_t fresh_shadow_max_age_epochs = 1000;
        // Zero disables the causal prediction gate.
        double candidate_max_prediction_error_m = 2.0;
    };

    struct Shadow {
        std::uint64_t source_id = 0;
        bool valid = false;
        bool fixed = false;
        bool healthy = false;
        Eigen::Vector3d position_ecef = Eigen::Vector3d::Zero();
        std::uint64_t age_epochs = std::numeric_limits<std::uint64_t>::max();
    };

    struct Input {
        bool primary_valid = false;
        bool primary_fixed = false;
        Eigen::Vector3d primary_position_ecef = Eigen::Vector3d::Zero();
        bool prediction_valid = false;
        Eigen::Vector3d predicted_position_ecef = Eigen::Vector3d::Zero();
        std::vector<Shadow> shadows;
    };

    struct Decision {
        bool replace_primary_position = false;
        Eigen::Vector3d consensus_position_ecef = Eigen::Vector3d::Zero();
        std::vector<std::uint64_t> source_ids;
        double cluster_diameter_m = std::numeric_limits<double>::quiet_NaN();
        double primary_separation_m = std::numeric_limits<double>::quiet_NaN();
        double prediction_error_m = std::numeric_limits<double>::quiet_NaN();
        std::uint32_t reasons = POSITION_NONE;
    };

    MultiShadowPositionConsensus();
    explicit MultiShadowPositionConsensus(Config config);

    Decision evaluate(const Input& input) const;

private:
    Config config_;
};

}  // namespace libgnss
