#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "libgnss++/core/solution.hpp"

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

// Causal output gate that combines the KF/FGO state machine with the frozen
// bounded-latency residual policy.  It never consumes reference truth.  A
// streak of N suspicious FIX candidates is held for N-1 epochs so its prefix
// can still be demoted when the Nth epoch confirms the sequence.
class RealtimeFixIntegrityGate {
public:
    struct Config {
        bool enable_consensus = true;
        bool enable_residual_policy = true;
        IntegrityConsensusManager::Config consensus;

        double primary_max_prefit_rms_m = 10.0;
        int primary_min_suppressed_outliers = 35;
        double primary_max_covariance_trace_m2 = 0.05;

        double recovery_min_ratio = 3.0;
        double recovery_max_separation_m = 6.0;
        double recovery_max_prefit_rms_m = 5.0;
        int recovery_max_suppressed_outliers = 20;
        bool allow_provisional_recovery_fixed = false;

        double residual_streak_min_prefit_rms_m = 40.0;
        double residual_streak_max_ratio = 15.0;
        int residual_streak_min_suppressed_outliers = 12;
        double residual_streak_min_outlier_fraction = 0.5;
        int residual_streak_epochs = 8;
        double residual_spike_min_prefit_rms_m = 40.0;
        int residual_spike_max_satellites = 14;

        // Frozen offline "base" low-satellite/ratio confidence gate, ported
        // field-for-field from scripts/experiments/ppc/apply_ppc_status_demotion.py's
        // should_demote()/should_exonerate() with the audited defaults
        // (--min-satellites 8 --low-satellite-ceiling 11
        // --low-satellite-max-ratio 15 --exonerate-min-satellites 11
        // --exonerate-max-prefit-rms-m 0.5 --exonerate-max-nis-per-obs 0.2).
        // Unlike the residual streak/spike rules this is per-epoch: it can
        // demote a FIX candidate immediately, with no streak confirmation,
        // so it still interacts correctly with the 7-epoch retroactive
        // buffer (same immediate-demote path as the spike rule).
        //
        // Off by default (opt-in): this gate was frozen/audited against a
        // separately-engineered staged FGO pipeline
        // (base_selected=0 on all six PPC runs there), where it reproduces
        // the offline audit's "22 caught / 0 harmed" result exactly (see
        // output/urbannav_realtime_fix_integrity_external.md). On the plain
        // KF PPC baseline it over-demotes: offline replay shows
        // base_selected=387/53/68 with correct_harmed=122/11/46 on
        // tokyo1/nagoya1/nagoya2, and the realtime A/B (with the cascade
        // effect) is net-harmful on nagoya2 (80 caught / 177 harmed; see
        // output/ppc_realtime_fix_integrity_matrix.md's "After fix"
        // section). Recommended for low-FIX-rate receivers / configurations
        // close to the audited offline external policy (e.g. UrbanNav
        // Trimble); enable explicitly via gnss_solve's
        // --integrity-base-gate flag once that trade-off has been reviewed
        // for the target receiver/pipeline.
        bool enable_base_confidence_policy = false;
        int base_confidence_min_satellites = 8;
        int base_confidence_low_satellite_ceiling = 11;
        double base_confidence_low_satellite_max_ratio = 15.0;
        int base_confidence_exonerate_min_satellites = 11;
        double base_confidence_exonerate_max_prefit_rms_m = 0.5;
        double base_confidence_exonerate_max_nis_per_obs = 0.2;
    };

    struct IndependentEstimate {
        bool present = false;
        IntegrityConsensusManager::Estimate estimate;
        double age_s = std::numeric_limits<double>::infinity();
        std::uint64_t reset_generation = 0;
    };

    struct EpochInput {
        PositionSolution primary;
        IndependentEstimate independent;
    };

    struct Telemetry {
        IntegrityConsensusManager::Decision consensus;
        bool fixed_candidate = false;
        bool primary_suspect = false;
        bool hard_primary_suspect = false;
        bool residual_streak_match = false;
        bool residual_streak_demoted = false;
        bool residual_spike_demoted = false;
        bool base_confidence_demoted = false;
        bool consensus_demoted = false;
        bool output_demoted = false;
        bool independent_present = false;
        bool independent_valid = false;
        double primary_covariance_trace_m2 =
            std::numeric_limits<double>::quiet_NaN();
        double independent_covariance_trace_m2 =
            std::numeric_limits<double>::quiet_NaN();
        double independent_age_s = std::numeric_limits<double>::infinity();
        std::uint64_t independent_reset_generation = 0;
        int output_latency_epochs = 0;
    };

    struct Emission {
        PositionSolution solution;
        Telemetry telemetry;
    };

    struct Update {
        Telemetry current;
        std::vector<Emission> emitted;
    };

    RealtimeFixIntegrityGate();
    explicit RealtimeFixIntegrityGate(Config config);

    Update push(EpochInput input);
    std::vector<Emission> flush();
    void reset();
    std::size_t pending() const { return pending_.size(); }
    int maxOutputLatencyEpochs() const;

private:
    struct PendingEpoch {
        PositionSolution solution;
        Telemetry telemetry;
        std::uint64_t arrival_index = 0;
    };

    Config config_;
    IntegrityConsensusManager consensus_;
    std::deque<PendingEpoch> pending_;
    int residual_streak_count_ = 0;
    std::uint64_t epoch_index_ = 0;

    bool residualStreakMatches(const PositionSolution& solution) const;
    bool residualSpikeMatches(const PositionSolution& solution) const;
    bool baseConfidenceMatches(const PositionSolution& solution) const;
    // True only when the independent estimate is present, position-valid,
    // fresh, and its own covariance trace is finite and within
    // config_.consensus.max_independent_covariance_trace_m2 -- i.e. the same
    // "independent_healthy" bar IntegrityConsensusManager::update() applies
    // internally for disagreement/agreement evidence. A present-but-unhealthy
    // independent (e.g. an unpopulated/placeholder covariance trace) must
    // behave exactly like an absent one for every consensus evidence path,
    // including hard_primary_suspect -- see docs/ppc_online_consensus_design.md.
    bool independentHealthy(const IndependentEstimate& independent) const;
    static void demote(PendingEpoch& epoch, bool Telemetry::*reason);
    Emission emitFront(std::uint64_t current_index);
};

// Truth-free health gate for an externally produced independent position
// shadow (e.g. an FGO CSV dump) before it may act as
// RealtimeFixIntegrityGate consensus/demotion authority. Health must bound
// *accuracy*, not merely availability: status/GDOP/DD-prefit-RMS/satellite
// count/age alone do not bound position error (see
// docs/ppc_online_consensus_design.md and the PPC tokyo1 shadow validation
// that motivated this class). A missing or non-positive covariance trace is
// treated as unhealthy by default -- absence (or an unpopulated placeholder)
// of accuracy evidence must not silently grant demotion authority.
class ShadowEstimateHealthGate {
public:
    struct Config {
        double max_age_s = 1.0;
        double max_gdop = 4.0;
        double max_ddpr_rms_m = 40.0;
        int min_satellites = 8;
        // Ceiling on the shadow's own reported position_covariance_trace_m2.
        double max_covariance_trace_m2 = 4.0;
        // A missing (or non-positive, see class doc comment) trace sample is
        // unhealthy unless the caller explicitly opts in to substituting
        // default_covariance_trace_m2. Off by default: silently assuming a
        // default covariance is exactly the bug this gate exists to close.
        bool assume_default_covariance_trace = false;
        double default_covariance_trace_m2 = 4.0;
        // Require the shadow sample's own status to be FIXED (not FLOAT) to
        // act as demotion authority. Empirically, on the PPC tokyo1 shipping
        // FGO shadow, FLOAT samples are frequently diverged by tens to
        // thousands of meters while GDOP/DDPR-RMS/nsat/age stay within the
        // otherwise-healthy range, so those signals alone cannot separate
        // trustworthy FLOAT from wrong-basin FLOAT. Defaults to true.
        bool require_fixed_status = true;
    };

    struct Sample {
        // True for a "FIXED"/"4" status token in the shadow source.
        bool status_fixed = false;
        // True for "FIXED"/"FLOAT"/"3"/"4" (i.e. not a dropped/invalid
        // epoch); required regardless of require_fixed_status.
        bool status_present = false;
        std::optional<double> gdop;
        std::optional<double> ddpr_rms_m;
        std::optional<int> num_satellites;
        std::optional<double> covariance_trace_m2;
        double age_s = std::numeric_limits<double>::infinity();
    };

    struct Result {
        bool healthy = false;
        // Trace to feed the consensus manager: the sample's own trace when
        // healthy, otherwise +infinity so it can never itself satisfy a
        // downstream max-covariance-trace aperture check.
        double covariance_trace_m2 = std::numeric_limits<double>::infinity();
    };

    ShadowEstimateHealthGate();
    explicit ShadowEstimateHealthGate(Config config);

    Result evaluate(const Sample& sample) const;

private:
    Config config_;
};

}  // namespace libgnss
