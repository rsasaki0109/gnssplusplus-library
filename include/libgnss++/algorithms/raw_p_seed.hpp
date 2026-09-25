#pragma once

#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

#include <cstddef>
#include <cstdint>
#include <array>
#include <limits>
#include <string>
#include <vector>

namespace libgnss {
namespace raw_p_seed {

/**
 * @brief Result of the position/clock geometry rank check.
 *
 * The rank is for the ECEF position plus native SPP receiver-clock and,
 * when configured, observed inter-system-bias columns.  It is a preflight
 * diagnostic; the native SPP solver remains the authority for the final
 * weighted solve.
 */
enum class RankStatus {
    NotEvaluated = 0,
    InsufficientRows,
    FullRank,
    RankDeficient,
    NumericallyInvalid,
};

struct GeometryRank {
    RankStatus status = RankStatus::NotEvaluated;
    int rank = 0;
    int required_rank = 4;
    std::size_t rows = 0;
};

/**
 * @brief One clock-group entry in an accepted epoch's native SPP result.
 *
 * Every known group is emitted, including groups absent after native
 * observation filtering.  An absent or non-estimated group has a NaN bias;
 * zero is never used as a placeholder.
 */
struct ClockGroupBias {
    GNSSSystem group = GNSSSystem::UNKNOWN;
    bool observed = false;
    bool is_reference = false;
    bool estimate_available = false;
    double bias_m = std::numeric_limits<double>::quiet_NaN();
    std::size_t corrected_rows = 0;
};

/**
 * @brief One terminal row reason copied from native SPP preprocessing.
 */
struct PreprocessRowDiagnostic {
    std::size_t input_row_index = std::numeric_limits<std::size_t>::max();
    GNSSSystem system = GNSSSystem::UNKNOWN;
    bool accepted = false;
    std::string reason;
};

struct PreprocessReasonCount {
    std::string reason;
    std::size_t count = 0;
};

/**
 * @brief Fail-closed status for one raw-P seed epoch.
 */
enum class EpochStatus {
    NotEvaluated = 0,
    Accepted,
    NonfiniteTime,
    DuplicateTime,
    NonmonotonicTime,
    TimeGap,
    InsufficientEpochs,
    UnsupportedClockGroups,
    InsufficientPseudorange,
    InsufficientGeometry,
    NonfiniteGeometry,
    RankDeficient,
    NonfiniteSolution,
    MissingClockBiasEstimate,
    SolverRejected,
    NonfiniteVelocity,
};

/**
 * @brief Explicit endpoint policy for the source-style position gradient.
 *
 * This is MATLAB's `gradient` convention: one-sided differences at the first
 * and last samples, and a centered difference at interior samples.  The
 * policy is part of the result contract rather than an implicit fallback.
 */
enum class VelocityEndpointPolicy {
    OneSidedEndpointsCenteredInterior = 0,
};

/**
 * @brief Configuration for the opt-in raw-P preparatory stage.
 *
 * `processor_config` and `spp_config` are passed to the existing native SPP
 * implementation.  The stage disables Doppler on its private observation
 * copy, so position/clock seeds cannot depend on a D row or on a D-derived
 * velocity.  No imported trajectory or persisted seed is accepted.
 */
struct Config {
    ProcessorConfig processor_config;
    SPPProcessor::SPPConfig spp_config;
    double max_gap_s = 2.0;
    std::size_t min_pseudorange_satellites = 4;
    bool derive_velocity = true;
    // Opt-in same-run cold-start anchor.  When enabled, the first epoch is
    // solved once with only the elevation gate disabled and a zero receiver
    // seed, then the resulting finite native SPP position is supplied to the
    // ordinary pass.  The configured elevation mask remains active for the
    // ordinary pass; no imported position or persisted seed is accepted.
    bool bootstrap_position_before_elevation = false;
    // Opt-in diagnostic mode.  Evaluate every epoch independently and retain
    // each failed epoch's terminal status/diagnostics instead of stopping at
    // the first failure.  This is metadata-only: failed epochs never become
    // seeds, and velocity derivation is disabled for this mode.
    bool collect_all_epochs_for_diagnostics = false;
    VelocityEndpointPolicy endpoint_policy =
        VelocityEndpointPolicy::OneSidedEndpointsCenteredInterior;
};

/**
 * @brief Typed same-run raw-P position/clock/velocity seed for one epoch.
 *
 * Position and clock fields are populated only for an accepted SPP solve and
 * remain NaN otherwise.  The source identity is copied from the input epoch;
 * it is never reconstructed from vector position or nearest time.
 */
struct EpochSeed {
    std::size_t input_epoch_index = std::numeric_limits<std::size_t>::max();
    GNSSTime time;
    std::size_t raw_source_index = std::numeric_limits<std::size_t>::max();
    std::int64_t raw_utc_time_millis = -1;

    EpochStatus status = EpochStatus::NotEvaluated;
    std::string reason;
    bool native_spp_status_available = false;
    SolutionStatus native_spp_status = SolutionStatus::NONE;
    GeometryRank geometry_rank;

    std::size_t raw_pseudorange_rows = 0;
    std::size_t raw_pseudorange_satellites = 0;
    std::size_t raw_clock_groups = 0;
    std::size_t corrected_pseudorange_rows = 0;
    std::size_t native_used_pseudorange_rows = 0;
    std::size_t corrected_clock_groups = 0;
    GNSSSystem reference_clock_group = GNSSSystem::UNKNOWN;
    std::vector<ClockGroupBias> clock_group_biases;
    bool preprocessing_diagnostics_available = false;
    std::size_t preprocessing_input_rows = 0;
    std::size_t preprocessing_accepted_rows = 0;
    std::size_t preprocessing_rejected_rows = 0;
    std::vector<PreprocessReasonCount> preprocessing_reason_counts;
    std::vector<PreprocessRowDiagnostic> preprocessing_rows;
    int satellites_used = 0;
    int iterations = 0;
    int degrees_of_freedom = 0;
    bool converged = false;

    double gdop = std::numeric_limits<double>::quiet_NaN();
    double pdop = std::numeric_limits<double>::quiet_NaN();
    double residual_rms_m = std::numeric_limits<double>::quiet_NaN();
    double max_abs_residual_m = std::numeric_limits<double>::quiet_NaN();

    Vector3d position_ecef =
        Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    double receiver_clock_bias_m = std::numeric_limits<double>::quiet_NaN();
    Vector3d velocity_ecef_mps =
        Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    bool has_velocity = false;
};

/**
 * @brief Result of the complete same-run raw-P seed stage.
 */
struct Result {
    bool ok = false;
    std::string failure_reason;
    EpochStatus failure_status = EpochStatus::NotEvaluated;
    bool collect_all_epochs_for_diagnostics = false;
    bool velocity_diagnostics_disabled = false;
    std::size_t input_epoch_count = 0;
    std::size_t evaluated_epoch_count = 0;
    std::size_t accepted_epoch_count = 0;
    std::size_t rejected_epoch_count = 0;
    VelocityEndpointPolicy endpoint_policy =
        VelocityEndpointPolicy::OneSidedEndpointsCenteredInterior;
    std::vector<EpochSeed> epochs;
};

/**
 * @brief Terminal status for the same-run raw-P handoff adapter.
 *
 * This adapter is intentionally narrower than the FGO graph.  It validates
 * that a raw-P result and the original in-memory epochs still have exact
 * identity alignment, then exposes finite position/velocity/clock fields and
 * an explicitly sourced raw receiver clock rate.  It never reads a saved
 * seed or derives a clock rate from position.
 */
enum class SeedAdapterStatus {
    NotEvaluated = 0,
    Accepted,
    InputSizeMismatch,
    EpochIdentityMismatch,
    RawPResultRejected,
    NonfinitePosition,
    MissingVelocity,
    NonfiniteVelocity,
    UnsupportedReferenceClockGroup,
    UnsupportedC7ClockMapping,
    MissingRawClockDrift,
    NonfiniteRawClockDrift,
};

/**
 * @brief One typed raw-P handoff record.
 *
 * C7 uses the official source order.  The adapter only certifies C[0] when
 * native SPP's reference group is GPS (QZSS is already folded into the native
 * GPS clock group).  C[0] remains certified even when supported non-GPS
 * source rows are present; their absent per-frequency initial values remain
 * unavailable and are only numerical graph guesses.  D is copied only from
 * the exact original epoch's finite raw receiver_clock_drift_mps field.
 */
struct RawPNoDopplerSeed {
    // A numerical starting value from bracketing same-run SPP solutions,
    // never an independently solved SPP epoch or a position observation.
    bool temporal_initial_guess = false;
    std::size_t initial_guess_left_source = std::numeric_limits<std::size_t>::max();
    std::size_t initial_guess_right_source = std::numeric_limits<std::size_t>::max();
    std::size_t epoch_index = std::numeric_limits<std::size_t>::max();
    GNSSTime time;
    std::size_t raw_source_index = std::numeric_limits<std::size_t>::max();
    std::int64_t raw_utc_time_millis = -1;
    SeedAdapterStatus status = SeedAdapterStatus::NotEvaluated;
    std::string reason;

    Vector3d position_ecef =
        Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Vector3d velocity_ecef_mps =
        Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    double clock_bias_m = std::numeric_limits<double>::quiet_NaN();
    double clock_rate_mps = std::numeric_limits<double>::quiet_NaN();
    GNSSSystem reference_clock_group = GNSSSystem::UNKNOWN;
    std::array<double, 7> clock_bias_components_m = [] {
        std::array<double, 7> values{};
        values.fill(std::numeric_limits<double>::quiet_NaN());
        return values;
    }();
    std::array<bool, 7> clock_bias_component_available{};
    bool has_position = false;
    bool has_velocity = false;
    bool has_clock = false;
    bool has_clock_rate = false;
    bool c7_clock_mapping_supported = false;
    std::size_t c7_supported_pseudorange_rows = 0;
    std::size_t c7_unsupported_pseudorange_rows = 0;
};

/**
 * @brief Result of adapting same-run raw-P output for a future no-D graph.
 *
 * `ok` means the typed handoff records are internally valid.  A valid
 * handoff may still have `graph_compatible == false` when native SPP did not
 * use GPS as its reference clock; source rows with unsupported C7 mappings
 * are counted per epoch and are rejected at the retained-factor boundary.
 * This distinction is deliberate: callers must not turn a structural
 * adapter success into graph admission without checking retained rows.
 */
struct RawPNoDopplerSeedAdapterResult {
    std::size_t temporal_initial_guess_count = 0;
    bool ok = false;
    bool graph_compatible = false;
    std::string graph_disabled_reason;
    SeedAdapterStatus status = SeedAdapterStatus::NotEvaluated;
    std::string failure_reason;
    std::size_t input_epoch_count = 0;
    std::size_t accepted_epoch_count = 0;
    std::size_t rejected_epoch_count = 0;
    std::vector<RawPNoDopplerSeed> seeds;
};

/**
 * @brief Return the official source C7 clock component for one P row.
 *
 * The numbering is shared with the native GTSAM source-clock factors.  A
 * negative result is an unsupported system/frequency mapping; callers must
 * reject that row rather than fold it into C[0] or invent an ISB estimate.
 */
int c7ClockComponentFor(GNSSSystem system, SignalType signal);

/**
 * @brief Adapt a completed native raw-P solve to a same-run typed handoff.
 *
 * `input_epochs` must be the exact in-memory vector passed to `solve`; the
 * function does not match by nearest time, satellite count, or retained
 * vector position.  Every accepted raw-P epoch needs a finite same-epoch raw
 * receiver clock drift.  Missing, nonfinite, or identity-misaligned D values
 * reject the adapter.  No graph is built or launched by this function.
 */
RawPNoDopplerSeedAdapterResult adaptSameRunNoDopplerSeeds(
    const std::vector<ObservationData>& input_epochs,
    const Result& raw_p_result);

// Explicit initialization-only completion. Original SPP statuses/coordinates
// are not changed. Only interior geometry/solver failures may be completed;
// exact identities, finite same-epoch raw D, GPS reference at both brackets,
// and a bounded bracket span in both GPS and UTC are required. C[0] at a
// completed epoch is an available numerical guess, not a measured clock.
RawPNoDopplerSeedAdapterResult initializeShortGaps(
    const std::vector<ObservationData>& input_epochs,
    const Result& diagnostic_result,
    double maximum_bracket_span_s = 4.0);

// Explicit numerical initialization for time-only leading graph states. Uses
// two later accepted native SPP anchors within four seconds, retaining every
// original rejected SPP diagnostic. Trailing gaps remain forbidden. Callers
// must verify continuous mapped IMU coverage and full native graph estimation;
// these guesses must never be emitted directly as final positions.
RawPNoDopplerSeedAdapterResult initializeWithLeadingGuesses(
    const std::vector<ObservationData>& input_epochs,
    const Result& diagnostic_result,
    double maximum_bracket_span_s = 4.0);


/**
 * @brief Compute the four-column position/shared-clock geometry rank.
 *
 * This helper is used by the stage before accepting an SPP result and is also
 * suitable for deterministic synthetic rank tests.  It consumes only
 * same-epoch satellite positions and the supplied numerical receiver seed;
 * no solution file or truth data is involved.
 */
GeometryRank assessGeometryRank(const std::vector<Vector3d>& satellite_positions,
                                const Vector3d& receiver_position_ecef);

/**
 * @brief Assess native SPP position/clock/ISB design rank.
 *
 * `clock_groups` is aligned with `satellite_positions` after native SPP
 * observation filtering.  With inter-system bias modeling enabled, one
 * column is added for every observed known group other than
 * `reference_clock_group`; the reference group uses the shared clock column.
 */
GeometryRank assessGeometryRank(
    const std::vector<Vector3d>& satellite_positions,
    const Vector3d& receiver_position_ecef,
    const std::vector<GNSSSystem>& clock_groups,
    GNSSSystem reference_clock_group,
    bool model_intersystem_bias);

/**
 * @brief Assess native SPP design rank with optional row weights.
 *
 * When `row_weights` is non-empty it is aligned with the filtered satellite
 * rows and the rank is evaluated on the same sqrt(weight)-scaled design
 * matrix used by native `SPPProcessor::solvePositionLS`.  An empty vector
 * preserves the unweighted preflight helper behavior.
 */
GeometryRank assessGeometryRank(
    const std::vector<Vector3d>& satellite_positions,
    const Vector3d& receiver_position_ecef,
    const std::vector<GNSSSystem>& clock_groups,
    GNSSSystem reference_clock_group,
    bool model_intersystem_bias,
    const std::vector<double>& row_weights);

/**
 * @brief Run the native SPP solver on a raw-P-only copy of every epoch.
 *
 * The input order and timestamps are preserved.  Any nonfinite, duplicate,
 * nonmonotonic, or over-gap timestamp, insufficient P geometry, rejected SPP
 * solve, unknown/unsupported clock grouping, insufficient post-filtered
 * position/clock/ISB rank, missing native clock estimate, or nonfinite output
 * fails the whole stage; no previous epoch is held and no missing velocity is
 * filled.  On success velocities use the explicit source-style endpoint
 * policy in `Config`.  Clock groups and rank columns follow native SPP's
 * GPS/QZSS reference and inter-system-bias model.  When
 * `Config::collect_all_epochs_for_diagnostics` is enabled, every epoch is
 * evaluated independently, failed epochs remain invalid, and velocity output
 * is intentionally unavailable rather than interpolated or held.
 */
Result solve(const std::vector<ObservationData>& input_epochs,
             const NavigationData& nav,
             const Config& config = {});

const char* rankStatusName(RankStatus status);
const char* epochStatusName(EpochStatus status);
const char* nativeSppStatusName(SolutionStatus status);
const char* velocityEndpointPolicyName(VelocityEndpointPolicy policy);
const char* clockGroupName(GNSSSystem group);
const char* seedAdapterStatusName(SeedAdapterStatus status);

}  // namespace raw_p_seed
}  // namespace libgnss
