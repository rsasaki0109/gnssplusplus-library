#pragma once

// WP7: optional per-epoch, per-satellite LOS/NLOS soft-weighting for the RTK
// measurement update. The weight source is external (e.g. a PLATEAU 3D-mesh
// BVH ray-trace mask, see experiments/build_per_epoch_nlos_csv.py in the
// parent gnss_gpu repo) and is loaded from a small CSV contract:
//
//   tow,sat,los_prob
//   187470.000,G01,1.0
//   187470.000,G14,0.0
//
// ``los_prob`` in [0, 1]: 1.0 = confident line-of-sight, 0.0 = confident
// non-line-of-sight (NLOS/multipath). Missing (tow, sat) pairs default to
// los_prob = 1.0 (LOS, no inflation) so a partial mask never *hides*
// satellites, it only ever down-weights the ones it has evidence for.
//
// Everything here is a free function / value type — no file I/O happens
// inside RTKProcessor itself, so RTKConfig (copied by value all over the
// existing CLI apps) stays POD-simple. Callers (apps/gnss_solve.cpp) load
// the table once via loadNlosWeightsCsv() and hand a shared_ptr to
// RTKProcessor::setNlosWeightTable().

#include <map>
#include <string>

namespace libgnss {
namespace nlos_weights {

/// Sigma-inflation mapping applied to a per-satellite LOS probability.
/// OFF (default) is always a no-op, independent of whether a weight table
/// is loaded — this is what keeps the feature bit-identical when unused.
enum class NlosWeightMode {
    OFF = 0,
    TWO_TIER = 1,
    CONTINUOUS = 2,
    /// WP8: hard exclusion. Satellites with los_prob below
    /// nlos_exclude_threshold are dropped from DD formation entirely
    /// (rather than down-weighted) — see nlosShouldExclude() /
    /// nlosExclusionGuardAllows() and RTKProcessor::buildSelectionSnapshot(),
    /// the single function both the float KF (buildMeasurementBlocks()) and
    /// the AR candidate set (buildDoubleDifferencePairs()) call, so
    /// filtering there applies identically to both consumers.
    EXCLUDE = 3,
};

/// Per-epoch, per-satellite LOS-probability table.
///
/// Keyed by ``tow`` (seconds, ascending) -> {satellite id string -> los_prob}.
/// Lookup tolerates small tow mismatches (receiver logging jitter) via
/// ``lookupLosProb``'s tolerance parameter, mirroring
/// ``gnss_gpu.nlos_mask.lookup_nlos_sets`` in the Python asset-generation
/// pipeline this table is usually built from.
struct NlosWeightTable {
    std::map<double, std::map<std::string, double>> by_tow;

    bool empty() const { return by_tow.empty(); }
};

/// Parses a ``tow,sat,los_prob`` (or the equivalent ``tow,epoch_idx,prn,
/// is_los,...`` extended contract emitted by build_per_epoch_nlos_csv.py)
/// CSV file. Column matching is header-driven and case-insensitive; extra
/// columns are ignored. Throws std::runtime_error if the file cannot be
/// opened or the header is missing a usable satellite/tow/probability
/// column triple.
NlosWeightTable loadNlosWeightsCsv(const std::string& path);

/// Resolves the LOS probability for one (tow, satellite) pair.
/// Returns 1.0 (LOS, no inflation) when the table is empty or no entry is
/// found within ``tow_tolerance_s`` of ``tow`` for ``sat_id``.
double lookupLosProb(const NlosWeightTable& table,
                      double tow,
                      const std::string& sat_id,
                      double tow_tolerance_s);

/// Maps a LOS probability to a measurement *variance* (sigma^2) multiplier
/// (always >= 1.0; 1.0 = no-op).
///
/// - OFF: always 1.0, regardless of los_prob.
/// - TWO_TIER: los_prob < two_tier_los_threshold -> two_tier_inflation^2,
///   else 1.0. Simple binary classification, matching the existing
///   LOS/NLOS mask contract (is_los in {0, 1}).
/// - CONTINUOUS: 1 / max(los_prob, floor), i.e. sigma *= 1 / sqrt(max(los_prob,
///   floor)) (the mapping suggested by the WP7 task spec). Degrades to a
///   TWO_TIER-like step for binary los_prob inputs when floor is small.
double nlosVarianceInflationFactor(double los_prob,
                                    NlosWeightMode mode,
                                    double continuous_floor,
                                    double two_tier_los_threshold,
                                    double two_tier_sigma_inflation);

/// WP8 hard-exclusion per-satellite decision: true when ``mode ==
/// EXCLUDE`` and ``los_prob`` is strictly below ``exclude_threshold``.
/// Always false for every other mode (OFF/TWO_TIER/CONTINUOUS never
/// exclude, only inflate-or-not) and false for a non-finite ``los_prob``
/// (missing/unclassified satellites are never excluded, mirroring
/// lookupLosProb()'s "default to LOS" behavior). This is a pure,
/// per-satellite building block — the two epoch-level safety guards
/// (minimum surviving satellite count, never dropping a system's last
/// reference-satellite candidate) are applied by the caller; see
/// nlosExclusionGuardAllows() for the first guard and
/// RTKProcessor::buildSelectionSnapshot() (rtk.cpp) for both, plugged in
/// upstream of both DD-formation consumers.
bool nlosShouldExclude(double los_prob, NlosWeightMode mode, double exclude_threshold);

/// WP8 exclusion safety guard ("--nlos-min-sats"): given how many
/// satellites are in the epoch's full candidate set (``total_sats``) and
/// how many the per-satellite threshold test (nlosShouldExclude) marked
/// for exclusion (``excluded_count``), returns whether exclusion should
/// actually be applied this epoch. Returns false (guard trips, caller
/// keeps every satellite, i.e. exclusion is skipped for that epoch) when
/// nothing was marked for exclusion, or when excluding would leave fewer
/// than ``min_sats`` satellites remaining — never degrade geometry below
/// solvability for the sake of dropping NLOS satellites.
bool nlosExclusionGuardAllows(int total_sats, int excluded_count, int min_sats);

/// WP10 (WP8 recommendation 2, "--nlos-min-los-sats"): AR-acceptance-side
/// gate, independent of and orthogonal to EXCLUDE mode -- this never
/// removes satellites from the float-KF measurement update, it only
/// vetoes *attempting/accepting* an ambiguity-resolution fix for the
/// epoch when too few of the AR candidate set's satellites are
/// LOS-flagged. Returns true (AR may proceed) when ``min_los_sats <= 0``
/// (disabled, the default) or when ``los_sat_count >= min_los_sats``.
/// The caller (RTKProcessor::resolveAmbiguities()) is responsible for
/// counting ``los_sat_count`` over the unique satellites already present
/// in the AR candidate set's DD pairs -- this function is the pure,
/// unit-testable threshold comparison only.
bool nlosMinLosSatsGateAllows(int los_sat_count, int min_los_sats);

}  // namespace nlos_weights
}  // namespace libgnss
