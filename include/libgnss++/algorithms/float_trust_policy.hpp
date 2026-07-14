#pragma once

// WP9: graceful degradation of the RTK float filter's per-epoch
// position-trust/reseed policy (see RTKProcessor::resetPositionToSPP() /
// rememberSolution() in rtk.cpp). WP8's canyon forensics found the
// root cause of the ~119 m float divergence in urban canyons: every
// epoch that fails to "refresh trust" (rtk.cpp:3747-3762 -- FIXED, or
// FLOAT with >=5 sats and a small jump vs the last trusted position)
// gets its position covariance reset to a wide 900 m^2/axis prior and
// reseeded from a fresh, possibly NLOS-corrupted SPP fix.
//
// This header holds the pure, unit-testable math for two opt-in
// alternatives to that unconditional wide reset, selected via
// RTKConfig::float_trust_policy (default LEGACY, bit-identical to
// pre-WP9 behavior):
//
// - CV_PREDICT: propagate the previous position with a constant-velocity
//   predict instead of reseeding from SPP, and grow the position
//   covariance by a process-noise rate (trust_lapse_qpos_m2_per_s)
//   instead of jumping straight to 900 m^2/axis.
// - SCALED_RESET: keep the SPP reseed, but scale the reset variance with
//   how long trust has been lapsed instead of using a flat 900.
// - LAPSE_GATED (WP10): the SCALED_RESET law regressed run2/run3 because
//   its dt=0 variance (25 m^2) is tighter than legacy's 900 m^2 for
//   *every* lapse, however short/benign. LAPSE_GATED makes this
//   conditional: while the continuous trust lapse is shorter than
//   trust_lapse_gate_s, behave EXACTLY like LEGACY (falls through to the
//   unmodified wide-reset branch in resetPositionToSPP() -- bit-identical
//   below the gate by construction, not just numerically close); once the
//   lapse's duration exceeds the gate, switch to the SCALED_RESET law.
//   The lapse duration is "time since the position last refreshed trust"
//   (dt_since_trust in resetPositionToSPP()), which is naturally reset to
//   ~0 every time rememberSolution() refreshes last_trusted_time_ -- no
//   separate clock/counter state is needed. See lapseGateExceeded() below.
//
// All three non-LEGACY policies are capped at the legacy 900 m^2/axis
// value once engaged, so behavior is bounded-divergent from legacy by
// construction under a long enough trust drought, and all are strict
// no-ops (LEGACY code path taken unconditionally) whenever the previous
// epoch actually refreshed trust -- i.e. on every epoch of a healthy
// segment, matching WP8's own framing that this should only engage
// during a trust drought. LAPSE_GATED goes one step further: it is also
// a strict no-op (LEGACY path) for any *short* trust lapse below its
// gate, which is exactly the WP10 fix for SCALED_RESET's run2/run3
// regression (see WP10_REPORT.md).
//
// Everything here is a free function over plain doubles/Eigen vectors --
// no RTK-specific types (GNSSTime, SatelliteId, ...) -- mirroring
// nlos_weights.hpp's separation of pure math from RTKProcessor wiring.

#include <Eigen/Dense>

namespace libgnss {
namespace float_trust_policy {

/// Selects which policy resetPositionToSPP() uses once trust has lapsed.
/// LEGACY (default) never consults any of the functions below --
/// RTKProcessor's existing unconditional wide-reset code path runs
/// unchanged, so this is bit-identical to pre-WP9 behavior.
enum class FloatTrustPolicy {
    LEGACY = 0,
    CV_PREDICT = 1,
    SCALED_RESET = 2,
    /// WP10: SCALED_RESET, but only once a continuous trust lapse exceeds
    /// a configured gate (RTKConfig::trust_lapse_gate_s); below the gate,
    /// behaves exactly like LEGACY. See lapseGateExceeded().
    LAPSE_GATED = 3,
};

/// True when "trust" has lapsed, i.e. graceful degradation should engage
/// instead of the legacy unconditional wide reset.
///
/// ``has_last_trusted`` is whether any trusted position/time has ever
/// been recorded at all (false only very early in a run, before the
/// first FIXED/good-FLOAT solution). ``trust_refreshed_last_epoch`` is
/// whether the immediately preceding *processed* epoch specifically
/// refreshed trust (i.e. the caller's own last_trusted_time_ equals its
/// last_epoch_time_) -- trust "lapses" the moment one epoch fails to
/// refresh it, matching WP8's framing of a self-reinforcing drought that
/// starts as soon as trust-refresh opportunities dry up.
bool hasTrustLapsed(bool has_last_trusted, bool trust_refreshed_last_epoch);

/// CV_PREDICT: grows a previous per-axis position variance by a
/// process-noise rate (``qpos_m2_per_s``, m^2/s) integrated over ``dt_s``
/// seconds, capped at ``legacy_var_m2`` (900 by construction) so a long
/// enough trust drought converges to the same wide prior legacy always
/// uses. Non-finite/negative inputs are clamped defensively (previous
/// variance defaults to the legacy cap; qpos/dt default to 0, i.e. no
/// growth) rather than propagating NaN into the filter.
double growPositionVarianceCvPredict(double previous_var_m2,
                                      double qpos_m2_per_s,
                                      double dt_s,
                                      double legacy_var_m2);

/// SCALED_RESET: reset variance that grows *quadratically* with how long
/// trust has been lapsed (``dt_since_trust_s``), starting from a tight
/// ``base_var_m2`` (25 m^2/axis, matching the existing trusted-seed tight
/// variance) and capped at ``legacy_var_m2``. Matches the task-specified
/// formula ``var_pos = min(900, base + qpos * dt^2)``.
double scaledResetPositionVariance(double base_var_m2,
                                    double qpos_m2_per_s,
                                    double dt_since_trust_s,
                                    double legacy_var_m2);

/// Two-point finite-difference ECEF velocity estimate from the last two
/// *trusted* position samples (the task's "last N trusted deltas" source,
/// N=2 here -- documented in WP9_REPORT.md). Returns zero velocity
/// (caller should then just hold position) when ``dt_s`` is non-finite,
/// non-positive, or implausibly large (> ``max_dt_s``, i.e. the two
/// trusted samples are too far apart in time to give a meaningful
/// instantaneous-velocity estimate), or either position is non-finite.
Eigen::Vector3d estimateVelocityFromTrustedDeltas(const Eigen::Vector3d& newer_position,
                                                   const Eigen::Vector3d& older_position,
                                                   double dt_s,
                                                   double max_dt_s);

/// Constant-velocity position predict: propagates ``previous_position``
/// by ``velocity_mps`` (ECEF, m/s) over ``dt_s`` seconds. Falls back to
/// holding ``previous_position`` unchanged (zero-velocity CV predict) if
/// either input is non-finite.
Eigen::Vector3d predictPositionConstantVelocity(const Eigen::Vector3d& previous_position,
                                                 const Eigen::Vector3d& velocity_mps,
                                                 double dt_s);

/// WP10 LAPSE_GATED boundary test: true once a continuous trust lapse of
/// ``dt_since_trust_s`` seconds has reached or exceeded ``gate_s``
/// seconds, i.e. the caller should switch from LEGACY's flat 900 m^2/axis
/// reset to the SCALED_RESET law for this epoch. The boundary is
/// inclusive (``dt_since_trust_s == gate_s`` returns true) so a gate of
/// exactly 0 s means "always use the scaled law once trust has lapsed at
/// all", the natural degenerate case swept at the low end of a gate grid.
/// Non-finite/negative inputs are clamped to 0 (a non-finite/negative
/// elapsed lapse duration is defensively treated as "no lapse yet" and a
/// non-finite/negative gate as "trigger immediately"), matching this
/// module's existing defensive-clamping convention.
bool lapseGateExceeded(double dt_since_trust_s, double gate_s);

}  // namespace float_trust_policy
}  // namespace libgnss
