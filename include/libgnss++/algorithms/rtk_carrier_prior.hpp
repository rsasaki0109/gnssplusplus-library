#pragma once

#include <Eigen/Dense>

#include <vector>

namespace libgnss::rtk_carrier_prior {

/**
 * Carrier-propagated position prior for stationary/slow NLOS canyons
 * (RTKConfig::enable_canyon_carrier_prior).
 *
 * Failure mode addressed: float ambiguities are seeded and averaged from
 * single-difference code (b = cp - pr / lambda). Under an overpass the direct
 * signal of high-elevation satellites is blocked (C/N0 -12..-15 dB), their code
 * is biased +4..+14 m while the carrier stays continuous. The kinematic filter
 * re-seeds the position with a 900 m^2 prior every epoch and so discards the
 * position/ambiguity cross-covariance; with no geometry change the n-3 code
 * combinations cannot separate a code bias from position, and the biased code
 * defines the 3 remaining ambiguity combinations (10-29 m vertical error).
 *
 * Remedy: keep the previous posterior (with its cross-covariances) and advance
 * it by a validated time-differenced carrier (TDCP) displacement instead of
 * re-seeding it from code. Engaged only while the code is inconsistent with
 * that carrier-propagated position.
 */

/// Constants. Each one is justified physically; none was tuned on a score.
struct TdcpConfig {
    /// Zenith 1-sigma of the epoch-to-epoch difference of one single-difference
    /// (rover - base) L1 carrier phase [m]: ~1-2 mm tracking noise per receiver
    /// at 40 dB-Hz, sqrt(2) for the rover-base difference and sqrt(2) for the
    /// epoch difference => ~4 mm, rounded up for residual multipath: 5 mm.
    /// Scaled by 1 / sin(elevation) per row.
    double sigma_zenith_m = 0.005;
    /// Lowest sin(elevation) used for row weighting (10 deg => 0.17).
    double min_sin_elevation = 0.17;
    /// 3 position + 1 receiver-clock unknowns + at least one redundant row so a
    /// single cycle slip is observable.
    int min_rows = 5;
    /// RMS of the unweighted residuals [m]. Pure noise (5 mm) stays below
    /// 1 cm; 2 cm is 4 sigma of noise and well below a half-cycle slip (L1:
    /// 9.5 cm), which with >= 6 rows leaves a visible residual. With exactly
    /// 5 rows (one redundant equation) detectability of a single slip depends
    /// on the geometry of that row.
    double max_rms_m = 0.02;
    /// Largest normalized residual |r| / sigma_i accepted for a single row.
    double max_normalized_residual = 4.0;
};

/// One satellite: y = -los . dx + dclk (all in metres).
struct TdcpRow {
    /// Unit vector rover -> satellite at the current epoch.
    Eigen::Vector3d los = Eigen::Vector3d::Zero();
    /// Observed epoch-difference of the SD carrier [m] minus the predicted
    /// epoch-difference of the SD geometry at the previous position [m].
    double delta_m = 0.0;
    /// Row 1-sigma [m] (already elevation scaled).
    double sigma_m = 0.005;
};

struct TdcpSolution {
    bool ok = false;
    Eigen::Vector3d displacement_ecef = Eigen::Vector3d::Zero();
    /// Covariance of the displacement [m^2], scaled up when the weighted
    /// residuals exceed their nominal level.
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    double clock_term_m = 0.0;
    double rms_m = 0.0;
    int rows_used = 0;
    int rows_rejected = 0;
};

/// Weighted least squares for the displacement with iterative removal of the
/// worst row while the RMS / normalized-residual tests fail and more than
/// `min_rows` rows remain. Returns ok = false when the tests cannot be met.
TdcpSolution solveTdcpDisplacement(const std::vector<TdcpRow>& rows,
                                   const TdcpConfig& config = TdcpConfig{});

/// Code-minus-geometry residuals of the satellites [m] (receiver clock still
/// inside, removed by the median). Counts the satellites at or above
/// `min_elevation_rad` whose residual is farther than `threshold_m` from the
/// median of all satellites in the vector.
struct CodeConsistency {
    int inconsistent_rows = 0;
    int rows = 0;
    double median_m = 0.0;
};
CodeConsistency evaluateCodeConsistency(const std::vector<double>& residuals_m,
                                        const std::vector<double>& elevations_rad,
                                        double threshold_m,
                                        double min_elevation_rad);

struct MonitorConfig {
    /// Zenith-side satellites whose code disagrees with the carrier-propagated
    /// position by more than the filter's own 3 m outlier gate. One row is
    /// enough: above 45 deg elevation ordinary multipath is bounded to ~1-2 m
    /// (the reflector path excess shrinks with elevation), so a >3 m error on
    /// a zenith-side satellite means its direct signal is blocked, and these
    /// satellites carry the leverage on the vertical position. Onset must be
    /// caught on the first epoch: one legacy update with the biased row already
    /// absorbs it into position/ambiguities and the evidence disappears.
    int min_inconsistent_rows = 1;
    /// Elevation above which a code error is attributed to blockage [rad].
    double min_elevation_rad = 0.7853981633974483;  // 45 deg
    /// The legacy re-seed resumes only after this long without a zenith-side
    /// code inconsistency AND with a code/carrier-consistent update on every
    /// epoch: 5 s (50 epochs at 10 Hz). A satellite dropping out of tracking
    /// removes the evidence without removing the blocked sky, so the absence
    /// of evidence alone must not release the prior.
    double release_hold_s = 5.0;
    /// A chain starts only from a posterior that has been code/carrier
    /// consistent (prefit RMS <= 1 m) on every epoch for this long. A posterior
    /// seeded after an outage or fresh ambiguity re-initialisation can be
    /// consistent and still metres off; 5 s lets the code average (0.3-0.5 m
    /// DD code noise over 50 epochs) and rejects those transients.
    double min_healthy_streak_s = 5.0;
    /// Epoch gap that breaks a healthy streak [s].
    double max_streak_gap_s = 1.5;
    /// Upper bound on one engaged episode. The carrier-rate error budget is
    /// ~1 cm/s (multipath-induced phase-rate bias) => 1.8 m after 180 s, and
    /// stops at signalised junctions rarely exceed 3 minutes.
    double max_engaged_s = 180.0;
};

struct MonitorInput {
    double time_s = 0.0;
    /// A carrier-propagated position exists for this epoch (TDCP solution or a
    /// short constant-velocity bridge).
    bool prior_available = false;
    /// The prior comes from the constant-velocity bridge, not a TDCP solution.
    bool prior_bridged = false;
    /// The previous epoch's update finished with code/carrier prefit RMS small
    /// enough to trust its posterior as the start of a chain.
    bool previous_epoch_healthy = false;
    int inconsistent_rows = 0;
};

/// Engage / hold / release decisions of the prior. Pure state machine.
class Monitor {
public:
    Monitor() = default;
    explicit Monitor(const MonitorConfig& config) : config_(config) {}

    /// Returns true when the carrier prior must replace the re-seed this epoch.
    bool update(const MonitorInput& input);
    /// The prior was contradicted by the measurements (posterior pulled
    /// farther than the prior could plausibly be wrong, or most code rows
    /// rejected against it): release and require a fresh healthy streak.
    void reportContradiction();

    bool engaged() const { return engaged_; }
    double engagedSince() const { return engaged_since_s_; }
    /// Seconds of uninterrupted code/carrier-consistent updates before `time_s`.
    double healthyStreak(double time_s) const {
        return has_healthy_since_ ? time_s - healthy_since_s_ : 0.0;
    }
    void reset();

private:
    MonitorConfig config_;
    bool engaged_ = false;
    double engaged_since_s_ = 0.0;
    double last_bad_s_ = 0.0;
    bool has_healthy_since_ = false;
    double healthy_since_s_ = 0.0;
    bool has_last_call_ = false;
    double last_call_s_ = 0.0;
};

}  // namespace libgnss::rtk_carrier_prior
