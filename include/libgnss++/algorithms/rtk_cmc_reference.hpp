#pragma once

// Phase 2a (docs/imu_fusion.md-adjacent RTK work): CMC-aware DD reference-
// satellite selection with hysteresis for the Kalman-filter RTK path
// (RTKProcessor::RTKConfig::cmc_aware_reference_selection, opt-in, default
// OFF).
//
// Background: the KF path's reference selection (rtk_selection::
// selectSystemReferenceSatellite) picks the highest-elevation satellite per
// (system) group with no regard for multipath/NLOS-driven code-minus-
// carrier (CMC) deviations. Every DD pair in the group is formed against
// that one reference, so a biased high-elevation satellite poisons the
// whole group's DD residuals at once. An earlier FGO-pipeline experiment
// (commit 8cdff0c, "Add CMC-aware DD reference selection knob") ported a
// blanket "avoid CMC-excluded reference" rule and found it regressed:
// ~10.5k switch decisions flip-flopped between the top-2 elevation
// candidates on single-epoch CMC flicker, and because FGO's DD ambiguity/
// segment keys include the reference satellite, every switch severed
// fix-and-hold continuity for the whole group. The KF path (RTKProcessor)
// keys ambiguities per-satellite (SD states; the reference only enters the
// H matrix), so reference switching is structurally safer here -- but
// hysteresis is still required to avoid the same flapping.
//
// This header factors the two stateful pieces of the feature into small,
// independently unit-testable classes with no RTKProcessor/RINEX
// dependency:
//   - CmcSuspectTracker: per-satellite EWMA code-minus-phase baseline,
//     classifying each epoch's SD CMC deviation as suspect or not.
//   - ReferenceHysteresis: per-system switch-away/switch-back state
//     machine consuming per-epoch candidate (elevation, dual-freq, suspect)
//     snapshots.
// RTKProcessor (src/algorithms/rtk.cpp) owns one CmcSuspectTracker and one
// ReferenceHysteresis per system, feeding them from collectSatelliteData()/
// updateBias()'s existing per-satellite code/phase and slip-detection
// signals.

#include "../core/observation.hpp"

#include <cstddef>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace libgnss {
namespace rtk_cmc_reference {

// Per-satellite EWMA single-difference (rover-base) code-minus-phase
// baseline tracker. Mirrors FGOProcessor's CmcState (src/algorithms/
// fgo.cpp) but lives independently: the KF path
// (rtk_slip_detection.hpp) only exposes the raw
// singleDifferenceCodeMinusPhaseM() building block, with no standalone
// per-satellite CMC screening state of its own.
class CmcSuspectTracker {
public:
    // level_threshold_m: a satellite is classified suspect this call when
    // |cmc_m - running_baseline| exceeds this (RTKConfig::cmc_ref_level_m,
    // default 0.75 m, mirrors the FGO pipeline's --cmc-level). <= 0.0
    // disables classification entirely (classify() always returns false).
    // warmup_epochs/baseline_alpha mirror FGOConfig::
    // code_minus_carrier_warmup_epochs/_baseline_alpha's defaults (5, 0.05)
    // -- not separately exposed as RTKConfig knobs; Phase 2a's design only
    // calls for cmc_ref_level_m plus the hysteresis knobs below.
    explicit CmcSuspectTracker(double level_threshold_m, int warmup_epochs = 5,
                               double baseline_alpha = 0.05);

    // Classifies one satellite's this-epoch SD code-minus-phase deviation
    // (cmc_m, meters) against its own running baseline and updates internal
    // state. Returns true iff the satellite is CMC-suspect this epoch.
    // arc_restarted: caller-supplied signal (cycle slip / LLI / fresh
    // reacquisition) that resets this satellite's baseline instead of
    // classifying against a baseline that predates an ambiguity change --
    // mirrors FGOProcessor's rover_arc_restarted handling. A non-finite
    // cmc_m always returns false without disturbing existing state.
    bool classify(const SatelliteId& sat, double cmc_m, bool arc_restarted);

    // Drops tracked satellites not present in `seen`. Call once per epoch
    // after classify()-ing every currently observed satellite, to bound
    // memory over long runs.
    void pruneMissing(const std::set<SatelliteId>& seen);

private:
    struct BaselineState {
        double baseline_m = 0.0;
        bool has_baseline = false;
        int warmup_count = 0;
    };

    double level_threshold_m_;
    int warmup_epochs_;
    double baseline_alpha_;
    std::map<SatelliteId, BaselineState> baseline_by_sat_;
};

// Per-system DD reference-satellite hysteresis state machine. Tracks the
// currently active reference and decides, once per epoch, whether to
// switch away from it (after `switch_epochs` consecutive CMC-suspect
// epochs) or back to the natural (CMC-blind, highest-elevation) candidate
// (after that candidate has been non-suspect for `switch_epochs`
// consecutive epochs AND clears `return_min_elev_rad` over the current
// reference's elevation).
class ReferenceHysteresis {
public:
    struct Candidate {
        SatelliteId satellite;
        double elevation_rad = 0.0;
        // has_l2 && n2_active -- matches selectSystemReferenceSatellite's
        // own dual-frequency-preferred tie-break.
        bool dual_frequency = false;
        bool suspect = false;
    };

    ReferenceHysteresis() = default;

    // candidates: every this-epoch-eligible satellite in the system group
    // (has_l1 && n1_active), independent of CMC suspect status.
    // natural_ref/natural_ref_elevation_rad: the plain CMC-blind highest-
    // elevation pick for this epoch (dual-freq preferred), as already
    // computed by rtk_selection::selectSystemReferenceSatellite -- passed
    // in rather than recomputed here to avoid a second linear scan over the
    // candidate set.
    // switch_epochs/return_min_elev_rad: RTKConfig::cmc_ref_switch_epochs
    // and RTKConfig::cmc_ref_return_min_elev_deg (already converted to
    // radians) -- passed per call rather than fixed at construction so a
    // config change takes effect immediately without resetting state.
    //
    // switch_away_max_elev_drop_rad/switch_away_min_elev_rad: elevation-
    // quality gate on the SWITCH-AWAY decision only (RTKConfig::
    // cmc_ref_switch_max_elev_drop_deg/cmc_ref_switch_min_elev_deg, already
    // converted to radians). Even once suspect_streak_ reaches
    // switch_epochs, the switch is only actually performed if the best
    // non-suspect candidate's elevation is (a) within
    // switch_away_max_elev_drop_rad below the current (suspect)
    // reference's elevation this epoch, AND (b) itself at least
    // switch_away_min_elev_rad. When the gate blocks the switch, the
    // current reference is kept -- same as the pre-existing "every
    // candidate suspect" fallback. Defaults (+infinity / -infinity)
    // disable both checks, preserving prior callers' behavior unchanged.
    // The switch-back path (natural-reference return) is untouched by this
    // gate: it already requires natural_ref_elevation_rad to exceed the
    // current reference by return_min_elev_rad, which is a strictly
    // stronger elevation condition in the return direction.
    //
    // Returns false (state left untouched, out_ref unset) iff `candidates`
    // is empty -- caller should skip DD formation for the system entirely
    // in that case, matching the pre-existing no-candidate behavior.
    // Otherwise returns true and sets out_ref to the (possibly unchanged)
    // active reference; `switched` reports whether this call changed the
    // active reference's identity relative to entering the call (false on
    // the very first successful call for a fresh instance -- initial
    // acquisition is not a "switch").
    bool update(const std::vector<Candidate>& candidates, const SatelliteId& natural_ref,
               double natural_ref_elevation_rad, int switch_epochs, double return_min_elev_rad,
               SatelliteId& out_ref, bool& switched,
               double switch_away_max_elev_drop_rad = std::numeric_limits<double>::infinity(),
               double switch_away_min_elev_rad = -std::numeric_limits<double>::infinity());

    bool hasReference() const { return has_current_ref_; }
    SatelliteId currentReference() const { return current_ref_; }

private:
    bool has_current_ref_ = false;
    SatelliteId current_ref_;
    int suspect_streak_ = 0;
    bool has_return_candidate_ = false;
    SatelliteId return_candidate_;
    int return_candidate_clean_streak_ = 0;
};

}  // namespace rtk_cmc_reference
}  // namespace libgnss
