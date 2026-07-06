#pragma once

// Stage 2 (design-only, docs/design.md 4): tightly-coupled DD-RTK + IMU path.
//
// This header intentionally declares NO functions and has NO corresponding
// .cpp -- it exists purely to record, in compilable form, the struct shapes a
// future tightly-coupled extension would need, per the integration decisions
// in docs/design.md 4.1-4.2 and 7:
//
//   - Run a SEPARATE, parallel 15-state ESKF (fusion::FusionState) rather
//     than extending RTKProcessor's own fixed-size state layout
//     (rtk.hpp's MAXSAT-indexed ambiguity/iono slots), to avoid an invasive
//     rewrite of that delicate, already-well-tested machinery.
//   - The parallel filter would CALL INTO RTKProcessor's existing
//     double-difference measurement construction
//     (rtk_measurement::assembleMeasurementSystem /
//     RTKProcessor::buildMeasurementBlocks) re-derived against the fused
//     position instead of RTKProcessor's own baseline-only state, then hand
//     the resulting (design_matrix, residuals, covariance) triple to
//     fusion_update::applyDenseUpdate (docs/design.md 4.2) instead of
//     rtk_update::applyMeasurementUpdate.
//   - Ambiguity state augmentation mirrors RTKProcessor's existing
//     getOrCreateN1Index/getOrCreateN2Index seeding + lock-count/hold
//     lifecycle bookkeeping (rtk.hpp) close to verbatim, since that
//     bookkeeping is about *which DD pairs exist and are trustworthy*, not
//     about *which estimator consumes them* (docs/design.md 4.1).
//   - Cycle-slip detection (rtk_slip_detection.hpp) and post-hoc validation
//     (rtk_validation.hpp's PositionSolution-stream guards) are already
//     estimator-agnostic and would be reused unmodified.
//
// See docs/design.md 4.3 for why fgo.hpp is NOT the near-term host for IMU
// preintegration (no SO(3)/manifold state or retraction step exists in the
// optimizer core today) -- the sequential ESKF above is the recommended
// vehicle for both Stage 1 (this module) and Stage 2.

#include <vector>

#include <Eigen/Dense>

#include <libgnss++/fusion/fusion_types.hpp>

namespace libgnss {
namespace dd_imu_bridge {

/**
 * @brief Stage 2 sketch: one live double-difference ambiguity, appended as an
 * extra scalar block to the parallel ESKF's covariance on first sight of a
 * (satellite, frequency) pair -- the ESKF-side analogue of
 * RTKProcessor::getOrCreateN1Index/getOrCreateN2Index's seeding formula, not
 * a new bookkeeping concept.
 */
struct AmbiguityErrorState {
    int satellite_prn = 0;
    int frequency_index = -1;
    int generation = 0;      ///< bumped on cycle slip / forced reset, forces a fresh scalar slot
    double float_value_cycles = 0.0;
    double variance_cycles2 = 0.0;
    bool held = false;       ///< fixed-and-held: folded into the DD measurement as a constant
};

/**
 * @brief Stage 2 sketch: the augmented state a tightly-coupled filter would
 * carry -- the existing 15-state FusionState plus a variable-length block of
 * live DD ambiguities. Deliberately NOT wired into FusionState itself (which
 * stays exactly 15-wide for Stage 1) since the ambiguity block's size changes
 * every epoch as satellites rise/set/slip.
 */
struct TightlyCoupledState {
    FusionState eskf;
    std::vector<AmbiguityErrorState> ambiguities;
};

}  // namespace dd_imu_bridge
}  // namespace libgnss
