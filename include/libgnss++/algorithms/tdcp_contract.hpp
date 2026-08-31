#pragma once

#include <cmath>

namespace libgnss::tdcp_contract {

/**
 * @brief Truth-free decision for one adjacent same-satellite/same-signal ADR
 * pair.
 *
 * Android's accumulated-delta-range value is converted to metres by the raw
 * adapter before it reaches the FGO problem builder.  This helper deliberately
 * contains only the temporal gates: the caller owns the
 * (SatelliteId, SignalType) key lookup, while this contract owns the physical
 * time, loss-of-lock, finiteness, and code-minus-carrier jump checks.  Keeping
 * the decision in one small function makes the raw and synthetic paths share
 * exactly the same fail-closed rules.
 */
enum class PairRejectReason {
    Accepted,
    Gap,
    ClockDiscontinuity,
    LossOfLock,
    NonFiniteMeasurement,
    CodePhaseJump,
};

struct PairDecision {
    PairRejectReason reason = PairRejectReason::NonFiniteMeasurement;

    bool accepted() const { return reason == PairRejectReason::Accepted; }
};

inline PairDecision evaluateAdjacentPair(
    double dt_s,
    bool previous_loss_of_lock,
    bool current_loss_of_lock,
    double delta_carrier_m,
    double delta_code_m,
    double max_gap_s,
    bool reject_loss_of_lock,
    bool reject_code_phase_jump,
    double code_phase_jump_threshold_m,
    bool previous_clock_discontinuity = false,
    bool current_clock_discontinuity = false) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0 ||
        (max_gap_s > 0.0 && dt_s > max_gap_s)) {
        return {PairRejectReason::Gap};
    }
    if (previous_clock_discontinuity || current_clock_discontinuity) {
        return {PairRejectReason::ClockDiscontinuity};
    }
    if (reject_loss_of_lock &&
        (previous_loss_of_lock || current_loss_of_lock)) {
        return {PairRejectReason::LossOfLock};
    }
    if (!std::isfinite(delta_carrier_m) || !std::isfinite(delta_code_m)) {
        return {PairRejectReason::NonFiniteMeasurement};
    }
    const double code_phase_jump_m =
        std::abs(delta_carrier_m - delta_code_m);
    if (reject_code_phase_jump && code_phase_jump_threshold_m > 0.0 &&
        (!std::isfinite(code_phase_jump_m) ||
         code_phase_jump_m > code_phase_jump_threshold_m)) {
        return {PairRejectReason::CodePhaseJump};
    }
    return {PairRejectReason::Accepted};
}

}  // namespace libgnss::tdcp_contract
