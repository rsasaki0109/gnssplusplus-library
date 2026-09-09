#pragma once

#include <cmath>
#include <limits>

namespace libgnss::tdcp_contract {

/**
 * @brief Result of the Phase138 fixed-initial-range TDCP adapter.
 *
 * `tdcp_native_m` is the already prepared Phase118 carrier/clock/atmosphere
 * difference.  The Phase135 affine factor models position only as a delta
 * about its initial endpoint, so the source-consistent initial range change
 * is moved into the measurement constant exactly once:
 *
 *   tdcp_phase138 = tdcp_native - (rho_current_initial-rho_previous_initial)
 *
 * This helper intentionally has no pair admission, wavelength, noise, or
 * Jacobian behavior.  Callers must provide the same source geometry used by
 * the affine factor's LOS and reject nonfinite values rather than fallback.
 */
struct Phase138AffineTdcpMeasurement {
    double tdcp_m = std::numeric_limits<double>::quiet_NaN();
    double range_delta_m = std::numeric_limits<double>::quiet_NaN();
};

inline bool applyPhase138AffineTdcpAnchorRangeConstant(
    double tdcp_native_m, double previous_initial_range_m,
    double current_initial_range_m, Phase138AffineTdcpMeasurement& output) {
    if (!std::isfinite(tdcp_native_m) ||
        !std::isfinite(previous_initial_range_m) ||
        !std::isfinite(current_initial_range_m)) {
        return false;
    }
    output.range_delta_m = current_initial_range_m - previous_initial_range_m;
    output.tdcp_m = tdcp_native_m - output.range_delta_m;
    return std::isfinite(output.range_delta_m) && std::isfinite(output.tdcp_m);
}

/**
 * @brief Build the metre-valued ordinary TDCP carrier observable.
 *
 * The historical native path keeps the broadcast ionosphere/troposphere
 * terms in the prepared carrier value.  Phase120's opt-in source-parity
 * selector follows the official `resL` observable and retains only the raw
 * carrier metres plus the satellite-clock metres.  The factor and all pair
 * admission rules remain outside this helper.
 *
 * A non-finite raw carrier or satellite-clock term is always fail-closed.  In
 * the legacy branch atmospheric terms are also required to be finite because
 * that branch uses them; the source-parity branch deliberately does not read
 * them into the ordinary TDCP measurement.
 */
inline double ordinaryTdcpCarrierMeters(
    double raw_carrier_m, double satellite_clock_m,
    double ionosphere_delay_m, double troposphere_delay_m,
    bool official_resl_atmosphere_cancellation) {
    if (!std::isfinite(raw_carrier_m) ||
        !std::isfinite(satellite_clock_m)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (official_resl_atmosphere_cancellation) {
        const double measurement_m = raw_carrier_m + satellite_clock_m;
        return std::isfinite(measurement_m)
                   ? measurement_m
                   : std::numeric_limits<double>::quiet_NaN();
    }
    if (!std::isfinite(ionosphere_delay_m) ||
        !std::isfinite(troposphere_delay_m)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double measurement_m =
        raw_carrier_m + satellite_clock_m - troposphere_delay_m +
        ionosphere_delay_m;
    return std::isfinite(measurement_m)
               ? measurement_m
               : std::numeric_limits<double>::quiet_NaN();
}

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
