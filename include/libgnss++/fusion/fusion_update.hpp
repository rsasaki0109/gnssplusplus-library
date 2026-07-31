#pragma once

#include <Eigen/Dense>

#include <libgnss++/fusion/fusion_measurement.hpp>
#include <libgnss++/fusion/fusion_types.hpp>

namespace libgnss {
namespace fusion_update {

/**
 * @brief Diagnostic result of applyDenseUpdate(). Same diagnostic shape as
 * rtk_update::FilterUpdateResult (rtk_update.hpp) for consistency of
 * downstream JSON/CSV reporting conventions, trimmed to the fields relevant
 * to a dense (no outlier-suppression) update, plus an explicit
 * rejected_by_innovation_gate flag mirroring FilterUpdateResult's field of
 * the same name (needed so callers/tests can distinguish "rejected by NIS
 * gate" from "singular innovation covariance").
 */
struct FusionUpdateResult {
    bool ok = false;
    bool rejected_by_innovation_gate = false;
    int observation_count = 0;
    double normalized_innovation_squared = 0.0;
    double normalized_innovation_squared_per_observation = 0.0;
};

/**
 * @brief Dense Joseph-form EKF measurement update over the 15-wide error
 * state.
 *
 * NOT a reuse of kalman.hpp's kalmanFilter(): that function gates on
 * `x[i] != 0.0 && P[i,i] > 0.0` to find "active" states, a convention correct
 * for RTKLIB's sparse total-state vector but wrong for an error state that is
 * deliberately, legitimately all-zero after every injection (docs/design.md
 * 0.4, 3.5) -- kalmanFilter() would find zero active states and fail on the
 * very first update. This function is therefore always dense (no active-state
 * gating) and uses the more numerically robust Joseph-form covariance update
 * `P = (I-KH) P (I-KH)^T + K R K^T` (kalman.hpp uses the cheaper but less
 * robust `(I-KH) P`), since this filter has no existing
 * regularization/subset-AR safety net to fall back on if P loses
 * positive-definiteness.
 *
 * `error_state` is expected to be all-zero on entry (the normal
 * LooseCouplingProcessor usage pattern: propagate -> single measurement
 * update -> inject into the nominal state -> reset to zero), so the
 * correction is computed as `error_state += K * residual` without an
 * `H * error_state` innovation-correction term; passing a nonzero
 * `error_state` in is supported arithmetically but any prior error is not
 * itself re-corrected against the new residual.
 *
 * @param error_state  15x1 error state, modified in place (delta added)
 * @param covariance   15x15 error covariance, modified in place
 * @param system       Measurement system (H, residual, R) from fusion_measurement
 * @param max_normalized_innovation_squared_per_observation
 *        NIS gate threshold; <= 0 or non-finite disables the gate
 * @return              Diagnostic result
 */
FusionUpdateResult applyDenseUpdate(Eigen::Matrix<double, 15, 1>& error_state,
                                    Eigen::Matrix<double, 15, 15>& covariance,
                                    const fusion_measurement::FusionMeasurementSystem& system,
                                    double max_normalized_innovation_squared_per_observation = 0.0);

}  // namespace fusion_update
}  // namespace libgnss
