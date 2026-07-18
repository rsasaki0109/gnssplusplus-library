#include <libgnss++/algorithms/rtk_tdcp_diagnostics.hpp>

#include <cmath>

namespace libgnss::rtk_tdcp_diagnostics {

Result evaluate(double previous_phase_m,
                double current_phase_m,
                double previous_range_rate_mps,
                double current_range_rate_mps,
                double dt_s,
                bool loss_of_lock,
                const Config& config) {
    Result result;
    if (!std::isfinite(previous_phase_m) || !std::isfinite(current_phase_m) ||
        !std::isfinite(previous_range_rate_mps) ||
        !std::isfinite(current_range_rate_mps) || !std::isfinite(dt_s) ||
        !std::isfinite(config.max_gap_s) || config.max_gap_s <= 0.0) {
        return result;
    }
    if (dt_s <= 0.0 || dt_s > config.max_gap_s) {
        result.status = Status::INVALID_GAP;
        return result;
    }
    if (loss_of_lock) {
        result.status = Status::LOSS_OF_LOCK;
        return result;
    }
    const double phase_change_m = current_phase_m - previous_phase_m;
    const double integrated_range_rate_m =
        0.5 * (previous_range_rate_mps + current_range_rate_mps) * dt_s;
    result.residual_m = phase_change_m - integrated_range_rate_m;
    if (!std::isfinite(result.residual_m)) return Result{};
    result.status = Status::VALID;
    return result;
}

}  // namespace libgnss::rtk_tdcp_diagnostics
